#include "contact_solver.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <type_traits>

namespace bench::phys {

namespace {

using simd::FloatW;
using simd::FloatW8;
using solver::BodyState;
using solver::ContactConstraint;
using solver::null_index;
using solver::Q4;
using solver::Sym3;
using solver::V3;

constexpr float pi = std::numbers::pi_v<float>;
constexpr float speculative_scale = 4.0f;
constexpr std::size_t parallel_threshold = 512;
constexpr uint32_t contacts_per_block = 16;
constexpr uint32_t bodies_per_block = 256;
constexpr float max_rotation_per_step = 0.25f * pi;

template <class F> V3<F> operator+(V3<F> a, V3<F> b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
template <class F> V3<F> operator-(V3<F> a, V3<F> b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
template <class F> V3<F> operator*(V3<F> a, F s) { return {a.x * s, a.y * s, a.z * s}; }
template <class F> F dot(V3<F> a, V3<F> b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

/// Cross product.
template <class F> V3<F> cross(V3<F> a, V3<F> b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

/// Symmetric matrix times vector.
template <class F> V3<F> multiply(const Sym3<F>& m, V3<F> v) {
    return {m.xx * v.x + m.xy * v.y + m.xz * v.z, m.xy * v.x + m.yy * v.y + m.yz * v.z,
            m.xz * v.x + m.yz * v.y + m.zz * v.z};
}

/// Rotate a vector by a unit quaternion.
template <class F> V3<F> rotate(const Q4<F>& q, V3<F> v) {
    const V3<F> axis{q.x, q.y, q.z};
    const V3<F> t = cross(axis, v) * F(2.0f);
    return v + t * q.w + cross(axis, t);
}

/// Upper triangle of a symmetric basis.
Sym3<float> sym_from_basis(const vm::Basis& m) { return {m.x.x, m.x.y, m.x.z, m.y.y, m.y.z, m.z.z}; }

/// Body velocities and pose deltas, lane-wise.
template <class F> struct BodyRefs {
    V3<F> velocity{};
    V3<F> angular_velocity{};
    V3<F> delta_position{};
    Q4<F> delta_rotation{};
};

template <class F> struct BodyPair {
    BodyRefs<F> a{};
    BodyRefs<F> b{};
};

template <class F> struct AnchorPair {
    V3<F> a{};
    V3<F> b{};
};

template <class F> struct FrictionBudget {
    F normal_impulse{};
    F twist_limit{};
};

template <class F> using LaneIndices = std::array<uint32_t, solver::width_of<F>>;

/// Loads the bodies named by each lane: four-float rows per body, transposed
/// four lanes at a time and joined.
template <class F> BodyRefs<F> gather(const BodyState* states, const LaneIndices<F>& index) {
    if constexpr (std::is_same_v<F, FloatW8>) {
        BodyRefs<F> out;
        std::array<std::array<std::array<FloatW, 4>, 4>, 2> halves{};
        for (std::size_t half = 0; half < 2; ++half) {
            auto& groups = halves[half];
            for (std::size_t lane = 0; lane < 4; ++lane) {
                const float* base = reinterpret_cast<const float*>(&states[index[half * 4 + lane]]);
                for (std::size_t group = 0; group < 4; ++group) groups[group][lane] = simd::load4(base + 4 * group);
            }
            for (auto& group : groups) simd::transpose4(group[0], group[1], group[2], group[3]);
        }
        const auto join = [&](std::size_t group, std::size_t row) { return F(halves[0][group][row], halves[1][group][row]); };
        out.velocity = {join(0, 0), join(0, 1), join(0, 2)};
        out.angular_velocity = {join(1, 0), join(1, 1), join(1, 2)};
        out.delta_position = {join(2, 0), join(2, 1), join(2, 2)};
        out.delta_rotation = {join(3, 0), join(3, 1), join(3, 2), join(3, 3)};
        return out;
    }
}

/// Stores velocities back, skipping read-only lanes.
template <class F>
void scatter(BodyState* states, uint32_t writable, const LaneIndices<F>& index, const BodyRefs<F>& body) {
    for (std::size_t lane = 0; lane < solver::width_of<F>; ++lane) {
        if (index[lane] >= writable) continue;
        float* base = reinterpret_cast<float*>(&states[index[lane]]);
        simd::store4(base, FloatW{simd::get_lane(body.velocity.x, lane), simd::get_lane(body.velocity.y, lane),
                                  simd::get_lane(body.velocity.z, lane), 0.0f});
        simd::store4(base + 4,
                     FloatW{simd::get_lane(body.angular_velocity.x, lane), simd::get_lane(body.angular_velocity.y, lane),
                            simd::get_lane(body.angular_velocity.z, lane), 0.0f});
    }
}

template <class F> BodyPair<F> gather_pair(const BodyState* states, const ContactConstraint<F>& c) {
    return {gather<F>(states, c.body_a), gather<F>(states, c.body_b)};
}

template <class F>
void scatter_pair(BodyState* states, uint32_t writable, const ContactConstraint<F>& c, const BodyPair<F>& bodies) {
    scatter<F>(states, writable, c.body_a, bodies.a);
    scatter<F>(states, writable, c.body_b, bodies.b);
}

template <class F> AnchorPair<F> point_anchors(const solver::PointConstraint<F>& p) { return {p.anchor_a, p.anchor_b}; }

/// Equal and opposite impulse at a contact anchor.
template <class F>
void apply_impulse(const ContactConstraint<F>& c, BodyPair<F>& bodies, const AnchorPair<F>& r, V3<F> impulse) {
    BodyRefs<F>& a = bodies.a;
    BodyRefs<F>& b = bodies.b;
    a.velocity = a.velocity - impulse * c.inverse_mass_a;
    a.angular_velocity = a.angular_velocity - multiply(c.inverse_inertia_a, cross(r.a, impulse));
    b.velocity = b.velocity + impulse * c.inverse_mass_b;
    b.angular_velocity = b.angular_velocity + multiply(c.inverse_inertia_b, cross(r.b, impulse));
}

/// Equal and opposite angular impulse.
template <class F> void apply_angular(const ContactConstraint<F>& c, BodyPair<F>& bodies, V3<F> impulse) {
    bodies.a.angular_velocity = bodies.a.angular_velocity - multiply(c.inverse_inertia_a, impulse);
    bodies.b.angular_velocity = bodies.b.angular_velocity + multiply(c.inverse_inertia_b, impulse);
}

/// Relative anchor velocity of B against A along a direction.
template <class F> F relative_velocity_along(const BodyPair<F>& bodies, const AnchorPair<F>& r, V3<F> direction) {
    const V3<F> va = bodies.a.velocity + cross(bodies.a.angular_velocity, r.a);
    const V3<F> vb = bodies.b.velocity + cross(bodies.b.angular_velocity, r.b);
    return dot(vb - va, direction);
}

template <class F> F clamp_between(F x, F low, F high) { return simd::min(simd::max(x, low), high); }

/// Applies last step's accumulated impulses.
template <class F> void warm_start_constraint(ContactConstraint<F>& c, BodyState* states, uint32_t writable) {
    BodyPair<F> bodies = gather_pair(states, c);
    for (uint32_t i = 0; i < c.point_count; ++i) {
        const auto& p = c.points[i];
        apply_impulse(c, bodies, point_anchors(p), c.normal * p.normal_impulse);
    }
    const V3<F> tangential = c.tangent1 * c.tangent_impulse_x + c.tangent2 * c.tangent_impulse_y;
    const AnchorPair<F> friction_anchors{c.friction_anchor_a, c.friction_anchor_b};
    apply_impulse(c, bodies, friction_anchors, tangential);
    apply_angular(c, bodies, c.normal * c.twist_impulse + c.rolling_impulse);
    scatter_pair(states, writable, c, bodies);
}

/// Tangential, twist and rolling friction.
template <class F> void solve_friction(ContactConstraint<F>& c, BodyPair<F>& bodies, const FrictionBudget<F>& budget) {
    const F zero(0.0f);
    const F one(1.0f);
    const F total_normal_impulse = budget.normal_impulse;
    const AnchorPair<F> anchors{c.friction_anchor_a, c.friction_anchor_b};
    const F max_friction = c.friction * total_normal_impulse;
    const F vt1 = relative_velocity_along(bodies, anchors, c.tangent1);
    const F vt2 = relative_velocity_along(bodies, anchors, c.tangent2);
    const F delta_x = -(c.tangent_mass_xx * vt1 + c.tangent_mass_xy * vt2);
    const F delta_y = -(c.tangent_mass_xy * vt1 + c.tangent_mass_yy * vt2);
    F total_x = c.tangent_impulse_x + delta_x;
    F total_y = c.tangent_impulse_y + delta_y;
    const F magnitude = simd::sqrt(total_x * total_x + total_y * total_y);
    const auto clamped = simd::both(simd::greater(magnitude, max_friction), simd::greater(magnitude, zero));
    const F scale = simd::select(clamped, max_friction / magnitude, one);
    total_x = total_x * scale;
    total_y = total_y * scale;
    const V3<F> impulse = c.tangent1 * (total_x - c.tangent_impulse_x) + c.tangent2 * (total_y - c.tangent_impulse_y);
    c.tangent_impulse_x = total_x;
    c.tangent_impulse_y = total_y;
    apply_impulse(c, bodies, anchors, impulse);

    const F max_twist = c.friction * budget.twist_limit;
    const F twist_velocity = dot(bodies.b.angular_velocity - bodies.a.angular_velocity, c.normal);
    F twist = c.twist_impulse - c.twist_mass * twist_velocity;
    twist = clamp_between(twist, -max_twist, max_twist);
    const F twist_delta = twist - c.twist_impulse;
    c.twist_impulse = twist;
    apply_angular(c, bodies, c.normal * twist_delta);

    const F max_rolling = c.rolling_resistance * total_normal_impulse;
    const V3<F> relative = bodies.b.angular_velocity - bodies.a.angular_velocity;
    V3<F> rolling = c.rolling_impulse - multiply(c.rolling_mass, relative);
    const F rolling_magnitude = simd::sqrt(dot(rolling, rolling));
    const auto rolling_clamped =
        simd::both(simd::greater(rolling_magnitude, max_rolling), simd::greater(rolling_magnitude, zero));
    const F rolling_scale = simd::select(rolling_clamped, max_rolling / rolling_magnitude, one);
    rolling = rolling * rolling_scale;
    const V3<F> rolling_delta = rolling - c.rolling_impulse;
    c.rolling_impulse = rolling;
    apply_angular(c, bodies, rolling_delta);
}

/// Normal constraint of every point; friction too on the relax pass.
template <class F, bool use_bias>
void solve_constraint(ContactConstraint<F>& c, BodyState* states, uint32_t writable, const SolverContext& context) {
    BodyPair<F> bodies = gather_pair(states, c);
    const BodyRefs<F>& a = bodies.a;
    const BodyRefs<F>& b = bodies.b;
    const F zero(0.0f);
    const F one(1.0f);
    const F inv_h(context.inv_h);
    const F push_out(-context.push_out_speed);
    const V3<F> dp = b.delta_position - a.delta_position;

    F total_normal_impulse(0.0f);
    F total_twist_limit(0.0f);
    for (uint32_t i = 0; i < c.point_count; ++i) {
        auto& p = c.points[i];
        const V3<F> ds = dp + rotate(b.delta_rotation, p.anchor_b) - rotate(a.delta_rotation, p.anchor_a);
        const F s = dot(ds, c.normal) + p.base_separation;
        const auto separated = simd::greater(s, zero);
        F bias = s * inv_h;
        F mass_scale = one;
        F impulse_scale = zero;
        if constexpr (use_bias) {
            const F soft_bias = simd::max(c.mass_scale * c.bias_rate * s, push_out);
            bias = simd::select(separated, bias, soft_bias);
            mass_scale = simd::select(separated, one, c.mass_scale);
            impulse_scale = simd::select(separated, zero, c.impulse_scale);
        } else {
            bias = simd::select(separated, bias, zero);
        }
        const F vn = relative_velocity_along(bodies, point_anchors(p), c.normal);
        F delta_impulse = -p.normal_mass * (mass_scale * vn + bias) - impulse_scale * p.normal_impulse;
        const F new_impulse = simd::max(p.normal_impulse + delta_impulse, zero);
        delta_impulse = new_impulse - p.normal_impulse;
        p.normal_impulse = new_impulse;
        p.total_normal_impulse = p.total_normal_impulse + new_impulse;
        p.peak_normal_impulse = simd::max(p.peak_normal_impulse, new_impulse);
        total_normal_impulse = total_normal_impulse + new_impulse;
        total_twist_limit = total_twist_limit + p.lever_arm * new_impulse;
        apply_impulse(c, bodies, point_anchors(p), c.normal * delta_impulse);
    }

    if constexpr (!use_bias) solve_friction(c, bodies, FrictionBudget<F>{total_normal_impulse, total_twist_limit});

    scatter_pair(states, writable, c, bodies);
}

/// Restitution for points that approached faster than the threshold.
template <class F>
void restitution_constraint(ContactConstraint<F>& c, BodyState* states, uint32_t writable, const SolverContext& context) {
    const F zero(0.0f);
    const F threshold(-context.restitution_threshold);
    const auto bouncy = simd::greater(c.restitution, zero);
    if (!simd::any(bouncy)) return;
    BodyPair<F> bodies = gather_pair(states, c);
    for (uint32_t i = 0; i < c.point_count; ++i) {
        auto& p = c.points[i];
        const auto active = simd::both(simd::both(bouncy, simd::less_equal(p.relative_velocity, threshold)),
                                       simd::not_equal(p.total_normal_impulse, zero));
        const F vn = relative_velocity_along(bodies, point_anchors(p), c.normal);
        F impulse = -p.normal_mass * (vn + c.restitution * p.relative_velocity);
        const F new_impulse = simd::max(p.normal_impulse + impulse, zero);
        impulse = simd::select(active, new_impulse - p.normal_impulse, zero);
        p.normal_impulse = simd::select(active, new_impulse, p.normal_impulse);
        p.total_normal_impulse = p.total_normal_impulse + simd::select(active, new_impulse, zero);
        p.peak_normal_impulse = simd::select(active, simd::max(p.peak_normal_impulse, new_impulse), p.peak_normal_impulse);
        apply_impulse(c, bodies, point_anchors(p), c.normal * impulse);
    }
    scatter_pair(states, writable, c, bodies);
}

/// Some unit vector perpendicular to the unit vector `n`.
vm::Vec3 perpendicular(vm::Vec3 n) {
    if (std::fabs(n.x) > 0.57735f) return vm::normalize(vm::Vec3{n.y, -n.x, 0.0f});
    return vm::normalize(vm::Vec3{0.0f, n.z, -n.y});
}

V3<float> pack(vm::Vec3 v) { return {v.x, v.y, v.z}; }

/// Scalar view of one body while preparing constraints.
struct PrepBody {
    vm::Vec3 velocity{};
    vm::Vec3 angular_velocity{};
    float inverse_mass{};
    vm::Basis inverse_inertia{vm::Basis{} * 0.0f};
    vm::Vec3 center{};
};

/// Inverse of the combined inverse mass along a direction at two anchors.
float effective_mass(const PrepBody& a, const PrepBody& b, vm::Vec3 ra, vm::Vec3 rb, vm::Vec3 direction) {
    const vm::Vec3 rna = vm::cross(ra, direction);
    const vm::Vec3 rnb = vm::cross(rb, direction);
    const float k = a.inverse_mass + b.inverse_mass + vm::dot(rna, a.inverse_inertia * rna) +
                    vm::dot(rnb, b.inverse_inertia * rnb);
    return k > 0.0f ? 1.0f / k : 0.0f;
}

/// Relative anchor velocity of B against A along a direction.
float prep_relative_velocity(const PrepBody& a, const PrepBody& b, vm::Vec3 ra, vm::Vec3 rb, vm::Vec3 direction) {
    const vm::Vec3 va = a.velocity + vm::cross(a.angular_velocity, ra);
    const vm::Vec3 vb = b.velocity + vm::cross(b.angular_velocity, rb);
    return vm::dot(vb - va, direction);
}

/// Friction anchors, lever arms and tangent, twist and rolling masses.
void prepare_friction(ContactConstraint<float>& c, const Manifold& manifold, const PrepBody& a, const PrepBody& b,
                      vm::Vec3 tangent1, vm::Vec3 tangent2, float rolling_resistance, float speculative) {
    vm::Vec3 center_a{};
    vm::Vec3 center_b{};
    float total_weight = 0.0f;
    const float inv_tau = 1.0f / speculative;
    for (uint32_t i = 0; i < manifold.point_count; ++i) {
        const float weight = std::clamp(2.0f - manifold.points[i].separation * inv_tau, min_friction_weight, 1.0f);
        const auto& pc = c.points[i];
        center_a = center_a + vm::Vec3{pc.anchor_a.x, pc.anchor_a.y, pc.anchor_a.z} * weight;
        center_b = center_b + vm::Vec3{pc.anchor_b.x, pc.anchor_b.y, pc.anchor_b.z} * weight;
        total_weight += weight;
    }
    const float inv_weight = total_weight > 0.0f ? 1.0f / total_weight : 0.0f;
    const vm::Vec3 anchor_a = center_a * inv_weight;
    const vm::Vec3 anchor_b = center_b * inv_weight;
    c.friction_anchor_a = pack(anchor_a);
    c.friction_anchor_b = pack(anchor_b);

    for (uint32_t i = 0; i < manifold.point_count; ++i) {
        auto& pc = c.points[i];
        pc.lever_arm = vm::length(vm::Vec3{pc.anchor_a.x, pc.anchor_a.y, pc.anchor_a.z} - anchor_a);
    }

    const vm::Vec3 rta1 = vm::cross(anchor_a, tangent1);
    const vm::Vec3 rta2 = vm::cross(anchor_a, tangent2);
    const vm::Vec3 rtb1 = vm::cross(anchor_b, tangent1);
    const vm::Vec3 rtb2 = vm::cross(anchor_b, tangent2);
    const float inv_mass = a.inverse_mass + b.inverse_mass;
    const float kxx = inv_mass + vm::dot(rta1, a.inverse_inertia * rta1) + vm::dot(rtb1, b.inverse_inertia * rtb1);
    const float kyy = inv_mass + vm::dot(rta2, a.inverse_inertia * rta2) + vm::dot(rtb2, b.inverse_inertia * rtb2);
    const float kxy = vm::dot(rta1, a.inverse_inertia * rta2) + vm::dot(rtb1, b.inverse_inertia * rtb2);
    const float tangent_det = kxx * kyy - kxy * kxy;
    if (tangent_det != 0.0f) {
        const float inv = 1.0f / tangent_det;
        c.tangent_mass_xx = kyy * inv;
        c.tangent_mass_xy = -kxy * inv;
        c.tangent_mass_yy = kxx * inv;
    }

    const vm::Vec3 normal = manifold.normal;
    const vm::Basis angular = a.inverse_inertia + b.inverse_inertia;
    const float twist = vm::dot(normal, angular * normal);
    c.twist_mass = twist > 0.0f ? 1.0f / twist : 0.0f;
    if (rolling_resistance > 0.0f && vm::determinant(angular) > 0.0f) {
        c.rolling_mass = sym_from_basis(vm::inverse(angular));
    }
}

template <class Fn> void zip_vec(V3<FloatW8>& a, V3<float>& b, Fn& fn) {
    fn(a.x, b.x);
    fn(a.y, b.y);
    fn(a.z, b.z);
}

template <class Fn> void zip_sym(Sym3<FloatW8>& a, Sym3<float>& b, Fn& fn) {
    fn(a.xx, b.xx);
    fn(a.xy, b.xy);
    fn(a.xz, b.xz);
    fn(a.yy, b.yy);
    fn(a.yz, b.yz);
    fn(a.zz, b.zz);
}

/// Applies `fn` to every value field of a bundle and a scalar constraint.
template <class Fn> void zip_constraint_values(ContactConstraint<FloatW8>& a, ContactConstraint<float>& b, Fn&& fn) {
    fn(a.inverse_mass_a, b.inverse_mass_a);
    fn(a.inverse_mass_b, b.inverse_mass_b);
    zip_sym(a.inverse_inertia_a, b.inverse_inertia_a, fn);
    zip_sym(a.inverse_inertia_b, b.inverse_inertia_b, fn);
    zip_vec(a.normal, b.normal, fn);
    zip_vec(a.tangent1, b.tangent1, fn);
    zip_vec(a.tangent2, b.tangent2, fn);
    fn(a.bias_rate, b.bias_rate);
    fn(a.mass_scale, b.mass_scale);
    fn(a.impulse_scale, b.impulse_scale);
    fn(a.friction, b.friction);
    fn(a.restitution, b.restitution);
    fn(a.rolling_resistance, b.rolling_resistance);
    for (std::size_t i = 0; i < solver::max_points; ++i) {
        auto& pa = a.points[i];
        auto& pb = b.points[i];
        zip_vec(pa.anchor_a, pb.anchor_a, fn);
        zip_vec(pa.anchor_b, pb.anchor_b, fn);
        fn(pa.base_separation, pb.base_separation);
        fn(pa.normal_mass, pb.normal_mass);
        fn(pa.relative_velocity, pb.relative_velocity);
        fn(pa.normal_impulse, pb.normal_impulse);
        fn(pa.total_normal_impulse, pb.total_normal_impulse);
        fn(pa.peak_normal_impulse, pb.peak_normal_impulse);
        fn(pa.lever_arm, pb.lever_arm);
    }
    zip_vec(a.friction_anchor_a, b.friction_anchor_a, fn);
    zip_vec(a.friction_anchor_b, b.friction_anchor_b, fn);
    fn(a.tangent_mass_xx, b.tangent_mass_xx);
    fn(a.tangent_mass_xy, b.tangent_mass_xy);
    fn(a.tangent_mass_yy, b.tangent_mass_yy);
    fn(a.tangent_impulse_x, b.tangent_impulse_x);
    fn(a.tangent_impulse_y, b.tangent_impulse_y);
    fn(a.twist_mass, b.twist_mass);
    fn(a.twist_impulse, b.twist_impulse);
    zip_sym(a.rolling_mass, b.rolling_mass, fn);
    zip_vec(a.rolling_impulse, b.rolling_impulse, fn);
}

} // namespace

void ContactSolver::build_bodies(const SolverInputs& inputs) {
    const SolverContext& context = inputs.context;
    const auto count = static_cast<uint32_t>(inputs.active_bodies.size());
    writable_count_ = count;
    dummy_index_ = count;
    states_.assign(static_cast<std::size_t>(count) + 1u, BodyState{});
    props_.assign(static_cast<std::size_t>(count) + 1u, BodyProps{});
    static_lookup_.assign(inputs.static_motions.size(), null_index);
    for (uint32_t i = 0; i < count; ++i) {
        const RigidBody& body = *inputs.active_bodies[i].body;
        BodyState& state = states_[i];
        BodyProps& props = props_[i];
        state.velocity = body.linear_velocity;
        state.angular_velocity = body.angular_velocity;
        props.center = body.world_center_of_mass();
        props.inverse_mass = body.inverse_mass;
        props.inverse_inertia = body.inverse_inertia_world;
        props.rotation = body.rotation;
        props.force = body.applied_force + body.constant_force + context.gravity * (body.gravity_scale * body.mass);
        props.torque = body.applied_torque + body.constant_torque;
        props.linear_damping = 1.0f / (1.0f + context.h * body.linear_damp);
        props.angular_damping = 1.0f / (1.0f + context.h * body.angular_damp);
    }
}

uint32_t ContactSolver::read_only_static(const SolverInputs& inputs, uint32_t fixed_index) {
    if (fixed_index >= inputs.static_motions.size()) return dummy_index_;
    uint32_t& entry = static_lookup_[fixed_index];
    if (entry == null_index) {
        const KinematicMotion& motion = inputs.static_motions[fixed_index];
        BodyState state{};
        state.velocity = motion.velocity;
        state.angular_velocity = motion.angular_velocity;
        BodyProps props{};
        props.center = motion.center;
        entry = static_cast<uint32_t>(states_.size());
        states_.push_back(state);
        props_.push_back(props);
    }
    return entry;
}

void ContactSolver::collect_entries(const SolverInputs& inputs) {
    entries_.clear();
    colors_.clear();
    for (uint32_t color = 0; color < graph_color_count; ++color) {
        for (const uint32_t index : inputs.colors[color]) {
            if (color == overflow_color) {
                std::fprintf(stderr, "solver: the overflow colour is not supported\n");
                std::exit(4);
            }
            const Contact& contact = inputs.contacts[index];
            const uint32_t body_a = inputs.body_local[contact.shape_a.body];
            const bool b_fixed = contact.shape_b.is_static;
            const uint32_t body_b = b_fixed ? read_only_static(inputs, contact.shape_b.body) : inputs.body_local[contact.shape_b.body];
            entries_.push_back({index, body_a, body_b, b_fixed});
            colors_.push_back(static_cast<uint8_t>(color));
        }
    }
}

void ContactSolver::prepare_entry(const SolverInputs& inputs, const Entry& entry, Scalar& out) const {
    const SolverContext& context = inputs.context;
    Contact& contact = inputs.contacts[entry.contact];
    const auto prep = [&](uint32_t body) {
        PrepBody view{};
        const BodyProps& props = props_[body];
        view.velocity = states_[body].velocity;
        view.angular_velocity = states_[body].angular_velocity;
        view.inverse_mass = props.inverse_mass;
        view.inverse_inertia = props.inverse_inertia;
        view.center = props.center;
        return view;
    };
    const PrepBody a = prep(entry.body_a);
    const PrepBody b = prep(entry.body_b);

    Scalar c{};
    c.body_a = {entry.body_a};
    c.body_b = {entry.body_b};
    c.contact = {entry.contact};
    Manifold& manifold = contact.manifold;
    c.point_count = manifold.point_count;
    const vm::Vec3 n = manifold.normal;
    const vm::Vec3 tangent1 = perpendicular(n);
    const vm::Vec3 tangent2 = vm::cross(tangent1, n);
    c.normal = pack(n);
    c.tangent1 = pack(tangent1);
    c.tangent2 = pack(tangent2);
    const Softness& softness = entry.b_fixed ? context.static_softness : context.contact_softness;
    c.bias_rate = softness.bias_rate;
    c.mass_scale = softness.mass_scale;
    c.impulse_scale = softness.impulse_scale;
    c.friction = contact.friction;
    c.restitution = contact.restitution;
    c.rolling_resistance = contact.rolling_resistance;
    c.inverse_mass_a = a.inverse_mass;
    c.inverse_mass_b = b.inverse_mass;
    c.inverse_inertia_a = sym_from_basis(a.inverse_inertia);
    c.inverse_inertia_b = sym_from_basis(b.inverse_inertia);

    for (uint32_t i = 0; i < manifold.point_count; ++i) {
        ManifoldPoint& point = manifold.points[i];
        auto& pc = c.points[i];
        const vm::Vec3 anchor_a = point.point - a.center;
        const vm::Vec3 anchor_b = point.point - b.center;
        pc.anchor_a = pack(anchor_a);
        pc.anchor_b = pack(anchor_b);
        pc.base_separation = point.separation - vm::dot(anchor_b - anchor_a, n);
        pc.normal_mass = effective_mass(a, b, anchor_a, anchor_b, n);
        pc.relative_velocity = prep_relative_velocity(a, b, anchor_a, anchor_b, n);
        pc.normal_impulse = point.normal_impulse;
        point.relative_velocity = pc.relative_velocity;
    }
    prepare_friction(c, manifold, a, b, tangent1, tangent2, contact.rolling_resistance, speculative_scale * context.linear_slop);
    const FrictionImpulses& impulses = contact.friction_impulses;
    c.tangent_impulse_x = impulses.tangent_x;
    c.tangent_impulse_y = impulses.tangent_y;
    c.twist_impulse = impulses.twist;
    c.rolling_impulse = pack(impulses.rolling);
    out = c;
}

void ContactSolver::prepare_constraints(const SolverInputs& inputs) {
    collect_entries(inputs);
    scalars_.resize(entries_.size());
    const auto prepare_range = [&](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) prepare_entry(inputs, entries_[i], scalars_[i]);
    };
    if (inputs.pool != nullptr && entries_.size() >= parallel_threshold) {
        inputs.pool->parallel_for(entries_.size(), 128, prepare_range);
    } else {
        prepare_range(0, entries_.size());
    }
    bundle_colors(inputs);
}

void ContactSolver::bundle_colors(const SolverInputs& inputs) {
    std::array<uint32_t, graph_color_count + 1> counts{};
    for (const uint8_t color : colors_) ++counts[color + 1u];
    for (uint32_t color = 0; color < graph_color_count; ++color) counts[color + 1u] += counts[color];
    color_start_ = counts;
    order_.resize(entries_.size());
    for (uint32_t i = 0; i < entries_.size(); ++i) order_[counts[colors_[i]]++] = i;

    constexpr uint32_t width = solver::lanes;
    uint32_t bundles = 0;
    for (uint32_t color = 0; color < overflow_color; ++color) {
        const uint32_t members = color_start_[color + 1u] - color_start_[color];
        color_begin_[color] = bundles;
        bundles += (members + width - 1u) / width;
        color_end_[color] = bundles;
    }

    bundles_.resize(bundles);
    const auto pack_range = [&](std::size_t begin, std::size_t end) {
        for (std::size_t bundle_index = begin; bundle_index < end; ++bundle_index) {
            uint32_t color = 0;
            while (color_end_[color] <= bundle_index) ++color;
            const uint32_t first = color_start_[color] + (static_cast<uint32_t>(bundle_index) - color_begin_[color]) * width;
            const uint32_t last = std::min(first + width, color_start_[color + 1u]);
            Bundle bundle{};
            bundle.body_a.fill(dummy_index_);
            bundle.body_b.fill(dummy_index_);
            bundle.contact.fill(null_index);
            for (uint32_t member = first; member < last; ++member) {
                const uint32_t lane = member - first;
                Scalar source = scalars_[order_[member]];
                bundle.body_a[lane] = source.body_a[0];
                bundle.body_b[lane] = source.body_b[0];
                bundle.contact[lane] = source.contact[0];
                bundle.point_count = std::max(bundle.point_count, source.point_count);
                zip_constraint_values(bundle, source, [lane](FloatW8& target, float value) { simd::set_lane(target, lane, value); });
            }
            bundles_[bundle_index] = bundle;
        }
    };
    if (inputs.pool != nullptr && bundles >= parallel_threshold / width) {
        inputs.pool->parallel_for(bundles, 32, pack_range);
    } else {
        pack_range(0, bundles);
    }
}

void ContactSolver::add_stage(StageKind kind, uint32_t color, uint32_t begin, uint32_t end, uint32_t grain) {
    if (begin == end) return;
    Stage stage{};
    stage.kind = kind;
    stage.color = color;
    stage.begin = begin;
    stage.end = end;
    stage.grain = grain;
    stage.blocks = (end - begin + grain - 1u) / grain;
    stages_.push_back(stage);
}

void ContactSolver::add_constraint_stages(StageKind kind) {
    for (uint32_t color = 0; color < overflow_color; ++color) {
        add_stage(kind, color, color_begin_[color], color_end_[color], std::max(1u, contacts_per_block / solver::lanes));
    }
}

void ContactSolver::plan_stages(const SolverContext& context) {
    stages_.clear();
    for (uint32_t substep = 0; substep < context.substeps; ++substep) {
        add_stage(StageKind::IntegrateVelocities, 0, 0, writable_count_, bodies_per_block);
        add_constraint_stages(StageKind::WarmStart);
        add_constraint_stages(StageKind::SolveBiased);
        add_stage(StageKind::IntegratePositions, 0, 0, writable_count_, bodies_per_block);
        add_constraint_stages(StageKind::SolveRelax);
    }
    add_constraint_stages(StageKind::Restitution);
    if (progress_capacity_ < stages_.size()) {
        progress_capacity_ = stages_.size() * 2;
        progress_ = std::make_unique<StageProgress[]>(progress_capacity_);
    }
    for (std::size_t i = 0; i < stages_.size(); ++i) {
        progress_[i].next.store(0, std::memory_order_relaxed);
        progress_[i].done.store(0, std::memory_order_relaxed);
    }
}

void ContactSolver::run_block(const Stage& stage, uint32_t block, const SolverContext& context) {
    BodyState* states = states_.data();
    const uint32_t begin = stage.begin + block * stage.grain;
    const uint32_t end = std::min(stage.end, begin + stage.grain);
    switch (stage.kind) {
    case StageKind::IntegrateVelocities:
        for (uint32_t i = begin; i < end; ++i) {
            BodyState& state = states_[i];
            const BodyProps& props = props_[i];
            const float h = context.h;
            state.velocity = (state.velocity + props.force * (props.inverse_mass * h)) * props.linear_damping;
            state.angular_velocity = (state.angular_velocity + props.inverse_inertia * props.torque * h) * props.angular_damping;
        }
        return;
    case StageKind::IntegratePositions: {
        const float h = context.h;
        const float max_angular_speed = max_rotation_per_step * context.inv_dt;
        for (uint32_t i = begin; i < end; ++i) {
            BodyState& state = states_[i];
            BodyProps& props = props_[i];
            const float angular_speed = vm::length(state.angular_velocity);
            if (angular_speed > max_angular_speed) {
                state.angular_velocity = state.angular_velocity * (max_angular_speed / angular_speed);
            }
            const vm::Vec3 step = state.velocity * h;
            const vm::Vec3 turn = state.angular_velocity * h;
            props.center = props.center + step;
            props.rotation = vm::integrate_rotation(props.rotation, turn);
            state.delta_position = state.delta_position + step;
            state.delta_rotation = vm::integrate_rotation(state.delta_rotation, turn);
        }
        return;
    }
    default:
        break;
    }
    for (uint32_t i = begin; i < end; ++i) {
        Bundle& c = bundles_[i];
        switch (stage.kind) {
        case StageKind::WarmStart:
            warm_start_constraint<FloatW8>(c, states, writable_count_);
            break;
        case StageKind::SolveBiased:
            solve_constraint<FloatW8, true>(c, states, writable_count_, context);
            break;
        case StageKind::SolveRelax:
            solve_constraint<FloatW8, false>(c, states, writable_count_, context);
            break;
        case StageKind::Restitution:
            restitution_constraint<FloatW8>(c, states, writable_count_, context);
            break;
        default:
            break;
        }
    }
}

void ContactSolver::run_stages(const SolverContext& context, TaskPool* pool) {
    const auto stage_count = static_cast<uint32_t>(stages_.size());
    const bool parallel = pool != nullptr && pool->thread_count() > 1 && entries_.size() >= parallel_threshold;
    if (!parallel) {
        for (const Stage& stage : stages_) {
            for (uint32_t block = 0; block < stage.blocks; ++block) run_block(stage, block, context);
        }
        return;
    }

    std::atomic<uint32_t> current{0};
    const auto work = [&](uint32_t index) {
        const Stage& stage = stages_[index];
        StageProgress& progress = progress_[index];
        for (;;) {
            const uint32_t block = progress.next.fetch_add(1, std::memory_order_relaxed);
            if (block >= stage.blocks) return;
            run_block(stage, block, context);
            progress.done.fetch_add(1, std::memory_order_release);
        }
    };
    pool->run([&](uint32_t worker) {
        if (worker == 0) {
            for (uint32_t index = 0; index < stage_count; ++index) {
                if (index != 0) current.store(index, std::memory_order_release);
                work(index);
                while (progress_[index].done.load(std::memory_order_acquire) != stages_[index].blocks) TaskPool::relax();
            }
            current.store(stage_count, std::memory_order_release);
            return;
        }
        uint32_t seen = null_index;
        for (;;) {
            const uint32_t index = current.load(std::memory_order_acquire);
            if (index >= stage_count) return;
            if (index == seen) {
                TaskPool::relax();
                continue;
            }
            seen = index;
            work(index);
        }
    });
}

void ContactSolver::store_results(const SolverInputs& inputs) {
    const auto spread = [&](std::size_t count, std::size_t grain, auto&& body) {
        if (inputs.pool != nullptr && entries_.size() >= parallel_threshold) {
            inputs.pool->parallel_for(count, grain, body);
        } else {
            body(std::size_t{0}, count);
        }
    };
    spread(bundles_.size(), 64, [&](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            const Bundle& c = bundles_[i];
            for (std::size_t lane = 0; lane < solver::lanes; ++lane) {
                if (c.contact[lane] == null_index) continue;
                Contact& contact = inputs.contacts[c.contact[lane]];
                Manifold& manifold = contact.manifold;
                for (uint32_t k = 0; k < manifold.point_count; ++k) {
                    ManifoldPoint& point = manifold.points[k];
                    point.normal_impulse = simd::get_lane(c.points[k].normal_impulse, lane);
                    point.total_normal_impulse = simd::get_lane(c.points[k].total_normal_impulse, lane);
                    point.peak_normal_impulse = simd::get_lane(c.points[k].peak_normal_impulse, lane);
                }
                FrictionImpulses& friction = contact.friction_impulses;
                friction.tangent_x = simd::get_lane(c.tangent_impulse_x, lane);
                friction.tangent_y = simd::get_lane(c.tangent_impulse_y, lane);
                friction.twist = simd::get_lane(c.twist_impulse, lane);
                friction.rolling = {simd::get_lane(c.rolling_impulse.x, lane), simd::get_lane(c.rolling_impulse.y, lane),
                                    simd::get_lane(c.rolling_impulse.z, lane)};
            }
        }
    });
    spread(writable_count_, 128, [&](std::size_t begin, std::size_t end) {
        for (std::size_t i = begin; i < end; ++i) {
            const BodyState& state = states_[i];
            const BodyProps& props = props_[i];
            RigidBody& body = *inputs.active_bodies[i].body;
            body.linear_velocity = state.velocity;
            body.angular_velocity = state.angular_velocity;
            set_pose(body, props.center, props.rotation);
            inputs.deltas[i] = BodyDelta{state.delta_position, state.delta_rotation};
        }
    });
}

void ContactSolver::solve(const SolverInputs& inputs) {
    build_bodies(inputs);
    prepare_constraints(inputs);
    plan_stages(inputs.context);
    run_stages(inputs.context, inputs.pool);
    store_results(inputs);
}

} // namespace bench::phys
