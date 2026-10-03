#include "contact_solver.h"

#include <math.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "array.h"
#include "simd.h"

enum {
    MAX_POINTS = 4,
    LANES = 8,
    PARALLEL_THRESHOLD = 512,
    CONTACTS_PER_BLOCK = 16,
    BODIES_PER_BLOCK = 256,
};

static const uint32_t null_index = 0xFFFFFFFFu;
static const float pi = 3.14159265358979323846f;
static const float speculative_scale = 4.0f;

/// Per-body velocities and accumulated pose delta, padded to 64 bytes for
/// four-float lane loads.
typedef struct {
    _Alignas(16) Vec3 velocity;
    float pad_velocity;
    Vec3 angular_velocity;
    float pad_angular;
    Vec3 delta_position;
    float pad_position;
    Quat delta_rotation;
} BodyState;

_Static_assert(sizeof(BodyState) == 64, "BodyState is four rows of four floats");

typedef struct {
    Vec3 center;
    float inverse_mass;
    Basis inverse_inertia;
    Quat rotation;
    Vec3 force;
    Vec3 torque;
    float linear_damping;
    float angular_damping;
} BodyProps;

typedef struct {
    uint32_t contact;
    uint32_t body_a;
    uint32_t body_b;
    bool b_fixed;
} Entry;

/// Symmetric 3x3 matrix stored as its six unique components.
typedef struct { float xx, xy, xz, yy, yz, zz; } Sym3;

/// Solver data of one contact point.
typedef struct {
    Vec3 anchor_a, anchor_b;
    float base_separation, normal_mass, relative_velocity, normal_impulse, total_normal_impulse, peak_normal_impulse,
        lever_arm;
} ScalarPoint;

/// Solver data of one contact manifold.
typedef struct {
    uint32_t body_a, body_b, contact, point_count;
    float inverse_mass_a, inverse_mass_b;
    Sym3 inverse_inertia_a, inverse_inertia_b;
    Vec3 normal, tangent1, tangent2;
    float bias_rate, mass_scale, impulse_scale, friction, restitution, rolling_resistance;
    ScalarPoint points[MAX_POINTS];
    Vec3 friction_anchor_a, friction_anchor_b;
    float tangent_mass_xx, tangent_mass_xy, tangent_mass_yy, tangent_impulse_x, tangent_impulse_y, twist_mass,
        twist_impulse;
    Sym3 rolling_mass;
    Vec3 rolling_impulse;
} Scalar;

typedef struct { W x, y, z; } V3W;
typedef struct { W x, y, z, w; } Q4W;
typedef struct { W xx, xy, xz, yy, yz, zz; } Sym3W;

/// Solver data of one contact point, one lane per contact.
typedef struct {
    V3W anchor_a, anchor_b;
    W base_separation, normal_mass, relative_velocity, normal_impulse, total_normal_impulse, peak_normal_impulse, lever_arm;
} PointW;

/// Eight contact manifolds of one colour, one per lane.
typedef struct {
    uint32_t body_a[LANES], body_b[LANES], contact[LANES], point_count;
    W inverse_mass_a, inverse_mass_b;
    Sym3W inverse_inertia_a, inverse_inertia_b;
    V3W normal, tangent1, tangent2;
    W bias_rate, mass_scale, impulse_scale, friction, restitution, rolling_resistance;
    PointW points[MAX_POINTS];
    V3W friction_anchor_a, friction_anchor_b;
    W tangent_mass_xx, tangent_mass_xy, tangent_mass_yy, tangent_impulse_x, tangent_impulse_y, twist_mass, twist_impulse;
    Sym3W rolling_mass;
    V3W rolling_impulse;
} Bundle;

typedef enum { INTEGRATE_VELOCITIES, WARM_START, SOLVE_BIASED, INTEGRATE_POSITIONS, SOLVE_RELAX, RESTITUTION } StageKind;

/// A run of blocks of one kind, executed between barriers.
typedef struct {
    StageKind kind;
    uint32_t color;
    uint32_t begin;
    uint32_t end;
    uint32_t grain;
    uint32_t blocks;
} Stage;

typedef struct {
    atomic_uint next;
    atomic_uint done;
} StageProgress;

struct ContactSolver {
    ARRAY_OF(BodyState) states;
    ARRAY_OF(BodyProps) props;
    U32Array static_lookup;
    uint32_t writable_count;
    uint32_t dummy_index;
    ARRAY_OF(Entry) entries;
    U8Array colors;
    ARRAY_OF(Scalar) scalars;
    U32Array order;
    ARRAY_OF(Bundle) bundles;
    uint32_t color_start[GRAPH_COLOR_COUNT + 1];
    uint32_t color_begin[GRAPH_COLOR_COUNT];
    uint32_t color_end[GRAPH_COLOR_COUNT];
    ARRAY_OF(Stage) stages;
    StageProgress* progress;
    size_t progress_capacity;
};

/// The solver and the inputs of the solve a parallel range works on.
typedef struct {
    ContactSolver* solver;
    const SolverInputs* inputs;
} SolveJob;

/// Shared state of the stage barrier loop.
typedef struct {
    ContactSolver* solver;
    const SolverContext* context;
    atomic_uint current;
    uint32_t stage_count;
} StageRun;

/// Lane-wise vector sum.
static inline V3W v3w_add(V3W a, V3W b) { return (V3W){ w_add(a.x, b.x), w_add(a.y, b.y), w_add(a.z, b.z) }; }
/// Lane-wise vector difference.
static inline V3W v3w_sub(V3W a, V3W b) { return (V3W){ w_sub(a.x, b.x), w_sub(a.y, b.y), w_sub(a.z, b.z) }; }
/// Lane-wise vector scaled by a lane-wise scalar.
static inline V3W v3w_scale(V3W a, W s) { return (V3W){ w_mul(a.x, s), w_mul(a.y, s), w_mul(a.z, s) }; }
/// Lane-wise dot product.
static inline W v3w_dot(V3W a, V3W b) { return w_add(w_add(w_mul(a.x, b.x), w_mul(a.y, b.y)), w_mul(a.z, b.z)); }

/// Cross product.
static inline V3W v3w_cross(V3W a, V3W b) {
    return (V3W){ w_sub(w_mul(a.y, b.z), w_mul(a.z, b.y)), w_sub(w_mul(a.z, b.x), w_mul(a.x, b.z)),
                  w_sub(w_mul(a.x, b.y), w_mul(a.y, b.x)) };
}

/// Symmetric matrix times vector.
static inline V3W sym_multiply(const Sym3W* m, V3W v) {
    return (V3W){ w_add(w_add(w_mul(m->xx, v.x), w_mul(m->xy, v.y)), w_mul(m->xz, v.z)),
                  w_add(w_add(w_mul(m->xy, v.x), w_mul(m->yy, v.y)), w_mul(m->yz, v.z)),
                  w_add(w_add(w_mul(m->xz, v.x), w_mul(m->yz, v.y)), w_mul(m->zz, v.z)) };
}

/// Rotate a vector by a unit quaternion.
static inline V3W q4w_rotate(const Q4W* q, V3W v) {
    V3W axis = { q->x, q->y, q->z };
    V3W t = v3w_scale(v3w_cross(axis, v), w_splat(2.0f));
    return v3w_add(v3w_add(v, v3w_scale(t, q->w)), v3w_cross(axis, t));
}

/// Body velocities and pose deltas, lane-wise.
typedef struct { V3W velocity, angular_velocity, delta_position; Q4W delta_rotation; } BodyRefs;
typedef struct { BodyRefs a, b; } BodyPair;
typedef struct { V3W a, b; } AnchorPair;

/// Loads the bodies named by each lane: four-float rows per body, transposed
/// four lanes at a time and joined.
static BodyRefs gather(const BodyState* states, const uint32_t* index) {
    __m128 rows[2][4][4];
    for (int half = 0; half < 2; half++) {
        for (int lane = 0; lane < 4; lane++) {
            const float* base = (const float*)&states[index[half * 4 + lane]];
            for (int group = 0; group < 4; group++) rows[half][group][lane] = _mm_loadu_ps(base + 4 * group);
        }
        for (int group = 0; group < 4; group++) {
            _MM_TRANSPOSE4_PS(rows[half][group][0], rows[half][group][1], rows[half][group][2], rows[half][group][3]);
        }
    }
#define JOIN(group, row) _mm256_set_m128(rows[1][group][row], rows[0][group][row])
    BodyRefs out = {
        { JOIN(0, 0), JOIN(0, 1), JOIN(0, 2) },
        { JOIN(1, 0), JOIN(1, 1), JOIN(1, 2) },
        { JOIN(2, 0), JOIN(2, 1), JOIN(2, 2) },
        { JOIN(3, 0), JOIN(3, 1), JOIN(3, 2), JOIN(3, 3) },
    };
#undef JOIN
    return out;
}

/// Stores velocities back, skipping read-only lanes.
static void scatter(BodyState* states, uint32_t writable, const uint32_t* index, const BodyRefs* body) {
    float vx[LANES], vy[LANES], vz[LANES], wx[LANES], wy[LANES], wz[LANES];
    _mm256_storeu_ps(vx, body->velocity.x);
    _mm256_storeu_ps(vy, body->velocity.y);
    _mm256_storeu_ps(vz, body->velocity.z);
    _mm256_storeu_ps(wx, body->angular_velocity.x);
    _mm256_storeu_ps(wy, body->angular_velocity.y);
    _mm256_storeu_ps(wz, body->angular_velocity.z);
    for (int lane = 0; lane < LANES; lane++) {
        if (index[lane] >= writable) continue;
        float* base = (float*)&states[index[lane]];
        _mm_storeu_ps(base, _mm_setr_ps(vx[lane], vy[lane], vz[lane], 0.0f));
        _mm_storeu_ps(base + 4, _mm_setr_ps(wx[lane], wy[lane], wz[lane], 0.0f));
    }
}

/// Loads both bodies of every lane.
static BodyPair gather_pair(const BodyState* states, const Bundle* c) {
    return (BodyPair){ gather(states, c->body_a), gather(states, c->body_b) };
}

/// Stores both bodies of every lane back, skipping read-only lanes.
static void scatter_pair(BodyState* states, uint32_t writable, const Bundle* c, const BodyPair* bodies) {
    scatter(states, writable, c->body_a, &bodies->a);
    scatter(states, writable, c->body_b, &bodies->b);
}

/// Anchors of one contact point on both bodies.
static AnchorPair point_anchors(const PointW* p) { return (AnchorPair){ p->anchor_a, p->anchor_b }; }

/// Equal and opposite impulse at a contact anchor.
static void apply_impulse(const Bundle* c, BodyPair* bodies, const AnchorPair* r, V3W impulse) {
    BodyRefs* a = &bodies->a;
    BodyRefs* b = &bodies->b;
    a->velocity = v3w_sub(a->velocity, v3w_scale(impulse, c->inverse_mass_a));
    a->angular_velocity = v3w_sub(a->angular_velocity, sym_multiply(&c->inverse_inertia_a, v3w_cross(r->a, impulse)));
    b->velocity = v3w_add(b->velocity, v3w_scale(impulse, c->inverse_mass_b));
    b->angular_velocity = v3w_add(b->angular_velocity, sym_multiply(&c->inverse_inertia_b, v3w_cross(r->b, impulse)));
}

/// Equal and opposite angular impulse.
static void apply_angular(const Bundle* c, BodyPair* bodies, V3W impulse) {
    bodies->a.angular_velocity = v3w_sub(bodies->a.angular_velocity, sym_multiply(&c->inverse_inertia_a, impulse));
    bodies->b.angular_velocity = v3w_add(bodies->b.angular_velocity, sym_multiply(&c->inverse_inertia_b, impulse));
}

/// Relative anchor velocity of B against A along a direction.
static W relative_velocity_along(const BodyPair* bodies, const AnchorPair* r, V3W direction) {
    V3W va = v3w_add(bodies->a.velocity, v3w_cross(bodies->a.angular_velocity, r->a));
    V3W vb = v3w_add(bodies->b.velocity, v3w_cross(bodies->b.angular_velocity, r->b));
    return v3w_dot(v3w_sub(vb, va), direction);
}

/// Applies last step's accumulated impulses.
static void warm_start_constraint(Bundle* c, BodyState* states, uint32_t writable) {
    BodyPair bodies = gather_pair(states, c);
    for (uint32_t i = 0; i < c->point_count; i++) {
        const PointW* p = &c->points[i];
        AnchorPair anchors = point_anchors(p);
        apply_impulse(c, &bodies, &anchors, v3w_scale(c->normal, p->normal_impulse));
    }
    V3W tangential = v3w_add(v3w_scale(c->tangent1, c->tangent_impulse_x), v3w_scale(c->tangent2, c->tangent_impulse_y));
    AnchorPair friction_anchors = { c->friction_anchor_a, c->friction_anchor_b };
    apply_impulse(c, &bodies, &friction_anchors, tangential);
    apply_angular(c, &bodies, v3w_add(v3w_scale(c->normal, c->twist_impulse), c->rolling_impulse));
    scatter_pair(states, writable, c, &bodies);
}

/// Tangential, twist and rolling friction.
static void solve_friction(Bundle* c, BodyPair* bodies, W total_normal_impulse, W twist_limit) {
    W zero = w_splat(0.0f);
    W one = w_splat(1.0f);
    AnchorPair anchors = { c->friction_anchor_a, c->friction_anchor_b };
    W max_friction = w_mul(c->friction, total_normal_impulse);
    W vt1 = relative_velocity_along(bodies, &anchors, c->tangent1);
    W vt2 = relative_velocity_along(bodies, &anchors, c->tangent2);
    W delta_x = w_neg(w_add(w_mul(c->tangent_mass_xx, vt1), w_mul(c->tangent_mass_xy, vt2)));
    W delta_y = w_neg(w_add(w_mul(c->tangent_mass_xy, vt1), w_mul(c->tangent_mass_yy, vt2)));
    W total_x = w_add(c->tangent_impulse_x, delta_x);
    W total_y = w_add(c->tangent_impulse_y, delta_y);
    W magnitude = w_sqrt(w_add(w_mul(total_x, total_x), w_mul(total_y, total_y)));
    W clamped = w_both(w_greater(magnitude, max_friction), w_greater(magnitude, zero));
    W scale = w_select(clamped, w_div(max_friction, magnitude), one);
    total_x = w_mul(total_x, scale);
    total_y = w_mul(total_y, scale);
    V3W impulse = v3w_add(v3w_scale(c->tangent1, w_sub(total_x, c->tangent_impulse_x)),
                          v3w_scale(c->tangent2, w_sub(total_y, c->tangent_impulse_y)));
    c->tangent_impulse_x = total_x;
    c->tangent_impulse_y = total_y;
    apply_impulse(c, bodies, &anchors, impulse);

    W max_twist = w_mul(c->friction, twist_limit);
    W twist_velocity = v3w_dot(v3w_sub(bodies->b.angular_velocity, bodies->a.angular_velocity), c->normal);
    W twist = w_sub(c->twist_impulse, w_mul(c->twist_mass, twist_velocity));
    twist = w_min(w_max(twist, w_neg(max_twist)), max_twist);
    W twist_delta = w_sub(twist, c->twist_impulse);
    c->twist_impulse = twist;
    apply_angular(c, bodies, v3w_scale(c->normal, twist_delta));

    W max_rolling = w_mul(c->rolling_resistance, total_normal_impulse);
    V3W relative = v3w_sub(bodies->b.angular_velocity, bodies->a.angular_velocity);
    V3W rolling = v3w_sub(c->rolling_impulse, sym_multiply(&c->rolling_mass, relative));
    W rolling_magnitude = w_sqrt(v3w_dot(rolling, rolling));
    W rolling_clamped = w_both(w_greater(rolling_magnitude, max_rolling), w_greater(rolling_magnitude, zero));
    W rolling_scale = w_select(rolling_clamped, w_div(max_rolling, rolling_magnitude), one);
    rolling = v3w_scale(rolling, rolling_scale);
    V3W rolling_delta = v3w_sub(rolling, c->rolling_impulse);
    c->rolling_impulse = rolling;
    apply_angular(c, bodies, rolling_delta);
}

/// Normal constraint of every point; friction too on the relax pass.
static void solve_constraint(Bundle* c, BodyState* states, uint32_t writable, const SolverContext* context, bool use_bias) {
    BodyPair bodies = gather_pair(states, c);
    W zero = w_splat(0.0f);
    W one = w_splat(1.0f);
    W inv_h = w_splat(context->inv_h);
    W push_out = w_splat(-context->push_out_speed);
    V3W dp = v3w_sub(bodies.b.delta_position, bodies.a.delta_position);

    W total_normal_impulse = w_splat(0.0f);
    W total_twist_limit = w_splat(0.0f);
    for (uint32_t i = 0; i < c->point_count; i++) {
        PointW* p = &c->points[i];
        V3W ds = v3w_sub(v3w_add(dp, q4w_rotate(&bodies.b.delta_rotation, p->anchor_b)),
                         q4w_rotate(&bodies.a.delta_rotation, p->anchor_a));
        W s = w_add(v3w_dot(ds, c->normal), p->base_separation);
        W separated = w_greater(s, zero);
        W bias = w_mul(s, inv_h);
        W mass_scale = one;
        W impulse_scale = zero;
        if (use_bias) {
            W soft_bias = w_max(w_mul(w_mul(c->mass_scale, c->bias_rate), s), push_out);
            bias = w_select(separated, bias, soft_bias);
            mass_scale = w_select(separated, one, c->mass_scale);
            impulse_scale = w_select(separated, zero, c->impulse_scale);
        } else {
            bias = w_select(separated, bias, zero);
        }
        AnchorPair anchors = point_anchors(p);
        W vn = relative_velocity_along(&bodies, &anchors, c->normal);
        W delta_impulse = w_sub(w_mul(w_neg(p->normal_mass), w_add(w_mul(mass_scale, vn), bias)),
                                w_mul(impulse_scale, p->normal_impulse));
        W new_impulse = w_max(w_add(p->normal_impulse, delta_impulse), zero);
        delta_impulse = w_sub(new_impulse, p->normal_impulse);
        p->normal_impulse = new_impulse;
        p->total_normal_impulse = w_add(p->total_normal_impulse, new_impulse);
        p->peak_normal_impulse = w_max(p->peak_normal_impulse, new_impulse);
        total_normal_impulse = w_add(total_normal_impulse, new_impulse);
        total_twist_limit = w_add(total_twist_limit, w_mul(p->lever_arm, new_impulse));
        apply_impulse(c, &bodies, &anchors, v3w_scale(c->normal, delta_impulse));
    }

    if (!use_bias) solve_friction(c, &bodies, total_normal_impulse, total_twist_limit);

    scatter_pair(states, writable, c, &bodies);
}

/// Restitution for points that approached faster than the threshold.
static void restitution_constraint(Bundle* c, BodyState* states, uint32_t writable, const SolverContext* context) {
    W zero = w_splat(0.0f);
    W threshold = w_splat(-context->restitution_threshold);
    W bouncy = w_greater(c->restitution, zero);
    if (!w_any(bouncy)) return;
    BodyPair bodies = gather_pair(states, c);
    for (uint32_t i = 0; i < c->point_count; i++) {
        PointW* p = &c->points[i];
        W active = w_both(w_both(bouncy, w_less_equal(p->relative_velocity, threshold)),
                          w_not_equal(p->total_normal_impulse, zero));
        AnchorPair anchors = point_anchors(p);
        W vn = relative_velocity_along(&bodies, &anchors, c->normal);
        W impulse = w_mul(w_neg(p->normal_mass), w_add(vn, w_mul(c->restitution, p->relative_velocity)));
        W new_impulse = w_max(w_add(p->normal_impulse, impulse), zero);
        impulse = w_select(active, w_sub(new_impulse, p->normal_impulse), zero);
        p->normal_impulse = w_select(active, new_impulse, p->normal_impulse);
        p->total_normal_impulse = w_add(p->total_normal_impulse, w_select(active, new_impulse, zero));
        p->peak_normal_impulse = w_select(active, w_max(p->peak_normal_impulse, new_impulse), p->peak_normal_impulse);
        apply_impulse(c, &bodies, &anchors, v3w_scale(c->normal, impulse));
    }
    scatter_pair(states, writable, c, &bodies);
}

/// Some unit vector perpendicular to the unit vector `n`.
static Vec3 perpendicular(Vec3 n) {
    if (fabsf(n.x) > 0.57735f) return v3_normalize((Vec3){ n.y, -n.x, 0.0f });
    return v3_normalize((Vec3){ 0.0f, n.z, -n.y });
}

/// Upper triangle of a symmetric basis.
static Sym3 sym_from_basis(const Basis* m) { return (Sym3){ m->x.x, m->x.y, m->x.z, m->y.y, m->y.z, m->z.z }; }

/// Scalar view of one body while preparing constraints.
typedef struct {
    Vec3 velocity, angular_velocity;
    float inverse_mass;
    Basis inverse_inertia;
    Vec3 center;
} PrepBody;

/// Inverse of the combined inverse mass along a direction at two anchors.
static float effective_mass(const PrepBody* a, const PrepBody* b, Vec3 ra, Vec3 rb, Vec3 direction) {
    Vec3 rna = v3_cross(ra, direction);
    Vec3 rnb = v3_cross(rb, direction);
    float k = a->inverse_mass + b->inverse_mass + v3_dot(rna, basis_apply(&a->inverse_inertia, rna)) +
              v3_dot(rnb, basis_apply(&b->inverse_inertia, rnb));
    return k > 0.0f ? 1.0f / k : 0.0f;
}

/// Relative anchor velocity of B against A along a direction.
static float prep_relative_velocity(const PrepBody* a, const PrepBody* b, Vec3 ra, Vec3 rb, Vec3 direction) {
    Vec3 va = v3_add(a->velocity, v3_cross(a->angular_velocity, ra));
    Vec3 vb = v3_add(b->velocity, v3_cross(b->angular_velocity, rb));
    return v3_dot(v3_sub(vb, va), direction);
}

/// Friction anchors, lever arms and tangent, twist and rolling masses.
static void prepare_friction(Scalar* c, const Manifold* manifold, const PrepBody* a, const PrepBody* b, Vec3 tangent1,
                             Vec3 tangent2, float rolling_resistance, float speculative) {
    Vec3 center_a = { 0.0f, 0.0f, 0.0f };
    Vec3 center_b = { 0.0f, 0.0f, 0.0f };
    float total_weight = 0.0f;
    float inv_tau = 1.0f / speculative;
    for (uint32_t i = 0; i < manifold->point_count; i++) {
        float weight = f32_clamp(2.0f - manifold->points[i].separation * inv_tau, MIN_FRICTION_WEIGHT, 1.0f);
        center_a = v3_add(center_a, v3_scale(c->points[i].anchor_a, weight));
        center_b = v3_add(center_b, v3_scale(c->points[i].anchor_b, weight));
        total_weight += weight;
    }
    float inv_weight = total_weight > 0.0f ? 1.0f / total_weight : 0.0f;
    Vec3 anchor_a = v3_scale(center_a, inv_weight);
    Vec3 anchor_b = v3_scale(center_b, inv_weight);
    c->friction_anchor_a = anchor_a;
    c->friction_anchor_b = anchor_b;

    for (uint32_t i = 0; i < manifold->point_count; i++) {
        c->points[i].lever_arm = v3_length(v3_sub(c->points[i].anchor_a, anchor_a));
    }

    Vec3 rta1 = v3_cross(anchor_a, tangent1);
    Vec3 rta2 = v3_cross(anchor_a, tangent2);
    Vec3 rtb1 = v3_cross(anchor_b, tangent1);
    Vec3 rtb2 = v3_cross(anchor_b, tangent2);
    float inv_mass = a->inverse_mass + b->inverse_mass;
    float kxx = inv_mass + v3_dot(rta1, basis_apply(&a->inverse_inertia, rta1)) +
                v3_dot(rtb1, basis_apply(&b->inverse_inertia, rtb1));
    float kyy = inv_mass + v3_dot(rta2, basis_apply(&a->inverse_inertia, rta2)) +
                v3_dot(rtb2, basis_apply(&b->inverse_inertia, rtb2));
    float kxy = v3_dot(rta1, basis_apply(&a->inverse_inertia, rta2)) + v3_dot(rtb1, basis_apply(&b->inverse_inertia, rtb2));
    float tangent_det = kxx * kyy - kxy * kxy;
    if (tangent_det != 0.0f) {
        float inv = 1.0f / tangent_det;
        c->tangent_mass_xx = kyy * inv;
        c->tangent_mass_xy = -kxy * inv;
        c->tangent_mass_yy = kxx * inv;
    }

    Vec3 normal = manifold->normal;
    Basis angular = basis_add(&a->inverse_inertia, &b->inverse_inertia);
    float twist = v3_dot(normal, basis_apply(&angular, normal));
    c->twist_mass = twist > 0.0f ? 1.0f / twist : 0.0f;
    if (rolling_resistance > 0.0f && basis_determinant(&angular) > 0.0f) {
        Basis inv = basis_inverse(&angular);
        c->rolling_mass = sym_from_basis(&inv);
    }
}


/// Packs a scalar vector into one lane.
static void set_v3(V3W* w, int lane, Vec3 v) {
    w_set_lane(&w->x, lane, v.x);
    w_set_lane(&w->y, lane, v.y);
    w_set_lane(&w->z, lane, v.z);
}

/// Packs a scalar symmetric matrix into one lane.
static void set_sym(Sym3W* w, int lane, Sym3 m) {
    w_set_lane(&w->xx, lane, m.xx);
    w_set_lane(&w->xy, lane, m.xy);
    w_set_lane(&w->xz, lane, m.xz);
    w_set_lane(&w->yy, lane, m.yy);
    w_set_lane(&w->yz, lane, m.yz);
    w_set_lane(&w->zz, lane, m.zz);
}

/// Copies every value field of a scalar constraint into one lane of a bundle.
static void pack_lane(Bundle* w, int lane, const Scalar* c) {
    w_set_lane(&w->inverse_mass_a, lane, c->inverse_mass_a);
    w_set_lane(&w->inverse_mass_b, lane, c->inverse_mass_b);
    set_sym(&w->inverse_inertia_a, lane, c->inverse_inertia_a);
    set_sym(&w->inverse_inertia_b, lane, c->inverse_inertia_b);
    set_v3(&w->normal, lane, c->normal);
    set_v3(&w->tangent1, lane, c->tangent1);
    set_v3(&w->tangent2, lane, c->tangent2);
    w_set_lane(&w->bias_rate, lane, c->bias_rate);
    w_set_lane(&w->mass_scale, lane, c->mass_scale);
    w_set_lane(&w->impulse_scale, lane, c->impulse_scale);
    w_set_lane(&w->friction, lane, c->friction);
    w_set_lane(&w->restitution, lane, c->restitution);
    w_set_lane(&w->rolling_resistance, lane, c->rolling_resistance);
    for (int i = 0; i < MAX_POINTS; i++) {
        PointW* pw = &w->points[i];
        const ScalarPoint* pc = &c->points[i];
        set_v3(&pw->anchor_a, lane, pc->anchor_a);
        set_v3(&pw->anchor_b, lane, pc->anchor_b);
        w_set_lane(&pw->base_separation, lane, pc->base_separation);
        w_set_lane(&pw->normal_mass, lane, pc->normal_mass);
        w_set_lane(&pw->relative_velocity, lane, pc->relative_velocity);
        w_set_lane(&pw->normal_impulse, lane, pc->normal_impulse);
        w_set_lane(&pw->total_normal_impulse, lane, pc->total_normal_impulse);
        w_set_lane(&pw->peak_normal_impulse, lane, pc->peak_normal_impulse);
        w_set_lane(&pw->lever_arm, lane, pc->lever_arm);
    }
    set_v3(&w->friction_anchor_a, lane, c->friction_anchor_a);
    set_v3(&w->friction_anchor_b, lane, c->friction_anchor_b);
    w_set_lane(&w->tangent_mass_xx, lane, c->tangent_mass_xx);
    w_set_lane(&w->tangent_mass_xy, lane, c->tangent_mass_xy);
    w_set_lane(&w->tangent_mass_yy, lane, c->tangent_mass_yy);
    w_set_lane(&w->tangent_impulse_x, lane, c->tangent_impulse_x);
    w_set_lane(&w->tangent_impulse_y, lane, c->tangent_impulse_y);
    w_set_lane(&w->twist_mass, lane, c->twist_mass);
    w_set_lane(&w->twist_impulse, lane, c->twist_impulse);
    set_sym(&w->rolling_mass, lane, c->rolling_mass);
    set_v3(&w->rolling_impulse, lane, c->rolling_impulse);
}

/// Runs `fn` over [0, count) on the pool when `parallel`, else inline.
static void run_range(TaskPool* pool, bool parallel, size_t count, size_t grain, TaskRangeFn fn, void* data) {
    if (pool != NULL && parallel) {
        task_pool_parallel_for(pool, count, grain, fn, data);
    } else {
        fn(data, 0, count);
    }
}

/// A body state at rest with an identity rotation delta.
static BodyState body_state_new(void) { return (BodyState){ .delta_rotation = { 0.0f, 0.0f, 0.0f, 1.0f } }; }

/// Body properties of a fixed body at the origin.
static BodyProps body_props_new(void) { return (BodyProps){ .rotation = { 0.0f, 0.0f, 0.0f, 1.0f } }; }

/// Fills body states and properties from the active bodies.
static void build_bodies(ContactSolver* s, const SolverInputs* inputs) {
    const SolverContext* context = &inputs->context;
    uint32_t count = (uint32_t)inputs->active_count;
    s->writable_count = count;
    s->dummy_index = count;
    s->states.len = 0;
    s->props.len = 0;
    for (uint32_t i = 0; i < count + 1u; ++i) {
        ARRAY_PUSH(s->states, body_state_new());
        ARRAY_PUSH(s->props, body_props_new());
    }
    s->static_lookup.len = 0;
    for (size_t i = 0; i < inputs->static_count; ++i) ARRAY_PUSH(s->static_lookup, null_index);
    for (uint32_t i = 0; i < count; ++i) {
        const RigidBody* body = inputs->active_bodies[i].body;
        BodyState* state = &s->states.data[i];
        BodyProps* props = &s->props.data[i];
        state->velocity = body->linear_velocity;
        state->angular_velocity = body->angular_velocity;
        props->center = world_center_of_mass(body);
        props->inverse_mass = body->inverse_mass;
        props->inverse_inertia = body->inverse_inertia_world;
        props->rotation = body->rotation;
        props->force = v3_add(v3_add(body->applied_force, body->constant_force),
                              v3_scale(context->gravity, body->gravity_scale * body->mass));
        props->torque = v3_add(body->applied_torque, body->constant_torque);
        props->linear_damping = 1.0f / (1.0f + context->h * body->linear_damp);
        props->angular_damping = 1.0f / (1.0f + context->h * body->angular_damp);
    }
}

/// Slot of a static body, created read-only on first use.
static uint32_t read_only_static(ContactSolver* s, const SolverInputs* inputs, uint32_t fixed_index) {
    if (fixed_index >= inputs->static_count) return s->dummy_index;
    uint32_t entry = s->static_lookup.data[fixed_index];
    if (entry == null_index) {
        const KinematicMotion* motion = &inputs->static_motions[fixed_index];
        BodyState state = body_state_new();
        state.velocity = motion->velocity;
        state.angular_velocity = motion->angular_velocity;
        BodyProps props = body_props_new();
        props.center = motion->center;
        entry = (uint32_t)s->states.len;
        s->static_lookup.data[fixed_index] = entry;
        ARRAY_PUSH(s->states, state);
        ARRAY_PUSH(s->props, props);
    }
    return entry;
}

/// Lists contacts colour by colour with their body slots.
static void collect_entries(ContactSolver* s, const SolverInputs* inputs) {
    s->entries.len = 0;
    s->colors.len = 0;
    for (uint32_t color = 0; color < GRAPH_COLOR_COUNT; ++color) {
        const U32Span* list = &inputs->colors[color];
        for (size_t k = 0; k < list->len; ++k) {
            uint32_t index = list->data[k];
            if (color == OVERFLOW_COLOR) {
                fprintf(stderr, "solver: the overflow colour is not supported\n");
                exit(4);
            }
            const Contact* contact = &inputs->contacts[index];
            uint32_t body_a = inputs->body_local[contact->shape_a.body];
            bool b_fixed = contact->shape_b.is_static;
            uint32_t body_b =
                b_fixed ? read_only_static(s, inputs, contact->shape_b.body) : inputs->body_local[contact->shape_b.body];
            ARRAY_PUSH(s->entries, (Entry){ index, body_a, body_b, b_fixed });
            ARRAY_PUSH(s->colors, (uint8_t)color);
        }
    }
}

/// Scalar view of the body in a slot.
static PrepBody prep_body(const ContactSolver* s, uint32_t body) {
    const BodyProps* props = &s->props.data[body];
    PrepBody view = { s->states.data[body].velocity, s->states.data[body].angular_velocity, props->inverse_mass,
                      props->inverse_inertia, props->center };
    return view;
}

/// Builds the scalar constraint of one contact.
static void prepare_entry(const ContactSolver* s, const SolverInputs* inputs, const Entry* entry, Scalar* out) {
    const SolverContext* context = &inputs->context;
    Contact* contact = &inputs->contacts[entry->contact];
    PrepBody a = prep_body(s, entry->body_a);
    PrepBody b = prep_body(s, entry->body_b);

    Scalar c;
    memset(&c, 0, sizeof c);
    c.body_a = entry->body_a;
    c.body_b = entry->body_b;
    c.contact = entry->contact;
    Manifold* manifold = &contact->manifold;
    c.point_count = manifold->point_count;
    Vec3 n = manifold->normal;
    Vec3 tangent1 = perpendicular(n);
    Vec3 tangent2 = v3_cross(tangent1, n);
    c.normal = n;
    c.tangent1 = tangent1;
    c.tangent2 = tangent2;
    const Softness* softness = entry->b_fixed ? &context->static_softness : &context->contact_softness;
    c.bias_rate = softness->bias_rate;
    c.mass_scale = softness->mass_scale;
    c.impulse_scale = softness->impulse_scale;
    c.friction = contact->friction;
    c.restitution = contact->restitution;
    c.rolling_resistance = contact->rolling_resistance;
    c.inverse_mass_a = a.inverse_mass;
    c.inverse_mass_b = b.inverse_mass;
    c.inverse_inertia_a = sym_from_basis(&a.inverse_inertia);
    c.inverse_inertia_b = sym_from_basis(&b.inverse_inertia);

    for (uint32_t i = 0; i < manifold->point_count; i++) {
        ManifoldPoint* point = &manifold->points[i];
        ScalarPoint* pc = &c.points[i];
        Vec3 anchor_a = v3_sub(point->point, a.center);
        Vec3 anchor_b = v3_sub(point->point, b.center);
        pc->anchor_a = anchor_a;
        pc->anchor_b = anchor_b;
        pc->base_separation = point->separation - v3_dot(v3_sub(anchor_b, anchor_a), n);
        pc->normal_mass = effective_mass(&a, &b, anchor_a, anchor_b, n);
        pc->relative_velocity = prep_relative_velocity(&a, &b, anchor_a, anchor_b, n);
        pc->normal_impulse = point->normal_impulse;
        point->relative_velocity = pc->relative_velocity;
    }
    prepare_friction(&c, manifold, &a, &b, tangent1, tangent2, contact->rolling_resistance,
                     speculative_scale * context->linear_slop);
    const FrictionImpulses* impulses = &contact->friction_impulses;
    c.tangent_impulse_x = impulses->tangent_x;
    c.tangent_impulse_y = impulses->tangent_y;
    c.twist_impulse = impulses->twist;
    c.rolling_impulse = impulses->rolling;
    *out = c;
}

/// Prepares the constraints of entries [begin, end).
static void prepare_range(void* data, size_t begin, size_t end) {
    SolveJob* job = data;
    ContactSolver* s = job->solver;
    for (size_t i = begin; i < end; ++i) prepare_entry(s, job->inputs, &s->entries.data[i], &s->scalars.data[i]);
}

/// Packs bundles [begin, end) from the sorted scalar constraints.
static void pack_range(void* data, size_t begin, size_t end) {
    SolveJob* job = data;
    ContactSolver* s = job->solver;
    for (size_t bundle_index = begin; bundle_index < end; ++bundle_index) {
        uint32_t color = 0;
        while (s->color_end[color] <= bundle_index) ++color;
        uint32_t first = s->color_start[color] + ((uint32_t)bundle_index - s->color_begin[color]) * LANES;
        uint32_t last = first + LANES < s->color_start[color + 1u] ? first + LANES : s->color_start[color + 1u];
        Bundle* bundle = &s->bundles.data[bundle_index];
        memset(bundle, 0, sizeof *bundle);
        for (int lane = 0; lane < LANES; lane++) {
            bundle->body_a[lane] = s->dummy_index;
            bundle->body_b[lane] = s->dummy_index;
            bundle->contact[lane] = null_index;
        }
        for (uint32_t member = first; member < last; member++) {
            int lane = (int)(member - first);
            const Scalar* source = &s->scalars.data[s->order.data[member]];
            bundle->body_a[lane] = source->body_a;
            bundle->body_b[lane] = source->body_b;
            bundle->contact[lane] = source->contact;
            if (bundle->point_count < source->point_count) bundle->point_count = source->point_count;
            pack_lane(bundle, lane, source);
        }
    }
}

/// Sorts constraints by colour and packs each colour into bundles.
static void bundle_colors(ContactSolver* s, const SolverInputs* inputs) {
    uint32_t counts[GRAPH_COLOR_COUNT + 1] = { 0 };
    for (size_t i = 0; i < s->colors.len; i++) ++counts[s->colors.data[i] + 1u];
    for (uint32_t color = 0; color < GRAPH_COLOR_COUNT; color++) counts[color + 1u] += counts[color];
    memcpy(s->color_start, counts, sizeof counts);
    ARRAY_RESIZE(s->order, s->entries.len);
    for (uint32_t i = 0; i < s->entries.len; i++) s->order.data[counts[s->colors.data[i]]++] = i;

    uint32_t bundles = 0;
    for (uint32_t color = 0; color < OVERFLOW_COLOR; color++) {
        uint32_t members = s->color_start[color + 1u] - s->color_start[color];
        s->color_begin[color] = bundles;
        bundles += (members + LANES - 1u) / LANES;
        s->color_end[color] = bundles;
    }

    ARRAY_RESIZE(s->bundles, bundles);
    SolveJob job = { s, inputs };
    run_range(inputs->pool, bundles >= PARALLEL_THRESHOLD / LANES, bundles, 32, pack_range, &job);
}

/// Prepares every constraint, in parallel when large enough, and bundles
/// them.
static void prepare_constraints(ContactSolver* s, const SolverInputs* inputs) {
    collect_entries(s, inputs);
    ARRAY_RESIZE(s->scalars, s->entries.len);
    SolveJob job = { s, inputs };
    run_range(inputs->pool, s->entries.len >= PARALLEL_THRESHOLD, s->entries.len, 128, prepare_range, &job);
    bundle_colors(s, inputs);
}

/// Appends a stage, ignoring empty ranges.
static void add_stage(ContactSolver* s, StageKind kind, uint32_t color, uint32_t begin, uint32_t end, uint32_t grain) {
    if (begin == end) return;
    Stage stage = { kind, color, begin, end, grain, (end - begin + grain - 1u) / grain };
    ARRAY_PUSH(s->stages, stage);
}

/// Adds one stage per colour for a constraint kind.
static void add_constraint_stages(ContactSolver* s, StageKind kind) {
    uint32_t grain = CONTACTS_PER_BLOCK / LANES > 1u ? CONTACTS_PER_BLOCK / LANES : 1u;
    for (uint32_t color = 0; color < OVERFLOW_COLOR; ++color) {
        add_stage(s, kind, color, s->color_begin[color], s->color_end[color], grain);
    }
}

/// Builds the stage list for every substep plus restitution.
static void plan_stages(ContactSolver* s, const SolverContext* context) {
    s->stages.len = 0;
    for (uint32_t substep = 0; substep < context->substeps; ++substep) {
        add_stage(s, INTEGRATE_VELOCITIES, 0, 0, s->writable_count, BODIES_PER_BLOCK);
        add_constraint_stages(s, WARM_START);
        add_constraint_stages(s, SOLVE_BIASED);
        add_stage(s, INTEGRATE_POSITIONS, 0, 0, s->writable_count, BODIES_PER_BLOCK);
        add_constraint_stages(s, SOLVE_RELAX);
    }
    add_constraint_stages(s, RESTITUTION);
    if (s->progress_capacity < s->stages.len) {
        free(s->progress);
        s->progress_capacity = s->stages.len * 2;
        s->progress = xalloc(s->progress_capacity * sizeof(StageProgress));
    }
    for (size_t i = 0; i < s->stages.len; ++i) {
        atomic_store_explicit(&s->progress[i].next, 0, memory_order_relaxed);
        atomic_store_explicit(&s->progress[i].done, 0, memory_order_relaxed);
    }
}

/// Velocity integration of bodies [begin, end) under force and damping.
static void integrate_velocities(ContactSolver* s, uint32_t begin, uint32_t end, const SolverContext* context) {
    for (uint32_t i = begin; i < end; ++i) {
        BodyState* state = &s->states.data[i];
        const BodyProps* props = &s->props.data[i];
        float h = context->h;
        state->velocity = v3_scale(v3_add(state->velocity, v3_scale(props->force, props->inverse_mass * h)), props->linear_damping);
        state->angular_velocity =
            v3_scale(v3_add(state->angular_velocity, v3_scale(basis_apply(&props->inverse_inertia, props->torque), h)),
                     props->angular_damping);
    }
}

/// Position and rotation integration of bodies [begin, end) with the
/// per-step rotation cap.
static void integrate_positions(ContactSolver* s, uint32_t begin, uint32_t end, const SolverContext* context) {
    float h = context->h;
    float max_angular_speed = 0.25f * pi * context->inv_dt;
    for (uint32_t i = begin; i < end; ++i) {
        BodyState* state = &s->states.data[i];
        BodyProps* props = &s->props.data[i];
        float angular_speed = v3_length(state->angular_velocity);
        if (angular_speed > max_angular_speed) {
            state->angular_velocity = v3_scale(state->angular_velocity, max_angular_speed / angular_speed);
        }
        Vec3 step = v3_scale(state->velocity, h);
        Vec3 turn = v3_scale(state->angular_velocity, h);
        props->center = v3_add(props->center, step);
        props->rotation = integrate_rotation(props->rotation, turn);
        state->delta_position = v3_add(state->delta_position, step);
        state->delta_rotation = integrate_rotation(state->delta_rotation, turn);
    }
}

/// Executes one block of a stage.
static void run_block(ContactSolver* s, const Stage* stage, uint32_t block, const SolverContext* context) {
    BodyState* states = s->states.data;
    uint32_t begin = stage->begin + block * stage->grain;
    uint32_t end = begin + stage->grain < stage->end ? begin + stage->grain : stage->end;
    uint32_t writable = s->writable_count;
    switch (stage->kind) {
    case INTEGRATE_VELOCITIES:
        integrate_velocities(s, begin, end, context);
        return;
    case INTEGRATE_POSITIONS:
        integrate_positions(s, begin, end, context);
        return;
    default:
        break;
    }
    for (uint32_t i = begin; i < end; ++i) {
        Bundle* c = &s->bundles.data[i];
        switch (stage->kind) {
        case WARM_START:
            warm_start_constraint(c, states, writable);
            break;
        case SOLVE_BIASED:
            solve_constraint(c, states, writable, context, true);
            break;
        case SOLVE_RELAX:
            solve_constraint(c, states, writable, context, false);
            break;
        case RESTITUTION:
            restitution_constraint(c, states, writable, context);
            break;
        default:
            break;
        }
    }
}

/// Claims and runs blocks of one stage until none are left.
static void stage_work(ContactSolver* s, uint32_t index, const SolverContext* context) {
    const Stage* stage = &s->stages.data[index];
    StageProgress* progress = &s->progress[index];
    for (;;) {
        uint32_t block = atomic_fetch_add_explicit(&progress->next, 1, memory_order_relaxed);
        if (block >= stage->blocks) return;
        run_block(s, stage, block, context);
        atomic_fetch_add_explicit(&progress->done, 1, memory_order_release);
    }
}

/// Worker 0 advances the stages after each barrier; the others join every
/// stage they see published.
static void stage_worker(void* data, uint32_t worker) {
    StageRun* run = data;
    ContactSolver* s = run->solver;
    if (worker == 0) {
        for (uint32_t index = 0; index < run->stage_count; ++index) {
            if (index != 0) atomic_store_explicit(&run->current, index, memory_order_release);
            stage_work(s, index, run->context);
            while (atomic_load_explicit(&s->progress[index].done, memory_order_acquire) != s->stages.data[index].blocks) {
                task_pool_relax();
            }
        }
        atomic_store_explicit(&run->current, run->stage_count, memory_order_release);
        return;
    }
    uint32_t seen = null_index;
    for (;;) {
        uint32_t index = atomic_load_explicit(&run->current, memory_order_acquire);
        if (index >= run->stage_count) return;
        if (index == seen) {
            task_pool_relax();
            continue;
        }
        seen = index;
        stage_work(s, index, run->context);
    }
}

/// Runs all stages, spread over the pool when there are enough contacts.
static void run_stages(ContactSolver* s, const SolverContext* context, TaskPool* pool) {
    uint32_t stage_count = (uint32_t)s->stages.len;
    bool parallel = pool != NULL && task_pool_thread_count(pool) > 1 && s->entries.len >= PARALLEL_THRESHOLD;
    if (!parallel) {
        for (uint32_t index = 0; index < stage_count; ++index) {
            const Stage* stage = &s->stages.data[index];
            for (uint32_t block = 0; block < stage->blocks; ++block) run_block(s, stage, block, context);
        }
        return;
    }
    StageRun run = { .solver = s, .context = context, .stage_count = stage_count };
    atomic_init(&run.current, 0);
    task_pool_run(pool, stage_worker, &run);
}

/// Writes impulses of bundles [begin, end) back to their contacts.
static void store_contacts_range(void* data, size_t begin, size_t end) {
    SolveJob* job = data;
    for (size_t i = begin; i < end; ++i) {
        const Bundle* c = &job->solver->bundles.data[i];
        for (int lane = 0; lane < LANES; lane++) {
            if (c->contact[lane] == null_index) continue;
            Contact* contact = &job->inputs->contacts[c->contact[lane]];
            Manifold* manifold = &contact->manifold;
            for (uint32_t k = 0; k < manifold->point_count; k++) {
                ManifoldPoint* point = &manifold->points[k];
                point->normal_impulse = w_lane(c->points[k].normal_impulse, lane);
                point->total_normal_impulse = w_lane(c->points[k].total_normal_impulse, lane);
                point->peak_normal_impulse = w_lane(c->points[k].peak_normal_impulse, lane);
            }
            FrictionImpulses* friction = &contact->friction_impulses;
            friction->tangent_x = w_lane(c->tangent_impulse_x, lane);
            friction->tangent_y = w_lane(c->tangent_impulse_y, lane);
            friction->twist = w_lane(c->twist_impulse, lane);
            friction->rolling = (Vec3){ w_lane(c->rolling_impulse.x, lane), w_lane(c->rolling_impulse.y, lane),
                                        w_lane(c->rolling_impulse.z, lane) };
        }
    }
}

/// Writes velocities, poses and deltas of bodies [begin, end) back.
static void store_bodies_range(void* data, size_t begin, size_t end) {
    SolveJob* job = data;
    for (size_t i = begin; i < end; ++i) {
        const BodyState* state = &job->solver->states.data[i];
        const BodyProps* props = &job->solver->props.data[i];
        RigidBody* body = job->inputs->active_bodies[i].body;
        body->linear_velocity = state->velocity;
        body->angular_velocity = state->angular_velocity;
        set_pose(body, props->center, props->rotation);
        job->inputs->deltas[i] = (BodyDelta){ state->delta_position, state->delta_rotation };
    }
}

/// Writes impulses, velocities, poses and deltas back.
static void store_results(ContactSolver* s, const SolverInputs* inputs) {
    bool parallel = s->entries.len >= PARALLEL_THRESHOLD;
    SolveJob job = { s, inputs };
    run_range(inputs->pool, parallel, s->bundles.len, 64, store_contacts_range, &job);
    run_range(inputs->pool, parallel, s->writable_count, 128, store_bodies_range, &job);
}

ContactSolver* contact_solver_create(void) { return xalloc(sizeof(ContactSolver)); }

void contact_solver_destroy(ContactSolver* solver) {
    ARRAY_FREE(solver->states);
    ARRAY_FREE(solver->props);
    ARRAY_FREE(solver->static_lookup);
    ARRAY_FREE(solver->entries);
    ARRAY_FREE(solver->colors);
    ARRAY_FREE(solver->scalars);
    ARRAY_FREE(solver->order);
    ARRAY_FREE(solver->bundles);
    ARRAY_FREE(solver->stages);
    free(solver->progress);
    free(solver);
}

void contact_solver_solve(ContactSolver* solver, const SolverInputs* inputs) {
    build_bodies(solver, inputs);
    prepare_constraints(solver, inputs);
    plan_stages(solver, &inputs->context);
    run_stages(solver, &inputs->context, inputs->pool);
    store_results(solver, inputs);
}
