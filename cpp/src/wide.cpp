#include "wide.hpp"

#include <algorithm>

#include "hash.hpp"

namespace bench {

Wide::Wide() {
    phys::SolverContext& c = context_;
    c.dt = 1.0f / 60.0f;
    c.inv_dt = 1.0f / c.dt;
    c.substeps = 4;
    c.h = c.dt / static_cast<float>(c.substeps);
    c.inv_h = 1.0f / c.h;
    c.gravity = {0.0f, -9.81f, 0.0f};
    const float contact_hertz = std::min(30.0f, 0.125f * c.inv_h);
    c.contact_softness = phys::make_softness(contact_hertz, 10.0f, c.h);
    c.static_softness = phys::make_softness(2.0f * contact_hertz, 0.5f * 10.0f, c.h);
    c.push_out_speed = 3.0f;
    c.restitution_threshold = 1.0f;
    c.linear_slop = 0.005f;

    Rng rng{0x501e};
    initial_bodies_.resize(bodies_n);
    for (uint32_t cz = 0; cz < grid; cz++) {
        for (uint32_t cx = 0; cx < grid; cx++) {
            for (uint32_t level = 0; level < height; level++) {
                const uint32_t i = (cz * grid + cx) * height + level;
                phys::RigidBody& b = initial_bodies_[i];
                const vm::Vec3 half = vm::random_vec3(rng, 0.4f, 0.5f);
                vm::Quat tilt;
                tilt.x = rng.unit() * 0.1f - 0.05f;
                tilt.y = rng.unit() * 0.2f - 0.1f;
                tilt.z = rng.unit() * 0.1f - 0.05f;
                tilt.w = 1.0f;
                b.rotation = vm::normalize(tilt);
                b.transform.basis = vm::basis_from_quat(b.rotation);
                b.transform.origin = {static_cast<float>(cx) * 1.1f, static_cast<float>(level) * 1.0f + 0.5f,
                                      static_cast<float>(cz) * 1.1f};
                phys::set_box_mass(b, half);
                b.linear_velocity = vm::random_vec3(rng, -0.1f, 0.1f);
                if (i % 16 == 0) b.linear_velocity.y = -2.0f;
                b.angular_velocity = vm::random_vec3(rng, -0.1f, 0.1f);
                b.angular_damp = 0.05f;
            }
        }
    }

    const auto add_contact = [&](uint32_t a, phys::ShapeRef b, vm::Vec3 normal) -> phys::Contact& {
        const auto index = static_cast<uint32_t>(initial_contacts_.size());
        phys::Contact& contact = initial_contacts_.emplace_back();
        contact.shape_a = {a, false};
        contact.shape_b = b;
        contact.alive = true;
        contact.manifold.normal = normal;
        contact.friction = 0.6f;
        contact.restitution = index % 8 == 0 ? 0.3f : 0.0f;
        contact.rolling_resistance = index % 5 == 0 ? 0.05f : 0.0f;
        return contact;
    };
    const auto add_point = [&](phys::Contact& contact, vm::Vec3 point) {
        phys::ManifoldPoint& p = contact.manifold.points[contact.manifold.point_count++];
        p.point = point;
        p.separation = rng.unit() * 0.025f - 0.02f;
    };
    constexpr std::array<std::array<float, 2>, 4> corners{{{1.0f, 1.0f}, {-1.0f, 1.0f}, {-1.0f, -1.0f}, {1.0f, -1.0f}}};
    const vm::Vec3 half{0.45f, 0.45f, 0.45f};
    for (uint32_t cz = 0; cz < grid; cz++) {
        for (uint32_t cx = 0; cx < grid; cx++) {
            for (uint32_t level = 0; level < height; level++) {
                const uint32_t i = (cz * grid + cx) * height + level;
                const vm::Vec3 center = initial_bodies_[i].transform.origin;
                phys::Contact& down = level == 0 ? add_contact(i, {0, true}, {0.0f, -1.0f, 0.0f})
                                                 : add_contact(i, {i - 1, false}, {0.0f, -1.0f, 0.0f});
                for (const auto& corner : corners) add_point(down, center + vm::Vec3{corner[0] * half.x, -half.y, corner[1] * half.z});
                if (cx + 1 < grid && rng.next() % 4 == 0) {
                    phys::Contact& side = add_contact(i, {i + height, false}, {1.0f, 0.0f, 0.0f});
                    add_point(side, center + vm::Vec3{half.x, 0.5f * half.y, 0.0f});
                    add_point(side, center + vm::Vec3{half.x, -0.5f * half.y, 0.0f});
                }
                if (cz + 1 < grid && rng.next() % 4 == 0) {
                    phys::Contact& side = add_contact(i, {i + grid * height, false}, {0.0f, 0.0f, 1.0f});
                    add_point(side, center + vm::Vec3{0.0f, 0.5f * half.y, half.z});
                    add_point(side, center + vm::Vec3{0.0f, -0.5f * half.y, half.z});
                }
            }
        }
    }

    graph_.reserve_bodies(bodies_n);
    for (uint32_t index = 0; index < initial_contacts_.size(); ++index) {
        phys::Contact& contact = initial_contacts_[index];
        const bool b_static = contact.shape_b.is_static;
        const phys::GraphSlot slot = graph_.add(index, {contact.shape_a.body, b_static ? 0u : contact.shape_b.body, b_static});
        contact.color = slot.color;
        contact.local = slot.local;
    }
    for (uint32_t color = 0; color < phys::graph_color_count; ++color) colors_[color] = graph_.contacts(color);

    bodies_ = initial_bodies_;
    contacts_ = initial_contacts_;
    body_local_.resize(bodies_n);
    active_.resize(bodies_n);
    for (uint32_t i = 0; i < bodies_n; ++i) {
        body_local_[i] = i;
        active_[i] = {&bodies_[i], i};
    }
    static_motions_.resize(1);
    deltas_.resize(bodies_n);
}

uint64_t Wide::run() {
    std::copy(initial_bodies_.begin(), initial_bodies_.end(), bodies_.begin());
    std::copy(initial_contacts_.begin(), initial_contacts_.end(), contacts_.begin());
    phys::SolverInputs inputs{};
    inputs.contacts = contacts_;
    inputs.colors = colors_;
    inputs.body_local = body_local_;
    inputs.active_bodies = active_;
    inputs.static_motions = static_motions_;
    inputs.deltas = deltas_;
    inputs.context = context_;
    for (int step = 0; step < steps; ++step) solver_.solve(inputs);
    uint64_t h = 0;
    for (uint32_t i = 0; i < bodies_n; ++i) {
        const phys::RigidBody& b = bodies_[i];
        h = hash_add(h, static_cast<uint64_t>(f32_bits(b.linear_velocity.x)) | (static_cast<uint64_t>(f32_bits(b.linear_velocity.y)) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(b.transform.origin.y)) | (static_cast<uint64_t>(f32_bits(b.rotation.w)) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(b.angular_velocity.z)) | (static_cast<uint64_t>(f32_bits(deltas_[i].rotation.x)) << 32));
    }
    for (const phys::Contact& c : contacts_) {
        const phys::Manifold& m = c.manifold;
        for (uint32_t k = 0; k < m.point_count; ++k) {
            const phys::ManifoldPoint& p = m.points[k];
            h = hash_add(h, static_cast<uint64_t>(f32_bits(p.normal_impulse)) | (static_cast<uint64_t>(f32_bits(p.total_normal_impulse)) << 32));
            h = hash_add(h, static_cast<uint64_t>(f32_bits(p.peak_normal_impulse)) | (static_cast<uint64_t>(f32_bits(p.relative_velocity)) << 32));
        }
        const phys::FrictionImpulses& f = c.friction_impulses;
        h = hash_add(h, static_cast<uint64_t>(f32_bits(f.tangent_x)) | (static_cast<uint64_t>(f32_bits(f.tangent_y)) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(f.twist)) | (static_cast<uint64_t>(f32_bits(f.rolling.y)) << 32));
    }
    return h;
}

} // namespace bench
