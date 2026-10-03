#include "boxbox.hpp"

#include <algorithm>

#include "box_collision.hpp"
#include "hash.hpp"

namespace bench {

using vm::BoxPose;
using vm::Vec3;

BoxBox::BoxBox() : pairs_(pairs_n), manifolds_(pairs_n) {
    Rng rng{0xb0b0};
    for (uint32_t i = 0; i < pairs_n; i++) {
        Pair& p = pairs_[i];
        const vm::Quat rotation = vm::random_quat(rng);
        p.a.half_extents = vm::random_vec3(rng, 0.25f, 1.5f);
        p.a.center = vm::random_vec3(rng, 0.0f, 1000.0f);
        p.a.basis = vm::basis_from_quat(rotation);
        p.b.half_extents = vm::random_vec3(rng, 0.25f, 1.5f);
        if (i % 4 == 0) {
            const vm::Quat yaw = vm::normalize(vm::Quat{0.0f, rng.unit() - 0.5f, 0.0f, 1.0f});
            p.b.basis = vm::basis_from_quat(rotation * yaw);
            const Vec3 slide = vm::random_vec3(rng, -0.3f, 0.3f);
            const float lift = p.a.half_extents.y + p.b.half_extents.y - 0.01f;
            p.b.center = (p.a.center + p.a.basis.y * lift) + (p.a.basis.x * slide.x + p.a.basis.z * slide.z);
        } else {
            p.b.basis = vm::basis_from_quat(vm::random_quat(rng));
            const float reach = (vm::length(p.a.half_extents) + vm::length(p.b.half_extents)) * 0.6f;
            p.b.center = p.a.center + vm::random_vec3(rng, -1.0f, 1.0f) * reach;
        }
        p.velocity = vm::random_vec3(rng, -0.02f, 0.02f);
    }
}

uint64_t BoxBox::run() {
    std::fill(manifolds_.begin(), manifolds_.end(), phys::Manifold{});
    for (int f = 0; f < frames; f++) {
        const float t = static_cast<float>(f);
        for (uint32_t i = 0; i < pairs_n; i++) {
            const Pair& p = pairs_[i];
            BoxPose b = p.b;
            b.center = b.center + p.velocity * t;
            phys::collide_boxes(p.a, b, phys::CollisionTolerances{}, manifolds_[i]);
        }
    }
    uint64_t h = 0;
    for (const phys::Manifold& m : manifolds_) {
        h = hash_add(h, static_cast<uint64_t>(f32_bits(m.normal.x)) | (static_cast<uint64_t>(f32_bits(m.normal.z)) << 32));
        h = hash_add(h, m.point_count);
        for (uint32_t k = 0; k < m.point_count; k++) {
            const phys::ManifoldPoint& p = m.points[k];
            h = hash_add(h, static_cast<uint64_t>(f32_bits(p.point.y)) | (static_cast<uint64_t>(f32_bits(p.separation)) << 32));
            h = hash_add(h, static_cast<uint64_t>(p.feature_id) | (static_cast<uint64_t>(p.persisted) << 32));
        }
    }
    return h;
}

} // namespace bench
