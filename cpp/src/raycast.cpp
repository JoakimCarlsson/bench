#include "raycast.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <utility>

#include "hash.hpp"

namespace bench {

namespace {

/// Slab test of a ray against an oriented box, in the box's frame. A hit
/// needs an entry fraction in (0, max_fraction].
Raycast::Hit ray_cast_box(const Raycast::Ray& ray, const vm::BoxPose& box, float max_fraction) {
    const vm::Basis inverse_basis = vm::transposed(box.basis);
    const vm::Vec3 origin = inverse_basis * (ray.origin - box.center);
    const vm::Vec3 translation = inverse_basis * ray.translation;
    const std::array<float, 3> origins{origin.x, origin.y, origin.z};
    const std::array<float, 3> translations{translation.x, translation.y, translation.z};
    const std::array<float, 3> extents{box.half_extents.x, box.half_extents.y, box.half_extents.z};
    float entry_fraction = 0.0f;
    float exit_fraction = max_fraction;
    int entry_axis = -1;
    float entry_sign = 0.0f;

    for (int axis = 0; axis < 3; axis++) {
        const float origin_axis = origins[axis];
        const float translation_axis = translations[axis];
        const float extent = extents[axis];
        if (std::fabs(translation_axis) <= 1e-8f) {
            if (origin_axis < -extent || origin_axis > extent) return {};
            continue;
        }
        float first = (-extent - origin_axis) / translation_axis;
        float last = (extent - origin_axis) / translation_axis;
        float normal_sign = -1.0f;
        if (first > last) {
            std::swap(first, last);
            normal_sign = 1.0f;
        }
        if (first > entry_fraction) {
            entry_fraction = first;
            entry_axis = axis;
            entry_sign = normal_sign;
        }
        exit_fraction = std::min(exit_fraction, last);
        if (entry_fraction > exit_fraction) return {};
    }

    if (entry_axis < 0 || entry_fraction <= 0.0f || entry_fraction > max_fraction) return {};

    vm::Vec3 local_normal{};
    if (entry_axis == 0) {
        local_normal.x = entry_sign;
    } else if (entry_axis == 1) {
        local_normal.y = entry_sign;
    } else {
        local_normal.z = entry_sign;
    }
    return {ray.origin + ray.translation * entry_fraction, box.basis * local_normal, entry_fraction, true};
}

} // namespace

Raycast::Raycast() : boxes_(boxes_n), rays_(rays_n) {
    Rng rng{0x7a1c};
    for (vm::BoxPose& b : boxes_) {
        b.half_extents = vm::random_vec3(rng, 0.25f, 2.0f);
        b.center = vm::random_vec3(rng, 0.0f, 64.0f);
        b.basis = vm::basis_from_quat(vm::random_quat(rng));
    }
    for (Ray& r : rays_) {
        r.origin = vm::random_vec3(rng, -8.0f, 72.0f);
        r.translation = vm::random_vec3(rng, -1.0f, 1.0f) * 80.0f;
    }
}

uint64_t Raycast::run() {
    uint64_t h = 0;
    for (const Ray& ray : rays_) {
        Hit closest{{}, {}, 1.0f, false};
        uint32_t closest_box = 0xffffffffu;
        for (uint32_t j = 0; j < boxes_n; j++) {
            const Hit hit = ray_cast_box(ray, boxes_[j], closest.fraction);
            if (hit.hit) {
                closest = hit;
                closest_box = j;
            }
        }
        h = hash_add(h, static_cast<uint64_t>(f32_bits(closest.fraction)) | (static_cast<uint64_t>(closest_box) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(closest.point.x)) | (static_cast<uint64_t>(f32_bits(closest.normal.y)) << 32));
    }
    return h;
}

} // namespace bench
