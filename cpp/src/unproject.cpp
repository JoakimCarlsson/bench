#include "unproject.hpp"

#include <array>

#include "hash.hpp"

namespace bench {

namespace {

constexpr std::array<vm::Vec3, 4> ndc{{
    {-0.5f, -0.5f, -1.0f},
    {0.5f, -0.5f, 0.0f},
    {0.5f, 0.5f, 0.5f},
    {-0.25f, 0.75f, 0.999f},
}};

/// View-projection matrix of a camera.
vm::Mat4 view_projection(const Unproject::Camera& c) {
    const vm::Mat4 view = vm::to_mat4(vm::inverse_orthonormal(vm::looking_at(c.eye, c.target, {0.0f, 1.0f, 0.0f})));
    return vm::perspective(c.tan_half_fov, c.aspect, c.z_near, c.z_far) * view;
}

} // namespace

Unproject::Unproject() : cameras_(cameras_n) {
    Rng rng{0xca3e};
    for (Camera& c : cameras_) {
        c.eye = vm::random_vec3(rng, -50.0f, 50.0f);
        c.target = c.eye + vm::random_vec3(rng, -10.0f, 10.0f);
        c.tan_half_fov = 0.3f + rng.unit() * 1.0f;
        c.aspect = 1.0f + rng.unit() * 1.0f;
        c.z_near = 0.05f + rng.unit() * 0.45f;
        c.z_far = 100.0f + rng.unit() * 3900.0f;
    }
}

uint64_t Unproject::run() {
    uint64_t h = 0;
    for (const Camera& c : cameras_) {
        const vm::Mat4 vp = view_projection(c);
        const vm::Mat4 inv = vm::inverse(vp);
        for (const vm::Vec3& p : ndc) {
            const vm::Vec3 world = vm::transform_point(inv, p);
            const vm::Vec3 back = vm::transform_point(vp, world);
            h = hash_add(h, static_cast<uint64_t>(f32_bits(world.x)) | (static_cast<uint64_t>(f32_bits(world.y)) << 32));
            h = hash_add(h, static_cast<uint64_t>(f32_bits(world.z)) | (static_cast<uint64_t>(f32_bits(back.x)) << 32));
        }
    }
    return h;
}

} // namespace bench
