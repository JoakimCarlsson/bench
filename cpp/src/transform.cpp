#include "transform.hpp"

#include <algorithm>

#include "hash.hpp"

namespace bench {

Transform::Transform()
    : nodes_(nodes_n), initial_(nodes_n), rotations_(nodes_n), world_(nodes_n), mvp_(nodes_n) {
    Rng rng{0x7f0e};
    for (uint32_t i = 0; i < nodes_n; i++) {
        Node& n = nodes_[i];
        n.position = vm::random_vec3(rng, -2.0f, 2.0f);
        n.scale = vm::random_vec3(rng, 0.9f, 1.1f);
        n.spin = vm::random_vec3(rng, -0.05f, 0.05f);
        n.parent = i == 0 ? 0 : (i - 1) / 4;
        initial_[i] = vm::random_quat(rng);
    }
    const vm::Transform eye = vm::looking_at({40.0f, 30.0f, 40.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f});
    const vm::Mat4 view = vm::to_mat4(vm::inverse_orthonormal(eye));
    view_projection_ = vm::perspective(0.75f, 16.0f / 9.0f, 0.05f, 4000.0f) * view;
}

void Transform::frame() {
    for (uint32_t i = 0; i < nodes_n; i++) {
        const Node& n = nodes_[i];
        rotations_[i] = vm::integrate_rotation(rotations_[i], n.spin);
        const vm::Transform local{vm::basis_from_rotation_scale(rotations_[i], n.scale), n.position};
        world_[i] = i == 0 ? local : world_[n.parent] * local;
        mvp_[i] = view_projection_ * vm::to_mat4(world_[i]);
    }
}

uint64_t Transform::run() {
    std::copy(initial_.begin(), initial_.end(), rotations_.begin());
    for (int f = 0; f < frames; f++) frame();
    uint64_t h = 0;
    for (const vm::Mat4& mvp : mvp_) {
        const auto& m = mvp.m;
        h = hash_add(h, static_cast<uint64_t>(f32_bits(m[0])) | (static_cast<uint64_t>(f32_bits(m[5])) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(m[12])) | (static_cast<uint64_t>(f32_bits(m[13])) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(m[14])) | (static_cast<uint64_t>(f32_bits(m[15])) << 32));
    }
    return h;
}

} // namespace bench
