#include "decompose.hpp"

#include "hash.hpp"

namespace bench {

Decompose::Decompose() : items_(items_n) {
    Rng rng{0xdec0};
    for (uint32_t i = 0; i < items_n; i++) {
        Item& it = items_[i];
        it.rotation = vm::random_quat(rng);
        it.scale = vm::random_vec3(rng, 0.25f, 4.0f);
        if (i % 4 == 0) it.scale.x = -it.scale.x;
        it.point = vm::random_vec3(rng, -10.0f, 10.0f);
    }
}

uint64_t Decompose::run() {
    uint64_t h = 0;
    for (const Item& it : items_) {
        const vm::Basis basis = vm::basis_from_rotation_scale(it.rotation, it.scale);
        const vm::Vec3 scale = vm::basis_scale(basis);
        const vm::Quat rotation = vm::basis_rotation(basis);
        const vm::Basis inv = vm::inverse(basis);
        const vm::Vec3 rotated = vm::rotate(rotation, it.point);
        const vm::Vec3 round_trip = inv * (basis * it.point);
        h = hash_add(h, static_cast<uint64_t>(f32_bits(scale.x)) | (static_cast<uint64_t>(f32_bits(scale.z)) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(rotation.x)) | (static_cast<uint64_t>(f32_bits(rotation.w)) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(rotated.y)) | (static_cast<uint64_t>(f32_bits(round_trip.z)) << 32));
    }
    return h;
}

} // namespace bench
