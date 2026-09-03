#include "integrate.hpp"

#include <algorithm>
#include <cmath>

#include "hash.hpp"

namespace bench {

Integrate::Integrate() : bodies_(bodies_n), initial_(bodies_n) {
    Rng rng{0x1a7e};
    for (Body& b : initial_) {
        b.px = rng.unit() * 100.0f; b.py = rng.unit() * 100.0f; b.pz = rng.unit() * 100.0f;
        b.vx = rng.unit() * 4.0f - 2.0f; b.vy = rng.unit() * 4.0f - 2.0f; b.vz = rng.unit() * 4.0f - 2.0f;
        b.wx = rng.unit() * 2.0f - 1.0f; b.wy = rng.unit() * 2.0f - 1.0f; b.wz = rng.unit() * 2.0f - 1.0f;
        b.qx = 0.0f; b.qy = 0.0f; b.qz = 0.0f; b.qw = 1.0f;
    }
}

void Integrate::step(float dt) {
    const float half = 0.5f * dt;
    for (Body& b : bodies_) {
        b.vy += -9.81f * dt;
        b.px += b.vx * dt;
        b.py += b.vy * dt;
        b.pz += b.vz * dt;
        b.wx *= 0.999f;
        b.wy *= 0.999f;
        b.wz *= 0.999f;
        float qx = b.qx + half * (b.wx * b.qw + b.wy * b.qz - b.wz * b.qy);
        float qy = b.qy + half * (b.wy * b.qw + b.wz * b.qx - b.wx * b.qz);
        float qz = b.qz + half * (b.wz * b.qw + b.wx * b.qy - b.wy * b.qx);
        float qw = b.qw + half * (-b.wx * b.qx - b.wy * b.qy - b.wz * b.qz);
        float inv = 1.0f / std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        b.qx = qx * inv;
        b.qy = qy * inv;
        b.qz = qz * inv;
        b.qw = qw * inv;
    }
}

uint64_t Integrate::run() {
    std::copy(initial_.begin(), initial_.end(), bodies_.begin());
    for (int i = 0; i < steps; i++) step(1.0f / 60.0f);
    uint64_t h = 0;
    for (const Body& b : bodies_) {
        h = hash_add(h, static_cast<uint64_t>(f32_bits(b.px)) | (static_cast<uint64_t>(f32_bits(b.py)) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(b.qx)) | (static_cast<uint64_t>(f32_bits(b.qw)) << 32));
    }
    return h;
}

} // namespace bench
