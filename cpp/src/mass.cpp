#include "mass.hpp"

#include "hash.hpp"

namespace bench {

namespace {

constexpr float voxel_size = 0.1f;
constexpr float density[16] = {
    0.0f, 2400.0f, 700.0f, 7800.0f, 1600.0f, 2500.0f, 900.0f, 1200.0f,
    1800.0f, 8900.0f, 500.0f, 1500.0f, 2700.0f, 300.0f, 1100.0f, 2000.0f,
};

} // namespace

Mass::Mass() : mat_(static_cast<size_t>(bodies_n) * voxels) {
    for (uint32_t i = 0; i < static_cast<uint32_t>(bodies_n * voxels); i++) {
        uint64_t r = mix64(i ^ 0x3a55);
        mat_[i] = (r & 3) == 0 ? 0 : static_cast<uint8_t>((r >> 2) & 15);
    }
}

Mass::Props Mass::properties(const uint8_t* mat) {
    float m = 0.0f, cx = 0.0f, cy = 0.0f, cz = 0.0f;
    float ixx = 0.0f, iyy = 0.0f, izz = 0.0f, ixy = 0.0f, ixz = 0.0f, iyz = 0.0f;
    const float cube = voxel_size * voxel_size / 6.0f;
    for (int z = 0; z < dim; z++) {
        for (int y = 0; y < dim; y++) {
            for (int x = 0; x < dim; x++) {
                uint8_t id = mat[(z * dim + y) * dim + x];
                if (id == 0) continue;
                float dm = density[id] * (voxel_size * voxel_size * voxel_size);
                float px = (static_cast<float>(x) + 0.5f) * voxel_size;
                float py = (static_cast<float>(y) + 0.5f) * voxel_size;
                float pz = (static_cast<float>(z) + 0.5f) * voxel_size;
                m += dm;
                cx += dm * px; cy += dm * py; cz += dm * pz;
                ixx += dm * (py * py + pz * pz + cube);
                iyy += dm * (px * px + pz * pz + cube);
                izz += dm * (px * px + py * py + cube);
                ixy -= dm * px * py;
                ixz -= dm * px * pz;
                iyz -= dm * py * pz;
            }
        }
    }
    Props p;
    p.mass = m;
    float inv = m > 0.0f ? 1.0f / m : 0.0f;
    p.com[0] = cx * inv; p.com[1] = cy * inv; p.com[2] = cz * inv;
    float ox = p.com[0], oy = p.com[1], oz = p.com[2];
    p.inertia[0] = ixx - m * (oy * oy + oz * oz);
    p.inertia[1] = iyy - m * (ox * ox + oz * oz);
    p.inertia[2] = izz - m * (ox * ox + oy * oy);
    p.inertia[3] = ixy + m * ox * oy;
    p.inertia[4] = ixz + m * ox * oz;
    p.inertia[5] = iyz + m * oy * oz;
    return p;
}

uint64_t Mass::run() {
    uint64_t h = 0;
    for (int b = 0; b < bodies_n; b++) {
        Props p = properties(mat_.data() + static_cast<size_t>(b) * voxels);
        h = hash_add(h, static_cast<uint64_t>(f32_bits(p.mass)) | (static_cast<uint64_t>(f32_bits(p.com[0])) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(p.com[1])) | (static_cast<uint64_t>(f32_bits(p.com[2])) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(p.inertia[0])) | (static_cast<uint64_t>(f32_bits(p.inertia[1])) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(p.inertia[2])) | (static_cast<uint64_t>(f32_bits(p.inertia[3])) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(p.inertia[4])) | (static_cast<uint64_t>(f32_bits(p.inertia[5])) << 32));
    }
    return h;
}

} // namespace bench
