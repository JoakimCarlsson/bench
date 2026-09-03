#include "solve.hpp"

#include <algorithm>
#include <cmath>

#include "hash.hpp"

namespace bench {

Solve::Solve() : bodies_(bodies_n), initial_(bodies_n), contacts_(contacts_n) {
    Rng rng{0xb0d1e5};
    for (int i = 0; i < bodies_n; i++) {
        Body& b = initial_[static_cast<size_t>(i)];
        b.px = rng.unit() * 64.0f;
        b.py = rng.unit() * 64.0f;
        b.pz = rng.unit() * 64.0f;
        b.vx = rng.unit() * 2.0f - 1.0f;
        b.vy = rng.unit() * 2.0f - 1.0f;
        b.vz = rng.unit() * 2.0f - 1.0f;
        b.inv_mass = (i % 8 == 0) ? 0.0f : 1.0f / (0.5f + rng.unit() * 2.0f);
    }
    for (Contact& c : contacts_) {
        c.a = static_cast<uint32_t>(rng.next() % bodies_n);
        c.b = static_cast<uint32_t>(rng.next() % bodies_n);
        if (c.b == c.a) c.b = (c.a + 1) % bodies_n;
        float nx = rng.unit() * 2.0f - 1.0f;
        float ny = rng.unit() * 2.0f - 1.0f;
        float nz = rng.unit() * 2.0f - 1.0f;
        float len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len < 1e-3f) { nx = 0.0f; ny = 1.0f; nz = 0.0f; len = 1.0f; }
        c.nx = nx / len;
        c.ny = ny / len;
        c.nz = nz / len;
        c.depth = rng.unit() * 0.05f;
        c.impulse = 0.0f;
    }
}

void Solve::integrate_gravity(float h) {
    for (Body& b : bodies_) {
        if (b.inv_mass > 0.0f) b.vy += -9.81f * h;
    }
}

void Solve::solve_contacts(float h) {
    for (Contact& c : contacts_) {
        Body& a = bodies_[c.a];
        Body& b = bodies_[c.b];
        float k = a.inv_mass + b.inv_mass;
        if (k == 0.0f) continue;
        float rvx = b.vx - a.vx, rvy = b.vy - a.vy, rvz = b.vz - a.vz;
        float vn = rvx * c.nx + rvy * c.ny + rvz * c.nz;
        float bias = c.depth * 0.2f / h;
        float lambda = (-vn + bias) / k;
        float acc = c.impulse + lambda;
        if (acc < 0.0f) acc = 0.0f;
        lambda = acc - c.impulse;
        c.impulse = acc;
        a.vx -= c.nx * lambda * a.inv_mass;
        a.vy -= c.ny * lambda * a.inv_mass;
        a.vz -= c.nz * lambda * a.inv_mass;
        b.vx += c.nx * lambda * b.inv_mass;
        b.vy += c.ny * lambda * b.inv_mass;
        b.vz += c.nz * lambda * b.inv_mass;
    }
}

void Solve::integrate_positions(float h) {
    for (Body& b : bodies_) {
        b.px += b.vx * h;
        b.py += b.vy * h;
        b.pz += b.vz * h;
    }
}

uint64_t Solve::checksum() const {
    uint64_t h = 0;
    for (const Body& b : bodies_) {
        h = hash_add(h, static_cast<uint64_t>(f32_bits(b.px)) | (static_cast<uint64_t>(f32_bits(b.vx)) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(b.py)) | (static_cast<uint64_t>(f32_bits(b.vy)) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(b.pz)) | (static_cast<uint64_t>(f32_bits(b.vz)) << 32));
    }
    return h;
}

uint64_t Solve::run() {
    std::copy(initial_.begin(), initial_.end(), bodies_.begin());
    for (Contact& c : contacts_) c.impulse = 0.0f;
    const float h = (1.0f / 60.0f) / static_cast<float>(substeps);
    for (int frame = 0; frame < frames; frame++) {
        for (int sub = 0; sub < substeps; sub++) {
            integrate_gravity(h);
            for (int it = 0; it < iters; it++) solve_contacts(h);
            integrate_positions(h);
        }
    }
    return checksum();
}

} // namespace bench
