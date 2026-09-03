#include "dda.hpp"

#include <cmath>

#include "hash.hpp"

namespace bench {

Dda::Dda() : words_(N * N * N / 64) {
    for (uint32_t i = 0; i < N * N * N; i++) {
        if ((mix64(i) & 7) == 0) words_[i >> 6] |= 1ull << (i & 63);
    }
}

bool Dda::solid(int x, int y, int z) const {
    uint32_t i = (static_cast<uint32_t>(x) * N + static_cast<uint32_t>(y)) * N + static_cast<uint32_t>(z);
    return (words_[i >> 6] >> (i & 63)) & 1;
}

uint64_t Dda::run() {
    Rng rng{0x1234};
    uint64_t h = 0;
    for (int r = 0; r < rays; r++) {
        float ox = rng.unit() * N, oy = rng.unit() * N, oz = rng.unit() * N;
        float dx = rng.unit() * 2.0f - 1.0f;
        float dy = rng.unit() * 2.0f - 1.0f;
        float dz = rng.unit() * 2.0f - 1.0f;
        float len = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-3f) { dx = 1.0f; dy = 0.0f; dz = 0.0f; len = 1.0f; }
        dx /= len; dy /= len; dz /= len;
        if (std::fabs(dx) < 1e-6f) dx = 1e-6f;
        if (std::fabs(dy) < 1e-6f) dy = 1e-6f;
        if (std::fabs(dz) < 1e-6f) dz = 1e-6f;

        int ix = static_cast<int>(ox), iy = static_cast<int>(oy), iz = static_cast<int>(oz);
        int sx = dx > 0 ? 1 : -1, sy = dy > 0 ? 1 : -1, sz = dz > 0 ? 1 : -1;
        float invx = 1.0f / dx, invy = 1.0f / dy, invz = 1.0f / dz;
        float tdx = std::fabs(invx), tdy = std::fabs(invy), tdz = std::fabs(invz);
        float tx = dx > 0 ? (static_cast<float>(ix + 1) - ox) * invx : (static_cast<float>(ix) - ox) * invx;
        float ty = dy > 0 ? (static_cast<float>(iy + 1) - oy) * invy : (static_cast<float>(iy) - oy) * invy;
        float tz = dz > 0 ? (static_cast<float>(iz + 1) - oz) * invz : (static_cast<float>(iz) - oz) * invz;

        uint32_t steps = 0;
        for (;;) {
            if (solid(ix, iy, iz)) {
                uint64_t cell = (static_cast<uint64_t>(ix) * N + static_cast<uint64_t>(iy)) * N + static_cast<uint64_t>(iz);
                h = hash_add(h, cell | (static_cast<uint64_t>(steps) << 32));
                break;
            }
            if (tx < ty) {
                if (tx < tz) { ix += sx; tx += tdx; } else { iz += sz; tz += tdz; }
            } else {
                if (ty < tz) { iy += sy; ty += tdy; } else { iz += sz; tz += tdz; }
            }
            steps++;
            if (static_cast<unsigned>(ix) >= N || static_cast<unsigned>(iy) >= N || static_cast<unsigned>(iz) >= N) {
                h = hash_add(h, 0xffffffffull ^ steps);
                break;
            }
        }
    }
    return h;
}

} // namespace bench
