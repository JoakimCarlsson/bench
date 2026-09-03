#include "dda.h"

#include <math.h>
#include <stdlib.h>

#include "alloc.h"
#include "hash.h"

enum { N = 128, RAYS = 100000 };

typedef struct { uint64_t* words; } Dda;

static void setup(void* state) {
    Dda* d = state;
    const uint32_t cells = N * N * N;
    d->words = xalloc(cells / 64 * sizeof(uint64_t));
    for (uint32_t i = 0; i < cells; i++) {
        if ((mix64(i) & 7) == 0) d->words[i >> 6] |= 1ull << (i & 63);
    }
}

static inline int solid(const Dda* d, int x, int y, int z) {
    uint32_t i = ((uint32_t)x * N + (uint32_t)y) * N + (uint32_t)z;
    return (d->words[i >> 6] >> (i & 63)) & 1;
}

static uint64_t run(void* state) {
    Dda* d = state;
    Rng rng = { 0x1234 };
    uint64_t h = 0;
    for (int r = 0; r < RAYS; r++) {
        float ox = rng_unit(&rng) * N, oy = rng_unit(&rng) * N, oz = rng_unit(&rng) * N;
        float dx = rng_unit(&rng) * 2.0f - 1.0f;
        float dy = rng_unit(&rng) * 2.0f - 1.0f;
        float dz = rng_unit(&rng) * 2.0f - 1.0f;
        float len = sqrtf(dx * dx + dy * dy + dz * dz);
        if (len < 1e-3f) { dx = 1.0f; dy = 0.0f; dz = 0.0f; len = 1.0f; }
        dx /= len; dy /= len; dz /= len;
        if (fabsf(dx) < 1e-6f) dx = 1e-6f;
        if (fabsf(dy) < 1e-6f) dy = 1e-6f;
        if (fabsf(dz) < 1e-6f) dz = 1e-6f;

        int ix = (int)ox, iy = (int)oy, iz = (int)oz;
        int sx = dx > 0 ? 1 : -1, sy = dy > 0 ? 1 : -1, sz = dz > 0 ? 1 : -1;
        float invx = 1.0f / dx, invy = 1.0f / dy, invz = 1.0f / dz;
        float tdx = fabsf(invx), tdy = fabsf(invy), tdz = fabsf(invz);
        float tx = dx > 0 ? ((float)(ix + 1) - ox) * invx : ((float)ix - ox) * invx;
        float ty = dy > 0 ? ((float)(iy + 1) - oy) * invy : ((float)iy - oy) * invy;
        float tz = dz > 0 ? ((float)(iz + 1) - oz) * invz : ((float)iz - oz) * invz;

        uint32_t steps = 0;
        for (;;) {
            if (solid(d, ix, iy, iz)) {
                uint64_t cell = ((uint64_t)ix * N + (uint64_t)iy) * N + (uint64_t)iz;
                h = hash_add(h, cell | ((uint64_t)steps << 32));
                break;
            }
            if (tx < ty) {
                if (tx < tz) { ix += sx; tx += tdx; } else { iz += sz; tz += tdz; }
            } else {
                if (ty < tz) { iy += sy; ty += tdy; } else { iz += sz; tz += tdz; }
            }
            steps++;
            if ((unsigned)ix >= N || (unsigned)iy >= N || (unsigned)iz >= N) {
                h = hash_add(h, 0xffffffffull ^ steps);
                break;
            }
        }
    }
    return h;
}

static void teardown(void* state) {
    Dda* d = state;
    free(d->words);
}

const Case dda_case = { "dda", sizeof(Dda), setup, run, teardown };
