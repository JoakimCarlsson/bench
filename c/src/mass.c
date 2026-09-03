#include "mass.h"

#include <stdlib.h>

#include "alloc.h"
#include "hash.h"

enum { BODIES = 128, DIM = 32, VOXELS = DIM * DIM * DIM };

static const float voxel_size = 0.1f;
static const float density[16] = {
    0.0f, 2400.0f, 700.0f, 7800.0f, 1600.0f, 2500.0f, 900.0f, 1200.0f,
    1800.0f, 8900.0f, 500.0f, 1500.0f, 2700.0f, 300.0f, 1100.0f, 2000.0f,
};

typedef struct { float mass, com[3], inertia[6]; } Props;
typedef struct { uint8_t* mat; } Mass;

static void setup(void* state) {
    Mass* s = state;
    s->mat = xalloc((size_t)BODIES * VOXELS);
    for (uint32_t i = 0; i < BODIES * VOXELS; i++) {
        uint64_t r = mix64(i ^ 0x3a55);
        s->mat[i] = (r & 3) == 0 ? 0 : (uint8_t)((r >> 2) & 15);
    }
}

static Props properties(const uint8_t* mat) {
    float m = 0.0f, cx = 0.0f, cy = 0.0f, cz = 0.0f;
    float ixx = 0.0f, iyy = 0.0f, izz = 0.0f, ixy = 0.0f, ixz = 0.0f, iyz = 0.0f;
    const float cube = voxel_size * voxel_size / 6.0f;
    for (int z = 0; z < DIM; z++) {
        for (int y = 0; y < DIM; y++) {
            for (int x = 0; x < DIM; x++) {
                uint8_t id = mat[(z * DIM + y) * DIM + x];
                if (id == 0) continue;
                float dm = density[id] * (voxel_size * voxel_size * voxel_size);
                float px = ((float)x + 0.5f) * voxel_size;
                float py = ((float)y + 0.5f) * voxel_size;
                float pz = ((float)z + 0.5f) * voxel_size;
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

static uint64_t run(void* state) {
    Mass* s = state;
    uint64_t h = 0;
    for (int b = 0; b < BODIES; b++) {
        Props p = properties(s->mat + (size_t)b * VOXELS);
        h = hash_add(h, (uint64_t)f32_bits(p.mass) | ((uint64_t)f32_bits(p.com[0]) << 32));
        h = hash_add(h, (uint64_t)f32_bits(p.com[1]) | ((uint64_t)f32_bits(p.com[2]) << 32));
        h = hash_add(h, (uint64_t)f32_bits(p.inertia[0]) | ((uint64_t)f32_bits(p.inertia[1]) << 32));
        h = hash_add(h, (uint64_t)f32_bits(p.inertia[2]) | ((uint64_t)f32_bits(p.inertia[3]) << 32));
        h = hash_add(h, (uint64_t)f32_bits(p.inertia[4]) | ((uint64_t)f32_bits(p.inertia[5]) << 32));
    }
    return h;
}

static void teardown(void* state) {
    Mass* s = state;
    free(s->mat);
}

const Case mass_case = { "mass", sizeof(Mass), setup, run, teardown };
