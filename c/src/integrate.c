#include "integrate.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "hash.h"

enum { BODIES = 65536, STEPS = 32 };

typedef struct { float px, py, pz, vx, vy, vz, wx, wy, wz, qx, qy, qz, qw; } Body;
typedef struct { Body* bodies; Body* initial; } Integrate;

static void setup(void* state) {
    Integrate* s = state;
    s->bodies = xalloc(BODIES * sizeof(Body));
    s->initial = xalloc(BODIES * sizeof(Body));
    Rng rng = { 0x1a7e };
    for (int i = 0; i < BODIES; i++) {
        Body* b = &s->initial[i];
        b->px = rng_unit(&rng) * 100.0f; b->py = rng_unit(&rng) * 100.0f; b->pz = rng_unit(&rng) * 100.0f;
        b->vx = rng_unit(&rng) * 4.0f - 2.0f; b->vy = rng_unit(&rng) * 4.0f - 2.0f; b->vz = rng_unit(&rng) * 4.0f - 2.0f;
        b->wx = rng_unit(&rng) * 2.0f - 1.0f; b->wy = rng_unit(&rng) * 2.0f - 1.0f; b->wz = rng_unit(&rng) * 2.0f - 1.0f;
        b->qx = 0.0f; b->qy = 0.0f; b->qz = 0.0f; b->qw = 1.0f;
    }
}

static void step(Body* bodies, float dt) {
    const float half = 0.5f * dt;
    for (int i = 0; i < BODIES; i++) {
        Body* b = &bodies[i];
        b->vy += -9.81f * dt;
        b->px += b->vx * dt;
        b->py += b->vy * dt;
        b->pz += b->vz * dt;
        b->wx *= 0.999f;
        b->wy *= 0.999f;
        b->wz *= 0.999f;
        float qx = b->qx + half * (b->wx * b->qw + b->wy * b->qz - b->wz * b->qy);
        float qy = b->qy + half * (b->wy * b->qw + b->wz * b->qx - b->wx * b->qz);
        float qz = b->qz + half * (b->wz * b->qw + b->wx * b->qy - b->wy * b->qx);
        float qw = b->qw + half * (-b->wx * b->qx - b->wy * b->qy - b->wz * b->qz);
        float inv = 1.0f / sqrtf(qx * qx + qy * qy + qz * qz + qw * qw);
        b->qx = qx * inv;
        b->qy = qy * inv;
        b->qz = qz * inv;
        b->qw = qw * inv;
    }
}

static uint64_t run(void* state) {
    Integrate* s = state;
    memcpy(s->bodies, s->initial, BODIES * sizeof(Body));
    for (int i = 0; i < STEPS; i++) step(s->bodies, 1.0f / 60.0f);
    uint64_t h = 0;
    for (int i = 0; i < BODIES; i++) {
        const Body* b = &s->bodies[i];
        h = hash_add(h, (uint64_t)f32_bits(b->px) | ((uint64_t)f32_bits(b->py) << 32));
        h = hash_add(h, (uint64_t)f32_bits(b->qx) | ((uint64_t)f32_bits(b->qw) << 32));
    }
    return h;
}

static void teardown(void* state) {
    Integrate* s = state;
    free(s->bodies);
    free(s->initial);
}

const Case integrate_case = { "integrate", sizeof(Integrate), setup, run, teardown };
