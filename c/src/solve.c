#include "solve.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "hash.h"

enum { BODIES = 4096, CONTACTS = 16384, SUBSTEPS = 8, ITERS = 4, FRAMES = 8 };

typedef struct { float px, py, pz, vx, vy, vz, inv_mass; } Body;
typedef struct { uint32_t a, b; float nx, ny, nz, depth, impulse; } Contact;
typedef struct { Body* bodies; Body* initial; Contact* contacts; } Solve;

static void setup(void* state) {
    Solve* s = state;
    s->bodies = xalloc(BODIES * sizeof(Body));
    s->initial = xalloc(BODIES * sizeof(Body));
    s->contacts = xalloc(CONTACTS * sizeof(Contact));
    Rng rng = { 0xb0d1e5 };
    for (int i = 0; i < BODIES; i++) {
        Body* b = &s->initial[i];
        b->px = rng_unit(&rng) * 64.0f;
        b->py = rng_unit(&rng) * 64.0f;
        b->pz = rng_unit(&rng) * 64.0f;
        b->vx = rng_unit(&rng) * 2.0f - 1.0f;
        b->vy = rng_unit(&rng) * 2.0f - 1.0f;
        b->vz = rng_unit(&rng) * 2.0f - 1.0f;
        b->inv_mass = (i % 8 == 0) ? 0.0f : 1.0f / (0.5f + rng_unit(&rng) * 2.0f);
    }
    for (int i = 0; i < CONTACTS; i++) {
        Contact* c = &s->contacts[i];
        c->a = (uint32_t)(rng_next(&rng) % BODIES);
        c->b = (uint32_t)(rng_next(&rng) % BODIES);
        if (c->b == c->a) c->b = (c->a + 1) % BODIES;
        float nx = rng_unit(&rng) * 2.0f - 1.0f;
        float ny = rng_unit(&rng) * 2.0f - 1.0f;
        float nz = rng_unit(&rng) * 2.0f - 1.0f;
        float len = sqrtf(nx * nx + ny * ny + nz * nz);
        if (len < 1e-3f) { nx = 0.0f; ny = 1.0f; nz = 0.0f; len = 1.0f; }
        c->nx = nx / len;
        c->ny = ny / len;
        c->nz = nz / len;
        c->depth = rng_unit(&rng) * 0.05f;
        c->impulse = 0.0f;
    }
}

static void integrate_gravity(Body* bodies, float h) {
    for (int i = 0; i < BODIES; i++) {
        if (bodies[i].inv_mass > 0.0f) bodies[i].vy += -9.81f * h;
    }
}

static void solve_contacts(Body* bodies, Contact* contacts, float h) {
    for (int i = 0; i < CONTACTS; i++) {
        Contact* c = &contacts[i];
        Body* a = &bodies[c->a];
        Body* b = &bodies[c->b];
        float k = a->inv_mass + b->inv_mass;
        if (k == 0.0f) continue;
        float rvx = b->vx - a->vx, rvy = b->vy - a->vy, rvz = b->vz - a->vz;
        float vn = rvx * c->nx + rvy * c->ny + rvz * c->nz;
        float bias = c->depth * 0.2f / h;
        float lambda = (-vn + bias) / k;
        float acc = c->impulse + lambda;
        if (acc < 0.0f) acc = 0.0f;
        lambda = acc - c->impulse;
        c->impulse = acc;
        a->vx -= c->nx * lambda * a->inv_mass;
        a->vy -= c->ny * lambda * a->inv_mass;
        a->vz -= c->nz * lambda * a->inv_mass;
        b->vx += c->nx * lambda * b->inv_mass;
        b->vy += c->ny * lambda * b->inv_mass;
        b->vz += c->nz * lambda * b->inv_mass;
    }
}

static void integrate_positions(Body* bodies, float h) {
    for (int i = 0; i < BODIES; i++) {
        Body* b = &bodies[i];
        b->px += b->vx * h;
        b->py += b->vy * h;
        b->pz += b->vz * h;
    }
}

static uint64_t checksum(const Body* bodies) {
    uint64_t h = 0;
    for (int i = 0; i < BODIES; i++) {
        const Body* b = &bodies[i];
        h = hash_add(h, (uint64_t)f32_bits(b->px) | ((uint64_t)f32_bits(b->vx) << 32));
        h = hash_add(h, (uint64_t)f32_bits(b->py) | ((uint64_t)f32_bits(b->vy) << 32));
        h = hash_add(h, (uint64_t)f32_bits(b->pz) | ((uint64_t)f32_bits(b->vz) << 32));
    }
    return h;
}

static uint64_t run(void* state) {
    Solve* s = state;
    memcpy(s->bodies, s->initial, BODIES * sizeof(Body));
    for (int i = 0; i < CONTACTS; i++) s->contacts[i].impulse = 0.0f;
    const float h = (1.0f / 60.0f) / (float)SUBSTEPS;
    for (int frame = 0; frame < FRAMES; frame++) {
        for (int sub = 0; sub < SUBSTEPS; sub++) {
            integrate_gravity(s->bodies, h);
            for (int it = 0; it < ITERS; it++) solve_contacts(s->bodies, s->contacts, h);
            integrate_positions(s->bodies, h);
        }
    }
    return checksum(s->bodies);
}

static void teardown(void* state) {
    Solve* s = state;
    free(s->bodies);
    free(s->initial);
    free(s->contacts);
}

const Case solve_case = { "solve", sizeof(Solve), setup, run, teardown };
