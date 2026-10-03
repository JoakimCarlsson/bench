#include "decompose.h"

#include <stdlib.h>

#include "alloc.h"
#include "hash.h"
#include "vecmath.h"

enum { ITEMS = 262144 };

typedef struct { Quat rotation; Vec3 scale; Vec3 point; } Item;
typedef struct { Item* items; } Decompose;

static void setup(void* state) {
    Decompose* s = state;
    s->items = xalloc(ITEMS * sizeof(Item));
    Rng rng = { 0xdec0 };
    for (int i = 0; i < ITEMS; i++) {
        Item* it = &s->items[i];
        it->rotation = quat_random(&rng);
        it->scale = v3_random(&rng, 0.25f, 4.0f);
        if (i % 4 == 0) it->scale.x = -it->scale.x;
        it->point = v3_random(&rng, -10.0f, 10.0f);
    }
}

static uint64_t run(void* state) {
    Decompose* s = state;
    uint64_t h = 0;
    for (int i = 0; i < ITEMS; i++) {
        const Item* it = &s->items[i];
        Basis basis = basis_from_rotation_scale(it->rotation, it->scale);
        Vec3 scale = basis_scale(&basis);
        Quat rotation = basis_rotation(&basis);
        Basis inv = basis_inverse(&basis);
        Vec3 rotated = quat_rotate(rotation, it->point);
        Vec3 round_trip = basis_apply(&inv, basis_apply(&basis, it->point));
        h = hash_add(h, (uint64_t)f32_bits(scale.x) | ((uint64_t)f32_bits(scale.z) << 32));
        h = hash_add(h, (uint64_t)f32_bits(rotation.x) | ((uint64_t)f32_bits(rotation.w) << 32));
        h = hash_add(h, (uint64_t)f32_bits(rotated.y) | ((uint64_t)f32_bits(round_trip.z) << 32));
    }
    return h;
}

static void teardown(void* state) {
    Decompose* s = state;
    free(s->items);
}

const Case decompose_case = { "decompose", sizeof(Decompose), setup, run, teardown };
