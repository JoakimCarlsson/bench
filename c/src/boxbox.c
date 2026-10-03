#include "boxbox.h"

#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "box_collision.h"
#include "hash.h"
#include "vecmath.h"

enum { PAIRS = 8192, FRAMES = 8 };

typedef struct { BoxPose a; BoxPose b; Vec3 velocity; } Pair;
typedef struct { Pair* pairs; Manifold* manifolds; } BoxBox;

/// Builds the pairs, a quarter of them stacked face to face.
static void setup(void* state) {
    BoxBox* s = state;
    s->pairs = xalloc(PAIRS * sizeof(Pair));
    s->manifolds = xalloc(PAIRS * sizeof(Manifold));
    Rng rng = { 0xb0b0 };
    for (int i = 0; i < PAIRS; i++) {
        Pair* p = &s->pairs[i];
        Quat rotation = quat_random(&rng);
        p->a.half_extents = v3_random(&rng, 0.25f, 1.5f);
        p->a.center = v3_random(&rng, 0.0f, 1000.0f);
        p->a.basis = basis_from_quat(rotation);
        p->b.half_extents = v3_random(&rng, 0.25f, 1.5f);
        if (i % 4 == 0) {
            Quat yaw = quat_normalize((Quat){ 0.0f, rng_unit(&rng) - 0.5f, 0.0f, 1.0f });
            p->b.basis = basis_from_quat(quat_mul(rotation, yaw));
            Vec3 slide = v3_random(&rng, -0.3f, 0.3f);
            float lift = p->a.half_extents.y + p->b.half_extents.y - 0.01f;
            p->b.center = v3_add(v3_add(p->a.center, v3_scale(p->a.basis.y, lift)),
                                 v3_add(v3_scale(p->a.basis.x, slide.x), v3_scale(p->a.basis.z, slide.z)));
        } else {
            p->b.basis = basis_from_quat(quat_random(&rng));
            float reach = (v3_length(p->a.half_extents) + v3_length(p->b.half_extents)) * 0.6f;
            p->b.center = v3_add(p->a.center, v3_scale(v3_random(&rng, -1.0f, 1.0f), reach));
        }
        p->velocity = v3_random(&rng, -0.02f, 0.02f);
    }
}

/// Collides every pair for each frame and hashes the manifolds.
static uint64_t run(void* state) {
    BoxBox* s = state;
    memset(s->manifolds, 0, PAIRS * sizeof(Manifold));
    CollisionTolerances tolerances = collision_tolerances_default();
    for (int f = 0; f < FRAMES; f++) {
        float t = (float)f;
        for (int i = 0; i < PAIRS; i++) {
            const Pair* p = &s->pairs[i];
            BoxPose b = p->b;
            b.center = v3_add(b.center, v3_scale(p->velocity, t));
            collide_boxes(&p->a, &b, &tolerances, &s->manifolds[i]);
        }
    }
    uint64_t h = 0;
    for (int i = 0; i < PAIRS; i++) {
        const Manifold* m = &s->manifolds[i];
        h = hash_add(h, (uint64_t)f32_bits(m->normal.x) | ((uint64_t)f32_bits(m->normal.z) << 32));
        h = hash_add(h, m->point_count);
        for (uint32_t k = 0; k < m->point_count; k++) {
            const ManifoldPoint* p = &m->points[k];
            h = hash_add(h, (uint64_t)f32_bits(p->point.y) | ((uint64_t)f32_bits(p->separation) << 32));
            h = hash_add(h, (uint64_t)p->feature_id | ((uint64_t)p->persisted << 32));
        }
    }
    return h;
}

/// Frees the pairs and manifolds.
static void teardown(void* state) {
    BoxBox* s = state;
    free(s->pairs);
    free(s->manifolds);
}

const Case boxbox_case = { "boxbox", sizeof(BoxBox), setup, run, teardown };
