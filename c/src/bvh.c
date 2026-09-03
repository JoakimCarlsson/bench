#include "bvh.h"

#include <math.h>
#include <stdlib.h>

#include "alloc.h"
#include "hash.h"

enum { BOXES = 16384, RAYS = 20000, LEAF = 4, MAX_NODES = 2 * BOXES, STACK = 64 };

typedef struct { float min[3], max[3]; } Box;
typedef struct { float min[3], max[3]; uint32_t first, count; } Node;
typedef struct { Box* boxes; Node* nodes; uint32_t* order; uint32_t node_count; } Bvh;

static void setup(void* state) {
    Bvh* s = state;
    s->boxes = xalloc(BOXES * sizeof(Box));
    s->nodes = xalloc(MAX_NODES * sizeof(Node));
    s->order = xalloc(BOXES * sizeof(uint32_t));
    Rng rng = { 0xb4 };
    for (int i = 0; i < BOXES; i++) {
        Box* b = &s->boxes[i];
        for (int k = 0; k < 3; k++) {
            b->min[k] = rng_unit(&rng) * 200.0f;
            b->max[k] = b->min[k] + 0.5f + rng_unit(&rng) * 3.0f;
        }
    }
}

static inline float centroid(const Box* b, int axis) {
    return (b->min[axis] + b->max[axis]) * 0.5f;
}

static void build(Bvh* s, uint32_t node, uint32_t lo, uint32_t hi) {
    Node* n = &s->nodes[node];
    float cmin[3], cmax[3];
    for (int k = 0; k < 3; k++) { n->min[k] = 1e30f; n->max[k] = -1e30f; cmin[k] = 1e30f; cmax[k] = -1e30f; }
    for (uint32_t i = lo; i < hi; i++) {
        const Box* b = &s->boxes[s->order[i]];
        for (int k = 0; k < 3; k++) {
            if (b->min[k] < n->min[k]) n->min[k] = b->min[k];
            if (b->max[k] > n->max[k]) n->max[k] = b->max[k];
            float c = centroid(b, k);
            if (c < cmin[k]) cmin[k] = c;
            if (c > cmax[k]) cmax[k] = c;
        }
    }
    if (hi - lo <= LEAF) { n->first = lo; n->count = hi - lo; return; }

    int axis = 0;
    if (cmax[1] - cmin[1] > cmax[axis] - cmin[axis]) axis = 1;
    if (cmax[2] - cmin[2] > cmax[axis] - cmin[axis]) axis = 2;
    float split = (cmin[axis] + cmax[axis]) * 0.5f;
    uint32_t i = lo, j = hi;
    while (i < j) {
        if (centroid(&s->boxes[s->order[i]], axis) < split) {
            i++;
        } else {
            j--;
            uint32_t t = s->order[i]; s->order[i] = s->order[j]; s->order[j] = t;
        }
    }
    uint32_t mid = i;
    if (mid == lo || mid == hi) mid = lo + (hi - lo) / 2;

    n->first = s->node_count;
    n->count = 0;
    s->node_count += 2;
    build(s, n->first, lo, mid);
    build(s, n->first + 1, mid, hi);
}

static inline int slab(const float* min, const float* max, const float* o, const float* inv) {
    float tmin = 0.0f, tmax = 1000.0f;
    for (int k = 0; k < 3; k++) {
        float t1 = (min[k] - o[k]) * inv[k], t2 = (max[k] - o[k]) * inv[k];
        float lo = t1 < t2 ? t1 : t2, hi = t1 < t2 ? t2 : t1;
        if (lo > tmin) tmin = lo;
        if (hi < tmax) tmax = hi;
    }
    return tmax >= tmin;
}

static uint32_t trace(const Bvh* s, const float* o, const float* inv) {
    uint32_t stack[STACK];
    uint32_t sp = 0, hits = 0;
    stack[sp++] = 0;
    while (sp > 0) {
        const Node* n = &s->nodes[stack[--sp]];
        if (!slab(n->min, n->max, o, inv)) continue;
        if (n->count == 0) {
            stack[sp++] = n->first + 1;
            stack[sp++] = n->first;
            continue;
        }
        for (uint32_t i = 0; i < n->count; i++) {
            const Box* b = &s->boxes[s->order[n->first + i]];
            hits += slab(b->min, b->max, o, inv);
        }
    }
    return hits;
}

static uint64_t run(void* state) {
    Bvh* s = state;
    for (uint32_t i = 0; i < BOXES; i++) s->order[i] = i;
    s->node_count = 1;
    build(s, 0, 0, BOXES);

    Rng rng = { 0x7ace };
    uint64_t h = 0, total = 0;
    for (int r = 0; r < RAYS; r++) {
        float o[3], d[3], inv[3];
        for (int k = 0; k < 3; k++) o[k] = rng_unit(&rng) * 200.0f;
        for (int k = 0; k < 3; k++) d[k] = rng_unit(&rng) * 2.0f - 1.0f;
        float len = sqrtf(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if (len < 1e-3f) { d[0] = 1.0f; d[1] = 0.0f; d[2] = 0.0f; len = 1.0f; }
        for (int k = 0; k < 3; k++) {
            d[k] /= len;
            if (fabsf(d[k]) < 1e-6f) d[k] = 1e-6f;
            inv[k] = 1.0f / d[k];
        }
        uint32_t hits = trace(s, o, inv);
        h = hash_add(h, hits);
        total += hits;
    }
    return hash_add(hash_add(h, total), s->node_count);
}

static void teardown(void* state) {
    Bvh* s = state;
    free(s->boxes);
    free(s->nodes);
    free(s->order);
}

const Case bvh_case = { "bvh", sizeof(Bvh), setup, run, teardown };
