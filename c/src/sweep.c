#include "sweep.h"

#include <stdlib.h>

#include "alloc.h"
#include "hash.h"

enum { BOXES = 16384 };

typedef struct { float min[3], max[3]; } Box;
typedef struct { Box* boxes; uint32_t* order; } Sweep;

static const Box* sort_boxes;

static int by_min_x(const void* pa, const void* pb) {
    uint32_t a = *(const uint32_t*)pa, b = *(const uint32_t*)pb;
    float xa = sort_boxes[a].min[0], xb = sort_boxes[b].min[0];
    if (xa != xb) return xa < xb ? -1 : 1;
    return a < b ? -1 : a > b;
}

static void setup(void* state) {
    Sweep* s = state;
    s->boxes = xalloc(BOXES * sizeof(Box));
    s->order = xalloc(BOXES * sizeof(uint32_t));
    Rng rng = { 0xb0c5 };
    for (int i = 0; i < BOXES; i++) {
        Box* b = &s->boxes[i];
        for (int k = 0; k < 3; k++) {
            b->min[k] = rng_unit(&rng) * 200.0f;
            b->max[k] = b->min[k] + 0.5f + rng_unit(&rng) * 3.0f;
        }
    }
}

static uint64_t run(void* state) {
    Sweep* s = state;
    for (uint32_t i = 0; i < BOXES; i++) s->order[i] = i;
    sort_boxes = s->boxes;
    qsort(s->order, BOXES, sizeof(uint32_t), by_min_x);

    uint64_t h = 0, pairs = 0;
    for (uint32_t i = 0; i < BOXES; i++) {
        const Box* a = &s->boxes[s->order[i]];
        for (uint32_t j = i + 1; j < BOXES; j++) {
            const Box* b = &s->boxes[s->order[j]];
            if (b->min[0] > a->max[0]) break;
            if (b->min[1] > a->max[1] || a->min[1] > b->max[1]) continue;
            if (b->min[2] > a->max[2] || a->min[2] > b->max[2]) continue;
            h = hash_add(h, ((uint64_t)s->order[i] << 32) | s->order[j]);
            pairs++;
        }
    }
    return hash_add(h, pairs);
}

static void teardown(void* state) {
    Sweep* s = state;
    free(s->boxes);
    free(s->order);
}

const Case sweep_case = { "sweep", sizeof(Sweep), setup, run, teardown };
