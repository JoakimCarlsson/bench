#include "sort.h"

#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "hash.h"

enum { KEYS = 1 << 19 };

typedef struct { uint64_t* keys; uint64_t* initial; } Sort;

static int cmp_u64(const void* a, const void* b) {
    uint64_t x = *(const uint64_t*)a, y = *(const uint64_t*)b;
    return x < y ? -1 : x > y;
}

static void setup(void* state) {
    Sort* s = state;
    s->keys = xalloc(KEYS * sizeof(uint64_t));
    s->initial = xalloc(KEYS * sizeof(uint64_t));
    Rng rng = { 0x5027 };
    for (int i = 0; i < KEYS; i++) s->initial[i] = rng_next(&rng);
}

static uint64_t run(void* state) {
    Sort* s = state;
    memcpy(s->keys, s->initial, KEYS * sizeof(uint64_t));
    qsort(s->keys, KEYS, sizeof(uint64_t), cmp_u64);
    uint64_t h = 0;
    for (int i = 0; i < KEYS; i += 977) h = hash_add(h, s->keys[i]);
    return hash_add(h, s->keys[KEYS - 1]);
}

static void teardown(void* state) {
    Sort* s = state;
    free(s->keys);
    free(s->initial);
}

const Case sort_case = { "sort", sizeof(Sort), setup, run, teardown };
