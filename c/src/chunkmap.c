#include "chunkmap.h"

#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "hash.h"

enum { CAP = 1 << 19, INSERTS = 200000, LOOKUPS = 2000000 };

#define EMPTY UINT64_MAX

typedef struct { uint64_t* keys; uint32_t* values; uint64_t* coords; } ChunkMap;

static uint64_t pack(Rng* rng) {
    uint64_t r = rng_next(rng);
    uint64_t x = r & 0x3ff, y = (r >> 10) & 0x3ff, z = (r >> 20) & 0x3ff;
    return x | (y << 21) | (z << 42);
}

static void setup(void* state) {
    ChunkMap* m = state;
    m->keys = xalloc(CAP * sizeof(uint64_t));
    m->values = xalloc(CAP * sizeof(uint32_t));
    m->coords = xalloc(INSERTS * sizeof(uint64_t));
    Rng rng = { 0xc4a4 };
    for (int i = 0; i < INSERTS; i++) m->coords[i] = pack(&rng);
}

static void insert(ChunkMap* m, uint64_t key, uint32_t value) {
    uint32_t i = (uint32_t)(mix64(key) & (CAP - 1));
    while (m->keys[i] != EMPTY && m->keys[i] != key) i = (i + 1) & (CAP - 1);
    m->keys[i] = key;
    m->values[i] = value;
}

static int lookup(const ChunkMap* m, uint64_t key, uint32_t* value) {
    uint32_t i = (uint32_t)(mix64(key) & (CAP - 1));
    while (m->keys[i] != EMPTY) {
        if (m->keys[i] == key) { *value = m->values[i]; return 1; }
        i = (i + 1) & (CAP - 1);
    }
    return 0;
}

static uint64_t run(void* state) {
    ChunkMap* m = state;
    memset(m->keys, 0xff, CAP * sizeof(uint64_t));
    for (uint32_t i = 0; i < INSERTS; i++) insert(m, m->coords[i], i);

    Rng rng = { 0x100c };
    uint64_t sum = 0, hits = 0;
    uint32_t value;
    for (int i = 0; i < LOOKUPS; i++) {
        uint64_t r = rng_next(&rng);
        uint64_t key = (r & 1) ? m->coords[(r >> 1) % INSERTS] : pack(&rng);
        if (lookup(m, key, &value)) { sum += value; hits++; }
    }
    return hash_add(sum, hits);
}

static void teardown(void* state) {
    ChunkMap* m = state;
    free(m->keys);
    free(m->values);
    free(m->coords);
}

const Case chunkmap_case = { "chunkmap", sizeof(ChunkMap), setup, run, teardown };
