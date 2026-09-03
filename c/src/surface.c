#include "surface.h"

#include <stdlib.h>

#include "alloc.h"
#include "hash.h"

enum { N = 128, CELLS = N * N * N };

typedef struct { uint8_t* occ; } Surface;

static void setup(void* state) {
    Surface* s = state;
    s->occ = xalloc(CELLS);
    for (uint32_t i = 0; i < CELLS; i++) s->occ[i] = (mix64(i ^ 0xface) & 3) != 0;
}

static inline int empty_at(const uint8_t* occ, int x, int y, int z) {
    if ((unsigned)x >= N || (unsigned)y >= N || (unsigned)z >= N) return 1;
    return occ[((uint32_t)x * N + (uint32_t)y) * N + (uint32_t)z] == 0;
}

static uint64_t run(void* state) {
    Surface* s = state;
    uint64_t h = 0, faces_total = 0;
    for (int x = 0; x < N; x++) {
        for (int y = 0; y < N; y++) {
            for (int z = 0; z < N; z++) {
                uint32_t cell = ((uint32_t)x * N + (uint32_t)y) * N + (uint32_t)z;
                if (!s->occ[cell]) continue;
                uint32_t faces = 0;
                faces += empty_at(s->occ, x - 1, y, z);
                faces += empty_at(s->occ, x + 1, y, z);
                faces += empty_at(s->occ, x, y - 1, z);
                faces += empty_at(s->occ, x, y + 1, z);
                faces += empty_at(s->occ, x, y, z - 1);
                faces += empty_at(s->occ, x, y, z + 1);
                if (faces == 0) continue;
                h = hash_add(h, ((uint64_t)cell << 3) | faces);
                faces_total += faces;
            }
        }
    }
    return hash_add(h, faces_total);
}

static void teardown(void* state) {
    Surface* s = state;
    free(s->occ);
}

const Case surface_case = { "surface", sizeof(Surface), setup, run, teardown };
