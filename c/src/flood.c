#include "flood.h"

#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "hash.h"

enum { N = 128, CELLS = N * N * N };

typedef struct { uint8_t* occ; uint32_t* label; uint32_t* queue; } Flood;

static void setup(void* state) {
    Flood* f = state;
    f->occ = xalloc(CELLS);
    f->label = xalloc(CELLS * sizeof(uint32_t));
    f->queue = xalloc(CELLS * sizeof(uint32_t));
    for (uint32_t i = 0; i < CELLS; i++) f->occ[i] = (mix64(i ^ 0x5eed) & 0xff) < 77;
}

static uint64_t run(void* state) {
    Flood* f = state;
    memset(f->label, 0, CELLS * sizeof(uint32_t));
    uint64_t h = 0;
    uint32_t comp = 0;
    for (uint32_t start = 0; start < CELLS; start++) {
        if (!f->occ[start] || f->label[start]) continue;
        comp++;
        f->label[start] = comp;
        uint32_t head = 0, tail = 0, size = 0;
        f->queue[tail++] = start;
        while (head < tail) {
            uint32_t c = f->queue[head++];
            size++;
            uint32_t x = c / (N * N), y = (c / N) % N, z = c % N;
            uint32_t nb[6];
            int count = 0;
            if (x > 0) nb[count++] = c - N * N;
            if (x + 1 < N) nb[count++] = c + N * N;
            if (y > 0) nb[count++] = c - N;
            if (y + 1 < N) nb[count++] = c + N;
            if (z > 0) nb[count++] = c - 1;
            if (z + 1 < N) nb[count++] = c + 1;
            for (int k = 0; k < count; k++) {
                uint32_t n = nb[k];
                if (f->occ[n] && !f->label[n]) {
                    f->label[n] = comp;
                    f->queue[tail++] = n;
                }
            }
        }
        h = hash_add(h, size);
    }
    return hash_add(h, comp);
}

static void teardown(void* state) {
    Flood* f = state;
    free(f->occ);
    free(f->label);
    free(f->queue);
}

const Case flood_case = { "flood", sizeof(Flood), setup, run, teardown };
