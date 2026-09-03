#include "mips.h"

#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "hash.h"

enum {
    CHUNKS = 512,
    DIM = 32,
    VOXELS = DIM * DIM * DIM,
    ROWS = DIM * DIM,
    MIP_DIM = DIM / 4,
    MIP_WORDS = MIP_DIM * MIP_DIM * MIP_DIM / 64,
};

typedef struct { uint8_t* mat; uint32_t* rows; uint64_t* mip; } Mips;

static void setup(void* state) {
    Mips* s = state;
    s->mat = xalloc((size_t)CHUNKS * VOXELS);
    s->rows = xalloc((size_t)CHUNKS * ROWS * sizeof(uint32_t));
    s->mip = xalloc((size_t)CHUNKS * MIP_WORDS * sizeof(uint64_t));
    for (uint32_t i = 0; i < CHUNKS * VOXELS; i++) s->mat[i] = (mix64(i ^ 0x3ea) & 3) == 0;
}

static uint64_t run(void* state) {
    Mips* s = state;
    uint64_t h = 0;
    for (int c = 0; c < CHUNKS; c++) {
        const uint8_t* mat = s->mat + (size_t)c * VOXELS;
        uint32_t* rows = s->rows + (size_t)c * ROWS;
        uint64_t* mip = s->mip + (size_t)c * MIP_WORDS;
        memset(mip, 0, MIP_WORDS * sizeof(uint64_t));
        for (int z = 0; z < DIM; z++) {
            for (int y = 0; y < DIM; y++) {
                const uint8_t* row = mat + (z * DIM + y) * DIM;
                uint32_t word = 0;
                for (int x = 0; x < DIM; x++) word |= (uint32_t)(row[x] != 0) << x;
                rows[z * DIM + y] = word;
                if (word == 0) continue;
                for (int bx = 0; bx < MIP_DIM; bx++) {
                    if (((word >> (bx * 4)) & 0xf) == 0) continue;
                    uint32_t bit = ((z / 4) * MIP_DIM + (y / 4)) * MIP_DIM + bx;
                    mip[bit >> 6] |= 1ull << (bit & 63);
                }
            }
        }
        uint32_t solid = 0, blocks = 0;
        for (int i = 0; i < ROWS; i++) solid += (uint32_t)__builtin_popcount(rows[i]);
        for (int i = 0; i < MIP_WORDS; i++) blocks += (uint32_t)__builtin_popcountll(mip[i]);
        h = hash_add(h, ((uint64_t)solid << 32) | blocks);
    }
    return h;
}

static void teardown(void* state) {
    Mips* s = state;
    free(s->mat);
    free(s->rows);
    free(s->mip);
}

const Case mips_case = { "mips", sizeof(Mips), setup, run, teardown };
