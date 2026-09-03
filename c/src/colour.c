#include "colour.h"

#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "hash.h"

enum { BODIES = 65536, CONTACTS = 524288, OVERFLOW = 31 };

typedef struct { uint32_t a, b; } Pair;
typedef struct { uint8_t* dynamic; Pair* contacts; uint32_t* used; uint8_t* colour; } Colour;

static void setup(void* state) {
    Colour* s = state;
    s->dynamic = xalloc(BODIES);
    s->contacts = xalloc(CONTACTS * sizeof(Pair));
    s->used = xalloc(BODIES * sizeof(uint32_t));
    s->colour = xalloc(CONTACTS);
    for (uint32_t i = 0; i < BODIES; i++) s->dynamic[i] = (mix64(i ^ 0xc0) & 7) != 0;
    Rng rng = { 0xc01c };
    for (int i = 0; i < CONTACTS; i++) {
        uint32_t a = (uint32_t)(rng_next(&rng) % BODIES);
        uint32_t b = (uint32_t)(rng_next(&rng) % BODIES);
        if (b == a) b = (a + 1) % BODIES;
        s->contacts[i].a = a;
        s->contacts[i].b = b;
    }
}

static uint64_t run(void* state) {
    Colour* s = state;
    memset(s->used, 0, BODIES * sizeof(uint32_t));
    uint32_t highest = 0;
    for (int i = 0; i < CONTACTS; i++) {
        uint32_t a = s->contacts[i].a, b = s->contacts[i].b;
        uint32_t mask = (s->dynamic[a] ? s->used[a] : 0) | (s->dynamic[b] ? s->used[b] : 0);
        uint32_t c = ~mask == 0 ? OVERFLOW : (uint32_t)__builtin_ctz(~mask);
        if (c >= OVERFLOW) c = OVERFLOW;
        if (s->dynamic[a]) s->used[a] |= 1u << c;
        if (s->dynamic[b]) s->used[b] |= 1u << c;
        s->colour[i] = (uint8_t)c;
        if (c > highest) highest = c;
    }
    uint64_t h = 0;
    for (int i = 0; i < CONTACTS; i += 8) {
        uint64_t word = 0;
        for (int k = 0; k < 8; k++) word |= (uint64_t)s->colour[i + k] << (k * 8);
        h = hash_add(h, word);
    }
    return hash_add(h, highest);
}

static void teardown(void* state) {
    Colour* s = state;
    free(s->dynamic);
    free(s->contacts);
    free(s->used);
    free(s->colour);
}

const Case colour_case = { "colour", sizeof(Colour), setup, run, teardown };
