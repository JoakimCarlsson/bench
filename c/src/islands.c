#include "islands.h"

#include <stdlib.h>

#include "alloc.h"
#include "hash.h"

enum { BODIES = 65536, CONTACTS = 524288 };

typedef struct { uint32_t a, b; } Pair;
typedef struct { uint8_t* dynamic; Pair* contacts; uint32_t* parent; } Islands;

static void setup(void* state) {
    Islands* s = state;
    s->dynamic = xalloc(BODIES);
    s->contacts = xalloc(CONTACTS * sizeof(Pair));
    s->parent = xalloc(BODIES * sizeof(uint32_t));
    for (uint32_t i = 0; i < BODIES; i++) s->dynamic[i] = (mix64(i ^ 0x15) & 7) != 0;
    Rng rng = { 0x151a };
    for (int i = 0; i < CONTACTS; i++) {
        uint32_t a = (uint32_t)(rng_next(&rng) % BODIES);
        uint32_t b = (uint32_t)(rng_next(&rng) % BODIES);
        if (b == a) b = (a + 1) % BODIES;
        s->contacts[i].a = a;
        s->contacts[i].b = b;
    }
}

static uint32_t find(uint32_t* parent, uint32_t i) {
    while (parent[i] != i) {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    return i;
}

static uint64_t run(void* state) {
    Islands* s = state;
    for (uint32_t i = 0; i < BODIES; i++) s->parent[i] = i;
    for (int i = 0; i < CONTACTS; i++) {
        uint32_t a = s->contacts[i].a, b = s->contacts[i].b;
        if (!s->dynamic[a] || !s->dynamic[b]) continue;
        uint32_t ra = find(s->parent, a), rb = find(s->parent, b);
        if (ra == rb) continue;
        if (ra < rb) s->parent[rb] = ra; else s->parent[ra] = rb;
    }
    uint64_t h = 0, islands = 0;
    for (uint32_t i = 0; i < BODIES; i++) {
        if (!s->dynamic[i]) continue;
        uint32_t root = find(s->parent, i);
        if (root == i) islands++;
        h = hash_add(h, ((uint64_t)i << 32) | root);
    }
    return hash_add(h, islands);
}

static void teardown(void* state) {
    Islands* s = state;
    free(s->dynamic);
    free(s->contacts);
    free(s->parent);
}

const Case islands_case = { "islands", sizeof(Islands), setup, run, teardown };
