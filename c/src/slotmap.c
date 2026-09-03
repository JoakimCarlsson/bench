#include "slotmap.h"

#include <stdlib.h>

#include "alloc.h"
#include "hash.h"

enum { CAP = 65536, OPS = 4000000 };

typedef struct { uint32_t gen; uint32_t next; uint64_t value; } Slot;
typedef struct { Slot* slots; uint64_t* handles; uint32_t head; uint32_t count; } SlotMap;

static void setup(void* state) {
    SlotMap* m = state;
    m->slots = xalloc(CAP * sizeof(Slot));
    m->handles = xalloc(CAP * sizeof(uint64_t));
}

static void reset(SlotMap* m) {
    for (uint32_t i = 0; i < CAP; i++) {
        m->slots[i].gen = 1;
        m->slots[i].next = i + 1;
        m->slots[i].value = 0;
    }
    m->head = 0;
    m->count = 0;
}

static void insert(SlotMap* m, uint64_t value) {
    if (m->count >= CAP) return;
    uint32_t idx = m->head;
    m->head = m->slots[idx].next;
    m->slots[idx].value = value;
    m->handles[m->count++] = ((uint64_t)m->slots[idx].gen << 32) | idx;
}

static void remove_at(SlotMap* m, uint32_t k) {
    uint64_t hd = m->handles[k];
    m->handles[k] = m->handles[--m->count];
    uint32_t idx = (uint32_t)hd;
    m->slots[idx].gen++;
    m->slots[idx].next = m->head;
    m->head = idx;
}

static int lookup(const SlotMap* m, uint64_t hd, uint64_t* value) {
    uint32_t idx = (uint32_t)hd;
    if (m->slots[idx].gen != (uint32_t)(hd >> 32)) return 0;
    *value = m->slots[idx].value;
    return 1;
}

static uint64_t run(void* state) {
    SlotMap* m = state;
    reset(m);
    uint64_t sum = 0, misses = 0, value;
    Rng rng = { 0x510 };
    for (int op = 0; op < OPS; op++) {
        uint64_t r = rng_next(&rng);
        switch (r & 3) {
        case 0:
        case 1:
            insert(m, r);
            break;
        case 2:
            if (m->count > 0) remove_at(m, (uint32_t)((r >> 2) % m->count));
            break;
        default:
            if (m->count > 0) {
                uint64_t hd = m->handles[(r >> 2) % m->count];
                if (lookup(m, hd, &value)) sum += value;
                if (lookup(m, hd ^ (1ull << 32), &value)) sum += value; else misses++;
            }
            break;
        }
    }
    return hash_add(hash_add(sum, misses), m->count);
}

static void teardown(void* state) {
    SlotMap* m = state;
    free(m->slots);
    free(m->handles);
}

const Case slotmap_case = { "slotmap", sizeof(SlotMap), setup, run, teardown };
