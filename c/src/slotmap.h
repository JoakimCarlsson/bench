#ifndef BENCH_SLOTMAP_H
#define BENCH_SLOTMAP_H

#include "harness.h"

/// Generational slot map: 4M random insert, remove and lookup operations
/// over 65536 slots, with a stale-handle lookup on every hit. Branchy
/// integer code over a free list, the shape of an entity or body registry.
extern const Case slotmap_case;

#endif
