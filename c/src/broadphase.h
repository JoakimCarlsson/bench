#ifndef BENCH_BROADPHASE_H
#define BENCH_BROADPHASE_H

#include "harness.h"

/// Broad phase: 16 frames of 4096 tumbling boxes over a field of 1024
/// static tiles, through the engine's dynamic AABB tree. Each frame refits
/// fat bounds that no longer hold, reinserts those leaves with rotations,
/// finds the new pairs of every moved proxy, records them in a hash map
/// keyed by shape pair, and drops pairs whose fat bounds parted.
extern const Case broadphase_case;

#endif
