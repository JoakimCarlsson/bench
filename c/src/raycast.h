#ifndef BENCH_RAYCAST_H
#define BENCH_RAYCAST_H

#include "harness.h"

/// Closest-hit ray casts: 1024 rays, each tested against 1024 oriented boxes
/// with the slab test in box space, shrinking the search to the closest hit
/// so far.
extern const Case raycast_case;

#endif
