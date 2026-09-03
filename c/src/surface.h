#ifndef BENCH_SURFACE_H
#define BENCH_SURFACE_H

#include "harness.h"

/// Surface extraction: count the exposed faces of every solid voxel in a
/// 128^3 grid at 75% fill, the pass that finds what a raymarcher can see and
/// what a contact can touch.
extern const Case surface_case;

#endif
