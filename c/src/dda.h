#ifndef BENCH_DDA_H
#define BENCH_DDA_H

#include "harness.h"

/// Voxel raycast: 100k rays walked cell by cell through a 128^3 occupancy
/// bitset until they hit a solid cell or leave the grid.
extern const Case dda_case;

#endif
