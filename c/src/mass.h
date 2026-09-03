#ifndef BENCH_MASS_H
#define BENCH_MASS_H

#include "harness.h"

/// Mass properties: for 128 bodies of 32^3 voxels with per-material density,
/// accumulate mass, centre of mass and the inertia tensor voxel by voxel, then
/// shift the tensor to the centre of mass.
extern const Case mass_case;

#endif
