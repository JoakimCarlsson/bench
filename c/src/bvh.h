#ifndef BENCH_BVH_H
#define BENCH_BVH_H

#include "harness.h"

/// Bounding volume hierarchy: build a tree over 16384 boxes by midpoint
/// partition on the longest centroid axis, leaves of at most four, then trace
/// 50k rays through it counting the boxes each one hits.
extern const Case bvh_case;

#endif
