#ifndef BENCH_TRANSFORM_H
#define BENCH_TRANSFORM_H

#include "harness.h"

/// Scene-graph propagation: 8 frames over a 4-ary tree of 65536 nodes. Each
/// node advances its rotation, builds a scaled local transform, composes it
/// with its parent's world transform, and multiplies the result into a
/// model-view-projection matrix.
extern const Case transform_case;

#endif
