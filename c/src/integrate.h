#ifndef BENCH_INTEGRATE_H
#define BENCH_INTEGRATE_H

#include "harness.h"

/// Rigid-body integration: 32 steps over 65536 bodies, each with a position,
/// a velocity, an angular velocity and a quaternion that is renormalised every
/// step. Streaming float math with no indirection.
extern const Case integrate_case;

#endif
