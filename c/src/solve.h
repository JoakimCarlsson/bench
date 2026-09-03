#ifndef BENCH_SOLVE_H
#define BENCH_SOLVE_H

#include "harness.h"

/// Contact solver: 8 frames of substepped sequential impulses over 4096
/// bodies and 16384 contacts, with accumulated-impulse clamping. Float math
/// through indirect body indices, the shape of a rigid-body solve loop.
extern const Case solve_case;

#endif
