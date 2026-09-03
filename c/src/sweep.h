#ifndef BENCH_SWEEP_H
#define BENCH_SWEEP_H

#include "harness.h"

/// Sort-and-sweep broadphase: 16384 boxes sorted on min x with the standard
/// library sort, then swept for overlapping pairs.
extern const Case sweep_case;

#endif
