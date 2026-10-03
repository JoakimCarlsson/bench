#ifndef BENCH_WIDE_H
#define BENCH_WIDE_H

#include "harness.h"

/// The engine's eight-lane contact solver, single-threaded: 4 steps of a
/// 32x32 field of 8-box stacks, 8192 bodies and about 12k contacts coloured
/// by the engine's constraint graph. Each step prepares constraints, packs
/// each colour into eight-lane bundles, runs 4 substeps of warm start, soft
/// biased solve, position integration and relaxed solve with friction, then
/// restitution, and writes impulses and poses back. AVX intrinsics.
extern const Case wide_case;

#endif
