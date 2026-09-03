#ifndef BENCH_COLOUR_H
#define BENCH_COLOUR_H

#include "harness.h"

/// Graph colouring: assign each of 524288 contacts the lowest colour not used
/// by another contact on either of its dynamic bodies, so every colour is a
/// set of contacts a solver can run in parallel. Bitmask per body, 31 colours
/// plus an overflow bucket.
extern const Case colour_case;

#endif
