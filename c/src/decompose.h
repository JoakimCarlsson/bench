#ifndef BENCH_DECOMPOSE_H
#define BENCH_DECOMPOSE_H

#include "harness.h"

/// Transform decomposition: for 262144 rotation-and-scale pairs, a quarter of
/// them mirrored, build the basis, recover its scale and rotation through
/// the four-branch basis-to-quaternion conversion, invert it, and rotate a
/// point both ways.
extern const Case decompose_case;

#endif
