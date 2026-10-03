#ifndef BENCH_UNPROJECT_H
#define BENCH_UNPROJECT_H

#include "harness.h"

/// Camera picking: for 131072 cameras build the view from a look-at, the
/// perspective projection and their product, invert it by cofactors, and
/// unproject four NDC points to world space and back.
extern const Case unproject_case;

#endif
