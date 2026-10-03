#ifndef BENCH_BOXBOX_H
#define BENCH_BOXBOX_H

#include "harness.h"

/// Box-box narrowphase: 8 frames over 8192 pairs of oriented boxes, a
/// quarter of them stacked face to face. Separating-axis tests over 15 axes,
/// incident-face clipping against the reference face, reduction to four
/// points, and warm starting from the previous frame's manifold.
extern const Case boxbox_case;

#endif
