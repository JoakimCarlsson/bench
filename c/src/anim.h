#ifndef BENCH_ANIM_H
#define BENCH_ANIM_H

#include "harness.h"

/// Animation playback: 1024 players of 3 clips each, every clip 8 tracks of 48
/// keys, stepped for 48 frames of 1/30 s. The engine's `AnimationWorld`
/// samples every playing clip and every blend from the previous clip, by key
/// search, easing and interpolation, accumulates the samples by target path
/// in a string-keyed accumulator list, emits per-target samples, sums root
/// motion, fires event keys, and chains to the next or queued clip. A scripted
/// driver changes clips, blends from captured poses, queues, seeks and plays
/// sections. Differences from the engine: slerp is a normalised lerp, sine and
/// exponential easing are polynomials, and `fmod` is a floor; each keeps the
/// kernel free of libm calls, which round differently between languages.
extern const Case anim_case;

#endif
