#ifndef BENCH_CLOCK_H
#define BENCH_CLOCK_H

#include <stdint.h>

/// Monotonic nanoseconds from an unspecified origin.
uint64_t now_ns(void);

#endif
