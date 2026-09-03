#ifndef BENCH_SORT_H
#define BENCH_SORT_H

#include "harness.h"

/// Standard-library sort of 2^19 random u64 keys: `qsort`, `std::sort` and
/// `std.mem.sortUnstable`. The one kernel that measures a library rather than
/// the code written here.
extern const Case sort_case;

#endif
