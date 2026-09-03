#ifndef BENCH_ALLOC_H
#define BENCH_ALLOC_H

#include <stddef.h>

/// Zeroed allocation that exits the process on failure. Kernels allocate in
/// setup only, never in the timed region.
void* xalloc(size_t bytes);

#endif
