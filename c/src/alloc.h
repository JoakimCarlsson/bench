#ifndef BENCH_ALLOC_H
#define BENCH_ALLOC_H

#include <stddef.h>

/// Zeroed allocation that exits the process on failure. Kernels allocate in
/// setup only, never in the timed region.
void* xalloc(size_t bytes);

/// Uninitialised allocation aligned to `alignment`, a power of two, that
/// exits the process on failure. Release it with `xfree_aligned`.
void* xalloc_aligned(size_t bytes, size_t alignment);

/// Releases memory from `xalloc_aligned`; null is ignored.
void xfree_aligned(void* p);

#endif
