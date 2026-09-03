#ifndef BENCH_CHUNKMAP_H
#define BENCH_CHUNKMAP_H

#include "harness.h"

/// Chunk lookup: an open-addressing hash map from packed chunk coordinates to
/// slots. 200k inserts, then 2M lookups at a 50% hit rate.
extern const Case chunkmap_case;

#endif
