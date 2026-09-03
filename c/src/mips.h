#ifndef BENCH_MIPS_H
#define BENCH_MIPS_H

#include "harness.h"

/// Chunk occupancy rebuild: for 512 chunks of 32^3 material bytes, rebuild
/// the per-row occupancy bitset and the 4^3-block mip, then popcount both.
extern const Case mips_case;

#endif
