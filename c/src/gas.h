#ifndef BENCH_GAS_H
#define BENCH_GAS_H

#include "harness.h"

/// Particle gas: 4 substeps of the engine's gas solver over 8192 particles
/// stirred by 4 moving boxes. Each substep hashes particles into cells,
/// sorts the keys, searches the 27 neighbouring cells with a lower-bound
/// binary search for pressure and viscosity, pushes particles out of and
/// along the movers, then integrates positions.
extern const Case gas_case;

#endif
