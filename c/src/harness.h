#ifndef BENCH_HARNESS_H
#define BENCH_HARNESS_H

#include <stddef.h>
#include <stdint.h>

/// One kernel: how big its state is and the three functions over it.
/// `run` must be deterministic; the harness rejects a kernel whose checksum
/// changes between repetitions.
typedef struct {
    const char* name;
    size_t state_size;
    void (*setup)(void* state);
    uint64_t (*run)(void* state);
    void (*teardown)(void* state);
} Case;

/// Set up, run `warmup + reps` times, tear down, and print one line:
/// `name min_ns median_ns checksum_hex`. Returns 0, or non-zero on a
/// nondeterministic kernel.
int run_case(const Case* c, int reps, int warmup);

#endif
