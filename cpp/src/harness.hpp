#pragma once

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "clock.hpp"

namespace bench {

/// Construct a `Case`, run it `warmup + reps` times, and print one line:
/// `name min_ns median_ns checksum_hex`. A `Case` has a default constructor
/// that allocates its state and a `uint64_t run()` that must be
/// deterministic; a checksum that changes between repetitions is rejected.
/// Returns 0, or non-zero on a nondeterministic kernel.
template <typename Case>
int run_case(const char* name, int reps, int warmup) {
    Case st;
    uint64_t checksum = 0;
    std::vector<uint64_t> times(static_cast<size_t>(reps));
    for (int i = 0; i < warmup + reps; i++) {
        uint64_t t0 = now_ns();
        uint64_t sum = st.run();
        uint64_t t1 = now_ns();
        if (i == 0) {
            checksum = sum;
        } else if (sum != checksum) {
            std::fprintf(stderr, "%s: nondeterministic\n", name);
            return 3;
        }
        if (i >= warmup) times[static_cast<size_t>(i - warmup)] = t1 - t0;
    }
    std::sort(times.begin(), times.end());
    std::printf("%s %llu %llu %016llx\n", name,
                static_cast<unsigned long long>(times.front()),
                static_cast<unsigned long long>(times[static_cast<size_t>(reps / 2)]),
                static_cast<unsigned long long>(checksum));
    return 0;
}

} // namespace bench
