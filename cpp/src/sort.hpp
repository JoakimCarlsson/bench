#pragma once

#include <cstdint>
#include <vector>

namespace bench {

/// Standard-library sort of 2^19 random u64 keys: `qsort`, `std::sort` and
/// `std.mem.sortUnstable`. The one kernel that measures a library rather than
/// the code written here.
class Sort {
public:
    static constexpr const char* name = "sort";

    Sort();
    uint64_t run();

private:
    static constexpr int keys_n = 1 << 19;

    std::vector<uint64_t> keys_;
    std::vector<uint64_t> initial_;
};

} // namespace bench
