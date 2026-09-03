#pragma once

#include <cstdint>
#include <vector>

namespace bench {

/// Sort-and-sweep broadphase: 16384 boxes sorted on min x with the standard
/// library sort, then swept for overlapping pairs.
class Sweep {
public:
    static constexpr const char* name = "sweep";

    struct Box { float min[3], max[3]; };

    Sweep();
    uint64_t run();

private:
    static constexpr uint32_t boxes_n = 16384;

    std::vector<Box> boxes_;
    std::vector<uint32_t> order_;
};

} // namespace bench
