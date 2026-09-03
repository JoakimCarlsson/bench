#pragma once

#include <cstdint>
#include <vector>

namespace bench {

/// Connected components: 6-connected breadth-first labelling of a 128^3
/// occupancy grid at 30% fill, the pass a collapse step runs to find what
/// still holds together.
class Flood {
public:
    static constexpr const char* name = "flood";

    Flood();
    uint64_t run();

private:
    static constexpr uint32_t N = 128;
    static constexpr uint32_t cells = N * N * N;

    std::vector<uint8_t> occ_;
    std::vector<uint32_t> label_;
    std::vector<uint32_t> queue_;
};

} // namespace bench
