#pragma once

#include <cstdint>
#include <vector>

namespace bench {

/// Voxel raycast: 100k rays walked cell by cell through a 128^3 occupancy
/// bitset until they hit a solid cell or leave the grid.
class Dda {
public:
    static constexpr const char* name = "dda";

    Dda();
    uint64_t run();

private:
    static constexpr uint32_t N = 128;
    static constexpr int rays = 100000;

    bool solid(int x, int y, int z) const;

    std::vector<uint64_t> words_;
};

} // namespace bench
