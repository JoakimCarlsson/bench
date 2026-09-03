#pragma once

#include <cstdint>
#include <vector>

namespace bench {

/// Surface extraction: count the exposed faces of every solid voxel in a
/// 128^3 grid at 75% fill, the pass that finds what a raymarcher can see and
/// what a contact can touch.
class Surface {
public:
    static constexpr const char* name = "surface";

    Surface();
    uint64_t run();

private:
    static constexpr uint32_t N = 128;
    static constexpr uint32_t cells = N * N * N;

    bool empty_at(int x, int y, int z) const;

    std::vector<uint8_t> occ_;
};

} // namespace bench
