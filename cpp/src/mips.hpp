#pragma once

#include <cstdint>
#include <vector>

namespace bench {

/// Chunk occupancy rebuild: for 512 chunks of 32^3 material bytes, rebuild
/// the per-row occupancy bitset and the 4^3-block mip, then popcount both.
class Mips {
public:
    static constexpr const char* name = "mips";

    Mips();
    uint64_t run();

private:
    static constexpr int chunks = 512;
    static constexpr int dim = 32;
    static constexpr int voxels = dim * dim * dim;
    static constexpr int rows = dim * dim;
    static constexpr int mip_dim = dim / 4;
    static constexpr int mip_words = mip_dim * mip_dim * mip_dim / 64;

    std::vector<uint8_t> mat_;
    std::vector<uint32_t> rows_;
    std::vector<uint64_t> mip_;
};

} // namespace bench
