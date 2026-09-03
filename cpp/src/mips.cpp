#include "mips.hpp"

#include <algorithm>
#include <bit>

#include "hash.hpp"

namespace bench {

Mips::Mips()
    : mat_(static_cast<size_t>(chunks) * voxels),
      rows_(static_cast<size_t>(chunks) * rows),
      mip_(static_cast<size_t>(chunks) * mip_words) {
    for (uint32_t i = 0; i < static_cast<uint32_t>(chunks * voxels); i++) mat_[i] = (mix64(i ^ 0x3ea) & 3) == 0;
}

uint64_t Mips::run() {
    uint64_t h = 0;
    for (int c = 0; c < chunks; c++) {
        const uint8_t* mat = mat_.data() + static_cast<size_t>(c) * voxels;
        uint32_t* rows_p = rows_.data() + static_cast<size_t>(c) * rows;
        uint64_t* mip = mip_.data() + static_cast<size_t>(c) * mip_words;
        std::fill(mip, mip + mip_words, 0ull);
        for (int z = 0; z < dim; z++) {
            for (int y = 0; y < dim; y++) {
                const uint8_t* row = mat + (z * dim + y) * dim;
                uint32_t word = 0;
                for (int x = 0; x < dim; x++) word |= static_cast<uint32_t>(row[x] != 0) << x;
                rows_p[z * dim + y] = word;
                if (word == 0) continue;
                for (int bx = 0; bx < mip_dim; bx++) {
                    if (((word >> (bx * 4)) & 0xf) == 0) continue;
                    uint32_t bit = static_cast<uint32_t>(((z / 4) * mip_dim + (y / 4)) * mip_dim + bx);
                    mip[bit >> 6] |= 1ull << (bit & 63);
                }
            }
        }
        uint32_t solid = 0, blocks = 0;
        for (int i = 0; i < rows; i++) solid += static_cast<uint32_t>(std::popcount(rows_p[i]));
        for (int i = 0; i < mip_words; i++) blocks += static_cast<uint32_t>(std::popcount(mip[i]));
        h = hash_add(h, (static_cast<uint64_t>(solid) << 32) | blocks);
    }
    return h;
}

} // namespace bench
