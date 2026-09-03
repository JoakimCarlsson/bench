#include "surface.hpp"

#include "hash.hpp"

namespace bench {

Surface::Surface() : occ_(cells) {
    for (uint32_t i = 0; i < cells; i++) occ_[i] = (mix64(i ^ 0xface) & 3) != 0;
}

bool Surface::empty_at(int x, int y, int z) const {
    if (static_cast<unsigned>(x) >= N || static_cast<unsigned>(y) >= N || static_cast<unsigned>(z) >= N) return true;
    return occ_[(static_cast<uint32_t>(x) * N + static_cast<uint32_t>(y)) * N + static_cast<uint32_t>(z)] == 0;
}

uint64_t Surface::run() {
    uint64_t h = 0, faces_total = 0;
    for (int x = 0; x < static_cast<int>(N); x++) {
        for (int y = 0; y < static_cast<int>(N); y++) {
            for (int z = 0; z < static_cast<int>(N); z++) {
                uint32_t cell = (static_cast<uint32_t>(x) * N + static_cast<uint32_t>(y)) * N + static_cast<uint32_t>(z);
                if (!occ_[cell]) continue;
                uint32_t faces = 0;
                faces += empty_at(x - 1, y, z);
                faces += empty_at(x + 1, y, z);
                faces += empty_at(x, y - 1, z);
                faces += empty_at(x, y + 1, z);
                faces += empty_at(x, y, z - 1);
                faces += empty_at(x, y, z + 1);
                if (faces == 0) continue;
                h = hash_add(h, (static_cast<uint64_t>(cell) << 3) | faces);
                faces_total += faces;
            }
        }
    }
    return hash_add(h, faces_total);
}

} // namespace bench
