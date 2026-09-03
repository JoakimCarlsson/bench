#include "chunkmap.hpp"

#include <algorithm>

#include "hash.hpp"

namespace bench {

namespace {

uint64_t pack(Rng& rng) {
    uint64_t r = rng.next();
    uint64_t x = r & 0x3ff, y = (r >> 10) & 0x3ff, z = (r >> 20) & 0x3ff;
    return x | (y << 21) | (z << 42);
}

} // namespace

ChunkMap::ChunkMap() : keys_(cap), values_(cap), coords_(inserts) {
    Rng rng{0xc4a4};
    for (uint64_t& c : coords_) c = pack(rng);
}

void ChunkMap::insert(uint64_t key, uint32_t value) {
    uint32_t i = static_cast<uint32_t>(mix64(key) & (cap - 1));
    while (keys_[i] != empty && keys_[i] != key) i = (i + 1) & (cap - 1);
    keys_[i] = key;
    values_[i] = value;
}

std::optional<uint32_t> ChunkMap::lookup(uint64_t key) const {
    uint32_t i = static_cast<uint32_t>(mix64(key) & (cap - 1));
    while (keys_[i] != empty) {
        if (keys_[i] == key) return values_[i];
        i = (i + 1) & (cap - 1);
    }
    return std::nullopt;
}

uint64_t ChunkMap::run() {
    std::fill(keys_.begin(), keys_.end(), empty);
    for (uint32_t i = 0; i < static_cast<uint32_t>(inserts); i++) insert(coords_[i], i);

    Rng rng{0x100c};
    uint64_t sum = 0, hits = 0;
    for (int i = 0; i < lookups; i++) {
        uint64_t r = rng.next();
        uint64_t key = (r & 1) ? coords_[(r >> 1) % inserts] : pack(rng);
        if (auto v = lookup(key)) { sum += *v; hits++; }
    }
    return hash_add(sum, hits);
}

} // namespace bench
