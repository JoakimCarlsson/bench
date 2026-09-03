#pragma once

#include <bit>
#include <cstdint>

namespace bench {

/// splitmix64 finaliser. The one hash every kernel and checksum uses, so a
/// checksum is comparable across the three implementations.
constexpr uint64_t mix64(uint64_t z) {
    z ^= z >> 30;
    z *= 0xbf58476d1ce4e5b9ull;
    z ^= z >> 27;
    z *= 0x94d049bb133111ebull;
    z ^= z >> 31;
    return z;
}

/// Deterministic generator: a counter through `mix64`.
struct Rng {
    uint64_t s;

    uint64_t next() {
        s += 0x9e3779b97f4a7c15ull;
        return mix64(s);
    }

    /// Uniform in [0, 1) with 24 bits of precision, so it is exact in f32.
    float unit() {
        return static_cast<float>(next() >> 40) * (1.0f / 16777216.0f);
    }
};

/// Fold `v` into the running checksum `h`.
constexpr uint64_t hash_add(uint64_t h, uint64_t v) {
    return mix64(h ^ v);
}

inline uint32_t f32_bits(float f) {
    return std::bit_cast<uint32_t>(f);
}

} // namespace bench
