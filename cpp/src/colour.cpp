#include "colour.hpp"

#include <algorithm>
#include <bit>

#include "hash.hpp"

namespace bench {

Colour::Colour() : dynamic_(bodies_n), contacts_(contacts_n), used_(bodies_n), colour_(contacts_n) {
    for (uint32_t i = 0; i < bodies_n; i++) dynamic_[i] = (mix64(i ^ 0xc0) & 7) != 0;
    Rng rng{0xc01c};
    for (Pair& c : contacts_) {
        uint32_t a = static_cast<uint32_t>(rng.next() % bodies_n);
        uint32_t b = static_cast<uint32_t>(rng.next() % bodies_n);
        if (b == a) b = (a + 1) % bodies_n;
        c = Pair{a, b};
    }
}

uint64_t Colour::run() {
    std::fill(used_.begin(), used_.end(), 0u);
    uint32_t highest = 0;
    for (int i = 0; i < contacts_n; i++) {
        const Pair& p = contacts_[static_cast<size_t>(i)];
        uint32_t mask = (dynamic_[p.a] ? used_[p.a] : 0) | (dynamic_[p.b] ? used_[p.b] : 0);
        uint32_t c = ~mask == 0 ? overflow : static_cast<uint32_t>(std::countr_zero(~mask));
        if (c >= overflow) c = overflow;
        if (dynamic_[p.a]) used_[p.a] |= 1u << c;
        if (dynamic_[p.b]) used_[p.b] |= 1u << c;
        colour_[static_cast<size_t>(i)] = static_cast<uint8_t>(c);
        if (c > highest) highest = c;
    }
    uint64_t h = 0;
    for (int i = 0; i < contacts_n; i += 8) {
        uint64_t word = 0;
        for (int k = 0; k < 8; k++) word |= static_cast<uint64_t>(colour_[static_cast<size_t>(i + k)]) << (k * 8);
        h = hash_add(h, word);
    }
    return hash_add(h, highest);
}

} // namespace bench
