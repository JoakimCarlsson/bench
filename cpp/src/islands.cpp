#include "islands.hpp"

#include "hash.hpp"

namespace bench {

Islands::Islands() : dynamic_(bodies_n), contacts_(contacts_n), parent_(bodies_n) {
    for (uint32_t i = 0; i < bodies_n; i++) dynamic_[i] = (mix64(i ^ 0x15) & 7) != 0;
    Rng rng{0x151a};
    for (Pair& c : contacts_) {
        uint32_t a = static_cast<uint32_t>(rng.next() % bodies_n);
        uint32_t b = static_cast<uint32_t>(rng.next() % bodies_n);
        if (b == a) b = (a + 1) % bodies_n;
        c = Pair{a, b};
    }
}

uint32_t Islands::find(uint32_t i) {
    while (parent_[i] != i) {
        parent_[i] = parent_[parent_[i]];
        i = parent_[i];
    }
    return i;
}

uint64_t Islands::run() {
    for (uint32_t i = 0; i < bodies_n; i++) parent_[i] = i;
    for (const Pair& c : contacts_) {
        if (!dynamic_[c.a] || !dynamic_[c.b]) continue;
        uint32_t ra = find(c.a), rb = find(c.b);
        if (ra == rb) continue;
        if (ra < rb) parent_[rb] = ra; else parent_[ra] = rb;
    }
    uint64_t h = 0, islands = 0;
    for (uint32_t i = 0; i < bodies_n; i++) {
        if (!dynamic_[i]) continue;
        uint32_t root = find(i);
        if (root == i) islands++;
        h = hash_add(h, (static_cast<uint64_t>(i) << 32) | root);
    }
    return hash_add(h, islands);
}

} // namespace bench
