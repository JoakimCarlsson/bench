#include "sweep.hpp"

#include <algorithm>

#include "hash.hpp"

namespace bench {

Sweep::Sweep() : boxes_(boxes_n), order_(boxes_n) {
    Rng rng{0xb0c5};
    for (Box& b : boxes_) {
        for (int k = 0; k < 3; k++) {
            b.min[k] = rng.unit() * 200.0f;
            b.max[k] = b.min[k] + 0.5f + rng.unit() * 3.0f;
        }
    }
}

uint64_t Sweep::run() {
    for (uint32_t i = 0; i < boxes_n; i++) order_[i] = i;
    std::sort(order_.begin(), order_.end(), [this](uint32_t a, uint32_t b) {
        float xa = boxes_[a].min[0], xb = boxes_[b].min[0];
        return xa != xb ? xa < xb : a < b;
    });

    uint64_t h = 0, pairs = 0;
    for (uint32_t i = 0; i < boxes_n; i++) {
        const Box& a = boxes_[order_[i]];
        for (uint32_t j = i + 1; j < boxes_n; j++) {
            const Box& b = boxes_[order_[j]];
            if (b.min[0] > a.max[0]) break;
            if (b.min[1] > a.max[1] || a.min[1] > b.max[1]) continue;
            if (b.min[2] > a.max[2] || a.min[2] > b.max[2]) continue;
            h = hash_add(h, (static_cast<uint64_t>(order_[i]) << 32) | order_[j]);
            pairs++;
        }
    }
    return hash_add(h, pairs);
}

} // namespace bench
