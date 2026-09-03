#include "flood.hpp"

#include <algorithm>

#include "hash.hpp"

namespace bench {

Flood::Flood() : occ_(cells), label_(cells), queue_(cells) {
    for (uint32_t i = 0; i < cells; i++) occ_[i] = (mix64(i ^ 0x5eed) & 0xff) < 77;
}

uint64_t Flood::run() {
    std::fill(label_.begin(), label_.end(), 0u);
    uint64_t h = 0;
    uint32_t comp = 0;
    for (uint32_t start = 0; start < cells; start++) {
        if (!occ_[start] || label_[start]) continue;
        comp++;
        label_[start] = comp;
        uint32_t head = 0, tail = 0, size = 0;
        queue_[tail++] = start;
        while (head < tail) {
            uint32_t c = queue_[head++];
            size++;
            uint32_t x = c / (N * N), y = (c / N) % N, z = c % N;
            uint32_t nb[6];
            int count = 0;
            if (x > 0) nb[count++] = c - N * N;
            if (x + 1 < N) nb[count++] = c + N * N;
            if (y > 0) nb[count++] = c - N;
            if (y + 1 < N) nb[count++] = c + N;
            if (z > 0) nb[count++] = c - 1;
            if (z + 1 < N) nb[count++] = c + 1;
            for (int k = 0; k < count; k++) {
                uint32_t n = nb[k];
                if (occ_[n] && !label_[n]) {
                    label_[n] = comp;
                    queue_[tail++] = n;
                }
            }
        }
        h = hash_add(h, size);
    }
    return hash_add(h, comp);
}

} // namespace bench
