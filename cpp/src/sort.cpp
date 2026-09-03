#include "sort.hpp"

#include <algorithm>

#include "hash.hpp"

namespace bench {

Sort::Sort() : keys_(keys_n), initial_(keys_n) {
    Rng rng{0x5027};
    for (uint64_t& k : initial_) k = rng.next();
}

uint64_t Sort::run() {
    std::copy(initial_.begin(), initial_.end(), keys_.begin());
    std::sort(keys_.begin(), keys_.end());
    uint64_t h = 0;
    for (int i = 0; i < keys_n; i += 977) h = hash_add(h, keys_[static_cast<size_t>(i)]);
    return hash_add(h, keys_.back());
}

} // namespace bench
