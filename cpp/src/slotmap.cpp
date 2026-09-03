#include "slotmap.hpp"

#include "hash.hpp"

namespace bench {

SlotMap::SlotMap() : slots_(cap), handles_(cap) {}

void SlotMap::reset() {
    for (uint32_t i = 0; i < cap; i++) slots_[i] = Slot{1, i + 1, 0};
    head_ = 0;
    count_ = 0;
}

void SlotMap::insert(uint64_t value) {
    if (count_ >= cap) return;
    uint32_t idx = head_;
    head_ = slots_[idx].next;
    slots_[idx].value = value;
    handles_[count_++] = (static_cast<uint64_t>(slots_[idx].gen) << 32) | idx;
}

void SlotMap::remove_at(uint32_t k) {
    uint64_t hd = handles_[k];
    handles_[k] = handles_[--count_];
    uint32_t idx = static_cast<uint32_t>(hd);
    slots_[idx].gen++;
    slots_[idx].next = head_;
    head_ = idx;
}

std::optional<uint64_t> SlotMap::lookup(uint64_t hd) const {
    uint32_t idx = static_cast<uint32_t>(hd);
    if (slots_[idx].gen != static_cast<uint32_t>(hd >> 32)) return std::nullopt;
    return slots_[idx].value;
}

uint64_t SlotMap::run() {
    reset();
    uint64_t sum = 0, misses = 0;
    Rng rng{0x510};
    for (int op = 0; op < ops; op++) {
        uint64_t r = rng.next();
        switch (r & 3) {
        case 0:
        case 1:
            insert(r);
            break;
        case 2:
            if (count_ > 0) remove_at(static_cast<uint32_t>((r >> 2) % count_));
            break;
        default:
            if (count_ > 0) {
                uint64_t hd = handles_[(r >> 2) % count_];
                if (auto v = lookup(hd)) sum += *v;
                if (auto v = lookup(hd ^ (1ull << 32))) sum += *v; else misses++;
            }
            break;
        }
    }
    return hash_add(hash_add(sum, misses), count_);
}

} // namespace bench
