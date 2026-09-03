#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace bench {

/// Generational slot map: 4M random insert, remove and lookup operations
/// over 65536 slots, with a stale-handle lookup on every hit. Branchy
/// integer code over a free list, the shape of an entity or body registry.
class SlotMap {
public:
    static constexpr const char* name = "slotmap";

    struct Slot { uint32_t gen; uint32_t next; uint64_t value; };

    SlotMap();
    uint64_t run();

private:
    static constexpr uint32_t cap = 65536;
    static constexpr int ops = 4000000;

    void reset();
    void insert(uint64_t value);
    void remove_at(uint32_t k);
    std::optional<uint64_t> lookup(uint64_t handle) const;

    std::vector<Slot> slots_;
    std::vector<uint64_t> handles_;
    uint32_t head_ = 0;
    uint32_t count_ = 0;
};

} // namespace bench
