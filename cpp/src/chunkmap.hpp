#pragma once

#include <cstdint>
#include <optional>
#include <vector>

namespace bench {

/// Chunk lookup: an open-addressing hash map from packed chunk coordinates to
/// slots. 200k inserts, then 2M lookups at a 50% hit rate.
class ChunkMap {
public:
    static constexpr const char* name = "chunkmap";

    ChunkMap();
    uint64_t run();

private:
    static constexpr uint32_t cap = 1 << 19;
    static constexpr int inserts = 200000;
    static constexpr int lookups = 2000000;
    static constexpr uint64_t empty = UINT64_MAX;

    void insert(uint64_t key, uint32_t value);
    std::optional<uint32_t> lookup(uint64_t key) const;

    std::vector<uint64_t> keys_;
    std::vector<uint32_t> values_;
    std::vector<uint64_t> coords_;
};

} // namespace bench
