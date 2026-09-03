#pragma once

#include <cstdint>
#include <vector>

namespace bench {

/// Bounding volume hierarchy: build a tree over 16384 boxes by midpoint
/// partition on the longest centroid axis, leaves of at most four, then trace
/// 20k rays through it counting the boxes each one hits.
class Bvh {
public:
    static constexpr const char* name = "bvh";

    struct Box { float min[3], max[3]; };
    struct Node { float min[3], max[3]; uint32_t first, count; };

    Bvh();
    uint64_t run();

private:
    static constexpr uint32_t boxes_n = 16384;
    static constexpr int rays = 20000;
    static constexpr uint32_t leaf = 4;
    static constexpr uint32_t max_nodes = 2 * boxes_n;
    static constexpr uint32_t stack_depth = 64;

    void build(uint32_t node, uint32_t lo, uint32_t hi);
    uint32_t trace(const float* o, const float* inv) const;

    std::vector<Box> boxes_;
    std::vector<Node> nodes_;
    std::vector<uint32_t> order_;
    uint32_t node_count_ = 0;
};

} // namespace bench
