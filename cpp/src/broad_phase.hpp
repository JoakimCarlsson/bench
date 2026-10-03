#pragma once

#include <cstdint>
#include <vector>

#include "aabb_tree.hpp"
#include "contact.hpp"

/// The engine's broad phase: static, kinematic and dynamic AABB trees, a
/// move buffer, and pair finding for the proxies that moved.
namespace bench::phys {

inline constexpr float max_aabb_margin = 0.05f;
inline constexpr float aabb_margin_fraction = 0.125f;

/// Fat-bounds margin of a box with half extents `h`, the engine's
/// shape_margin.
inline float shape_margin(vm::Vec3 h) {
    const float extent = h.x > h.y ? (h.x > h.z ? h.x : h.z) : (h.y > h.z ? h.y : h.z);
    return max_aabb_margin < aabb_margin_fraction * 2.0f * extent ? max_aabb_margin : aabb_margin_fraction * 2.0f * extent;
}

class BroadPhase final {
  public:
    using ProxyId = std::int32_t;
    static constexpr ProxyId null_proxy = -1;

    /// Adds a proxy to the tree for its kind and buffers it as moved.
    [[nodiscard]] ProxyId create_proxy(ShapeRef ref, const vm::Aabb& fat_aabb,
                                       bool kinematic = false);
    /// Removes a proxy and frees its id.
    void destroy_proxy(ProxyId id);
    /// Gives a proxy new fat bounds and buffers it as moved.
    void move_proxy(ProxyId id, const vm::Aabb& fat_aabb);

    /// Fat bounds of a proxy.
    [[nodiscard]] const vm::Aabb& fat_aabb(ProxyId id) const noexcept {
        return proxies_[static_cast<std::size_t>(id)].fat_aabb;
    }
    /// Shape a proxy stands for.
    [[nodiscard]] const ShapeRef& shape(ProxyId id) const noexcept {
        return proxies_[static_cast<std::size_t>(id)].ref;
    }
    /// Number of buffered moves.
    [[nodiscard]] std::size_t moved_count() const noexcept {
        return move_buffer_.size();
    }

    /// Calls `on_pair` once for every new overlap of the moved proxies in
    /// [begin, end), dynamic before static and lower id first.
    template <class Callback>
    void query_moved(std::size_t begin, std::size_t end,
                     Callback&& on_pair) const {
        for (std::size_t slot = begin; slot < end; ++slot) {
            const ProxyId query_id = move_buffer_[slot];
            const Proxy& query = proxies_[static_cast<std::size_t>(query_id)];
            if (!query.alive) {
                continue;
            }
            const AabbTree* found_tree = &dynamic_tree_;
            const auto visit = [&](AabbTree::NodeId node) {
                const ProxyId found_id =
                    static_cast<ProxyId>(found_tree->user_data(node));
                if (found_id == query_id) {
                    return true;
                }
                const Proxy& found =
                    proxies_[static_cast<std::size_t>(found_id)];
                if (!query.is_static && !found.is_static &&
                    query.ref.body == found.ref.body) {
                    return true;
                }
                if (found.moved && found_id < query_id) {
                    return true;
                }
                const bool query_first =
                    !query.is_static &&
                    (found.is_static || query_id < found_id);
                if (query_first) {
                    on_pair(query_id, found_id);
                } else {
                    on_pair(found_id, query_id);
                }
                return true;
            };
            dynamic_tree_.query(query.fat_aabb, visit);
            if (!query.is_static) {
                found_tree = &static_tree_;
                static_tree_.query(query.fat_aabb, visit);
                found_tree = &kinematic_tree_;
                kinematic_tree_.query(query.fat_aabb, visit);
            }
        }
    }

    /// Reports the pairs of every moved proxy, then clears the move buffer.
    template <class Callback> void update_pairs(Callback&& on_pair) {
        query_moved(0, move_buffer_.size(), on_pair);
        clear_moves();
    }

    /// Clears the move buffer and the proxies' moved flags.
    void clear_moves() {
        for (const ProxyId id : move_buffer_) {
            proxies_[static_cast<std::size_t>(id)].moved = false;
        }
        move_buffer_.clear();
    }

  private:
    struct Proxy {
        ShapeRef ref{};
        vm::Aabb fat_aabb{};
        AabbTree::NodeId node{AabbTree::null_node};
        bool is_static{};
        bool kinematic{};
        bool moved{};
        bool alive{};
    };

    /// Tree that holds a proxy of this kind.
    [[nodiscard]] AabbTree& tree_of(const Proxy& proxy) noexcept;
    /// Adds a proxy to the move buffer once.
    void buffer_move(ProxyId id);

    std::vector<Proxy> proxies_;
    std::vector<ProxyId> free_;
    std::vector<ProxyId> move_buffer_;
    AabbTree static_tree_;
    AabbTree kinematic_tree_;
    AabbTree dynamic_tree_;
};

} // namespace bench::phys
