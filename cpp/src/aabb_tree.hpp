#pragma once

#include <cstdint>
#include <vector>

#include "vecmath.hpp"

/// The engine's dynamic AABB tree: surface-area-heuristic insertion with
/// AVL-style rotations, AABB queries and ray casts.
namespace bench::phys {

class AabbTree final {
  public:
    using NodeId = std::int32_t;
    static constexpr NodeId null_node = -1;

    /// Inserts a leaf for `aabb` carrying `user_data`.
    [[nodiscard]] NodeId create_proxy(const vm::Aabb& aabb,
                                      std::uint32_t user_data);
    /// Removes a leaf and frees its node.
    void destroy_proxy(NodeId proxy);
    /// Reinserts a leaf with new bounds.
    void move_proxy(NodeId proxy, const vm::Aabb& aabb);

    /// Bounds of a leaf.
    [[nodiscard]] const vm::Aabb& aabb(NodeId proxy) const noexcept {
        return nodes_[static_cast<std::size_t>(proxy)].aabb;
    }
    /// User data of a leaf.
    [[nodiscard]] std::uint32_t user_data(NodeId proxy) const noexcept {
        return nodes_[static_cast<std::size_t>(proxy)].user_data;
    }
    /// Number of leaves.
    [[nodiscard]] std::size_t proxy_count() const noexcept {
        return proxy_count_;
    }
    /// Calls `callback` with every leaf overlapping `aabb` until it returns
    /// false.
    template <class Callback>
    void query(const vm::Aabb& aabb, Callback&& callback) const {
        std::vector<NodeId> stack;
        stack.reserve(64);
        stack.push_back(root_);
        while (!stack.empty()) {
            const NodeId id = stack.back();
            stack.pop_back();
            if (id == null_node) {
                continue;
            }
            const Node& node = nodes_[static_cast<std::size_t>(id)];
            if (!overlaps(node.aabb, aabb)) {
                continue;
            }
            if (node.is_leaf()) {
                if (!callback(id)) {
                    return;
                }
            } else {
                stack.push_back(node.child1);
                stack.push_back(node.child2);
            }
        }
    }

  private:
    struct Node {
        vm::Aabb aabb{};
        std::uint32_t user_data{};
        NodeId parent{null_node};
        NodeId child1{null_node};
        NodeId child2{null_node};
        std::int32_t height{};

        /// Whether the node has no children.
        [[nodiscard]] bool is_leaf() const noexcept {
            return child1 == null_node;
        }
    };

    /// A node from the free list, or a new one.
    [[nodiscard]] NodeId allocate_node();
    /// Returns a node to the free list.
    void free_node(NodeId id);
    /// Descends by the surface area heuristic, adds a parent over the chosen
    /// sibling, and refits and balances up to the root.
    void insert_leaf(NodeId leaf);
    /// Replaces the leaf's parent with its sibling and refits up to the root.
    void remove_leaf(NodeId leaf);
    /// Rotates a node whose children differ in height by more than one;
    /// returns the node now in its place.
    [[nodiscard]] NodeId balance(NodeId id);
    /// Node by id.
    [[nodiscard]] Node& node(NodeId id) noexcept {
        return nodes_[static_cast<std::size_t>(id)];
    }
    /// Node by id.
    [[nodiscard]] const Node& node(NodeId id) const noexcept {
        return nodes_[static_cast<std::size_t>(id)];
    }

    std::vector<Node> nodes_;
    std::vector<NodeId> free_list_;
    NodeId root_{null_node};
    std::size_t proxy_count_{};
};

} // namespace bench::phys
