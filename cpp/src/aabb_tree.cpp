#include "aabb_tree.hpp"

#include <algorithm>

namespace bench::phys {

using vm::Aabb;
using vm::Vec3;

AabbTree::NodeId AabbTree::allocate_node() {
    if (free_list_.empty()) {
        nodes_.emplace_back();
        return static_cast<NodeId>(nodes_.size() - 1);
    }
    const NodeId id = free_list_.back();
    free_list_.pop_back();
    node(id) = Node{};
    return id;
}

void AabbTree::free_node(NodeId id) {
    node(id) = Node{};
    node(id).height = -1;
    free_list_.push_back(id);
}

AabbTree::NodeId AabbTree::create_proxy(const Aabb& aabb,
                                        std::uint32_t user_data) {
    const NodeId id = allocate_node();
    Node& leaf = node(id);
    leaf.aabb = aabb;
    leaf.user_data = user_data;
    leaf.height = 0;
    insert_leaf(id);
    ++proxy_count_;
    return id;
}

void AabbTree::destroy_proxy(NodeId proxy) {
    remove_leaf(proxy);
    free_node(proxy);
    --proxy_count_;
}

void AabbTree::move_proxy(NodeId proxy, const Aabb& aabb) {
    remove_leaf(proxy);
    node(proxy).aabb = aabb;
    insert_leaf(proxy);
}

void AabbTree::insert_leaf(NodeId leaf) {
    if (root_ == null_node) {
        root_ = leaf;
        node(root_).parent = null_node;
        return;
    }

    const Aabb leaf_aabb = node(leaf).aabb;
    NodeId index = root_;
    while (!node(index).is_leaf()) {
        const NodeId child1 = node(index).child1;
        const NodeId child2 = node(index).child2;

        const float area = surface_area(node(index).aabb);
        const float combined_area =
            surface_area(merge(node(index).aabb, leaf_aabb));
        const float cost = 2.0F * combined_area;
        const float inheritance_cost = 2.0F * (combined_area - area);

        const auto child_cost = [&](NodeId child) {
            const Node& c = node(child);
            const float merged = surface_area(merge(leaf_aabb, c.aabb));
            if (c.is_leaf()) {
                return merged + inheritance_cost;
            }
            return merged - surface_area(c.aabb) + inheritance_cost;
        };
        const float cost1 = child_cost(child1);
        const float cost2 = child_cost(child2);

        if (cost < cost1 && cost < cost2) {
            break;
        }
        index = cost1 < cost2 ? child1 : child2;
    }

    const NodeId sibling = index;
    const NodeId old_parent = node(sibling).parent;
    const NodeId new_parent = allocate_node();
    node(new_parent).parent = old_parent;
    node(new_parent).aabb = merge(leaf_aabb, node(sibling).aabb);
    node(new_parent).height = node(sibling).height + 1;

    if (old_parent != null_node) {
        if (node(old_parent).child1 == sibling) {
            node(old_parent).child1 = new_parent;
        } else {
            node(old_parent).child2 = new_parent;
        }
    } else {
        root_ = new_parent;
    }
    node(new_parent).child1 = sibling;
    node(new_parent).child2 = leaf;
    node(sibling).parent = new_parent;
    node(leaf).parent = new_parent;

    index = node(leaf).parent;
    while (index != null_node) {
        index = balance(index);
        const NodeId child1 = node(index).child1;
        const NodeId child2 = node(index).child2;
        node(index).height =
            1 + std::max(node(child1).height, node(child2).height);
        node(index).aabb = merge(node(child1).aabb, node(child2).aabb);
        index = node(index).parent;
    }
}

void AabbTree::remove_leaf(NodeId leaf) {
    if (leaf == root_) {
        root_ = null_node;
        return;
    }
    const NodeId parent = node(leaf).parent;
    const NodeId grand_parent = node(parent).parent;
    const NodeId sibling =
        node(parent).child1 == leaf ? node(parent).child2 : node(parent).child1;

    if (grand_parent != null_node) {
        if (node(grand_parent).child1 == parent) {
            node(grand_parent).child1 = sibling;
        } else {
            node(grand_parent).child2 = sibling;
        }
        node(sibling).parent = grand_parent;
        free_node(parent);

        NodeId index = grand_parent;
        while (index != null_node) {
            index = balance(index);
            const NodeId child1 = node(index).child1;
            const NodeId child2 = node(index).child2;
            node(index).aabb = merge(node(child1).aabb, node(child2).aabb);
            node(index).height =
                1 + std::max(node(child1).height, node(child2).height);
            index = node(index).parent;
        }
    } else {
        root_ = sibling;
        node(sibling).parent = null_node;
        free_node(parent);
    }
}

AabbTree::NodeId AabbTree::balance(NodeId a_id) {
    Node& a = node(a_id);
    if (a.is_leaf() || a.height < 2) {
        return a_id;
    }
    const NodeId b_id = a.child1;
    const NodeId c_id = a.child2;
    Node& b = node(b_id);
    Node& c = node(c_id);
    const std::int32_t imbalance = c.height - b.height;

    if (imbalance > 1) {
        const NodeId f_id = c.child1;
        const NodeId g_id = c.child2;
        Node& f = node(f_id);
        Node& g = node(g_id);

        c.child1 = a_id;
        c.parent = a.parent;
        a.parent = c_id;
        if (c.parent != null_node) {
            if (node(c.parent).child1 == a_id) {
                node(c.parent).child1 = c_id;
            } else {
                node(c.parent).child2 = c_id;
            }
        } else {
            root_ = c_id;
        }
        if (f.height > g.height) {
            c.child2 = f_id;
            a.child2 = g_id;
            g.parent = a_id;
            a.aabb = merge(b.aabb, g.aabb);
            c.aabb = merge(a.aabb, f.aabb);
            a.height = 1 + std::max(b.height, g.height);
            c.height = 1 + std::max(a.height, f.height);
        } else {
            c.child2 = g_id;
            a.child2 = f_id;
            f.parent = a_id;
            a.aabb = merge(b.aabb, f.aabb);
            c.aabb = merge(a.aabb, g.aabb);
            a.height = 1 + std::max(b.height, f.height);
            c.height = 1 + std::max(a.height, g.height);
        }
        return c_id;
    }

    if (imbalance < -1) {
        const NodeId d_id = b.child1;
        const NodeId e_id = b.child2;
        Node& d = node(d_id);
        Node& e = node(e_id);

        b.child1 = a_id;
        b.parent = a.parent;
        a.parent = b_id;
        if (b.parent != null_node) {
            if (node(b.parent).child1 == a_id) {
                node(b.parent).child1 = b_id;
            } else {
                node(b.parent).child2 = b_id;
            }
        } else {
            root_ = b_id;
        }
        if (d.height > e.height) {
            b.child2 = d_id;
            a.child1 = e_id;
            e.parent = a_id;
            a.aabb = merge(c.aabb, e.aabb);
            b.aabb = merge(a.aabb, d.aabb);
            a.height = 1 + std::max(c.height, e.height);
            b.height = 1 + std::max(a.height, d.height);
        } else {
            b.child2 = e_id;
            a.child1 = d_id;
            d.parent = a_id;
            a.aabb = merge(c.aabb, d.aabb);
            b.aabb = merge(a.aabb, e.aabb);
            a.height = 1 + std::max(c.height, d.height);
            b.height = 1 + std::max(a.height, e.height);
        }
        return b_id;
    }
    return a_id;
}

} // namespace bench::phys
