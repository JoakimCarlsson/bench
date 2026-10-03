//! The engine's dynamic AABB tree: surface-area-heuristic insertion with
//! AVL-style rotations and AABB queries.
const std = @import("std");
const vm = @import("vecmath.zig");

const Aabb = vm.Aabb;
const Allocator = std.mem.Allocator;

const AabbTree = @This();

pub const NodeId = i32;
pub const null_node: NodeId = -1;

const Node = struct {
    aabb: Aabb = .{},
    user_data: u32 = 0,
    parent: NodeId = null_node,
    child1: NodeId = null_node,
    child2: NodeId = null_node,
    height: i32 = 0,

    /// Whether the node has no children.
    fn isLeaf(self: *const Node) bool {
        return self.child1 == null_node;
    }
};

nodes: std.ArrayList(Node) = .empty,
free_list: std.ArrayList(NodeId) = .empty,
root: NodeId = null_node,
proxy_count: usize = 0,

/// Frees the nodes and the free list.
pub fn deinit(self: *AabbTree, gpa: Allocator) void {
    self.nodes.deinit(gpa);
    self.free_list.deinit(gpa);
    self.* = .{};
}

/// Inserts a leaf for `aabb` carrying `user_data`.
pub fn createProxy(self: *AabbTree, gpa: Allocator, aabb: Aabb, user_data: u32) Allocator.Error!NodeId {
    const id = try self.allocateNode(gpa);
    const leaf = self.node(id);
    leaf.aabb = aabb;
    leaf.user_data = user_data;
    leaf.height = 0;
    try self.insertLeaf(gpa, id);
    self.proxy_count += 1;
    return id;
}

/// Removes a leaf and frees its node.
pub fn destroyProxy(self: *AabbTree, gpa: Allocator, proxy: NodeId) Allocator.Error!void {
    try self.removeLeaf(gpa, proxy);
    try self.freeNode(gpa, proxy);
    self.proxy_count -= 1;
}

/// Reinserts a leaf with new bounds.
pub fn moveProxy(self: *AabbTree, gpa: Allocator, proxy: NodeId, aabb: Aabb) Allocator.Error!void {
    try self.removeLeaf(gpa, proxy);
    self.node(proxy).aabb = aabb;
    try self.insertLeaf(gpa, proxy);
}

/// Bounds of a leaf.
pub fn aabbOf(self: *const AabbTree, proxy: NodeId) Aabb {
    return self.nodes.items[@intCast(proxy)].aabb;
}

/// User data of a leaf.
pub fn userData(self: *const AabbTree, proxy: NodeId) u32 {
    return self.nodes.items[@intCast(proxy)].user_data;
}

/// Calls `visitor.visit(id)` with every leaf overlapping `aabb` until it
/// returns false.
pub fn query(self: *const AabbTree, gpa: Allocator, aabb: Aabb, visitor: anytype) Allocator.Error!void {
    var stack: std.ArrayList(NodeId) = try .initCapacity(gpa, 64);
    defer stack.deinit(gpa);
    try stack.append(gpa, self.root);
    while (stack.pop()) |id| {
        if (id == null_node) continue;
        const current = &self.nodes.items[@intCast(id)];
        if (!current.aabb.overlaps(aabb)) continue;
        if (current.isLeaf()) {
            if (!try visitor.visit(id)) return;
        } else {
            try stack.append(gpa, current.child1);
            try stack.append(gpa, current.child2);
        }
    }
}

/// Node by id.
fn node(self: *AabbTree, id: NodeId) *Node {
    return &self.nodes.items[@intCast(id)];
}

/// A node from the free list, or a new one.
fn allocateNode(self: *AabbTree, gpa: Allocator) Allocator.Error!NodeId {
    if (self.free_list.pop()) |id| {
        self.node(id).* = .{};
        return id;
    }
    try self.nodes.append(gpa, .{});
    return @intCast(self.nodes.items.len - 1);
}

/// Returns a node to the free list.
fn freeNode(self: *AabbTree, gpa: Allocator, id: NodeId) Allocator.Error!void {
    self.node(id).* = .{ .height = -1 };
    try self.free_list.append(gpa, id);
}

/// Insertion cost of descending into `child` for a leaf with bounds `leaf_aabb`.
fn childCost(self: *AabbTree, child: NodeId, leaf_aabb: Aabb, inheritance_cost: f32) f32 {
    const c = self.node(child);
    const merged = leaf_aabb.merge(c.aabb).surfaceArea();
    if (c.isLeaf()) return merged + inheritance_cost;
    return merged - c.aabb.surfaceArea() + inheritance_cost;
}

/// Descends by the surface area heuristic, adds a parent over the chosen
/// sibling, and refits and balances up to the root.
fn insertLeaf(self: *AabbTree, gpa: Allocator, leaf: NodeId) Allocator.Error!void {
    if (self.root == null_node) {
        self.root = leaf;
        self.node(leaf).parent = null_node;
        return;
    }

    const leaf_aabb = self.node(leaf).aabb;
    var index = self.root;
    while (!self.node(index).isLeaf()) {
        const child1 = self.node(index).child1;
        const child2 = self.node(index).child2;

        const area = self.node(index).aabb.surfaceArea();
        const combined_area = self.node(index).aabb.merge(leaf_aabb).surfaceArea();
        const cost = 2.0 * combined_area;
        const inheritance_cost = 2.0 * (combined_area - area);

        const cost1 = self.childCost(child1, leaf_aabb, inheritance_cost);
        const cost2 = self.childCost(child2, leaf_aabb, inheritance_cost);

        if (cost < cost1 and cost < cost2) break;
        index = if (cost1 < cost2) child1 else child2;
    }

    const sibling = index;
    const old_parent = self.node(sibling).parent;
    const new_parent = try self.allocateNode(gpa);
    self.node(new_parent).parent = old_parent;
    self.node(new_parent).aabb = leaf_aabb.merge(self.node(sibling).aabb);
    self.node(new_parent).height = self.node(sibling).height + 1;

    if (old_parent != null_node) {
        if (self.node(old_parent).child1 == sibling) {
            self.node(old_parent).child1 = new_parent;
        } else {
            self.node(old_parent).child2 = new_parent;
        }
    } else {
        self.root = new_parent;
    }
    self.node(new_parent).child1 = sibling;
    self.node(new_parent).child2 = leaf;
    self.node(sibling).parent = new_parent;
    self.node(leaf).parent = new_parent;

    index = self.node(leaf).parent;
    while (index != null_node) {
        index = self.balance(index);
        const child1 = self.node(index).child1;
        const child2 = self.node(index).child2;
        self.node(index).height = 1 + @max(self.node(child1).height, self.node(child2).height);
        self.node(index).aabb = self.node(child1).aabb.merge(self.node(child2).aabb);
        index = self.node(index).parent;
    }
}

/// Replaces the leaf's parent with its sibling and refits up to the root.
fn removeLeaf(self: *AabbTree, gpa: Allocator, leaf: NodeId) Allocator.Error!void {
    if (leaf == self.root) {
        self.root = null_node;
        return;
    }
    const parent = self.node(leaf).parent;
    const grand_parent = self.node(parent).parent;
    const sibling = if (self.node(parent).child1 == leaf) self.node(parent).child2 else self.node(parent).child1;

    if (grand_parent != null_node) {
        if (self.node(grand_parent).child1 == parent) {
            self.node(grand_parent).child1 = sibling;
        } else {
            self.node(grand_parent).child2 = sibling;
        }
        self.node(sibling).parent = grand_parent;
        try self.freeNode(gpa, parent);

        var index = grand_parent;
        while (index != null_node) {
            index = self.balance(index);
            const child1 = self.node(index).child1;
            const child2 = self.node(index).child2;
            self.node(index).aabb = self.node(child1).aabb.merge(self.node(child2).aabb);
            self.node(index).height = 1 + @max(self.node(child1).height, self.node(child2).height);
            index = self.node(index).parent;
        }
    } else {
        self.root = sibling;
        self.node(sibling).parent = null_node;
        try self.freeNode(gpa, parent);
    }
}

/// Points the parent of `old_child` at `new_child`, or makes it the root.
fn replaceChild(self: *AabbTree, parent: NodeId, old_child: NodeId, new_child: NodeId) void {
    if (parent != null_node) {
        if (self.node(parent).child1 == old_child) {
            self.node(parent).child1 = new_child;
        } else {
            self.node(parent).child2 = new_child;
        }
    } else {
        self.root = new_child;
    }
}

/// Rotates a node whose children differ in height by more than one;
/// returns the node now in its place.
fn balance(self: *AabbTree, a_id: NodeId) NodeId {
    const a = self.node(a_id);
    if (a.isLeaf() or a.height < 2) return a_id;
    const b_id = a.child1;
    const c_id = a.child2;
    const b = self.node(b_id);
    const c = self.node(c_id);
    const imbalance = c.height - b.height;

    if (imbalance > 1) {
        const f_id = c.child1;
        const g_id = c.child2;
        const f = self.node(f_id);
        const g = self.node(g_id);

        c.child1 = a_id;
        c.parent = a.parent;
        a.parent = c_id;
        self.replaceChild(c.parent, a_id, c_id);
        if (f.height > g.height) {
            c.child2 = f_id;
            a.child2 = g_id;
            g.parent = a_id;
            a.aabb = b.aabb.merge(g.aabb);
            c.aabb = a.aabb.merge(f.aabb);
            a.height = 1 + @max(b.height, g.height);
            c.height = 1 + @max(a.height, f.height);
        } else {
            c.child2 = g_id;
            a.child2 = f_id;
            f.parent = a_id;
            a.aabb = b.aabb.merge(f.aabb);
            c.aabb = a.aabb.merge(g.aabb);
            a.height = 1 + @max(b.height, f.height);
            c.height = 1 + @max(a.height, g.height);
        }
        return c_id;
    }

    if (imbalance < -1) {
        const d_id = b.child1;
        const e_id = b.child2;
        const d = self.node(d_id);
        const e = self.node(e_id);

        b.child1 = a_id;
        b.parent = a.parent;
        a.parent = b_id;
        self.replaceChild(b.parent, a_id, b_id);
        if (d.height > e.height) {
            b.child2 = d_id;
            a.child1 = e_id;
            e.parent = a_id;
            a.aabb = c.aabb.merge(e.aabb);
            b.aabb = a.aabb.merge(d.aabb);
            a.height = 1 + @max(c.height, e.height);
            b.height = 1 + @max(a.height, d.height);
        } else {
            b.child2 = e_id;
            a.child1 = d_id;
            d.parent = a_id;
            a.aabb = c.aabb.merge(d.aabb);
            b.aabb = a.aabb.merge(e.aabb);
            a.height = 1 + @max(c.height, d.height);
            b.height = 1 + @max(a.height, e.height);
        }
        return b_id;
    }
    return a_id;
}
