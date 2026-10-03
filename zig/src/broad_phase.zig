//! The engine's broad phase: static, kinematic and dynamic AABB trees, a
//! move buffer, and pair finding for the proxies that moved.
const std = @import("std");
const vm = @import("vecmath.zig");
const AabbTree = @import("aabb_tree.zig");
const ShapeRef = @import("contact.zig").ShapeRef;

const Aabb = vm.Aabb;
const Allocator = std.mem.Allocator;

const BroadPhase = @This();

pub const max_aabb_margin: f32 = 0.05;
pub const aabb_margin_fraction: f32 = 0.125;

/// Fat margin for a box, the engine's shape_margin.
pub fn shapeMargin(h: vm.Vec3) f32 {
    return vm.minf(max_aabb_margin, aabb_margin_fraction * 2.0 * vm.max3(h.x, h.y, h.z));
}

pub const ProxyId = i32;
pub const null_proxy: ProxyId = -1;

const Proxy = struct {
    ref: ShapeRef = .{},
    fat_aabb: Aabb = .{},
    node: AabbTree.NodeId = AabbTree.null_node,
    is_static: bool = false,
    kinematic: bool = false,
    moved: bool = false,
    alive: bool = false,
};

proxies: std.ArrayList(Proxy) = .empty,
free: std.ArrayList(ProxyId) = .empty,
move_buffer: std.ArrayList(ProxyId) = .empty,
static_tree: AabbTree = .{},
kinematic_tree: AabbTree = .{},
dynamic_tree: AabbTree = .{},

/// Frees the proxies, the buffers and the trees.
pub fn deinit(self: *BroadPhase, gpa: Allocator) void {
    self.proxies.deinit(gpa);
    self.free.deinit(gpa);
    self.move_buffer.deinit(gpa);
    self.static_tree.deinit(gpa);
    self.kinematic_tree.deinit(gpa);
    self.dynamic_tree.deinit(gpa);
    self.* = .{};
}

/// Adds a proxy to the tree for its kind and buffers it as moved.
pub fn createProxy(self: *BroadPhase, gpa: Allocator, ref: ShapeRef, fat_aabb: Aabb, kinematic: bool) Allocator.Error!ProxyId {
    var id: ProxyId = undefined;
    if (self.free.pop()) |reused| {
        id = reused;
    } else {
        try self.proxies.append(gpa, .{});
        id = @intCast(self.proxies.items.len - 1);
    }
    const proxy = self.proxyAt(id);
    proxy.* = .{};
    proxy.ref = ref;
    proxy.fat_aabb = fat_aabb;
    proxy.is_static = ref.is_static;
    proxy.kinematic = kinematic and proxy.is_static;
    proxy.alive = true;
    proxy.node = try self.treeOf(proxy).createProxy(gpa, fat_aabb, @intCast(id));
    try self.bufferMove(gpa, id);
    return id;
}

/// Removes a proxy and frees its id.
pub fn destroyProxy(self: *BroadPhase, gpa: Allocator, id: ProxyId) Allocator.Error!void {
    const proxy = self.proxyAt(id);
    try self.treeOf(proxy).destroyProxy(gpa, proxy.node);
    proxy.* = .{};
    try self.free.append(gpa, id);
}

/// Gives a proxy new fat bounds and buffers it as moved.
pub fn moveProxy(self: *BroadPhase, gpa: Allocator, id: ProxyId, fat_aabb: Aabb) Allocator.Error!void {
    const proxy = self.proxyAt(id);
    proxy.fat_aabb = fat_aabb;
    try self.treeOf(proxy).moveProxy(gpa, proxy.node, fat_aabb);
    try self.bufferMove(gpa, id);
}

/// Fat bounds of a proxy.
pub fn fatAabb(self: *const BroadPhase, id: ProxyId) Aabb {
    return self.proxies.items[@intCast(id)].fat_aabb;
}

/// Shape a proxy stands for.
pub fn shape(self: *const BroadPhase, id: ProxyId) ShapeRef {
    return self.proxies.items[@intCast(id)].ref;
}

/// Number of buffered moves.
pub fn movedCount(self: *const BroadPhase) usize {
    return self.move_buffer.items.len;
}

/// Calls `on_pair.pair(a, b)` once for every new overlap of the moved
/// proxies in [begin, end), dynamic before static and lower id first.
pub fn queryMoved(self: *const BroadPhase, gpa: Allocator, begin: usize, end: usize, on_pair: anytype) Allocator.Error!void {
    for (self.move_buffer.items[begin..end]) |query_id| {
        const query = &self.proxies.items[@intCast(query_id)];
        if (!query.alive) continue;
        var visitor: MovedVisitor(@TypeOf(on_pair)) = .{
            .broad_phase = self,
            .found_tree = &self.dynamic_tree,
            .query = query,
            .query_id = query_id,
            .on_pair = on_pair,
        };
        try self.dynamic_tree.query(gpa, query.fat_aabb, &visitor);
        if (!query.is_static) {
            visitor.found_tree = &self.static_tree;
            try self.static_tree.query(gpa, query.fat_aabb, &visitor);
            visitor.found_tree = &self.kinematic_tree;
            try self.kinematic_tree.query(gpa, query.fat_aabb, &visitor);
        }
    }
}

/// Reports the pairs of every moved proxy, then clears the move buffer.
pub fn updatePairs(self: *BroadPhase, gpa: Allocator, on_pair: anytype) Allocator.Error!void {
    try self.queryMoved(gpa, 0, self.move_buffer.items.len, on_pair);
    self.clearMoves();
}

/// Clears the move buffer and the proxies' moved flags.
pub fn clearMoves(self: *BroadPhase) void {
    for (self.move_buffer.items) |id| self.proxyAt(id).moved = false;
    self.move_buffer.clearRetainingCapacity();
}

/// Tree visitor of one moved proxy, filtering and ordering the pairs it finds.
fn MovedVisitor(comptime OnPair: type) type {
    return struct {
        broad_phase: *const BroadPhase,
        found_tree: *const AabbTree,
        query: *const Proxy,
        query_id: ProxyId,
        on_pair: OnPair,

        /// Reports the pair with one found leaf; always continues the query.
        pub fn visit(self: *@This(), node: AabbTree.NodeId) Allocator.Error!bool {
            const found_id: ProxyId = @intCast(self.found_tree.userData(node));
            if (found_id == self.query_id) return true;
            const found = &self.broad_phase.proxies.items[@intCast(found_id)];
            if (!self.query.is_static and !found.is_static and self.query.ref.body == found.ref.body) return true;
            if (found.moved and found_id < self.query_id) return true;
            const query_first = !self.query.is_static and (found.is_static or self.query_id < found_id);
            if (query_first) {
                try self.on_pair.pair(self.query_id, found_id);
            } else {
                try self.on_pair.pair(found_id, self.query_id);
            }
            return true;
        }
    };
}

/// Proxy by id.
fn proxyAt(self: *BroadPhase, id: ProxyId) *Proxy {
    return &self.proxies.items[@intCast(id)];
}

/// Tree that holds a proxy of this kind.
fn treeOf(self: *BroadPhase, proxy: *const Proxy) *AabbTree {
    if (!proxy.is_static) return &self.dynamic_tree;
    return if (proxy.kinematic) &self.kinematic_tree else &self.static_tree;
}

/// Adds a proxy to the move buffer once.
fn bufferMove(self: *BroadPhase, gpa: Allocator, id: ProxyId) Allocator.Error!void {
    const proxy = self.proxyAt(id);
    if (proxy.moved) return;
    proxy.moved = true;
    try self.move_buffer.append(gpa, id);
}
