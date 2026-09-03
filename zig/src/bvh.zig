//! Bounding volume hierarchy: build a tree over 16384 boxes by midpoint
//! partition on the longest centroid axis, leaves of at most four, then trace
//! 20k rays through it counting the boxes each one hits.
const std = @import("std");
const hash = @import("hash.zig");

const Bvh = @This();

pub const name = "bvh";

const box_count: u32 = 16384;
const rays: usize = 20_000;
const leaf: u32 = 4;
const max_nodes: u32 = 2 * box_count;
const stack_depth: usize = 64;

pub const Box = struct { min: [3]f32, max: [3]f32 };
pub const Node = struct { min: [3]f32, max: [3]f32, first: u32, count: u32 };

boxes: []Box,
nodes: []Node,
order: []u32,
node_count: u32 = 0,

pub fn init(gpa: std.mem.Allocator) !Bvh {
    const boxes = try gpa.alloc(Box, box_count);
    errdefer gpa.free(boxes);
    const nodes = try gpa.alloc(Node, max_nodes);
    errdefer gpa.free(nodes);
    const order = try gpa.alloc(u32, box_count);
    var rng: hash.Rng = .{ .s = 0xb4 };
    for (boxes) |*b| {
        for (0..3) |k| {
            b.min[k] = rng.unit() * 200.0;
            b.max[k] = b.min[k] + 0.5 + rng.unit() * 3.0;
        }
    }
    return .{ .boxes = boxes, .nodes = nodes, .order = order };
}

pub fn deinit(self: *Bvh, gpa: std.mem.Allocator) void {
    gpa.free(self.boxes);
    gpa.free(self.nodes);
    gpa.free(self.order);
}

inline fn centroid(b: *const Box, axis: usize) f32 {
    return (b.min[axis] + b.max[axis]) * 0.5;
}

fn build(self: *Bvh, node: u32, lo: u32, hi: u32) void {
    const n = &self.nodes[node];
    var cmin: [3]f32 = .{ 1e30, 1e30, 1e30 };
    var cmax: [3]f32 = .{ -1e30, -1e30, -1e30 };
    n.min = .{ 1e30, 1e30, 1e30 };
    n.max = .{ -1e30, -1e30, -1e30 };
    for (self.order[lo..hi]) |ib| {
        const b = &self.boxes[ib];
        for (0..3) |k| {
            if (b.min[k] < n.min[k]) n.min[k] = b.min[k];
            if (b.max[k] > n.max[k]) n.max[k] = b.max[k];
            const c = centroid(b, k);
            if (c < cmin[k]) cmin[k] = c;
            if (c > cmax[k]) cmax[k] = c;
        }
    }
    if (hi - lo <= leaf) {
        n.first = lo;
        n.count = hi - lo;
        return;
    }

    var axis: usize = 0;
    if (cmax[1] - cmin[1] > cmax[axis] - cmin[axis]) axis = 1;
    if (cmax[2] - cmin[2] > cmax[axis] - cmin[axis]) axis = 2;
    const split = (cmin[axis] + cmax[axis]) * 0.5;
    var i = lo;
    var j = hi;
    while (i < j) {
        if (centroid(&self.boxes[self.order[i]], axis) < split) {
            i += 1;
        } else {
            j -= 1;
            std.mem.swap(u32, &self.order[i], &self.order[j]);
        }
    }
    var mid = i;
    if (mid == lo or mid == hi) mid = lo + (hi - lo) / 2;

    n.first = self.node_count;
    n.count = 0;
    self.node_count += 2;
    self.build(n.first, lo, mid);
    self.build(n.first + 1, mid, hi);
}

inline fn slab(min: *const [3]f32, max: *const [3]f32, o: *const [3]f32, inv: *const [3]f32) bool {
    var tmin: f32 = 0.0;
    var tmax: f32 = 1000.0;
    for (0..3) |k| {
        const t1 = (min[k] - o[k]) * inv[k];
        const t2 = (max[k] - o[k]) * inv[k];
        const lo = if (t1 < t2) t1 else t2;
        const hi = if (t1 < t2) t2 else t1;
        if (lo > tmin) tmin = lo;
        if (hi < tmax) tmax = hi;
    }
    return tmax >= tmin;
}

fn trace(self: *const Bvh, o: *const [3]f32, inv: *const [3]f32) u32 {
    var stack: [stack_depth]u32 = undefined;
    var sp: usize = 0;
    var hits: u32 = 0;
    stack[sp] = 0;
    sp += 1;
    while (sp > 0) {
        sp -= 1;
        const n = &self.nodes[stack[sp]];
        if (!slab(&n.min, &n.max, o, inv)) continue;
        if (n.count == 0) {
            stack[sp] = n.first + 1;
            stack[sp + 1] = n.first;
            sp += 2;
            continue;
        }
        for (self.order[n.first .. n.first + n.count]) |ib| {
            const b = &self.boxes[ib];
            hits += @intFromBool(slab(&b.min, &b.max, o, inv));
        }
    }
    return hits;
}

pub fn run(self: *Bvh) u64 {
    for (self.order, 0..) |*o, i| o.* = @intCast(i);
    self.node_count = 1;
    self.build(0, 0, box_count);

    var rng: hash.Rng = .{ .s = 0x7ace };
    var h: u64 = 0;
    var total: u64 = 0;
    for (0..rays) |_| {
        var o: [3]f32 = undefined;
        var d: [3]f32 = undefined;
        var inv: [3]f32 = undefined;
        for (0..3) |k| o[k] = rng.unit() * 200.0;
        for (0..3) |k| d[k] = rng.unit() * 2.0 - 1.0;
        var len = @sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
        if (len < 1e-3) {
            d = .{ 1.0, 0.0, 0.0 };
            len = 1.0;
        }
        for (0..3) |k| {
            d[k] /= len;
            if (@abs(d[k]) < 1e-6) d[k] = 1e-6;
            inv[k] = 1.0 / d[k];
        }
        const hits = self.trace(&o, &inv);
        h = hash.add(h, hits);
        total += hits;
    }
    return hash.add(hash.add(h, total), self.node_count);
}
