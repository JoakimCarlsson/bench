//! Sort-and-sweep broadphase: 16384 boxes sorted on min x with the standard
//! library sort, then swept for overlapping pairs.
const std = @import("std");
const hash = @import("hash.zig");

const Sweep = @This();

pub const name = "sweep";

const box_count: u32 = 16384;

pub const Box = struct { min: [3]f32, max: [3]f32 };

boxes: []Box,
order: []u32,

pub fn init(gpa: std.mem.Allocator) !Sweep {
    const boxes = try gpa.alloc(Box, box_count);
    errdefer gpa.free(boxes);
    const order = try gpa.alloc(u32, box_count);
    var rng: hash.Rng = .{ .s = 0xb0c5 };
    for (boxes) |*b| {
        for (0..3) |k| {
            b.min[k] = rng.unit() * 200.0;
            b.max[k] = b.min[k] + 0.5 + rng.unit() * 3.0;
        }
    }
    return .{ .boxes = boxes, .order = order };
}

pub fn deinit(self: *Sweep, gpa: std.mem.Allocator) void {
    gpa.free(self.boxes);
    gpa.free(self.order);
}

fn byMinX(boxes: []const Box, a: u32, b: u32) bool {
    const xa = boxes[a].min[0];
    const xb = boxes[b].min[0];
    return if (xa != xb) xa < xb else a < b;
}

pub fn run(self: *Sweep) u64 {
    for (self.order, 0..) |*o, i| o.* = @intCast(i);
    std.mem.sortUnstable(u32, self.order, @as([]const Box, self.boxes), byMinX);

    var h: u64 = 0;
    var pairs: u64 = 0;
    for (self.order, 0..) |ia, i| {
        const a = &self.boxes[ia];
        for (self.order[i + 1 ..]) |ib| {
            const b = &self.boxes[ib];
            if (b.min[0] > a.max[0]) break;
            if (b.min[1] > a.max[1] or a.min[1] > b.max[1]) continue;
            if (b.min[2] > a.max[2] or a.min[2] > b.max[2]) continue;
            h = hash.add(h, (@as(u64, ia) << 32) | ib);
            pairs += 1;
        }
    }
    return hash.add(h, pairs);
}
