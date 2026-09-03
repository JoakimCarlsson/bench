//! Island partition: union-find over 524288 contacts between 65536 bodies,
//! joining only pairs where both are dynamic, then labelling every dynamic
//! body by the lowest slot in its island.
const std = @import("std");
const hash = @import("hash.zig");

const Islands = @This();

pub const name = "islands";

const body_count: u32 = 65536;
const contact_count: usize = 524288;

pub const Pair = struct { a: u32, b: u32 };

dynamic: []u8,
contacts: []Pair,
parent: []u32,

pub fn init(gpa: std.mem.Allocator) !Islands {
    const dynamic = try gpa.alloc(u8, body_count);
    errdefer gpa.free(dynamic);
    const contacts = try gpa.alloc(Pair, contact_count);
    errdefer gpa.free(contacts);
    const parent = try gpa.alloc(u32, body_count);
    for (dynamic, 0..) |*d, i| d.* = @intFromBool((hash.mix64(i ^ 0x15) & 7) != 0);
    var rng: hash.Rng = .{ .s = 0x151a };
    for (contacts) |*c| {
        const a: u32 = @intCast(rng.next() % body_count);
        var b: u32 = @intCast(rng.next() % body_count);
        if (b == a) b = (a + 1) % body_count;
        c.* = .{ .a = a, .b = b };
    }
    return .{ .dynamic = dynamic, .contacts = contacts, .parent = parent };
}

pub fn deinit(self: *Islands, gpa: std.mem.Allocator) void {
    gpa.free(self.dynamic);
    gpa.free(self.contacts);
    gpa.free(self.parent);
}

fn find(parent: []u32, start: u32) u32 {
    var i = start;
    while (parent[i] != i) {
        parent[i] = parent[parent[i]];
        i = parent[i];
    }
    return i;
}

pub fn run(self: *Islands) u64 {
    for (self.parent, 0..) |*p, i| p.* = @intCast(i);
    for (self.contacts) |c| {
        if (self.dynamic[c.a] == 0 or self.dynamic[c.b] == 0) continue;
        const ra = find(self.parent, c.a);
        const rb = find(self.parent, c.b);
        if (ra == rb) continue;
        if (ra < rb) self.parent[rb] = ra else self.parent[ra] = rb;
    }
    var h: u64 = 0;
    var islands: u64 = 0;
    for (0..body_count) |i_usize| {
        const i: u32 = @intCast(i_usize);
        if (self.dynamic[i] == 0) continue;
        const root = find(self.parent, i);
        if (root == i) islands += 1;
        h = hash.add(h, (@as(u64, i) << 32) | root);
    }
    return hash.add(h, islands);
}
