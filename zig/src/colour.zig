//! Graph colouring: assign each of 524288 contacts the lowest colour not used
//! by another contact on either of its dynamic bodies, so every colour is a
//! set of contacts a solver can run in parallel. Bitmask per body, 31 colours
//! plus an overflow bucket.
const std = @import("std");
const hash = @import("hash.zig");

const Colour = @This();

pub const name = "colour";

const body_count: u32 = 65536;
const contact_count: usize = 524288;
const overflow: u32 = 31;

pub const Pair = struct { a: u32, b: u32 };

dynamic: []u8,
contacts: []Pair,
used: []u32,
colour: []u8,

pub fn init(gpa: std.mem.Allocator) !Colour {
    const dynamic = try gpa.alloc(u8, body_count);
    errdefer gpa.free(dynamic);
    const contacts = try gpa.alloc(Pair, contact_count);
    errdefer gpa.free(contacts);
    const used = try gpa.alloc(u32, body_count);
    errdefer gpa.free(used);
    const colour = try gpa.alloc(u8, contact_count);
    for (dynamic, 0..) |*d, i| d.* = @intFromBool((hash.mix64(i ^ 0xc0) & 7) != 0);
    var rng: hash.Rng = .{ .s = 0xc01c };
    for (contacts) |*c| {
        const a: u32 = @intCast(rng.next() % body_count);
        var b: u32 = @intCast(rng.next() % body_count);
        if (b == a) b = (a + 1) % body_count;
        c.* = .{ .a = a, .b = b };
    }
    return .{ .dynamic = dynamic, .contacts = contacts, .used = used, .colour = colour };
}

pub fn deinit(self: *Colour, gpa: std.mem.Allocator) void {
    gpa.free(self.dynamic);
    gpa.free(self.contacts);
    gpa.free(self.used);
    gpa.free(self.colour);
}

pub fn run(self: *Colour) u64 {
    @memset(self.used, 0);
    var highest: u32 = 0;
    for (self.contacts, self.colour) |p, *out| {
        const dyn_a = self.dynamic[p.a] != 0;
        const dyn_b = self.dynamic[p.b] != 0;
        const mask = (if (dyn_a) self.used[p.a] else 0) | (if (dyn_b) self.used[p.b] else 0);
        var c: u32 = @ctz(~mask);
        if (c >= overflow) c = overflow;
        if (dyn_a) self.used[p.a] |= @as(u32, 1) << @intCast(c);
        if (dyn_b) self.used[p.b] |= @as(u32, 1) << @intCast(c);
        out.* = @intCast(c);
        if (c > highest) highest = c;
    }
    var h: u64 = 0;
    var i: usize = 0;
    while (i < contact_count) : (i += 8) {
        var word: u64 = 0;
        for (0..8) |k| word |= @as(u64, self.colour[i + k]) << @intCast(k * 8);
        h = hash.add(h, word);
    }
    return hash.add(h, highest);
}
