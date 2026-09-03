//! Standard-library sort of 2^19 random u64 keys: `qsort`, `std::sort` and
//! `std.mem.sortUnstable`. The one kernel that measures a library rather than
//! the code written here.
const std = @import("std");
const hash = @import("hash.zig");

const Sort = @This();

pub const name = "sort";

const key_count: usize = 1 << 19;

keys: []u64,
initial: []u64,

pub fn init(gpa: std.mem.Allocator) !Sort {
    const keys = try gpa.alloc(u64, key_count);
    errdefer gpa.free(keys);
    const initial = try gpa.alloc(u64, key_count);
    var rng: hash.Rng = .{ .s = 0x5027 };
    for (initial) |*k| k.* = rng.next();
    return .{ .keys = keys, .initial = initial };
}

pub fn deinit(self: *Sort, gpa: std.mem.Allocator) void {
    gpa.free(self.keys);
    gpa.free(self.initial);
}

pub fn run(self: *Sort) u64 {
    @memcpy(self.keys, self.initial);
    std.mem.sortUnstable(u64, self.keys, {}, std.sort.asc(u64));
    var h: u64 = 0;
    var i: usize = 0;
    while (i < key_count) : (i += 977) h = hash.add(h, self.keys[i]);
    return hash.add(h, self.keys[key_count - 1]);
}
