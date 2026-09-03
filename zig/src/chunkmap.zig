//! Chunk lookup: an open-addressing hash map from packed chunk coordinates to
//! slots. 200k inserts, then 2M lookups at a 50% hit rate.
const std = @import("std");
const hash = @import("hash.zig");

const ChunkMap = @This();

pub const name = "chunkmap";

const cap: u32 = 1 << 19;
const inserts: u32 = 200_000;
const lookups: usize = 2_000_000;
const empty: u64 = std.math.maxInt(u64);

keys: []u64,
values: []u32,
coords: []u64,

fn pack(rng: *hash.Rng) u64 {
    const r = rng.next();
    const x = r & 0x3ff;
    const y = (r >> 10) & 0x3ff;
    const z = (r >> 20) & 0x3ff;
    return x | (y << 21) | (z << 42);
}

pub fn init(gpa: std.mem.Allocator) !ChunkMap {
    const keys = try gpa.alloc(u64, cap);
    errdefer gpa.free(keys);
    const values = try gpa.alloc(u32, cap);
    errdefer gpa.free(values);
    const coords = try gpa.alloc(u64, inserts);
    var rng: hash.Rng = .{ .s = 0xc4a4 };
    for (coords) |*c| c.* = pack(&rng);
    return .{ .keys = keys, .values = values, .coords = coords };
}

pub fn deinit(self: *ChunkMap, gpa: std.mem.Allocator) void {
    gpa.free(self.keys);
    gpa.free(self.values);
    gpa.free(self.coords);
}

fn insert(self: *ChunkMap, key: u64, value: u32) void {
    var i: u32 = @truncate(hash.mix64(key) & (cap - 1));
    while (self.keys[i] != empty and self.keys[i] != key) i = (i + 1) & (cap - 1);
    self.keys[i] = key;
    self.values[i] = value;
}

fn lookup(self: *const ChunkMap, key: u64) ?u32 {
    var i: u32 = @truncate(hash.mix64(key) & (cap - 1));
    while (self.keys[i] != empty) {
        if (self.keys[i] == key) return self.values[i];
        i = (i + 1) & (cap - 1);
    }
    return null;
}

pub fn run(self: *ChunkMap) u64 {
    @memset(self.keys, empty);
    for (self.coords, 0..) |c, i| self.insert(c, @intCast(i));

    var rng: hash.Rng = .{ .s = 0x100c };
    var sum: u64 = 0;
    var hits: u64 = 0;
    for (0..lookups) |_| {
        const r = rng.next();
        const key = if (r & 1 != 0) self.coords[(r >> 1) % inserts] else pack(&rng);
        if (self.lookup(key)) |v| {
            sum += v;
            hits += 1;
        }
    }
    return hash.add(sum, hits);
}
