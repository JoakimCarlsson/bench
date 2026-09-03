//! Chunk occupancy rebuild: for 512 chunks of 32^3 material bytes, rebuild
//! the per-row occupancy bitset and the 4^3-block mip, then popcount both.
const std = @import("std");
const hash = @import("hash.zig");

const Mips = @This();

pub const name = "mips";

const chunks: usize = 512;
const dim: usize = 32;
const voxels: usize = dim * dim * dim;
const row_count: usize = dim * dim;
const mip_dim: usize = dim / 4;
const mip_words: usize = mip_dim * mip_dim * mip_dim / 64;

mat: []u8,
rows: []u32,
mip: []u64,

pub fn init(gpa: std.mem.Allocator) !Mips {
    const mat = try gpa.alloc(u8, chunks * voxels);
    errdefer gpa.free(mat);
    const row_words = try gpa.alloc(u32, chunks * row_count);
    errdefer gpa.free(row_words);
    const mip = try gpa.alloc(u64, chunks * mip_words);
    for (mat, 0..) |*m, i| m.* = @intFromBool((hash.mix64(i ^ 0x3ea) & 3) == 0);
    return .{ .mat = mat, .rows = row_words, .mip = mip };
}

pub fn deinit(self: *Mips, gpa: std.mem.Allocator) void {
    gpa.free(self.mat);
    gpa.free(self.rows);
    gpa.free(self.mip);
}

pub fn run(self: *Mips) u64 {
    var h: u64 = 0;
    for (0..chunks) |c| {
        const mat = self.mat[c * voxels ..][0..voxels];
        const row_words = self.rows[c * row_count ..][0..row_count];
        const mip = self.mip[c * mip_words ..][0..mip_words];
        @memset(mip, 0);
        for (0..dim) |z| {
            for (0..dim) |y| {
                const row = mat[(z * dim + y) * dim ..][0..dim];
                var word: u32 = 0;
                for (row, 0..) |v, x| word |= @as(u32, @intFromBool(v != 0)) << @intCast(x);
                row_words[z * dim + y] = word;
                if (word == 0) continue;
                for (0..mip_dim) |bx| {
                    if (((word >> @intCast(bx * 4)) & 0xf) == 0) continue;
                    const bit = ((z / 4) * mip_dim + (y / 4)) * mip_dim + bx;
                    mip[bit >> 6] |= @as(u64, 1) << @intCast(bit & 63);
                }
            }
        }
        var solid: u32 = 0;
        var blocks: u32 = 0;
        for (row_words) |w| solid += @popCount(w);
        for (mip) |w| blocks += @popCount(w);
        h = hash.add(h, (@as(u64, solid) << 32) | blocks);
    }
    return h;
}
