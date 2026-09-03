//! Voxel raycast: 100k rays walked cell by cell through a 128^3 occupancy
//! bitset until they hit a solid cell or leave the grid.
const std = @import("std");
const hash = @import("hash.zig");

const Dda = @This();

pub const name = "dda";

const n: u32 = 128;
const rays: usize = 100_000;

words: []u64,

pub fn init(gpa: std.mem.Allocator) !Dda {
    const cells: u32 = n * n * n;
    const words = try gpa.alloc(u64, cells / 64);
    @memset(words, 0);
    for (0..cells) |i| {
        if ((hash.mix64(i) & 7) == 0) words[i >> 6] |= @as(u64, 1) << @intCast(i & 63);
    }
    return .{ .words = words };
}

pub fn deinit(self: *Dda, gpa: std.mem.Allocator) void {
    gpa.free(self.words);
}

inline fn solid(self: *const Dda, x: i32, y: i32, z: i32) bool {
    const i = (@as(u32, @bitCast(x)) * n + @as(u32, @bitCast(y))) * n + @as(u32, @bitCast(z));
    return (self.words[i >> 6] >> @intCast(i & 63)) & 1 != 0;
}

pub fn run(self: *Dda) u64 {
    var rng: hash.Rng = .{ .s = 0x1234 };
    var h: u64 = 0;
    for (0..rays) |_| {
        const ox = rng.unit() * n;
        const oy = rng.unit() * n;
        const oz = rng.unit() * n;
        var dx = rng.unit() * 2.0 - 1.0;
        var dy = rng.unit() * 2.0 - 1.0;
        var dz = rng.unit() * 2.0 - 1.0;
        var len = @sqrt(dx * dx + dy * dy + dz * dz);
        if (len < 1e-3) {
            dx = 1.0;
            dy = 0.0;
            dz = 0.0;
            len = 1.0;
        }
        dx /= len;
        dy /= len;
        dz /= len;
        if (@abs(dx) < 1e-6) dx = 1e-6;
        if (@abs(dy) < 1e-6) dy = 1e-6;
        if (@abs(dz) < 1e-6) dz = 1e-6;

        var ix: i32 = @intFromFloat(ox);
        var iy: i32 = @intFromFloat(oy);
        var iz: i32 = @intFromFloat(oz);
        const sx: i32 = if (dx > 0) 1 else -1;
        const sy: i32 = if (dy > 0) 1 else -1;
        const sz: i32 = if (dz > 0) 1 else -1;
        const invx = 1.0 / dx;
        const invy = 1.0 / dy;
        const invz = 1.0 / dz;
        const tdx = @abs(invx);
        const tdy = @abs(invy);
        const tdz = @abs(invz);
        var tx = if (dx > 0) (@as(f32, @floatFromInt(ix + 1)) - ox) * invx else (@as(f32, @floatFromInt(ix)) - ox) * invx;
        var ty = if (dy > 0) (@as(f32, @floatFromInt(iy + 1)) - oy) * invy else (@as(f32, @floatFromInt(iy)) - oy) * invy;
        var tz = if (dz > 0) (@as(f32, @floatFromInt(iz + 1)) - oz) * invz else (@as(f32, @floatFromInt(iz)) - oz) * invz;

        var steps: u32 = 0;
        while (true) {
            if (self.solid(ix, iy, iz)) {
                const cell = (@as(u64, @bitCast(@as(i64, ix))) * n + @as(u64, @bitCast(@as(i64, iy)))) * n + @as(u64, @bitCast(@as(i64, iz)));
                h = hash.add(h, cell | (@as(u64, steps) << 32));
                break;
            }
            if (tx < ty) {
                if (tx < tz) {
                    ix += sx;
                    tx += tdx;
                } else {
                    iz += sz;
                    tz += tdz;
                }
            } else {
                if (ty < tz) {
                    iy += sy;
                    ty += tdy;
                } else {
                    iz += sz;
                    tz += tdz;
                }
            }
            steps += 1;
            if (@as(u32, @bitCast(ix)) >= n or @as(u32, @bitCast(iy)) >= n or @as(u32, @bitCast(iz)) >= n) {
                h = hash.add(h, 0xffffffff ^ @as(u64, steps));
                break;
            }
        }
    }
    return h;
}
