//! Surface extraction: count the exposed faces of every solid voxel in a
//! 128^3 grid at 75% fill, the pass that finds what a raymarcher can see and
//! what a contact can touch.
const std = @import("std");
const hash = @import("hash.zig");

const Surface = @This();

pub const name = "surface";

const n: u32 = 128;
const cells: u32 = n * n * n;

occ: []u8,

pub fn init(gpa: std.mem.Allocator) !Surface {
    const occ = try gpa.alloc(u8, cells);
    for (occ, 0..) |*o, i| o.* = @intFromBool((hash.mix64(i ^ 0xface) & 3) != 0);
    return .{ .occ = occ };
}

pub fn deinit(self: *Surface, gpa: std.mem.Allocator) void {
    gpa.free(self.occ);
}

inline fn emptyAt(occ: []const u8, x: i32, y: i32, z: i32) u32 {
    if (@as(u32, @bitCast(x)) >= n or @as(u32, @bitCast(y)) >= n or @as(u32, @bitCast(z)) >= n) return 1;
    const i = (@as(u32, @bitCast(x)) * n + @as(u32, @bitCast(y))) * n + @as(u32, @bitCast(z));
    return @intFromBool(occ[i] == 0);
}

pub fn run(self: *Surface) u64 {
    var h: u64 = 0;
    var faces_total: u64 = 0;
    var x: i32 = 0;
    while (x < n) : (x += 1) {
        var y: i32 = 0;
        while (y < n) : (y += 1) {
            var z: i32 = 0;
            while (z < n) : (z += 1) {
                const cell = (@as(u32, @bitCast(x)) * n + @as(u32, @bitCast(y))) * n + @as(u32, @bitCast(z));
                if (self.occ[cell] == 0) continue;
                var faces: u32 = 0;
                faces += emptyAt(self.occ, x - 1, y, z);
                faces += emptyAt(self.occ, x + 1, y, z);
                faces += emptyAt(self.occ, x, y - 1, z);
                faces += emptyAt(self.occ, x, y + 1, z);
                faces += emptyAt(self.occ, x, y, z - 1);
                faces += emptyAt(self.occ, x, y, z + 1);
                if (faces == 0) continue;
                h = hash.add(h, (@as(u64, cell) << 3) | faces);
                faces_total += faces;
            }
        }
    }
    return hash.add(h, faces_total);
}
