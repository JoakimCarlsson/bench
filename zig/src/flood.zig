//! Connected components: 6-connected breadth-first labelling of a 128^3
//! occupancy grid at 30% fill, the pass a collapse step runs to find what
//! still holds together.
const std = @import("std");
const hash = @import("hash.zig");

const Flood = @This();

pub const name = "flood";

const n: u32 = 128;
const cells: u32 = n * n * n;

occ: []u8,
label: []u32,
queue: []u32,

pub fn init(gpa: std.mem.Allocator) !Flood {
    const occ = try gpa.alloc(u8, cells);
    errdefer gpa.free(occ);
    const label = try gpa.alloc(u32, cells);
    errdefer gpa.free(label);
    const queue = try gpa.alloc(u32, cells);
    for (occ, 0..) |*o, i| o.* = @intFromBool((hash.mix64(i ^ 0x5eed) & 0xff) < 77);
    return .{ .occ = occ, .label = label, .queue = queue };
}

pub fn deinit(self: *Flood, gpa: std.mem.Allocator) void {
    gpa.free(self.occ);
    gpa.free(self.label);
    gpa.free(self.queue);
}

pub fn run(self: *Flood) u64 {
    @memset(self.label, 0);
    var h: u64 = 0;
    var comp: u32 = 0;
    for (0..cells) |start_usize| {
        const start: u32 = @intCast(start_usize);
        if (self.occ[start] == 0 or self.label[start] != 0) continue;
        comp += 1;
        self.label[start] = comp;
        var head: u32 = 0;
        var tail: u32 = 0;
        var size: u32 = 0;
        self.queue[tail] = start;
        tail += 1;
        while (head < tail) {
            const c = self.queue[head];
            head += 1;
            size += 1;
            const x = c / (n * n);
            const y = (c / n) % n;
            const z = c % n;
            var nb: [6]u32 = undefined;
            var count: usize = 0;
            if (x > 0) {
                nb[count] = c - n * n;
                count += 1;
            }
            if (x + 1 < n) {
                nb[count] = c + n * n;
                count += 1;
            }
            if (y > 0) {
                nb[count] = c - n;
                count += 1;
            }
            if (y + 1 < n) {
                nb[count] = c + n;
                count += 1;
            }
            if (z > 0) {
                nb[count] = c - 1;
                count += 1;
            }
            if (z + 1 < n) {
                nb[count] = c + 1;
                count += 1;
            }
            for (nb[0..count]) |nbr| {
                if (self.occ[nbr] != 0 and self.label[nbr] == 0) {
                    self.label[nbr] = comp;
                    self.queue[tail] = nbr;
                    tail += 1;
                }
            }
        }
        h = hash.add(h, size);
    }
    return hash.add(h, comp);
}
