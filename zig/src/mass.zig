//! Mass properties: for 128 bodies of 32^3 voxels with per-material density,
//! accumulate mass, centre of mass and the inertia tensor voxel by voxel, then
//! shift the tensor to the centre of mass.
const std = @import("std");
const hash = @import("hash.zig");

const Mass = @This();

pub const name = "mass";

const body_count: usize = 128;
const dim: usize = 32;
const voxels: usize = dim * dim * dim;

const voxel_size: f32 = 0.1;
const density = [16]f32{
    0.0,    2400.0, 700.0, 7800.0, 1600.0, 2500.0, 900.0,  1200.0,
    1800.0, 8900.0, 500.0, 1500.0, 2700.0, 300.0,  1100.0, 2000.0,
};

pub const Props = struct { mass: f32, com: [3]f32, inertia: [6]f32 };

mat: []u8,

pub fn init(gpa: std.mem.Allocator) !Mass {
    const mat = try gpa.alloc(u8, body_count * voxels);
    for (mat, 0..) |*m, i| {
        const r = hash.mix64(i ^ 0x3a55);
        m.* = if ((r & 3) == 0) 0 else @intCast((r >> 2) & 15);
    }
    return .{ .mat = mat };
}

pub fn deinit(self: *Mass, gpa: std.mem.Allocator) void {
    gpa.free(self.mat);
}

fn properties(mat: []const u8) Props {
    var m: f32 = 0.0;
    var cx: f32 = 0.0;
    var cy: f32 = 0.0;
    var cz: f32 = 0.0;
    var ixx: f32 = 0.0;
    var iyy: f32 = 0.0;
    var izz: f32 = 0.0;
    var ixy: f32 = 0.0;
    var ixz: f32 = 0.0;
    var iyz: f32 = 0.0;
    const cube: f32 = voxel_size * voxel_size / 6.0;
    for (0..dim) |z| {
        for (0..dim) |y| {
            for (0..dim) |x| {
                const id = mat[(z * dim + y) * dim + x];
                if (id == 0) continue;
                const dm = density[id] * (voxel_size * voxel_size * voxel_size);
                const px = (@as(f32, @floatFromInt(x)) + 0.5) * voxel_size;
                const py = (@as(f32, @floatFromInt(y)) + 0.5) * voxel_size;
                const pz = (@as(f32, @floatFromInt(z)) + 0.5) * voxel_size;
                m += dm;
                cx += dm * px;
                cy += dm * py;
                cz += dm * pz;
                ixx += dm * (py * py + pz * pz + cube);
                iyy += dm * (px * px + pz * pz + cube);
                izz += dm * (px * px + py * py + cube);
                ixy -= dm * px * py;
                ixz -= dm * px * pz;
                iyz -= dm * py * pz;
            }
        }
    }
    const inv: f32 = if (m > 0.0) 1.0 / m else 0.0;
    const ox = cx * inv;
    const oy = cy * inv;
    const oz = cz * inv;
    return .{
        .mass = m,
        .com = .{ ox, oy, oz },
        .inertia = .{
            ixx - m * (oy * oy + oz * oz),
            iyy - m * (ox * ox + oz * oz),
            izz - m * (ox * ox + oy * oy),
            ixy + m * ox * oy,
            ixz + m * ox * oz,
            iyz + m * oy * oz,
        },
    };
}

pub fn run(self: *Mass) u64 {
    var h: u64 = 0;
    for (0..body_count) |b| {
        const p = properties(self.mat[b * voxels ..][0..voxels]);
        h = hash.add(h, @as(u64, hash.f32Bits(p.mass)) | (@as(u64, hash.f32Bits(p.com[0])) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(p.com[1])) | (@as(u64, hash.f32Bits(p.com[2])) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(p.inertia[0])) | (@as(u64, hash.f32Bits(p.inertia[1])) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(p.inertia[2])) | (@as(u64, hash.f32Bits(p.inertia[3])) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(p.inertia[4])) | (@as(u64, hash.f32Bits(p.inertia[5])) << 32));
    }
    return h;
}
