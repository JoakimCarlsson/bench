//! Box-box narrowphase: 8 frames over 8192 pairs of oriented boxes, a
//! quarter of them stacked face to face. Separating-axis tests over 15 axes,
//! incident-face clipping against the reference face, reduction to four
//! points, and warm starting from the previous frame's manifold.
const std = @import("std");
const hash = @import("hash.zig");
const vm = @import("vecmath.zig");
const contact = @import("contact.zig");
const box_collision = @import("box_collision.zig");

const Vec3 = vm.Vec3;
const BoxPose = vm.BoxPose;

const BoxBox = @This();

pub const name = "boxbox";

const pair_count: usize = 8192;
const frames: usize = 8;

pub const Pair = struct { a: BoxPose, b: BoxPose, velocity: Vec3 };

pairs: []Pair,
manifolds: []contact.Manifold,

/// Draws the box pairs.
pub fn init(gpa: std.mem.Allocator) !BoxBox {
    const pairs = try gpa.alloc(Pair, pair_count);
    errdefer gpa.free(pairs);
    const manifolds = try gpa.alloc(contact.Manifold, pair_count);
    var rng: hash.Rng = .{ .s = 0xb0b0 };
    for (pairs, 0..) |*p, i| {
        const rotation = vm.Quat.random(&rng);
        p.a.half_extents = Vec3.random(&rng, 0.25, 1.5);
        p.a.center = Vec3.random(&rng, 0.0, 1000.0);
        p.a.basis = vm.Basis.fromQuat(rotation);
        p.b.half_extents = Vec3.random(&rng, 0.25, 1.5);
        if (i % 4 == 0) {
            const raw_yaw: vm.Quat = .{ .x = 0.0, .y = rng.unit() - 0.5, .z = 0.0, .w = 1.0 };
            p.b.basis = vm.Basis.fromQuat(rotation.mul(raw_yaw.normalize()));
            const slide = Vec3.random(&rng, -0.3, 0.3);
            const lift = p.a.half_extents.y + p.b.half_extents.y - 0.01;
            p.b.center = p.a.center.add(p.a.basis.y.scale(lift)).add(p.a.basis.x.scale(slide.x).add(p.a.basis.z.scale(slide.z)));
        } else {
            p.b.basis = vm.Basis.fromQuat(vm.Quat.random(&rng));
            const reach = (p.a.half_extents.length() + p.b.half_extents.length()) * 0.6;
            p.b.center = p.a.center.add(Vec3.random(&rng, -1.0, 1.0).scale(reach));
        }
        p.velocity = Vec3.random(&rng, -0.02, 0.02);
    }
    return .{ .pairs = pairs, .manifolds = manifolds };
}

/// Frees the pairs and manifolds.
pub fn deinit(self: *BoxBox, gpa: std.mem.Allocator) void {
    gpa.free(self.pairs);
    gpa.free(self.manifolds);
}


/// Eight frames of collision from empty manifolds, then the checksum.
pub fn run(self: *BoxBox) u64 {
    @memset(self.manifolds, .{});
    for (0..frames) |f| {
        const t: f32 = @floatFromInt(f);
        for (self.pairs, self.manifolds) |*p, *manifold| {
            var b = p.b;
            b.center = b.center.add(p.velocity.scale(t));
            box_collision.collideBoxes(&p.a, &b, .{}, manifold);
        }
    }
    var h: u64 = 0;
    for (self.manifolds) |*m| {
        h = hash.add(h, @as(u64, hash.f32Bits(m.normal.x)) | (@as(u64, hash.f32Bits(m.normal.z)) << 32));
        h = hash.add(h, m.point_count);
        for (m.points[0..m.point_count]) |p| {
            h = hash.add(h, @as(u64, hash.f32Bits(p.point.y)) | (@as(u64, hash.f32Bits(p.separation)) << 32));
            h = hash.add(h, @as(u64, p.feature_id) | (@as(u64, @intFromBool(p.persisted)) << 32));
        }
    }
    return h;
}
