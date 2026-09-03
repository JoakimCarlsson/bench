//! Rigid-body integration: 32 steps over 65536 bodies, each with a position,
//! a velocity, an angular velocity and a quaternion that is renormalised every
//! step. Streaming float math with no indirection.
const std = @import("std");
const hash = @import("hash.zig");

const Integrate = @This();

pub const name = "integrate";

const body_count: usize = 65536;
const steps: usize = 32;

pub const Body = struct {
    px: f32,
    py: f32,
    pz: f32,
    vx: f32,
    vy: f32,
    vz: f32,
    wx: f32,
    wy: f32,
    wz: f32,
    qx: f32,
    qy: f32,
    qz: f32,
    qw: f32,
};

bodies: []Body,
initial: []Body,

pub fn init(gpa: std.mem.Allocator) !Integrate {
    const bodies = try gpa.alloc(Body, body_count);
    errdefer gpa.free(bodies);
    const initial = try gpa.alloc(Body, body_count);
    var rng: hash.Rng = .{ .s = 0x1a7e };
    for (initial) |*b| {
        b.px = rng.unit() * 100.0;
        b.py = rng.unit() * 100.0;
        b.pz = rng.unit() * 100.0;
        b.vx = rng.unit() * 4.0 - 2.0;
        b.vy = rng.unit() * 4.0 - 2.0;
        b.vz = rng.unit() * 4.0 - 2.0;
        b.wx = rng.unit() * 2.0 - 1.0;
        b.wy = rng.unit() * 2.0 - 1.0;
        b.wz = rng.unit() * 2.0 - 1.0;
        b.qx = 0.0;
        b.qy = 0.0;
        b.qz = 0.0;
        b.qw = 1.0;
    }
    return .{ .bodies = bodies, .initial = initial };
}

pub fn deinit(self: *Integrate, gpa: std.mem.Allocator) void {
    gpa.free(self.bodies);
    gpa.free(self.initial);
}

fn step(bodies: []Body, dt: f32) void {
    const half = 0.5 * dt;
    for (bodies) |*b| {
        b.vy += -9.81 * dt;
        b.px += b.vx * dt;
        b.py += b.vy * dt;
        b.pz += b.vz * dt;
        b.wx *= 0.999;
        b.wy *= 0.999;
        b.wz *= 0.999;
        const qx = b.qx + half * (b.wx * b.qw + b.wy * b.qz - b.wz * b.qy);
        const qy = b.qy + half * (b.wy * b.qw + b.wz * b.qx - b.wx * b.qz);
        const qz = b.qz + half * (b.wz * b.qw + b.wx * b.qy - b.wy * b.qx);
        const qw = b.qw + half * (-b.wx * b.qx - b.wy * b.qy - b.wz * b.qz);
        const inv = 1.0 / @sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
        b.qx = qx * inv;
        b.qy = qy * inv;
        b.qz = qz * inv;
        b.qw = qw * inv;
    }
}

pub fn run(self: *Integrate) u64 {
    @memcpy(self.bodies, self.initial);
    const dt: f32 = @as(f32, 1.0) / @as(f32, 60.0);
    for (0..steps) |_| step(self.bodies, dt);
    var h: u64 = 0;
    for (self.bodies) |b| {
        h = hash.add(h, @as(u64, hash.f32Bits(b.px)) | (@as(u64, hash.f32Bits(b.py)) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(b.qx)) | (@as(u64, hash.f32Bits(b.qw)) << 32));
    }
    return h;
}
