//! Contact solver: 8 frames of substepped sequential impulses over 4096
//! bodies and 16384 contacts, with accumulated-impulse clamping. Float math
//! through indirect body indices, the shape of a rigid-body solve loop.
const std = @import("std");
const hash = @import("hash.zig");

const Solve = @This();

pub const name = "solve";

const body_count: usize = 4096;
const contact_count: usize = 16384;
const substeps: usize = 8;
const iters: usize = 4;
const frames: usize = 8;

pub const Body = struct { px: f32, py: f32, pz: f32, vx: f32, vy: f32, vz: f32, inv_mass: f32 };
pub const Contact = struct { a: u32, b: u32, nx: f32, ny: f32, nz: f32, depth: f32, impulse: f32 };

bodies: []Body,
initial: []Body,
contacts: []Contact,

pub fn init(gpa: std.mem.Allocator) !Solve {
    const bodies = try gpa.alloc(Body, body_count);
    errdefer gpa.free(bodies);
    const initial = try gpa.alloc(Body, body_count);
    errdefer gpa.free(initial);
    const contacts = try gpa.alloc(Contact, contact_count);

    var rng: hash.Rng = .{ .s = 0xb0d1e5 };
    for (initial, 0..) |*b, i| {
        b.px = rng.unit() * 64.0;
        b.py = rng.unit() * 64.0;
        b.pz = rng.unit() * 64.0;
        b.vx = rng.unit() * 2.0 - 1.0;
        b.vy = rng.unit() * 2.0 - 1.0;
        b.vz = rng.unit() * 2.0 - 1.0;
        b.inv_mass = if (i % 8 == 0) 0.0 else 1.0 / (0.5 + rng.unit() * 2.0);
    }
    for (contacts) |*c| {
        c.a = @intCast(rng.next() % body_count);
        c.b = @intCast(rng.next() % body_count);
        if (c.b == c.a) c.b = (c.a + 1) % @as(u32, body_count);
        var nx = rng.unit() * 2.0 - 1.0;
        var ny = rng.unit() * 2.0 - 1.0;
        var nz = rng.unit() * 2.0 - 1.0;
        var len = @sqrt(nx * nx + ny * ny + nz * nz);
        if (len < 1e-3) {
            nx = 0.0;
            ny = 1.0;
            nz = 0.0;
            len = 1.0;
        }
        c.nx = nx / len;
        c.ny = ny / len;
        c.nz = nz / len;
        c.depth = rng.unit() * 0.05;
        c.impulse = 0.0;
    }
    return .{ .bodies = bodies, .initial = initial, .contacts = contacts };
}

pub fn deinit(self: *Solve, gpa: std.mem.Allocator) void {
    gpa.free(self.bodies);
    gpa.free(self.initial);
    gpa.free(self.contacts);
}

fn integrateGravity(bodies: []Body, h: f32) void {
    for (bodies) |*b| {
        if (b.inv_mass > 0.0) b.vy += -9.81 * h;
    }
}

fn solveContacts(bodies: []Body, contacts: []Contact, h: f32) void {
    for (contacts) |*c| {
        const a = &bodies[c.a];
        const b = &bodies[c.b];
        const k = a.inv_mass + b.inv_mass;
        if (k == 0.0) continue;
        const rvx = b.vx - a.vx;
        const rvy = b.vy - a.vy;
        const rvz = b.vz - a.vz;
        const vn = rvx * c.nx + rvy * c.ny + rvz * c.nz;
        const bias = c.depth * 0.2 / h;
        var lambda = (-vn + bias) / k;
        var acc = c.impulse + lambda;
        if (acc < 0.0) acc = 0.0;
        lambda = acc - c.impulse;
        c.impulse = acc;
        a.vx -= c.nx * lambda * a.inv_mass;
        a.vy -= c.ny * lambda * a.inv_mass;
        a.vz -= c.nz * lambda * a.inv_mass;
        b.vx += c.nx * lambda * b.inv_mass;
        b.vy += c.ny * lambda * b.inv_mass;
        b.vz += c.nz * lambda * b.inv_mass;
    }
}

fn integratePositions(bodies: []Body, h: f32) void {
    for (bodies) |*b| {
        b.px += b.vx * h;
        b.py += b.vy * h;
        b.pz += b.vz * h;
    }
}

fn checksum(bodies: []const Body) u64 {
    var h: u64 = 0;
    for (bodies) |b| {
        h = hash.add(h, @as(u64, hash.f32Bits(b.px)) | (@as(u64, hash.f32Bits(b.vx)) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(b.py)) | (@as(u64, hash.f32Bits(b.vy)) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(b.pz)) | (@as(u64, hash.f32Bits(b.vz)) << 32));
    }
    return h;
}

pub fn run(self: *Solve) u64 {
    @memcpy(self.bodies, self.initial);
    for (self.contacts) |*c| c.impulse = 0.0;
    const h: f32 = @as(f32, 1.0) / @as(f32, 60.0) / @as(f32, substeps);
    for (0..frames) |_| {
        for (0..substeps) |_| {
            integrateGravity(self.bodies, h);
            for (0..iters) |_| solveContacts(self.bodies, self.contacts, h);
            integratePositions(self.bodies, h);
        }
    }
    return checksum(self.bodies);
}
