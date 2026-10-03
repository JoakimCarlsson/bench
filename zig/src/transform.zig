//! Scene-graph propagation: 8 frames over a 4-ary tree of 65536 nodes. Each
//! node advances its rotation, builds a scaled local transform, composes it
//! with its parent's world transform, and multiplies the result into a
//! model-view-projection matrix.
const std = @import("std");
const hash = @import("hash.zig");
const vm = @import("vecmath.zig");

const TransformCase = @This();

pub const name = "transform";

const node_count: usize = 65536;
const frames: usize = 8;

pub const Node = struct { position: vm.Vec3, scale: vm.Vec3, spin: vm.Vec3, parent: u32 };

nodes: []Node,
initial: []vm.Quat,
rotations: []vm.Quat,
world: []vm.Transform,
mvp: []vm.Mat4,
view_projection: vm.Mat4,

pub fn init(gpa: std.mem.Allocator) !TransformCase {
    const nodes = try gpa.alloc(Node, node_count);
    errdefer gpa.free(nodes);
    const initial = try gpa.alloc(vm.Quat, node_count);
    errdefer gpa.free(initial);
    const rotations = try gpa.alloc(vm.Quat, node_count);
    errdefer gpa.free(rotations);
    const world = try gpa.alloc(vm.Transform, node_count);
    errdefer gpa.free(world);
    const mvp = try gpa.alloc(vm.Mat4, node_count);

    var rng: hash.Rng = .{ .s = 0x7f0e };
    for (nodes, initial, 0..) |*n, *q, i| {
        n.position = vm.Vec3.random(&rng, -2.0, 2.0);
        n.scale = vm.Vec3.random(&rng, 0.9, 1.1);
        n.spin = vm.Vec3.random(&rng, -0.05, 0.05);
        n.parent = if (i == 0) 0 else @intCast((i - 1) / 4);
        q.* = vm.Quat.random(&rng);
    }
    const eye = vm.Transform.lookingAt(vm.Vec3.init(40.0, 30.0, 40.0), vm.Vec3.init(0.0, 0.0, 0.0), vm.Vec3.init(0.0, 1.0, 0.0));
    const view = eye.inverseOrthonormal().toMat4();
    const aspect: f32 = @as(f32, 16.0) / @as(f32, 9.0);
    return .{
        .nodes = nodes,
        .initial = initial,
        .rotations = rotations,
        .world = world,
        .mvp = mvp,
        .view_projection = vm.Mat4.perspective(0.75, aspect, 0.05, 4000.0).mul(view),
    };
}

pub fn deinit(self: *TransformCase, gpa: std.mem.Allocator) void {
    gpa.free(self.nodes);
    gpa.free(self.initial);
    gpa.free(self.rotations);
    gpa.free(self.world);
    gpa.free(self.mvp);
}

/// Advance every node one frame; parents come before their children.
fn frame(self: *TransformCase) void {
    for (self.nodes, 0..) |n, i| {
        self.rotations[i] = self.rotations[i].integrate(n.spin);
        const local: vm.Transform = .{ .basis = vm.Basis.fromRotationScale(self.rotations[i], n.scale), .origin = n.position };
        self.world[i] = if (i == 0) local else self.world[n.parent].mul(local);
        self.mvp[i] = self.view_projection.mul(self.world[i].toMat4());
    }
}

pub fn run(self: *TransformCase) u64 {
    @memcpy(self.rotations, self.initial);
    for (0..frames) |_| self.frame();
    var h: u64 = 0;
    for (self.mvp) |mvp| {
        const m = &mvp.m;
        h = hash.add(h, @as(u64, hash.f32Bits(m[0])) | (@as(u64, hash.f32Bits(m[5])) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(m[12])) | (@as(u64, hash.f32Bits(m[13])) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(m[14])) | (@as(u64, hash.f32Bits(m[15])) << 32));
    }
    return h;
}
