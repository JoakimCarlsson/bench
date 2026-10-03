//! Camera picking: for 131072 cameras build the view from a look-at, the
//! perspective projection and their product, invert it by cofactors, and
//! unproject four NDC points to world space and back.
const std = @import("std");
const hash = @import("hash.zig");
const vm = @import("vecmath.zig");

const Unproject = @This();

pub const name = "unproject";

const camera_count: usize = 131072;

const ndc = [_]vm.Vec3{
    vm.Vec3.init(-0.5, -0.5, -1.0),
    vm.Vec3.init(0.5, -0.5, 0.0),
    vm.Vec3.init(0.5, 0.5, 0.5),
    vm.Vec3.init(-0.25, 0.75, 0.999),
};

pub const Camera = struct {
    eye: vm.Vec3,
    target: vm.Vec3,
    tan_half_fov: f32,
    aspect: f32,
    z_near: f32,
    z_far: f32,

    /// View-projection matrix of the camera.
    fn viewProjection(c: Camera) vm.Mat4 {
        const view = vm.Transform.lookingAt(c.eye, c.target, vm.Vec3.init(0.0, 1.0, 0.0)).inverseOrthonormal().toMat4();
        return vm.Mat4.perspective(c.tan_half_fov, c.aspect, c.z_near, c.z_far).mul(view);
    }
};

cameras: []Camera,

pub fn init(gpa: std.mem.Allocator) !Unproject {
    const cameras = try gpa.alloc(Camera, camera_count);
    var rng: hash.Rng = .{ .s = 0xca3e };
    for (cameras) |*c| {
        c.eye = vm.Vec3.random(&rng, -50.0, 50.0);
        c.target = c.eye.add(vm.Vec3.random(&rng, -10.0, 10.0));
        c.tan_half_fov = 0.3 + rng.unit() * 1.0;
        c.aspect = 1.0 + rng.unit() * 1.0;
        c.z_near = 0.05 + rng.unit() * 0.45;
        c.z_far = 100.0 + rng.unit() * 3900.0;
    }
    return .{ .cameras = cameras };
}

pub fn deinit(self: *Unproject, gpa: std.mem.Allocator) void {
    gpa.free(self.cameras);
}

pub fn run(self: *Unproject) u64 {
    var h: u64 = 0;
    for (self.cameras) |c| {
        const vp = c.viewProjection();
        const inv = vp.inverse();
        for (ndc) |p| {
            const world = inv.transformPoint(p);
            const back = vp.transformPoint(world);
            h = hash.add(h, @as(u64, hash.f32Bits(world.x)) | (@as(u64, hash.f32Bits(world.y)) << 32));
            h = hash.add(h, @as(u64, hash.f32Bits(world.z)) | (@as(u64, hash.f32Bits(back.x)) << 32));
        }
    }
    return h;
}
