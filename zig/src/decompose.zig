//! Transform decomposition: for 262144 rotation-and-scale pairs, a quarter of
//! them mirrored, build the basis, recover its scale and rotation through
//! the four-branch basis-to-quaternion conversion, invert it, and rotate a
//! point both ways.
const std = @import("std");
const hash = @import("hash.zig");
const vm = @import("vecmath.zig");

const Decompose = @This();

pub const name = "decompose";

const item_count: usize = 262144;

pub const Item = struct { rotation: vm.Quat, scale: vm.Vec3, point: vm.Vec3 };

items: []Item,

pub fn init(gpa: std.mem.Allocator) !Decompose {
    const items = try gpa.alloc(Item, item_count);
    var rng: hash.Rng = .{ .s = 0xdec0 };
    for (items, 0..) |*it, i| {
        it.rotation = vm.Quat.random(&rng);
        it.scale = vm.Vec3.random(&rng, 0.25, 4.0);
        if (i % 4 == 0) it.scale.x = -it.scale.x;
        it.point = vm.Vec3.random(&rng, -10.0, 10.0);
    }
    return .{ .items = items };
}

pub fn deinit(self: *Decompose, gpa: std.mem.Allocator) void {
    gpa.free(self.items);
}

pub fn run(self: *Decompose) u64 {
    var h: u64 = 0;
    for (self.items) |it| {
        const basis = vm.Basis.fromRotationScale(it.rotation, it.scale);
        const scale = basis.scaleOf();
        const rotation = basis.rotationOf();
        const inv = basis.inverse();
        const rotated = rotation.rotate(it.point);
        const round_trip = inv.apply(basis.apply(it.point));
        h = hash.add(h, @as(u64, hash.f32Bits(scale.x)) | (@as(u64, hash.f32Bits(scale.z)) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(rotation.x)) | (@as(u64, hash.f32Bits(rotation.w)) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(rotated.y)) | (@as(u64, hash.f32Bits(round_trip.z)) << 32));
    }
    return h;
}
