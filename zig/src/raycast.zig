//! Closest-hit ray casts: 1024 rays, each tested against 1024 oriented boxes
//! with the slab test in box space, shrinking the search to the closest hit
//! so far.
const std = @import("std");
const hash = @import("hash.zig");
const vm = @import("vecmath.zig");

const Raycast = @This();

pub const name = "raycast";

const box_count: usize = 1024;
const ray_count: usize = 1024;

pub const Ray = struct { origin: vm.Vec3, translation: vm.Vec3 };
pub const Hit = struct { point: vm.Vec3 = .{}, normal: vm.Vec3 = .{}, fraction: f32 = 0.0 };

boxes: []vm.BoxPose,
rays: []Ray,

pub fn init(gpa: std.mem.Allocator) !Raycast {
    const boxes = try gpa.alloc(vm.BoxPose, box_count);
    errdefer gpa.free(boxes);
    const rays = try gpa.alloc(Ray, ray_count);
    var rng: hash.Rng = .{ .s = 0x7a1c };
    for (boxes) |*b| {
        b.half_extents = vm.Vec3.random(&rng, 0.25, 2.0);
        b.center = vm.Vec3.random(&rng, 0.0, 64.0);
        b.basis = vm.Basis.fromQuat(vm.Quat.random(&rng));
    }
    for (rays) |*r| {
        r.origin = vm.Vec3.random(&rng, -8.0, 72.0);
        r.translation = vm.Vec3.random(&rng, -1.0, 1.0).scale(80.0);
    }
    return .{ .boxes = boxes, .rays = rays };
}

pub fn deinit(self: *Raycast, gpa: std.mem.Allocator) void {
    gpa.free(self.boxes);
    gpa.free(self.rays);
}

/// Slab test of a ray against an oriented box, in the box's frame. A hit
/// needs an entry fraction in (0, max_fraction].
fn rayCastBox(ray: Ray, box: vm.BoxPose, max_fraction: f32) ?Hit {
    const inverse_basis = box.basis.transposed();
    const origin = inverse_basis.apply(ray.origin.sub(box.center));
    const translation = inverse_basis.apply(ray.translation);
    const origins = [3]f32{ origin.x, origin.y, origin.z };
    const translations = [3]f32{ translation.x, translation.y, translation.z };
    const extents = [3]f32{ box.half_extents.x, box.half_extents.y, box.half_extents.z };
    var entry_fraction: f32 = 0.0;
    var exit_fraction = max_fraction;
    var entry_axis: ?usize = null;
    var entry_sign: f32 = 0.0;

    for (origins, translations, extents, 0..) |origin_axis, translation_axis, extent, axis| {
        if (@abs(translation_axis) <= 1e-8) {
            if (origin_axis < -extent or origin_axis > extent) return null;
            continue;
        }
        var first = (-extent - origin_axis) / translation_axis;
        var last = (extent - origin_axis) / translation_axis;
        var normal_sign: f32 = -1.0;
        if (first > last) {
            std.mem.swap(f32, &first, &last);
            normal_sign = 1.0;
        }
        if (first > entry_fraction) {
            entry_fraction = first;
            entry_axis = axis;
            entry_sign = normal_sign;
        }
        exit_fraction = @min(exit_fraction, last);
        if (entry_fraction > exit_fraction) return null;
    }

    const axis = entry_axis orelse return null;
    if (entry_fraction <= 0.0 or entry_fraction > max_fraction) return null;
    var local_normal: vm.Vec3 = .{};
    switch (axis) {
        0 => local_normal.x = entry_sign,
        1 => local_normal.y = entry_sign,
        else => local_normal.z = entry_sign,
    }
    return .{
        .point = ray.origin.add(ray.translation.scale(entry_fraction)),
        .normal = box.basis.apply(local_normal),
        .fraction = entry_fraction,
    };
}

pub fn run(self: *Raycast) u64 {
    var h: u64 = 0;
    for (self.rays) |ray| {
        var closest: Hit = .{ .fraction = 1.0 };
        var closest_box: u32 = std.math.maxInt(u32);
        for (self.boxes, 0..) |box, j| {
            if (rayCastBox(ray, box, closest.fraction)) |hit| {
                closest = hit;
                closest_box = @intCast(j);
            }
        }
        h = hash.add(h, @as(u64, hash.f32Bits(closest.fraction)) | (@as(u64, closest_box) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(closest.point.x)) | (@as(u64, hash.f32Bits(closest.normal.y)) << 32));
    }
    return h;
}
