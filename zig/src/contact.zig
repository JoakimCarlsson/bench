//! The engine's contact.hpp and the pieces of broad_phase.hpp it needs: a
//! contact between two box shapes, its manifold, friction state, cached poses
//! and the links that place it in sets, the graph and islands.
const std = @import("std");
const vm = @import("vecmath.zig");

const Vec3 = vm.Vec3;

pub const max_manifold_points = 4;
pub const null_link: u32 = 0xFFFFFFFF;
pub const awake_set: u32 = 0;
pub const disabled_set: u32 = 0xFFFFFFFE;
pub const min_friction_weight: f32 = 1e-10;

/// One shape: a rigid body slot, or a static body index.
pub const ShapeRef = struct {
    body: u32 = 0,
    is_static: bool = false,

    /// The shape packed into 32 bits: static flag, body index and shape index 0.
    pub fn pack(ref: ShapeRef) u32 {
        return (if (ref.is_static) @as(u32, 0x80000000) else 0) | ((ref.body & 0x7FFFF) << 12);
    }
};

/// Order-independent key of a shape pair.
pub fn pairKey(a: ShapeRef, b: ShapeRef) u64 {
    const pa = a.pack();
    const pb = b.pack();
    const lo = if (pa < pb) pa else pb;
    const hi = if (pa < pb) pb else pa;
    return (@as(u64, lo) << 32) | hi;
}

pub const ManifoldPoint = struct {
    point: Vec3 = .{},
    separation: f32 = 0.0,
    normal_impulse: f32 = 0.0,
    total_normal_impulse: f32 = 0.0,
    peak_normal_impulse: f32 = 0.0,
    relative_velocity: f32 = 0.0,
    local_a: Vec3 = .{},
    local_b: Vec3 = .{},
    cached_separation: f32 = 0.0,
    feature_id: u32 = 0,
    persisted: bool = false,
};

pub const Manifold = struct {
    normal: Vec3 = .{},
    separating_axis: Vec3 = .{},
    local_normal: Vec3 = .{},
    points: [max_manifold_points]ManifoldPoint = @splat(.{}),
    point_count: u32 = 0,
};

/// Accumulated tangent, twist and rolling friction impulses carried between
/// steps.
pub const FrictionImpulses = struct {
    tangent_x: f32 = 0.0,
    tangent_y: f32 = 0.0,
    twist: f32 = 0.0,
    rolling: Vec3 = .{},
};

/// Soft constraint coefficients.
pub const Softness = struct {
    bias_rate: f32 = 0.0,
    mass_scale: f32 = 1.0,
    impulse_scale: f32 = 0.0,

    /// Coefficients of a spring at `hertz` with `damping_ratio` over substep `h`.
    pub fn init(hertz: f32, damping_ratio: f32, h: f32) Softness {
        if (hertz <= 0.0) return .{ .bias_rate = 0.0, .mass_scale = 1.0, .impulse_scale = 0.0 };
        const omega = @as(f32, 2.0) * @as(f32, std.math.pi) * hertz;
        const a1 = 2.0 * damping_ratio + h * omega;
        const a2 = h * omega * a1;
        const a3 = 1.0 / (1.0 + a2);
        return .{ .bias_rate = omega / a1, .mass_scale = a2 * a3, .impulse_scale = a3 };
    }
};

/// Poses cached to recycle a contact while its bodies barely move.
pub const ContactCache = struct {
    rotation_a: vm.Quat = .{},
    rotation_b: vm.Quat = .{},
    relative_pose: vm.Transform = .{},
    valid: bool = false,
};

pub const Contact = struct {
    shape_a: ShapeRef = .{},
    shape_b: ShapeRef = .{},
    proxy_a: i32 = -1,
    proxy_b: i32 = -1,
    manifold: Manifold = .{},
    friction_impulses: FrictionImpulses = .{},
    cache: ContactCache = .{},
    friction: f32 = 0.0,
    restitution: f32 = 0.0,
    rolling_resistance: f32 = 0.0,
    touching: bool = false,
    was_touching: bool = false,
    linked: bool = false,
    alive: bool = false,
    set: u32 = null_link,
    color: u32 = null_link,
    local: u32 = null_link,
    island: u32 = null_link,
    island_local: u32 = null_link,
    edge_local: [2]u32 = .{ null_link, null_link },
};
