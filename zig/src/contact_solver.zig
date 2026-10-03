//! The engine's soft step contact solver on eight lanes, with the solver
//! types of solver_types.hpp: bundles each colour of the constraint graph
//! into lanes and runs the stages of every substep, in parallel when given
//! a pool. The overflow colour is not supported.
const std = @import("std");
const vm = @import("vecmath.zig");
const simd = @import("simd.zig");
const contact_types = @import("contact.zig");
const graph = @import("constraint_graph.zig");
const RigidBody = @import("rigid_body.zig").RigidBody;
const TaskPool = @import("task_pool.zig");

const Vec3 = vm.Vec3;
const Quat = vm.Quat;
const Basis = vm.Basis;
const Contact = contact_types.Contact;
const Manifold = contact_types.Manifold;
const Softness = contact_types.Softness;
const Allocator = std.mem.Allocator;
const Atomic = std.atomic.Value;
const F = simd.F8;
const F4 = simd.F4;
const lanes = simd.lanes;
const splat = simd.splat;

const ContactSolver = @This();

const max_points = contact_types.max_manifold_points;
const null_index: u32 = 0xFFFFFFFF;
const pi: f32 = std.math.pi;
const speculative_scale: f32 = 4.0;
const parallel_threshold: usize = 512;
const contacts_per_block: u32 = 16;
const bodies_per_block: u32 = 256;
const max_rotation_per_step: f32 = 0.25 * pi;
const zero_basis: Basis = .{ .x = .{}, .y = .{}, .z = .{} };

/// Position and rotation change of a body over one solve.
pub const BodyDelta = struct {
    position: Vec3 = .{},
    rotation: Quat = .{},
};

/// Step timing, gravity and softness shared by every constraint in a solve.
pub const SolverContext = struct {
    dt: f32 = 0.0,
    inv_dt: f32 = 0.0,
    h: f32 = 0.0,
    inv_h: f32 = 0.0,
    substeps: u32 = 0,
    gravity: Vec3 = .{},
    contact_softness: Softness = .{},
    static_softness: Softness = .{},
    push_out_speed: f32 = 0.0,
    restitution_threshold: f32 = 0.0,
    linear_slop: f32 = 0.0,

    /// The engine's default step: 60 Hz, 4 substeps, soft contacts at 30 Hz.
    pub fn standard() SolverContext {
        var c: SolverContext = .{};
        c.dt = @as(f32, 1.0) / @as(f32, 60.0);
        c.inv_dt = 1.0 / c.dt;
        c.substeps = 4;
        c.h = c.dt / @as(f32, @floatFromInt(c.substeps));
        c.inv_h = 1.0 / c.h;
        c.gravity = Vec3.init(0.0, -9.81, 0.0);
        const contact_hertz = vm.minf(30.0, 0.125 * c.inv_h);
        c.contact_softness = Softness.init(contact_hertz, 10.0, c.h);
        c.static_softness = Softness.init(2.0 * contact_hertz, @as(f32, 0.5) * @as(f32, 10.0), c.h);
        c.push_out_speed = 3.0;
        c.restitution_threshold = 1.0;
        c.linear_slop = 0.005;
        return c;
    }
};

/// Velocity and rotation centre of a static body.
pub const KinematicMotion = struct {
    velocity: Vec3 = .{},
    angular_velocity: Vec3 = .{},
    center: Vec3 = .{},
};

/// A dynamic body taking part in the solve and its slot.
pub const ActiveBody = struct {
    body: *RigidBody,
    slot: u32,
};

/// Everything one solve reads and writes.
pub const SolverInputs = struct {
    contacts: []Contact,
    colors: *const [graph.graph_color_count][]const u32,
    body_local: []const u32,
    active_bodies: []const ActiveBody,
    static_motions: []const KinematicMotion,
    deltas: []BodyDelta,
    context: SolverContext,
    pool: ?*TaskPool = null,
};

/// Per-body velocities and accumulated pose delta, padded to 64 bytes for
/// four-float lane loads.
const BodyState = extern struct {
    velocity: [3]f32 align(16) = .{ 0.0, 0.0, 0.0 },
    pad_velocity: f32 = 0.0,
    angular_velocity: [3]f32 = .{ 0.0, 0.0, 0.0 },
    pad_angular: f32 = 0.0,
    delta_position: [3]f32 = .{ 0.0, 0.0, 0.0 },
    pad_position: f32 = 0.0,
    delta_rotation: [4]f32 = .{ 0.0, 0.0, 0.0, 1.0 },

    comptime {
        std.debug.assert(@sizeOf(BodyState) == 64);
    }
};

/// A Vec3 from three packed floats.
fn vec3Of(a: [3]f32) Vec3 {
    return .{ .x = a[0], .y = a[1], .z = a[2] };
}

/// Three packed floats from a Vec3.
fn arrayOf(v: Vec3) [3]f32 {
    return .{ v.x, v.y, v.z };
}

const BodyProps = struct {
    center: Vec3 = .{},
    inverse_mass: f32 = 0.0,
    inverse_inertia: Basis = zero_basis,
    rotation: Quat = .{},
    force: Vec3 = .{},
    torque: Vec3 = .{},
    linear_damping: f32 = 0.0,
    angular_damping: f32 = 0.0,
};

const StageKind = enum(u8) { integrate_velocities, warm_start, solve_biased, integrate_positions, solve_relax, restitution };

/// A run of blocks of one kind, executed between barriers.
const Stage = struct {
    kind: StageKind = .integrate_velocities,
    color: u32 = 0,
    begin: u32 = 0,
    end: u32 = 0,
    grain: u32 = 0,
    blocks: u32 = 0,
};

const StageProgress = struct {
    next: Atomic(u32) = .init(0),
    done: Atomic(u32) = .init(0),
};

const Entry = struct { contact: u32, body_a: u32, body_b: u32, b_fixed: bool };

/// Zero of a scalar or vector type.
fn zeroOf(comptime T: type) T {
    return if (T == f32) 0.0 else @splat(0.0);
}

/// Symmetric 3x3 matrix stored as its six unique components.
fn Sym3(comptime T: type) type {
    return struct { xx: T = zeroOf(T), xy: T = zeroOf(T), xz: T = zeroOf(T), yy: T = zeroOf(T), yz: T = zeroOf(T), zz: T = zeroOf(T) };
}

/// Three components that are scalars or lanes.
fn V3(comptime T: type) type {
    return struct { x: T = zeroOf(T), y: T = zeroOf(T), z: T = zeroOf(T) };
}

/// Solver data of one contact point.
fn PointConstraint(comptime T: type) type {
    return struct {
        anchor_a: V3(T) = .{},
        anchor_b: V3(T) = .{},
        base_separation: T = zeroOf(T),
        normal_mass: T = zeroOf(T),
        relative_velocity: T = zeroOf(T),
        normal_impulse: T = zeroOf(T),
        total_normal_impulse: T = zeroOf(T),
        peak_normal_impulse: T = zeroOf(T),
        lever_arm: T = zeroOf(T),
    };
}

/// Solver data of one contact manifold, or of one lane per contact in a
/// bundle.
fn ContactConstraint(comptime T: type) type {
    const width = if (T == f32) 1 else lanes;
    return struct {
        body_a: [width]u32 = @splat(0),
        body_b: [width]u32 = @splat(0),
        contact: [width]u32 = @splat(0),
        point_count: u32 = 0,
        inverse_mass_a: T = zeroOf(T),
        inverse_mass_b: T = zeroOf(T),
        inverse_inertia_a: Sym3(T) = .{},
        inverse_inertia_b: Sym3(T) = .{},
        normal: V3(T) = .{},
        tangent1: V3(T) = .{},
        tangent2: V3(T) = .{},
        bias_rate: T = zeroOf(T),
        mass_scale: T = zeroOf(T),
        impulse_scale: T = zeroOf(T),
        friction: T = zeroOf(T),
        restitution: T = zeroOf(T),
        rolling_resistance: T = zeroOf(T),
        points: [max_points]PointConstraint(T) = @splat(.{}),
        friction_anchor_a: V3(T) = .{},
        friction_anchor_b: V3(T) = .{},
        tangent_mass_xx: T = zeroOf(T),
        tangent_mass_xy: T = zeroOf(T),
        tangent_mass_yy: T = zeroOf(T),
        tangent_impulse_x: T = zeroOf(T),
        tangent_impulse_y: T = zeroOf(T),
        twist_mass: T = zeroOf(T),
        twist_impulse: T = zeroOf(T),
        rolling_mass: Sym3(T) = .{},
        rolling_impulse: V3(T) = .{},
    };
}

const Scalar = ContactConstraint(f32);
const Bundle = ContactConstraint(F);
const V3W = V3(F);
const Sym3W = Sym3(F);
const Q4W = struct { x: F, y: F, z: F, w: F };

/// Lane-wise sum.
inline fn add(a: V3W, b: V3W) V3W {
    return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };
}

/// Lane-wise difference.
inline fn sub(a: V3W, b: V3W) V3W {
    return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };
}

/// Scale by a lane-wise scalar.
inline fn scale(a: V3W, s: F) V3W {
    return .{ .x = a.x * s, .y = a.y * s, .z = a.z * s };
}

/// Lane-wise dot product.
inline fn dot(a: V3W, b: V3W) F {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

/// Cross product.
inline fn cross(a: V3W, b: V3W) V3W {
    return .{ .x = a.y * b.z - a.z * b.y, .y = a.z * b.x - a.x * b.z, .z = a.x * b.y - a.y * b.x };
}

/// Symmetric matrix times vector.
inline fn multiply(m: Sym3W, v: V3W) V3W {
    return .{
        .x = m.xx * v.x + m.xy * v.y + m.xz * v.z,
        .y = m.xy * v.x + m.yy * v.y + m.yz * v.z,
        .z = m.xz * v.x + m.yz * v.y + m.zz * v.z,
    };
}

/// Rotate a vector by a unit quaternion.
inline fn rotate(q: Q4W, v: V3W) V3W {
    const axis: V3W = .{ .x = q.x, .y = q.y, .z = q.z };
    const t = scale(cross(axis, v), splat(2.0));
    return add(add(v, scale(t, q.w)), cross(axis, t));
}

/// Body velocities and pose deltas, lane-wise.
const BodyRefs = struct { velocity: V3W, angular_velocity: V3W, delta_position: V3W, delta_rotation: Q4W };
const BodyPair = struct { a: BodyRefs, b: BodyRefs };
const AnchorPair = struct { a: V3W, b: V3W };

/// Loads the bodies named by each lane: four-float rows per body, transposed
/// four lanes at a time and joined.
fn gather(states: []const BodyState, index: [lanes]u32) BodyRefs {
    var columns: [2][4][4]F4 = undefined;
    for (0..2) |half| {
        var rows: [4][4]F4 = undefined;
        for (0..4) |lane| {
            const flat: *const [16]f32 = @ptrCast(&states[index[half * 4 + lane]]);
            for (0..4) |group| rows[group][lane] = flat[4 * group ..][0..4].*;
        }
        for (0..4) |group| columns[half][group] = simd.transpose4(rows[group]);
    }
    const lo = columns[0];
    const hi = columns[1];
    const join = simd.join;
    return .{
        .velocity = .{ .x = join(lo[0][0], hi[0][0]), .y = join(lo[0][1], hi[0][1]), .z = join(lo[0][2], hi[0][2]) },
        .angular_velocity = .{ .x = join(lo[1][0], hi[1][0]), .y = join(lo[1][1], hi[1][1]), .z = join(lo[1][2], hi[1][2]) },
        .delta_position = .{ .x = join(lo[2][0], hi[2][0]), .y = join(lo[2][1], hi[2][1]), .z = join(lo[2][2], hi[2][2]) },
        .delta_rotation = .{ .x = join(lo[3][0], hi[3][0]), .y = join(lo[3][1], hi[3][1]), .z = join(lo[3][2], hi[3][2]), .w = join(lo[3][3], hi[3][3]) },
    };
}

/// Stores velocities back, skipping read-only lanes.
fn scatter(states: []BodyState, writable: u32, index: [lanes]u32, body: *const BodyRefs) void {
    const vx: [lanes]f32 = body.velocity.x;
    const vy: [lanes]f32 = body.velocity.y;
    const vz: [lanes]f32 = body.velocity.z;
    const wx: [lanes]f32 = body.angular_velocity.x;
    const wy: [lanes]f32 = body.angular_velocity.y;
    const wz: [lanes]f32 = body.angular_velocity.z;
    for (index, 0..) |slot, lane| {
        if (slot >= writable) continue;
        const s = &states[slot];
        s.velocity = .{ vx[lane], vy[lane], vz[lane] };
        s.pad_velocity = 0.0;
        s.angular_velocity = .{ wx[lane], wy[lane], wz[lane] };
        s.pad_angular = 0.0;
    }
}

/// Both bodies of every lane.
fn gatherPair(states: []const BodyState, c: *const Bundle) BodyPair {
    return .{ .a = gather(states, c.body_a), .b = gather(states, c.body_b) };
}

/// Stores both bodies of every lane.
fn scatterPair(states: []BodyState, writable: u32, c: *const Bundle, bodies: *const BodyPair) void {
    scatter(states, writable, c.body_a, &bodies.a);
    scatter(states, writable, c.body_b, &bodies.b);
}

/// The two anchors of a contact point.
inline fn pointAnchors(p: *const PointConstraint(F)) AnchorPair {
    return .{ .a = p.anchor_a, .b = p.anchor_b };
}

/// Equal and opposite impulse at a contact anchor.
inline fn applyImpulse(c: *const Bundle, bodies: *BodyPair, r: AnchorPair, impulse: V3W) void {
    bodies.a.velocity = sub(bodies.a.velocity, scale(impulse, c.inverse_mass_a));
    bodies.a.angular_velocity = sub(bodies.a.angular_velocity, multiply(c.inverse_inertia_a, cross(r.a, impulse)));
    bodies.b.velocity = add(bodies.b.velocity, scale(impulse, c.inverse_mass_b));
    bodies.b.angular_velocity = add(bodies.b.angular_velocity, multiply(c.inverse_inertia_b, cross(r.b, impulse)));
}

/// Equal and opposite angular impulse.
inline fn applyAngular(c: *const Bundle, bodies: *BodyPair, impulse: V3W) void {
    bodies.a.angular_velocity = sub(bodies.a.angular_velocity, multiply(c.inverse_inertia_a, impulse));
    bodies.b.angular_velocity = add(bodies.b.angular_velocity, multiply(c.inverse_inertia_b, impulse));
}

/// Relative anchor velocity of B against A along a direction.
inline fn relativeVelocityAlong(bodies: *const BodyPair, r: AnchorPair, direction: V3W) F {
    const va = add(bodies.a.velocity, cross(bodies.a.angular_velocity, r.a));
    const vb = add(bodies.b.velocity, cross(bodies.b.angular_velocity, r.b));
    return dot(sub(vb, va), direction);
}

/// `x` clamped lane-wise to [low, high].
inline fn clampBetween(x: F, low: F, high: F) F {
    return simd.min(simd.max(x, low), high);
}

/// Applies last step's accumulated impulses.
fn warmStartConstraint(c: *Bundle, states: []BodyState, writable: u32) void {
    var bodies = gatherPair(states, c);
    for (c.points[0..c.point_count]) |*p| {
        applyImpulse(c, &bodies, pointAnchors(p), scale(c.normal, p.normal_impulse));
    }
    const tangential = add(scale(c.tangent1, c.tangent_impulse_x), scale(c.tangent2, c.tangent_impulse_y));
    applyImpulse(c, &bodies, .{ .a = c.friction_anchor_a, .b = c.friction_anchor_b }, tangential);
    applyAngular(c, &bodies, add(scale(c.normal, c.twist_impulse), c.rolling_impulse));
    scatterPair(states, writable, c, &bodies);
}

/// Tangential, twist and rolling friction.
fn solveFriction(c: *Bundle, bodies: *BodyPair, total_normal_impulse: F, twist_limit: F) void {
    const zero = splat(0.0);
    const one = splat(1.0);
    const anchors: AnchorPair = .{ .a = c.friction_anchor_a, .b = c.friction_anchor_b };
    const max_friction = c.friction * total_normal_impulse;
    const vt1 = relativeVelocityAlong(bodies, anchors, c.tangent1);
    const vt2 = relativeVelocityAlong(bodies, anchors, c.tangent2);
    const delta_x = -(c.tangent_mass_xx * vt1 + c.tangent_mass_xy * vt2);
    const delta_y = -(c.tangent_mass_xy * vt1 + c.tangent_mass_yy * vt2);
    var total_x = c.tangent_impulse_x + delta_x;
    var total_y = c.tangent_impulse_y + delta_y;
    const magnitude = @sqrt(total_x * total_x + total_y * total_y);
    const clamped = simd.both(magnitude > max_friction, magnitude > zero);
    const s = @select(f32, clamped, max_friction / magnitude, one);
    total_x = total_x * s;
    total_y = total_y * s;
    const impulse = add(scale(c.tangent1, total_x - c.tangent_impulse_x), scale(c.tangent2, total_y - c.tangent_impulse_y));
    c.tangent_impulse_x = total_x;
    c.tangent_impulse_y = total_y;
    applyImpulse(c, bodies, anchors, impulse);

    const max_twist = c.friction * twist_limit;
    const twist_velocity = dot(sub(bodies.b.angular_velocity, bodies.a.angular_velocity), c.normal);
    var twist = c.twist_impulse - c.twist_mass * twist_velocity;
    twist = clampBetween(twist, -max_twist, max_twist);
    const twist_delta = twist - c.twist_impulse;
    c.twist_impulse = twist;
    applyAngular(c, bodies, scale(c.normal, twist_delta));

    const max_rolling = c.rolling_resistance * total_normal_impulse;
    const relative = sub(bodies.b.angular_velocity, bodies.a.angular_velocity);
    var rolling = sub(c.rolling_impulse, multiply(c.rolling_mass, relative));
    const rolling_magnitude = @sqrt(dot(rolling, rolling));
    const rolling_clamped = simd.both(rolling_magnitude > max_rolling, rolling_magnitude > zero);
    const rolling_scale = @select(f32, rolling_clamped, max_rolling / rolling_magnitude, one);
    rolling = scale(rolling, rolling_scale);
    const rolling_delta = sub(rolling, c.rolling_impulse);
    c.rolling_impulse = rolling;
    applyAngular(c, bodies, rolling_delta);
}

/// Normal constraint of every point; friction too on the relax pass.
fn solveConstraint(comptime use_bias: bool, c: *Bundle, states: []BodyState, writable: u32, context: *const SolverContext) void {
    var bodies = gatherPair(states, c);
    const zero = splat(0.0);
    const one = splat(1.0);
    const inv_h = splat(context.inv_h);
    const push_out = splat(-context.push_out_speed);
    const dp = sub(bodies.b.delta_position, bodies.a.delta_position);

    var total_normal_impulse = splat(0.0);
    var total_twist_limit = splat(0.0);
    for (c.points[0..c.point_count]) |*p| {
        const ds = sub(add(dp, rotate(bodies.b.delta_rotation, p.anchor_b)), rotate(bodies.a.delta_rotation, p.anchor_a));
        const s = dot(ds, c.normal) + p.base_separation;
        const separated = s > zero;
        var bias = s * inv_h;
        var mass_scale = one;
        var impulse_scale = zero;
        if (use_bias) {
            const soft_bias = simd.max(c.mass_scale * c.bias_rate * s, push_out);
            bias = @select(f32, separated, bias, soft_bias);
            mass_scale = @select(f32, separated, one, c.mass_scale);
            impulse_scale = @select(f32, separated, zero, c.impulse_scale);
        } else {
            bias = @select(f32, separated, bias, zero);
        }
        const vn = relativeVelocityAlong(&bodies, pointAnchors(p), c.normal);
        var delta_impulse = -p.normal_mass * (mass_scale * vn + bias) - impulse_scale * p.normal_impulse;
        const new_impulse = simd.max(p.normal_impulse + delta_impulse, zero);
        delta_impulse = new_impulse - p.normal_impulse;
        p.normal_impulse = new_impulse;
        p.total_normal_impulse = p.total_normal_impulse + new_impulse;
        p.peak_normal_impulse = simd.max(p.peak_normal_impulse, new_impulse);
        total_normal_impulse = total_normal_impulse + new_impulse;
        total_twist_limit = total_twist_limit + p.lever_arm * new_impulse;
        applyImpulse(c, &bodies, pointAnchors(p), scale(c.normal, delta_impulse));
    }

    if (!use_bias) solveFriction(c, &bodies, total_normal_impulse, total_twist_limit);

    scatterPair(states, writable, c, &bodies);
}

/// Restitution for points that approached faster than the threshold.
fn restitutionConstraint(c: *Bundle, states: []BodyState, writable: u32, context: *const SolverContext) void {
    const zero = splat(0.0);
    const threshold = splat(-context.restitution_threshold);
    const bouncy = c.restitution > zero;
    if (!simd.any(bouncy)) return;
    var bodies = gatherPair(states, c);
    for (c.points[0..c.point_count]) |*p| {
        const active = simd.both(simd.both(bouncy, p.relative_velocity <= threshold), p.total_normal_impulse != zero);
        const vn = relativeVelocityAlong(&bodies, pointAnchors(p), c.normal);
        var impulse = -p.normal_mass * (vn + c.restitution * p.relative_velocity);
        const new_impulse = simd.max(p.normal_impulse + impulse, zero);
        impulse = @select(f32, active, new_impulse - p.normal_impulse, zero);
        p.normal_impulse = @select(f32, active, new_impulse, p.normal_impulse);
        p.total_normal_impulse = p.total_normal_impulse + @select(f32, active, new_impulse, zero);
        p.peak_normal_impulse = @select(f32, active, simd.max(p.peak_normal_impulse, new_impulse), p.peak_normal_impulse);
        applyImpulse(c, &bodies, pointAnchors(p), scale(c.normal, impulse));
    }
    scatterPair(states, writable, c, &bodies);
}

/// Some unit vector perpendicular to the unit vector `n`.
fn perpendicular(n: Vec3) Vec3 {
    if (@abs(n.x) > 0.57735) return Vec3.init(n.y, -n.x, 0.0).normalize();
    return Vec3.init(0.0, n.z, -n.y).normalize();
}

/// Scalar solver vector of a Vec3.
fn pack(v: Vec3) V3(f32) {
    return .{ .x = v.x, .y = v.y, .z = v.z };
}

/// Vec3 of a scalar solver vector.
fn unpack(v: V3(f32)) Vec3 {
    return .{ .x = v.x, .y = v.y, .z = v.z };
}

/// Upper triangle of a symmetric basis.
fn symFromBasis(m: Basis) Sym3(f32) {
    return .{ .xx = m.x.x, .xy = m.x.y, .xz = m.x.z, .yy = m.y.y, .yz = m.y.z, .zz = m.z.z };
}

/// Scalar view of one body while preparing constraints.
const PrepBody = struct { velocity: Vec3, angular_velocity: Vec3, inverse_mass: f32, inverse_inertia: Basis, center: Vec3 };

/// Inverse of the combined inverse mass along a direction at two anchors.
fn effectiveMass(a: PrepBody, b: PrepBody, ra: Vec3, rb: Vec3, direction: Vec3) f32 {
    const rna = ra.cross(direction);
    const rnb = rb.cross(direction);
    const k = a.inverse_mass + b.inverse_mass + rna.dot(a.inverse_inertia.apply(rna)) + rnb.dot(b.inverse_inertia.apply(rnb));
    return if (k > 0.0) 1.0 / k else 0.0;
}

/// Relative anchor velocity of B against A along a direction.
fn prepRelativeVelocity(a: PrepBody, b: PrepBody, ra: Vec3, rb: Vec3, direction: Vec3) f32 {
    const va = a.velocity.add(a.angular_velocity.cross(ra));
    const vb = b.velocity.add(b.angular_velocity.cross(rb));
    return vb.sub(va).dot(direction);
}

/// Friction anchors, lever arms and tangent, twist and rolling masses.
fn prepareFriction(c: *Scalar, manifold: *const Manifold, a: PrepBody, b: PrepBody, tangent1: Vec3, tangent2: Vec3, rolling_resistance: f32, speculative: f32) void {
    var center_a: Vec3 = .{};
    var center_b: Vec3 = .{};
    var total_weight: f32 = 0.0;
    const inv_tau = 1.0 / speculative;
    for (manifold.points[0..manifold.point_count], c.points[0..manifold.point_count]) |point, pc| {
        const weight = vm.clampf(2.0 - point.separation * inv_tau, contact_types.min_friction_weight, 1.0);
        center_a = center_a.add(unpack(pc.anchor_a).scale(weight));
        center_b = center_b.add(unpack(pc.anchor_b).scale(weight));
        total_weight += weight;
    }
    const inv_weight: f32 = if (total_weight > 0.0) 1.0 / total_weight else 0.0;
    const anchor_a = center_a.scale(inv_weight);
    const anchor_b = center_b.scale(inv_weight);
    c.friction_anchor_a = pack(anchor_a);
    c.friction_anchor_b = pack(anchor_b);

    for (c.points[0..manifold.point_count]) |*pc| pc.lever_arm = unpack(pc.anchor_a).sub(anchor_a).length();

    const rta1 = anchor_a.cross(tangent1);
    const rta2 = anchor_a.cross(tangent2);
    const rtb1 = anchor_b.cross(tangent1);
    const rtb2 = anchor_b.cross(tangent2);
    const inv_mass = a.inverse_mass + b.inverse_mass;
    const kxx = inv_mass + rta1.dot(a.inverse_inertia.apply(rta1)) + rtb1.dot(b.inverse_inertia.apply(rtb1));
    const kyy = inv_mass + rta2.dot(a.inverse_inertia.apply(rta2)) + rtb2.dot(b.inverse_inertia.apply(rtb2));
    const kxy = rta1.dot(a.inverse_inertia.apply(rta2)) + rtb1.dot(b.inverse_inertia.apply(rtb2));
    const tangent_det = kxx * kyy - kxy * kxy;
    if (tangent_det != 0.0) {
        const inv = 1.0 / tangent_det;
        c.tangent_mass_xx = kyy * inv;
        c.tangent_mass_xy = -kxy * inv;
        c.tangent_mass_yy = kxx * inv;
    }

    const normal = manifold.normal;
    const angular = a.inverse_inertia.add(b.inverse_inertia);
    const twist = normal.dot(angular.apply(normal));
    c.twist_mass = if (twist > 0.0) 1.0 / twist else 0.0;
    if (rolling_resistance > 0.0 and angular.determinant() > 0.0) {
        c.rolling_mass = symFromBasis(angular.inverse());
    }
}

/// Copies every value field of a scalar constraint into one lane of a
/// bundle, walking both structs field by field at compile time.
fn packLane(target: anytype, source: anytype, lane: usize) void {
    const T = @TypeOf(target.*);
    if (T == F) {
        simd.setLane(target, lane, source);
        return;
    }
    switch (@typeInfo(T)) {
        .@"struct" => |info| inline for (info.fields) |field| {
            if (comptime std.mem.eql(u8, field.name, "body_a") or std.mem.eql(u8, field.name, "body_b") or
                std.mem.eql(u8, field.name, "contact") or std.mem.eql(u8, field.name, "point_count")) continue;
            packLane(&@field(target.*, field.name), @field(source, field.name), lane);
        },
        .array => for (target, source) |*t, s| packLane(t, s, lane),
        else => @compileError("unexpected field type " ++ @typeName(T)),
    }
}

/// Runs `task.range(0, count)` here, or spread over the pool when `parallel`.
fn spread(pool: ?*TaskPool, parallel: bool, count: usize, grain: usize, task: anytype) void {
    if (parallel) {
        pool.?.parallelFor(count, grain, task);
    } else {
        task.range(0, count);
    }
}

states: std.ArrayList(BodyState) = .empty,
props: std.ArrayList(BodyProps) = .empty,
static_lookup: std.ArrayList(u32) = .empty,
writable_count: u32 = 0,
dummy_index: u32 = 0,
entries: std.ArrayList(Entry) = .empty,
entry_colors: std.ArrayList(u8) = .empty,
scalars: std.ArrayList(Scalar) = .empty,
order: std.ArrayList(u32) = .empty,
bundles: std.ArrayList(Bundle) = .empty,
color_start: [graph.graph_color_count + 1]u32 = @splat(0),
color_begin: [graph.graph_color_count]u32 = @splat(0),
color_end: [graph.graph_color_count]u32 = @splat(0),
stages: std.ArrayList(Stage) = .empty,
progress: []StageProgress = &.{},

/// Frees every buffer.
pub fn deinit(self: *ContactSolver, gpa: Allocator) void {
    self.states.deinit(gpa);
    self.props.deinit(gpa);
    self.static_lookup.deinit(gpa);
    self.entries.deinit(gpa);
    self.entry_colors.deinit(gpa);
    self.scalars.deinit(gpa);
    self.order.deinit(gpa);
    self.bundles.deinit(gpa);
    self.stages.deinit(gpa);
    gpa.free(self.progress);
    self.* = .{};
}

/// Solves every contact for one step and writes velocities, poses,
/// impulses and deltas back.
pub fn solve(self: *ContactSolver, gpa: Allocator, inputs: *const SolverInputs) !void {
    try self.buildBodies(gpa, inputs);
    try self.prepareConstraints(gpa, inputs);
    try self.planStages(gpa, &inputs.context);
    self.runStages(&inputs.context, inputs.pool);
    self.storeResults(inputs);
}

/// Replaces a list's contents with `count` copies of `value`.
fn assign(comptime T: type, list: *std.ArrayList(T), gpa: Allocator, count: usize, value: T) Allocator.Error!void {
    list.clearRetainingCapacity();
    try list.appendNTimes(gpa, value, count);
}

/// Fills body states and properties from the active bodies.
fn buildBodies(self: *ContactSolver, gpa: Allocator, inputs: *const SolverInputs) Allocator.Error!void {
    const context = &inputs.context;
    const count: u32 = @intCast(inputs.active_bodies.len);
    self.writable_count = count;
    self.dummy_index = count;
    try assign(BodyState, &self.states, gpa, count + 1, .{});
    try assign(BodyProps, &self.props, gpa, count + 1, .{});
    try assign(u32, &self.static_lookup, gpa, inputs.static_motions.len, null_index);
    for (inputs.active_bodies, self.states.items[0..count], self.props.items[0..count]) |active, *state, *props| {
        const body = active.body;
        state.velocity = arrayOf(body.linear_velocity);
        state.angular_velocity = arrayOf(body.angular_velocity);
        props.center = body.worldCenterOfMass();
        props.inverse_mass = body.inverse_mass;
        props.inverse_inertia = body.inverse_inertia_world;
        props.rotation = body.rotation;
        props.force = body.applied_force.add(body.constant_force).add(context.gravity.scale(body.gravity_scale * body.mass));
        props.torque = body.applied_torque.add(body.constant_torque);
        props.linear_damping = 1.0 / (1.0 + context.h * body.linear_damp);
        props.angular_damping = 1.0 / (1.0 + context.h * body.angular_damp);
    }
}

/// Slot of a static body, created read-only on first use.
fn readOnlyStatic(self: *ContactSolver, gpa: Allocator, inputs: *const SolverInputs, fixed_index: u32) Allocator.Error!u32 {
    if (fixed_index >= inputs.static_motions.len) return self.dummy_index;
    const entry = &self.static_lookup.items[fixed_index];
    if (entry.* == null_index) {
        const motion = inputs.static_motions[fixed_index];
        var state: BodyState = .{};
        state.velocity = arrayOf(motion.velocity);
        state.angular_velocity = arrayOf(motion.angular_velocity);
        entry.* = @intCast(self.states.items.len);
        try self.states.append(gpa, state);
        try self.props.append(gpa, .{ .center = motion.center });
    }
    return entry.*;
}

/// Lists contacts colour by colour with their body slots.
fn collectEntries(self: *ContactSolver, gpa: Allocator, inputs: *const SolverInputs) !void {
    self.entries.clearRetainingCapacity();
    self.entry_colors.clearRetainingCapacity();
    for (inputs.colors, 0..) |list, color| {
        for (list) |index| {
            if (color == graph.overflow_color) {
                std.debug.print("solver: the overflow colour is not supported\n", .{});
                return error.OverflowColor;
            }
            const contact = &inputs.contacts[index];
            const body_a = inputs.body_local[contact.shape_a.body];
            const b_fixed = contact.shape_b.is_static;
            const body_b = if (b_fixed) try self.readOnlyStatic(gpa, inputs, contact.shape_b.body) else inputs.body_local[contact.shape_b.body];
            try self.entries.append(gpa, .{ .contact = index, .body_a = body_a, .body_b = body_b, .b_fixed = b_fixed });
            try self.entry_colors.append(gpa, @intCast(color));
        }
    }
}

/// Scalar view of the body in a slot.
fn prepBody(self: *const ContactSolver, body: u32) PrepBody {
    const props = &self.props.items[body];
    const state = &self.states.items[body];
    return .{
        .velocity = vec3Of(state.velocity),
        .angular_velocity = vec3Of(state.angular_velocity),
        .inverse_mass = props.inverse_mass,
        .inverse_inertia = props.inverse_inertia,
        .center = props.center,
    };
}

/// Builds the scalar constraint of one contact.
fn prepareEntry(self: *const ContactSolver, inputs: *const SolverInputs, entry: Entry, out: *Scalar) void {
    const context = &inputs.context;
    const contact = &inputs.contacts[entry.contact];
    const a = self.prepBody(entry.body_a);
    const b = self.prepBody(entry.body_b);

    var c: Scalar = .{};
    c.body_a = .{entry.body_a};
    c.body_b = .{entry.body_b};
    c.contact = .{entry.contact};
    const manifold = &contact.manifold;
    c.point_count = manifold.point_count;
    const n = manifold.normal;
    const tangent1 = perpendicular(n);
    const tangent2 = tangent1.cross(n);
    c.normal = pack(n);
    c.tangent1 = pack(tangent1);
    c.tangent2 = pack(tangent2);
    const softness = if (entry.b_fixed) context.static_softness else context.contact_softness;
    c.bias_rate = softness.bias_rate;
    c.mass_scale = softness.mass_scale;
    c.impulse_scale = softness.impulse_scale;
    c.friction = contact.friction;
    c.restitution = contact.restitution;
    c.rolling_resistance = contact.rolling_resistance;
    c.inverse_mass_a = a.inverse_mass;
    c.inverse_mass_b = b.inverse_mass;
    c.inverse_inertia_a = symFromBasis(a.inverse_inertia);
    c.inverse_inertia_b = symFromBasis(b.inverse_inertia);

    for (manifold.points[0..manifold.point_count], c.points[0..manifold.point_count]) |*point, *pc| {
        const anchor_a = point.point.sub(a.center);
        const anchor_b = point.point.sub(b.center);
        pc.anchor_a = pack(anchor_a);
        pc.anchor_b = pack(anchor_b);
        pc.base_separation = point.separation - anchor_b.sub(anchor_a).dot(n);
        pc.normal_mass = effectiveMass(a, b, anchor_a, anchor_b, n);
        pc.relative_velocity = prepRelativeVelocity(a, b, anchor_a, anchor_b, n);
        pc.normal_impulse = point.normal_impulse;
        point.relative_velocity = pc.relative_velocity;
    }
    prepareFriction(&c, manifold, a, b, tangent1, tangent2, contact.rolling_resistance, speculative_scale * context.linear_slop);
    const impulses = contact.friction_impulses;
    c.tangent_impulse_x = impulses.tangent_x;
    c.tangent_impulse_y = impulses.tangent_y;
    c.twist_impulse = impulses.twist;
    c.rolling_impulse = pack(impulses.rolling);
    out.* = c;
}

/// Prepares the scalar constraints of a range of entries.
const PrepareTask = struct {
    solver: *ContactSolver,
    inputs: *const SolverInputs,

    /// Prepares entries [begin, end).
    pub fn range(self: *const PrepareTask, begin: usize, end: usize) void {
        const solver = self.solver;
        for (solver.entries.items[begin..end], solver.scalars.items[begin..end]) |entry, *out| solver.prepareEntry(self.inputs, entry, out);
    }
};

/// Prepares every constraint, in parallel when large enough, and bundles
/// them.
fn prepareConstraints(self: *ContactSolver, gpa: Allocator, inputs: *const SolverInputs) !void {
    try self.collectEntries(gpa, inputs);
    try self.scalars.resize(gpa, self.entries.items.len);
    const task: PrepareTask = .{ .solver = self, .inputs = inputs };
    spread(inputs.pool, inputs.pool != null and self.entries.items.len >= parallel_threshold, self.entries.items.len, 128, &task);
    try self.bundleColors(gpa, inputs);
}

/// Packs a range of bundles from the sorted scalar constraints.
const PackTask = struct {
    solver: *ContactSolver,

    /// Packs bundles [begin, end).
    pub fn range(self: *const PackTask, begin: usize, end: usize) void {
        const solver = self.solver;
        for (solver.bundles.items[begin..end], begin..) |*bundle, bundle_index| {
            var color: usize = 0;
            while (solver.color_end[color] <= bundle_index) color += 1;
            const first = solver.color_start[color] + (@as(u32, @intCast(bundle_index)) - solver.color_begin[color]) * lanes;
            const last = @min(first + lanes, solver.color_start[color + 1]);
            var packed_bundle: Bundle = .{};
            packed_bundle.body_a = @splat(solver.dummy_index);
            packed_bundle.body_b = @splat(solver.dummy_index);
            packed_bundle.contact = @splat(null_index);
            for (first..last) |member| {
                const lane = member - first;
                const source = solver.scalars.items[solver.order.items[member]];
                packed_bundle.body_a[lane] = source.body_a[0];
                packed_bundle.body_b[lane] = source.body_b[0];
                packed_bundle.contact[lane] = source.contact[0];
                packed_bundle.point_count = @max(packed_bundle.point_count, source.point_count);
                packLane(&packed_bundle, source, lane);
            }
            bundle.* = packed_bundle;
        }
    }
};

/// Sorts constraints by colour and packs each colour into bundles.
fn bundleColors(self: *ContactSolver, gpa: Allocator, inputs: *const SolverInputs) Allocator.Error!void {
    var counts: [graph.graph_color_count + 1]u32 = @splat(0);
    for (self.entry_colors.items) |color| counts[color + 1] += 1;
    for (0..graph.graph_color_count) |color| counts[color + 1] += counts[color];
    self.color_start = counts;
    try self.order.resize(gpa, self.entries.items.len);
    for (self.entry_colors.items, 0..) |color, i| {
        self.order.items[counts[color]] = @intCast(i);
        counts[color] += 1;
    }

    var bundles: u32 = 0;
    for (0..graph.overflow_color) |color| {
        const members = self.color_start[color + 1] - self.color_start[color];
        self.color_begin[color] = bundles;
        bundles += (members + lanes - 1) / lanes;
        self.color_end[color] = bundles;
    }

    try self.bundles.resize(gpa, bundles);
    const task: PackTask = .{ .solver = self };
    spread(inputs.pool, inputs.pool != null and bundles >= parallel_threshold / lanes, bundles, 32, &task);
}

/// Appends a stage, ignoring empty ranges.
fn addStage(self: *ContactSolver, gpa: Allocator, kind: StageKind, color: u32, begin: u32, end: u32, grain: u32) Allocator.Error!void {
    if (begin == end) return;
    try self.stages.append(gpa, .{
        .kind = kind,
        .color = color,
        .begin = begin,
        .end = end,
        .grain = grain,
        .blocks = (end - begin + grain - 1) / grain,
    });
}

/// Adds one stage per colour for a constraint kind.
fn addConstraintStages(self: *ContactSolver, gpa: Allocator, kind: StageKind) Allocator.Error!void {
    for (0..graph.overflow_color) |color| {
        try self.addStage(gpa, kind, @intCast(color), self.color_begin[color], self.color_end[color], @max(1, contacts_per_block / lanes));
    }
}

/// Builds the stage list for every substep plus restitution.
fn planStages(self: *ContactSolver, gpa: Allocator, context: *const SolverContext) Allocator.Error!void {
    self.stages.clearRetainingCapacity();
    for (0..context.substeps) |_| {
        try self.addStage(gpa, .integrate_velocities, 0, 0, self.writable_count, bodies_per_block);
        try self.addConstraintStages(gpa, .warm_start);
        try self.addConstraintStages(gpa, .solve_biased);
        try self.addStage(gpa, .integrate_positions, 0, 0, self.writable_count, bodies_per_block);
        try self.addConstraintStages(gpa, .solve_relax);
    }
    try self.addConstraintStages(gpa, .restitution);
    if (self.progress.len < self.stages.items.len) {
        const progress = try gpa.alloc(StageProgress, self.stages.items.len * 2);
        gpa.free(self.progress);
        self.progress = progress;
    }
    for (self.progress[0..self.stages.items.len]) |*progress| {
        progress.next.store(0, .monotonic);
        progress.done.store(0, .monotonic);
    }
}

/// Velocity integration under gravity and damping for slots [begin, end).
fn integrateVelocities(self: *ContactSolver, begin: u32, end: u32, context: *const SolverContext) void {
    const h = context.h;
    for (self.states.items[begin..end], self.props.items[begin..end]) |*state, props| {
        const v = vec3Of(state.velocity);
        const w = vec3Of(state.angular_velocity);
        state.velocity = arrayOf(v.add(props.force.scale(props.inverse_mass * h)).scale(props.linear_damping));
        state.angular_velocity = arrayOf(w.add(props.inverse_inertia.apply(props.torque).scale(h)).scale(props.angular_damping));
    }
}

/// Position and rotation integration with the per-step rotation cap for
/// slots [begin, end).
fn integratePositions(self: *ContactSolver, begin: u32, end: u32, context: *const SolverContext) void {
    const h = context.h;
    const max_angular_speed = max_rotation_per_step * context.inv_dt;
    for (self.states.items[begin..end], self.props.items[begin..end]) |*state, *props| {
        var w = vec3Of(state.angular_velocity);
        const angular_speed = w.length();
        if (angular_speed > max_angular_speed) {
            w = w.scale(max_angular_speed / angular_speed);
            state.angular_velocity = arrayOf(w);
        }
        const step = vec3Of(state.velocity).scale(h);
        const turn = w.scale(h);
        props.center = props.center.add(step);
        props.rotation = props.rotation.integrate(turn);
        state.delta_position = arrayOf(vec3Of(state.delta_position).add(step));
        const dr = state.delta_rotation;
        const q = (Quat{ .x = dr[0], .y = dr[1], .z = dr[2], .w = dr[3] }).integrate(turn);
        state.delta_rotation = .{ q.x, q.y, q.z, q.w };
    }
}

/// Executes one block of a stage.
fn runBlock(self: *ContactSolver, stage: *const Stage, block: u32, context: *const SolverContext) void {
    const begin = stage.begin + block * stage.grain;
    const end = @min(stage.end, begin + stage.grain);
    const states = self.states.items;
    const writable = self.writable_count;
    switch (stage.kind) {
        .integrate_velocities => self.integrateVelocities(begin, end, context),
        .integrate_positions => self.integratePositions(begin, end, context),
        .warm_start => for (self.bundles.items[begin..end]) |*c| warmStartConstraint(c, states, writable),
        .solve_biased => for (self.bundles.items[begin..end]) |*c| solveConstraint(true, c, states, writable, context),
        .solve_relax => for (self.bundles.items[begin..end]) |*c| solveConstraint(false, c, states, writable, context),
        .restitution => for (self.bundles.items[begin..end]) |*c| restitutionConstraint(c, states, writable, context),
    }
}

/// Runs the stage list across the pool: worker 0 steps through the stages
/// and waits for each to finish, the others join whichever stage is current.
const StageRunner = struct {
    solver: *ContactSolver,
    context: *const SolverContext,
    current: Atomic(u32) = .init(0),

    /// Claims and runs blocks of one stage until none are left.
    fn workStage(self: *StageRunner, index: u32) void {
        const solver = self.solver;
        const stage = &solver.stages.items[index];
        const progress = &solver.progress[index];
        while (true) {
            const block = progress.next.fetchAdd(1, .monotonic);
            if (block >= stage.blocks) return;
            solver.runBlock(stage, block, self.context);
            _ = progress.done.fetchAdd(1, .release);
        }
    }

    /// One thread's share of every stage.
    pub fn work(self: *StageRunner, worker: u32) void {
        const solver = self.solver;
        const stage_count: u32 = @intCast(solver.stages.items.len);
        if (worker == 0) {
            for (0..stage_count) |i| {
                const index: u32 = @intCast(i);
                if (index != 0) self.current.store(index, .release);
                self.workStage(index);
                while (solver.progress[index].done.load(.acquire) != solver.stages.items[index].blocks) TaskPool.relax();
            }
            self.current.store(stage_count, .release);
            return;
        }
        var seen = null_index;
        while (true) {
            const index = self.current.load(.acquire);
            if (index >= stage_count) return;
            if (index == seen) {
                TaskPool.relax();
                continue;
            }
            seen = index;
            self.workStage(index);
        }
    }
};

/// Runs all stages, spread over the pool when there are enough contacts.
fn runStages(self: *ContactSolver, context: *const SolverContext, pool: ?*TaskPool) void {
    const parallel = pool != null and pool.?.threadCount() > 1 and self.entries.items.len >= parallel_threshold;
    if (!parallel) {
        for (self.stages.items) |*stage| {
            for (0..stage.blocks) |block| self.runBlock(stage, @intCast(block), context);
        }
        return;
    }
    var runner: StageRunner = .{ .solver = self, .context = context };
    pool.?.run(&runner);
}

/// Writes a range of bundles' impulses back to their contacts.
const StoreContactsTask = struct {
    solver: *const ContactSolver,
    inputs: *const SolverInputs,

    /// Stores bundles [begin, end).
    pub fn range(self: *const StoreContactsTask, begin: usize, end: usize) void {
        for (self.solver.bundles.items[begin..end]) |*c| {
            for (c.contact, 0..) |index, lane| {
                if (index == null_index) continue;
                const contact = &self.inputs.contacts[index];
                const manifold = &contact.manifold;
                for (manifold.points[0..manifold.point_count], c.points[0..manifold.point_count]) |*point, *p| {
                    point.normal_impulse = simd.getLane(&p.normal_impulse, lane);
                    point.total_normal_impulse = simd.getLane(&p.total_normal_impulse, lane);
                    point.peak_normal_impulse = simd.getLane(&p.peak_normal_impulse, lane);
                }
                const friction = &contact.friction_impulses;
                friction.tangent_x = simd.getLane(&c.tangent_impulse_x, lane);
                friction.tangent_y = simd.getLane(&c.tangent_impulse_y, lane);
                friction.twist = simd.getLane(&c.twist_impulse, lane);
                friction.rolling = Vec3.init(simd.getLane(&c.rolling_impulse.x, lane), simd.getLane(&c.rolling_impulse.y, lane), simd.getLane(&c.rolling_impulse.z, lane));
            }
        }
    }
};

/// Writes a range of bodies' velocities, poses and deltas back.
const StoreBodiesTask = struct {
    solver: *const ContactSolver,
    inputs: *const SolverInputs,

    /// Stores bodies [begin, end).
    pub fn range(self: *const StoreBodiesTask, begin: usize, end: usize) void {
        const solver = self.solver;
        for (begin..end) |i| {
            const state = &solver.states.items[i];
            const props = &solver.props.items[i];
            const body = self.inputs.active_bodies[i].body;
            body.linear_velocity = vec3Of(state.velocity);
            body.angular_velocity = vec3Of(state.angular_velocity);
            body.setPose(props.center, props.rotation);
            const dr = state.delta_rotation;
            self.inputs.deltas[i] = .{ .position = vec3Of(state.delta_position), .rotation = .{ .x = dr[0], .y = dr[1], .z = dr[2], .w = dr[3] } };
        }
    }
};

/// Writes impulses, velocities, poses and deltas back.
fn storeResults(self: *ContactSolver, inputs: *const SolverInputs) void {
    const parallel = inputs.pool != null and self.entries.items.len >= parallel_threshold;
    const contacts_task: StoreContactsTask = .{ .solver = self, .inputs = inputs };
    spread(inputs.pool, parallel, self.bundles.items.len, 64, &contacts_task);
    const bodies_task: StoreBodiesTask = .{ .solver = self, .inputs = inputs };
    spread(inputs.pool, parallel, self.writable_count, 128, &bodies_task);
}
