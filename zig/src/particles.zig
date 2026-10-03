//! Particle emitters: 12 emitters with 2048-particle pools run 4 updates of
//! 6 substeps each through a port of the engine's `ParticleWorld` step.
//! Each substep integrates every live particle with seeded parameter
//! curves, drag, gravity, wind, curl-noise turbulence and collision against
//! a ground plane and static boxes, emits continuously and in bursts by the
//! effect's phase, and spawns sub emitter particles. Particle colour, size
//! and angle come from gradients and curves each step.
//!
//! Deviations from the engine, all to keep the results bit-identical across
//! languages (only `+ - * /`, `sqrt` and `floor` are allowed on floats):
//! - `sin` and `cos` (spread cone, hue rotation) are Taylor polynomials.
//! - The wind response `1 - exp(-k)` is `k / (1 + k)`.
//! - The turbulence blend `1 - pow(1 - i, s)` is `i * s`.
//! - `fmod(phase, 1)` is `phase - floor(phase)`, which is exact below 2.
//! - Emission shapes are Point and Box; Sphere, SphereSurface and Ring need
//!   `sin`, `cos` and `cbrt`.
//! - The collider is a ground plane and six boxes swept with the slab test
//!   of the engine's grid collider, not a voxel grid walked cell by cell.
//! - Emitters simulate in world space only; local coordinates, the gas
//!   solver, manual spawn requests and the effect field are left out.
//! - The substep count is fixed at 6 instead of `ceil(longest / max_step)`,
//!   and the preprocess is 45 fixed steps run with the collider, not
//!   `ceil(preprocess / step)` steps without one.
const std = @import("std");
const hash = @import("hash.zig");
const vm = @import("vecmath.zig");

const Vec3 = vm.Vec3;
const Basis = vm.Basis;
const Allocator = std.mem.Allocator;

const Particles = @This();

pub const name = "particles";

const emitter_count: u32 = 12;
const pool_size: u32 = 2048;
const sub_pool_size: u32 = 256;
const updates: u32 = 4;
const substeps: u32 = 6;
const preprocess_steps: u32 = 45;
const preprocess_seconds: f32 = 1.5;
const update_seconds: f32 = 0.016666668;
const two_pi: f32 = 6.2831855;
const degrees_to_radians: f32 = 0.017453292;
const minimum_lifetime: f32 = 1e-4;
const contact_offset: f32 = 1e-3;
const impact_speed: f32 = 0.05;
const turbulence_reference_rate: f32 = 60.0;
const wind_response: f32 = 2.0;
const one_third: f32 = 0.33333334;
const golden: u32 = 0x9E3779B9;
const no_effect: u32 = 0xFFFFFFFF;
const curve_capacity = 8;
const gradient_capacity = 6;
const effect_count = 6;
const box_count = 7;
const world_up: Vec3 = .{ .x = 0.0, .y = 1.0, .z = 0.0 };
const zero: Vec3 = .{};

const salt_lifetime: u32 = 1;
const salt_ratio: u32 = 2;
const salt_initial_ramp: u32 = 3;
const salt_phase: u32 = 4;
const salt_shape_a: u32 = 5;
const salt_shape_b: u32 = 6;
const salt_shape_c: u32 = 7;
const salt_direction_a: u32 = 8;
const salt_direction_b: u32 = 9;
const salt_turbulence_speed: u32 = 10;
const salt_parameter_base: u32 = 32;

const Param = enum(u32) {
    initial_velocity,
    angular_velocity,
    orbit_velocity,
    radial_velocity,
    linear_acceleration,
    radial_acceleration,
    tangential_acceleration,
    damping,
    angle,
    scale,
    hue_variation,
    turbulence_influence,
};

const param_count = @typeInfo(Param).@"enum".fields.len;

const CollisionMode = enum { disabled, rigid, hide_on_contact };

const SubMode = enum { disabled, constant, at_end, at_collision };

const Rgba = struct {
    r: f32 = 1.0,
    g: f32 = 1.0,
    b: f32 = 1.0,
    a: f32 = 1.0,
};

const CurvePoint = struct {
    offset: f32 = 0.0,
    value: f32 = 1.0,
};

const Curve = struct {
    smooth: bool = false,
    count: u32 = 0,
    points: [curve_capacity]CurvePoint = [_]CurvePoint{.{}} ** curve_capacity,
};

const Stop = struct {
    offset: f32 = 0.0,
    value: Rgba = .{},
};

const Gradient = struct {
    count: u32 = 0,
    stops: [gradient_capacity]Stop = [_]Stop{.{}} ** gradient_capacity,
};

const Parameter = struct {
    minimum: f32 = 0.0,
    maximum: f32 = 0.0,
    curve: Curve = .{},
};

const Effect = struct {
    amount: u32 = 16,
    lifetime: f32 = 1.0,
    one_shot: bool = false,
    explosiveness: f32 = 0.0,
    randomness: f32 = 0.0,
    lifetime_randomness: f32 = 0.0,
    box_shape: bool = false,
    offset: Vec3 = .{},
    box_extents: Vec3 = .{ .x = 1.0, .y = 1.0, .z = 1.0 },
    direction: Vec3 = .{ .x = 0.0, .y = 1.0, .z = 0.0 },
    spread_degrees: f32 = 45.0,
    flatness: f32 = 0.0,
    inherit_velocity: f32 = 0.0,
    gravity: Vec3 = .{ .x = 0.0, .y = -9.8, .z = 0.0 },
    wind_influence: f32 = 0.0,
    parameters: [param_count]Parameter = unitParameters(),
    color: Rgba = .{},
    color_ramp: Gradient = .{},
    color_initial_ramp: Gradient = .{},
    alpha_curve: Curve = .{},
    turbulence_enabled: bool = false,
    turbulence_strength: f32 = 1.0,
    turbulence_scale: f32 = 4.0,
    turbulence_speed: Vec3 = .{ .x = 0.0, .y = 0.5, .z = 0.0 },
    turbulence_speed_random: f32 = 0.2,
    collision: CollisionMode = .disabled,
    friction: f32 = 0.0,
    bounce: f32 = 0.0,
    use_scale: bool = false,
    sub_mode: SubMode = .disabled,
    sub_frequency: f32 = 4.0,
    sub_amount: u32 = 1,
    sub_keep_velocity: bool = false,
    sub_effect: u32 = no_effect,
    draw_size: f32 = 0.25,
};

const Particle = struct {
    position: Vec3 = .{},
    velocity: Vec3 = .{},
    origin: Vec3 = .{},
    axis: Vec3 = .{ .x = 0.0, .y = 1.0, .z = 0.0 },
    color: Rgba = .{},
    tint: Rgba = .{},
    angle: f32 = 0.0,
    spin: f32 = 0.0,
    size: f32 = 0.0,
    age: f32 = 0.0,
    lifetime: f32 = 0.0,
    sub_timer: f32 = 0.0,
    seed: u32 = 0,
    active: bool = false,
};

const Emitter = struct {
    effect: u32 = 0,
    sub_effect: u32 = no_effect,
    first: u32 = 0,
    sub_first: u32 = 0,
    sub_count: u32 = 0,
    emit_cursor: u32 = 0,
    sub_cursor: u32 = 0,
    bursts: u32 = 0,
    current_seed: u32 = 0,
    burst_pending: u32 = 0,
    alive: u32 = 0,
    cycle: u64 = 0,
    phase: f32 = 0.0,
    elapsed: f32 = 0.0,
    emitting: bool = true,
    amount_ratio: f32 = 1.0,
    speed_scale: f32 = 1.0,
    tint: Rgba = .{},
    basis: Basis = .{},
    origin: Vec3 = .{},
    move: Vec3 = .{},
    velocity: Vec3 = .{},
};

const Scene = struct {
    effects: [effect_count]Effect = undefined,
    boxes: [box_count]vm.Aabb = undefined,
    wind: Vec3 = .{},
};

const Pools = struct {
    main: []Particle,
    sub: []Particle,
};

const Context = struct {
    effect: *const Effect,
    basis: Basis = .{},
    origin: Vec3 = .{},
    emitter_velocity: Vec3 = .{},
    time: f32 = 0.0,
    amount_ratio: f32 = 1.0,
    wind: Vec3 = .{},
    tint: Rgba = .{},
};

const Contact = struct {
    position: Vec3,
    normal: Vec3,
};

const Outcome = struct {
    died: bool = false,
    collided: bool = false,
    where: Vec3 = .{},
    velocity: Vec3 = .{},
};

const SubOrigin = struct {
    position: Vec3,
    velocity: Vec3,
};

const Clip = struct {
    enter: f32 = 0.0,
    exit: f32 = 1.0,
    axis: i32 = -1,
};

gpa: Allocator,
scene: Scene,
collisions: u64,
emitters: []Emitter,
initial_emitters: []Emitter,
particles: []Particle,
initial_particles: []Particle,
sub_particles: []Particle,
initial_sub_particles: []Particle,

/// The parameter table with unit initial velocity and scale.
fn unitParameters() [param_count]Parameter {
    var table = [_]Parameter{.{}} ** param_count;
    table[@intFromEnum(Param.initial_velocity)] = .{ .minimum = 1.0, .maximum = 1.0 };
    table[@intFromEnum(Param.scale)] = .{ .minimum = 1.0, .maximum = 1.0 };
    return table;
}

/// The engine's 32-bit bit mixer.
fn mixBits(input: u32) u32 {
    var value = input;
    value ^= value >> 16;
    value *%= 0x7FEB352D;
    value ^= value >> 15;
    value *%= 0x846CA68B;
    value ^= value >> 16;
    return value;
}

/// A deterministic value in [0, 1) from `seed` and `salt`.
fn unitRandom(seed: u32, salt: u32) f32 {
    return @as(f32, @floatFromInt(mixBits(seed ^ mixBits(salt +% golden)) >> 8)) / 16777216.0;
}

/// Linear interpolation from `a` to `b` by `t`.
fn mixFloat(a: f32, b: f32, t: f32) f32 {
    return a + ((b - a) * t);
}

/// Linear interpolation between two vectors.
fn lerp(a: Vec3, b: Vec3, t: f32) Vec3 {
    return a.add(b.sub(a).scale(t));
}

/// Degrees in radians.
fn radians(degrees: f32) f32 {
    return degrees * degrees_to_radians;
}

/// Taylor polynomial of the sine, accurate for small angles.
fn sinPoly(x: f32) f32 {
    return x - ((x * x * x) / 6.0) + ((x * x * x * x * x) / 120.0);
}

/// Taylor polynomial of the cosine, accurate for small angles.
fn cosPoly(x: f32) f32 {
    return 1.0 - ((x * x) / 2.0) + ((x * x * x * x) / 24.0);
}

/// Unit vector along `value`, or `fallback` when it is too short.
fn safeNormalize(value: Vec3, fallback: Vec3) Vec3 {
    const size = value.length();
    return if (size > 1e-6) value.div(size) else fallback;
}

/// The component of `value` along axis 0, 1 or 2.
fn component(value: Vec3, axis: i32) f32 {
    return if (axis == 0) value.x else if (axis == 1) value.y else value.z;
}

/// Smoothstep of a blend factor.
fn smoothBlend(blend: f32) f32 {
    return blend * blend * (3.0 - (2.0 * blend));
}

/// A curve point.
fn pt(offset: f32, value: f32) CurvePoint {
    return .{ .offset = offset, .value = value };
}

/// A gradient stop.
fn stop(offset: f32, r: f32, g: f32, b: f32, a: f32) Stop {
    return .{ .offset = offset, .value = .{ .r = r, .g = g, .b = b, .a = a } };
}

/// Samples a curve, clamping outside its range; 1 for an empty curve.
fn sampleCurve(curve: *const Curve, offset: f32) f32 {
    if (curve.count == 0) return 1.0;
    if (offset <= curve.points[0].offset) return curve.points[0].value;
    if (offset >= curve.points[curve.count - 1].offset) return curve.points[curve.count - 1].value;
    var upper: u32 = 1;
    while (upper < curve.count and curve.points[upper].offset < offset) upper += 1;
    const from = curve.points[upper - 1];
    const to = curve.points[upper];
    const span = to.offset - from.offset;
    var blend: f32 = if (span > 0.0) (offset - from.offset) / span else 1.0;
    if (curve.smooth) blend = smoothBlend(blend);
    return from.value + ((to.value - from.value) * blend);
}

/// Samples a gradient, clamping outside its range; white for an empty one.
fn sampleGradient(gradient: *const Gradient, offset: f32) Rgba {
    if (gradient.count == 0) return .{};
    if (offset <= gradient.stops[0].offset) return gradient.stops[0].value;
    if (offset >= gradient.stops[gradient.count - 1].offset) return gradient.stops[gradient.count - 1].value;
    var upper: u32 = 1;
    while (upper < gradient.count and gradient.stops[upper].offset < offset) upper += 1;
    const from = gradient.stops[upper - 1];
    const to = gradient.stops[upper];
    const span = to.offset - from.offset;
    const blend: f32 = if (span > 0.0) (offset - from.offset) / span else 1.0;
    return .{
        .r = mixFloat(from.value.r, to.value.r, blend),
        .g = mixFloat(from.value.g, to.value.g, blend),
        .b = mixFloat(from.value.b, to.value.b, blend),
        .a = mixFloat(from.value.a, to.value.a, blend),
    };
}

/// The seeded random value between a parameter's minimum and maximum.
fn parameterRandom(parameter: *const Parameter, seed: u32, id: Param) f32 {
    return mixFloat(parameter.minimum, parameter.maximum, unitRandom(seed, salt_parameter_base + @intFromEnum(id)));
}

/// A parameter at life `offset`: its seeded value scaled by its curve.
fn parameterValue(effect: *const Effect, id: Param, seed: u32, offset: f32) f32 {
    const parameter = &effect.parameters[@intFromEnum(id)];
    return parameterRandom(parameter, seed, id) * sampleCurve(&parameter.curve, offset);
}

/// A seeded point in the emission shape, in emitter space.
fn shapePoint(effect: *const Effect, seed: u32) Vec3 {
    const a = unitRandom(seed, salt_shape_a);
    const b = unitRandom(seed, salt_shape_b);
    const c = unitRandom(seed, salt_shape_c);
    if (!effect.box_shape) return zero;
    return Vec3.init(
        ((a * 2.0) - 1.0) * effect.box_extents.x,
        ((b * 2.0) - 1.0) * effect.box_extents.y,
        ((c * 2.0) - 1.0) * effect.box_extents.z,
    );
}

/// A seeded unit launch direction within the effect's spread cone.
fn spreadDirection(effect: *const Effect, seed: u32) Vec3 {
    const spread = radians(effect.spread_degrees);
    const first = ((unitRandom(seed, salt_direction_a) * 2.0) - 1.0) * spread;
    const second = ((unitRandom(seed, salt_direction_b) * 2.0) - 1.0) * spread * (1.0 - effect.flatness);
    const across = Vec3.init(sinPoly(first), 0.0, cosPoly(first));
    var up = Vec3.init(0.0, sinPoly(second), cosPoly(second));
    up.z = up.z / vm.maxf(0.0001, @sqrt(if (up.z < 0.0) -up.z else up.z));
    const local = Vec3.init(across.x * up.z, up.y, across.z * up.z);
    const forward = safeNormalize(effect.direction, world_up);
    var binormal = world_up.cross(forward);
    binormal = if (binormal.length() < 0.0001) Vec3.init(0.0, 0.0, 1.0) else binormal.normalize();
    const normal = binormal.cross(forward);
    const mixed = binormal.scale(local.x).add(normal.scale(local.y)).add(forward.scale(local.z));
    return safeNormalize(mixed, forward);
}

/// Rotates the hue of `color` by `turns` of a full circle.
fn rotateHue(color: Rgba, turns: f32) Rgba {
    if (turns == 0.0) return color;
    const rotation = turns * two_pi;
    const cosine = cosPoly(rotation);
    const sine = sinPoly(rotation);
    const root = @sqrt(one_third);
    const shared = (1.0 - cosine) * one_third;
    const a = cosine + shared;
    const b = shared - (root * sine);
    const c = shared + (root * sine);
    return .{
        .r = (color.r * a) + (color.g * b) + (color.b * c),
        .g = (color.r * c) + (color.g * a) + (color.b * b),
        .b = (color.r * b) + (color.g * c) + (color.b * a),
        .a = color.a,
    };
}

/// The hashed lattice value in [-1, 1) at an integer grid point.
fn lattice(x: i32, y: i32, z: i32, salt: u32) f32 {
    const hashed = mixBits((@as(u32, @bitCast(x)) *% 73856093) ^ (@as(u32, @bitCast(y)) *% 19349663) ^
        (@as(u32, @bitCast(z)) *% 83492791) ^ mixBits(salt));
    return (@as(f32, @floatFromInt(hashed >> 8)) / 8388608.0) - 1.0;
}

/// The analytic gradient of smooth 3D value noise at `point`.
fn noiseGradient(point: Vec3, salt: u32) Vec3 {
    const fx = @floor(point.x);
    const fy = @floor(point.y);
    const fz = @floor(point.z);
    const x: i32 = @intFromFloat(fx);
    const y: i32 = @intFromFloat(fy);
    const z: i32 = @intFromFloat(fz);
    const f = Vec3.init(point.x - fx, point.y - fy, point.z - fz);
    const u = Vec3.init(f.x * f.x * (3.0 - (2.0 * f.x)), f.y * f.y * (3.0 - (2.0 * f.y)), f.z * f.z * (3.0 - (2.0 * f.z)));
    const du = Vec3.init(6.0 * f.x * (1.0 - f.x), 6.0 * f.y * (1.0 - f.y), 6.0 * f.z * (1.0 - f.z));
    const a = lattice(x, y, z, salt);
    const b = lattice(x + 1, y, z, salt);
    const c = lattice(x, y + 1, z, salt);
    const d = lattice(x + 1, y + 1, z, salt);
    const e = lattice(x, y, z + 1, salt);
    const g = lattice(x + 1, y, z + 1, salt);
    const h = lattice(x, y + 1, z + 1, salt);
    const k = lattice(x + 1, y + 1, z + 1, salt);
    const k1 = b - a;
    const k2 = c - a;
    const k3 = e - a;
    const k4 = a - b - c + d;
    const k5 = a - c - e + h;
    const k6 = a - b - e + g;
    const k7 = -a + b + c - d + e - g - h + k;
    return Vec3.init(
        du.x * (k1 + (k4 * u.y) + (k6 * u.z) + (k7 * u.y * u.z)),
        du.y * (k2 + (k5 * u.z) + (k4 * u.x) + (k7 * u.z * u.x)),
        du.z * (k3 + (k6 * u.x) + (k5 * u.y) + (k7 * u.x * u.y)),
    );
}

/// A divergence-free vector from the curl of three noise fields.
fn curlNoise(point: Vec3) Vec3 {
    const first = noiseGradient(point, 1);
    const second = noiseGradient(point, 2);
    const third = noiseGradient(point, 3);
    return Vec3.init(third.y - second.z, first.z - third.x, second.x - first.y);
}

/// Axes of `basis` normalised, removing scale.
fn rotationOf(basis: Basis) Basis {
    return .{
        .x = safeNormalize(basis.x, Vec3.init(1.0, 0.0, 0.0)),
        .y = safeNormalize(basis.y, Vec3.init(0.0, 1.0, 0.0)),
        .z = safeNormalize(basis.z, Vec3.init(0.0, 0.0, 1.0)),
    };
}

/// Updates a particle's colour, size and angle from its age, ramps, curves and tint.
fn applyDisplay(particle: *Particle, context: *const Context) void {
    const effect = context.effect;
    const offset = vm.clampf(particle.age / particle.lifetime, 0.0, 1.0);
    const seed = particle.seed;
    var color = effect.color;
    if (effect.color_initial_ramp.count != 0) {
        const initial = sampleGradient(&effect.color_initial_ramp, unitRandom(seed, salt_initial_ramp));
        color = .{ .r = color.r * initial.r, .g = color.g * initial.g, .b = color.b * initial.b, .a = color.a * initial.a };
    }
    if (effect.color_ramp.count != 0) {
        const ramp = sampleGradient(&effect.color_ramp, offset);
        color = .{ .r = color.r * ramp.r, .g = color.g * ramp.g, .b = color.b * ramp.b, .a = color.a * ramp.a };
    }
    color.a *= sampleCurve(&effect.alpha_curve, offset);
    color.a = vm.clampf(color.a, 0.0, 1.0);
    color = rotateHue(color, parameterValue(effect, .hue_variation, seed, offset));
    color = .{ .r = color.r * particle.tint.r, .g = color.g * particle.tint.g, .b = color.b * particle.tint.b, .a = color.a };
    particle.color = color;
    particle.size = effect.draw_size * vm.maxf(parameterValue(effect, .scale, seed, offset), 0.0);
    particle.angle = radians(parameterValue(effect, .angle, seed, offset)) + particle.spin;
}

/// Initialises `particle` from `seed` at the emission transform; false when
/// the amount ratio culls it.
fn spawn(particle: *Particle, seed: u32, context: *const Context) bool {
    const effect = context.effect;
    particle.* = .{};
    particle.seed = seed;
    particle.tint = context.tint;
    if (context.amount_ratio < 1.0 and unitRandom(seed, salt_ratio) >= context.amount_ratio) return false;
    particle.lifetime = vm.maxf(effect.lifetime * (1.0 - (effect.lifetime_randomness * unitRandom(seed, salt_lifetime))), minimum_lifetime);
    const local_position = shapePoint(effect, seed).add(effect.offset);
    const initial = &effect.parameters[@intFromEnum(Param.initial_velocity)];
    const local_velocity = spreadDirection(effect, seed).scale(parameterRandom(initial, seed, .initial_velocity));
    const rotation = rotationOf(context.basis);
    particle.position = context.basis.apply(local_position).add(context.origin);
    particle.velocity = rotation.apply(local_velocity).add(context.emitter_velocity.scale(effect.inherit_velocity));
    particle.origin = context.origin;
    particle.axis = rotation.y;
    particle.active = true;
    applyDisplay(particle, context);
    return true;
}

/// Clips the segment's parameter range against one slab of a box; false
/// when the segment misses it.
fn clipAxis(clip: *Clip, start: f32, delta: f32, low: f32, high: f32, axis: i32) bool {
    const magnitude = if (delta < 0.0) -delta else delta;
    if (magnitude < 1e-9) return !(start < low or start >= high);
    var near = (low - start) / delta;
    var far = (high - start) / delta;
    if (near > far) std.mem.swap(f32, &near, &far);
    if (near > clip.enter) {
        clip.enter = near;
        clip.axis = axis;
    }
    clip.exit = vm.minf(clip.exit, far);
    return true;
}

/// The surface normal where a segment enters a box along `axis`, or against
/// the segment when it starts inside.
fn contactNormal(axis: i32, delta: Vec3) Vec3 {
    var local: Vec3 = undefined;
    if (axis < 0) {
        local = delta.neg();
    } else {
        const sign: f32 = if (component(delta, axis) > 0.0) -1.0 else 1.0;
        local = if (axis == 0) Vec3.init(sign, 0.0, 0.0) else if (axis == 1) Vec3.init(0.0, sign, 0.0) else Vec3.init(0.0, 0.0, sign);
    }
    const size = local.length();
    return if (size > 1e-9) local.div(size) else Vec3.init(0.0, 1.0, 0.0);
}

/// The nearest point where the segment from `from` to `to` enters a box.
fn sweep(scene: *const Scene, from: Vec3, to: Vec3) ?Contact {
    const segment: vm.Aabb = .{ .min = from.min(to), .max = from.max(to) };
    const delta = to.sub(from);
    var best: f32 = 1e30;
    var hit: ?Contact = null;
    for (scene.boxes) |box| {
        if (!box.overlaps(segment)) continue;
        var clip: Clip = .{};
        const crosses = clipAxis(&clip, from.x, delta.x, box.min.x, box.max.x, 0) and
            clipAxis(&clip, from.y, delta.y, box.min.y, box.max.y, 1) and
            clipAxis(&clip, from.z, delta.z, box.min.z, box.max.z, 2);
        if (!crosses or clip.enter > clip.exit or clip.enter >= best) continue;
        best = clip.enter;
        hit = .{ .position = lerp(from, to, clip.enter), .normal = contactNormal(clip.axis, delta) };
    }
    return hit;
}

/// Advances a particle by `dt`: forces, wind, damping, turbulence,
/// controlled velocity and collision.
fn integrate(scene: *const Scene, particle: *Particle, dt: f32, context: *const Context) Outcome {
    const effect = context.effect;
    var outcome: Outcome = .{};
    particle.age += dt;
    if (particle.age >= particle.lifetime) {
        outcome.died = true;
        outcome.where = particle.position;
        outcome.velocity = particle.velocity;
        particle.active = false;
        return outcome;
    }
    const offset = particle.age / particle.lifetime;
    const seed = particle.seed;
    const diff = particle.position.sub(particle.origin);
    const radial = safeNormalize(diff, zero);
    const tangent = safeNormalize(particle.axis.cross(diff), zero);
    const heading = safeNormalize(particle.velocity, zero);
    const linear = parameterValue(effect, .linear_acceleration, seed, offset);
    const radial_force = parameterValue(effect, .radial_acceleration, seed, offset);
    const tangential = parameterValue(effect, .tangential_acceleration, seed, offset);
    const force = effect.gravity.add(heading.scale(linear)).add(radial.scale(radial_force)).add(tangent.scale(tangential));
    particle.velocity = particle.velocity.add(force.scale(dt));
    if (effect.wind_influence > 0.0) {
        const rate = effect.wind_influence * wind_response * dt;
        const pull = rate / (1.0 + rate);
        particle.velocity.x += (context.wind.x - particle.velocity.x) * pull;
        particle.velocity.z += (context.wind.z - particle.velocity.z) * pull;
        particle.velocity.y += context.wind.y * effect.wind_influence * wind_response * dt;
    }
    const drag = parameterValue(effect, .damping, seed, offset);
    if (drag > 0.0) {
        const speed = particle.velocity.length();
        if (speed > 0.0) particle.velocity = particle.velocity.scale(vm.maxf(speed - (drag * dt), 0.0) / speed);
    }
    if (effect.turbulence_enabled) {
        const rate = 1.0 + (effect.turbulence_speed_random * ((unitRandom(seed, salt_turbulence_speed) * 2.0) - 1.0));
        const sample_point = particle.position.div(vm.maxf(effect.turbulence_scale, 1e-3)).add(effect.turbulence_speed.scale(context.time * rate));
        const direction = safeNormalize(curlNoise(sample_point), zero);
        const influence = vm.clampf(parameterValue(effect, .turbulence_influence, seed, offset), 0.0, 1.0);
        const blend = influence * (dt * turbulence_reference_rate);
        const speed = particle.velocity.length();
        particle.velocity = lerp(particle.velocity, direction.scale(speed), blend).add(direction.scale(effect.turbulence_strength * dt));
    }
    const radial_speed = parameterValue(effect, .radial_velocity, seed, offset);
    const orbit = parameterValue(effect, .orbit_velocity, seed, offset);
    const controlled = radial.scale(radial_speed).add(particle.axis.cross(diff).scale(orbit * two_pi));
    const from = particle.position;
    const to = from.add(particle.velocity.add(controlled).scale(dt));
    particle.position = to;
    if (effect.collision != .disabled) {
        if (sweep(scene, from, to)) |contact| {
            outcome.where = contact.position;
            var velocity = particle.velocity;
            if (effect.collision == .hide_on_contact) {
                outcome.collided = true;
                outcome.velocity = velocity;
                particle.active = false;
                outcome.died = true;
                return outcome;
            }
            const radius: f32 = if (effect.use_scale) particle.size * 0.5 else 0.0;
            const rest = contact.position.add(contact.normal.scale(radius + contact_offset));
            const normal = contact.normal;
            const into = velocity.dot(normal);
            outcome.collided = -into > impact_speed;
            if (into < 0.0) {
                const slide = velocity.sub(normal.scale(into));
                velocity = slide.scale(1.0 - effect.friction).sub(normal.scale(into * effect.bounce));
            }
            particle.position = rest;
            particle.velocity = velocity;
            outcome.velocity = velocity;
        }
    }
    const spin = parameterValue(effect, .angular_velocity, seed, offset);
    particle.spin += radians(spin) * dt;
    applyDisplay(particle, context);
    return outcome;
}

/// The next inactive particle from the emit cursor, advancing the cursor;
/// null when the pool is full.
fn freeParticle(emitter: *Emitter, pool: []Particle) ?*Particle {
    const count: u32 = @intCast(pool.len);
    var probe: u32 = 0;
    while (probe < count) : (probe += 1) {
        const index = (emitter.emit_cursor + probe) % count;
        if (!pool[index].active) {
            emitter.emit_cursor = (index + 1) % count;
            return &pool[index];
        }
    }
    return null;
}

/// Spawns the parent's sub emitter particles at `origin` into free sub slots.
fn emitSub(emitter: *Emitter, sub_pool: []Particle, parent: *const Effect, sub: *const Effect, origin: SubOrigin) void {
    if (emitter.sub_count == 0) return;
    var context: Context = .{ .effect = sub };
    context.origin = origin.position;
    context.time = emitter.elapsed;
    const pool = emitter.sub_count;
    var count: u32 = 0;
    while (count < parent.sub_amount) : (count += 1) {
        var found = pool;
        var probe: u32 = 0;
        while (probe < pool) : (probe += 1) {
            const index = (emitter.sub_cursor + probe) % pool;
            if (!sub_pool[index].active) {
                found = index;
                break;
            }
        }
        if (found == pool) return;
        emitter.sub_cursor = (found + 1) % pool;
        const number = 0xB5297A4D +% emitter.bursts;
        emitter.bursts +%= 1;
        const seed = mixBits(emitter.current_seed ^ mixBits(number));
        const particle = &sub_pool[found];
        if (spawn(particle, seed, &context) and parent.sub_keep_velocity) particle.velocity = particle.velocity.add(origin.velocity);
    }
}

/// Counts a collision and spawns sub emitter particles by the sub emitter mode.
fn react(
    emitter: *Emitter,
    sub_pool: []Particle,
    collisions: *u64,
    parent: *const Effect,
    sub: ?*const Effect,
    particle: *Particle,
    outcome: Outcome,
    step_seconds: f32,
) void {
    if (outcome.collided) collisions.* += 1;
    const sub_effect = sub orelse return;
    switch (parent.sub_mode) {
        .at_end => {
            if (outcome.died and !outcome.collided) emitSub(emitter, sub_pool, parent, sub_effect, .{ .position = outcome.where, .velocity = outcome.velocity });
        },
        .at_collision => {
            if (outcome.collided) emitSub(emitter, sub_pool, parent, sub_effect, .{ .position = outcome.where, .velocity = outcome.velocity });
        },
        .constant => {
            if (!particle.active) return;
            const interval = 1.0 / vm.maxf(parent.sub_frequency, 1e-3);
            particle.sub_timer += step_seconds;
            while (particle.sub_timer >= interval) {
                particle.sub_timer -= interval;
                emitSub(emitter, sub_pool, parent, sub_effect, .{ .position = particle.position, .velocity = particle.velocity });
            }
        },
        .disabled => {},
    }
}

/// A seed for one numbered emission of an emitter.
fn emissionSeed(emitter: *const Emitter, number: u32) u32 {
    return mixBits(emitter.current_seed ^ mixBits(number));
}

/// The phase interval of one emission pass.
const Interval = struct {
    from: f32,
    to: f32,
    remaining: f32,
    cycle: u64,
};

/// Emits the particles whose restart phase falls inside one phase interval.
fn emitInterval(
    scene: *const Scene,
    emitter: *Emitter,
    pools: Pools,
    collisions: *u64,
    context: *const Context,
    sub: ?*const Effect,
    interval: Interval,
    step_seconds: f32,
) void {
    const effect = context.effect;
    const amount: u32 = pool_size;
    const lifetime = vm.maxf(effect.lifetime, minimum_lifetime);
    var index: u32 = 0;
    while (index < amount) : (index += 1) {
        const number: u32 = @truncate((interval.cycle *% amount) +% index);
        const seed = emissionSeed(emitter, number);
        var restart_phase = @as(f32, @floatFromInt(index)) / @as(f32, @floatFromInt(amount));
        if (effect.randomness > 0.0) restart_phase += effect.randomness * unitRandom(seed, salt_phase) / @as(f32, @floatFromInt(amount));
        restart_phase *= 1.0 - effect.explosiveness;
        if (restart_phase < interval.from or restart_phase >= interval.to) continue;
        const particle = &pools.main[index];
        if (!spawn(particle, seed, context)) continue;
        const local_delta = vm.clampf((interval.remaining - restart_phase) * lifetime, 0.0, step_seconds);
        const outcome = integrate(scene, particle, local_delta, context);
        react(emitter, pools.sub, collisions, effect, sub, particle, outcome, local_delta);
    }
}

/// Advances one emitter by `step_seconds`: integrates particles, emits by
/// phase, then spawns bursts.
fn stepEmitter(scene: *const Scene, emitter: *Emitter, pools: Pools, collisions: *u64, step_seconds: f32, origin: Vec3) void {
    const effect = &scene.effects[emitter.effect];
    const sub: ?*const Effect = if (emitter.sub_effect == no_effect) null else &scene.effects[emitter.sub_effect];
    var context: Context = .{ .effect = effect };
    context.basis = emitter.basis;
    context.origin = origin;
    context.emitter_velocity = emitter.velocity;
    context.time = emitter.elapsed;
    context.amount_ratio = vm.clampf(emitter.amount_ratio, 0.0, 1.0);
    context.wind = scene.wind;
    context.tint = emitter.tint;
    for (pools.main) |*particle| {
        if (!particle.active) continue;
        const outcome = integrate(scene, particle, step_seconds, &context);
        react(emitter, pools.sub, collisions, effect, sub, particle, outcome, step_seconds);
    }
    if (sub) |sub_effect| {
        var sub_context: Context = .{ .effect = sub_effect };
        sub_context.time = emitter.elapsed;
        sub_context.wind = scene.wind;
        sub_context.tint = emitter.tint;
        var index: u32 = 0;
        while (index < emitter.sub_count) : (index += 1) {
            const particle = &pools.sub[index];
            if (!particle.active) continue;
            const outcome = integrate(scene, particle, step_seconds, &sub_context);
            if (outcome.collided) collisions.* += 1;
        }
    }
    const lifetime = vm.maxf(effect.lifetime, minimum_lifetime);
    if (emitter.emitting) {
        const previous = emitter.phase;
        var phase = previous + (step_seconds / lifetime);
        const wrapped = phase >= 1.0;
        if (wrapped) phase = phase - @floor(phase);
        if (!wrapped) {
            emitInterval(scene, emitter, pools, collisions, &context, sub, .{ .from = previous, .to = phase, .remaining = phase, .cycle = emitter.cycle }, step_seconds);
        } else {
            emitInterval(scene, emitter, pools, collisions, &context, sub, .{ .from = previous, .to = 1.0, .remaining = 1.0 + phase, .cycle = emitter.cycle }, step_seconds);
            if (!effect.one_shot) {
                emitInterval(scene, emitter, pools, collisions, &context, sub, .{ .from = 0.0, .to = phase, .remaining = phase, .cycle = emitter.cycle + 1 }, step_seconds);
            }
        }
        emitter.phase = phase;
        if (wrapped) {
            emitter.cycle += 1;
            if (effect.one_shot) {
                emitter.emitting = false;
                emitter.phase = 0.0;
            }
        }
    }
    var burst_context = context;
    burst_context.amount_ratio = 1.0;
    while (emitter.burst_pending > 0) {
        emitter.burst_pending -= 1;
        const slot = freeParticle(emitter, pools.main) orelse {
            emitter.burst_pending = 0;
            break;
        };
        const number = 0x68E31DA4 +% emitter.bursts;
        emitter.bursts +%= 1;
        _ = spawn(slot, emissionSeed(emitter, number), &burst_context);
    }
    emitter.elapsed += step_seconds;
}

/// Counts the live particles of an emitter.
fn finish(emitter: *Emitter, pools: Pools) void {
    var alive: u32 = 0;
    for (pools.main) |particle| alive += if (particle.active) 1 else 0;
    var index: u32 = 0;
    while (index < emitter.sub_count) : (index += 1) alive += if (pools.sub[index].active) 1 else 0;
    emitter.alive = alive;
}

/// A curve from its points.
fn makeCurve(smooth: bool, points: []const CurvePoint) Curve {
    var curve: Curve = .{ .smooth = smooth };
    for (points) |point| {
        curve.points[curve.count] = point;
        curve.count += 1;
    }
    return curve;
}

/// A gradient from its stops.
fn makeGradient(stops: []const Stop) Gradient {
    var gradient: Gradient = .{};
    for (stops) |entry| {
        gradient.stops[gradient.count] = entry;
        gradient.count += 1;
    }
    return gradient;
}

/// Sets one parameter of an effect.
fn setParameter(effect: *Effect, id: Param, minimum: f32, maximum: f32, curve: Curve) void {
    effect.parameters[@intFromEnum(id)] = .{ .minimum = minimum, .maximum = maximum, .curve = curve };
}

/// A shower of bouncing sparks that leave dust where they land.
fn makeSparks() Effect {
    var e: Effect = .{};
    e.lifetime = 1.6;
    e.lifetime_randomness = 0.4;
    e.randomness = 0.5;
    e.box_shape = true;
    e.box_extents = Vec3.init(0.2, 0.05, 0.2);
    e.spread_degrees = 35.0;
    e.flatness = 0.2;
    e.inherit_velocity = 0.5;
    e.wind_influence = 0.1;
    e.draw_size = 0.15;
    e.color = .{ .r = 1.0, .g = 0.6, .b = 0.3, .a = 1.0 };
    e.color_ramp = makeGradient(&.{ stop(0.0, 1.0, 1.0, 0.8, 1.0), stop(0.4, 1.0, 0.5, 0.2, 1.0), stop(1.0, 0.3, 0.1, 0.1, 0.0) });
    e.alpha_curve = makeCurve(false, &.{ pt(0.0, 0.0), pt(0.1, 1.0), pt(1.0, 0.0) });
    setParameter(&e, .initial_velocity, 4.0, 9.0, .{});
    setParameter(&e, .linear_acceleration, 0.0, 1.0, makeCurve(false, &.{ pt(0.0, 1.0), pt(1.0, 0.0) }));
    setParameter(&e, .damping, 0.2, 0.6, makeCurve(true, &.{ pt(0.0, 1.0), pt(1.0, 0.2) }));
    setParameter(&e, .angle, 0.0, 360.0, .{});
    setParameter(&e, .angular_velocity, -90.0, 90.0, .{});
    setParameter(&e, .scale, 0.6, 1.2, makeCurve(false, &.{ pt(0.0, 0.2), pt(0.2, 1.0), pt(1.0, 0.1) }));
    setParameter(&e, .hue_variation, -0.05, 0.05, .{});
    e.collision = .rigid;
    e.friction = 0.3;
    e.bounce = 0.5;
    e.use_scale = true;
    e.sub_mode = .at_collision;
    e.sub_amount = 2;
    e.sub_effect = 4;
    return e;
}

/// Buoyant smoke pushed by wind and curl noise that vanishes on contact.
fn makeSmoke() Effect {
    var e: Effect = .{};
    e.lifetime = 2.5;
    e.lifetime_randomness = 0.3;
    e.randomness = 1.0;
    e.box_shape = true;
    e.box_extents = Vec3.init(0.5, 0.1, 0.5);
    e.spread_degrees = 25.0;
    e.gravity = Vec3.init(0.0, 0.6, 0.0);
    e.wind_influence = 0.8;
    e.draw_size = 0.8;
    e.color = .{ .r = 0.5, .g = 0.5, .b = 0.55, .a = 1.0 };
    e.color_initial_ramp = makeGradient(&.{ stop(0.0, 1.0, 0.9, 0.8, 1.0), stop(1.0, 0.8, 0.9, 1.0, 0.7) });
    e.color_ramp = makeGradient(&.{ stop(0.0, 1.0, 1.0, 1.0, 1.0), stop(1.0, 0.4, 0.4, 0.4, 1.0) });
    e.alpha_curve = makeCurve(true, &.{ pt(0.0, 0.0), pt(0.2, 0.6), pt(1.0, 0.0) });
    setParameter(&e, .initial_velocity, 0.5, 1.5, .{});
    setParameter(&e, .radial_acceleration, -0.2, 0.2, .{});
    setParameter(&e, .tangential_acceleration, 0.5, 1.0, .{});
    setParameter(&e, .damping, 0.3, 0.8, .{});
    setParameter(&e, .scale, 1.0, 2.5, makeCurve(true, &.{ pt(0.0, 0.3), pt(1.0, 1.0) }));
    setParameter(&e, .turbulence_influence, 0.3, 0.7, makeCurve(false, &.{ pt(0.0, 0.2), pt(1.0, 1.0) }));
    e.turbulence_enabled = true;
    e.turbulence_strength = 1.5;
    e.turbulence_scale = 3.0;
    e.collision = .hide_on_contact;
    return e;
}

/// Debris that erupts at once, tumbles, and bursts into embers where it ends.
fn makeDebris() Effect {
    var e: Effect = .{};
    e.lifetime = 2.0;
    e.randomness = 0.3;
    e.explosiveness = 0.7;
    e.box_shape = true;
    e.box_extents = Vec3.init(0.4, 0.4, 0.4);
    e.spread_degrees = 60.0;
    e.draw_size = 0.3;
    e.color = .{ .r = 0.6, .g = 0.5, .b = 0.4, .a = 1.0 };
    e.color_ramp = makeGradient(&.{ stop(0.0, 1.0, 1.0, 1.0, 1.0), stop(1.0, 0.5, 0.5, 0.5, 1.0) });
    setParameter(&e, .initial_velocity, 3.0, 8.0, .{});
    setParameter(&e, .orbit_velocity, 0.0, 0.1, .{});
    setParameter(&e, .angle, 0.0, 360.0, .{});
    setParameter(&e, .angular_velocity, -180.0, 180.0, .{});
    setParameter(&e, .scale, 0.5, 1.5, .{});
    e.collision = .rigid;
    e.friction = 0.5;
    e.bounce = 0.4;
    e.sub_mode = .at_end;
    e.sub_amount = 3;
    e.sub_keep_velocity = true;
    e.sub_effect = 5;
    return e;
}

/// A fountain that bounces and sheds dust as it flies.
fn makeFountain() Effect {
    var e: Effect = .{};
    e.lifetime = 1.2;
    e.lifetime_randomness = 0.2;
    e.spread_degrees = 12.0;
    e.draw_size = 0.2;
    e.color = .{ .r = 0.5, .g = 0.7, .b = 1.0, .a = 1.0 };
    e.alpha_curve = makeCurve(false, &.{ pt(0.0, 1.0), pt(0.8, 1.0), pt(1.0, 0.0) });
    setParameter(&e, .initial_velocity, 8.0, 12.0, .{});
    setParameter(&e, .radial_velocity, 0.0, 0.5, .{});
    e.collision = .rigid;
    e.friction = 0.1;
    e.bounce = 0.6;
    e.sub_mode = .constant;
    e.sub_frequency = 6.0;
    e.sub_amount = 1;
    e.sub_keep_velocity = true;
    e.sub_effect = 4;
    return e;
}

/// Short-lived dust puffs.
fn makeDust() Effect {
    var e: Effect = .{};
    e.amount = sub_pool_size;
    e.lifetime = 0.4;
    e.spread_degrees = 80.0;
    e.gravity = Vec3.init(0.0, -1.0, 0.0);
    e.draw_size = 0.1;
    e.color = .{ .r = 0.7, .g = 0.65, .b = 0.6, .a = 1.0 };
    e.alpha_curve = makeCurve(false, &.{ pt(0.0, 1.0), pt(1.0, 0.0) });
    setParameter(&e, .initial_velocity, 0.5, 2.0, .{});
    return e;
}

/// Embers that bounce once.
fn makeEmber() Effect {
    var e: Effect = .{};
    e.amount = sub_pool_size;
    e.lifetime = 0.6;
    e.spread_degrees = 80.0;
    e.gravity = Vec3.init(0.0, -4.0, 0.0);
    e.draw_size = 0.08;
    e.color = .{ .r = 1.0, .g = 0.4, .b = 0.1, .a = 1.0 };
    setParameter(&e, .initial_velocity, 1.0, 3.0, .{});
    e.collision = .rigid;
    e.bounce = 0.3;
    return e;
}

/// The six effects, the ground and boxes, and the wind.
fn makeScene(rng: *hash.Rng) Scene {
    var scene: Scene = .{};
    scene.effects = .{ makeSparks(), makeSmoke(), makeDebris(), makeFountain(), makeDust(), makeEmber() };
    for (scene.effects[0..4]) |*effect| effect.amount = pool_size;
    scene.wind = Vec3.init(3.0, 0.0, 1.0);
    scene.boxes[0] = .{ .min = Vec3.init(-1000.0, -1000.0, -1000.0), .max = Vec3.init(1000.0, 0.0, 1000.0) };
    for (scene.boxes[1..]) |*box| {
        const where = Vec3.random(rng, 0.0, 1.0);
        const half = Vec3.random(rng, 0.5, 2.0);
        const centre = Vec3.init(where.x * 40.0, half.y, where.z * 40.0);
        box.* = .{ .min = centre.sub(half), .max = centre.add(half) };
    }
    return scene;
}

/// An emitter at a random place and orientation, moving at a random velocity.
fn makeEmitter(index: u32, scene: *const Scene, rng: *hash.Rng) Emitter {
    var emitter: Emitter = .{};
    emitter.effect = index % 4;
    emitter.sub_effect = scene.effects[emitter.effect].sub_effect;
    emitter.first = index * pool_size;
    emitter.sub_first = index * sub_pool_size;
    emitter.sub_count = if (emitter.sub_effect == no_effect) 0 else scene.effects[emitter.sub_effect].amount;
    emitter.basis = Basis.fromQuat(vm.Quat.random(rng));
    const where = Vec3.random(rng, 0.0, 1.0);
    emitter.origin = Vec3.init(where.x * 40.0, 2.0 + (where.y * 4.0), where.z * 40.0);
    emitter.move = Vec3.random(rng, -3.0, 3.0);
    const red = 0.6 + (rng.unit() * 0.4);
    const green = 0.6 + (rng.unit() * 0.4);
    const blue = 0.6 + (rng.unit() * 0.4);
    emitter.tint = .{ .r = red, .g = green, .b = blue, .a = 1.0 };
    emitter.amount_ratio = 0.8 + (rng.unit() * 0.2);
    emitter.speed_scale = 0.9 + (rng.unit() * 0.2);
    emitter.current_seed = mixBits((index *% golden) ^ mixBits(1));
    return emitter;
}

/// Two floats packed by bit pattern into one word.
fn packPair(low: f32, high: f32) u64 {
    return @as(u64, hash.f32Bits(low)) | (@as(u64, hash.f32Bits(high)) << 32);
}

/// Folds one particle's state into a 64-bit word.
fn foldParticle(p: *const Particle) u64 {
    var word = packPair(p.position.x, p.position.y);
    word ^= packPair(p.position.z, p.velocity.x) *% 0x9E3779B97F4A7C15;
    word ^= packPair(p.velocity.y, p.velocity.z) *% 0xC2B2AE3D27D4EB4F;
    word ^= packPair(p.size, p.color.r) *% 0x165667B19E3779F9;
    word ^= packPair(p.color.g, p.color.b) *% 0x85EBCA77C2B2AE63;
    word ^= packPair(p.color.a, if (p.active) 1.0 else 0.0) *% 0x27D4EB2F165667C5;
    return word;
}

/// Builds the scene and emitters, runs their preprocess and keeps that state.
pub fn init(gpa: Allocator) !Particles {
    var rng: hash.Rng = .{ .s = 0x9a7 };
    var self: Particles = undefined;
    self.gpa = gpa;
    self.scene = makeScene(&rng);
    self.collisions = 0;
    self.particles = try gpa.alloc(Particle, @as(usize, emitter_count) * pool_size);
    @memset(self.particles, .{});
    self.sub_particles = try gpa.alloc(Particle, @as(usize, emitter_count) * sub_pool_size);
    @memset(self.sub_particles, .{});
    self.emitters = try gpa.alloc(Emitter, emitter_count);
    for (self.emitters, 0..) |*emitter, index| emitter.* = makeEmitter(@intCast(index), &self.scene, &rng);
    var index: u32 = 0;
    while (index < emitter_count) : (index += 1) self.preprocess(index);
    self.initial_emitters = try gpa.dupe(Emitter, self.emitters);
    self.initial_particles = try gpa.dupe(Particle, self.particles);
    self.initial_sub_particles = try gpa.dupe(Particle, self.sub_particles);
    return self;
}

/// Frees the emitters and particle pools.
pub fn deinit(self: *Particles, gpa: Allocator) void {
    gpa.free(self.emitters);
    gpa.free(self.initial_emitters);
    gpa.free(self.particles);
    gpa.free(self.initial_particles);
    gpa.free(self.sub_particles);
    gpa.free(self.initial_sub_particles);
}

/// The particle pools of emitter `index`.
fn poolsOf(self: *Particles, index: u32) Pools {
    const emitter = &self.emitters[index];
    return .{
        .main = self.particles[emitter.first..][0..pool_size],
        .sub = self.sub_particles[emitter.sub_first..][0..sub_pool_size],
    };
}

/// Runs one emitter's preprocess steps from a fresh start.
fn preprocess(self: *Particles, index: u32) void {
    const emitter = &self.emitters[index];
    const step_seconds = preprocess_seconds / @as(f32, @floatFromInt(preprocess_steps));
    var count: u32 = 0;
    while (count < preprocess_steps) : (count += 1) {
        stepEmitter(&self.scene, emitter, self.poolsOf(index), &self.collisions, step_seconds, emitter.origin);
    }
}

/// Advances every emitter by one update in shared substeps.
fn advance(self: *Particles) void {
    var start: [emitter_count]Vec3 = undefined;
    var end: [emitter_count]Vec3 = undefined;
    var scaled: [emitter_count]f32 = undefined;
    for (self.emitters, 0..) |*emitter, index| {
        start[index] = emitter.origin;
        end[index] = emitter.origin.add(emitter.move.scale(update_seconds));
        emitter.origin = end[index];
        emitter.velocity = end[index].sub(start[index]).div(update_seconds);
        scaled[index] = update_seconds * vm.maxf(emitter.speed_scale, 0.0);
    }
    var sub: u32 = 0;
    while (sub < substeps) : (sub += 1) {
        const fraction = @as(f32, @floatFromInt(sub + 1)) / @as(f32, @floatFromInt(substeps));
        for (self.emitters, 0..) |*emitter, index| {
            const origin = lerp(start[index], end[index], fraction);
            stepEmitter(&self.scene, emitter, self.poolsOf(@intCast(index)), &self.collisions, scaled[index] / @as(f32, @floatFromInt(substeps)), origin);
        }
    }
    for (self.emitters, 0..) |*emitter, index| finish(emitter, self.poolsOf(@intCast(index)));
}

/// Resets to the preprocessed state, runs every update and hashes the result.
pub fn run(self: *Particles) u64 {
    @memcpy(self.emitters, self.initial_emitters);
    @memcpy(self.particles, self.initial_particles);
    @memcpy(self.sub_particles, self.initial_sub_particles);
    self.collisions = 0;
    var update: u32 = 0;
    while (update < updates) : (update += 1) {
        if (update % 2 == 0) {
            var index: u32 = 0;
            while (index < emitter_count) : (index += 3) self.emitters[index].burst_pending += 256;
        }
        self.advance();
    }
    var h = hash.add(0, self.collisions);
    for (self.emitters) |emitter| h = hash.add(h, emitter.alive);
    for (self.particles) |*p| h = hash.add(h, foldParticle(p));
    for (self.sub_particles) |*p| h = hash.add(h, foldParticle(p));
    return h;
}
