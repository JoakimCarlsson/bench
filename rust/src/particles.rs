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
use crate::harness::Case;
use crate::hash;
use crate::vecmath::{self, Aabb, Basis, Quat, Vec3};

const EMITTER_COUNT: usize = 12;
const POOL_SIZE: usize = 2048;
const SUB_POOL_SIZE: usize = 256;
const UPDATES: u32 = 4;
const SUBSTEPS: u32 = 6;
const PREPROCESS_STEPS: u32 = 45;
const PREPROCESS_SECONDS: f32 = 1.5;
const UPDATE_SECONDS: f32 = 0.016666668;
const TWO_PI: f32 = 6.2831855;
const DEGREES_TO_RADIANS: f32 = 0.017453292;
const MINIMUM_LIFETIME: f32 = 1e-4;
const CONTACT_OFFSET: f32 = 1e-3;
const IMPACT_SPEED: f32 = 0.05;
const TURBULENCE_REFERENCE_RATE: f32 = 60.0;
const WIND_RESPONSE: f32 = 2.0;
const ONE_THIRD: f32 = 0.33333334;
const GOLDEN: u32 = 0x9E37_79B9;
const NO_EFFECT: u32 = 0xFFFF_FFFF;
const CURVE_CAPACITY: usize = 8;
const GRADIENT_CAPACITY: usize = 6;
const EFFECT_COUNT: usize = 6;
const BOX_COUNT: usize = 7;
const WORLD_UP: Vec3 = Vec3::new(0.0, 1.0, 0.0);
const ZERO: Vec3 = Vec3::new(0.0, 0.0, 0.0);

const SALT_LIFETIME: u32 = 1;
const SALT_RATIO: u32 = 2;
const SALT_INITIAL_RAMP: u32 = 3;
const SALT_PHASE: u32 = 4;
const SALT_SHAPE_A: u32 = 5;
const SALT_SHAPE_B: u32 = 6;
const SALT_SHAPE_C: u32 = 7;
const SALT_DIRECTION_A: u32 = 8;
const SALT_DIRECTION_B: u32 = 9;
const SALT_TURBULENCE_SPEED: u32 = 10;
const SALT_PARAMETER_BASE: u32 = 32;

#[derive(Clone, Copy)]
enum Param {
    InitialVelocity,
    AngularVelocity,
    OrbitVelocity,
    RadialVelocity,
    LinearAcceleration,
    RadialAcceleration,
    TangentialAcceleration,
    Damping,
    Angle,
    Scale,
    HueVariation,
    TurbulenceInfluence,
}

const PARAM_COUNT: usize = 12;

#[derive(Clone, Copy, PartialEq)]
enum CollisionMode {
    Disabled,
    Rigid,
    HideOnContact,
}

#[derive(Clone, Copy)]
enum SubMode {
    Disabled,
    Constant,
    AtEnd,
    AtCollision,
}

#[derive(Clone, Copy)]
struct Rgba {
    r: f32,
    g: f32,
    b: f32,
    a: f32,
}

impl Rgba {
    /// A colour from its channels.
    const fn new(r: f32, g: f32, b: f32, a: f32) -> Rgba {
        Rgba { r, g, b, a }
    }
}

impl Default for Rgba {
    /// Opaque white.
    fn default() -> Rgba {
        Rgba::new(1.0, 1.0, 1.0, 1.0)
    }
}

#[derive(Clone, Copy, Default)]
struct CurvePoint {
    offset: f32,
    value: f32,
}

#[derive(Clone, Copy, Default)]
struct Curve {
    smooth: bool,
    count: usize,
    points: [CurvePoint; CURVE_CAPACITY],
}

impl Curve {
    /// A curve from `(offset, value)` points.
    fn new(smooth: bool, points: &[(f32, f32)]) -> Curve {
        let mut curve = Curve { smooth, ..Curve::default() };
        for &(offset, value) in points {
            curve.points[curve.count] = CurvePoint { offset, value };
            curve.count += 1;
        }
        curve
    }

    /// Samples the curve, clamping outside its range; 1 for an empty curve.
    fn sample(&self, offset: f32) -> f32 {
        if self.count == 0 {
            return 1.0;
        }
        if offset <= self.points[0].offset {
            return self.points[0].value;
        }
        if offset >= self.points[self.count - 1].offset {
            return self.points[self.count - 1].value;
        }
        let mut upper = 1;
        while upper < self.count && self.points[upper].offset < offset {
            upper += 1;
        }
        let from = self.points[upper - 1];
        let to = self.points[upper];
        let span = to.offset - from.offset;
        let mut blend = if span > 0.0 { (offset - from.offset) / span } else { 1.0 };
        if self.smooth {
            blend = smooth_blend(blend);
        }
        from.value + ((to.value - from.value) * blend)
    }
}

#[derive(Clone, Copy, Default)]
struct Stop {
    offset: f32,
    value: Rgba,
}

#[derive(Clone, Copy, Default)]
struct Gradient {
    count: usize,
    stops: [Stop; GRADIENT_CAPACITY],
}

impl Gradient {
    /// A gradient from `(offset, [r, g, b, a])` stops.
    fn new(stops: &[(f32, [f32; 4])]) -> Gradient {
        let mut gradient = Gradient::default();
        for &(offset, c) in stops {
            gradient.stops[gradient.count] = Stop { offset, value: Rgba::new(c[0], c[1], c[2], c[3]) };
            gradient.count += 1;
        }
        gradient
    }

    /// Samples the gradient, clamping outside its range; white for an empty one.
    fn sample(&self, offset: f32) -> Rgba {
        if self.count == 0 {
            return Rgba::default();
        }
        if offset <= self.stops[0].offset {
            return self.stops[0].value;
        }
        if offset >= self.stops[self.count - 1].offset {
            return self.stops[self.count - 1].value;
        }
        let mut upper = 1;
        while upper < self.count && self.stops[upper].offset < offset {
            upper += 1;
        }
        let from = self.stops[upper - 1];
        let to = self.stops[upper];
        let span = to.offset - from.offset;
        let blend = if span > 0.0 { (offset - from.offset) / span } else { 1.0 };
        Rgba::new(
            mix_float(from.value.r, to.value.r, blend),
            mix_float(from.value.g, to.value.g, blend),
            mix_float(from.value.b, to.value.b, blend),
            mix_float(from.value.a, to.value.a, blend),
        )
    }
}

#[derive(Clone, Copy, Default)]
struct Parameter {
    minimum: f32,
    maximum: f32,
    curve: Curve,
}

#[derive(Clone, Copy)]
struct Effect {
    amount: u32,
    lifetime: f32,
    one_shot: bool,
    explosiveness: f32,
    randomness: f32,
    lifetime_randomness: f32,
    box_shape: bool,
    offset: Vec3,
    box_extents: Vec3,
    direction: Vec3,
    spread_degrees: f32,
    flatness: f32,
    inherit_velocity: f32,
    gravity: Vec3,
    wind_influence: f32,
    parameters: [Parameter; PARAM_COUNT],
    color: Rgba,
    color_ramp: Gradient,
    color_initial_ramp: Gradient,
    alpha_curve: Curve,
    turbulence_enabled: bool,
    turbulence_strength: f32,
    turbulence_scale: f32,
    turbulence_speed: Vec3,
    turbulence_speed_random: f32,
    collision: CollisionMode,
    friction: f32,
    bounce: f32,
    use_scale: bool,
    sub_mode: SubMode,
    sub_frequency: f32,
    sub_amount: u32,
    sub_keep_velocity: bool,
    sub_effect: u32,
    draw_size: f32,
}

impl Default for Effect {
    /// The engine's defaults, with unit initial velocity and scale.
    fn default() -> Effect {
        let mut parameters = [Parameter::default(); PARAM_COUNT];
        parameters[Param::InitialVelocity as usize] = Parameter { minimum: 1.0, maximum: 1.0, curve: Curve::default() };
        parameters[Param::Scale as usize] = Parameter { minimum: 1.0, maximum: 1.0, curve: Curve::default() };
        Effect {
            amount: 16,
            lifetime: 1.0,
            one_shot: false,
            explosiveness: 0.0,
            randomness: 0.0,
            lifetime_randomness: 0.0,
            box_shape: false,
            offset: ZERO,
            box_extents: Vec3::new(1.0, 1.0, 1.0),
            direction: Vec3::new(0.0, 1.0, 0.0),
            spread_degrees: 45.0,
            flatness: 0.0,
            inherit_velocity: 0.0,
            gravity: Vec3::new(0.0, -9.8, 0.0),
            wind_influence: 0.0,
            parameters,
            color: Rgba::default(),
            color_ramp: Gradient::default(),
            color_initial_ramp: Gradient::default(),
            alpha_curve: Curve::default(),
            turbulence_enabled: false,
            turbulence_strength: 1.0,
            turbulence_scale: 4.0,
            turbulence_speed: Vec3::new(0.0, 0.5, 0.0),
            turbulence_speed_random: 0.2,
            collision: CollisionMode::Disabled,
            friction: 0.0,
            bounce: 0.0,
            use_scale: false,
            sub_mode: SubMode::Disabled,
            sub_frequency: 4.0,
            sub_amount: 1,
            sub_keep_velocity: false,
            sub_effect: NO_EFFECT,
            draw_size: 0.25,
        }
    }
}

impl Effect {
    /// Sets one parameter.
    fn set(&mut self, id: Param, minimum: f32, maximum: f32, curve: Curve) {
        self.parameters[id as usize] = Parameter { minimum, maximum, curve };
    }

    /// The seeded random value between a parameter's minimum and maximum.
    fn parameter_random(&self, id: Param, seed: u32) -> f32 {
        let parameter = &self.parameters[id as usize];
        mix_float(parameter.minimum, parameter.maximum, unit_random(seed, SALT_PARAMETER_BASE + id as u32))
    }

    /// A parameter at life `offset`: its seeded value scaled by its curve.
    fn parameter_value(&self, id: Param, seed: u32, offset: f32) -> f32 {
        self.parameter_random(id, seed) * self.parameters[id as usize].curve.sample(offset)
    }
}

#[derive(Clone, Copy)]
struct Particle {
    position: Vec3,
    velocity: Vec3,
    origin: Vec3,
    axis: Vec3,
    color: Rgba,
    tint: Rgba,
    angle: f32,
    spin: f32,
    size: f32,
    age: f32,
    lifetime: f32,
    sub_timer: f32,
    seed: u32,
    active: bool,
}

impl Default for Particle {
    /// An inactive particle with the engine's defaults.
    fn default() -> Particle {
        Particle {
            position: ZERO,
            velocity: ZERO,
            origin: ZERO,
            axis: WORLD_UP,
            color: Rgba::default(),
            tint: Rgba::default(),
            angle: 0.0,
            spin: 0.0,
            size: 0.0,
            age: 0.0,
            lifetime: 0.0,
            sub_timer: 0.0,
            seed: 0,
            active: false,
        }
    }
}

#[derive(Clone, Copy)]
struct Emitter {
    effect: u32,
    sub_effect: u32,
    first: usize,
    sub_first: usize,
    sub_count: usize,
    emit_cursor: u32,
    sub_cursor: u32,
    bursts: u32,
    current_seed: u32,
    burst_pending: u32,
    alive: u32,
    cycle: u64,
    phase: f32,
    elapsed: f32,
    emitting: bool,
    amount_ratio: f32,
    speed_scale: f32,
    tint: Rgba,
    basis: Basis,
    origin: Vec3,
    movement: Vec3,
    velocity: Vec3,
}

impl Default for Emitter {
    /// An emitter with the engine's defaults.
    fn default() -> Emitter {
        Emitter {
            effect: 0,
            sub_effect: NO_EFFECT,
            first: 0,
            sub_first: 0,
            sub_count: 0,
            emit_cursor: 0,
            sub_cursor: 0,
            bursts: 0,
            current_seed: 0,
            burst_pending: 0,
            alive: 0,
            cycle: 0,
            phase: 0.0,
            elapsed: 0.0,
            emitting: true,
            amount_ratio: 1.0,
            speed_scale: 1.0,
            tint: Rgba::default(),
            basis: Basis::default(),
            origin: ZERO,
            movement: ZERO,
            velocity: ZERO,
        }
    }
}

struct Scene {
    effects: [Effect; EFFECT_COUNT],
    boxes: [Aabb; BOX_COUNT],
    wind: Vec3,
}

#[derive(Clone, Copy)]
struct Context<'a> {
    effect: &'a Effect,
    basis: Basis,
    origin: Vec3,
    emitter_velocity: Vec3,
    time: f32,
    amount_ratio: f32,
    wind: Vec3,
    tint: Rgba,
}

impl<'a> Context<'a> {
    /// A step context for `effect` with the engine's defaults.
    fn new(effect: &'a Effect) -> Context<'a> {
        Context {
            effect,
            basis: Basis::default(),
            origin: ZERO,
            emitter_velocity: ZERO,
            time: 0.0,
            amount_ratio: 1.0,
            wind: ZERO,
            tint: Rgba::default(),
        }
    }
}

struct Contact {
    position: Vec3,
    normal: Vec3,
}

#[derive(Default)]
struct Outcome {
    died: bool,
    collided: bool,
    place: Vec3,
    velocity: Vec3,
}

struct SubOrigin {
    position: Vec3,
    velocity: Vec3,
}

struct Clip {
    enter: f32,
    exit: f32,
    axis: i32,
}

struct Interval {
    from: f32,
    to: f32,
    remaining: f32,
    cycle: u64,
}

pub struct Particles {
    scene: Scene,
    collisions: u64,
    emitters: Vec<Emitter>,
    initial_emitters: Vec<Emitter>,
    particles: Vec<Particle>,
    initial_particles: Vec<Particle>,
    sub_particles: Vec<Particle>,
    initial_sub_particles: Vec<Particle>,
}

/// The engine's 32-bit bit mixer.
fn mix_bits(mut value: u32) -> u32 {
    value ^= value >> 16;
    value = value.wrapping_mul(0x7FEB_352D);
    value ^= value >> 15;
    value = value.wrapping_mul(0x846C_A68B);
    value ^= value >> 16;
    value
}

/// A deterministic value in [0, 1) from `seed` and `salt`.
fn unit_random(seed: u32, salt: u32) -> f32 {
    (mix_bits(seed ^ mix_bits(salt.wrapping_add(GOLDEN))) >> 8) as f32 / 16777216.0
}

/// Linear interpolation from `a` to `b` by `t`.
fn mix_float(a: f32, b: f32, t: f32) -> f32 {
    a + ((b - a) * t)
}

/// Linear interpolation between two vectors.
fn lerp(a: Vec3, b: Vec3, t: f32) -> Vec3 {
    a + (b - a) * t
}

/// Degrees in radians.
fn radians(degrees: f32) -> f32 {
    degrees * DEGREES_TO_RADIANS
}

/// Taylor polynomial of the sine, accurate for small angles.
fn sin_poly(x: f32) -> f32 {
    x - ((x * x * x) / 6.0) + ((x * x * x * x * x) / 120.0)
}

/// Taylor polynomial of the cosine, accurate for small angles.
fn cos_poly(x: f32) -> f32 {
    1.0 - ((x * x) / 2.0) + ((x * x * x * x) / 24.0)
}

/// Unit vector along `value`, or `fallback` when it is too short.
fn safe_normalize(value: Vec3, fallback: Vec3) -> Vec3 {
    let size = value.length();
    if size > 1e-6 { value / size } else { fallback }
}

/// The component of `value` along axis 0, 1 or 2.
fn component(value: Vec3, axis: i32) -> f32 {
    if axis == 0 {
        value.x
    } else if axis == 1 {
        value.y
    } else {
        value.z
    }
}

/// Smoothstep of a blend factor.
fn smooth_blend(blend: f32) -> f32 {
    blend * blend * (3.0 - (2.0 * blend))
}

/// A seeded point in the emission shape, in emitter space.
fn shape_point(effect: &Effect, seed: u32) -> Vec3 {
    let a = unit_random(seed, SALT_SHAPE_A);
    let b = unit_random(seed, SALT_SHAPE_B);
    let c = unit_random(seed, SALT_SHAPE_C);
    if !effect.box_shape {
        return ZERO;
    }
    Vec3::new(
        ((a * 2.0) - 1.0) * effect.box_extents.x,
        ((b * 2.0) - 1.0) * effect.box_extents.y,
        ((c * 2.0) - 1.0) * effect.box_extents.z,
    )
}

/// A seeded unit launch direction within the effect's spread cone.
fn spread_direction(effect: &Effect, seed: u32) -> Vec3 {
    let spread = radians(effect.spread_degrees);
    let first = ((unit_random(seed, SALT_DIRECTION_A) * 2.0) - 1.0) * spread;
    let second = ((unit_random(seed, SALT_DIRECTION_B) * 2.0) - 1.0) * spread * (1.0 - effect.flatness);
    let across = Vec3::new(sin_poly(first), 0.0, cos_poly(first));
    let mut up = Vec3::new(0.0, sin_poly(second), cos_poly(second));
    up.z = up.z / vecmath::max(0.0001, (if up.z < 0.0 { -up.z } else { up.z }).sqrt());
    let local = Vec3::new(across.x * up.z, up.y, across.z * up.z);
    let forward = safe_normalize(effect.direction, WORLD_UP);
    let mut binormal = WORLD_UP.cross(forward);
    binormal = if binormal.length() < 0.0001 { Vec3::new(0.0, 0.0, 1.0) } else { binormal.normalize() };
    let normal = binormal.cross(forward);
    safe_normalize((binormal * local.x) + (normal * local.y) + (forward * local.z), forward)
}

/// Rotates the hue of `color` by `turns` of a full circle.
fn rotate_hue(color: Rgba, turns: f32) -> Rgba {
    if turns == 0.0 {
        return color;
    }
    let rotation = turns * TWO_PI;
    let cosine = cos_poly(rotation);
    let sine = sin_poly(rotation);
    let root = ONE_THIRD.sqrt();
    let shared = (1.0 - cosine) * ONE_THIRD;
    let a = cosine + shared;
    let b = shared - (root * sine);
    let c = shared + (root * sine);
    Rgba::new(
        (color.r * a) + (color.g * b) + (color.b * c),
        (color.r * c) + (color.g * a) + (color.b * b),
        (color.r * b) + (color.g * c) + (color.b * a),
        color.a,
    )
}

/// The hashed lattice value in [-1, 1) at an integer grid point.
fn lattice(x: i32, y: i32, z: i32, salt: u32) -> f32 {
    let hashed = mix_bits(
        (x as u32).wrapping_mul(73856093)
            ^ (y as u32).wrapping_mul(19349663)
            ^ (z as u32).wrapping_mul(83492791)
            ^ mix_bits(salt),
    );
    ((hashed >> 8) as f32 / 8388608.0) - 1.0
}

/// The analytic gradient of smooth 3D value noise at `point`.
fn noise_gradient(point: Vec3, salt: u32) -> Vec3 {
    let fx = point.x.floor();
    let fy = point.y.floor();
    let fz = point.z.floor();
    let x = fx as i32;
    let y = fy as i32;
    let z = fz as i32;
    let f = Vec3::new(point.x - fx, point.y - fy, point.z - fz);
    let u = Vec3::new(f.x * f.x * (3.0 - (2.0 * f.x)), f.y * f.y * (3.0 - (2.0 * f.y)), f.z * f.z * (3.0 - (2.0 * f.z)));
    let du = Vec3::new(6.0 * f.x * (1.0 - f.x), 6.0 * f.y * (1.0 - f.y), 6.0 * f.z * (1.0 - f.z));
    let a = lattice(x, y, z, salt);
    let b = lattice(x + 1, y, z, salt);
    let c = lattice(x, y + 1, z, salt);
    let d = lattice(x + 1, y + 1, z, salt);
    let e = lattice(x, y, z + 1, salt);
    let g = lattice(x + 1, y, z + 1, salt);
    let h = lattice(x, y + 1, z + 1, salt);
    let k = lattice(x + 1, y + 1, z + 1, salt);
    let k1 = b - a;
    let k2 = c - a;
    let k3 = e - a;
    let k4 = a - b - c + d;
    let k5 = a - c - e + h;
    let k6 = a - b - e + g;
    let k7 = -a + b + c - d + e - g - h + k;
    Vec3::new(
        du.x * (k1 + (k4 * u.y) + (k6 * u.z) + (k7 * u.y * u.z)),
        du.y * (k2 + (k5 * u.z) + (k4 * u.x) + (k7 * u.z * u.x)),
        du.z * (k3 + (k6 * u.x) + (k5 * u.y) + (k7 * u.x * u.y)),
    )
}

/// A divergence-free vector from the curl of three noise fields.
fn curl_noise(point: Vec3) -> Vec3 {
    let first = noise_gradient(point, 1);
    let second = noise_gradient(point, 2);
    let third = noise_gradient(point, 3);
    Vec3::new(third.y - second.z, first.z - third.x, second.x - first.y)
}

/// Axes of `basis` normalised, removing scale.
fn rotation_of(basis: &Basis) -> Basis {
    Basis {
        x: safe_normalize(basis.x, Vec3::new(1.0, 0.0, 0.0)),
        y: safe_normalize(basis.y, Vec3::new(0.0, 1.0, 0.0)),
        z: safe_normalize(basis.z, Vec3::new(0.0, 0.0, 1.0)),
    }
}

/// Updates a particle's colour, size and angle from its age, ramps, curves and tint.
fn apply_display(particle: &mut Particle, context: &Context) {
    let effect = context.effect;
    let offset = vecmath::clamp(particle.age / particle.lifetime, 0.0, 1.0);
    let seed = particle.seed;
    let mut color = effect.color;
    if effect.color_initial_ramp.count != 0 {
        let initial = effect.color_initial_ramp.sample(unit_random(seed, SALT_INITIAL_RAMP));
        color = Rgba::new(color.r * initial.r, color.g * initial.g, color.b * initial.b, color.a * initial.a);
    }
    if effect.color_ramp.count != 0 {
        let ramp = effect.color_ramp.sample(offset);
        color = Rgba::new(color.r * ramp.r, color.g * ramp.g, color.b * ramp.b, color.a * ramp.a);
    }
    color.a *= effect.alpha_curve.sample(offset);
    color.a = vecmath::clamp(color.a, 0.0, 1.0);
    color = rotate_hue(color, effect.parameter_value(Param::HueVariation, seed, offset));
    color = Rgba::new(color.r * particle.tint.r, color.g * particle.tint.g, color.b * particle.tint.b, color.a);
    particle.color = color;
    particle.size = effect.draw_size * vecmath::max(effect.parameter_value(Param::Scale, seed, offset), 0.0);
    particle.angle = radians(effect.parameter_value(Param::Angle, seed, offset)) + particle.spin;
}

/// Initialises `particle` from `seed` at the emission transform; false when
/// the amount ratio culls it.
fn spawn(particle: &mut Particle, seed: u32, context: &Context) -> bool {
    let effect = context.effect;
    *particle = Particle::default();
    particle.seed = seed;
    particle.tint = context.tint;
    if context.amount_ratio < 1.0 && unit_random(seed, SALT_RATIO) >= context.amount_ratio {
        return false;
    }
    particle.lifetime = vecmath::max(
        effect.lifetime * (1.0 - (effect.lifetime_randomness * unit_random(seed, SALT_LIFETIME))),
        MINIMUM_LIFETIME,
    );
    let local_position = shape_point(effect, seed) + effect.offset;
    let local_velocity = spread_direction(effect, seed) * effect.parameter_random(Param::InitialVelocity, seed);
    let rotation = rotation_of(&context.basis);
    particle.position = context.basis * local_position + context.origin;
    particle.velocity = (rotation * local_velocity) + (context.emitter_velocity * effect.inherit_velocity);
    particle.origin = context.origin;
    particle.axis = rotation.y;
    particle.active = true;
    apply_display(particle, context);
    true
}

/// Clips the segment's parameter range against one slab of a box; false
/// when the segment misses it.
fn clip_axis(clip: &mut Clip, start: f32, delta: f32, low: f32, high: f32, axis: i32) -> bool {
    let magnitude = if delta < 0.0 { -delta } else { delta };
    if magnitude < 1e-9 {
        return !(start < low || start >= high);
    }
    let mut near = (low - start) / delta;
    let mut far = (high - start) / delta;
    if near > far {
        std::mem::swap(&mut near, &mut far);
    }
    if near > clip.enter {
        clip.enter = near;
        clip.axis = axis;
    }
    clip.exit = vecmath::min(clip.exit, far);
    true
}

/// The surface normal where a segment enters a box along `axis`, or against
/// the segment when it starts inside.
fn contact_normal(axis: i32, delta: Vec3) -> Vec3 {
    let local = if axis < 0 {
        -delta
    } else {
        let sign = if component(delta, axis) > 0.0 { -1.0 } else { 1.0 };
        if axis == 0 {
            Vec3::new(sign, 0.0, 0.0)
        } else if axis == 1 {
            Vec3::new(0.0, sign, 0.0)
        } else {
            Vec3::new(0.0, 0.0, sign)
        }
    };
    let size = local.length();
    if size > 1e-9 { local / size } else { Vec3::new(0.0, 1.0, 0.0) }
}

/// The nearest point where the segment from `from` to `to` enters a box.
fn sweep(scene: &Scene, from: Vec3, to: Vec3) -> Option<Contact> {
    let segment = Aabb { min: from.min(to), max: from.max(to) };
    let delta = to - from;
    let mut best = 1e30f32;
    let mut hit = None;
    for box_ in &scene.boxes {
        if !box_.overlaps(&segment) {
            continue;
        }
        let mut clip = Clip { enter: 0.0, exit: 1.0, axis: -1 };
        let crosses = clip_axis(&mut clip, from.x, delta.x, box_.min.x, box_.max.x, 0)
            && clip_axis(&mut clip, from.y, delta.y, box_.min.y, box_.max.y, 1)
            && clip_axis(&mut clip, from.z, delta.z, box_.min.z, box_.max.z, 2);
        if !crosses || clip.enter > clip.exit || clip.enter >= best {
            continue;
        }
        best = clip.enter;
        hit = Some(Contact { position: lerp(from, to, clip.enter), normal: contact_normal(clip.axis, delta) });
    }
    hit
}

/// Advances a particle by `step`: forces, wind, damping, turbulence,
/// controlled velocity and collision.
fn integrate(scene: &Scene, particle: &mut Particle, step: f32, context: &Context) -> Outcome {
    let effect = context.effect;
    let mut outcome = Outcome::default();
    particle.age += step;
    if particle.age >= particle.lifetime {
        outcome.died = true;
        outcome.place = particle.position;
        outcome.velocity = particle.velocity;
        particle.active = false;
        return outcome;
    }
    let offset = particle.age / particle.lifetime;
    let seed = particle.seed;
    let diff = particle.position - particle.origin;
    let radial = safe_normalize(diff, ZERO);
    let tangent = safe_normalize(particle.axis.cross(diff), ZERO);
    let heading = safe_normalize(particle.velocity, ZERO);
    let linear = effect.parameter_value(Param::LinearAcceleration, seed, offset);
    let radial_force = effect.parameter_value(Param::RadialAcceleration, seed, offset);
    let tangential = effect.parameter_value(Param::TangentialAcceleration, seed, offset);
    let force = effect.gravity + (heading * linear) + (radial * radial_force) + (tangent * tangential);
    particle.velocity = particle.velocity + (force * step);
    if effect.wind_influence > 0.0 {
        let rate = effect.wind_influence * WIND_RESPONSE * step;
        let pull = rate / (1.0 + rate);
        particle.velocity.x += (context.wind.x - particle.velocity.x) * pull;
        particle.velocity.z += (context.wind.z - particle.velocity.z) * pull;
        particle.velocity.y += context.wind.y * effect.wind_influence * WIND_RESPONSE * step;
    }
    let drag = effect.parameter_value(Param::Damping, seed, offset);
    if drag > 0.0 {
        let speed = particle.velocity.length();
        if speed > 0.0 {
            particle.velocity = particle.velocity * (vecmath::max(speed - (drag * step), 0.0) / speed);
        }
    }
    if effect.turbulence_enabled {
        let rate = 1.0 + (effect.turbulence_speed_random * ((unit_random(seed, SALT_TURBULENCE_SPEED) * 2.0) - 1.0));
        let sample_point =
            (particle.position / vecmath::max(effect.turbulence_scale, 1e-3)) + (effect.turbulence_speed * (context.time * rate));
        let direction = safe_normalize(curl_noise(sample_point), ZERO);
        let influence = vecmath::clamp(effect.parameter_value(Param::TurbulenceInfluence, seed, offset), 0.0, 1.0);
        let blend = influence * (step * TURBULENCE_REFERENCE_RATE);
        let speed = particle.velocity.length();
        particle.velocity = lerp(particle.velocity, direction * speed, blend) + (direction * (effect.turbulence_strength * step));
    }
    let radial_speed = effect.parameter_value(Param::RadialVelocity, seed, offset);
    let orbit = effect.parameter_value(Param::OrbitVelocity, seed, offset);
    let controlled = (radial * radial_speed) + (particle.axis.cross(diff) * (orbit * TWO_PI));
    let from = particle.position;
    let to = from + ((particle.velocity + controlled) * step);
    particle.position = to;
    if effect.collision != CollisionMode::Disabled {
        if let Some(contact) = sweep(scene, from, to) {
            outcome.place = contact.position;
            let mut velocity = particle.velocity;
            if effect.collision == CollisionMode::HideOnContact {
                outcome.collided = true;
                outcome.velocity = velocity;
                particle.active = false;
                outcome.died = true;
                return outcome;
            }
            let radius = if effect.use_scale { particle.size * 0.5 } else { 0.0 };
            let rest = contact.position + (contact.normal * (radius + CONTACT_OFFSET));
            let normal = contact.normal;
            let into = velocity.dot(normal);
            outcome.collided = -into > IMPACT_SPEED;
            if into < 0.0 {
                let slide = velocity - (normal * into);
                velocity = (slide * (1.0 - effect.friction)) - (normal * (into * effect.bounce));
            }
            particle.position = rest;
            particle.velocity = velocity;
            outcome.velocity = velocity;
        }
    }
    let spin = effect.parameter_value(Param::AngularVelocity, seed, offset);
    particle.spin += radians(spin) * step;
    apply_display(particle, context);
    outcome
}

/// The index of the next inactive particle from the emit cursor, advancing
/// the cursor; none when the pool is full.
fn free_particle(emitter: &mut Emitter, pool: &[Particle]) -> Option<usize> {
    let count = pool.len() as u32;
    for probe in 0..count {
        let index = (emitter.emit_cursor + probe) % count;
        if !pool[index as usize].active {
            emitter.emit_cursor = (index + 1) % count;
            return Some(index as usize);
        }
    }
    None
}

/// Spawns the parent's sub emitter particles at `origin` into free sub slots.
fn emit_sub(emitter: &mut Emitter, sub_pool: &mut [Particle], parent: &Effect, sub: &Effect, origin: SubOrigin) {
    if emitter.sub_count == 0 {
        return;
    }
    let mut context = Context::new(sub);
    context.origin = origin.position;
    context.time = emitter.elapsed;
    let pool = emitter.sub_count as u32;
    for _ in 0..parent.sub_amount {
        let mut found = pool;
        for probe in 0..pool {
            let index = (emitter.sub_cursor + probe) % pool;
            if !sub_pool[index as usize].active {
                found = index;
                break;
            }
        }
        if found == pool {
            return;
        }
        emitter.sub_cursor = (found + 1) % pool;
        let number = 0xB529_7A4Du32.wrapping_add(emitter.bursts);
        emitter.bursts = emitter.bursts.wrapping_add(1);
        let seed = mix_bits(emitter.current_seed ^ mix_bits(number));
        let particle = &mut sub_pool[found as usize];
        if spawn(particle, seed, &context) && parent.sub_keep_velocity {
            particle.velocity = particle.velocity + origin.velocity;
        }
    }
}

/// Counts a collision and spawns sub emitter particles by the sub emitter mode.
#[allow(clippy::too_many_arguments)]
fn react(
    emitter: &mut Emitter,
    sub_pool: &mut [Particle],
    collisions: &mut u64,
    parent: &Effect,
    sub: Option<&Effect>,
    particle: &mut Particle,
    outcome: &Outcome,
    step_seconds: f32,
) {
    if outcome.collided {
        *collisions += 1;
    }
    let Some(sub) = sub else { return };
    match parent.sub_mode {
        SubMode::AtEnd => {
            if outcome.died && !outcome.collided {
                emit_sub(emitter, sub_pool, parent, sub, SubOrigin { position: outcome.place, velocity: outcome.velocity });
            }
        }
        SubMode::AtCollision => {
            if outcome.collided {
                emit_sub(emitter, sub_pool, parent, sub, SubOrigin { position: outcome.place, velocity: outcome.velocity });
            }
        }
        SubMode::Constant => {
            if !particle.active {
                return;
            }
            let interval = 1.0 / vecmath::max(parent.sub_frequency, 1e-3);
            particle.sub_timer += step_seconds;
            while particle.sub_timer >= interval {
                particle.sub_timer -= interval;
                emit_sub(emitter, sub_pool, parent, sub, SubOrigin { position: particle.position, velocity: particle.velocity });
            }
        }
        SubMode::Disabled => {}
    }
}

/// A seed for one numbered emission of an emitter.
fn emission_seed(emitter: &Emitter, number: u32) -> u32 {
    mix_bits(emitter.current_seed ^ mix_bits(number))
}

/// Emits the particles whose restart phase falls inside one phase interval.
#[allow(clippy::too_many_arguments)]
fn emit_interval(
    scene: &Scene,
    emitter: &mut Emitter,
    main: &mut [Particle],
    sub_pool: &mut [Particle],
    collisions: &mut u64,
    context: &Context,
    sub: Option<&Effect>,
    interval: &Interval,
    step_seconds: f32,
) {
    let effect = context.effect;
    let amount = POOL_SIZE as u32;
    let lifetime = vecmath::max(effect.lifetime, MINIMUM_LIFETIME);
    for index in 0..amount {
        let number = interval.cycle.wrapping_mul(amount as u64).wrapping_add(index as u64) as u32;
        let seed = emission_seed(emitter, number);
        let mut restart_phase = index as f32 / amount as f32;
        if effect.randomness > 0.0 {
            restart_phase += effect.randomness * unit_random(seed, SALT_PHASE) / amount as f32;
        }
        restart_phase *= 1.0 - effect.explosiveness;
        if restart_phase < interval.from || restart_phase >= interval.to {
            continue;
        }
        let particle = &mut main[index as usize];
        if !spawn(particle, seed, context) {
            continue;
        }
        let local_delta = vecmath::clamp((interval.remaining - restart_phase) * lifetime, 0.0, step_seconds);
        let outcome = integrate(scene, particle, local_delta, context);
        react(emitter, sub_pool, collisions, effect, sub, particle, &outcome, local_delta);
    }
}

/// Advances one emitter by `step_seconds`: integrates particles, emits by
/// phase, then spawns bursts.
fn step_emitter(
    scene: &Scene,
    emitter: &mut Emitter,
    main: &mut [Particle],
    sub_pool: &mut [Particle],
    collisions: &mut u64,
    step_seconds: f32,
    origin: Vec3,
) {
    let effect = &scene.effects[emitter.effect as usize];
    let sub = if emitter.sub_effect == NO_EFFECT { None } else { Some(&scene.effects[emitter.sub_effect as usize]) };
    let mut context = Context::new(effect);
    context.basis = emitter.basis;
    context.origin = origin;
    context.emitter_velocity = emitter.velocity;
    context.time = emitter.elapsed;
    context.amount_ratio = vecmath::clamp(emitter.amount_ratio, 0.0, 1.0);
    context.wind = scene.wind;
    context.tint = emitter.tint;
    for particle in main.iter_mut() {
        if !particle.active {
            continue;
        }
        let outcome = integrate(scene, particle, step_seconds, &context);
        react(emitter, sub_pool, collisions, effect, sub, particle, &outcome, step_seconds);
    }
    if let Some(sub_effect) = sub {
        let mut sub_context = Context::new(sub_effect);
        sub_context.time = emitter.elapsed;
        sub_context.wind = scene.wind;
        sub_context.tint = emitter.tint;
        for particle in sub_pool.iter_mut().take(emitter.sub_count) {
            if !particle.active {
                continue;
            }
            let outcome = integrate(scene, particle, step_seconds, &sub_context);
            if outcome.collided {
                *collisions += 1;
            }
        }
    }
    let lifetime = vecmath::max(effect.lifetime, MINIMUM_LIFETIME);
    if emitter.emitting {
        let previous = emitter.phase;
        let mut phase = previous + (step_seconds / lifetime);
        let wrapped = phase >= 1.0;
        if wrapped {
            phase = phase - phase.floor();
        }
        if !wrapped {
            let interval = Interval { from: previous, to: phase, remaining: phase, cycle: emitter.cycle };
            emit_interval(scene, emitter, main, sub_pool, collisions, &context, sub, &interval, step_seconds);
        } else {
            let interval = Interval { from: previous, to: 1.0, remaining: 1.0 + phase, cycle: emitter.cycle };
            emit_interval(scene, emitter, main, sub_pool, collisions, &context, sub, &interval, step_seconds);
            if !effect.one_shot {
                let interval = Interval { from: 0.0, to: phase, remaining: phase, cycle: emitter.cycle + 1 };
                emit_interval(scene, emitter, main, sub_pool, collisions, &context, sub, &interval, step_seconds);
            }
        }
        emitter.phase = phase;
        if wrapped {
            emitter.cycle += 1;
            if effect.one_shot {
                emitter.emitting = false;
                emitter.phase = 0.0;
            }
        }
    }
    let mut burst_context = context;
    burst_context.amount_ratio = 1.0;
    while emitter.burst_pending > 0 {
        emitter.burst_pending -= 1;
        let Some(slot) = free_particle(emitter, main) else {
            emitter.burst_pending = 0;
            break;
        };
        let number = 0x68E3_1DA4u32.wrapping_add(emitter.bursts);
        emitter.bursts = emitter.bursts.wrapping_add(1);
        let seed = emission_seed(emitter, number);
        spawn(&mut main[slot], seed, &burst_context);
    }
    emitter.elapsed += step_seconds;
}

/// Counts the live particles of an emitter.
fn finish(emitter: &mut Emitter, main: &[Particle], sub_pool: &[Particle]) {
    let mut alive = 0;
    for particle in main {
        alive += if particle.active { 1 } else { 0 };
    }
    for particle in sub_pool.iter().take(emitter.sub_count) {
        alive += if particle.active { 1 } else { 0 };
    }
    emitter.alive = alive;
}

/// A shower of bouncing sparks that leave dust where they land.
fn make_sparks() -> Effect {
    let mut e = Effect::default();
    e.lifetime = 1.6;
    e.lifetime_randomness = 0.4;
    e.randomness = 0.5;
    e.box_shape = true;
    e.box_extents = Vec3::new(0.2, 0.05, 0.2);
    e.spread_degrees = 35.0;
    e.flatness = 0.2;
    e.inherit_velocity = 0.5;
    e.wind_influence = 0.1;
    e.draw_size = 0.15;
    e.color = Rgba::new(1.0, 0.6, 0.3, 1.0);
    e.color_ramp = Gradient::new(&[(0.0, [1.0, 1.0, 0.8, 1.0]), (0.4, [1.0, 0.5, 0.2, 1.0]), (1.0, [0.3, 0.1, 0.1, 0.0])]);
    e.alpha_curve = Curve::new(false, &[(0.0, 0.0), (0.1, 1.0), (1.0, 0.0)]);
    e.set(Param::InitialVelocity, 4.0, 9.0, Curve::default());
    e.set(Param::LinearAcceleration, 0.0, 1.0, Curve::new(false, &[(0.0, 1.0), (1.0, 0.0)]));
    e.set(Param::Damping, 0.2, 0.6, Curve::new(true, &[(0.0, 1.0), (1.0, 0.2)]));
    e.set(Param::Angle, 0.0, 360.0, Curve::default());
    e.set(Param::AngularVelocity, -90.0, 90.0, Curve::default());
    e.set(Param::Scale, 0.6, 1.2, Curve::new(false, &[(0.0, 0.2), (0.2, 1.0), (1.0, 0.1)]));
    e.set(Param::HueVariation, -0.05, 0.05, Curve::default());
    e.collision = CollisionMode::Rigid;
    e.friction = 0.3;
    e.bounce = 0.5;
    e.use_scale = true;
    e.sub_mode = SubMode::AtCollision;
    e.sub_amount = 2;
    e.sub_effect = 4;
    e
}

/// Buoyant smoke pushed by wind and curl noise that vanishes on contact.
fn make_smoke() -> Effect {
    let mut e = Effect::default();
    e.lifetime = 2.5;
    e.lifetime_randomness = 0.3;
    e.randomness = 1.0;
    e.box_shape = true;
    e.box_extents = Vec3::new(0.5, 0.1, 0.5);
    e.spread_degrees = 25.0;
    e.gravity = Vec3::new(0.0, 0.6, 0.0);
    e.wind_influence = 0.8;
    e.draw_size = 0.8;
    e.color = Rgba::new(0.5, 0.5, 0.55, 1.0);
    e.color_initial_ramp = Gradient::new(&[(0.0, [1.0, 0.9, 0.8, 1.0]), (1.0, [0.8, 0.9, 1.0, 0.7])]);
    e.color_ramp = Gradient::new(&[(0.0, [1.0, 1.0, 1.0, 1.0]), (1.0, [0.4, 0.4, 0.4, 1.0])]);
    e.alpha_curve = Curve::new(true, &[(0.0, 0.0), (0.2, 0.6), (1.0, 0.0)]);
    e.set(Param::InitialVelocity, 0.5, 1.5, Curve::default());
    e.set(Param::RadialAcceleration, -0.2, 0.2, Curve::default());
    e.set(Param::TangentialAcceleration, 0.5, 1.0, Curve::default());
    e.set(Param::Damping, 0.3, 0.8, Curve::default());
    e.set(Param::Scale, 1.0, 2.5, Curve::new(true, &[(0.0, 0.3), (1.0, 1.0)]));
    e.set(Param::TurbulenceInfluence, 0.3, 0.7, Curve::new(false, &[(0.0, 0.2), (1.0, 1.0)]));
    e.turbulence_enabled = true;
    e.turbulence_strength = 1.5;
    e.turbulence_scale = 3.0;
    e.collision = CollisionMode::HideOnContact;
    e
}

/// Debris that erupts at once, tumbles, and bursts into embers where it ends.
fn make_debris() -> Effect {
    let mut e = Effect::default();
    e.lifetime = 2.0;
    e.randomness = 0.3;
    e.explosiveness = 0.7;
    e.box_shape = true;
    e.box_extents = Vec3::new(0.4, 0.4, 0.4);
    e.spread_degrees = 60.0;
    e.draw_size = 0.3;
    e.color = Rgba::new(0.6, 0.5, 0.4, 1.0);
    e.color_ramp = Gradient::new(&[(0.0, [1.0, 1.0, 1.0, 1.0]), (1.0, [0.5, 0.5, 0.5, 1.0])]);
    e.set(Param::InitialVelocity, 3.0, 8.0, Curve::default());
    e.set(Param::OrbitVelocity, 0.0, 0.1, Curve::default());
    e.set(Param::Angle, 0.0, 360.0, Curve::default());
    e.set(Param::AngularVelocity, -180.0, 180.0, Curve::default());
    e.set(Param::Scale, 0.5, 1.5, Curve::default());
    e.collision = CollisionMode::Rigid;
    e.friction = 0.5;
    e.bounce = 0.4;
    e.sub_mode = SubMode::AtEnd;
    e.sub_amount = 3;
    e.sub_keep_velocity = true;
    e.sub_effect = 5;
    e
}

/// A fountain that bounces and sheds dust as it flies.
fn make_fountain() -> Effect {
    let mut e = Effect::default();
    e.lifetime = 1.2;
    e.lifetime_randomness = 0.2;
    e.spread_degrees = 12.0;
    e.draw_size = 0.2;
    e.color = Rgba::new(0.5, 0.7, 1.0, 1.0);
    e.alpha_curve = Curve::new(false, &[(0.0, 1.0), (0.8, 1.0), (1.0, 0.0)]);
    e.set(Param::InitialVelocity, 8.0, 12.0, Curve::default());
    e.set(Param::RadialVelocity, 0.0, 0.5, Curve::default());
    e.collision = CollisionMode::Rigid;
    e.friction = 0.1;
    e.bounce = 0.6;
    e.sub_mode = SubMode::Constant;
    e.sub_frequency = 6.0;
    e.sub_amount = 1;
    e.sub_keep_velocity = true;
    e.sub_effect = 4;
    e
}

/// Short-lived dust puffs.
fn make_dust() -> Effect {
    let mut e = Effect::default();
    e.amount = SUB_POOL_SIZE as u32;
    e.lifetime = 0.4;
    e.spread_degrees = 80.0;
    e.gravity = Vec3::new(0.0, -1.0, 0.0);
    e.draw_size = 0.1;
    e.color = Rgba::new(0.7, 0.65, 0.6, 1.0);
    e.alpha_curve = Curve::new(false, &[(0.0, 1.0), (1.0, 0.0)]);
    e.set(Param::InitialVelocity, 0.5, 2.0, Curve::default());
    e
}

/// Embers that bounce once.
fn make_ember() -> Effect {
    let mut e = Effect::default();
    e.amount = SUB_POOL_SIZE as u32;
    e.lifetime = 0.6;
    e.spread_degrees = 80.0;
    e.gravity = Vec3::new(0.0, -4.0, 0.0);
    e.draw_size = 0.08;
    e.color = Rgba::new(1.0, 0.4, 0.1, 1.0);
    e.set(Param::InitialVelocity, 1.0, 3.0, Curve::default());
    e.collision = CollisionMode::Rigid;
    e.bounce = 0.3;
    e
}

/// The six effects, the ground and boxes, and the wind.
fn make_scene(rng: &mut hash::Rng) -> Scene {
    let mut effects = [make_sparks(), make_smoke(), make_debris(), make_fountain(), make_dust(), make_ember()];
    for effect in effects.iter_mut().take(4) {
        effect.amount = POOL_SIZE as u32;
    }
    let empty = Aabb { min: ZERO, max: ZERO };
    let mut boxes = [empty; BOX_COUNT];
    boxes[0] = Aabb { min: Vec3::new(-1000.0, -1000.0, -1000.0), max: Vec3::new(1000.0, 0.0, 1000.0) };
    for slot in boxes.iter_mut().skip(1) {
        let place = Vec3::random(rng, 0.0, 1.0);
        let half = Vec3::random(rng, 0.5, 2.0);
        let centre = Vec3::new(place.x * 40.0, half.y, place.z * 40.0);
        *slot = Aabb { min: centre - half, max: centre + half };
    }
    Scene { effects, boxes, wind: Vec3::new(3.0, 0.0, 1.0) }
}

/// An emitter at a random place and orientation, moving at a random velocity.
fn make_emitter(index: usize, scene: &Scene, rng: &mut hash::Rng) -> Emitter {
    let mut emitter = Emitter::default();
    emitter.effect = (index % 4) as u32;
    emitter.sub_effect = scene.effects[emitter.effect as usize].sub_effect;
    emitter.first = index * POOL_SIZE;
    emitter.sub_first = index * SUB_POOL_SIZE;
    emitter.sub_count =
        if emitter.sub_effect == NO_EFFECT { 0 } else { scene.effects[emitter.sub_effect as usize].amount as usize };
    emitter.basis = Basis::from_quat(Quat::random(rng));
    let place = Vec3::random(rng, 0.0, 1.0);
    emitter.origin = Vec3::new(place.x * 40.0, 2.0 + (place.y * 4.0), place.z * 40.0);
    emitter.movement = Vec3::random(rng, -3.0, 3.0);
    let red = 0.6 + (rng.unit() * 0.4);
    let green = 0.6 + (rng.unit() * 0.4);
    let blue = 0.6 + (rng.unit() * 0.4);
    emitter.tint = Rgba::new(red, green, blue, 1.0);
    emitter.amount_ratio = 0.8 + (rng.unit() * 0.2);
    emitter.speed_scale = 0.9 + (rng.unit() * 0.2);
    emitter.current_seed = mix_bits((index as u32).wrapping_mul(GOLDEN) ^ mix_bits(1));
    emitter
}

/// Two floats packed by bit pattern into one word.
fn pack_pair(low: f32, high: f32) -> u64 {
    hash::f32_bits(low) as u64 | ((hash::f32_bits(high) as u64) << 32)
}

/// Folds one particle's state into a 64-bit word.
fn fold_particle(p: &Particle) -> u64 {
    let mut word = pack_pair(p.position.x, p.position.y);
    word ^= pack_pair(p.position.z, p.velocity.x).wrapping_mul(0x9E37_79B9_7F4A_7C15);
    word ^= pack_pair(p.velocity.y, p.velocity.z).wrapping_mul(0xC2B2_AE3D_27D4_EB4F);
    word ^= pack_pair(p.size, p.color.r).wrapping_mul(0x1656_67B1_9E37_79F9);
    word ^= pack_pair(p.color.g, p.color.b).wrapping_mul(0x85EB_CA77_C2B2_AE63);
    word ^= pack_pair(p.color.a, if p.active { 1.0 } else { 0.0 }).wrapping_mul(0x27D4_EB2F_1656_67C5);
    word
}

impl Particles {
    /// Runs one emitter's preprocess steps from a fresh start.
    fn preprocess(&mut self, index: usize) {
        let emitter = &mut self.emitters[index];
        let step_seconds = PREPROCESS_SECONDS / PREPROCESS_STEPS as f32;
        let main = &mut self.particles[emitter.first..][..POOL_SIZE];
        let sub_pool = &mut self.sub_particles[emitter.sub_first..][..SUB_POOL_SIZE];
        for _ in 0..PREPROCESS_STEPS {
            let origin = emitter.origin;
            step_emitter(&self.scene, emitter, main, sub_pool, &mut self.collisions, step_seconds, origin);
        }
    }

    /// Advances every emitter by one update in shared substeps.
    fn advance(&mut self) {
        let mut start = [ZERO; EMITTER_COUNT];
        let mut end = [ZERO; EMITTER_COUNT];
        let mut scaled = [0.0f32; EMITTER_COUNT];
        for (index, emitter) in self.emitters.iter_mut().enumerate() {
            start[index] = emitter.origin;
            end[index] = emitter.origin + (emitter.movement * UPDATE_SECONDS);
            emitter.origin = end[index];
            emitter.velocity = (end[index] - start[index]) / UPDATE_SECONDS;
            scaled[index] = UPDATE_SECONDS * vecmath::max(emitter.speed_scale, 0.0);
        }
        for sub in 0..SUBSTEPS {
            let fraction = (sub + 1) as f32 / SUBSTEPS as f32;
            for (index, emitter) in self.emitters.iter_mut().enumerate() {
                let main = &mut self.particles[emitter.first..][..POOL_SIZE];
                let sub_pool = &mut self.sub_particles[emitter.sub_first..][..SUB_POOL_SIZE];
                let origin = lerp(start[index], end[index], fraction);
                step_emitter(&self.scene, emitter, main, sub_pool, &mut self.collisions, scaled[index] / SUBSTEPS as f32, origin);
            }
        }
        for emitter in self.emitters.iter_mut() {
            let main = &self.particles[emitter.first..][..POOL_SIZE];
            let sub_pool = &self.sub_particles[emitter.sub_first..][..SUB_POOL_SIZE];
            finish(emitter, main, sub_pool);
        }
    }
}

impl Case for Particles {
    const NAME: &'static str = "particles";

    /// Builds the scene and emitters, runs their preprocess and keeps that state.
    fn init() -> Particles {
        let mut rng = hash::Rng::new(0x9a7);
        let scene = make_scene(&mut rng);
        let mut emitters = Vec::with_capacity(EMITTER_COUNT);
        for index in 0..EMITTER_COUNT {
            emitters.push(make_emitter(index, &scene, &mut rng));
        }
        let mut state = Particles {
            scene,
            collisions: 0,
            emitters,
            initial_emitters: Vec::new(),
            particles: vec![Particle::default(); EMITTER_COUNT * POOL_SIZE],
            initial_particles: Vec::new(),
            sub_particles: vec![Particle::default(); EMITTER_COUNT * SUB_POOL_SIZE],
            initial_sub_particles: Vec::new(),
        };
        for index in 0..EMITTER_COUNT {
            state.preprocess(index);
        }
        state.initial_emitters = state.emitters.clone();
        state.initial_particles = state.particles.clone();
        state.initial_sub_particles = state.sub_particles.clone();
        state
    }

    /// Resets to the preprocessed state, runs every update and hashes the result.
    fn run(&mut self) -> u64 {
        self.emitters.copy_from_slice(&self.initial_emitters);
        self.particles.copy_from_slice(&self.initial_particles);
        self.sub_particles.copy_from_slice(&self.initial_sub_particles);
        self.collisions = 0;
        for update in 0..UPDATES {
            if update % 2 == 0 {
                for emitter in self.emitters.iter_mut().step_by(3) {
                    emitter.burst_pending += 256;
                }
            }
            self.advance();
        }
        let mut h = hash::add(0, self.collisions);
        for emitter in &self.emitters {
            h = hash::add(h, emitter.alive as u64);
        }
        for p in &self.particles {
            h = hash::add(h, fold_particle(p));
        }
        for p in &self.sub_particles {
            h = hash::add(h, fold_particle(p));
        }
        h
    }
}
