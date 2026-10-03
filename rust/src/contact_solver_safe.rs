//! The engine's soft step contact solver on eight lanes: bundles each colour
//! of the constraint graph into lanes and runs the stages of every substep,
//! in parallel on the current rayon pool. The overflow colour is not supported.
//! The safe variant: body states in relaxed atomics, disjoint
//! writes through `chunks_mut`.
use crate::constraint_graph::{GRAPH_COLOR_COUNT, OVERFLOW_COLOR};
use crate::contact::{Contact, MIN_FRICTION_WEIGHT, Manifold, Softness};
use crate::rigid_body::RigidBody;
use crate::lanes::F8;
use crate::task_pool_safe::{AtomicF32, each_chunk_mut, each_range};
use crate::vecmath::{self, Basis, Quat, Vec3};
use std::ops::{Add, Mul, Sub};

const LANES: usize = 8;
const MAX_POINTS: usize = 4;
const NULL_INDEX: u32 = u32::MAX;
const PI: f32 = std::f32::consts::PI;
const SPECULATIVE_SCALE: f32 = 4.0;
const PARALLEL_THRESHOLD: usize = 512;
const CONTACTS_PER_BLOCK: u32 = 16;
const BODIES_PER_BLOCK: u32 = 256;
const MAX_ROTATION_PER_STEP: f32 = 0.25 * PI;

/// Position and rotation change of a body over one solve.
#[derive(Clone, Copy, Default)]
pub struct BodyDelta {
    pub position: Vec3,
    pub rotation: Quat,
}

/// Step timing, gravity and softness shared by every constraint in a solve.
#[derive(Clone, Copy, Default)]
pub struct SolverContext {
    pub dt: f32,
    pub inv_dt: f32,
    pub h: f32,
    pub inv_h: f32,
    pub substeps: u32,
    pub gravity: Vec3,
    pub contact_softness: Softness,
    pub static_softness: Softness,
    pub push_out_speed: f32,
    pub restitution_threshold: f32,
    pub linear_slop: f32,
}

impl SolverContext {
    /// The engine's step settings at 60 Hz with 4 substeps.
    pub fn engine_default() -> SolverContext {
        let dt = 1.0f32 / 60.0;
        let substeps = 4u32;
        let h = dt / substeps as f32;
        let inv_h = 1.0 / h;
        let contact_hertz = vecmath::min(30.0, 0.125 * inv_h);
        SolverContext {
            dt,
            inv_dt: 1.0 / dt,
            h,
            inv_h,
            substeps,
            gravity: Vec3::new(0.0, -9.81, 0.0),
            contact_softness: crate::contact::make_softness(contact_hertz, 10.0, h),
            static_softness: crate::contact::make_softness(2.0 * contact_hertz, 0.5 * 10.0, h),
            push_out_speed: 3.0,
            restitution_threshold: 1.0,
            linear_slop: 0.005,
        }
    }
}

/// Velocity and rotation centre of a static body.
#[derive(Clone, Copy, Default)]
pub struct KinematicMotion {
    pub velocity: Vec3,
    pub angular_velocity: Vec3,
    pub center: Vec3,
}

/// Everything one solve reads and writes. `active_bodies` lists the slots in
/// `bodies` taking part, in solver order.
pub struct SolverInputs<'a> {
    pub contacts: &'a mut [Contact],
    pub colors: [&'a [u32]; GRAPH_COLOR_COUNT],
    pub body_local: &'a [u32],
    pub active_bodies: &'a [u32],
    pub bodies: &'a mut [RigidBody],
    pub static_motions: &'a [KinematicMotion],
    pub deltas: &'a mut [BodyDelta],
    pub context: SolverContext,
}

/// Per-body velocities and accumulated pose delta, one cache line per body.
/// Constraint stages of one colour read and write them from several threads,
/// so every float is a relaxed atomic.
#[derive(Default)]
#[repr(C, align(64))]
struct BodyState {
    velocity: [AtomicF32; 3],
    angular_velocity: [AtomicF32; 3],
    delta_position: [AtomicF32; 3],
    delta_rotation: [AtomicF32; 4],
}

/// Reads a vector out of atomic cells.
#[inline(always)]
fn load_vec3(cells: &[AtomicF32; 3]) -> Vec3 {
    Vec3::new(cells[0].load(), cells[1].load(), cells[2].load())
}

/// Writes a vector into atomic cells.
#[inline(always)]
fn store_vec3(cells: &[AtomicF32; 3], v: Vec3) {
    cells[0].store(v.x);
    cells[1].store(v.y);
    cells[2].store(v.z);
}

impl BodyState {
    /// At rest with an identity rotation delta.
    fn at_rest() -> BodyState {
        let state = BodyState::default();
        state.delta_rotation[3].store(1.0);
        state
    }

    /// A body that carries the given velocities and no delta.
    fn moving(velocity: Vec3, angular_velocity: Vec3) -> BodyState {
        let state = BodyState::at_rest();
        store_vec3(&state.velocity, velocity);
        store_vec3(&state.angular_velocity, angular_velocity);
        state
    }

    /// The rotation delta as a quaternion.
    #[inline(always)]
    fn rotation_delta(&self) -> Quat {
        let [x, y, z, w] = &self.delta_rotation;
        Quat { x: x.load(), y: y.load(), z: z.load(), w: w.load() }
    }

    /// Replaces the rotation delta.
    #[inline(always)]
    fn set_rotation_delta(&self, q: Quat) {
        let [x, y, z, w] = &self.delta_rotation;
        x.store(q.x);
        y.store(q.y);
        z.store(q.z);
        w.store(q.w);
    }
}

#[derive(Clone, Copy)]
struct BodyProps {
    center: Vec3,
    inverse_mass: f32,
    inverse_inertia: Basis,
    rotation: Quat,
    force: Vec3,
    torque: Vec3,
    linear_damping: f32,
    angular_damping: f32,
}

impl Default for BodyProps {
    /// Immovable, at the origin.
    fn default() -> BodyProps {
        BodyProps {
            center: Vec3::default(),
            inverse_mass: 0.0,
            inverse_inertia: Basis::default() * 0.0,
            rotation: Quat::default(),
            force: Vec3::default(),
            torque: Vec3::default(),
            linear_damping: 0.0,
            angular_damping: 0.0,
        }
    }
}

#[derive(Clone, Copy, Default)]
struct Entry {
    contact: u32,
    body_a: u32,
    body_b: u32,
    b_fixed: bool,
}

/// Symmetric 3x3 matrix stored as its six unique components.
#[derive(Clone, Copy, Default)]
struct Sym3<T> {
    xx: T,
    xy: T,
    xz: T,
    yy: T,
    yz: T,
    zz: T,
}

#[derive(Clone, Copy, Default)]
struct V3W {
    x: F8,
    y: F8,
    z: F8,
}

#[derive(Clone, Copy, Default)]
struct Q4W {
    x: F8,
    y: F8,
    z: F8,
    w: F8,
}

impl Add for V3W {
    type Output = V3W;

    /// Component-wise sum.
    #[inline(always)]
    fn add(self, b: V3W) -> V3W {
        V3W { x: self.x + b.x, y: self.y + b.y, z: self.z + b.z }
    }
}

impl Sub for V3W {
    type Output = V3W;

    /// Component-wise difference.
    #[inline(always)]
    fn sub(self, b: V3W) -> V3W {
        V3W { x: self.x - b.x, y: self.y - b.y, z: self.z - b.z }
    }
}

impl Mul<F8> for V3W {
    type Output = V3W;

    /// Scale by a vector of scalars.
    #[inline(always)]
    fn mul(self, s: F8) -> V3W {
        V3W { x: self.x * s, y: self.y * s, z: self.z * s }
    }
}

impl V3W {
    /// Dot product.
    #[inline(always)]
    fn dot(self, b: V3W) -> F8 {
        self.x * b.x + self.y * b.y + self.z * b.z
    }

    /// Cross product.
    #[inline(always)]
    fn cross(self, b: V3W) -> V3W {
        V3W { x: self.y * b.z - self.z * b.y, y: self.z * b.x - self.x * b.z, z: self.x * b.y - self.y * b.x }
    }

    /// Writes one lane.
    fn set_lane(&mut self, lane: usize, v: Vec3) {
        self.x.set_lane(lane, v.x);
        self.y.set_lane(lane, v.y);
        self.z.set_lane(lane, v.z);
    }

    /// Reads one lane.
    fn lane(&self, lane: usize) -> Vec3 {
        Vec3::new(self.x.lane(lane), self.y.lane(lane), self.z.lane(lane))
    }
}

impl Sym3<F8> {
    /// Symmetric matrix times vector.
    #[inline(always)]
    fn multiply(&self, v: V3W) -> V3W {
        V3W {
            x: self.xx * v.x + self.xy * v.y + self.xz * v.z,
            y: self.xy * v.x + self.yy * v.y + self.yz * v.z,
            z: self.xz * v.x + self.yz * v.y + self.zz * v.z,
        }
    }

    /// Writes one lane.
    fn set_lane(&mut self, lane: usize, m: &Sym3<f32>) {
        self.xx.set_lane(lane, m.xx);
        self.xy.set_lane(lane, m.xy);
        self.xz.set_lane(lane, m.xz);
        self.yy.set_lane(lane, m.yy);
        self.yz.set_lane(lane, m.yz);
        self.zz.set_lane(lane, m.zz);
    }
}

impl Q4W {
    /// Rotate a vector by a unit quaternion.
    #[inline(always)]
    fn rotate(&self, v: V3W) -> V3W {
        let axis = V3W { x: self.x, y: self.y, z: self.z };
        let t = axis.cross(v) * F8::splat(2.0);
        v + t * self.w + axis.cross(t)
    }
}

#[derive(Clone, Copy, Default)]
struct ScalarPoint {
    anchor_a: Vec3,
    anchor_b: Vec3,
    base_separation: f32,
    normal_mass: f32,
    relative_velocity: f32,
    normal_impulse: f32,
    total_normal_impulse: f32,
    peak_normal_impulse: f32,
    lever_arm: f32,
}

/// Solver data of one contact manifold.
#[derive(Clone, Copy, Default)]
struct Scalar {
    body_a: u32,
    body_b: u32,
    contact: u32,
    point_count: u32,
    inverse_mass_a: f32,
    inverse_mass_b: f32,
    inverse_inertia_a: Sym3<f32>,
    inverse_inertia_b: Sym3<f32>,
    normal: Vec3,
    tangent1: Vec3,
    tangent2: Vec3,
    bias_rate: f32,
    mass_scale: f32,
    impulse_scale: f32,
    friction: f32,
    restitution: f32,
    rolling_resistance: f32,
    points: [ScalarPoint; MAX_POINTS],
    friction_anchor_a: Vec3,
    friction_anchor_b: Vec3,
    tangent_mass_xx: f32,
    tangent_mass_xy: f32,
    tangent_mass_yy: f32,
    tangent_impulse_x: f32,
    tangent_impulse_y: f32,
    twist_mass: f32,
    twist_impulse: f32,
    rolling_mass: Sym3<f32>,
    rolling_impulse: Vec3,
}

#[derive(Clone, Copy, Default)]
struct PointW {
    anchor_a: V3W,
    anchor_b: V3W,
    base_separation: F8,
    normal_mass: F8,
    relative_velocity: F8,
    normal_impulse: F8,
    total_normal_impulse: F8,
    peak_normal_impulse: F8,
    lever_arm: F8,
}

/// Eight contact manifolds of one colour, one per lane.
#[derive(Clone, Copy, Default)]
struct Bundle {
    body_a: [u32; LANES],
    body_b: [u32; LANES],
    contact: [u32; LANES],
    point_count: u32,
    inverse_mass_a: F8,
    inverse_mass_b: F8,
    inverse_inertia_a: Sym3<F8>,
    inverse_inertia_b: Sym3<F8>,
    normal: V3W,
    tangent1: V3W,
    tangent2: V3W,
    bias_rate: F8,
    mass_scale: F8,
    impulse_scale: F8,
    friction: F8,
    restitution: F8,
    rolling_resistance: F8,
    points: [PointW; MAX_POINTS],
    friction_anchor_a: V3W,
    friction_anchor_b: V3W,
    tangent_mass_xx: F8,
    tangent_mass_xy: F8,
    tangent_mass_yy: F8,
    tangent_impulse_x: F8,
    tangent_impulse_y: F8,
    twist_mass: F8,
    twist_impulse: F8,
    rolling_mass: Sym3<F8>,
    rolling_impulse: V3W,
}

impl Bundle {
    /// Copies every value field of a scalar constraint into one lane.
    fn pack_lane(&mut self, lane: usize, c: &Scalar) {
        self.inverse_mass_a.set_lane(lane, c.inverse_mass_a);
        self.inverse_mass_b.set_lane(lane, c.inverse_mass_b);
        self.inverse_inertia_a.set_lane(lane, &c.inverse_inertia_a);
        self.inverse_inertia_b.set_lane(lane, &c.inverse_inertia_b);
        self.normal.set_lane(lane, c.normal);
        self.tangent1.set_lane(lane, c.tangent1);
        self.tangent2.set_lane(lane, c.tangent2);
        self.bias_rate.set_lane(lane, c.bias_rate);
        self.mass_scale.set_lane(lane, c.mass_scale);
        self.impulse_scale.set_lane(lane, c.impulse_scale);
        self.friction.set_lane(lane, c.friction);
        self.restitution.set_lane(lane, c.restitution);
        self.rolling_resistance.set_lane(lane, c.rolling_resistance);
        for (pw, pc) in self.points.iter_mut().zip(c.points.iter()) {
            pw.anchor_a.set_lane(lane, pc.anchor_a);
            pw.anchor_b.set_lane(lane, pc.anchor_b);
            pw.base_separation.set_lane(lane, pc.base_separation);
            pw.normal_mass.set_lane(lane, pc.normal_mass);
            pw.relative_velocity.set_lane(lane, pc.relative_velocity);
            pw.normal_impulse.set_lane(lane, pc.normal_impulse);
            pw.total_normal_impulse.set_lane(lane, pc.total_normal_impulse);
            pw.peak_normal_impulse.set_lane(lane, pc.peak_normal_impulse);
            pw.lever_arm.set_lane(lane, pc.lever_arm);
        }
        self.friction_anchor_a.set_lane(lane, c.friction_anchor_a);
        self.friction_anchor_b.set_lane(lane, c.friction_anchor_b);
        self.tangent_mass_xx.set_lane(lane, c.tangent_mass_xx);
        self.tangent_mass_xy.set_lane(lane, c.tangent_mass_xy);
        self.tangent_mass_yy.set_lane(lane, c.tangent_mass_yy);
        self.tangent_impulse_x.set_lane(lane, c.tangent_impulse_x);
        self.tangent_impulse_y.set_lane(lane, c.tangent_impulse_y);
        self.twist_mass.set_lane(lane, c.twist_mass);
        self.twist_impulse.set_lane(lane, c.twist_impulse);
        self.rolling_mass.set_lane(lane, &c.rolling_mass);
        self.rolling_impulse.set_lane(lane, c.rolling_impulse);
    }

    /// Writes one lane's impulses back to its contact.
    fn store_lane(&self, lane: usize, contact: &mut Contact) {
        for (point, p) in contact.manifold.active_mut().iter_mut().zip(self.points.iter()) {
            point.normal_impulse = p.normal_impulse.lane(lane);
            point.total_normal_impulse = p.total_normal_impulse.lane(lane);
            point.peak_normal_impulse = p.peak_normal_impulse.lane(lane);
            point.relative_velocity = p.relative_velocity.lane(lane);
        }
        let friction = &mut contact.friction_impulses;
        friction.tangent_x = self.tangent_impulse_x.lane(lane);
        friction.tangent_y = self.tangent_impulse_y.lane(lane);
        friction.twist = self.twist_impulse.lane(lane);
        friction.rolling = self.rolling_impulse.lane(lane);
    }
}

/// Body velocities and pose deltas, lane-wise.
#[derive(Clone, Copy)]
struct BodyRefs {
    velocity: V3W,
    angular_velocity: V3W,
    delta_position: V3W,
    delta_rotation: Q4W,
}

struct BodyPair {
    a: BodyRefs,
    b: BodyRefs,
}

#[derive(Clone, Copy)]
struct AnchorPair {
    a: V3W,
    b: V3W,
}

/// Body states as the constraint stages see them: shared by every thread of
/// a stage, read and written through their atomics.
type States<'a> = &'a [BodyState];

/// Builds a vector whose lane `i` is `f(i)`.
#[inline(always)]
fn lanes_from(f: impl Fn(usize) -> f32) -> F8 {
    let mut lanes = F8::splat(0.0);
    for lane in 0..LANES {
        lanes.set_lane(lane, f(lane));
    }
    lanes
}

/// Loads the bodies named by each lane.
#[inline(always)]
fn gather(states: States, index: &[u32; LANES]) -> BodyRefs {
    let s: [&BodyState; LANES] = std::array::from_fn(|lane| &states[index[lane] as usize]);
    let vec3 = |cells: fn(&BodyState) -> &[AtomicF32; 3]| V3W {
        x: lanes_from(|lane| cells(s[lane])[0].load()),
        y: lanes_from(|lane| cells(s[lane])[1].load()),
        z: lanes_from(|lane| cells(s[lane])[2].load()),
    };
    BodyRefs {
        velocity: vec3(|b| &b.velocity),
        angular_velocity: vec3(|b| &b.angular_velocity),
        delta_position: vec3(|b| &b.delta_position),
        delta_rotation: Q4W {
            x: lanes_from(|lane| s[lane].delta_rotation[0].load()),
            y: lanes_from(|lane| s[lane].delta_rotation[1].load()),
            z: lanes_from(|lane| s[lane].delta_rotation[2].load()),
            w: lanes_from(|lane| s[lane].delta_rotation[3].load()),
        },
    }
}

/// Stores velocities back, skipping read-only lanes.
#[inline(always)]
fn scatter(states: States, writable: u32, index: &[u32; LANES], body: &BodyRefs) {
    let (vx, vy, vz) = (body.velocity.x.to_array(), body.velocity.y.to_array(), body.velocity.z.to_array());
    let (wx, wy, wz) = (body.angular_velocity.x.to_array(), body.angular_velocity.y.to_array(), body.angular_velocity.z.to_array());
    for (lane, &slot) in index.iter().enumerate() {
        if slot >= writable {
            continue;
        }
        let state = &states[slot as usize];
        store_vec3(&state.velocity, Vec3::new(vx[lane], vy[lane], vz[lane]));
        store_vec3(&state.angular_velocity, Vec3::new(wx[lane], wy[lane], wz[lane]));
    }
}

/// Loads both bodies of every lane.
fn gather_pair(states: States, c: &Bundle) -> BodyPair {
    BodyPair { a: gather(states, &c.body_a), b: gather(states, &c.body_b) }
}

/// Stores both bodies of every lane.
fn scatter_pair(states: States, writable: u32, c: &Bundle, bodies: &BodyPair) {
    scatter(states, writable, &c.body_a, &bodies.a);
    scatter(states, writable, &c.body_b, &bodies.b);
}

/// The anchors of one point on both bodies.
fn point_anchors(p: &PointW) -> AnchorPair {
    AnchorPair { a: p.anchor_a, b: p.anchor_b }
}

/// Equal and opposite impulse at a contact anchor.
#[inline(always)]
fn apply_impulse(c: &Bundle, bodies: &mut BodyPair, r: AnchorPair, impulse: V3W) {
    let a = &mut bodies.a;
    a.velocity = a.velocity - impulse * c.inverse_mass_a;
    a.angular_velocity = a.angular_velocity - c.inverse_inertia_a.multiply(r.a.cross(impulse));
    let b = &mut bodies.b;
    b.velocity = b.velocity + impulse * c.inverse_mass_b;
    b.angular_velocity = b.angular_velocity + c.inverse_inertia_b.multiply(r.b.cross(impulse));
}

/// Equal and opposite angular impulse.
#[inline(always)]
fn apply_angular(c: &Bundle, bodies: &mut BodyPair, impulse: V3W) {
    bodies.a.angular_velocity = bodies.a.angular_velocity - c.inverse_inertia_a.multiply(impulse);
    bodies.b.angular_velocity = bodies.b.angular_velocity + c.inverse_inertia_b.multiply(impulse);
}

/// Relative anchor velocity of B against A along a direction.
#[inline(always)]
fn relative_velocity_along(bodies: &BodyPair, r: AnchorPair, direction: V3W) -> F8 {
    let va = bodies.a.velocity + bodies.a.angular_velocity.cross(r.a);
    let vb = bodies.b.velocity + bodies.b.angular_velocity.cross(r.b);
    (vb - va).dot(direction)
}

/// Applies last step's accumulated impulses.
fn warm_start_constraint(c: &mut Bundle, states: States, writable: u32) {
    let mut bodies = gather_pair(states, c);
    for p in &c.points[..c.point_count as usize] {
        apply_impulse(c, &mut bodies, point_anchors(p), c.normal * p.normal_impulse);
    }
    let tangential = c.tangent1 * c.tangent_impulse_x + c.tangent2 * c.tangent_impulse_y;
    apply_impulse(c, &mut bodies, AnchorPair { a: c.friction_anchor_a, b: c.friction_anchor_b }, tangential);
    apply_angular(c, &mut bodies, c.normal * c.twist_impulse + c.rolling_impulse);
    scatter_pair(states, writable, c, &bodies);
}

/// Tangential, twist and rolling friction.
#[inline(always)]
fn solve_friction(c: &mut Bundle, bodies: &mut BodyPair, total_normal_impulse: F8, twist_limit: F8) {
    let zero = F8::splat(0.0);
    let one = F8::splat(1.0);
    let anchors = AnchorPair { a: c.friction_anchor_a, b: c.friction_anchor_b };
    let max_friction = c.friction * total_normal_impulse;
    let vt1 = relative_velocity_along(bodies, anchors, c.tangent1);
    let vt2 = relative_velocity_along(bodies, anchors, c.tangent2);
    let delta_x = -(c.tangent_mass_xx * vt1 + c.tangent_mass_xy * vt2);
    let delta_y = -(c.tangent_mass_xy * vt1 + c.tangent_mass_yy * vt2);
    let mut total_x = c.tangent_impulse_x + delta_x;
    let mut total_y = c.tangent_impulse_y + delta_y;
    let magnitude = (total_x * total_x + total_y * total_y).sqrt();
    let clamped = magnitude.greater(max_friction).and(magnitude.greater(zero));
    let scale = clamped.select(max_friction / magnitude, one);
    total_x = total_x * scale;
    total_y = total_y * scale;
    let impulse = c.tangent1 * (total_x - c.tangent_impulse_x) + c.tangent2 * (total_y - c.tangent_impulse_y);
    c.tangent_impulse_x = total_x;
    c.tangent_impulse_y = total_y;
    apply_impulse(c, bodies, anchors, impulse);

    let max_twist = c.friction * twist_limit;
    let twist_velocity = (bodies.b.angular_velocity - bodies.a.angular_velocity).dot(c.normal);
    let twist = (c.twist_impulse - c.twist_mass * twist_velocity).max(-max_twist).min(max_twist);
    let twist_delta = twist - c.twist_impulse;
    c.twist_impulse = twist;
    apply_angular(c, bodies, c.normal * twist_delta);

    let max_rolling = c.rolling_resistance * total_normal_impulse;
    let relative = bodies.b.angular_velocity - bodies.a.angular_velocity;
    let mut rolling = c.rolling_impulse - c.rolling_mass.multiply(relative);
    let rolling_magnitude = rolling.dot(rolling).sqrt();
    let rolling_clamped = rolling_magnitude.greater(max_rolling).and(rolling_magnitude.greater(zero));
    let rolling_scale = rolling_clamped.select(max_rolling / rolling_magnitude, one);
    rolling = rolling * rolling_scale;
    let rolling_delta = rolling - c.rolling_impulse;
    c.rolling_impulse = rolling;
    apply_angular(c, bodies, rolling_delta);
}

/// Normal constraint of every point; friction too on the relax pass.
fn solve_constraint<const USE_BIAS: bool>(c: &mut Bundle, states: States, writable: u32, context: &SolverContext) {
    let mut bodies = gather_pair(states, c);
    let zero = F8::splat(0.0);
    let one = F8::splat(1.0);
    let inv_h = F8::splat(context.inv_h);
    let push_out = F8::splat(-context.push_out_speed);
    let dp = bodies.b.delta_position - bodies.a.delta_position;

    let mut total_normal_impulse = F8::splat(0.0);
    let mut total_twist_limit = F8::splat(0.0);
    for i in 0..c.point_count as usize {
        let p = c.points[i];
        let ds = dp + bodies.b.delta_rotation.rotate(p.anchor_b) - bodies.a.delta_rotation.rotate(p.anchor_a);
        let s = ds.dot(c.normal) + p.base_separation;
        let separated = s.greater(zero);
        let mut bias = s * inv_h;
        let mut mass_scale = one;
        let mut impulse_scale = zero;
        if USE_BIAS {
            let soft_bias = (c.mass_scale * c.bias_rate * s).max(push_out);
            bias = separated.select(bias, soft_bias);
            mass_scale = separated.select(one, c.mass_scale);
            impulse_scale = separated.select(zero, c.impulse_scale);
        } else {
            bias = separated.select(bias, zero);
        }
        let vn = relative_velocity_along(&bodies, point_anchors(&p), c.normal);
        let mut delta_impulse = -p.normal_mass * (mass_scale * vn + bias) - impulse_scale * p.normal_impulse;
        let new_impulse = (p.normal_impulse + delta_impulse).max(zero);
        delta_impulse = new_impulse - p.normal_impulse;
        let point = &mut c.points[i];
        point.normal_impulse = new_impulse;
        point.total_normal_impulse = point.total_normal_impulse + new_impulse;
        point.peak_normal_impulse = point.peak_normal_impulse.max(new_impulse);
        total_normal_impulse = total_normal_impulse + new_impulse;
        total_twist_limit = total_twist_limit + p.lever_arm * new_impulse;
        apply_impulse(c, &mut bodies, point_anchors(&p), c.normal * delta_impulse);
    }

    if !USE_BIAS {
        solve_friction(c, &mut bodies, total_normal_impulse, total_twist_limit);
    }

    scatter_pair(states, writable, c, &bodies);
}

/// Restitution for points that approached faster than the threshold.
fn restitution_constraint(c: &mut Bundle, states: States, writable: u32, context: &SolverContext) {
    let zero = F8::splat(0.0);
    let threshold = F8::splat(-context.restitution_threshold);
    let bouncy = c.restitution.greater(zero);
    if !bouncy.any() {
        return;
    }
    let mut bodies = gather_pair(states, c);
    for i in 0..c.point_count as usize {
        let p = c.points[i];
        let active = bouncy.and(p.relative_velocity.less_equal(threshold)).and(p.total_normal_impulse.not_equal(zero));
        let vn = relative_velocity_along(&bodies, point_anchors(&p), c.normal);
        let mut impulse = -p.normal_mass * (vn + c.restitution * p.relative_velocity);
        let new_impulse = (p.normal_impulse + impulse).max(zero);
        impulse = active.select(new_impulse - p.normal_impulse, zero);
        let point = &mut c.points[i];
        point.normal_impulse = active.select(new_impulse, p.normal_impulse);
        point.total_normal_impulse = p.total_normal_impulse + active.select(new_impulse, zero);
        point.peak_normal_impulse = active.select(p.peak_normal_impulse.max(new_impulse), p.peak_normal_impulse);
        apply_impulse(c, &mut bodies, point_anchors(&p), c.normal * impulse);
    }
    scatter_pair(states, writable, c, &bodies);
}

/// Some unit vector perpendicular to the unit vector `n`.
fn perpendicular(n: Vec3) -> Vec3 {
    if n.x.abs() > 0.57735 {
        return Vec3::new(n.y, -n.x, 0.0).normalize();
    }
    Vec3::new(0.0, n.z, -n.y).normalize()
}

/// Upper triangle of a symmetric basis.
fn sym_from_basis(m: &Basis) -> Sym3<f32> {
    Sym3 { xx: m.x.x, xy: m.x.y, xz: m.x.z, yy: m.y.y, yz: m.y.z, zz: m.z.z }
}

/// Scalar view of one body while preparing constraints.
struct PrepBody {
    velocity: Vec3,
    angular_velocity: Vec3,
    inverse_mass: f32,
    inverse_inertia: Basis,
    center: Vec3,
}

/// Inverse of the combined inverse mass along a direction at two anchors.
fn effective_mass(a: &PrepBody, b: &PrepBody, ra: Vec3, rb: Vec3, direction: Vec3) -> f32 {
    let rna = ra.cross(direction);
    let rnb = rb.cross(direction);
    let k = a.inverse_mass + b.inverse_mass + rna.dot(a.inverse_inertia * rna) + rnb.dot(b.inverse_inertia * rnb);
    if k > 0.0 { 1.0 / k } else { 0.0 }
}

/// Relative anchor velocity of B against A along a direction.
fn prep_relative_velocity(a: &PrepBody, b: &PrepBody, ra: Vec3, rb: Vec3, direction: Vec3) -> f32 {
    let va = a.velocity + a.angular_velocity.cross(ra);
    let vb = b.velocity + b.angular_velocity.cross(rb);
    (vb - va).dot(direction)
}

/// The tangent directions a friction constraint works along.
struct Tangents {
    first: Vec3,
    second: Vec3,
}

/// Friction anchors, lever arms and tangent, twist and rolling masses.
fn prepare_friction(c: &mut Scalar, manifold: &Manifold, bodies: (&PrepBody, &PrepBody), tangents: &Tangents, rolling_resistance: f32, speculative: f32) {
    let (a, b) = bodies;
    let count = manifold.point_count as usize;
    let mut center_a = Vec3::default();
    let mut center_b = Vec3::default();
    let mut total_weight = 0.0f32;
    let inv_tau = 1.0 / speculative;
    for (point, pc) in manifold.active().iter().zip(c.points[..count].iter()) {
        let weight = vecmath::clamp(2.0 - point.separation * inv_tau, MIN_FRICTION_WEIGHT, 1.0);
        center_a = center_a + pc.anchor_a * weight;
        center_b = center_b + pc.anchor_b * weight;
        total_weight += weight;
    }
    let inv_weight = if total_weight > 0.0 { 1.0 / total_weight } else { 0.0 };
    let anchor_a = center_a * inv_weight;
    let anchor_b = center_b * inv_weight;
    c.friction_anchor_a = anchor_a;
    c.friction_anchor_b = anchor_b;

    for pc in c.points[..count].iter_mut() {
        pc.lever_arm = (pc.anchor_a - anchor_a).length();
    }

    let rta1 = anchor_a.cross(tangents.first);
    let rta2 = anchor_a.cross(tangents.second);
    let rtb1 = anchor_b.cross(tangents.first);
    let rtb2 = anchor_b.cross(tangents.second);
    let inv_mass = a.inverse_mass + b.inverse_mass;
    let kxx = inv_mass + rta1.dot(a.inverse_inertia * rta1) + rtb1.dot(b.inverse_inertia * rtb1);
    let kyy = inv_mass + rta2.dot(a.inverse_inertia * rta2) + rtb2.dot(b.inverse_inertia * rtb2);
    let kxy = rta1.dot(a.inverse_inertia * rta2) + rtb1.dot(b.inverse_inertia * rtb2);
    let tangent_det = kxx * kyy - kxy * kxy;
    if tangent_det != 0.0 {
        let inv = 1.0 / tangent_det;
        c.tangent_mass_xx = kyy * inv;
        c.tangent_mass_xy = -kxy * inv;
        c.tangent_mass_yy = kxx * inv;
    }

    let normal = manifold.normal;
    let angular = a.inverse_inertia + b.inverse_inertia;
    let twist = normal.dot(angular * normal);
    c.twist_mass = if twist > 0.0 { 1.0 / twist } else { 0.0 };
    if rolling_resistance > 0.0 && angular.determinant() > 0.0 {
        c.rolling_mass = sym_from_basis(&angular.inverse());
    }
}

/// Body data the preparation reads, fixed for the whole preparation.
struct PrepSource<'a> {
    states: States<'a>,
    props: &'a [BodyProps],
    context: &'a SolverContext,
}

impl PrepSource<'_> {
    /// Scalar view of the body in a slot.
    fn body(&self, slot: u32) -> PrepBody {
        let props = &self.props[slot as usize];
        let state = &self.states[slot as usize];
        PrepBody {
            velocity: load_vec3(&state.velocity),
            angular_velocity: load_vec3(&state.angular_velocity),
            inverse_mass: props.inverse_mass,
            inverse_inertia: props.inverse_inertia,
            center: props.center,
        }
    }

    /// Builds the scalar constraint of one contact.
    fn prepare_entry(&self, entry: &Entry, contact: &Contact) -> Scalar {
        let context = self.context;
        let a = self.body(entry.body_a);
        let b = self.body(entry.body_b);

        let manifold = &contact.manifold;
        let mut c = Scalar { body_a: entry.body_a, body_b: entry.body_b, contact: entry.contact, point_count: manifold.point_count, ..Scalar::default() };
        let n = manifold.normal;
        let tangent1 = perpendicular(n);
        let tangent2 = tangent1.cross(n);
        c.normal = n;
        c.tangent1 = tangent1;
        c.tangent2 = tangent2;
        let softness = if entry.b_fixed { context.static_softness } else { context.contact_softness };
        c.bias_rate = softness.bias_rate;
        c.mass_scale = softness.mass_scale;
        c.impulse_scale = softness.impulse_scale;
        c.friction = contact.friction;
        c.restitution = contact.restitution;
        c.rolling_resistance = contact.rolling_resistance;
        c.inverse_mass_a = a.inverse_mass;
        c.inverse_mass_b = b.inverse_mass;
        c.inverse_inertia_a = sym_from_basis(&a.inverse_inertia);
        c.inverse_inertia_b = sym_from_basis(&b.inverse_inertia);

        for (point, pc) in manifold.active().iter().zip(c.points.iter_mut()) {
            let anchor_a = point.point - a.center;
            let anchor_b = point.point - b.center;
            pc.anchor_a = anchor_a;
            pc.anchor_b = anchor_b;
            pc.base_separation = point.separation - (anchor_b - anchor_a).dot(n);
            pc.normal_mass = effective_mass(&a, &b, anchor_a, anchor_b, n);
            pc.relative_velocity = prep_relative_velocity(&a, &b, anchor_a, anchor_b, n);
            pc.normal_impulse = point.normal_impulse;
        }
        let tangents = Tangents { first: tangent1, second: tangent2 };
        prepare_friction(&mut c, &contact.manifold, (&a, &b), &tangents, contact.rolling_resistance, SPECULATIVE_SCALE * context.linear_slop);
        let impulses = &contact.friction_impulses;
        c.tangent_impulse_x = impulses.tangent_x;
        c.tangent_impulse_y = impulses.tangent_y;
        c.twist_impulse = impulses.twist;
        c.rolling_impulse = impulses.rolling;
        c
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum StageKind {
    IntegrateVelocities,
    WarmStart,
    SolveBiased,
    IntegratePositions,
    SolveRelax,
    Restitution,
}

/// A run of blocks of one kind, executed between barriers.
#[derive(Clone, Copy)]
struct Stage {
    kind: StageKind,
    begin: u32,
    end: u32,
    grain: u32,
}

/// Applies the velocity half of a substep to the bodies in `begin..end`.
fn integrate_velocities(states: States, props: &[BodyProps], context: &SolverContext, begin: usize, end: usize) {
    let h = context.h;
    for (state, props) in states[begin..end].iter().zip(&props[begin..end]) {
        let v = (load_vec3(&state.velocity) + props.force * (props.inverse_mass * h)) * props.linear_damping;
        let w = (load_vec3(&state.angular_velocity) + props.inverse_inertia * props.torque * h) * props.angular_damping;
        store_vec3(&state.velocity, v);
        store_vec3(&state.angular_velocity, w);
    }
}

/// Moves the bodies of one chunk by their velocities, the first of which is
/// body `offset`, and accumulates the pose delta.
fn integrate_positions(states: States, props: &mut [BodyProps], context: &SolverContext, offset: usize) {
    let h = context.h;
    let max_angular_speed = MAX_ROTATION_PER_STEP * context.inv_dt;
    for (state, props) in states[offset..].iter().zip(props.iter_mut()) {
        let angular_speed = load_vec3(&state.angular_velocity).length();
        if angular_speed > max_angular_speed {
            store_vec3(&state.angular_velocity, load_vec3(&state.angular_velocity) * (max_angular_speed / angular_speed));
        }
        let step = load_vec3(&state.velocity) * h;
        let turn = load_vec3(&state.angular_velocity) * h;
        props.center = props.center + step;
        props.rotation = props.rotation.integrate(turn);
        store_vec3(&state.delta_position, load_vec3(&state.delta_position) + step);
        state.set_rotation_delta(state.rotation_delta().integrate(turn));
    }
}

/// Runs the constraint stage `kind` over a run of bundles.
fn solve_bundles(kind: StageKind, bundles: &mut [Bundle], states: States, writable: u32, context: &SolverContext) {
    for c in bundles {
        match kind {
            StageKind::WarmStart => warm_start_constraint(c, states, writable),
            StageKind::SolveBiased => solve_constraint::<true>(c, states, writable, context),
            StageKind::SolveRelax => solve_constraint::<false>(c, states, writable, context),
            _ => restitution_constraint(c, states, writable, context),
        }
    }
}

#[derive(Default)]
pub struct ContactSolver {
    states: Vec<BodyState>,
    props: Vec<BodyProps>,
    static_lookup: Vec<u32>,
    writable_count: u32,
    dummy_index: u32,
    entries: Vec<Entry>,
    entry_colors: Vec<u8>,
    scalars: Vec<Scalar>,
    order: Vec<u32>,
    bundles: Vec<Bundle>,
    contact_lane: Vec<u32>,
    color_start: [u32; GRAPH_COLOR_COUNT + 1],
    color_begin: [u32; GRAPH_COLOR_COUNT],
    color_end: [u32; GRAPH_COLOR_COUNT],
    stages: Vec<Stage>,
}

impl ContactSolver {
    /// Solves every contact for one step and writes velocities, poses,
    /// impulses and deltas back.
    pub fn solve(&mut self, inputs: &mut SolverInputs) {
        self.build_bodies(inputs);
        self.prepare_constraints(inputs);
        self.plan_stages(&inputs.context);
        self.run_stages(&inputs.context);
        self.store_results(inputs);
    }

    /// Whether the work is large enough to be worth spreading over the pool.
    fn parallel(&self) -> bool {
        self.entries.len() >= PARALLEL_THRESHOLD && rayon::current_num_threads() > 1
    }

    /// Fills body states and properties from the active bodies.
    fn build_bodies(&mut self, inputs: &SolverInputs) {
        let context = &inputs.context;
        let count = inputs.active_bodies.len();
        self.writable_count = count as u32;
        self.dummy_index = count as u32;
        self.states.clear();
        self.states.resize_with(count + 1, BodyState::at_rest);
        self.props.clear();
        self.props.resize(count + 1, BodyProps::default());
        self.static_lookup.clear();
        self.static_lookup.resize(inputs.static_motions.len(), NULL_INDEX);
        for ((&slot, state), props) in inputs.active_bodies.iter().zip(self.states.iter()).zip(self.props.iter_mut()) {
            let body = &inputs.bodies[slot as usize];
            store_vec3(&state.velocity, body.linear_velocity);
            store_vec3(&state.angular_velocity, body.angular_velocity);
            props.center = body.world_center_of_mass();
            props.inverse_mass = body.inverse_mass;
            props.inverse_inertia = body.inverse_inertia_world;
            props.rotation = body.rotation;
            props.force = body.applied_force + body.constant_force + context.gravity * (body.gravity_scale * body.mass);
            props.torque = body.applied_torque + body.constant_torque;
            props.linear_damping = 1.0 / (1.0 + context.h * body.linear_damp);
            props.angular_damping = 1.0 / (1.0 + context.h * body.angular_damp);
        }
    }

    /// Slot of a static body, created read-only on first use.
    fn read_only_static(&mut self, static_motions: &[KinematicMotion], fixed_index: u32) -> u32 {
        let Some(motion) = static_motions.get(fixed_index as usize) else {
            return self.dummy_index;
        };
        let entry = self.static_lookup[fixed_index as usize];
        if entry != NULL_INDEX {
            return entry;
        }
        let entry = self.states.len() as u32;
        self.states.push(BodyState::moving(motion.velocity, motion.angular_velocity));
        self.props.push(BodyProps { center: motion.center, ..BodyProps::default() });
        self.static_lookup[fixed_index as usize] = entry;
        entry
    }

    /// Lists contacts colour by colour with their body slots.
    fn collect_entries(&mut self, inputs: &SolverInputs) {
        self.entries.clear();
        self.entry_colors.clear();
        for (color, list) in inputs.colors.iter().enumerate() {
            for &index in list.iter() {
                if color == OVERFLOW_COLOR {
                    eprintln!("solver: the overflow colour is not supported");
                    std::process::exit(4);
                }
                let contact = &inputs.contacts[index as usize];
                let body_a = inputs.body_local[contact.shape_a.body as usize];
                let b_fixed = contact.shape_b.is_static;
                let body_b = if b_fixed { self.read_only_static(inputs.static_motions, contact.shape_b.body) } else { inputs.body_local[contact.shape_b.body as usize] };
                self.entries.push(Entry { contact: index, body_a, body_b, b_fixed });
                self.entry_colors.push(color as u8);
            }
        }
    }

    /// Prepares every constraint, in parallel when large enough, and bundles
    /// them.
    fn prepare_constraints(&mut self, inputs: &SolverInputs) {
        self.collect_entries(inputs);
        self.scalars.resize(self.entries.len(), Scalar::default());
        let parallel = self.parallel();
        let source = PrepSource { states: &self.states, props: &self.props, context: &inputs.context };
        let (entries, contacts) = (&self.entries, &*inputs.contacts);
        each_chunk_mut(parallel, &mut self.scalars, 128, |offset, out| {
            for (entry, scalar) in entries[offset..].iter().zip(out.iter_mut()) {
                *scalar = source.prepare_entry(entry, &contacts[entry.contact as usize]);
            }
        });
        self.bundle_colors(parallel);
    }

    /// Sorts constraints by colour and packs each colour into bundles.
    fn bundle_colors(&mut self, parallel: bool) {
        let mut counts = [0u32; GRAPH_COLOR_COUNT + 1];
        for &color in &self.entry_colors {
            counts[color as usize + 1] += 1;
        }
        for color in 0..GRAPH_COLOR_COUNT {
            counts[color + 1] += counts[color];
        }
        self.color_start = counts;
        self.order.resize(self.entries.len(), 0);
        for (i, &color) in self.entry_colors.iter().enumerate() {
            self.order[counts[color as usize] as usize] = i as u32;
            counts[color as usize] += 1;
        }

        let width = LANES as u32;
        let mut bundles = 0u32;
        for color in 0..OVERFLOW_COLOR {
            let members = self.color_start[color + 1] - self.color_start[color];
            self.color_begin[color] = bundles;
            bundles += members.div_ceil(width);
            self.color_end[color] = bundles;
        }

        self.bundles.resize(bundles as usize, Bundle::default());
        let (color_start, color_begin, color_end) = (&self.color_start, &self.color_begin, &self.color_end);
        let (scalars, order, dummy_index) = (&self.scalars, &self.order, self.dummy_index);
        each_chunk_mut(parallel && bundles as usize >= PARALLEL_THRESHOLD / LANES, &mut self.bundles, 32, |offset, out| {
            for (bundle_index, target) in (offset as u32..).zip(out.iter_mut()) {
                let mut color = 0;
                while color_end[color] <= bundle_index {
                    color += 1;
                }
                let first = color_start[color] + (bundle_index - color_begin[color]) * width;
                let last = (first + width).min(color_start[color + 1]);
                let mut bundle = Bundle { body_a: [dummy_index; LANES], body_b: [dummy_index; LANES], contact: [NULL_INDEX; LANES], ..Bundle::default() };
                for member in first..last {
                    let lane = (member - first) as usize;
                    let source = &scalars[order[member as usize] as usize];
                    bundle.body_a[lane] = source.body_a;
                    bundle.body_b[lane] = source.body_b;
                    bundle.contact[lane] = source.contact;
                    bundle.point_count = bundle.point_count.max(source.point_count);
                    bundle.pack_lane(lane, source);
                }
                *target = bundle;
            }
        });
    }

    /// Appends a stage, ignoring empty ranges.
    fn add_stage(&mut self, kind: StageKind, begin: u32, end: u32, grain: u32) {
        if begin == end {
            return;
        }
        self.stages.push(Stage { kind, begin, end, grain });
    }

    /// Adds one stage per colour for a constraint kind.
    fn add_constraint_stages(&mut self, kind: StageKind) {
        for color in 0..OVERFLOW_COLOR {
            self.add_stage(kind, self.color_begin[color], self.color_end[color], 1.max(CONTACTS_PER_BLOCK / LANES as u32));
        }
    }

    /// Builds the stage list for every substep plus restitution.
    fn plan_stages(&mut self, context: &SolverContext) {
        self.stages.clear();
        for _ in 0..context.substeps {
            self.add_stage(StageKind::IntegrateVelocities, 0, self.writable_count, BODIES_PER_BLOCK);
            self.add_constraint_stages(StageKind::WarmStart);
            self.add_constraint_stages(StageKind::SolveBiased);
            self.add_stage(StageKind::IntegratePositions, 0, self.writable_count, BODIES_PER_BLOCK);
            self.add_constraint_stages(StageKind::SolveRelax);
        }
        self.add_constraint_stages(StageKind::Restitution);
    }

    /// Runs the stages in order, each one spread over the pool when there are
    /// enough contacts; the pool's join between stages is the barrier.
    fn run_stages(&mut self, context: &SolverContext) {
        let parallel = self.parallel();
        let (states, writable) = (&self.states[..], self.writable_count);
        for stage in &self.stages {
            let (begin, end, grain) = (stage.begin as usize, stage.end as usize, stage.grain as usize);            match stage.kind {
                StageKind::IntegrateVelocities => {
                    let props = &self.props;
                    each_range(parallel, begin, end, grain, |first, last| integrate_velocities(states, props, context, first, last));
                }
                StageKind::IntegratePositions => {
                    each_chunk_mut(parallel, &mut self.props[begin..end], grain, |offset, props| integrate_positions(states, props, context, begin + offset));
                }
                kind => {
                    each_chunk_mut(parallel, &mut self.bundles[begin..end], grain, |_, bundles| solve_bundles(kind, bundles, states, writable, context));
                }
            }
        }
    }

    /// Records, for every contact in a bundle, the lane that solved it.
    fn index_contact_lanes(&mut self, contact_count: usize) {
        self.contact_lane.clear();
        self.contact_lane.resize(contact_count, NULL_INDEX);
        for (bundle_index, c) in self.bundles.iter().enumerate() {
            for (lane, &index) in c.contact.iter().enumerate() {
                if index != NULL_INDEX {
                    self.contact_lane[index as usize] = (bundle_index * LANES + lane) as u32;
                }
            }
        }
    }

    /// Writes impulses, velocities, poses and deltas back. Each contact and
    /// each body pulls its own result, so every write goes through a chunk of
    /// the destination slice.
    fn store_results(&mut self, inputs: &mut SolverInputs) {
        let parallel = self.parallel();
        self.index_contact_lanes(inputs.contacts.len());
        let (bundles, contact_lane) = (&self.bundles, &self.contact_lane);
        each_chunk_mut(parallel, inputs.contacts, 128, |offset, contacts| {
            for (contact, &lane) in contacts.iter_mut().zip(&contact_lane[offset..]) {
                if lane != NULL_INDEX {
                    bundles[lane as usize / LANES].store_lane(lane as usize % LANES, contact);
                }
            }
        });

        let (states, props, active, body_local) = (&self.states, &self.props, inputs.active_bodies, inputs.body_local);
        each_chunk_mut(parallel, inputs.bodies, 128, |offset, bodies| {
            for (slot, body) in (offset..).zip(bodies.iter_mut()) {
                let local = body_local[slot] as usize;
                if active.get(local) != Some(&(slot as u32)) {
                    continue;
                }
                let (state, props) = (&states[local], &props[local]);
                body.linear_velocity = load_vec3(&state.velocity);
                body.angular_velocity = load_vec3(&state.angular_velocity);
                body.set_pose(props.center, props.rotation);
            }
        });
        each_chunk_mut(parallel, inputs.deltas, 128, |offset, deltas| {
            for (state, delta) in states[offset..].iter().zip(deltas.iter_mut()) {
                *delta = BodyDelta { position: load_vec3(&state.delta_position), rotation: state.rotation_delta() };
            }
        });
    }
}
