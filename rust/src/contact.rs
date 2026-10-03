//! The engine's contact.hpp and the pieces of broad_phase.hpp it needs: a
//! contact between two box shapes, its manifold, friction state, cached poses
//! and the links that place it in sets, the graph and islands.
use crate::vecmath::{Quat, Transform, Vec3};

pub const MAX_MANIFOLD_POINTS: usize = 4;
pub const NULL_LINK: u32 = 0xFFFF_FFFF;
pub const AWAKE_SET: u32 = 0;
pub const DISABLED_SET: u32 = 0xFFFF_FFFE;
pub const MIN_FRICTION_WEIGHT: f32 = 1e-10;

/// One shape: a rigid body slot, or a static body index.
#[derive(Clone, Copy, Default, PartialEq, Eq)]
pub struct ShapeRef {
    pub body: u32,
    pub is_static: bool,
}

impl ShapeRef {
    /// The shape packed into 32 bits: static flag, body index and shape index 0.
    pub fn pack(self) -> u32 {
        (if self.is_static { 0x8000_0000 } else { 0 }) | ((self.body & 0x7FFFF) << 12)
    }
}

/// Order-independent key of a shape pair.
pub fn pair_key(a: ShapeRef, b: ShapeRef) -> u64 {
    let pa = a.pack();
    let pb = b.pack();
    let lo = if pa < pb { pa } else { pb };
    let hi = if pa < pb { pb } else { pa };
    ((lo as u64) << 32) | hi as u64
}

#[derive(Clone, Copy, Default)]
pub struct ManifoldPoint {
    pub point: Vec3,
    pub separation: f32,
    pub normal_impulse: f32,
    pub total_normal_impulse: f32,
    pub peak_normal_impulse: f32,
    pub relative_velocity: f32,
    pub local_a: Vec3,
    pub local_b: Vec3,
    pub cached_separation: f32,
    pub feature_id: u32,
    pub persisted: bool,
}

#[derive(Clone, Copy, Default)]
pub struct Manifold {
    pub normal: Vec3,
    pub separating_axis: Vec3,
    pub local_normal: Vec3,
    pub points: [ManifoldPoint; MAX_MANIFOLD_POINTS],
    pub point_count: u32,
}

impl Manifold {
    /// The points in use.
    pub fn active(&self) -> &[ManifoldPoint] {
        &self.points[..self.point_count as usize]
    }

    /// The points in use, writable.
    pub fn active_mut(&mut self) -> &mut [ManifoldPoint] {
        &mut self.points[..self.point_count as usize]
    }
}

/// Accumulated tangent, twist and rolling friction impulses carried between
/// steps.
#[derive(Clone, Copy, Default)]
pub struct FrictionImpulses {
    pub tangent_x: f32,
    pub tangent_y: f32,
    pub twist: f32,
    pub rolling: Vec3,
}

/// Soft constraint coefficients.
#[derive(Clone, Copy)]
pub struct Softness {
    pub bias_rate: f32,
    pub mass_scale: f32,
    pub impulse_scale: f32,
}

impl Default for Softness {
    /// Rigid: no bias, full mass.
    fn default() -> Softness {
        Softness { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 }
    }
}

/// Soft constraint coefficients of a spring at `hertz` with `damping_ratio`
/// over substep `h`.
pub fn make_softness(hertz: f32, damping_ratio: f32, h: f32) -> Softness {
    if hertz <= 0.0 {
        return Softness { bias_rate: 0.0, mass_scale: 1.0, impulse_scale: 0.0 };
    }
    let omega = 2.0 * std::f32::consts::PI * hertz;
    let a1 = 2.0 * damping_ratio + h * omega;
    let a2 = h * omega * a1;
    let a3 = 1.0 / (1.0 + a2);
    Softness { bias_rate: omega / a1, mass_scale: a2 * a3, impulse_scale: a3 }
}

/// Poses cached to recycle a contact while its bodies barely move.
#[derive(Clone, Copy, Default)]
pub struct ContactCache {
    pub rotation_a: Quat,
    pub rotation_b: Quat,
    pub relative_pose: Transform,
    pub valid: bool,
}

#[derive(Clone, Copy)]
pub struct Contact {
    pub shape_a: ShapeRef,
    pub shape_b: ShapeRef,
    pub proxy_a: i32,
    pub proxy_b: i32,
    pub manifold: Manifold,
    pub friction_impulses: FrictionImpulses,
    pub cache: ContactCache,
    pub friction: f32,
    pub restitution: f32,
    pub rolling_resistance: f32,
    pub touching: bool,
    pub was_touching: bool,
    pub linked: bool,
    pub alive: bool,
    pub set: u32,
    pub color: u32,
    pub local: u32,
    pub island: u32,
    pub island_local: u32,
    pub edge_local: [u32; 2],
}

impl Default for Contact {
    /// Unlinked, with no proxies.
    fn default() -> Contact {
        Contact {
            shape_a: ShapeRef::default(),
            shape_b: ShapeRef::default(),
            proxy_a: -1,
            proxy_b: -1,
            manifold: Manifold::default(),
            friction_impulses: FrictionImpulses::default(),
            cache: ContactCache::default(),
            friction: 0.0,
            restitution: 0.0,
            rolling_resistance: 0.0,
            touching: false,
            was_touching: false,
            linked: false,
            alive: false,
            set: NULL_LINK,
            color: NULL_LINK,
            local: NULL_LINK,
            island: NULL_LINK,
            island_local: NULL_LINK,
            edge_local: [NULL_LINK, NULL_LINK],
        }
    }
}
