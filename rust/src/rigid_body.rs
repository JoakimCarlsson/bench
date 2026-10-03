//! The parts of the engine's RigidBody the world step reads and writes, for
//! a body of one box shape.
use crate::vecmath::{Basis, Quat, Transform, Vec3};

#[derive(Clone, Copy)]
pub struct RigidBody {
    pub transform: Transform,
    pub rotation: Quat,
    pub center_of_mass: Vec3,
    pub linear_velocity: Vec3,
    pub angular_velocity: Vec3,
    pub mass: f32,
    pub inverse_mass: f32,
    pub inverse_inertia_local: Basis,
    pub inverse_inertia_world: Basis,
    pub applied_force: Vec3,
    pub applied_torque: Vec3,
    pub constant_force: Vec3,
    pub constant_torque: Vec3,
    pub gravity_scale: f32,
    pub linear_damp: f32,
    pub angular_damp: f32,
    pub half_extents: Vec3,
    pub max_extent: Vec3,
    pub friction: f32,
    pub restitution: f32,
    pub rolling_resistance: f32,
    pub sleeping: bool,
    pub can_sleep: bool,
    pub sleep_time: f32,
    pub sleep_velocity: f32,
    pub island: u32,
}

impl Default for RigidBody {
    /// A unit box of mass one at the origin, at rest.
    fn default() -> RigidBody {
        RigidBody {
            transform: Transform::default(),
            rotation: Quat::default(),
            center_of_mass: Vec3::default(),
            linear_velocity: Vec3::default(),
            angular_velocity: Vec3::default(),
            mass: 1.0,
            inverse_mass: 1.0,
            inverse_inertia_local: Basis::default(),
            inverse_inertia_world: Basis::default(),
            applied_force: Vec3::default(),
            applied_torque: Vec3::default(),
            constant_force: Vec3::default(),
            constant_torque: Vec3::default(),
            gravity_scale: 1.0,
            linear_damp: 0.0,
            angular_damp: 0.0,
            half_extents: Vec3::new(0.5, 0.5, 0.5),
            max_extent: Vec3::new(0.5, 0.5, 0.5),
            friction: 0.6,
            restitution: 0.0,
            rolling_resistance: 0.0,
            sleeping: false,
            can_sleep: true,
            sleep_time: 0.0,
            sleep_velocity: 0.0,
            island: 0,
        }
    }
}

impl RigidBody {
    /// Centre of mass in world space.
    pub fn world_center_of_mass(&self) -> Vec3 {
        self.transform.transform_point(self.center_of_mass)
    }

    /// Recomputes the world inverse inertia from the rotation.
    pub fn refresh_world_inertia(&mut self) {
        let rotation = self.transform.basis;
        self.inverse_inertia_world = rotation * self.inverse_inertia_local * rotation.transposed();
    }

    /// Moves the body so its centre of mass is at `center` with `rotation`.
    pub fn set_pose(&mut self, center: Vec3, rotation: Quat) {
        self.rotation = rotation;
        self.transform.basis = Basis::from_quat(rotation);
        self.transform.origin = center - self.transform.basis * self.center_of_mass;
        self.refresh_world_inertia();
    }

    /// Gives a box body its mass, inertia and extents from density 1.
    pub fn set_box_mass(&mut self, h: Vec3) {
        self.half_extents = h;
        self.max_extent = h;
        self.mass = 8.0 * h.x * h.y * h.z;
        self.inverse_mass = 1.0 / self.mass;
        let third = self.mass / 3.0;
        let inertia = Basis {
            x: Vec3::new(third * (h.y * h.y + h.z * h.z), 0.0, 0.0),
            y: Vec3::new(0.0, third * (h.x * h.x + h.z * h.z), 0.0),
            z: Vec3::new(0.0, 0.0, third * (h.x * h.x + h.y * h.y)),
        };
        self.inverse_inertia_local = inertia.inverse();
        self.refresh_world_inertia();
    }
}
