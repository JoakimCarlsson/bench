//! The parts of the engine's RigidBody the world step reads and writes, for
//! a body of one box shape.
const vm = @import("vecmath.zig");

const Vec3 = vm.Vec3;
const Basis = vm.Basis;

pub const RigidBody = struct {
    transform: vm.Transform = .{},
    rotation: vm.Quat = .{},
    center_of_mass: Vec3 = .{},
    linear_velocity: Vec3 = .{},
    angular_velocity: Vec3 = .{},
    mass: f32 = 1.0,
    inverse_mass: f32 = 1.0,
    inverse_inertia_local: Basis = .{},
    inverse_inertia_world: Basis = .{},
    applied_force: Vec3 = .{},
    applied_torque: Vec3 = .{},
    constant_force: Vec3 = .{},
    constant_torque: Vec3 = .{},
    gravity_scale: f32 = 1.0,
    linear_damp: f32 = 0.0,
    angular_damp: f32 = 0.0,
    half_extents: Vec3 = Vec3.init(0.5, 0.5, 0.5),
    max_extent: Vec3 = Vec3.init(0.5, 0.5, 0.5),
    friction: f32 = 0.6,
    restitution: f32 = 0.0,
    rolling_resistance: f32 = 0.0,
    sleeping: bool = false,
    can_sleep: bool = true,
    sleep_time: f32 = 0.0,
    sleep_velocity: f32 = 0.0,
    island: u32 = 0,

    /// Centre of mass in world space.
    pub fn worldCenterOfMass(self: *const RigidBody) Vec3 {
        return self.transform.transformPoint(self.center_of_mass);
    }

    /// Recomputes the world inverse inertia from the rotation.
    pub fn refreshWorldInertia(self: *RigidBody) void {
        const rotation = self.transform.basis;
        self.inverse_inertia_world = rotation.mul(self.inverse_inertia_local).mul(rotation.transposed());
    }

    /// Moves the body so its centre of mass is at `center` with `rotation`.
    pub fn setPose(self: *RigidBody, center: Vec3, rotation: vm.Quat) void {
        self.rotation = rotation;
        self.transform.basis = Basis.fromQuat(rotation);
        self.transform.origin = center.sub(self.transform.basis.apply(self.center_of_mass));
        self.refreshWorldInertia();
    }

    /// Gives a box body its mass, inertia and extents from density 1.
    pub fn setBoxMass(self: *RigidBody, h: Vec3) void {
        self.half_extents = h;
        self.max_extent = h;
        self.mass = 8.0 * h.x * h.y * h.z;
        self.inverse_mass = 1.0 / self.mass;
        const third = self.mass / 3.0;
        const inertia: Basis = .{
            .x = Vec3.init(third * (h.y * h.y + h.z * h.z), 0.0, 0.0),
            .y = Vec3.init(0.0, third * (h.x * h.x + h.z * h.z), 0.0),
            .z = Vec3.init(0.0, 0.0, third * (h.x * h.x + h.y * h.y)),
        };
        self.inverse_inertia_local = inertia.inverse();
        self.refreshWorldInertia();
    }
};
