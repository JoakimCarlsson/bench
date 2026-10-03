#pragma once

#include <cstdint>

#include "vecmath.hpp"

/// The parts of the engine's RigidBody the world step reads and writes, for
/// a body of one box shape.
namespace bench::phys {

struct RigidBody {
    vm::Transform transform{};
    vm::Quat rotation{};
    vm::Vec3 center_of_mass{};
    vm::Vec3 linear_velocity{};
    vm::Vec3 angular_velocity{};
    float mass{1.0f};
    float inverse_mass{1.0f};
    vm::Basis inverse_inertia_local{};
    vm::Basis inverse_inertia_world{};
    vm::Vec3 applied_force{};
    vm::Vec3 applied_torque{};
    vm::Vec3 constant_force{};
    vm::Vec3 constant_torque{};
    float gravity_scale{1.0f};
    float linear_damp{};
    float angular_damp{};
    vm::Vec3 half_extents{0.5f, 0.5f, 0.5f};
    vm::Vec3 max_extent{0.5f, 0.5f, 0.5f};
    float friction{0.6f};
    float restitution{};
    float rolling_resistance{};
    bool sleeping{};
    bool can_sleep{true};
    float sleep_time{};
    float sleep_velocity{};
    uint32_t island{};

    /// Centre of mass in world space.
    vm::Vec3 world_center_of_mass() const { return vm::transform_point(transform, center_of_mass); }
};

/// Recomputes the world inverse inertia from the rotation.
inline void refresh_world_inertia(RigidBody& body) {
    const vm::Basis& rotation = body.transform.basis;
    body.inverse_inertia_world = rotation * body.inverse_inertia_local * vm::transposed(rotation);
}

/// Moves a body so its centre of mass is at `center` with `rotation`.
inline void set_pose(RigidBody& body, vm::Vec3 center, vm::Quat rotation) {
    body.rotation = rotation;
    body.transform.basis = vm::basis_from_quat(rotation);
    body.transform.origin = center - body.transform.basis * body.center_of_mass;
    refresh_world_inertia(body);
}

/// Gives a box body its mass, inertia and extents from density 1.
inline void set_box_mass(RigidBody& body, vm::Vec3 h) {
    body.half_extents = h;
    body.max_extent = h;
    body.mass = 8.0f * h.x * h.y * h.z;
    body.inverse_mass = 1.0f / body.mass;
    const float third = body.mass / 3.0f;
    const vm::Basis inertia{{third * (h.y * h.y + h.z * h.z), 0.0f, 0.0f},
                            {0.0f, third * (h.x * h.x + h.z * h.z), 0.0f},
                            {0.0f, 0.0f, third * (h.x * h.x + h.y * h.y)}};
    body.inverse_inertia_local = vm::inverse(inertia);
    refresh_world_inertia(body);
}

} // namespace bench::phys
