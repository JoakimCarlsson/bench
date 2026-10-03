#ifndef BENCH_RIGID_BODY_H
#define BENCH_RIGID_BODY_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "vecmath.h"

/// The parts of the engine's RigidBody the world step reads and writes, for
/// a body of one box shape.
typedef struct {
    Transform transform;
    Quat rotation;
    Vec3 center_of_mass;
    Vec3 linear_velocity;
    Vec3 angular_velocity;
    float mass;
    float inverse_mass;
    Basis inverse_inertia_local;
    Basis inverse_inertia_world;
    Vec3 applied_force;
    Vec3 applied_torque;
    Vec3 constant_force;
    Vec3 constant_torque;
    float gravity_scale;
    float linear_damp;
    float angular_damp;
    Vec3 half_extents;
    Vec3 max_extent;
    float friction;
    float restitution;
    float rolling_resistance;
    bool sleeping;
    bool can_sleep;
    float sleep_time;
    float sleep_velocity;
    uint32_t island;
} RigidBody;

/// A unit cube of mass one at the origin, with the engine's defaults.
static inline RigidBody rigid_body_new(void) {
    RigidBody body;
    memset(&body, 0, sizeof body);
    body.transform = transform_identity();
    body.rotation = quat_identity();
    body.mass = 1.0f;
    body.inverse_mass = 1.0f;
    body.inverse_inertia_local = basis_identity();
    body.inverse_inertia_world = basis_identity();
    body.gravity_scale = 1.0f;
    body.half_extents = (Vec3){ 0.5f, 0.5f, 0.5f };
    body.max_extent = (Vec3){ 0.5f, 0.5f, 0.5f };
    body.friction = 0.6f;
    body.can_sleep = true;
    return body;
}

/// Centre of mass in world space.
static inline Vec3 world_center_of_mass(const RigidBody* body) {
    return transform_point(&body->transform, body->center_of_mass);
}

/// Recomputes the world inverse inertia from the rotation.
static inline void refresh_world_inertia(RigidBody* body) {
    const Basis* rotation = &body->transform.basis;
    Basis rotated = basis_mul(rotation, &body->inverse_inertia_local);
    Basis transposed = basis_transposed(rotation);
    body->inverse_inertia_world = basis_mul(&rotated, &transposed);
}

/// Moves a body so its centre of mass is at `center` with `rotation`.
static inline void set_pose(RigidBody* body, Vec3 center, Quat rotation) {
    body->rotation = rotation;
    body->transform.basis = basis_from_quat(rotation);
    body->transform.origin = v3_sub(center, basis_apply(&body->transform.basis, body->center_of_mass));
    refresh_world_inertia(body);
}

/// Gives a box body its mass, inertia and extents from density 1.
static inline void set_box_mass(RigidBody* body, Vec3 h) {
    body->half_extents = h;
    body->max_extent = h;
    body->mass = 8.0f * h.x * h.y * h.z;
    body->inverse_mass = 1.0f / body->mass;
    float third = body->mass / 3.0f;
    Basis inertia = { { third * (h.y * h.y + h.z * h.z), 0.0f, 0.0f },
                      { 0.0f, third * (h.x * h.x + h.z * h.z), 0.0f },
                      { 0.0f, 0.0f, third * (h.x * h.x + h.y * h.y) } };
    body->inverse_inertia_local = basis_inverse(&inertia);
    refresh_world_inertia(body);
}

#endif
