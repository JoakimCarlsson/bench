#include "contact_recycle.h"

#include <math.h>

/// A world point in a pose's local space.
static Vec3 to_local(const BoxPose* pose, Vec3 world) {
    Basis inverse = basis_transposed(&pose->basis);
    return basis_apply(&inverse, v3_sub(world, pose->center));
}

/// A local point in world space.
static Vec3 to_world(const BoxPose* pose, Vec3 local) { return v3_add(pose->center, basis_apply(&pose->basis, local)); }

/// Cross product with every term added, a conservative arc bound.
static Vec3 modified_cross(Vec3 a, Vec3 b) {
    return (Vec3){ a.y * b.z + a.z * b.y, a.z * b.x + a.x * b.z, a.x * b.y + a.y * b.x };
}

/// Squared dot product of two rotations, one when identical.
static float rotation_closeness(Quat current, Quat cached) {
    float cosine = quat_dot(current, cached);
    return cosine * cosine;
}

/// Rotation of `b` relative to `a`.
static Basis relative_basis(const Basis* a, const Basis* b) {
    Basis inverse = basis_transposed(a);
    return basis_mul(&inverse, b);
}

void cache_contact(Contact* contact, const ContactPoses* poses) {
    Manifold* manifold = &contact->manifold;
    contact->cache.valid = manifold->point_count > 0;
    if (!contact->cache.valid) return;
    contact->cache.rotation_a = quat_from_basis(&poses->a.basis);
    contact->cache.rotation_b = quat_from_basis(&poses->b.basis);
    contact->cache.relative_pose =
        (Transform){ relative_basis(&poses->a.basis, &poses->b.basis), to_local(&poses->a, poses->b.center) };
    manifold->local_normal = to_local(&poses->a, v3_add(poses->a.center, manifold->normal));
    for (uint32_t i = 0; i < manifold->point_count; ++i) {
        ManifoldPoint* point = &manifold->points[i];
        point->local_a = to_local(&poses->a, point->point);
        point->local_b = to_local(&poses->b, point->point);
        point->cached_separation = point->separation;
    }
}

bool try_recycle_contact(Contact* contact, const ContactPoses* poses, const CollisionTolerances* tolerances) {
    if (!contact->cache.valid || contact->manifold.point_count == 0) return false;

    Quat rotation_a = quat_from_basis(&poses->a.basis);
    Quat rotation_b = quat_from_basis(&poses->b.basis);
    float angular = f32_min(rotation_closeness(rotation_a, contact->cache.rotation_a),
                            rotation_closeness(rotation_b, contact->cache.rotation_b));
    if (angular < RECYCLE_ANGULAR_DISTANCE) return false;

    float tolerance = RECYCLE_DISTANCE_SCALE * tolerances->linear_slop;
    Vec3 relative = to_local(&poses->a, poses->b.center);
    Vec3 drift = v3_sub(relative, contact->cache.relative_pose.origin);
    float distance_squared = v3_dot(drift, drift);
    if (distance_squared >= tolerance * tolerance) return false;

    float slack = tolerance - sqrtf(distance_squared);
    Basis cached_inverse = basis_transposed(&contact->cache.relative_pose.basis);
    Basis current = relative_basis(&poses->a.basis, &poses->b.basis);
    Basis relative_rotation = basis_mul(&cached_inverse, &current);
    Quat turn = quat_from_basis(&relative_rotation);
    Vec3 extent = v3_max(poses->a.half_extents, poses->b.half_extents);
    Vec3 arc = modified_cross(v3_abs((Vec3){ turn.x, turn.y, turn.z }), extent);
    if (4.0f * v3_dot(arc, arc) >= slack * slack) return false;

    Manifold* manifold = &contact->manifold;
    Vec3 normal = v3_normalize(basis_apply(&poses->a.basis, manifold->local_normal));
    manifold->normal = normal;
    manifold->separating_axis = normal;
    for (uint32_t i = 0; i < manifold->point_count; ++i) {
        ManifoldPoint* point = &manifold->points[i];
        Vec3 world_a = to_world(&poses->a, point->local_a);
        Vec3 world_b = to_world(&poses->b, point->local_b);
        point->separation = point->cached_separation + v3_dot(v3_sub(world_b, world_a), normal);
        point->point = v3_scale(v3_add(world_a, world_b), 0.5f);
        point->persisted = true;
    }
    return true;
}
