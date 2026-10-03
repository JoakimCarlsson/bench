#include "contact_recycle.hpp"

#include <algorithm>
#include <cmath>

namespace bench::phys {

namespace {

using vm::BoxPose;
using vm::Quat;
using vm::Vec3;

/// A world point in a pose's local space.
Vec3 to_local(const BoxPose& pose, Vec3 world) { return vm::transposed(pose.basis) * (world - pose.center); }

/// A local point in world space.
Vec3 to_world(const BoxPose& pose, Vec3 local) { return pose.center + pose.basis * local; }

/// Cross product with every term added, a conservative arc bound.
Vec3 modified_cross(Vec3 a, Vec3 b) { return {a.y * b.z + a.z * b.y, a.z * b.x + a.x * b.z, a.x * b.y + a.y * b.x}; }

/// Squared dot product of two rotations, one when identical.
float rotation_closeness(Quat current, Quat cached) {
    const float cosine = vm::dot(current, cached);
    return cosine * cosine;
}

} // namespace

void cache_contact(Contact& contact, const ContactPoses& poses) {
    Manifold& manifold = contact.manifold;
    contact.cache.valid = manifold.point_count > 0;
    if (!contact.cache.valid) return;
    contact.cache.rotation_a = vm::quat_from_basis(poses.a.basis);
    contact.cache.rotation_b = vm::quat_from_basis(poses.b.basis);
    contact.cache.relative_pose = vm::Transform{vm::transposed(poses.a.basis) * poses.b.basis, to_local(poses.a, poses.b.center)};
    manifold.local_normal = to_local(poses.a, poses.a.center + manifold.normal);
    for (uint32_t i = 0; i < manifold.point_count; ++i) {
        ManifoldPoint& point = manifold.points[i];
        point.local_a = to_local(poses.a, point.point);
        point.local_b = to_local(poses.b, point.point);
        point.cached_separation = point.separation;
    }
}

bool try_recycle_contact(Contact& contact, const ContactPoses& poses, const CollisionTolerances& tolerances) {
    if (!contact.cache.valid || contact.manifold.point_count == 0) return false;

    const Quat rotation_a = vm::quat_from_basis(poses.a.basis);
    const Quat rotation_b = vm::quat_from_basis(poses.b.basis);
    const float angular = std::min(rotation_closeness(rotation_a, contact.cache.rotation_a),
                                   rotation_closeness(rotation_b, contact.cache.rotation_b));
    if (angular < recycle_angular_distance) return false;

    const float tolerance = recycle_distance_scale * tolerances.linear_slop;
    const Vec3 relative = to_local(poses.a, poses.b.center);
    const Vec3 drift = relative - contact.cache.relative_pose.origin;
    const float distance_squared = vm::dot(drift, drift);
    if (distance_squared >= tolerance * tolerance) return false;

    const float slack = tolerance - std::sqrt(distance_squared);
    const vm::Basis relative_rotation =
        vm::transposed(contact.cache.relative_pose.basis) * (vm::transposed(poses.a.basis) * poses.b.basis);
    const Quat turn = vm::quat_from_basis(relative_rotation);
    const Vec3 extent = vm::max(poses.a.half_extents, poses.b.half_extents);
    const Vec3 arc = modified_cross(vm::abs(Vec3{turn.x, turn.y, turn.z}), extent);
    if (4.0f * vm::dot(arc, arc) >= slack * slack) return false;

    Manifold& manifold = contact.manifold;
    const Vec3 normal = vm::normalize(poses.a.basis * manifold.local_normal);
    manifold.normal = normal;
    manifold.separating_axis = normal;
    for (uint32_t i = 0; i < manifold.point_count; ++i) {
        ManifoldPoint& point = manifold.points[i];
        const Vec3 world_a = to_world(poses.a, point.local_a);
        const Vec3 world_b = to_world(poses.b, point.local_b);
        point.separation = point.cached_separation + vm::dot(world_b - world_a, normal);
        point.point = (world_a + world_b) * 0.5f;
        point.persisted = true;
    }
    return true;
}

} // namespace bench::phys
