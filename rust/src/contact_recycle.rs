//! The engine's contact_recycle: reuses a manifold while both bodies have
//! barely moved relative to each other since it was built.
use crate::box_collision::CollisionTolerances;
use crate::contact::Contact;
use crate::vecmath::{self, BoxPose, Quat, Transform, Vec3};

const RECYCLE_DISTANCE_SCALE: f32 = 10.0;
const RECYCLE_ANGULAR_DISTANCE: f32 = 0.992_403_88;

pub struct ContactPoses {
    pub a: BoxPose,
    pub b: BoxPose,
}

/// A world point in a pose's local space.
fn to_local(pose: &BoxPose, world: Vec3) -> Vec3 {
    pose.basis.transposed() * (world - pose.center)
}

/// A local point in world space.
fn to_world(pose: &BoxPose, local: Vec3) -> Vec3 {
    pose.center + pose.basis * local
}

/// Cross product with every term added, a conservative arc bound.
fn modified_cross(a: Vec3, b: Vec3) -> Vec3 {
    Vec3::new(a.y * b.z + a.z * b.y, a.z * b.x + a.x * b.z, a.x * b.y + a.y * b.x)
}

/// Squared dot product of two rotations, one when identical.
fn rotation_closeness(current: Quat, cached: Quat) -> f32 {
    let cosine = current.dot(cached);
    cosine * cosine
}

/// Stores the poses and local anchors a later recycle compares against.
pub fn cache_contact(contact: &mut Contact, poses: &ContactPoses) {
    contact.cache.valid = contact.manifold.point_count > 0;
    if !contact.cache.valid {
        return;
    }
    contact.cache.rotation_a = Quat::from_basis(&poses.a.basis);
    contact.cache.rotation_b = Quat::from_basis(&poses.b.basis);
    contact.cache.relative_pose = Transform { basis: poses.a.basis.transposed() * poses.b.basis, origin: to_local(&poses.a, poses.b.center) };
    let manifold = &mut contact.manifold;
    manifold.local_normal = to_local(&poses.a, poses.a.center + manifold.normal);
    for point in manifold.active_mut() {
        point.local_a = to_local(&poses.a, point.point);
        point.local_b = to_local(&poses.b, point.point);
        point.cached_separation = point.separation;
    }
}

/// Moves the cached manifold with the bodies when they have barely moved;
/// false when it must be rebuilt.
pub fn try_recycle_contact(contact: &mut Contact, poses: &ContactPoses, tolerances: &CollisionTolerances) -> bool {
    if !contact.cache.valid || contact.manifold.point_count == 0 {
        return false;
    }

    let rotation_a = Quat::from_basis(&poses.a.basis);
    let rotation_b = Quat::from_basis(&poses.b.basis);
    let angular = vecmath::min(rotation_closeness(rotation_a, contact.cache.rotation_a), rotation_closeness(rotation_b, contact.cache.rotation_b));
    if angular < RECYCLE_ANGULAR_DISTANCE {
        return false;
    }

    let tolerance = RECYCLE_DISTANCE_SCALE * tolerances.linear_slop;
    let relative = to_local(&poses.a, poses.b.center);
    let drift = relative - contact.cache.relative_pose.origin;
    let distance_squared = drift.dot(drift);
    if distance_squared >= tolerance * tolerance {
        return false;
    }

    let slack = tolerance - distance_squared.sqrt();
    let relative_rotation = contact.cache.relative_pose.basis.transposed() * (poses.a.basis.transposed() * poses.b.basis);
    let turn = Quat::from_basis(&relative_rotation);
    let extent = poses.a.half_extents.max(poses.b.half_extents);
    let arc = modified_cross(Vec3::new(turn.x, turn.y, turn.z).abs(), extent);
    if 4.0 * arc.dot(arc) >= slack * slack {
        return false;
    }

    let manifold = &mut contact.manifold;
    let normal = (poses.a.basis * manifold.local_normal).normalize();
    manifold.normal = normal;
    manifold.separating_axis = normal;
    for point in manifold.active_mut() {
        let world_a = to_world(&poses.a, point.local_a);
        let world_b = to_world(&poses.b, point.local_b);
        point.separation = point.cached_separation + (world_b - world_a).dot(normal);
        point.point = (world_a + world_b) * 0.5;
        point.persisted = true;
    }
    true
}
