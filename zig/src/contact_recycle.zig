//! The engine's contact_recycle: reuses a manifold while both bodies have
//! barely moved relative to each other since it was built.
const vm = @import("vecmath.zig");
const contact_types = @import("contact.zig");
const CollisionTolerances = @import("box_collision.zig").CollisionTolerances;

const Vec3 = vm.Vec3;
const Quat = vm.Quat;
const BoxPose = vm.BoxPose;
const Contact = contact_types.Contact;

pub const recycle_distance_scale: f32 = 10.0;
pub const recycle_angular_distance: f32 = 0.99240388;

pub const ContactPoses = struct {
    a: BoxPose = .{},
    b: BoxPose = .{},
};

/// A world point in a pose's local space.
fn toLocal(pose: *const BoxPose, world: Vec3) Vec3 {
    return pose.basis.transposed().apply(world.sub(pose.center));
}

/// A local point in world space.
fn toWorld(pose: *const BoxPose, local: Vec3) Vec3 {
    return pose.center.add(pose.basis.apply(local));
}

/// Cross product with every term added, a conservative arc bound.
fn modifiedCross(a: Vec3, b: Vec3) Vec3 {
    return .{ .x = a.y * b.z + a.z * b.y, .y = a.z * b.x + a.x * b.z, .z = a.x * b.y + a.y * b.x };
}

/// Squared dot product of two rotations, one when identical.
fn rotationCloseness(current: Quat, cached: Quat) f32 {
    const cosine = current.dot(cached);
    return cosine * cosine;
}

/// Stores the poses and local anchors a later recycle compares against.
pub fn cacheContact(contact: *Contact, poses: *const ContactPoses) void {
    const manifold = &contact.manifold;
    contact.cache.valid = manifold.point_count > 0;
    if (!contact.cache.valid) return;
    contact.cache.rotation_a = Quat.fromBasis(poses.a.basis);
    contact.cache.rotation_b = Quat.fromBasis(poses.b.basis);
    contact.cache.relative_pose = .{ .basis = poses.a.basis.transposed().mul(poses.b.basis), .origin = toLocal(&poses.a, poses.b.center) };
    manifold.local_normal = toLocal(&poses.a, poses.a.center.add(manifold.normal));
    for (manifold.points[0..manifold.point_count]) |*point| {
        point.local_a = toLocal(&poses.a, point.point);
        point.local_b = toLocal(&poses.b, point.point);
        point.cached_separation = point.separation;
    }
}

/// Moves the cached manifold with the bodies when they have barely moved;
/// false when it must be rebuilt.
pub fn tryRecycleContact(contact: *Contact, poses: *const ContactPoses, tolerances: CollisionTolerances) bool {
    if (!contact.cache.valid or contact.manifold.point_count == 0) return false;

    const rotation_a = Quat.fromBasis(poses.a.basis);
    const rotation_b = Quat.fromBasis(poses.b.basis);
    const angular = vm.minf(rotationCloseness(rotation_a, contact.cache.rotation_a), rotationCloseness(rotation_b, contact.cache.rotation_b));
    if (angular < recycle_angular_distance) return false;

    const tolerance = recycle_distance_scale * tolerances.linear_slop;
    const relative = toLocal(&poses.a, poses.b.center);
    const drift = relative.sub(contact.cache.relative_pose.origin);
    const distance_squared = drift.dot(drift);
    if (distance_squared >= tolerance * tolerance) return false;

    const slack = tolerance - @sqrt(distance_squared);
    const relative_rotation = contact.cache.relative_pose.basis.transposed().mul(poses.a.basis.transposed().mul(poses.b.basis));
    const turn = Quat.fromBasis(relative_rotation);
    const extent = poses.a.half_extents.max(poses.b.half_extents);
    const arc = modifiedCross(Vec3.init(turn.x, turn.y, turn.z).abs(), extent);
    if (4.0 * arc.dot(arc) >= slack * slack) return false;

    const manifold = &contact.manifold;
    const normal = poses.a.basis.apply(manifold.local_normal).normalize();
    manifold.normal = normal;
    manifold.separating_axis = normal;
    for (manifold.points[0..manifold.point_count]) |*point| {
        const world_a = toWorld(&poses.a, point.local_a);
        const world_b = toWorld(&poses.b, point.local_b);
        point.separation = point.cached_separation + world_b.sub(world_a).dot(normal);
        point.point = world_a.add(world_b).scale(0.5);
        point.persisted = true;
    }
    return true;
}
