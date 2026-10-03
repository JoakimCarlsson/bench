#pragma once

#include "contact.hpp"
#include "box_collision.hpp"

/// The engine's contact_recycle: reuses a manifold while both bodies have
/// barely moved relative to each other since it was built.
namespace bench::phys {

inline constexpr float recycle_distance_scale = 10.0f;
inline constexpr float recycle_angular_distance = 0.99240388f;

struct ContactPoses {
    vm::BoxPose a{};
    vm::BoxPose b{};
};

/// Stores the poses and local anchors a later recycle compares against.
void cache_contact(Contact& contact, const ContactPoses& poses);

/// Moves the cached manifold with the bodies when they have barely moved;
/// false when it must be rebuilt.
bool try_recycle_contact(Contact& contact, const ContactPoses& poses, const CollisionTolerances& tolerances);

} // namespace bench::phys
