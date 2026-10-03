#pragma once

#include "contact.hpp"
#include "vecmath.hpp"

/// The engine's box_collision: separating-axis tests over 15 axes,
/// incident-face clipping, reduction to four points and warm starting.
namespace bench::phys {

struct CollisionTolerances {
    float speculative_distance{0.02f};
    float linear_slop{0.005f};
};

/// World bounds of an oriented box.
vm::Aabb box_aabb(const vm::BoxPose& box);

/// Rebuilds the manifold between two boxes, reusing last step's separating
/// axis as an early out and its points for warm starting.
void collide_boxes(const vm::BoxPose& a, const vm::BoxPose& b, const CollisionTolerances& tolerances, Manifold& manifold);

} // namespace bench::phys
