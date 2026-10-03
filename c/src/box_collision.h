#ifndef BENCH_BOX_COLLISION_H
#define BENCH_BOX_COLLISION_H

#include "contact.h"
#include "vecmath.h"

/// The engine's box_collision: separating-axis tests over 15 axes,
/// incident-face clipping, reduction to four points and warm starting.

typedef struct {
    float speculative_distance;
    float linear_slop;
} CollisionTolerances;

/// The engine's default tolerances.
static inline CollisionTolerances collision_tolerances_default(void) { return (CollisionTolerances){ 0.02f, 0.005f }; }

/// World bounds of an oriented box.
Aabb box_aabb(const BoxPose* box);

/// Rebuilds the manifold between two boxes, reusing last step's separating
/// axis as an early out and its points for warm starting.
void collide_boxes(const BoxPose* a, const BoxPose* b, const CollisionTolerances* tolerances, Manifold* manifold);

#endif
