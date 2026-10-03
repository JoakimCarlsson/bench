#ifndef BENCH_CONTACT_RECYCLE_H
#define BENCH_CONTACT_RECYCLE_H

#include <stdbool.h>

#include "box_collision.h"
#include "contact.h"

/// The engine's contact_recycle: reuses a manifold while both bodies have
/// barely moved relative to each other since it was built.

#define RECYCLE_DISTANCE_SCALE 10.0f
#define RECYCLE_ANGULAR_DISTANCE 0.99240388f

typedef struct {
    BoxPose a;
    BoxPose b;
} ContactPoses;

/// Stores the poses and local anchors a later recycle compares against.
void cache_contact(Contact* contact, const ContactPoses* poses);

/// Moves the cached manifold with the bodies when they have barely moved;
/// false when it must be rebuilt.
bool try_recycle_contact(Contact* contact, const ContactPoses* poses, const CollisionTolerances* tolerances);

#endif
