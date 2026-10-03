#ifndef BENCH_CONTACT_H
#define BENCH_CONTACT_H

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "vecmath.h"

/// The engine's contact.hpp and the pieces of broad_phase.hpp it needs: a
/// contact between two box shapes, its manifold, friction state, cached poses
/// and the links that place it in sets, the graph and islands.

enum { MAX_MANIFOLD_POINTS = 4 };

#define NULL_LINK 0xFFFFFFFFu
#define AWAKE_SET 0u
#define DISABLED_SET 0xFFFFFFFEu
#define MIN_FRICTION_WEIGHT 1e-10f

/// One shape: a rigid body slot, or a static body index.
typedef struct {
    uint32_t body;
    bool is_static;
} ShapeRef;

/// The shape packed into 32 bits: static flag, body index and shape index 0.
static inline uint32_t pack_shape_ref(ShapeRef ref) {
    return (ref.is_static ? 0x80000000u : 0u) | ((ref.body & 0x7FFFFu) << 12);
}

/// Order-independent key of a shape pair.
static inline uint64_t pair_key(ShapeRef a, ShapeRef b) {
    uint32_t pa = pack_shape_ref(a);
    uint32_t pb = pack_shape_ref(b);
    uint32_t lo = pa < pb ? pa : pb;
    uint32_t hi = pa < pb ? pb : pa;
    return ((uint64_t)lo << 32) | hi;
}

typedef struct {
    Vec3 point;
    float separation;
    float normal_impulse;
    float total_normal_impulse;
    float peak_normal_impulse;
    float relative_velocity;
    Vec3 local_a;
    Vec3 local_b;
    float cached_separation;
    uint32_t feature_id;
    bool persisted;
} ManifoldPoint;

typedef struct {
    Vec3 normal;
    Vec3 separating_axis;
    Vec3 local_normal;
    ManifoldPoint points[MAX_MANIFOLD_POINTS];
    uint32_t point_count;
} Manifold;

/// Accumulated tangent, twist and rolling friction impulses carried between
/// steps.
typedef struct {
    float tangent_x;
    float tangent_y;
    float twist;
    Vec3 rolling;
} FrictionImpulses;

/// Soft constraint coefficients.
typedef struct {
    float bias_rate;
    float mass_scale;
    float impulse_scale;
} Softness;

/// Soft constraint coefficients of a spring at `hertz` with `damping_ratio`
/// over substep `h`.
Softness make_softness(float hertz, float damping_ratio, float h);

/// Poses cached to recycle a contact while its bodies barely move.
typedef struct {
    Quat rotation_a;
    Quat rotation_b;
    Transform relative_pose;
    bool valid;
} ContactCache;

typedef struct {
    ShapeRef shape_a;
    ShapeRef shape_b;
    int32_t proxy_a;
    int32_t proxy_b;
    Manifold manifold;
    FrictionImpulses friction_impulses;
    ContactCache cache;
    float friction;
    float restitution;
    float rolling_resistance;
    bool touching;
    bool was_touching;
    bool linked;
    bool alive;
    uint32_t set;
    uint32_t color;
    uint32_t local;
    uint32_t island;
    uint32_t island_local;
    uint32_t edge_local[2];
} Contact;

/// A contact with the engine's defaults: no proxies, no links, identity
/// cached poses.
static inline Contact contact_new(void) {
    Contact c;
    memset(&c, 0, sizeof c);
    c.proxy_a = -1;
    c.proxy_b = -1;
    c.cache.rotation_a = quat_identity();
    c.cache.rotation_b = quat_identity();
    c.cache.relative_pose = transform_identity();
    c.set = NULL_LINK;
    c.color = NULL_LINK;
    c.local = NULL_LINK;
    c.island = NULL_LINK;
    c.island_local = NULL_LINK;
    c.edge_local[0] = NULL_LINK;
    c.edge_local[1] = NULL_LINK;
    return c;
}

#endif
