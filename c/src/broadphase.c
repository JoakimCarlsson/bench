#include "broadphase.h"

#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "array.h"
#include "box_collision.h"
#include "broad_phase.h"
#include "hash.h"
#include "u64map.h"
#include "vecmath.h"

enum { BODIES = 4096, TILES_SIDE = 32, TILES = TILES_SIDE * TILES_SIDE, FRAMES = 16 };

static const float speculative = 0.02f;
static const float max_aabb_margin = 0.05f;
static const float aabb_margin_fraction = 0.125f;
static const float dt = 1.0f / 60.0f;
static const float extent_xz = 45.0f;
static const float extent_y = 16.0f;

typedef struct {
    BoxPose pose;
    Quat rotation;
    Vec3 velocity;
    Vec3 spin;
} Body;

typedef struct {
    int32_t proxy_a;
    int32_t proxy_b;
    uint64_t key;
    bool alive;
} Pair;

typedef struct {
    Body* initial;
    Body* bodies;
    BoxPose* tiles;
    int32_t* body_proxies;
    BroadPhase broad_phase;
    ARRAY_OF(Pair) pairs;
    U32Array free_pairs;
    U64Map pair_index;
} BroadPhaseCase;

/// Fat margin for a box, the engine's shape_margin.
static float shape_margin(Vec3 h) { return f32_min(max_aabb_margin, aabb_margin_fraction * 2.0f * f32_max3(h.x, h.y, h.z)); }

/// Bounds the broad phase tests against: the box grown by the speculative
/// distance.
static Aabb tight_aabb(const BoxPose* pose) {
    Aabb box = box_aabb(pose);
    return aabb_grow(&box, speculative);
}

/// `value` wrapped into [0, extent).
static float wrap(float value, float extent) {
    if (value < 0.0f) return value + extent;
    if (value >= extent) return value - extent;
    return value;
}

/// Builds the tumbling boxes and the tile field.
static void setup(void* state) {
    BroadPhaseCase* s = state;
    s->initial = xalloc(BODIES * sizeof(Body));
    s->bodies = xalloc(BODIES * sizeof(Body));
    s->tiles = xalloc(TILES * sizeof(BoxPose));
    s->body_proxies = xalloc(BODIES * sizeof(int32_t));
    broad_phase_init(&s->broad_phase);
    Rng rng = { 0xb40ad };
    for (uint32_t i = 0; i < BODIES; ++i) {
        Body* b = &s->initial[i];
        b->pose.half_extents = v3_random(&rng, 0.25f, 0.75f);
        b->pose.center.x = rng_unit(&rng) * extent_xz;
        b->pose.center.y = rng_unit(&rng) * extent_y;
        b->pose.center.z = rng_unit(&rng) * extent_xz;
        b->rotation = quat_random(&rng);
        b->pose.basis = basis_from_quat(b->rotation);
        b->velocity = v3_random(&rng, -1.0f, 1.0f);
        b->spin = v3_random(&rng, -0.5f, 0.5f);
    }
    for (uint32_t z = 0; z < TILES_SIDE; ++z) {
        for (uint32_t x = 0; x < TILES_SIDE; ++x) {
            BoxPose* tile = &s->tiles[z * TILES_SIDE + x];
            tile->half_extents = (Vec3){ 1.0f, 0.5f, 1.0f };
            tile->center = (Vec3){ (float)x * 2.0f + 1.0f, -0.5f, (float)z * 2.0f + 1.0f };
            tile->basis = basis_identity();
        }
    }
}

/// Creates every proxy from the initial poses.
static void reset(BroadPhaseCase* s) {
    broad_phase_free(&s->broad_phase);
    broad_phase_init(&s->broad_phase);
    s->pairs.len = 0;
    s->free_pairs.len = 0;
    u64map_clear(&s->pair_index);
    memcpy(s->bodies, s->initial, BODIES * sizeof(Body));
    for (uint32_t i = 0; i < TILES; ++i) {
        Aabb tight = tight_aabb(&s->tiles[i]);
        broad_phase_create_proxy(&s->broad_phase, (ShapeRef){ i, true }, &tight, false);
    }
    for (uint32_t i = 0; i < BODIES; ++i) {
        Aabb tight = tight_aabb(&s->bodies[i].pose);
        Aabb fat = aabb_grow(&tight, shape_margin(s->bodies[i].pose.half_extents));
        s->body_proxies[i] = broad_phase_create_proxy(&s->broad_phase, (ShapeRef){ i, false }, &fat, false);
    }
}

/// Advances the bodies and moves the proxies whose fat bounds they left.
static void move_bodies(BroadPhaseCase* s) {
    for (uint32_t i = 0; i < BODIES; ++i) {
        Body* b = &s->bodies[i];
        Vec3 c = v3_add(b->pose.center, v3_scale(b->velocity, dt));
        b->pose.center = (Vec3){ wrap(c.x, extent_xz), wrap(c.y, extent_y), wrap(c.z, extent_xz) };
        b->rotation = integrate_rotation(b->rotation, v3_scale(b->spin, dt));
        b->pose.basis = basis_from_quat(b->rotation);
        Aabb tight = tight_aabb(&b->pose);
        if (!aabb_contains(broad_phase_fat_aabb(&s->broad_phase, s->body_proxies[i]), &tight)) {
            Aabb fat = aabb_grow(&tight, shape_margin(b->pose.half_extents));
            broad_phase_move_proxy(&s->broad_phase, s->body_proxies[i], &fat);
        }
    }
}

/// Records a new pair unless its key is already known.
static void record_pair(void* context, int32_t a, int32_t b) {
    BroadPhaseCase* s = context;
    uint64_t key = pair_key(broad_phase_shape(&s->broad_phase, a), broad_phase_shape(&s->broad_phase, b));
    if (u64map_contains(&s->pair_index, key)) return;
    uint32_t id;
    if (s->free_pairs.len == 0) {
        id = (uint32_t)s->pairs.len;
        ARRAY_PUSH(s->pairs, (Pair){ 0 });
    } else {
        id = ARRAY_POP(s->free_pairs);
    }
    s->pairs.data[id] = (Pair){ a, b, key, true };
    u64map_insert(&s->pair_index, key, id);
}

/// Drops pairs whose fat bounds no longer overlap.
static void drop_parted_pairs(BroadPhaseCase* s) {
    for (uint32_t id = 0; id < s->pairs.len; ++id) {
        Pair* pair = &s->pairs.data[id];
        if (!pair->alive) continue;
        if (aabb_overlaps(broad_phase_fat_aabb(&s->broad_phase, pair->proxy_a),
                          broad_phase_fat_aabb(&s->broad_phase, pair->proxy_b))) {
            continue;
        }
        u64map_erase(&s->pair_index, pair->key);
        pair->alive = false;
        ARRAY_PUSH(s->free_pairs, id);
    }
}

/// Steps every frame and hashes the pair counts, pairs and fat bounds.
static uint64_t run(void* state) {
    BroadPhaseCase* s = state;
    reset(s);
    uint64_t h = 0;
    for (int frame = 0; frame < FRAMES; ++frame) {
        move_bodies(s);
        broad_phase_update_pairs(&s->broad_phase, record_pair, s);
        drop_parted_pairs(s);
        h = hash_add(h, u64map_size(&s->pair_index));
    }
    for (size_t i = 0; i < s->pairs.len; ++i) {
        const Pair* pair = &s->pairs.data[i];
        h = hash_add(h, pair->alive ? pair->key : 0u);
    }
    for (uint32_t i = 0; i < BODIES; ++i) {
        const Aabb* fat = broad_phase_fat_aabb(&s->broad_phase, s->body_proxies[i]);
        h = hash_add(h, (uint64_t)f32_bits(fat->min.x) | ((uint64_t)f32_bits(fat->max.y) << 32));
    }
    return h;
}

/// Frees the bodies, tiles, broad phase and pair records.
static void teardown(void* state) {
    BroadPhaseCase* s = state;
    free(s->initial);
    free(s->bodies);
    free(s->tiles);
    free(s->body_proxies);
    broad_phase_free(&s->broad_phase);
    ARRAY_FREE(s->pairs);
    ARRAY_FREE(s->free_pairs);
    u64map_free(&s->pair_index);
}

const Case broadphase_case = { "broadphase", sizeof(BroadPhaseCase), setup, run, teardown };
