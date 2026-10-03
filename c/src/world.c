#include "world.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "array.h"
#include "box_collision.h"
#include "broad_phase.h"
#include "constraint_graph.h"
#include "contact.h"
#include "contact_recycle.h"
#include "contact_solver.h"
#include "hash.h"
#include "rigid_body.h"
#include "task_pool.h"
#include "u64map.h"
#include "vecmath.h"

enum {
    PILES_SIDE = 12,
    PILE_HEIGHT = 6,
    REFRESH_GRAIN = 256,
    PAIR_GRAIN = 64,
    COLLIDE_GRAIN = 128,
    STEPS = 64,
    SHOVE_STEP = 45,
};

static const float max_aabb_margin = 0.05f;
static const float aabb_margin_fraction = 0.125f;
static const float sleep_velocity_threshold = 0.05f;
static const float sleep_angular_velocity_threshold = 0.05f;
static const float time_to_sleep = 0.5f;
static const float pile_spacing = 3.0f;

typedef enum { EDGE_A = 0, EDGE_B = 1 } EdgeSide;

typedef struct {
    uint32_t set;
    uint32_t local;
    uint32_t island;
    uint32_t island_local;
    U32Array edges;
    bool alive;
    bool logged;
} BodyRecord;

typedef struct {
    uint32_t contact;
    uint32_t body_a;
    uint32_t body_b;
} ContactLink;

typedef ARRAY_OF(ContactLink) ContactLinkArray;

typedef struct {
    uint32_t set;
    uint32_t local;
    uint32_t remove_count;
    U32Array bodies;
    ContactLinkArray contacts;
    bool alive;
} Island;

typedef struct {
    U32Array bodies;
    U32Array contacts;
    U32Array islands;
    bool alive;
} SleepingSet;

typedef struct {
    int32_t proxy;
    Aabb fat;
} ProxyMove;

typedef struct {
    int32_t a;
    int32_t b;
    uint64_t key;
} ProxyPair;

typedef ARRAY_OF(ProxyMove) ProxyMoveArray;
typedef ARRAY_OF(ProxyPair) ProxyPairArray;

struct World {
    TaskPool* pool;
    SolverContext context;
    CollisionTolerances tolerances;

    ARRAY_OF(RigidBody) rigid;
    ARRAY_OF(BodyRecord) bodies;
    I32Array rigid_proxies;
    RigidBody ground;
    int32_t ground_proxy;
    KinematicMotion static_motions[1];

    BroadPhase broad_phase;
    ARRAY_OF(Contact) contacts;
    U32Array free_contacts;
    U64Map contact_index;
    U32Array awake_contacts;
    U64Array awake_bits;
    U32Array disabled_contacts;
    ConstraintGraph graph;
    ARRAY_OF(U32Array) static_edges;

    U32Array awake_bodies;
    ARRAY_OF(Island) islands;
    U32Array free_islands;
    U32Array awake_islands;
    ARRAY_OF(SleepingSet) sleeping_sets;
    U32Array free_sets;
    uint32_t live_islands;
    uint32_t sleeping_body_count;
    uint32_t sleeping_touching;
    uint32_t sleeping_points;
    uint32_t split_island_id;
    U32Array change_log;
    U32Array pending_changes;
    U8Array island_awake;

    U32Array collide_ids;
    ARRAY_OF(U32Array) changed_blocks;
    U32Array changed_ids;
    ARRAY_OF(ProxyMoveArray) proxy_moves;
    ARRAY_OF(ProxyPairArray) pair_candidates;
    ARRAY_OF(BodyDelta) deltas;
    ARRAY_OF(ActiveBody) active_bodies;
    U32Array body_local;
    U32Span color_lists[GRAPH_COLOR_COUNT];
    ContactSolver* solver;
};

/// A moved-proxy query's world and the block's list of new pairs.
typedef struct {
    const World* world;
    ProxyPairArray* found;
} PairQuery;

/// Fat margin for a box, the engine's shape_margin.
static float shape_margin(Vec3 h) { return f32_min(max_aabb_margin, aabb_margin_fraction * 2.0f * f32_max3(h.x, h.y, h.z)); }

/// World pose of a body's single box, shape transform identity.
static BoxPose world_pose(const Transform* body, Vec3 half_extents) {
    Transform shape = transform_identity();
    BoxPose pose;
    pose.half_extents = half_extents;
    pose.center = transform_point(body, shape.origin);
    pose.basis = basis_mul(&body->basis, &shape.basis);
    return pose;
}

/// Bounds of a body's box grown by the speculative distance.
static Aabb tight_bounds(const World* w, const RigidBody* body) {
    BoxPose pose = world_pose(&body->transform, body->half_extents);
    Aabb box = box_aabb(&pose);
    return aabb_grow(&box, w->tolerances.speculative_distance);
}

/// Orders unsigned 32-bit integers.
static int compare_u32(const void* a, const void* b) {
    uint32_t x = *(const uint32_t*)a;
    uint32_t y = *(const uint32_t*)b;
    return x < y ? -1 : x > y;
}

/// Runs `fn(world, begin, end)` over [0, count), on the pool when
/// worthwhile.
static void world_parallel_for(World* w, size_t count, size_t grain, TaskRangeFn fn) {
    if (w->pool != NULL && count >= 2 * grain) {
        task_pool_parallel_for(w->pool, count, grain, fn, w);
    } else if (count != 0) {
        fn(w, 0, count);
    }
}

/// A body record in no set and no island.
static BodyRecord body_record_new(void) {
    return (BodyRecord){ .set = NULL_LINK, .local = NULL_LINK, .island = NULL_LINK, .island_local = NULL_LINK };
}

/// An island in no set.
static Island island_new(void) { return (Island){ .set = NULL_LINK, .local = NULL_LINK }; }

/// Releases an island's lists.
static void island_release(Island* island) {
    ARRAY_FREE(island->bodies);
    ARRAY_FREE(island->contacts);
}

/// Releases a sleeping set's lists.
static void sleeping_set_release(SleepingSet* set) {
    ARRAY_FREE(set->bodies);
    ARRAY_FREE(set->contacts);
    ARRAY_FREE(set->islands);
}

/// Releases every body record's edge list and empties the records.
static void clear_bodies(World* w) {
    for (size_t i = 0; i < w->bodies.len; ++i) ARRAY_FREE(w->bodies.data[i].edges);
    w->bodies.len = 0;
}

/// Releases every island's lists and empties the islands.
static void clear_islands(World* w) {
    for (size_t i = 0; i < w->islands.len; ++i) island_release(&w->islands.data[i]);
    w->islands.len = 0;
}

/// Releases every sleeping set's lists and empties the sets.
static void clear_sleeping_sets(World* w) {
    for (size_t i = 0; i < w->sleeping_sets.len; ++i) sleeping_set_release(&w->sleeping_sets.data[i]);
    w->sleeping_sets.len = 0;
}

/// Releases every static edge list and empties them.
static void clear_static_edges(World* w) {
    for (size_t i = 0; i < w->static_edges.len; ++i) ARRAY_FREE(w->static_edges.data[i]);
    w->static_edges.len = 0;
}

/// A new island in `set`.
static uint32_t create_island(World* w, uint32_t set) {
    uint32_t id;
    if (w->free_islands.len == 0) {
        id = (uint32_t)w->islands.len;
        ARRAY_PUSH(w->islands, island_new());
    } else {
        id = ARRAY_POP(w->free_islands);
    }
    Island* island = &w->islands.data[id];
    island_release(island);
    *island = island_new();
    island->alive = true;
    island->set = set;
    if (set == AWAKE_SET) {
        island->local = (uint32_t)w->awake_islands.len;
        ARRAY_PUSH(w->awake_islands, id);
    }
    ++w->live_islands;
    return id;
}

/// Frees an island.
static void destroy_island(World* w, uint32_t id) {
    if (w->split_island_id == id) w->split_island_id = NULL_LINK;
    Island* island = &w->islands.data[id];
    U32Array* list = island->set == AWAKE_SET ? &w->awake_islands : &w->sleeping_sets.data[island->set].islands;
    uint32_t last = ARRAY_BACK(*list);
    list->data[island->local] = last;
    list->len--;
    if (island->local < list->len) w->islands.data[last].local = island->local;
    island_release(island);
    *island = island_new();
    ARRAY_PUSH(w->free_islands, id);
    --w->live_islands;
}

/// Puts a body in the awake set in an island of its own.
static void activate_body(World* w, uint32_t slot) {
    w->bodies.data[slot].set = AWAKE_SET;
    w->bodies.data[slot].local = (uint32_t)w->awake_bodies.len;
    ARRAY_PUSH(w->awake_bodies, slot);
    uint32_t island_id = create_island(w, AWAKE_SET);
    BodyRecord* record = &w->bodies.data[slot];
    record->island = island_id;
    record->island_local = 0;
    ARRAY_PUSH(w->islands.data[island_id].bodies, slot);
}

/// Adds a body at `position` with half extents `half` and `rotation`.
static void create_body(World* w, Vec3 position, Vec3 half, Quat rotation) {
    uint32_t slot = (uint32_t)w->rigid.len;
    ARRAY_PUSH(w->rigid, rigid_body_new());
    RigidBody* body = &ARRAY_BACK(w->rigid);
    body->rotation = rotation;
    body->transform = (Transform){ basis_from_quat(rotation), position };
    set_box_mass(body, half);
    BodyRecord record = body_record_new();
    record.alive = true;
    ARRAY_PUSH(w->bodies, record);
    activate_body(w, slot);
    Aabb tight = tight_bounds(w, body);
    Aabb fat = aabb_grow(&tight, shape_margin(half));
    ARRAY_PUSH(w->rigid_proxies, broad_phase_create_proxy(&w->broad_phase, (ShapeRef){ slot, false }, &fat, false));
}

/// Queues a body for the next step's change processing.
static void notify(World* w, uint32_t slot) {
    BodyRecord* record = &w->bodies.data[slot];
    if (record->logged) return;
    record->logged = true;
    ARRAY_PUSH(w->change_log, slot);
}

/// Body of a shape.
static const RigidBody* body_of(const World* w, ShapeRef ref) { return ref.is_static ? &w->ground : &w->rigid.data[ref.body]; }

/// Pose of a shape in world space.
static BoxPose pose_of(const World* w, ShapeRef ref) {
    const RigidBody* body = body_of(w, ref);
    return world_pose(&body->transform, body->half_extents);
}

/// Edge list of a shape's body.
static U32Array* edges_of(World* w, ShapeRef ref) {
    if (ref.is_static) return &w->static_edges.data[ref.body];
    return &w->bodies.data[ref.body].edges;
}

/// Records a contact in one of its bodies' edge lists.
static void add_edge(World* w, uint32_t id, EdgeSide side) {
    Contact* contact = &w->contacts.data[id];
    uint32_t index = (uint32_t)side;
    U32Array* list = edges_of(w, index == 0 ? contact->shape_a : contact->shape_b);
    contact->edge_local[index] = (uint32_t)list->len;
    ARRAY_PUSH(*list, (id << 1u) | index);
}

/// Removes a contact from one of its bodies' edge lists.
static void remove_edge(World* w, uint32_t id, EdgeSide side) {
    Contact* contact = &w->contacts.data[id];
    uint32_t index = (uint32_t)side;
    U32Array* list = edges_of(w, index == 0 ? contact->shape_a : contact->shape_b);
    uint32_t local = contact->edge_local[index];
    uint32_t last = ARRAY_BACK(*list);
    list->data[local] = last;
    list->len--;
    if (local < list->len) w->contacts.data[last >> 1u].edge_local[last & 1u] = local;
    contact->edge_local[index] = NULL_LINK;
}

/// Swap-removes a contact from a contact list.
static void list_remove(World* w, U32Array* list, uint32_t id) {
    uint32_t local = w->contacts.data[id].local;
    uint32_t last = ARRAY_BACK(*list);
    list->data[local] = last;
    list->len--;
    if (local < list->len) w->contacts.data[last].local = local;
    w->contacts.data[id].local = NULL_LINK;
}

/// Sets or clears a contact's bit in the awake bitset.
static void mark_awake(World* w, uint32_t id, bool awake) {
    size_t word = id / 64u;
    if (w->awake_bits.len <= word) ARRAY_RESIZE(w->awake_bits, word + 1);
    uint64_t bit = (uint64_t)1 << (id % 64u);
    if (awake) {
        w->awake_bits.data[word] |= bit;
    } else {
        w->awake_bits.data[word] &= ~bit;
    }
}

/// Adds a non-touching contact to the awake list.
static void awake_add(World* w, uint32_t id) {
    mark_awake(w, id, true);
    Contact* contact = &w->contacts.data[id];
    contact->set = AWAKE_SET;
    contact->color = NULL_LINK;
    contact->local = (uint32_t)w->awake_contacts.len;
    ARRAY_PUSH(w->awake_contacts, id);
}

/// Adds a contact to the list of contacts between sleeping bodies.
static void disabled_add(World* w, uint32_t id) {
    mark_awake(w, id, false);
    Contact* contact = &w->contacts.data[id];
    contact->set = DISABLED_SET;
    contact->color = NULL_LINK;
    contact->local = (uint32_t)w->disabled_contacts.len;
    ARRAY_PUSH(w->disabled_contacts, id);
}

/// The graph's view of a contact's bodies.
static GraphBodies graph_bodies(const Contact* contact) {
    bool b_static = contact->shape_b.is_static;
    return (GraphBodies){ contact->shape_a.body, b_static ? 0u : contact->shape_b.body, b_static };
}

/// Colours a touching contact into the constraint graph.
static void graph_add(World* w, uint32_t id) {
    Contact* contact = &w->contacts.data[id];
    constraint_graph_reserve_bodies(&w->graph, w->bodies.len);
    GraphSlot slot = constraint_graph_add(&w->graph, id, graph_bodies(contact));
    contact->color = slot.color;
    contact->local = slot.local;
    contact->set = AWAKE_SET;
    mark_awake(w, id, true);
}

/// Takes a contact out of the constraint graph.
static void graph_remove(World* w, uint32_t id) {
    Contact* contact = &w->contacts.data[id];
    uint32_t moved = constraint_graph_remove(&w->graph, (GraphSlot){ contact->color, contact->local }, graph_bodies(contact));
    if (moved != NULL_LINK) w->contacts.data[moved].local = contact->local;
    contact->color = NULL_LINK;
    contact->local = NULL_LINK;
}

/// Moves a sleeping set back to the awake set.
static void wake_set(World* w, uint32_t set_id) {
    SleepingSet* set = &w->sleeping_sets.data[set_id];
    for (size_t i = 0; i < set->bodies.len; ++i) {
        uint32_t slot = set->bodies.data[i];
        BodyRecord* record = &w->bodies.data[slot];
        record->set = AWAKE_SET;
        record->local = (uint32_t)w->awake_bodies.len;
        ARRAY_PUSH(w->awake_bodies, slot);
        RigidBody* body = &w->rigid.data[slot];
        body->sleeping = false;
        body->sleep_time = 0.0f;
        for (size_t e = 0; e < record->edges.len; ++e) {
            uint32_t contact_id = record->edges.data[e] >> 1u;
            if (w->contacts.data[contact_id].set == DISABLED_SET) {
                list_remove(w, &w->disabled_contacts, contact_id);
                awake_add(w, contact_id);
            }
        }
    }
    for (size_t i = 0; i < set->contacts.len; ++i) {
        uint32_t contact_id = set->contacts.data[i];
        --w->sleeping_touching;
        w->sleeping_points -= w->contacts.data[contact_id].manifold.point_count;
        graph_add(w, contact_id);
    }
    for (size_t i = 0; i < set->islands.len; ++i) {
        uint32_t island_id = set->islands.data[i];
        Island* island = &w->islands.data[island_id];
        island->set = AWAKE_SET;
        island->local = (uint32_t)w->awake_islands.len;
        ARRAY_PUSH(w->awake_islands, island_id);
    }
    w->sleeping_body_count -= (uint32_t)set->bodies.len;
    sleeping_set_release(set);
    *set = (SleepingSet){ 0 };
    ARRAY_PUSH(w->free_sets, set_id);
}

/// Wakes the sets of bodies whose state changed since the last step.
static void process_body_changes(World* w) {
    if (w->change_log.len == 0) return;
    w->pending_changes.len = 0;
    for (size_t i = 0; i < w->change_log.len; ++i) ARRAY_PUSH(w->pending_changes, w->change_log.data[i]);
    w->change_log.len = 0;
    qsort(w->pending_changes.data, w->pending_changes.len, sizeof(uint32_t), compare_u32);
    for (size_t i = 0; i < w->pending_changes.len; ++i) {
        uint32_t slot = w->pending_changes.data[i];
        BodyRecord* record = &w->bodies.data[slot];
        record->logged = false;
        if (!w->rigid.data[slot].sleeping && record->set != AWAKE_SET) wake_set(w, record->set);
    }
}

/// Creates a contact for a new pair.
static uint32_t create_contact(World* w, const ProxyPair* pair) {
    uint32_t id;
    if (w->free_contacts.len == 0) {
        id = (uint32_t)w->contacts.len;
        ARRAY_PUSH(w->contacts, contact_new());
    } else {
        id = ARRAY_POP(w->free_contacts);
    }
    Contact* contact = &w->contacts.data[id];
    *contact = contact_new();
    contact->alive = true;
    contact->shape_a = broad_phase_shape(&w->broad_phase, pair->a);
    contact->shape_b = broad_phase_shape(&w->broad_phase, pair->b);
    contact->proxy_a = pair->a;
    contact->proxy_b = pair->b;
    u64map_insert(&w->contact_index, pair->key, id);
    add_edge(w, id, EDGE_A);
    add_edge(w, id, EDGE_B);

    bool a_awake = w->bodies.data[contact->shape_a.body].set == AWAKE_SET;
    bool b_awake = !contact->shape_b.is_static && w->bodies.data[contact->shape_b.body].set == AWAKE_SET;
    if (a_awake || b_awake) {
        awake_add(w, id);
    } else {
        disabled_add(w, id);
    }
    return id;
}

/// Unlinks a contact that stopped touching from its island.
static void unlink_contact(World* w, uint32_t id) {
    Contact* contact = &w->contacts.data[id];
    Island* island = &w->islands.data[contact->island];
    uint32_t local = contact->island_local;
    ContactLink last = ARRAY_BACK(island->contacts);
    island->contacts.data[local] = last;
    island->contacts.len--;
    if (local < island->contacts.len) w->contacts.data[last.contact].island_local = local;
    ++island->remove_count;
    contact->island = NULL_LINK;
    contact->island_local = NULL_LINK;
    contact->linked = false;
}

/// Destroys a contact and unlinks it from everything.
static void destroy_contact(World* w, uint32_t id, bool wake) {
    Contact* contact = &w->contacts.data[id];
    u64map_erase(&w->contact_index, pair_key(contact->shape_a, contact->shape_b));
    uint32_t slot_a = contact->shape_a.body;
    uint32_t slot_b = contact->shape_b.is_static ? NULL_LINK : contact->shape_b.body;

    if (wake && contact->linked) {
        uint32_t set_a = w->bodies.data[slot_a].set;
        if (set_a != AWAKE_SET && set_a != NULL_LINK) wake_set(w, set_a);
        if (slot_b != NULL_LINK) {
            uint32_t set_b = w->bodies.data[slot_b].set;
            if (set_b != AWAKE_SET && set_b != NULL_LINK) wake_set(w, set_b);
        }
    }

    remove_edge(w, id, EDGE_A);
    remove_edge(w, id, EDGE_B);

    if (contact->island != NULL_LINK) unlink_contact(w, id);
    if (contact->color != NULL_LINK) {
        graph_remove(w, id);
    } else if (contact->set == AWAKE_SET) {
        list_remove(w, &w->awake_contacts, id);
    } else if (contact->set == DISABLED_SET) {
        list_remove(w, &w->disabled_contacts, id);
    } else if (contact->set != NULL_LINK) {
        SleepingSet* set = &w->sleeping_sets.data[contact->set];
        list_remove(w, &set->contacts, id);
        if (contact->touching) {
            --w->sleeping_touching;
            w->sleeping_points -= contact->manifold.point_count;
        }
    }
    mark_awake(w, id, false);
    contact->alive = false;
    contact->set = NULL_LINK;
    ARRAY_PUSH(w->free_contacts, id);
}

/// Merges the smaller island into the larger; returns the survivor.
static uint32_t merge_islands(World* w, uint32_t a, uint32_t b) {
    if (a == b) return a;
    if (a == NULL_LINK) return b;
    if (b == NULL_LINK) return a;
    uint32_t big = a;
    uint32_t small = b;
    if (w->islands.data[a].bodies.len < w->islands.data[b].bodies.len) {
        big = b;
        small = a;
    }
    Island* big_island = &w->islands.data[big];
    Island* small_island = &w->islands.data[small];
    for (size_t i = 0; i < small_island->bodies.len; ++i) {
        uint32_t slot = small_island->bodies.data[i];
        BodyRecord* record = &w->bodies.data[slot];
        record->island = big;
        record->island_local = (uint32_t)big_island->bodies.len;
        ARRAY_PUSH(big_island->bodies, slot);
    }
    for (size_t i = 0; i < small_island->contacts.len; ++i) {
        ContactLink link = small_island->contacts.data[i];
        Contact* contact = &w->contacts.data[link.contact];
        contact->island = big;
        contact->island_local = (uint32_t)big_island->contacts.len;
        ARRAY_PUSH(big_island->contacts, link);
    }
    big_island->remove_count += small_island->remove_count;
    destroy_island(w, small);
    return big;
}

/// Links a touching contact into its bodies' islands, merging them.
static void link_contact(World* w, uint32_t id) {
    Contact* contact = &w->contacts.data[id];
    uint32_t slot_a = contact->shape_a.body;
    bool b_static = contact->shape_b.is_static;
    uint32_t slot_b = b_static ? NULL_LINK : contact->shape_b.body;

    if (slot_b != NULL_LINK) {
        uint32_t set_a = w->bodies.data[slot_a].set;
        uint32_t set_b = w->bodies.data[slot_b].set;
        if (set_a == AWAKE_SET && set_b != AWAKE_SET && set_b != NULL_LINK) {
            wake_set(w, set_b);
        } else if (set_b == AWAKE_SET && set_a != AWAKE_SET && set_a != NULL_LINK) {
            wake_set(w, set_a);
        }
    }

    uint32_t island_a = w->bodies.data[slot_a].island;
    uint32_t island_b = slot_b == NULL_LINK ? NULL_LINK : w->bodies.data[slot_b].island;
    uint32_t merged = merge_islands(w, island_a, island_b);

    Island* island = &w->islands.data[merged];
    contact->island = merged;
    contact->island_local = (uint32_t)island->contacts.len;
    ARRAY_PUSH(island->contacts, (ContactLink){ id, slot_a, slot_b });
    contact->linked = true;
}

/// Root of `node` in a union-find forest, halving the path on the way.
static uint32_t find_root(uint32_t* parents, uint32_t node) {
    while (parents[node] != node) {
        parents[node] = parents[parents[node]];
        node = parents[node];
    }
    return node;
}

/// Splits an island into its connected components.
static void split_island(World* w, uint32_t base_id) {
    U32Array base_bodies = w->islands.data[base_id].bodies;
    ContactLinkArray base_contacts = w->islands.data[base_id].contacts;
    w->islands.data[base_id].bodies = (U32Array){ 0 };
    w->islands.data[base_id].contacts = (ContactLinkArray){ 0 };
    uint32_t count = (uint32_t)base_bodies.len;

    uint32_t* parents = xalloc(count * sizeof(uint32_t));
    uint32_t* ranks = xalloc(count * sizeof(uint32_t));
    for (uint32_t i = 0; i < count; ++i) parents[i] = i;
    for (size_t i = 0; i < base_contacts.len; ++i) {
        const ContactLink* link = &base_contacts.data[i];
        if (link->body_b == NULL_LINK) continue;
        uint32_t root_a = find_root(parents, w->bodies.data[link->body_a].island_local);
        uint32_t root_b = find_root(parents, w->bodies.data[link->body_b].island_local);
        if (root_a == root_b) continue;
        if (ranks[root_a] < ranks[root_b]) {
            uint32_t swap = root_a;
            root_a = root_b;
            root_b = swap;
        }
        parents[root_b] = root_a;
        if (ranks[root_a] == ranks[root_b]) ++ranks[root_a];
    }

    uint32_t components = 0;
    for (uint32_t i = 0; i < count; ++i) {
        parents[i] = find_root(parents, i);
        components += parents[i] == i ? 1u : 0u;
    }
    if (components == 1) {
        Island* island = &w->islands.data[base_id];
        island->bodies = base_bodies;
        island->contacts = base_contacts;
        island->remove_count = 0;
        free(parents);
        free(ranks);
        return;
    }

    uint32_t* root_island = xalloc(count * sizeof(uint32_t));
    U32Array island_ids = { 0 };
    ARRAY_RESERVE(island_ids, components);
    for (uint32_t i = 0; i < count; ++i) {
        root_island[i] = NULL_LINK;
        if (parents[i] == i) {
            root_island[i] = (uint32_t)island_ids.len;
            uint32_t created = create_island(w, AWAKE_SET);
            ARRAY_PUSH(island_ids, created);
        }
    }
    for (uint32_t i = 0; i < count; ++i) {
        uint32_t slot = base_bodies.data[i];
        uint32_t target = island_ids.data[root_island[parents[i]]];
        Island* island = &w->islands.data[target];
        BodyRecord* record = &w->bodies.data[slot];
        record->island = target;
        record->island_local = (uint32_t)island->bodies.len;
        ARRAY_PUSH(island->bodies, slot);
    }
    for (size_t i = 0; i < base_contacts.len; ++i) {
        ContactLink link = base_contacts.data[i];
        uint32_t target = w->bodies.data[link.body_a].island;
        Island* island = &w->islands.data[target];
        Contact* contact = &w->contacts.data[link.contact];
        contact->island = target;
        contact->island_local = (uint32_t)island->contacts.len;
        ARRAY_PUSH(island->contacts, link);
    }
    destroy_island(w, base_id);
    free(parents);
    free(ranks);
    free(root_island);
    ARRAY_FREE(island_ids);
    ARRAY_FREE(base_bodies);
    ARRAY_FREE(base_contacts);
}

/// Splits the island chosen by the last sleep pass.
static void split_pending_island(World* w) {
    uint32_t id = w->split_island_id;
    w->split_island_id = NULL_LINK;
    if (id == NULL_LINK || id >= w->islands.len) return;
    const Island* island = &w->islands.data[id];
    if (!island->alive || island->set != AWAKE_SET || island->remove_count == 0) return;
    split_island(w, id);
}

/// Moves the awake contacts of a body falling asleep whose other body is
/// not awake to the disabled list.
static void disable_resting_edges(World* w, const BodyRecord* record) {
    for (size_t e = 0; e < record->edges.len; ++e) {
        uint32_t key = record->edges.data[e];
        uint32_t contact_id = key >> 1u;
        const Contact* contact = &w->contacts.data[contact_id];
        if (contact->color != NULL_LINK) continue;
        uint32_t side = key & 1u;
        uint32_t other = NULL_LINK;
        if (side == 1u) {
            other = w->bodies.data[contact->shape_a.body].set;
        } else if (!contact->shape_b.is_static) {
            other = w->bodies.data[contact->shape_b.body].set;
        }
        if (other == AWAKE_SET) continue;
        list_remove(w, &w->awake_contacts, contact_id);
        disabled_add(w, contact_id);
    }
}

/// Moves a resting island and its contacts into a new sleeping set.
static void try_sleep_island(World* w, uint32_t id) {
    if (w->islands.data[id].remove_count > 0 && w->islands.data[id].bodies.len > 1) return;
    uint32_t set_id;
    if (w->free_sets.len == 0) {
        set_id = (uint32_t)w->sleeping_sets.len;
        ARRAY_PUSH(w->sleeping_sets, (SleepingSet){ 0 });
    } else {
        set_id = ARRAY_POP(w->free_sets);
    }
    SleepingSet* set = &w->sleeping_sets.data[set_id];
    sleeping_set_release(set);
    *set = (SleepingSet){ 0 };
    set->alive = true;
    Island* island = &w->islands.data[id];

    for (size_t i = 0; i < island->bodies.len; ++i) {
        uint32_t slot = island->bodies.data[i];
        BodyRecord* record = &w->bodies.data[slot];
        uint32_t last = ARRAY_BACK(w->awake_bodies);
        w->awake_bodies.data[record->local] = last;
        w->awake_bodies.len--;
        if (record->local < w->awake_bodies.len) w->bodies.data[last].local = record->local;
        RigidBody* body = &w->rigid.data[slot];
        body->sleeping = true;
        body->linear_velocity = (Vec3){ 0.0f, 0.0f, 0.0f };
        body->angular_velocity = (Vec3){ 0.0f, 0.0f, 0.0f };
        body->sleep_velocity = 0.0f;
        record->set = set_id;
        record->local = (uint32_t)set->bodies.len;
        ARRAY_PUSH(set->bodies, slot);
        disable_resting_edges(w, record);
    }

    for (size_t i = 0; i < island->contacts.len; ++i) {
        uint32_t contact_id = island->contacts.data[i].contact;
        Contact* contact = &w->contacts.data[contact_id];
        graph_remove(w, contact_id);
        mark_awake(w, contact_id, false);
        contact->set = set_id;
        contact->local = (uint32_t)set->contacts.len;
        ARRAY_PUSH(set->contacts, contact_id);
        ++w->sleeping_touching;
        w->sleeping_points += contact->manifold.point_count;
    }

    uint32_t last = ARRAY_BACK(w->awake_islands);
    w->awake_islands.data[island->local] = last;
    w->awake_islands.len--;
    if (island->local < w->awake_islands.len) w->islands.data[last].local = island->local;
    island->set = set_id;
    island->local = (uint32_t)set->islands.len;
    ARRAY_PUSH(set->islands, id);
    w->sleeping_body_count += (uint32_t)set->bodies.len;
    if (w->split_island_id == id) w->split_island_id = NULL_LINK;
}

/// Finds the awake bodies of [begin, end) whose bounds left their fat
/// bounds.
static void refresh_range(void* data, size_t begin, size_t end) {
    World* w = data;
    ProxyMoveArray* moves = &w->proxy_moves.data[begin / REFRESH_GRAIN];
    for (size_t index = begin; index < end; ++index) {
        uint32_t slot = w->awake_bodies.data[index];
        const RigidBody* body = &w->rigid.data[slot];
        Aabb tight = tight_bounds(w, body);
        int32_t proxy = w->rigid_proxies.data[slot];
        if (!aabb_contains(broad_phase_fat_aabb(&w->broad_phase, proxy), &tight)) {
            ARRAY_PUSH(*moves, (ProxyMove){ proxy, aabb_grow(&tight, shape_margin(body->half_extents)) });
        }
    }
}

/// Moves the proxies of awake bodies whose bounds left their fat bounds.
static void refresh_proxy_bounds(World* w) {
    size_t count = w->awake_bodies.len;
    if (count == 0) return;
    size_t blocks = (count + REFRESH_GRAIN - 1) / REFRESH_GRAIN;
    if (w->proxy_moves.len < blocks) ARRAY_RESIZE(w->proxy_moves, blocks);
    for (size_t block = 0; block < blocks; ++block) w->proxy_moves.data[block].len = 0;
    world_parallel_for(w, count, REFRESH_GRAIN, refresh_range);
    for (size_t block = 0; block < blocks; ++block) {
        const ProxyMoveArray* moves = &w->proxy_moves.data[block];
        for (size_t i = 0; i < moves->len; ++i) broad_phase_move_proxy(&w->broad_phase, moves->data[i].proxy, &moves->data[i].fat);
    }
}

/// Keeps a found pair that has no contact yet.
static void collect_pair(void* context, int32_t a, int32_t b) {
    PairQuery* query = context;
    const BroadPhase* bp = &query->world->broad_phase;
    uint64_t key = pair_key(broad_phase_shape(bp, a), broad_phase_shape(bp, b));
    if (!u64map_contains(&query->world->contact_index, key)) ARRAY_PUSH(*query->found, (ProxyPair){ a, b, key });
}

/// Finds the new pairs of the moved proxies in [begin, end).
static void pairs_range(void* data, size_t begin, size_t end) {
    World* w = data;
    PairQuery query = { w, &w->pair_candidates.data[begin / PAIR_GRAIN] };
    broad_phase_query_moved(&w->broad_phase, begin, end, collect_pair, &query);
}

/// Creates contacts for the new pairs of the moved proxies.
static void update_pairs(World* w) {
    size_t moved = broad_phase_moved_count(&w->broad_phase);
    size_t blocks = (moved + PAIR_GRAIN - 1) / PAIR_GRAIN;
    if (w->pair_candidates.len < blocks) ARRAY_RESIZE(w->pair_candidates, blocks);
    for (size_t block = 0; block < blocks; ++block) w->pair_candidates.data[block].len = 0;
    world_parallel_for(w, moved, PAIR_GRAIN, pairs_range);
    for (size_t block = 0; block < blocks; ++block) {
        const ProxyPairArray* found = &w->pair_candidates.data[block];
        for (size_t i = 0; i < found->len; ++i) {
            if (!u64map_contains(&w->contact_index, found->data[i].key)) create_contact(w, &found->data[i]);
        }
    }
    broad_phase_clear_moves(&w->broad_phase);
}

/// Collides the awake contacts [begin, end) of the collide list and records
/// those that began or ended touching.
static void collide_range(void* data, size_t begin, size_t end) {
    World* w = data;
    U32Array* changed = &w->changed_blocks.data[begin / COLLIDE_GRAIN];
    for (size_t i = begin; i < end; ++i) {
        uint32_t id = w->collide_ids.data[i];
        Contact* contact = &w->contacts.data[id];
        if (!aabb_overlaps(broad_phase_fat_aabb(&w->broad_phase, contact->proxy_a),
                           broad_phase_fat_aabb(&w->broad_phase, contact->proxy_b))) {
            contact->touching = false;
            ARRAY_PUSH(*changed, (id << 1u) | 1u);
            continue;
        }
        ContactPoses poses = { pose_of(w, contact->shape_a), pose_of(w, contact->shape_b) };
        if (!try_recycle_contact(contact, &poses, &w->tolerances)) {
            collide_boxes(&poses.a, &poses.b, &w->tolerances, &contact->manifold);
            cache_contact(contact, &poses);
        }
        contact->touching = contact->manifold.point_count > 0;
        const RigidBody* a = body_of(w, contact->shape_a);
        const RigidBody* b = body_of(w, contact->shape_b);
        contact->friction = sqrtf(a->friction * b->friction);
        contact->restitution = f32_max(a->restitution, b->restitution);
        contact->rolling_resistance = f32_max(a->rolling_resistance, b->rolling_resistance);
        if (contact->touching != contact->linked) ARRAY_PUSH(*changed, id << 1u);
    }
}

/// Applies the begin and end changes found by collide.
static void process_contact_changes(World* w) {
    for (size_t i = 0; i < w->changed_ids.len; ++i) {
        uint32_t encoded = w->changed_ids.data[i];
        uint32_t id = encoded >> 1u;
        Contact* contact = &w->contacts.data[id];
        if (!contact->alive) continue;
        if ((encoded & 1u) != 0u) {
            destroy_contact(w, id, false);
        } else if (contact->touching && !contact->linked) {
            link_contact(w, id);
            list_remove(w, &w->awake_contacts, id);
            graph_add(w, id);
        } else if (!contact->touching && contact->linked) {
            contact->was_touching = false;
            unlink_contact(w, id);
            graph_remove(w, id);
            awake_add(w, id);
        }
    }
}

/// Collides every awake contact and processes those that began or ended
/// touching.
static void collide(World* w) {
    w->collide_ids.len = 0;
    for (size_t word = 0; word < w->awake_bits.len; ++word) {
        uint64_t bits = w->awake_bits.data[word];
        while (bits != 0u) {
            ARRAY_PUSH(w->collide_ids, (uint32_t)(word * 64u + (unsigned)__builtin_ctzll(bits)));
            bits &= bits - 1u;
        }
    }
    size_t blocks = (w->collide_ids.len + COLLIDE_GRAIN - 1) / COLLIDE_GRAIN;
    if (w->changed_blocks.len < blocks) ARRAY_RESIZE(w->changed_blocks, blocks);
    for (size_t block = 0; block < blocks; ++block) w->changed_blocks.data[block].len = 0;
    world_parallel_for(w, w->collide_ids.len, COLLIDE_GRAIN, collide_range);

    w->changed_ids.len = 0;
    for (size_t block = 0; block < blocks; ++block) {
        const U32Array* changed = &w->changed_blocks.data[block];
        for (size_t i = 0; i < changed->len; ++i) ARRAY_PUSH(w->changed_ids, changed->data[i]);
    }
    qsort(w->changed_ids.data, w->changed_ids.len, sizeof(uint32_t), compare_u32);
    process_contact_changes(w);
}

/// Runs the contact solver over the awake bodies.
static void solve(World* w) {
    w->active_bodies.len = 0;
    if (w->body_local.len < w->bodies.len) ARRAY_RESIZE(w->body_local, w->bodies.len);
    w->body_local.len = w->bodies.len;
    for (uint32_t index = 0; index < w->awake_bodies.len; ++index) {
        uint32_t slot = w->awake_bodies.data[index];
        ARRAY_PUSH(w->active_bodies, (ActiveBody){ &w->rigid.data[slot], slot });
        w->body_local.data[slot] = index;
    }
    w->deltas.len = 0;
    for (size_t i = 0; i < w->awake_bodies.len; ++i) ARRAY_PUSH(w->deltas, (BodyDelta){ .rotation = quat_identity() });
    graph_color_spans(&w->graph, w->color_lists);
    SolverInputs inputs = {
        .contacts = w->contacts.data,
        .colors = w->color_lists,
        .body_local = w->body_local.data,
        .active_bodies = w->active_bodies.data,
        .active_count = w->active_bodies.len,
        .static_motions = w->static_motions,
        .static_count = 1,
        .deltas = w->deltas.data,
        .context = w->context,
        .pool = w->pool,
    };
    contact_solver_solve(w->solver, &inputs);
}

/// Updates sleep timers and puts resting islands to sleep.
static void finalize_sleep(World* w) {
    if (w->island_awake.len < w->islands.len) ARRAY_RESIZE(w->island_awake, w->islands.len);
    for (size_t i = 0; i < w->awake_islands.len; ++i) w->island_awake.data[w->awake_islands.data[i]] = 0u;
    uint32_t candidate = NULL_LINK;
    float candidate_time = 0.0f;
    for (uint32_t index = 0; index < w->awake_bodies.len; ++index) {
        uint32_t slot = w->awake_bodies.data[index];
        const BodyRecord* record = &w->bodies.data[slot];
        RigidBody* body = &w->rigid.data[slot];
        float reach = v3_length(body->max_extent);
        float velocity = v3_length(body->linear_velocity) + v3_length(body->angular_velocity) * reach;
        const BodyDelta* delta = &w->deltas.data[index];
        Vec3 axis = { delta->rotation.x, delta->rotation.y, delta->rotation.z };
        float angle = 2.0f * v3_length(axis);
        float moved = v3_length(delta->position) + 2.0f * angle * reach;
        float sleep_velocity = f32_max(velocity, 0.5f * w->context.inv_dt * moved);
        float angular_sleep_velocity = f32_max(v3_length(body->angular_velocity), angle * w->context.inv_dt);
        body->sleep_velocity = sleep_velocity;
        if (!body->can_sleep || sleep_velocity > sleep_velocity_threshold ||
            angular_sleep_velocity > sleep_angular_velocity_threshold) {
            body->sleep_time = 0.0f;
        } else {
            body->sleep_time += w->context.dt;
        }
        const Island* island = &w->islands.data[record->island];
        body->island = island->bodies.data[0];
        if (body->sleep_time <= time_to_sleep) {
            w->island_awake.data[record->island] = 1u;
        } else if (island->remove_count > 0 &&
                   (body->sleep_time > candidate_time || (body->sleep_time == candidate_time && record->island > candidate))) {
            candidate = record->island;
            candidate_time = body->sleep_time;
        }
    }
    w->split_island_id = candidate;
    for (size_t i = w->awake_islands.len; i > 0; --i) {
        uint32_t id = w->awake_islands.data[i - 1];
        if (w->island_awake.data[id] == 0u) try_sleep_island(w, id);
    }
}

World* world_create(uint32_t threads) {
    World* w = xalloc(sizeof(World));
    if (threads > 1) w->pool = task_pool_create(threads);
    SolverContext* c = &w->context;
    c->dt = 1.0f / 60.0f;
    c->inv_dt = 1.0f / c->dt;
    c->substeps = 4;
    c->h = c->dt / (float)c->substeps;
    c->inv_h = 1.0f / c->h;
    c->gravity = (Vec3){ 0.0f, -9.81f, 0.0f };
    float contact_hertz = f32_min(30.0f, 0.125f * c->inv_h);
    c->contact_softness = make_softness(contact_hertz, 10.0f, c->h);
    c->static_softness = make_softness(2.0f * contact_hertz, 0.5f * 10.0f, c->h);
    c->push_out_speed = 3.0f;
    c->restitution_threshold = 1.0f;
    c->linear_slop = 0.005f;
    w->tolerances = (CollisionTolerances){ 4.0f * c->linear_slop, c->linear_slop };
    w->ground = rigid_body_new();
    w->ground_proxy = BROAD_PHASE_NULL_PROXY;
    w->split_island_id = NULL_LINK;
    broad_phase_init(&w->broad_phase);
    w->solver = contact_solver_create();
    return w;
}

void world_destroy(World* w) {
    if (w->pool != NULL) task_pool_destroy(w->pool);
    ARRAY_FREE(w->rigid);
    clear_bodies(w);
    ARRAY_FREE(w->bodies);
    ARRAY_FREE(w->rigid_proxies);
    broad_phase_free(&w->broad_phase);
    ARRAY_FREE(w->contacts);
    ARRAY_FREE(w->free_contacts);
    u64map_free(&w->contact_index);
    ARRAY_FREE(w->awake_contacts);
    ARRAY_FREE(w->awake_bits);
    ARRAY_FREE(w->disabled_contacts);
    constraint_graph_free(&w->graph);
    clear_static_edges(w);
    ARRAY_FREE(w->static_edges);
    ARRAY_FREE(w->awake_bodies);
    clear_islands(w);
    ARRAY_FREE(w->islands);
    ARRAY_FREE(w->free_islands);
    ARRAY_FREE(w->awake_islands);
    clear_sleeping_sets(w);
    ARRAY_FREE(w->sleeping_sets);
    ARRAY_FREE(w->free_sets);
    ARRAY_FREE(w->change_log);
    ARRAY_FREE(w->pending_changes);
    ARRAY_FREE(w->island_awake);
    ARRAY_FREE(w->collide_ids);
    for (size_t i = 0; i < w->changed_blocks.len; ++i) ARRAY_FREE(w->changed_blocks.data[i]);
    ARRAY_FREE(w->changed_blocks);
    ARRAY_FREE(w->changed_ids);
    for (size_t i = 0; i < w->proxy_moves.len; ++i) ARRAY_FREE(w->proxy_moves.data[i]);
    ARRAY_FREE(w->proxy_moves);
    for (size_t i = 0; i < w->pair_candidates.len; ++i) ARRAY_FREE(w->pair_candidates.data[i]);
    ARRAY_FREE(w->pair_candidates);
    ARRAY_FREE(w->deltas);
    ARRAY_FREE(w->active_bodies);
    ARRAY_FREE(w->body_local);
    contact_solver_destroy(w->solver);
    free(w);
}

void world_reset(World* w) {
    w->rigid.len = 0;
    clear_bodies(w);
    w->rigid_proxies.len = 0;
    broad_phase_free(&w->broad_phase);
    broad_phase_init(&w->broad_phase);
    w->contacts.len = 0;
    w->free_contacts.len = 0;
    u64map_clear(&w->contact_index);
    w->awake_contacts.len = 0;
    w->awake_bits.len = 0;
    w->disabled_contacts.len = 0;
    constraint_graph_free(&w->graph);
    clear_static_edges(w);
    ARRAY_PUSH(w->static_edges, (U32Array){ 0 });
    w->awake_bodies.len = 0;
    clear_islands(w);
    w->free_islands.len = 0;
    w->awake_islands.len = 0;
    clear_sleeping_sets(w);
    ARRAY_PUSH(w->sleeping_sets, (SleepingSet){ 0 });
    w->free_sets.len = 0;
    w->live_islands = 0;
    w->sleeping_body_count = 0;
    w->sleeping_touching = 0;
    w->sleeping_points = 0;
    w->split_island_id = NULL_LINK;
    w->change_log.len = 0;

    Rng rng = { 0x3011d };
    ARRAY_RESERVE(w->rigid, PILES_SIDE * PILES_SIDE * PILE_HEIGHT + PILES_SIDE * PILES_SIDE / 4);
    for (uint32_t pz = 0; pz < PILES_SIDE; ++pz) {
        for (uint32_t px = 0; px < PILES_SIDE; ++px) {
            float x = (float)px * pile_spacing - 16.5f + rng_unit(&rng) * 0.04f - 0.02f;
            float z = (float)pz * pile_spacing - 16.5f + rng_unit(&rng) * 0.04f - 0.02f;
            float top = 0.0f;
            for (uint32_t level = 0; level < PILE_HEIGHT; ++level) {
                Vec3 half = v3_random(&rng, 0.4f, 0.5f);
                Quat yaw = quat_normalize((Quat){ 0.0f, rng_unit(&rng) * 0.2f - 0.1f, 0.0f, 1.0f });
                create_body(w, (Vec3){ x, top + half.y + 0.005f, z }, half, yaw);
                top = top + 2.0f * half.y + 0.005f;
            }
        }
    }

    for (uint32_t pile = 0; pile < PILES_SIDE * PILES_SIDE; pile += 4) {
        float x = (float)(pile % PILES_SIDE) * pile_spacing - 16.5f;
        float z = (float)(pile / PILES_SIDE) * pile_spacing - 16.5f;
        Vec3 half = v3_random(&rng, 0.3f, 0.5f);
        Vec3 position;
        position.x = x + rng_unit(&rng) * 0.4f - 0.2f;
        position.y = 9.5f + rng_unit(&rng);
        position.z = z + rng_unit(&rng) * 0.4f - 0.2f;
        Quat rotation = quat_random(&rng);
        create_body(w, position, half, rotation);
    }

    w->ground = rigid_body_new();
    w->ground.transform.origin = (Vec3){ 0.0f, -1.0f, 0.0f };
    w->ground.half_extents = (Vec3){ 24.0f, 1.0f, 24.0f };
    w->static_motions[0] = (KinematicMotion){ .center = w->ground.transform.origin };
    Aabb tight = tight_bounds(w, &w->ground);
    w->ground_proxy = broad_phase_create_proxy(&w->broad_phase, (ShapeRef){ 0, true }, &tight, false);
}

void world_shove(World* w) {
    for (uint32_t pile = 0; pile < PILES_SIDE * PILES_SIDE; pile += 8) {
        uint32_t slot = pile * PILE_HEIGHT + PILE_HEIGHT - 1;
        RigidBody* body = &w->rigid.data[slot];
        body->linear_velocity = (Vec3){ 2.0f, 0.0f, 1.0f };
        body->sleeping = false;
        body->sleep_time = 0.0f;
        notify(w, slot);
    }
}

void world_step(World* w) {
    process_body_changes(w);
    refresh_proxy_bounds(w);
    update_pairs(w);
    collide(w);
    split_pending_island(w);
    solve(w);
    finalize_sleep(w);
    for (size_t i = 0; i < w->active_bodies.len; ++i) {
        w->active_bodies.data[i].body->applied_force = (Vec3){ 0.0f, 0.0f, 0.0f };
        w->active_bodies.data[i].body->applied_torque = (Vec3){ 0.0f, 0.0f, 0.0f };
    }
}

uint64_t world_checksum(const World* w) {
    uint64_t h = 0;
    for (size_t i = 0; i < w->rigid.len; ++i) {
        const RigidBody* body = &w->rigid.data[i];
        Vec3 p = body->transform.origin;
        h = hash_add(h, (uint64_t)f32_bits(p.x) | ((uint64_t)f32_bits(p.y) << 32));
        h = hash_add(h, (uint64_t)f32_bits(p.z) | ((uint64_t)f32_bits(body->rotation.w) << 32));
        h = hash_add(h, (uint64_t)f32_bits(body->linear_velocity.y) | ((uint64_t)body->sleeping << 32));
    }
    h = hash_add(h, w->live_islands);
    h = hash_add(h, w->sleeping_body_count);
    h = hash_add(h, w->sleeping_touching);
    h = hash_add(h, w->sleeping_points);
    h = hash_add(h, u64map_size(&w->contact_index));
    h = hash_add(h, w->graph.size);
    return h;
}

size_t world_awake_count(const World* w) { return w->awake_bodies.len; }

/// Harness state of a world case.
typedef struct {
    World* world;
} WorldCase;

/// Creates a single-threaded world.
static void setup_one_thread(void* state) {
    WorldCase* s = state;
    s->world = world_create(1);
}

/// Creates a world stepped on four threads.
static void setup_four_threads(void* state) {
    WorldCase* s = state;
    s->world = world_create(4);
}

/// Builds the piles, steps them with a shove partway, and hashes the awake
/// counts and the final state.
static uint64_t run(void* state) {
    WorldCase* s = state;
    world_reset(s->world);
    uint64_t h = 0;
    for (int step = 0; step < STEPS; ++step) {
        if (step == SHOVE_STEP) world_shove(s->world);
        world_step(s->world);
        h = h * 31u + world_awake_count(s->world);
    }
    return h ^ world_checksum(s->world);
}

/// Destroys the world.
static void teardown(void* state) {
    WorldCase* s = state;
    world_destroy(s->world);
}

const Case world_case = { "world", sizeof(WorldCase), setup_one_thread, run, teardown };
const Case world4_case = { "world4", sizeof(WorldCase), setup_four_threads, run, teardown };
