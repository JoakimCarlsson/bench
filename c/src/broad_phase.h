#ifndef BENCH_BROAD_PHASE_H
#define BENCH_BROAD_PHASE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "aabb_tree.h"
#include "array.h"
#include "contact.h"

/// The engine's broad phase: static, kinematic and dynamic AABB trees, a
/// move buffer, and pair finding for the proxies that moved.

#define BROAD_PHASE_NULL_PROXY (-1)

typedef struct {
    ShapeRef ref;
    Aabb fat_aabb;
    int32_t node;
    bool is_static;
    bool kinematic;
    bool moved;
    bool alive;
} BroadPhaseProxy;

/// Broad phase storage; initialise with `broad_phase_init`.
typedef struct {
    ARRAY_OF(BroadPhaseProxy) proxies;
    I32Array free;
    I32Array move_buffer;
    AabbTree static_tree;
    AabbTree kinematic_tree;
    AabbTree dynamic_tree;
} BroadPhase;

/// Called with each new pair, proxy ids in reporting order.
typedef void (*BroadPhasePairFn)(void* context, int32_t a, int32_t b);

/// An empty broad phase.
void broad_phase_init(BroadPhase* bp);
/// Releases the broad phase's storage.
void broad_phase_free(BroadPhase* bp);
/// Adds a proxy to the tree for its kind and buffers it as moved.
int32_t broad_phase_create_proxy(BroadPhase* bp, ShapeRef ref, const Aabb* fat_aabb, bool kinematic);
/// Removes a proxy and frees its id.
void broad_phase_destroy_proxy(BroadPhase* bp, int32_t id);
/// Gives a proxy new fat bounds and buffers it as moved.
void broad_phase_move_proxy(BroadPhase* bp, int32_t id, const Aabb* fat_aabb);
/// Calls `on_pair` once for every new overlap of the moved proxies in
/// [begin, end), dynamic before static and lower id first.
void broad_phase_query_moved(const BroadPhase* bp, size_t begin, size_t end, BroadPhasePairFn on_pair, void* context);
/// Reports the pairs of every moved proxy, then clears the move buffer.
void broad_phase_update_pairs(BroadPhase* bp, BroadPhasePairFn on_pair, void* context);
/// Clears the move buffer and the proxies' moved flags.
void broad_phase_clear_moves(BroadPhase* bp);

/// Fat bounds of a proxy.
static inline const Aabb* broad_phase_fat_aabb(const BroadPhase* bp, int32_t id) { return &bp->proxies.data[id].fat_aabb; }
/// Shape a proxy stands for.
static inline ShapeRef broad_phase_shape(const BroadPhase* bp, int32_t id) { return bp->proxies.data[id].ref; }
/// Number of buffered moves.
static inline size_t broad_phase_moved_count(const BroadPhase* bp) { return bp->move_buffer.len; }

#endif
