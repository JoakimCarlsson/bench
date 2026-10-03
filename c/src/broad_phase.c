#include "broad_phase.h"

/// What a tree query of one moved proxy needs to report its pairs.
typedef struct {
    const BroadPhase* bp;
    const AabbTree* found_tree;
    int32_t query_id;
    const BroadPhaseProxy* query;
    BroadPhasePairFn on_pair;
    void* context;
} MovedQuery;

/// A detached, dead proxy.
static BroadPhaseProxy empty_proxy(void) { return (BroadPhaseProxy){ .node = AABB_TREE_NULL_NODE }; }

/// Tree that holds a proxy of this kind.
static AabbTree* tree_of(BroadPhase* bp, const BroadPhaseProxy* proxy) {
    if (!proxy->is_static) return &bp->dynamic_tree;
    return proxy->kinematic ? &bp->kinematic_tree : &bp->static_tree;
}

/// Adds a proxy to the move buffer once.
static void buffer_move(BroadPhase* bp, int32_t id) {
    BroadPhaseProxy* proxy = &bp->proxies.data[id];
    if (proxy->moved) return;
    proxy->moved = true;
    ARRAY_PUSH(bp->move_buffer, id);
}

void broad_phase_init(BroadPhase* bp) {
    *bp = (BroadPhase){ 0 };
    aabb_tree_init(&bp->static_tree);
    aabb_tree_init(&bp->kinematic_tree);
    aabb_tree_init(&bp->dynamic_tree);
}

void broad_phase_free(BroadPhase* bp) {
    ARRAY_FREE(bp->proxies);
    ARRAY_FREE(bp->free);
    ARRAY_FREE(bp->move_buffer);
    aabb_tree_free(&bp->static_tree);
    aabb_tree_free(&bp->kinematic_tree);
    aabb_tree_free(&bp->dynamic_tree);
}

int32_t broad_phase_create_proxy(BroadPhase* bp, ShapeRef ref, const Aabb* fat_aabb, bool kinematic) {
    int32_t id;
    if (bp->free.len == 0) {
        ARRAY_PUSH(bp->proxies, empty_proxy());
        id = (int32_t)(bp->proxies.len - 1);
    } else {
        id = ARRAY_POP(bp->free);
    }
    BroadPhaseProxy* proxy = &bp->proxies.data[id];
    *proxy = empty_proxy();
    proxy->ref = ref;
    proxy->fat_aabb = *fat_aabb;
    proxy->is_static = ref.is_static;
    proxy->kinematic = kinematic && proxy->is_static;
    proxy->alive = true;
    proxy->node = aabb_tree_create_proxy(tree_of(bp, proxy), fat_aabb, (uint32_t)id);
    buffer_move(bp, id);
    return id;
}

void broad_phase_destroy_proxy(BroadPhase* bp, int32_t id) {
    BroadPhaseProxy* proxy = &bp->proxies.data[id];
    aabb_tree_destroy_proxy(tree_of(bp, proxy), proxy->node);
    *proxy = empty_proxy();
    ARRAY_PUSH(bp->free, id);
}

void broad_phase_move_proxy(BroadPhase* bp, int32_t id, const Aabb* fat_aabb) {
    BroadPhaseProxy* proxy = &bp->proxies.data[id];
    proxy->fat_aabb = *fat_aabb;
    aabb_tree_move_proxy(tree_of(bp, proxy), proxy->node, fat_aabb);
    buffer_move(bp, id);
}

/// Reports the pair of the queried proxy with a leaf of the tree being
/// searched, unless it is itself, its own body or a lower moved proxy.
static bool visit_moved(void* context, int32_t node) {
    const MovedQuery* q = context;
    int32_t found_id = (int32_t)aabb_tree_user_data(q->found_tree, node);
    if (found_id == q->query_id) return true;
    const BroadPhaseProxy* found = &q->bp->proxies.data[found_id];
    if (!q->query->is_static && !found->is_static && q->query->ref.body == found->ref.body) return true;
    if (found->moved && found_id < q->query_id) return true;
    bool query_first = !q->query->is_static && (found->is_static || q->query_id < found_id);
    if (query_first) {
        q->on_pair(q->context, q->query_id, found_id);
    } else {
        q->on_pair(q->context, found_id, q->query_id);
    }
    return true;
}

void broad_phase_query_moved(const BroadPhase* bp, size_t begin, size_t end, BroadPhasePairFn on_pair, void* context) {
    for (size_t slot = begin; slot < end; ++slot) {
        int32_t query_id = bp->move_buffer.data[slot];
        const BroadPhaseProxy* query = &bp->proxies.data[query_id];
        if (!query->alive) continue;
        MovedQuery q = { bp, &bp->dynamic_tree, query_id, query, on_pair, context };
        aabb_tree_query(&bp->dynamic_tree, &query->fat_aabb, visit_moved, &q);
        if (!query->is_static) {
            q.found_tree = &bp->static_tree;
            aabb_tree_query(&bp->static_tree, &query->fat_aabb, visit_moved, &q);
            q.found_tree = &bp->kinematic_tree;
            aabb_tree_query(&bp->kinematic_tree, &query->fat_aabb, visit_moved, &q);
        }
    }
}

void broad_phase_update_pairs(BroadPhase* bp, BroadPhasePairFn on_pair, void* context) {
    broad_phase_query_moved(bp, 0, bp->move_buffer.len, on_pair, context);
    broad_phase_clear_moves(bp);
}

void broad_phase_clear_moves(BroadPhase* bp) {
    for (size_t i = 0; i < bp->move_buffer.len; ++i) bp->proxies.data[bp->move_buffer.data[i]].moved = false;
    bp->move_buffer.len = 0;
}
