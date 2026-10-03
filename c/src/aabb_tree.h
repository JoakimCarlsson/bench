#ifndef BENCH_AABB_TREE_H
#define BENCH_AABB_TREE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "array.h"
#include "vecmath.h"

/// The engine's dynamic AABB tree: surface-area-heuristic insertion with
/// AVL-style rotations and AABB queries.

#define AABB_TREE_NULL_NODE (-1)

typedef struct {
    Aabb aabb;
    uint32_t user_data;
    int32_t parent;
    int32_t child1;
    int32_t child2;
    int32_t height;
} AabbTreeNode;

/// Tree storage; initialise with `aabb_tree_init`.
typedef struct {
    ARRAY_OF(AabbTreeNode) nodes;
    I32Array free_list;
    int32_t root;
    size_t proxy_count;
} AabbTree;

/// Called with each leaf a query finds; returns false to stop the query.
typedef bool (*AabbTreeVisit)(void* context, int32_t node);

/// An empty tree.
void aabb_tree_init(AabbTree* tree);
/// Releases the tree's storage.
void aabb_tree_free(AabbTree* tree);
/// Inserts a leaf for `aabb` carrying `user_data`.
int32_t aabb_tree_create_proxy(AabbTree* tree, const Aabb* aabb, uint32_t user_data);
/// Removes a leaf and frees its node.
void aabb_tree_destroy_proxy(AabbTree* tree, int32_t proxy);
/// Reinserts a leaf with new bounds.
void aabb_tree_move_proxy(AabbTree* tree, int32_t proxy, const Aabb* aabb);
/// Calls `visit` with every leaf overlapping `aabb` until it returns false.
void aabb_tree_query(const AabbTree* tree, const Aabb* aabb, AabbTreeVisit visit, void* context);

/// Bounds of a leaf.
static inline const Aabb* aabb_tree_aabb(const AabbTree* tree, int32_t proxy) { return &tree->nodes.data[proxy].aabb; }
/// User data of a leaf.
static inline uint32_t aabb_tree_user_data(const AabbTree* tree, int32_t proxy) { return tree->nodes.data[proxy].user_data; }

#endif
