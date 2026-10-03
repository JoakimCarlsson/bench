#include "aabb_tree.h"

/// A detached node with no children.
static AabbTreeNode empty_node(void) {
    return (AabbTreeNode){ .parent = AABB_TREE_NULL_NODE, .child1 = AABB_TREE_NULL_NODE, .child2 = AABB_TREE_NULL_NODE };
}

/// Node by id.
static AabbTreeNode* node(const AabbTree* tree, int32_t id) { return &tree->nodes.data[id]; }

/// Whether the node has no children.
static bool is_leaf(const AabbTreeNode* n) { return n->child1 == AABB_TREE_NULL_NODE; }

/// Larger of two heights.
static int32_t max_height(int32_t a, int32_t b) { return a < b ? b : a; }

/// A node from the free list, or a new one.
static int32_t allocate_node(AabbTree* tree) {
    if (tree->free_list.len == 0) {
        ARRAY_PUSH(tree->nodes, empty_node());
        return (int32_t)(tree->nodes.len - 1);
    }
    int32_t id = ARRAY_POP(tree->free_list);
    *node(tree, id) = empty_node();
    return id;
}

/// Returns a node to the free list.
static void free_node(AabbTree* tree, int32_t id) {
    *node(tree, id) = empty_node();
    node(tree, id)->height = -1;
    ARRAY_PUSH(tree->free_list, id);
}

/// Points the parent of `old_child` at `new_child`, or makes it the root.
static void replace_child(AabbTree* tree, int32_t parent, int32_t old_child, int32_t new_child) {
    if (parent != AABB_TREE_NULL_NODE) {
        if (node(tree, parent)->child1 == old_child) {
            node(tree, parent)->child1 = new_child;
        } else {
            node(tree, parent)->child2 = new_child;
        }
    } else {
        tree->root = new_child;
    }
}

/// Rotates a node whose children differ in height by more than one;
/// returns the node now in its place.
static int32_t balance(AabbTree* tree, int32_t a_id) {
    AabbTreeNode* a = node(tree, a_id);
    if (is_leaf(a) || a->height < 2) return a_id;
    int32_t b_id = a->child1;
    int32_t c_id = a->child2;
    AabbTreeNode* b = node(tree, b_id);
    AabbTreeNode* c = node(tree, c_id);
    int32_t imbalance = c->height - b->height;

    if (imbalance > 1) {
        int32_t f_id = c->child1;
        int32_t g_id = c->child2;
        AabbTreeNode* f = node(tree, f_id);
        AabbTreeNode* g = node(tree, g_id);

        c->child1 = a_id;
        c->parent = a->parent;
        a->parent = c_id;
        replace_child(tree, c->parent, a_id, c_id);
        if (f->height > g->height) {
            c->child2 = f_id;
            a->child2 = g_id;
            g->parent = a_id;
            a->aabb = aabb_merge(&b->aabb, &g->aabb);
            c->aabb = aabb_merge(&a->aabb, &f->aabb);
            a->height = 1 + max_height(b->height, g->height);
            c->height = 1 + max_height(a->height, f->height);
        } else {
            c->child2 = g_id;
            a->child2 = f_id;
            f->parent = a_id;
            a->aabb = aabb_merge(&b->aabb, &f->aabb);
            c->aabb = aabb_merge(&a->aabb, &g->aabb);
            a->height = 1 + max_height(b->height, f->height);
            c->height = 1 + max_height(a->height, g->height);
        }
        return c_id;
    }

    if (imbalance < -1) {
        int32_t d_id = b->child1;
        int32_t e_id = b->child2;
        AabbTreeNode* d = node(tree, d_id);
        AabbTreeNode* e = node(tree, e_id);

        b->child1 = a_id;
        b->parent = a->parent;
        a->parent = b_id;
        replace_child(tree, b->parent, a_id, b_id);
        if (d->height > e->height) {
            b->child2 = d_id;
            a->child1 = e_id;
            e->parent = a_id;
            a->aabb = aabb_merge(&c->aabb, &e->aabb);
            b->aabb = aabb_merge(&a->aabb, &d->aabb);
            a->height = 1 + max_height(c->height, e->height);
            b->height = 1 + max_height(a->height, d->height);
        } else {
            b->child2 = e_id;
            a->child1 = d_id;
            d->parent = a_id;
            a->aabb = aabb_merge(&c->aabb, &d->aabb);
            b->aabb = aabb_merge(&a->aabb, &e->aabb);
            a->height = 1 + max_height(c->height, d->height);
            b->height = 1 + max_height(a->height, e->height);
        }
        return b_id;
    }
    return a_id;
}

/// Cost of descending into `child` when inserting `leaf_aabb`.
static float child_cost(const AabbTree* tree, int32_t child, const Aabb* leaf_aabb, float inheritance_cost) {
    const AabbTreeNode* c = node(tree, child);
    Aabb merged_box = aabb_merge(leaf_aabb, &c->aabb);
    float merged = aabb_surface_area(&merged_box);
    if (is_leaf(c)) return merged + inheritance_cost;
    return merged - aabb_surface_area(&c->aabb) + inheritance_cost;
}

/// Descends by the surface area heuristic, adds a parent over the chosen
/// sibling, and refits and balances up to the root.
static void insert_leaf(AabbTree* tree, int32_t leaf) {
    if (tree->root == AABB_TREE_NULL_NODE) {
        tree->root = leaf;
        node(tree, tree->root)->parent = AABB_TREE_NULL_NODE;
        return;
    }

    Aabb leaf_aabb = node(tree, leaf)->aabb;
    int32_t index = tree->root;
    while (!is_leaf(node(tree, index))) {
        int32_t child1 = node(tree, index)->child1;
        int32_t child2 = node(tree, index)->child2;

        float area = aabb_surface_area(&node(tree, index)->aabb);
        Aabb combined = aabb_merge(&node(tree, index)->aabb, &leaf_aabb);
        float combined_area = aabb_surface_area(&combined);
        float cost = 2.0f * combined_area;
        float inheritance_cost = 2.0f * (combined_area - area);

        float cost1 = child_cost(tree, child1, &leaf_aabb, inheritance_cost);
        float cost2 = child_cost(tree, child2, &leaf_aabb, inheritance_cost);

        if (cost < cost1 && cost < cost2) break;
        index = cost1 < cost2 ? child1 : child2;
    }

    int32_t sibling = index;
    int32_t old_parent = node(tree, sibling)->parent;
    int32_t new_parent = allocate_node(tree);
    node(tree, new_parent)->parent = old_parent;
    node(tree, new_parent)->aabb = aabb_merge(&leaf_aabb, &node(tree, sibling)->aabb);
    node(tree, new_parent)->height = node(tree, sibling)->height + 1;

    replace_child(tree, old_parent, sibling, new_parent);
    node(tree, new_parent)->child1 = sibling;
    node(tree, new_parent)->child2 = leaf;
    node(tree, sibling)->parent = new_parent;
    node(tree, leaf)->parent = new_parent;

    index = node(tree, leaf)->parent;
    while (index != AABB_TREE_NULL_NODE) {
        index = balance(tree, index);
        int32_t child1 = node(tree, index)->child1;
        int32_t child2 = node(tree, index)->child2;
        node(tree, index)->height = 1 + max_height(node(tree, child1)->height, node(tree, child2)->height);
        node(tree, index)->aabb = aabb_merge(&node(tree, child1)->aabb, &node(tree, child2)->aabb);
        index = node(tree, index)->parent;
    }
}

/// Replaces the leaf's parent with its sibling and refits up to the root.
static void remove_leaf(AabbTree* tree, int32_t leaf) {
    if (leaf == tree->root) {
        tree->root = AABB_TREE_NULL_NODE;
        return;
    }
    int32_t parent = node(tree, leaf)->parent;
    int32_t grand_parent = node(tree, parent)->parent;
    int32_t sibling = node(tree, parent)->child1 == leaf ? node(tree, parent)->child2 : node(tree, parent)->child1;

    if (grand_parent != AABB_TREE_NULL_NODE) {
        if (node(tree, grand_parent)->child1 == parent) {
            node(tree, grand_parent)->child1 = sibling;
        } else {
            node(tree, grand_parent)->child2 = sibling;
        }
        node(tree, sibling)->parent = grand_parent;
        free_node(tree, parent);

        int32_t index = grand_parent;
        while (index != AABB_TREE_NULL_NODE) {
            index = balance(tree, index);
            int32_t child1 = node(tree, index)->child1;
            int32_t child2 = node(tree, index)->child2;
            node(tree, index)->aabb = aabb_merge(&node(tree, child1)->aabb, &node(tree, child2)->aabb);
            node(tree, index)->height = 1 + max_height(node(tree, child1)->height, node(tree, child2)->height);
            index = node(tree, index)->parent;
        }
    } else {
        tree->root = sibling;
        node(tree, sibling)->parent = AABB_TREE_NULL_NODE;
        free_node(tree, parent);
    }
}

void aabb_tree_init(AabbTree* tree) {
    *tree = (AabbTree){ 0 };
    tree->root = AABB_TREE_NULL_NODE;
}

void aabb_tree_free(AabbTree* tree) {
    ARRAY_FREE(tree->nodes);
    ARRAY_FREE(tree->free_list);
    aabb_tree_init(tree);
}

int32_t aabb_tree_create_proxy(AabbTree* tree, const Aabb* aabb, uint32_t user_data) {
    int32_t id = allocate_node(tree);
    AabbTreeNode* leaf = node(tree, id);
    leaf->aabb = *aabb;
    leaf->user_data = user_data;
    leaf->height = 0;
    insert_leaf(tree, id);
    ++tree->proxy_count;
    return id;
}

void aabb_tree_destroy_proxy(AabbTree* tree, int32_t proxy) {
    remove_leaf(tree, proxy);
    free_node(tree, proxy);
    --tree->proxy_count;
}

void aabb_tree_move_proxy(AabbTree* tree, int32_t proxy, const Aabb* aabb) {
    remove_leaf(tree, proxy);
    node(tree, proxy)->aabb = *aabb;
    insert_leaf(tree, proxy);
}

void aabb_tree_query(const AabbTree* tree, const Aabb* aabb, AabbTreeVisit visit, void* context) {
    I32Array stack = { 0 };
    ARRAY_RESERVE(stack, 64);
    ARRAY_PUSH(stack, tree->root);
    while (stack.len != 0) {
        int32_t id = ARRAY_POP(stack);
        if (id == AABB_TREE_NULL_NODE) continue;
        const AabbTreeNode* n = node(tree, id);
        if (!aabb_overlaps(&n->aabb, aabb)) continue;
        if (is_leaf(n)) {
            if (!visit(context, id)) break;
        } else {
            ARRAY_PUSH(stack, n->child1);
            ARRAY_PUSH(stack, n->child2);
        }
    }
    ARRAY_FREE(stack);
}
