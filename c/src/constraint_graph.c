#include "constraint_graph.h"

#include "contact.h"

/// Whether `body` is not yet in `color`.
static bool is_free(const GraphColor* color, uint32_t body) {
    return (color->bodies.data[body / 64u] & ((uint64_t)1 << (body % 64u))) == 0u;
}

/// Marks `body` as in `color`.
static void take(GraphColor* color, uint32_t body) { color->bodies.data[body / 64u] |= (uint64_t)1 << (body % 64u); }

/// Clears `body` from `color`.
static void release(GraphColor* color, uint32_t body) { color->bodies.data[body / 64u] &= ~((uint64_t)1 << (body % 64u)); }

void constraint_graph_free(ConstraintGraph* graph) {
    for (uint32_t color = 0; color < GRAPH_COLOR_COUNT; ++color) {
        ARRAY_FREE(graph->colors[color].bodies);
        ARRAY_FREE(graph->colors[color].contacts);
    }
    graph->words = 0;
    graph->size = 0;
}

void constraint_graph_reserve_bodies(ConstraintGraph* graph, size_t body_count) {
    size_t words = (body_count + 63u) / 64u;
    if (words <= graph->words) return;
    graph->words = words;
    for (uint32_t color = 0; color < OVERFLOW_COLOR; ++color) ARRAY_RESIZE(graph->colors[color].bodies, words);
}

GraphSlot constraint_graph_add(ConstraintGraph* graph, uint32_t contact, GraphBodies bodies) {
    uint32_t chosen = OVERFLOW_COLOR;
    if (bodies.b_is_static) {
        for (uint32_t color = OVERFLOW_COLOR - 1; color >= 1; --color) {
            if (is_free(&graph->colors[color], bodies.a)) {
                take(&graph->colors[color], bodies.a);
                chosen = color;
                break;
            }
        }
    } else {
        for (uint32_t color = 0; color < DYNAMIC_COLOR_COUNT; ++color) {
            if (is_free(&graph->colors[color], bodies.a) && is_free(&graph->colors[color], bodies.b)) {
                take(&graph->colors[color], bodies.a);
                take(&graph->colors[color], bodies.b);
                chosen = color;
                break;
            }
        }
    }
    GraphColor* target = &graph->colors[chosen];
    uint32_t local = (uint32_t)target->contacts.len;
    ARRAY_PUSH(target->contacts, contact);
    ++graph->size;
    return (GraphSlot){ chosen, local };
}

uint32_t constraint_graph_remove(ConstraintGraph* graph, GraphSlot slot, GraphBodies bodies) {
    GraphColor* target = &graph->colors[slot.color];
    if (slot.color != OVERFLOW_COLOR) {
        release(target, bodies.a);
        if (!bodies.b_is_static) release(target, bodies.b);
    }
    uint32_t last = ARRAY_BACK(target->contacts);
    uint32_t moved = NULL_LINK;
    if (slot.local + 1u != target->contacts.len) {
        target->contacts.data[slot.local] = last;
        moved = last;
    }
    target->contacts.len--;
    --graph->size;
    return moved;
}
