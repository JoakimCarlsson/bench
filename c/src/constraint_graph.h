#ifndef BENCH_CONSTRAINT_GRAPH_H
#define BENCH_CONSTRAINT_GRAPH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "array.h"

/// The engine's constraint graph: colours contacts so that no two in a colour
/// share a dynamic body, keeping static contacts out of colour 0 and sending
/// what does not fit to the overflow colour.

enum {
    GRAPH_COLOR_COUNT = 24,
    OVERFLOW_COLOR = GRAPH_COLOR_COUNT - 1,
    DYNAMIC_COLOR_COUNT = GRAPH_COLOR_COUNT - 4,
};

typedef struct {
    uint32_t a;
    uint32_t b;
    bool b_is_static;
} GraphBodies;

typedef struct {
    uint32_t color;
    uint32_t local;
} GraphSlot;

typedef struct {
    U64Array bodies;
    U32Array contacts;
} GraphColor;

/// Graph storage; a zeroed struct is an empty graph.
typedef struct {
    GraphColor colors[GRAPH_COLOR_COUNT];
    size_t words;
    size_t size;
} ConstraintGraph;

/// Releases the graph's storage and leaves it empty.
void constraint_graph_free(ConstraintGraph* graph);
/// Grows every colour's body bitset to cover `body_count` bodies.
void constraint_graph_reserve_bodies(ConstraintGraph* graph, size_t body_count);
/// Adds a contact to the first colour free for its bodies.
GraphSlot constraint_graph_add(ConstraintGraph* graph, uint32_t contact, GraphBodies bodies);
/// Removes a contact; returns the contact moved into its place, or NULL_LINK.
uint32_t constraint_graph_remove(ConstraintGraph* graph, GraphSlot slot, GraphBodies bodies);

#endif
