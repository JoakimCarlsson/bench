#ifndef BENCH_CONTACT_SOLVER_H
#define BENCH_CONTACT_SOLVER_H

#include <stddef.h>
#include <stdint.h>

#include "constraint_graph.h"
#include "contact.h"
#include "rigid_body.h"
#include "task_pool.h"
#include "vecmath.h"

/// The engine's soft step contact solver on eight lanes: bundles each colour
/// of the constraint graph into lanes and runs the stages of every substep,
/// in parallel when given a pool. The overflow colour is not supported.

/// Position and rotation change of a body over one solve.
typedef struct {
    Vec3 position;
    Quat rotation;
} BodyDelta;

/// Step timing, gravity and softness shared by every constraint in a solve.
typedef struct {
    float dt;
    float inv_dt;
    float h;
    float inv_h;
    uint32_t substeps;
    Vec3 gravity;
    Softness contact_softness;
    Softness static_softness;
    float push_out_speed;
    float restitution_threshold;
    float linear_slop;
} SolverContext;

/// Velocity and rotation centre of a static body.
typedef struct {
    Vec3 velocity;
    Vec3 angular_velocity;
    Vec3 center;
} KinematicMotion;

/// A dynamic body taking part in the solve and its slot.
typedef struct {
    RigidBody* body;
    uint32_t slot;
} ActiveBody;

/// A read-only run of contact ids.
typedef struct {
    const uint32_t* data;
    size_t len;
} U32Span;

/// Everything one solve reads and writes.
typedef struct {
    Contact* contacts;
    const U32Span* colors;
    const uint32_t* body_local;
    const ActiveBody* active_bodies;
    size_t active_count;
    const KinematicMotion* static_motions;
    size_t static_count;
    BodyDelta* deltas;
    SolverContext context;
    TaskPool* pool;
} SolverInputs;

typedef struct ContactSolver ContactSolver;

/// A solver with no buffers yet.
ContactSolver* contact_solver_create(void);
/// Frees the solver and its buffers.
void contact_solver_destroy(ContactSolver* solver);
/// Solves every contact for one step and writes velocities, poses, impulses
/// and deltas back. `inputs->colors` holds GRAPH_COLOR_COUNT lists.
void contact_solver_solve(ContactSolver* solver, const SolverInputs* inputs);

/// Each colour of a constraint graph as a span.
static inline void graph_color_spans(const ConstraintGraph* graph, U32Span* spans) {
    for (uint32_t color = 0; color < GRAPH_COLOR_COUNT; ++color) {
        spans[color] = (U32Span){ graph->colors[color].contacts.data, graph->colors[color].contacts.len };
    }
}

#endif
