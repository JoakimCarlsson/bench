#include "wide.h"

#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "array.h"
#include "constraint_graph.h"
#include "contact.h"
#include "contact_solver.h"
#include "hash.h"
#include "rigid_body.h"
#include "vecmath.h"

enum { GRID = 32, HEIGHT = 8, BODIES = GRID * GRID * HEIGHT, STEPS = 4 };

typedef ARRAY_OF(Contact) ContactArray;

typedef struct {
    SolverContext context;
    RigidBody* initial_bodies;
    ContactArray initial_contacts;
    RigidBody* bodies;
    Contact* contacts;
    ConstraintGraph graph;
    U32Span colors[GRAPH_COLOR_COUNT];
    uint32_t* body_local;
    ActiveBody* active;
    KinematicMotion static_motions[1];
    BodyDelta* deltas;
    ContactSolver* solver;
} Wide;

/// The engine's step context at 60 Hz with 4 substeps.
static SolverContext make_context(void) {
    SolverContext c = { 0 };
    c.dt = 1.0f / 60.0f;
    c.inv_dt = 1.0f / c.dt;
    c.substeps = 4;
    c.h = c.dt / (float)c.substeps;
    c.inv_h = 1.0f / c.h;
    c.gravity = (Vec3){ 0.0f, -9.81f, 0.0f };
    float contact_hertz = f32_min(30.0f, 0.125f * c.inv_h);
    c.contact_softness = make_softness(contact_hertz, 10.0f, c.h);
    c.static_softness = make_softness(2.0f * contact_hertz, 0.5f * 10.0f, c.h);
    c.push_out_speed = 3.0f;
    c.restitution_threshold = 1.0f;
    c.linear_slop = 0.005f;
    return c;
}

/// Appends a contact from body `a` to shape `b` with the scene's material.
static Contact* add_contact(Wide* s, uint32_t a, ShapeRef b, Vec3 normal) {
    uint32_t index = (uint32_t)s->initial_contacts.len;
    ARRAY_PUSH(s->initial_contacts, contact_new());
    Contact* contact = &ARRAY_BACK(s->initial_contacts);
    contact->shape_a = (ShapeRef){ a, false };
    contact->shape_b = b;
    contact->alive = true;
    contact->manifold.normal = normal;
    contact->friction = 0.6f;
    contact->restitution = index % 8 == 0 ? 0.3f : 0.0f;
    contact->rolling_resistance = index % 5 == 0 ? 0.05f : 0.0f;
    return contact;
}

/// Appends a manifold point with a random separation.
static void add_point(Contact* contact, Rng* rng, Vec3 point) {
    ManifoldPoint* p = &contact->manifold.points[contact->manifold.point_count++];
    p->point = point;
    p->separation = rng_unit(rng) * 0.025f - 0.02f;
}

/// Builds the stacks.
static void build_bodies(Wide* s, Rng* rng) {
    for (uint32_t cz = 0; cz < GRID; cz++) {
        for (uint32_t cx = 0; cx < GRID; cx++) {
            for (uint32_t level = 0; level < HEIGHT; level++) {
                uint32_t i = (cz * GRID + cx) * HEIGHT + level;
                RigidBody* b = &s->initial_bodies[i];
                *b = rigid_body_new();
                Vec3 half = v3_random(rng, 0.4f, 0.5f);
                Quat tilt;
                tilt.x = rng_unit(rng) * 0.1f - 0.05f;
                tilt.y = rng_unit(rng) * 0.2f - 0.1f;
                tilt.z = rng_unit(rng) * 0.1f - 0.05f;
                tilt.w = 1.0f;
                b->rotation = quat_normalize(tilt);
                b->transform.basis = basis_from_quat(b->rotation);
                b->transform.origin = (Vec3){ (float)cx * 1.1f, (float)level * 1.0f + 0.5f, (float)cz * 1.1f };
                set_box_mass(b, half);
                b->linear_velocity = v3_random(rng, -0.1f, 0.1f);
                if (i % 16 == 0) b->linear_velocity.y = -2.0f;
                b->angular_velocity = v3_random(rng, -0.1f, 0.1f);
                b->angular_damp = 0.05f;
            }
        }
    }
}

/// Builds the contacts down each stack, to the ground, and between some
/// neighbours.
static void build_contacts(Wide* s, Rng* rng) {
    static const float corners[4][2] = { { 1.0f, 1.0f }, { -1.0f, 1.0f }, { -1.0f, -1.0f }, { 1.0f, -1.0f } };
    const Vec3 half = { 0.45f, 0.45f, 0.45f };
    for (uint32_t cz = 0; cz < GRID; cz++) {
        for (uint32_t cx = 0; cx < GRID; cx++) {
            for (uint32_t level = 0; level < HEIGHT; level++) {
                uint32_t i = (cz * GRID + cx) * HEIGHT + level;
                Vec3 center = s->initial_bodies[i].transform.origin;
                Contact* down = level == 0 ? add_contact(s, i, (ShapeRef){ 0, true }, (Vec3){ 0.0f, -1.0f, 0.0f })
                                           : add_contact(s, i, (ShapeRef){ i - 1, false }, (Vec3){ 0.0f, -1.0f, 0.0f });
                for (int k = 0; k < 4; k++) {
                    add_point(down, rng, v3_add(center, (Vec3){ corners[k][0] * half.x, -half.y, corners[k][1] * half.z }));
                }
                if (cx + 1 < GRID && rng_next(rng) % 4 == 0) {
                    Contact* side = add_contact(s, i, (ShapeRef){ i + HEIGHT, false }, (Vec3){ 1.0f, 0.0f, 0.0f });
                    add_point(side, rng, v3_add(center, (Vec3){ half.x, 0.5f * half.y, 0.0f }));
                    add_point(side, rng, v3_add(center, (Vec3){ half.x, -0.5f * half.y, 0.0f }));
                }
                if (cz + 1 < GRID && rng_next(rng) % 4 == 0) {
                    Contact* side = add_contact(s, i, (ShapeRef){ i + GRID * HEIGHT, false }, (Vec3){ 0.0f, 0.0f, 1.0f });
                    add_point(side, rng, v3_add(center, (Vec3){ 0.0f, 0.5f * half.y, half.z }));
                    add_point(side, rng, v3_add(center, (Vec3){ 0.0f, -0.5f * half.y, half.z }));
                }
            }
        }
    }
}

/// Colours every contact with the engine's constraint graph.
static void color_contacts(Wide* s) {
    constraint_graph_reserve_bodies(&s->graph, BODIES);
    for (uint32_t index = 0; index < s->initial_contacts.len; ++index) {
        Contact* contact = &s->initial_contacts.data[index];
        bool b_static = contact->shape_b.is_static;
        GraphBodies bodies = { contact->shape_a.body, b_static ? 0u : contact->shape_b.body, b_static };
        GraphSlot slot = constraint_graph_add(&s->graph, index, bodies);
        contact->color = slot.color;
        contact->local = slot.local;
    }
    graph_color_spans(&s->graph, s->colors);
}

/// Builds the scene, colours it and lays out the solver inputs.
static void setup(void* state) {
    Wide* s = state;
    s->context = make_context();
    s->initial_bodies = xalloc(BODIES * sizeof(RigidBody));
    Rng rng = { 0x501e };
    build_bodies(s, &rng);
    build_contacts(s, &rng);
    color_contacts(s);

    s->bodies = xalloc(BODIES * sizeof(RigidBody));
    s->contacts = xalloc(s->initial_contacts.len * sizeof(Contact));
    s->body_local = xalloc(BODIES * sizeof(uint32_t));
    s->active = xalloc(BODIES * sizeof(ActiveBody));
    for (uint32_t i = 0; i < BODIES; ++i) {
        s->body_local[i] = i;
        s->active[i] = (ActiveBody){ &s->bodies[i], i };
    }
    s->deltas = xalloc(BODIES * sizeof(BodyDelta));
    for (uint32_t i = 0; i < BODIES; ++i) s->deltas[i].rotation = quat_identity();
    s->solver = contact_solver_create();
}

/// Runs the solver for every step and hashes bodies and impulses.
static uint64_t run(void* state) {
    Wide* s = state;
    memcpy(s->bodies, s->initial_bodies, BODIES * sizeof(RigidBody));
    memcpy(s->contacts, s->initial_contacts.data, s->initial_contacts.len * sizeof(Contact));
    SolverInputs inputs = {
        .contacts = s->contacts,
        .colors = s->colors,
        .body_local = s->body_local,
        .active_bodies = s->active,
        .active_count = BODIES,
        .static_motions = s->static_motions,
        .static_count = 1,
        .deltas = s->deltas,
        .context = s->context,
        .pool = NULL,
    };
    for (int step = 0; step < STEPS; step++) contact_solver_solve(s->solver, &inputs);
    uint64_t h = 0;
    for (uint32_t i = 0; i < BODIES; i++) {
        const RigidBody* b = &s->bodies[i];
        h = hash_add(h, (uint64_t)f32_bits(b->linear_velocity.x) | ((uint64_t)f32_bits(b->linear_velocity.y) << 32));
        h = hash_add(h, (uint64_t)f32_bits(b->transform.origin.y) | ((uint64_t)f32_bits(b->rotation.w) << 32));
        h = hash_add(h, (uint64_t)f32_bits(b->angular_velocity.z) | ((uint64_t)f32_bits(s->deltas[i].rotation.x) << 32));
    }
    for (size_t i = 0; i < s->initial_contacts.len; i++) {
        const Manifold* m = &s->contacts[i].manifold;
        for (uint32_t k = 0; k < m->point_count; k++) {
            const ManifoldPoint* p = &m->points[k];
            h = hash_add(h, (uint64_t)f32_bits(p->normal_impulse) | ((uint64_t)f32_bits(p->total_normal_impulse) << 32));
            h = hash_add(h, (uint64_t)f32_bits(p->peak_normal_impulse) | ((uint64_t)f32_bits(p->relative_velocity) << 32));
        }
        const FrictionImpulses* f = &s->contacts[i].friction_impulses;
        h = hash_add(h, (uint64_t)f32_bits(f->tangent_x) | ((uint64_t)f32_bits(f->tangent_y) << 32));
        h = hash_add(h, (uint64_t)f32_bits(f->twist) | ((uint64_t)f32_bits(f->rolling.y) << 32));
    }
    return h;
}

/// Frees the scene and the solver.
static void teardown(void* state) {
    Wide* s = state;
    free(s->initial_bodies);
    ARRAY_FREE(s->initial_contacts);
    free(s->bodies);
    free(s->contacts);
    constraint_graph_free(&s->graph);
    free(s->body_local);
    free(s->active);
    free(s->deltas);
    contact_solver_destroy(s->solver);
}

const Case wide_case = { "wide", sizeof(Wide), setup, run, teardown };
