#include "transform.h"

#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "hash.h"
#include "vecmath.h"

enum { NODES = 65536, FRAMES = 8 };

typedef struct { Vec3 position; Vec3 scale; Vec3 spin; uint32_t parent; } Node;
typedef struct {
    Node* nodes;
    Quat* initial;
    Quat* rotations;
    Transform* world;
    Mat4* mvp;
    Mat4 view_projection;
} TransformState;

static void setup(void* state) {
    TransformState* s = state;
    s->nodes = xalloc(NODES * sizeof(Node));
    s->initial = xalloc(NODES * sizeof(Quat));
    s->rotations = xalloc(NODES * sizeof(Quat));
    s->world = xalloc(NODES * sizeof(Transform));
    s->mvp = xalloc(NODES * sizeof(Mat4));
    Rng rng = { 0x7f0e };
    for (int i = 0; i < NODES; i++) {
        Node* n = &s->nodes[i];
        n->position = v3_random(&rng, -2.0f, 2.0f);
        n->scale = v3_random(&rng, 0.9f, 1.1f);
        n->spin = v3_random(&rng, -0.05f, 0.05f);
        n->parent = i == 0 ? 0 : (uint32_t)(i - 1) / 4;
        s->initial[i] = quat_random(&rng);
    }
    Transform eye = looking_at((Vec3){ 40.0f, 30.0f, 40.0f }, (Vec3){ 0.0f, 0.0f, 0.0f }, (Vec3){ 0.0f, 1.0f, 0.0f });
    Transform view = inverse_orthonormal(&eye);
    Mat4 view_matrix = to_mat4(&view);
    Mat4 projection = perspective(0.75f, 16.0f / 9.0f, 0.05f, 4000.0f);
    s->view_projection = mat4_mul(&projection, &view_matrix);
}

/// Local transform of node `i` from its current rotation.
static Transform local_transform(const TransformState* s, int i) {
    const Node* n = &s->nodes[i];
    return (Transform){ basis_from_rotation_scale(s->rotations[i], n->scale), n->position };
}

/// Advance every node one frame; parents come before their children.
static void frame(TransformState* s) {
    for (int i = 0; i < NODES; i++) {
        s->rotations[i] = integrate_rotation(s->rotations[i], s->nodes[i].spin);
        Transform local = local_transform(s, i);
        s->world[i] = i == 0 ? local : transform_mul(&s->world[s->nodes[i].parent], &local);
        Mat4 model = to_mat4(&s->world[i]);
        s->mvp[i] = mat4_mul(&s->view_projection, &model);
    }
}

static uint64_t run(void* state) {
    TransformState* s = state;
    memcpy(s->rotations, s->initial, NODES * sizeof(Quat));
    for (int f = 0; f < FRAMES; f++) frame(s);
    uint64_t h = 0;
    for (int i = 0; i < NODES; i++) {
        const float* m = s->mvp[i].m;
        h = hash_add(h, (uint64_t)f32_bits(m[0]) | ((uint64_t)f32_bits(m[5]) << 32));
        h = hash_add(h, (uint64_t)f32_bits(m[12]) | ((uint64_t)f32_bits(m[13]) << 32));
        h = hash_add(h, (uint64_t)f32_bits(m[14]) | ((uint64_t)f32_bits(m[15]) << 32));
    }
    return h;
}

static void teardown(void* state) {
    TransformState* s = state;
    free(s->nodes);
    free(s->initial);
    free(s->rotations);
    free(s->world);
    free(s->mvp);
}

const Case transform_case = { "transform", sizeof(TransformState), setup, run, teardown };
