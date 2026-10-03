#include "unproject.h"

#include <stdlib.h>

#include "alloc.h"
#include "hash.h"
#include "vecmath.h"

enum { CAMERAS = 131072, POINTS = 4 };

typedef struct { Vec3 eye, target; float tan_half_fov, aspect, z_near, z_far; } Camera;
typedef struct { Camera* cameras; } Unproject;

static const Vec3 ndc[POINTS] = {
    { -0.5f, -0.5f, -1.0f },
    { 0.5f, -0.5f, 0.0f },
    { 0.5f, 0.5f, 0.5f },
    { -0.25f, 0.75f, 0.999f },
};

static void setup(void* state) {
    Unproject* s = state;
    s->cameras = xalloc(CAMERAS * sizeof(Camera));
    Rng rng = { 0xca3e };
    for (int i = 0; i < CAMERAS; i++) {
        Camera* c = &s->cameras[i];
        c->eye = v3_random(&rng, -50.0f, 50.0f);
        c->target = v3_add(c->eye, v3_random(&rng, -10.0f, 10.0f));
        c->tan_half_fov = 0.3f + rng_unit(&rng) * 1.0f;
        c->aspect = 1.0f + rng_unit(&rng) * 1.0f;
        c->z_near = 0.05f + rng_unit(&rng) * 0.45f;
        c->z_far = 100.0f + rng_unit(&rng) * 3900.0f;
    }
}

/// View-projection matrix of a camera.
static Mat4 view_projection(const Camera* c) {
    Transform eye = looking_at(c->eye, c->target, (Vec3){ 0.0f, 1.0f, 0.0f });
    Transform view = inverse_orthonormal(&eye);
    Mat4 view_matrix = to_mat4(&view);
    Mat4 projection = perspective(c->tan_half_fov, c->aspect, c->z_near, c->z_far);
    return mat4_mul(&projection, &view_matrix);
}

static uint64_t run(void* state) {
    Unproject* s = state;
    uint64_t h = 0;
    for (int i = 0; i < CAMERAS; i++) {
        Mat4 vp = view_projection(&s->cameras[i]);
        Mat4 inv = mat4_inverse(&vp);
        for (int k = 0; k < POINTS; k++) {
            Vec3 world = mat4_transform_point(&inv, ndc[k]);
            Vec3 back = mat4_transform_point(&vp, world);
            h = hash_add(h, (uint64_t)f32_bits(world.x) | ((uint64_t)f32_bits(world.y) << 32));
            h = hash_add(h, (uint64_t)f32_bits(world.z) | ((uint64_t)f32_bits(back.x) << 32));
        }
    }
    return h;
}

static void teardown(void* state) {
    Unproject* s = state;
    free(s->cameras);
}

const Case unproject_case = { "unproject", sizeof(Unproject), setup, run, teardown };
