#include "raycast.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>

#include "alloc.h"
#include "hash.h"
#include "vecmath.h"

enum { BOXES = 1024, RAYS = 1024 };

typedef struct { Vec3 origin; Vec3 translation; } Ray;
typedef struct { Vec3 point; Vec3 normal; float fraction; bool hit; } RayHit;
typedef struct { BoxPose* boxes; Ray* rays; } Raycast;

static void setup(void* state) {
    Raycast* s = state;
    s->boxes = xalloc(BOXES * sizeof(BoxPose));
    s->rays = xalloc(RAYS * sizeof(Ray));
    Rng rng = { 0x7a1c };
    for (int i = 0; i < BOXES; i++) {
        BoxPose* b = &s->boxes[i];
        b->half_extents = v3_random(&rng, 0.25f, 2.0f);
        b->center = v3_random(&rng, 0.0f, 64.0f);
        b->basis = basis_from_quat(quat_random(&rng));
    }
    for (int i = 0; i < RAYS; i++) {
        Ray* r = &s->rays[i];
        r->origin = v3_random(&rng, -8.0f, 72.0f);
        r->translation = v3_scale(v3_random(&rng, -1.0f, 1.0f), 80.0f);
    }
}

/// Slab test of a ray against an oriented box, in the box's frame. A hit
/// needs an entry fraction in (0, max_fraction].
static RayHit ray_cast_box(const Ray* ray, const BoxPose* box, float max_fraction) {
    RayHit none = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 0.0f, false };
    Basis inverse_basis = basis_transposed(&box->basis);
    Vec3 origin = basis_apply(&inverse_basis, v3_sub(ray->origin, box->center));
    Vec3 translation = basis_apply(&inverse_basis, ray->translation);
    const float origins[3] = { origin.x, origin.y, origin.z };
    const float translations[3] = { translation.x, translation.y, translation.z };
    const float extents[3] = { box->half_extents.x, box->half_extents.y, box->half_extents.z };
    float entry_fraction = 0.0f;
    float exit_fraction = max_fraction;
    int entry_axis = -1;
    float entry_sign = 0.0f;

    for (int axis = 0; axis < 3; axis++) {
        float origin_axis = origins[axis];
        float translation_axis = translations[axis];
        float extent = extents[axis];
        if (fabsf(translation_axis) <= 1e-8f) {
            if (origin_axis < -extent || origin_axis > extent) return none;
            continue;
        }
        float first = (-extent - origin_axis) / translation_axis;
        float last = (extent - origin_axis) / translation_axis;
        float normal_sign = -1.0f;
        if (first > last) {
            float t = first;
            first = last;
            last = t;
            normal_sign = 1.0f;
        }
        if (first > entry_fraction) {
            entry_fraction = first;
            entry_axis = axis;
            entry_sign = normal_sign;
        }
        if (last < exit_fraction) exit_fraction = last;
        if (entry_fraction > exit_fraction) return none;
    }

    if (entry_axis < 0 || entry_fraction <= 0.0f || entry_fraction > max_fraction) return none;

    Vec3 local_normal = { 0.0f, 0.0f, 0.0f };
    if (entry_axis == 0) {
        local_normal.x = entry_sign;
    } else if (entry_axis == 1) {
        local_normal.y = entry_sign;
    } else {
        local_normal.z = entry_sign;
    }
    return (RayHit){ v3_add(ray->origin, v3_scale(ray->translation, entry_fraction)),
                     basis_apply(&box->basis, local_normal), entry_fraction, true };
}

static uint64_t run(void* state) {
    Raycast* s = state;
    uint64_t h = 0;
    for (int i = 0; i < RAYS; i++) {
        RayHit closest = { { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f }, 1.0f, false };
        uint32_t closest_box = 0xffffffffu;
        for (int j = 0; j < BOXES; j++) {
            RayHit hit = ray_cast_box(&s->rays[i], &s->boxes[j], closest.fraction);
            if (hit.hit) {
                closest = hit;
                closest_box = (uint32_t)j;
            }
        }
        h = hash_add(h, (uint64_t)f32_bits(closest.fraction) | ((uint64_t)closest_box << 32));
        h = hash_add(h, (uint64_t)f32_bits(closest.point.x) | ((uint64_t)f32_bits(closest.normal.y) << 32));
    }
    return h;
}

static void teardown(void* state) {
    Raycast* s = state;
    free(s->boxes);
    free(s->rays);
}

const Case raycast_case = { "raycast", sizeof(Raycast), setup, run, teardown };
