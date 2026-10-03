#include "boxbox.h"

#include <float.h>
#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "hash.h"
#include "vecmath.h"

enum { PAIRS = 8192, FRAMES = 8, MAX_POINTS = 4, MAX_CLIP = 16 };

static const uint32_t reference_b_flag = 0x80000000u;
static const uint32_t edge_contact_flag = 0x40000000u;
static const float parallel_edge_tolerance = 0.005f;
static const float reduction_bias = 0.95f;
static const float speculative_distance = 0.02f;
static const float linear_slop = 0.005f;

typedef struct { Vec3 point; float separation; float normal_impulse; uint32_t feature_id; bool persisted; } ManifoldPoint;
typedef struct { Vec3 normal; Vec3 separating_axis; ManifoldPoint points[MAX_POINTS]; uint32_t point_count; } Manifold;
typedef struct { Vec3 point; uint32_t id; uint32_t edge; } ClipVertex;
typedef struct { Vec3 point; float separation; uint32_t feature_id; } Candidate;
typedef struct { Vec3 normal; float offset; uint32_t index; } ClipPlane;
typedef struct { const BoxPose* reference; const BoxPose* incident; int axis; bool positive; uint32_t flag; } ReferenceFace;
typedef struct { Vec3 center; Vec3 direction; float half_length; } EdgeSegment;
typedef struct { BoxPose a; BoxPose b; Vec3 velocity; } Pair;
typedef struct { Pair* pairs; Manifold* manifolds; } BoxBox;

static void setup(void* state) {
    BoxBox* s = state;
    s->pairs = xalloc(PAIRS * sizeof(Pair));
    s->manifolds = xalloc(PAIRS * sizeof(Manifold));
    Rng rng = { 0xb0b0 };
    for (int i = 0; i < PAIRS; i++) {
        Pair* p = &s->pairs[i];
        Quat rotation = quat_random(&rng);
        p->a.half_extents = v3_random(&rng, 0.25f, 1.5f);
        p->a.center = v3_random(&rng, 0.0f, 1000.0f);
        p->a.basis = basis_from_quat(rotation);
        p->b.half_extents = v3_random(&rng, 0.25f, 1.5f);
        if (i % 4 == 0) {
            Quat yaw = quat_normalize((Quat){ 0.0f, rng_unit(&rng) - 0.5f, 0.0f, 1.0f });
            p->b.basis = basis_from_quat(quat_mul(rotation, yaw));
            Vec3 slide = v3_random(&rng, -0.3f, 0.3f);
            float lift = p->a.half_extents.y + p->b.half_extents.y - 0.01f;
            p->b.center = v3_add(v3_add(p->a.center, v3_scale(p->a.basis.y, lift)),
                                 v3_add(v3_scale(p->a.basis.x, slide.x), v3_scale(p->a.basis.z, slide.z)));
        } else {
            p->b.basis = basis_from_quat(quat_random(&rng));
            float reach = (v3_length(p->a.half_extents) + v3_length(p->b.half_extents)) * 0.6f;
            p->b.center = v3_add(p->a.center, v3_scale(v3_random(&rng, -1.0f, 1.0f), reach));
        }
        p->velocity = v3_random(&rng, -0.02f, 0.02f);
    }
}

/// Column `index` of a basis.
static Vec3 axis_of(const Basis* basis, int index) {
    switch (index) {
    case 0: return basis->x;
    case 1: return basis->y;
    default: return basis->z;
    }
}

/// Component `index` of the half extents.
static float extent_of(Vec3 half_extents, int index) {
    switch (index) {
    case 0: return half_extents.x;
    case 1: return half_extents.y;
    default: return half_extents.z;
    }
}

/// Half the box's width along `n`.
static float projected_radius(const BoxPose* box, Vec3 n) {
    return fabsf(v3_dot(n, box->basis.x)) * box->half_extents.x + fabsf(v3_dot(n, box->basis.y)) * box->half_extents.y +
           fabsf(v3_dot(n, box->basis.z)) * box->half_extents.z;
}

/// Gap between the boxes along `n`; negative when they overlap.
static float separation_along(const BoxPose* a, const BoxPose* b, Vec3 n) {
    return fabsf(v3_dot(v3_sub(b->center, a->center), n)) - projected_radius(a, n) - projected_radius(b, n);
}

/// Some unit vector perpendicular to the unit vector `n`.
static Vec3 perpendicular(Vec3 n) {
    if (fabsf(n.x) > 0.57735f) return v3_normalize((Vec3){ n.y, -n.x, 0.0f });
    return v3_normalize((Vec3){ 0.0f, n.z, -n.y });
}

/// Feature id of a box corner from which side of each axis it lies on.
static uint32_t vertex_id(int face_axis, bool positive, int u_axis, bool u_positive, int v_axis, bool v_positive) {
    uint32_t id = 0;
    if (positive) id |= 1u << face_axis;
    if (u_positive) id |= 1u << u_axis;
    if (v_positive) id |= 1u << v_axis;
    return id;
}

/// The face of `box` most anti-parallel to `reference_normal`, as four corners.
static size_t incident_face(const BoxPose* box, Vec3 reference_normal, ClipVertex* polygon) {
    int best_axis = 0;
    float best = FLT_MAX;
    bool positive = false;
    for (int axis = 0; axis < 3; axis++) {
        float d = v3_dot(axis_of(&box->basis, axis), reference_normal);
        if (d < best) {
            best = d;
            best_axis = axis;
            positive = true;
        }
        if (-d < best) {
            best = -d;
            best_axis = axis;
            positive = false;
        }
    }
    int u_axis = (best_axis + 1) % 3;
    int v_axis = (best_axis + 2) % 3;
    Vec3 n = v3_scale(axis_of(&box->basis, best_axis), positive ? 1.0f : -1.0f);
    Vec3 u = axis_of(&box->basis, u_axis);
    Vec3 v = axis_of(&box->basis, v_axis);
    float hn = extent_of(box->half_extents, best_axis);
    float hu = extent_of(box->half_extents, u_axis);
    float hv = extent_of(box->half_extents, v_axis);
    Vec3 face_center = v3_add(box->center, v3_scale(n, hn));

    static const float corners[4][2] = { { 1.0f, 1.0f }, { -1.0f, 1.0f }, { -1.0f, -1.0f }, { 1.0f, -1.0f } };
    for (size_t i = 0; i < 4; i++) {
        float su = corners[i][0];
        float sv = corners[i][1];
        polygon[i].point = v3_add(v3_add(face_center, v3_scale(u, su * hu)), v3_scale(v, sv * hv));
        polygon[i].id = vertex_id(best_axis, positive, u_axis, su > 0.0f, v_axis, sv > 0.0f);
        polygon[i].edge = (uint32_t)i;
    }
    return 4;
}

/// Sutherland-Hodgman clip of a polygon against one plane, tracking the
/// feature id of every vertex it creates.
static size_t clip_polygon(const ClipVertex* input, size_t count, ClipPlane plane, ClipVertex* output) {
    uint32_t plane_index = plane.index;
    size_t out = 0;
    for (size_t i = 0; i < count; i++) {
        const ClipVertex* current = &input[i];
        const ClipVertex* next = &input[(i + 1) % count];
        float d_current = v3_dot(current->point, plane.normal) - plane.offset;
        float d_next = v3_dot(next->point, plane.normal) - plane.offset;
        if (d_current <= 0.0f) {
            if (out < MAX_CLIP) output[out++] = *current;
        }
        if ((d_current < 0.0f && d_next > 0.0f) || (d_current > 0.0f && d_next < 0.0f)) {
            float t = d_current / (d_current - d_next);
            ClipVertex vertex;
            vertex.point = v3_add(current->point, v3_scale(v3_sub(next->point, current->point), t));
            bool leaving = d_current < 0.0f;
            if (current->edge < 4) {
                vertex.id = 8u + current->edge * 4u + plane_index;
            } else {
                vertex.id = 24u + (current->edge - 4u) * 4u + plane_index;
            }
            vertex.edge = leaving ? 4u + plane_index : current->edge;
            if (out < MAX_CLIP) output[out++] = vertex;
        }
    }
    return out;
}

/// Clip the incident face to the reference face's side planes and keep the
/// points within the speculative distance of the reference face.
static size_t face_candidates(const ReferenceFace* face, float speculative, Candidate* candidates, Vec3* reference_normal) {
    const BoxPose* reference = face->reference;
    const BoxPose* incident = face->incident;
    int reference_axis = face->axis;
    bool reference_positive = face->positive;
    uint32_t flag = face->flag;
    *reference_normal = v3_scale(axis_of(&reference->basis, reference_axis), reference_positive ? 1.0f : -1.0f);
    int u_axis = (reference_axis + 1) % 3;
    int v_axis = (reference_axis + 2) % 3;
    Vec3 u = axis_of(&reference->basis, u_axis);
    Vec3 v = axis_of(&reference->basis, v_axis);
    float hn = extent_of(reference->half_extents, reference_axis);
    float hu = extent_of(reference->half_extents, u_axis);
    float hv = extent_of(reference->half_extents, v_axis);

    ClipVertex buffer_a[MAX_CLIP];
    ClipVertex buffer_b[MAX_CLIP];
    size_t count = incident_face(incident, *reference_normal, buffer_a);
    float cu = v3_dot(reference->center, u);
    float cv = v3_dot(reference->center, v);
    count = clip_polygon(buffer_a, count, (ClipPlane){ u, cu + hu, 0 }, buffer_b);
    count = clip_polygon(buffer_b, count, (ClipPlane){ v3_neg(u), -cu + hu, 1 }, buffer_a);
    count = clip_polygon(buffer_a, count, (ClipPlane){ v, cv + hv, 2 }, buffer_b);
    count = clip_polygon(buffer_b, count, (ClipPlane){ v3_neg(v), -cv + hv, 3 }, buffer_a);

    float face_offset = v3_dot(reference->center, *reference_normal) + hn;
    uint32_t face_bits = (uint32_t)(reference_axis * 2 + (reference_positive ? 0 : 1)) << 16u;
    size_t out = 0;
    for (size_t i = 0; i < count; i++) {
        float separation = v3_dot(buffer_a[i].point, *reference_normal) - face_offset;
        if (separation >= speculative) continue;
        Candidate* candidate = &candidates[out++];
        candidate->point = v3_sub(buffer_a[i].point, v3_scale(*reference_normal, 0.5f * separation));
        candidate->separation = separation;
        candidate->feature_id = flag | face_bits | buffer_a[i].id;
    }
    return out;
}

/// Append a candidate to the manifold.
static void emit(Manifold* manifold, const Candidate* candidate) {
    ManifoldPoint* point = &manifold->points[manifold->point_count++];
    point->point = candidate->point;
    point->separation = candidate->separation;
    point->feature_id = candidate->feature_id;
}

/// Remove candidate `index` by moving the last one into its place.
static Candidate take(Candidate* candidates, size_t* count, size_t index) {
    Candidate chosen = candidates[index];
    candidates[index] = candidates[*count - 1];
    --*count;
    return chosen;
}

/// Larger of two floats, `std::max` style: the first on a tie.
static float max2(float a, float b) { return a < b ? b : a; }

/// Keep at most four candidates: an extreme point, the one farthest from it,
/// the one spanning the largest triangle, and the one farthest outside it.
static void reduce_candidates(Candidate* candidates, size_t count, Vec3 normal, float speculative, Manifold* manifold) {
    if (count <= MAX_POINTS) {
        for (size_t i = 0; i < count; i++) emit(manifold, &candidates[i]);
        return;
    }

    Vec3 perp = perpendicular(normal);
    size_t best_index = 0;
    float best_score = -FLT_MAX;
    for (size_t i = 0; i < count; i++) {
        float score = -candidates[i].separation + v3_dot(perp, candidates[i].point);
        if (reduction_bias * score > best_score) {
            best_score = score;
            best_index = i;
        }
    }
    Candidate a = take(candidates, &count, best_index);

    best_index = 0;
    best_score = -FLT_MAX;
    for (size_t i = 0; i < count; i++) {
        Vec3 d = v3_sub(candidates[i].point, a.point);
        Vec3 tangential = v3_sub(d, v3_scale(normal, v3_dot(d, normal)));
        float depth = max2(0.0f, -candidates[i].separation);
        float score = v3_dot(tangential, tangential) + 4.0f * depth * depth;
        if (reduction_bias * score > best_score) {
            best_score = score;
            best_index = i;
        }
    }
    Candidate b = take(candidates, &count, best_index);

    float tolerance_sq = speculative * speculative;
    best_index = count;
    best_score = tolerance_sq;
    for (size_t i = 0; i < count; i++) {
        Vec3 area = v3_cross(v3_sub(b.point, a.point), v3_sub(candidates[i].point, a.point));
        float score = fabsf(v3_dot(area, normal));
        if (reduction_bias * score > best_score) {
            best_score = score;
            best_index = i;
        }
    }
    if (best_index == count) {
        emit(manifold, &a);
        emit(manifold, &b);
        return;
    }
    Candidate c = take(candidates, &count, best_index);
    float orientation = v3_dot(v3_cross(v3_sub(b.point, a.point), v3_sub(c.point, a.point)), normal) >= 0.0f ? 1.0f : -1.0f;

    best_index = count;
    best_score = tolerance_sq;
    for (size_t i = 0; i < count; i++) {
        Vec3 p = candidates[i].point;
        float area_ab = -orientation * v3_dot(v3_cross(v3_sub(b.point, a.point), v3_sub(p, a.point)), normal);
        float area_bc = -orientation * v3_dot(v3_cross(v3_sub(c.point, b.point), v3_sub(p, b.point)), normal);
        float area_ca = -orientation * v3_dot(v3_cross(v3_sub(a.point, c.point), v3_sub(p, c.point)), normal);
        float score = max2(max2(area_ab, area_bc), area_ca);
        if (reduction_bias * score > best_score) {
            best_score = score;
            best_index = i;
        }
    }
    emit(manifold, &a);
    emit(manifold, &b);
    emit(manifold, &c);
    if (best_index < count) emit(manifold, &candidates[best_index]);
}

/// The edge of `box` along `axis` that lies furthest along `direction`.
static EdgeSegment support_edge(const BoxPose* box, int axis, Vec3 direction) {
    EdgeSegment edge;
    edge.center = box->center;
    for (int k = 0; k < 3; k++) {
        if (k == axis) continue;
        Vec3 a = axis_of(&box->basis, k);
        float sign = v3_dot(a, direction) >= 0.0f ? 1.0f : -1.0f;
        edge.center = v3_add(edge.center, v3_scale(a, sign * extent_of(box->half_extents, k)));
    }
    edge.direction = axis_of(&box->basis, axis);
    edge.half_length = extent_of(box->half_extents, axis);
    return edge;
}

/// `v` limited to [lo, hi], `std::clamp` style.
static float clampf(float v, float lo, float hi) { return v < lo ? lo : hi < v ? hi : v; }

/// Closest points between two edge segments.
static void closest_points(const EdgeSegment* a, const EdgeSegment* b, Vec3* pa, Vec3* pb) {
    Vec3 r = v3_sub(a->center, b->center);
    float dd = v3_dot(a->direction, b->direction);
    float ra = v3_dot(a->direction, r);
    float rb = v3_dot(b->direction, r);
    float denominator = 1.0f - dd * dd;
    float s = 0.0f;
    if (denominator > 1e-6f) s = clampf((dd * rb - ra) / denominator, -a->half_length, a->half_length);
    float t = clampf(dd * s + rb, -b->half_length, b->half_length);
    s = clampf(dd * t - ra, -a->half_length, a->half_length);
    *pa = v3_add(a->center, v3_scale(a->direction, s));
    *pb = v3_add(b->center, v3_scale(b->direction, t));
}

/// Carry impulses over from points whose feature id persisted.
static void warm_start(const Manifold* previous, Manifold* manifold) {
    for (uint32_t i = 0; i < manifold->point_count; i++) {
        ManifoldPoint* point = &manifold->points[i];
        for (uint32_t j = 0; j < previous->point_count; j++) {
            const ManifoldPoint* old = &previous->points[j];
            if (old->feature_id == point->feature_id) {
                point->normal_impulse = old->normal_impulse;
                point->persisted = true;
                break;
            }
        }
    }
}

/// Rebuild the manifold between two boxes, reusing last frame's separating
/// axis as an early out and its points for warm starting.
static void collide_boxes(const BoxPose* a, const BoxPose* b, Manifold* manifold) {
    Manifold previous = *manifold;
    manifold->point_count = 0;
    memset(manifold->points, 0, sizeof manifold->points);

    if (v3_dot(previous.separating_axis, previous.separating_axis) > 0.5f) {
        if (separation_along(a, b, previous.separating_axis) > speculative_distance) {
            manifold->normal = previous.separating_axis;
            return;
        }
    }

    Vec3 d = v3_sub(b->center, a->center);
    int best_face = -1;
    float best_face_separation = -FLT_MAX;
    for (int i = 0; i < 6; i++) {
        Vec3 n = i < 3 ? axis_of(&a->basis, i) : axis_of(&b->basis, i - 3);
        float s = separation_along(a, b, n);
        if (s > speculative_distance) {
            manifold->separating_axis = n;
            manifold->normal = n;
            return;
        }
        if (s > best_face_separation) {
            best_face_separation = s;
            best_face = i;
        }
    }

    int best_edge_a = -1;
    int best_edge_b = -1;
    Vec3 best_edge_axis = { 0.0f, 0.0f, 0.0f };
    float best_edge_separation = -FLT_MAX;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Vec3 n = v3_cross(axis_of(&a->basis, i), axis_of(&b->basis, j));
            float length_sq = v3_dot(n, n);
            if (length_sq < parallel_edge_tolerance * parallel_edge_tolerance) continue;
            n = v3_div(n, sqrtf(length_sq));
            float s = separation_along(a, b, n);
            if (s > speculative_distance) {
                manifold->separating_axis = n;
                manifold->normal = n;
                return;
            }
            if (s > best_edge_separation) {
                best_edge_separation = s;
                best_edge_a = i;
                best_edge_b = j;
                best_edge_axis = v3_dot(n, d) >= 0.0f ? n : v3_neg(n);
            }
        }
    }

    Vec3 normal;
    if (best_edge_a >= 0 && best_edge_separation > best_face_separation + linear_slop) {
        normal = best_edge_axis;
        EdgeSegment edge_a = support_edge(a, best_edge_a, normal);
        EdgeSegment edge_b = support_edge(b, best_edge_b, v3_neg(normal));
        Vec3 pa, pb;
        closest_points(&edge_a, &edge_b, &pa, &pb);
        ManifoldPoint* point = &manifold->points[0];
        point->point = v3_scale(v3_add(pa, pb), 0.5f);
        point->separation = v3_dot(v3_sub(pb, pa), normal);
        point->feature_id = edge_contact_flag | ((uint32_t)best_edge_a << 8u) | (uint32_t)best_edge_b;
        manifold->point_count = 1;
    } else {
        Candidate candidates[MAX_CLIP];
        size_t count;
        Vec3 reference_normal;
        if (best_face < 3) {
            bool positive = v3_dot(axis_of(&a->basis, best_face), d) >= 0.0f;
            ReferenceFace face = { a, b, best_face, positive, 0u };
            count = face_candidates(&face, speculative_distance, candidates, &reference_normal);
            normal = reference_normal;
        } else {
            int axis = best_face - 3;
            bool positive = v3_dot(axis_of(&b->basis, axis), d) <= 0.0f;
            ReferenceFace face = { b, a, axis, positive, reference_b_flag };
            count = face_candidates(&face, speculative_distance, candidates, &reference_normal);
            normal = v3_neg(reference_normal);
        }
        if (count == 0) {
            manifold->separating_axis = normal;
            manifold->normal = normal;
            return;
        }
        reduce_candidates(candidates, count, normal, speculative_distance, manifold);
    }

    manifold->normal = normal;
    manifold->separating_axis = normal;
    warm_start(&previous, manifold);
}

static uint64_t run(void* state) {
    BoxBox* s = state;
    memset(s->manifolds, 0, PAIRS * sizeof(Manifold));
    for (int f = 0; f < FRAMES; f++) {
        float t = (float)f;
        for (int i = 0; i < PAIRS; i++) {
            const Pair* p = &s->pairs[i];
            BoxPose b = p->b;
            b.center = v3_add(b.center, v3_scale(p->velocity, t));
            collide_boxes(&p->a, &b, &s->manifolds[i]);
        }
    }
    uint64_t h = 0;
    for (int i = 0; i < PAIRS; i++) {
        const Manifold* m = &s->manifolds[i];
        h = hash_add(h, (uint64_t)f32_bits(m->normal.x) | ((uint64_t)f32_bits(m->normal.z) << 32));
        h = hash_add(h, m->point_count);
        for (uint32_t k = 0; k < m->point_count; k++) {
            const ManifoldPoint* p = &m->points[k];
            h = hash_add(h, (uint64_t)f32_bits(p->point.y) | ((uint64_t)f32_bits(p->separation) << 32));
            h = hash_add(h, (uint64_t)p->feature_id | ((uint64_t)p->persisted << 32));
        }
    }
    return h;
}

static void teardown(void* state) {
    BoxBox* s = state;
    free(s->pairs);
    free(s->manifolds);
}

const Case boxbox_case = { "boxbox", sizeof(BoxBox), setup, run, teardown };
