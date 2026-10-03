#include "box_collision.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <utility>

namespace bench::phys {

namespace {

using vm::BoxPose;
using vm::Vec3;

constexpr uint32_t reference_b_flag = 0x80000000u;
constexpr uint32_t edge_contact_flag = 0x40000000u;
constexpr float parallel_edge_tolerance = 0.005f;
constexpr float reduction_bias = 0.95f;
constexpr uint32_t max_points = max_manifold_points;
constexpr float float_max = std::numeric_limits<float>::max();

struct ClipVertex {
    Vec3 point{};
    uint32_t id{};
    uint32_t edge{};
};

struct Candidate {
    Vec3 point{};
    float separation{};
    uint32_t feature_id{};
};

using ClipPolygon = std::array<ClipVertex, 16>;
using Candidates = std::array<Candidate, 16>;

struct ClipPlane {
    Vec3 normal{};
    float offset{};
    uint32_t index{};
};

struct ReferenceFace {
    const BoxPose* reference{};
    const BoxPose* incident{};
    int axis{};
    bool positive{};
    uint32_t flag{};
};

struct EdgeSegment {
    Vec3 center{};
    Vec3 direction{};
    float half_length{};
};

/// Column `index` of a basis.
Vec3 axis_of(const vm::Basis& basis, int index) {
    switch (index) {
    case 0: return basis.x;
    case 1: return basis.y;
    default: return basis.z;
    }
}

/// Component `index` of the half extents.
float extent_of(Vec3 half_extents, int index) {
    switch (index) {
    case 0: return half_extents.x;
    case 1: return half_extents.y;
    default: return half_extents.z;
    }
}

/// Half the box's width along `n`.
float projected_radius(const BoxPose& box, Vec3 n) {
    return std::fabs(dot(n, box.basis.x)) * box.half_extents.x + std::fabs(dot(n, box.basis.y)) * box.half_extents.y +
           std::fabs(dot(n, box.basis.z)) * box.half_extents.z;
}

/// Gap between the boxes along `n`; negative when they overlap.
float separation_along(const BoxPose& a, const BoxPose& b, Vec3 n) {
    return std::fabs(dot(b.center - a.center, n)) - projected_radius(a, n) - projected_radius(b, n);
}

/// Some unit vector perpendicular to the unit vector `n`.
Vec3 perpendicular(Vec3 n) {
    if (std::fabs(n.x) > 0.57735f) return normalize(Vec3{n.y, -n.x, 0.0f});
    return normalize(Vec3{0.0f, n.z, -n.y});
}

/// Feature id of a box corner from which side of each axis it lies on.
uint32_t vertex_id(int face_axis, bool positive, int u_axis, bool u_positive, int v_axis, bool v_positive) {
    uint32_t id = 0;
    if (positive) id |= 1u << face_axis;
    if (u_positive) id |= 1u << u_axis;
    if (v_positive) id |= 1u << v_axis;
    return id;
}

/// The face of `box` most anti-parallel to `reference_normal`, as four corners.
size_t incident_face(const BoxPose& box, Vec3 reference_normal, ClipPolygon& polygon) {
    int best_axis = 0;
    float best = float_max;
    bool positive = false;
    for (int axis = 0; axis < 3; axis++) {
        const float d = dot(axis_of(box.basis, axis), reference_normal);
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
    const int u_axis = (best_axis + 1) % 3;
    const int v_axis = (best_axis + 2) % 3;
    const Vec3 n = axis_of(box.basis, best_axis) * (positive ? 1.0f : -1.0f);
    const Vec3 u = axis_of(box.basis, u_axis);
    const Vec3 v = axis_of(box.basis, v_axis);
    const float hn = extent_of(box.half_extents, best_axis);
    const float hu = extent_of(box.half_extents, u_axis);
    const float hv = extent_of(box.half_extents, v_axis);
    const Vec3 face_center = box.center + n * hn;

    constexpr std::array<std::pair<float, float>, 4> corners{
        std::pair{1.0f, 1.0f}, std::pair{-1.0f, 1.0f}, std::pair{-1.0f, -1.0f}, std::pair{1.0f, -1.0f}};
    for (size_t i = 0; i < 4; i++) {
        const auto [su, sv] = corners[i];
        polygon[i].point = face_center + u * (su * hu) + v * (sv * hv);
        polygon[i].id = vertex_id(best_axis, positive, u_axis, su > 0.0f, v_axis, sv > 0.0f);
        polygon[i].edge = static_cast<uint32_t>(i);
    }
    return 4;
}

/// Sutherland-Hodgman clip of a polygon against one plane, tracking the
/// feature id of every vertex it creates.
size_t clip_polygon(const ClipPolygon& input, size_t count, const ClipPlane& plane, ClipPolygon& output) {
    const uint32_t plane_index = plane.index;
    size_t out = 0;
    for (size_t i = 0; i < count; i++) {
        const ClipVertex& current = input[i];
        const ClipVertex& next = input[(i + 1) % count];
        const float d_current = dot(current.point, plane.normal) - plane.offset;
        const float d_next = dot(next.point, plane.normal) - plane.offset;
        if (d_current <= 0.0f) {
            if (out < output.size()) output[out++] = current;
        }
        if ((d_current < 0.0f && d_next > 0.0f) || (d_current > 0.0f && d_next < 0.0f)) {
            const float t = d_current / (d_current - d_next);
            ClipVertex vertex{};
            vertex.point = current.point + (next.point - current.point) * t;
            const bool leaving = d_current < 0.0f;
            if (current.edge < 4) {
                vertex.id = 8u + current.edge * 4u + plane_index;
            } else {
                vertex.id = 24u + (current.edge - 4u) * 4u + plane_index;
            }
            vertex.edge = leaving ? 4u + plane_index : current.edge;
            if (out < output.size()) output[out++] = vertex;
        }
    }
    return out;
}

/// Clip the incident face to the reference face's side planes and keep the
/// points within the speculative distance of the reference face.
size_t face_candidates(const ReferenceFace& face, float speculative, Candidates& candidates, Vec3& reference_normal) {
    const BoxPose& reference = *face.reference;
    const BoxPose& incident = *face.incident;
    const int reference_axis = face.axis;
    const bool reference_positive = face.positive;
    const uint32_t flag = face.flag;
    reference_normal = axis_of(reference.basis, reference_axis) * (reference_positive ? 1.0f : -1.0f);
    const int u_axis = (reference_axis + 1) % 3;
    const int v_axis = (reference_axis + 2) % 3;
    const Vec3 u = axis_of(reference.basis, u_axis);
    const Vec3 v = axis_of(reference.basis, v_axis);
    const float hn = extent_of(reference.half_extents, reference_axis);
    const float hu = extent_of(reference.half_extents, u_axis);
    const float hv = extent_of(reference.half_extents, v_axis);

    ClipPolygon buffer_a{};
    ClipPolygon buffer_b{};
    size_t count = incident_face(incident, reference_normal, buffer_a);
    const float cu = dot(reference.center, u);
    const float cv = dot(reference.center, v);
    count = clip_polygon(buffer_a, count, ClipPlane{u, cu + hu, 0}, buffer_b);
    count = clip_polygon(buffer_b, count, ClipPlane{-u, -cu + hu, 1}, buffer_a);
    count = clip_polygon(buffer_a, count, ClipPlane{v, cv + hv, 2}, buffer_b);
    count = clip_polygon(buffer_b, count, ClipPlane{-v, -cv + hv, 3}, buffer_a);

    const float face_offset = dot(reference.center, reference_normal) + hn;
    const uint32_t face_bits = static_cast<uint32_t>(reference_axis * 2 + (reference_positive ? 0 : 1)) << 16u;
    size_t out = 0;
    for (size_t i = 0; i < count; i++) {
        const float separation = dot(buffer_a[i].point, reference_normal) - face_offset;
        if (separation >= speculative) continue;
        Candidate& candidate = candidates[out++];
        candidate.point = buffer_a[i].point - reference_normal * (0.5f * separation);
        candidate.separation = separation;
        candidate.feature_id = flag | face_bits | buffer_a[i].id;
    }
    return out;
}

/// Keep at most four candidates: an extreme point, the one farthest from it,
/// the one spanning the largest triangle, and the one farthest outside it.
void reduce_candidates(Candidates& candidates, size_t count, Vec3 normal, float speculative, Manifold& manifold) {
    const auto emit = [&](const Candidate& candidate) {
        ManifoldPoint& point = manifold.points[manifold.point_count++];
        point.point = candidate.point;
        point.separation = candidate.separation;
        point.feature_id = candidate.feature_id;
    };

    if (count <= max_points) {
        for (size_t i = 0; i < count; i++) emit(candidates[i]);
        return;
    }

    const auto take = [&](size_t index) {
        const Candidate chosen = candidates[index];
        candidates[index] = candidates[count - 1];
        --count;
        return chosen;
    };

    const Vec3 perp = perpendicular(normal);
    size_t best_index = 0;
    float best_score = -float_max;
    for (size_t i = 0; i < count; i++) {
        const float score = -candidates[i].separation + dot(perp, candidates[i].point);
        if (reduction_bias * score > best_score) {
            best_score = score;
            best_index = i;
        }
    }
    const Candidate a = take(best_index);

    best_index = 0;
    best_score = -float_max;
    for (size_t i = 0; i < count; i++) {
        const Vec3 d = candidates[i].point - a.point;
        const Vec3 tangential = d - normal * dot(d, normal);
        const float depth = std::max(0.0f, -candidates[i].separation);
        const float score = dot(tangential, tangential) + 4.0f * depth * depth;
        if (reduction_bias * score > best_score) {
            best_score = score;
            best_index = i;
        }
    }
    const Candidate b = take(best_index);

    const float tolerance_sq = speculative * speculative;
    best_index = count;
    best_score = tolerance_sq;
    for (size_t i = 0; i < count; i++) {
        const Vec3 area = cross(b.point - a.point, candidates[i].point - a.point);
        const float score = std::fabs(dot(area, normal));
        if (reduction_bias * score > best_score) {
            best_score = score;
            best_index = i;
        }
    }
    if (best_index == count) {
        emit(a);
        emit(b);
        return;
    }
    const Candidate c = take(best_index);
    const float orientation = dot(cross(b.point - a.point, c.point - a.point), normal) >= 0.0f ? 1.0f : -1.0f;

    best_index = count;
    best_score = tolerance_sq;
    for (size_t i = 0; i < count; i++) {
        const Vec3 p = candidates[i].point;
        const float area_ab = -orientation * dot(cross(b.point - a.point, p - a.point), normal);
        const float area_bc = -orientation * dot(cross(c.point - b.point, p - b.point), normal);
        const float area_ca = -orientation * dot(cross(a.point - c.point, p - c.point), normal);
        const float score = std::max({area_ab, area_bc, area_ca});
        if (reduction_bias * score > best_score) {
            best_score = score;
            best_index = i;
        }
    }
    emit(a);
    emit(b);
    emit(c);
    if (best_index < count) emit(candidates[best_index]);
}

/// The edge of `box` along `axis` that lies furthest along `direction`.
EdgeSegment support_edge(const BoxPose& box, int axis, Vec3 direction) {
    EdgeSegment edge{};
    edge.center = box.center;
    for (int k = 0; k < 3; k++) {
        if (k == axis) continue;
        const Vec3 a = axis_of(box.basis, k);
        const float sign = dot(a, direction) >= 0.0f ? 1.0f : -1.0f;
        edge.center = edge.center + a * (sign * extent_of(box.half_extents, k));
    }
    edge.direction = axis_of(box.basis, axis);
    edge.half_length = extent_of(box.half_extents, axis);
    return edge;
}

/// Closest points between two edge segments.
void closest_points(const EdgeSegment& a, const EdgeSegment& b, Vec3& pa, Vec3& pb) {
    const Vec3 r = a.center - b.center;
    const float dd = dot(a.direction, b.direction);
    const float ra = dot(a.direction, r);
    const float rb = dot(b.direction, r);
    const float denominator = 1.0f - dd * dd;
    float s = 0.0f;
    if (denominator > 1e-6f) s = std::clamp((dd * rb - ra) / denominator, -a.half_length, a.half_length);
    const float t = std::clamp(dd * s + rb, -b.half_length, b.half_length);
    s = std::clamp(dd * t - ra, -a.half_length, a.half_length);
    pa = a.center + a.direction * s;
    pb = b.center + b.direction * t;
}

/// Carry impulses over from points whose feature id persisted.
void warm_start(const Manifold& previous, Manifold& manifold) {
    for (uint32_t i = 0; i < manifold.point_count; i++) {
        ManifoldPoint& point = manifold.points[i];
        for (uint32_t j = 0; j < previous.point_count; j++) {
            const ManifoldPoint& old = previous.points[j];
            if (old.feature_id == point.feature_id) {
                point.normal_impulse = old.normal_impulse;
                point.persisted = true;
                break;
            }
        }
    }
}

} // namespace

vm::Aabb box_aabb(const BoxPose& box) {
    const Vec3 extent =
        vm::abs(box.basis.x) * box.half_extents.x + vm::abs(box.basis.y) * box.half_extents.y + vm::abs(box.basis.z) * box.half_extents.z;
    return {box.center - extent, box.center + extent};
}

void collide_boxes(const BoxPose& a, const BoxPose& b, const CollisionTolerances& tolerances, Manifold& manifold) {
    const float speculative_distance = tolerances.speculative_distance;
    const float linear_slop = tolerances.linear_slop;
    const Manifold previous = manifold;
    manifold.point_count = 0;
    for (ManifoldPoint& point : manifold.points) point = ManifoldPoint{};

    if (dot(previous.separating_axis, previous.separating_axis) > 0.5f) {
        if (separation_along(a, b, previous.separating_axis) > speculative_distance) {
            manifold.normal = previous.separating_axis;
            return;
        }
    }

    const Vec3 d = b.center - a.center;
    int best_face = -1;
    float best_face_separation = -float_max;
    for (int i = 0; i < 6; i++) {
        const Vec3 n = i < 3 ? axis_of(a.basis, i) : axis_of(b.basis, i - 3);
        const float s = separation_along(a, b, n);
        if (s > speculative_distance) {
            manifold.separating_axis = n;
            manifold.normal = n;
            return;
        }
        if (s > best_face_separation) {
            best_face_separation = s;
            best_face = i;
        }
    }

    int best_edge_a = -1;
    int best_edge_b = -1;
    Vec3 best_edge_axis{};
    float best_edge_separation = -float_max;
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            Vec3 n = cross(axis_of(a.basis, i), axis_of(b.basis, j));
            const float length_sq = dot(n, n);
            if (length_sq < parallel_edge_tolerance * parallel_edge_tolerance) continue;
            n = n / std::sqrt(length_sq);
            const float s = separation_along(a, b, n);
            if (s > speculative_distance) {
                manifold.separating_axis = n;
                manifold.normal = n;
                return;
            }
            if (s > best_edge_separation) {
                best_edge_separation = s;
                best_edge_a = i;
                best_edge_b = j;
                best_edge_axis = dot(n, d) >= 0.0f ? n : -n;
            }
        }
    }

    Vec3 normal{};
    if (best_edge_a >= 0 && best_edge_separation > best_face_separation + linear_slop) {
        normal = best_edge_axis;
        const EdgeSegment edge_a = support_edge(a, best_edge_a, normal);
        const EdgeSegment edge_b = support_edge(b, best_edge_b, -normal);
        Vec3 pa{};
        Vec3 pb{};
        closest_points(edge_a, edge_b, pa, pb);
        ManifoldPoint& point = manifold.points[0];
        point.point = (pa + pb) * 0.5f;
        point.separation = dot(pb - pa, normal);
        point.feature_id = edge_contact_flag | (static_cast<uint32_t>(best_edge_a) << 8u) | static_cast<uint32_t>(best_edge_b);
        manifold.point_count = 1;
    } else {
        Candidates candidates{};
        size_t count = 0;
        Vec3 reference_normal{};
        if (best_face < 3) {
            const bool positive = dot(axis_of(a.basis, best_face), d) >= 0.0f;
            count = face_candidates(ReferenceFace{&a, &b, best_face, positive, 0u}, speculative_distance, candidates,
                                    reference_normal);
            normal = reference_normal;
        } else {
            const int axis = best_face - 3;
            const bool positive = dot(axis_of(b.basis, axis), d) <= 0.0f;
            count = face_candidates(ReferenceFace{&b, &a, axis, positive, reference_b_flag}, speculative_distance,
                                    candidates, reference_normal);
            normal = -reference_normal;
        }
        if (count == 0) {
            manifold.separating_axis = normal;
            manifold.normal = normal;
            return;
        }
        reduce_candidates(candidates, count, normal, speculative_distance, manifold);
    }

    manifold.normal = normal;
    manifold.separating_axis = normal;
    warm_start(previous, manifold);
}

Softness make_softness(float hertz, float damping_ratio, float h) {
    if (hertz <= 0.0f) return Softness{0.0f, 1.0f, 0.0f};
    const float omega = 2.0f * std::numbers::pi_v<float> * hertz;
    const float a1 = 2.0f * damping_ratio + h * omega;
    const float a2 = h * omega * a1;
    const float a3 = 1.0f / (1.0f + a2);
    return Softness{omega / a1, a2 * a3, a3};
}

} // namespace bench::phys
