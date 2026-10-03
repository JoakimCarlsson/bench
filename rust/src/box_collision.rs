//! The engine's box_collision: separating-axis tests over 15 axes,
//! incident-face clipping, reduction to four points and warm starting.
use crate::contact::{MAX_MANIFOLD_POINTS, Manifold, ManifoldPoint};
use crate::vecmath::{self, Aabb, Basis, BoxPose, Vec3};

const MAX_POINTS: usize = MAX_MANIFOLD_POINTS;
const MAX_CLIP: usize = 16;

const REFERENCE_B_FLAG: u32 = 0x8000_0000;
const EDGE_CONTACT_FLAG: u32 = 0x4000_0000;
const PARALLEL_EDGE_TOLERANCE: f32 = 0.005;
const REDUCTION_BIAS: f32 = 0.95;

#[derive(Clone, Copy)]
pub struct CollisionTolerances {
    pub speculative_distance: f32,
    pub linear_slop: f32,
}

impl Default for CollisionTolerances {
    /// The engine's defaults.
    fn default() -> CollisionTolerances {
        CollisionTolerances { speculative_distance: 0.02, linear_slop: 0.005 }
    }
}

#[derive(Clone, Copy, Default)]
struct ClipVertex {
    point: Vec3,
    id: u32,
    edge: u32,
}

#[derive(Clone, Copy, Default)]
struct Candidate {
    point: Vec3,
    separation: f32,
    feature_id: u32,
}

struct ClipPlane {
    normal: Vec3,
    offset: f32,
    index: u32,
}

struct ReferenceFace<'a> {
    reference: &'a BoxPose,
    incident: &'a BoxPose,
    axis: usize,
    positive: bool,
    flag: u32,
}

struct EdgeSegment {
    center: Vec3,
    direction: Vec3,
    half_length: f32,
}

/// Column `index` of a basis.
fn axis_of(basis: &Basis, index: usize) -> Vec3 {
    match index {
        0 => basis.x,
        1 => basis.y,
        _ => basis.z,
    }
}

/// Component `index` of the half extents.
fn extent_of(half_extents: Vec3, index: usize) -> f32 {
    match index {
        0 => half_extents.x,
        1 => half_extents.y,
        _ => half_extents.z,
    }
}

/// Half the box's width along `n`.
fn projected_radius(b: &BoxPose, n: Vec3) -> f32 {
    n.dot(b.basis.x).abs() * b.half_extents.x
        + n.dot(b.basis.y).abs() * b.half_extents.y
        + n.dot(b.basis.z).abs() * b.half_extents.z
}

/// Gap between the boxes along `n`; negative when they overlap.
fn separation_along(a: &BoxPose, b: &BoxPose, n: Vec3) -> f32 {
    (b.center - a.center).dot(n).abs() - projected_radius(a, n) - projected_radius(b, n)
}

/// Some unit vector perpendicular to the unit vector `n`.
fn perpendicular(n: Vec3) -> Vec3 {
    if n.x.abs() > 0.57735 {
        return Vec3::new(n.y, -n.x, 0.0).normalize();
    }
    Vec3::new(0.0, n.z, -n.y).normalize()
}

/// Feature id of a box corner from which side of each axis it lies on.
fn vertex_id(face_axis: usize, positive: bool, u_axis: usize, u_positive: bool, v_axis: usize, v_positive: bool) -> u32 {
    let mut id = 0u32;
    if positive {
        id |= 1 << face_axis;
    }
    if u_positive {
        id |= 1 << u_axis;
    }
    if v_positive {
        id |= 1 << v_axis;
    }
    id
}

/// The face of `b` most anti-parallel to `reference_normal`, as four corners.
fn incident_face(b: &BoxPose, reference_normal: Vec3, polygon: &mut [ClipVertex; MAX_CLIP]) -> usize {
    let mut best_axis = 0;
    let mut best = f32::MAX;
    let mut positive = false;
    for axis in 0..3 {
        let d = axis_of(&b.basis, axis).dot(reference_normal);
        if d < best {
            best = d;
            best_axis = axis;
            positive = true;
        }
        if -d < best {
            best = -d;
            best_axis = axis;
            positive = false;
        }
    }
    let u_axis = (best_axis + 1) % 3;
    let v_axis = (best_axis + 2) % 3;
    let n = axis_of(&b.basis, best_axis) * if positive { 1.0 } else { -1.0 };
    let u = axis_of(&b.basis, u_axis);
    let v = axis_of(&b.basis, v_axis);
    let hn = extent_of(b.half_extents, best_axis);
    let hu = extent_of(b.half_extents, u_axis);
    let hv = extent_of(b.half_extents, v_axis);
    let face_center = b.center + n * hn;

    const CORNERS: [(f32, f32); 4] = [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)];
    for (i, &(su, sv)) in CORNERS.iter().enumerate() {
        polygon[i] = ClipVertex {
            point: face_center + u * (su * hu) + v * (sv * hv),
            id: vertex_id(best_axis, positive, u_axis, su > 0.0, v_axis, sv > 0.0),
            edge: i as u32,
        };
    }
    4
}

/// Sutherland-Hodgman clip of a polygon against one plane, tracking the
/// feature id of every vertex it creates.
fn clip_polygon(input: &[ClipVertex; MAX_CLIP], count: usize, plane: &ClipPlane, output: &mut [ClipVertex; MAX_CLIP]) -> usize {
    let plane_index = plane.index;
    let mut out = 0;
    for i in 0..count {
        let current = &input[i];
        let next = &input[(i + 1) % count];
        let d_current = current.point.dot(plane.normal) - plane.offset;
        let d_next = next.point.dot(plane.normal) - plane.offset;
        if d_current <= 0.0 && out < MAX_CLIP {
            output[out] = *current;
            out += 1;
        }
        if (d_current < 0.0 && d_next > 0.0) || (d_current > 0.0 && d_next < 0.0) {
            let t = d_current / (d_current - d_next);
            let leaving = d_current < 0.0;
            let id = if current.edge < 4 {
                8 + current.edge * 4 + plane_index
            } else {
                24 + (current.edge - 4) * 4 + plane_index
            };
            let vertex = ClipVertex {
                point: current.point + (next.point - current.point) * t,
                id,
                edge: if leaving { 4 + plane_index } else { current.edge },
            };
            if out < MAX_CLIP {
                output[out] = vertex;
                out += 1;
            }
        }
    }
    out
}

/// Clip the incident face to the reference face's side planes and keep the
/// points within the speculative distance of the reference face. Returns the
/// candidate count and the reference face's normal.
fn face_candidates(face: &ReferenceFace, speculative: f32, candidates: &mut [Candidate; MAX_CLIP]) -> (usize, Vec3) {
    let reference = face.reference;
    let reference_axis = face.axis;
    let reference_positive = face.positive;
    let reference_normal = axis_of(&reference.basis, reference_axis) * if reference_positive { 1.0 } else { -1.0 };
    let u_axis = (reference_axis + 1) % 3;
    let v_axis = (reference_axis + 2) % 3;
    let u = axis_of(&reference.basis, u_axis);
    let v = axis_of(&reference.basis, v_axis);
    let hn = extent_of(reference.half_extents, reference_axis);
    let hu = extent_of(reference.half_extents, u_axis);
    let hv = extent_of(reference.half_extents, v_axis);

    let mut buffer_a = [ClipVertex::default(); MAX_CLIP];
    let mut buffer_b = [ClipVertex::default(); MAX_CLIP];
    let mut count = incident_face(face.incident, reference_normal, &mut buffer_a);
    let cu = reference.center.dot(u);
    let cv = reference.center.dot(v);
    count = clip_polygon(&buffer_a, count, &ClipPlane { normal: u, offset: cu + hu, index: 0 }, &mut buffer_b);
    count = clip_polygon(&buffer_b, count, &ClipPlane { normal: -u, offset: -cu + hu, index: 1 }, &mut buffer_a);
    count = clip_polygon(&buffer_a, count, &ClipPlane { normal: v, offset: cv + hv, index: 2 }, &mut buffer_b);
    count = clip_polygon(&buffer_b, count, &ClipPlane { normal: -v, offset: -cv + hv, index: 3 }, &mut buffer_a);

    let face_offset = reference.center.dot(reference_normal) + hn;
    let face_bits = ((reference_axis * 2 + if reference_positive { 0 } else { 1 }) as u32) << 16;
    let mut out = 0;
    for vertex in &buffer_a[..count] {
        let separation = vertex.point.dot(reference_normal) - face_offset;
        if separation >= speculative {
            continue;
        }
        candidates[out] = Candidate {
            point: vertex.point - reference_normal * (0.5 * separation),
            separation,
            feature_id: face.flag | face_bits | vertex.id,
        };
        out += 1;
    }
    (out, reference_normal)
}

/// Append a candidate to the manifold.
fn emit(manifold: &mut Manifold, candidate: &Candidate) {
    let point = &mut manifold.points[manifold.point_count as usize];
    point.point = candidate.point;
    point.separation = candidate.separation;
    point.feature_id = candidate.feature_id;
    manifold.point_count += 1;
}

/// Remove candidate `index` by moving the last one into its place.
fn take(candidates: &mut [Candidate; MAX_CLIP], count: &mut usize, index: usize) -> Candidate {
    let chosen = candidates[index];
    candidates[index] = candidates[*count - 1];
    *count -= 1;
    chosen
}

/// Keep at most four candidates: an extreme point, the one farthest from it,
/// the one spanning the largest triangle, and the one farthest outside it.
fn reduce_candidates(candidates: &mut [Candidate; MAX_CLIP], mut count: usize, normal: Vec3, speculative: f32, manifold: &mut Manifold) {
    if count <= MAX_POINTS {
        for candidate in &candidates[..count] {
            emit(manifold, candidate);
        }
        return;
    }

    let perp = perpendicular(normal);
    let mut best_index = 0;
    let mut best_score = -f32::MAX;
    for (i, candidate) in candidates[..count].iter().enumerate() {
        let score = -candidate.separation + perp.dot(candidate.point);
        if REDUCTION_BIAS * score > best_score {
            best_score = score;
            best_index = i;
        }
    }
    let a = take(candidates, &mut count, best_index);

    best_index = 0;
    best_score = -f32::MAX;
    for (i, candidate) in candidates[..count].iter().enumerate() {
        let d = candidate.point - a.point;
        let tangential = d - normal * d.dot(normal);
        let depth = vecmath::max(0.0, -candidate.separation);
        let score = tangential.dot(tangential) + 4.0 * depth * depth;
        if REDUCTION_BIAS * score > best_score {
            best_score = score;
            best_index = i;
        }
    }
    let b = take(candidates, &mut count, best_index);

    let tolerance_sq = speculative * speculative;
    best_index = count;
    best_score = tolerance_sq;
    for (i, candidate) in candidates[..count].iter().enumerate() {
        let area = (b.point - a.point).cross(candidate.point - a.point);
        let score = area.dot(normal).abs();
        if REDUCTION_BIAS * score > best_score {
            best_score = score;
            best_index = i;
        }
    }
    if best_index == count {
        emit(manifold, &a);
        emit(manifold, &b);
        return;
    }
    let c = take(candidates, &mut count, best_index);
    let orientation = if (b.point - a.point).cross(c.point - a.point).dot(normal) >= 0.0 { 1.0 } else { -1.0 };

    best_index = count;
    best_score = tolerance_sq;
    for (i, candidate) in candidates[..count].iter().enumerate() {
        let p = candidate.point;
        let area_ab = -orientation * (b.point - a.point).cross(p - a.point).dot(normal);
        let area_bc = -orientation * (c.point - b.point).cross(p - b.point).dot(normal);
        let area_ca = -orientation * (a.point - c.point).cross(p - c.point).dot(normal);
        let score = vecmath::max3(area_ab, area_bc, area_ca);
        if REDUCTION_BIAS * score > best_score {
            best_score = score;
            best_index = i;
        }
    }
    emit(manifold, &a);
    emit(manifold, &b);
    emit(manifold, &c);
    if best_index < count {
        emit(manifold, &candidates[best_index]);
    }
}

/// The edge of `b` along `axis` that lies furthest along `direction`.
fn support_edge(b: &BoxPose, axis: usize, direction: Vec3) -> EdgeSegment {
    let mut center = b.center;
    for k in 0..3 {
        if k == axis {
            continue;
        }
        let a = axis_of(&b.basis, k);
        let sign = if a.dot(direction) >= 0.0 { 1.0 } else { -1.0 };
        center = center + a * (sign * extent_of(b.half_extents, k));
    }
    EdgeSegment { center, direction: axis_of(&b.basis, axis), half_length: extent_of(b.half_extents, axis) }
}

/// Closest points between two edge segments.
fn closest_points(a: &EdgeSegment, b: &EdgeSegment) -> (Vec3, Vec3) {
    let r = a.center - b.center;
    let dd = a.direction.dot(b.direction);
    let ra = a.direction.dot(r);
    let rb = b.direction.dot(r);
    let denominator = 1.0 - dd * dd;
    let mut s = 0.0f32;
    if denominator > 1e-6 {
        s = vecmath::clamp((dd * rb - ra) / denominator, -a.half_length, a.half_length);
    }
    let t = vecmath::clamp(dd * s + rb, -b.half_length, b.half_length);
    s = vecmath::clamp(dd * t - ra, -a.half_length, a.half_length);
    (a.center + a.direction * s, b.center + b.direction * t)
}

/// Carry impulses over from points whose feature id persisted.
fn warm_start(previous: &Manifold, manifold: &mut Manifold) {
    for point in manifold.active_mut() {
        if let Some(old) = previous.active().iter().find(|old| old.feature_id == point.feature_id) {
            point.normal_impulse = old.normal_impulse;
            point.persisted = true;
        }
    }
}

/// World bounds of an oriented box.
pub fn box_aabb(b: &BoxPose) -> Aabb {
    let extent = b.basis.x.abs() * b.half_extents.x + b.basis.y.abs() * b.half_extents.y + b.basis.z.abs() * b.half_extents.z;
    Aabb { min: b.center - extent, max: b.center + extent }
}

/// Rebuilds the manifold between two boxes, reusing last step's separating
/// axis as an early out and its points for warm starting.
pub fn collide_boxes(a: &BoxPose, b: &BoxPose, tolerances: &CollisionTolerances, manifold: &mut Manifold) {
    let speculative_distance = tolerances.speculative_distance;
    let linear_slop = tolerances.linear_slop;
    let previous = *manifold;
    manifold.point_count = 0;
    manifold.points = [ManifoldPoint::default(); MAX_POINTS];

    if previous.separating_axis.dot(previous.separating_axis) > 0.5
        && separation_along(a, b, previous.separating_axis) > speculative_distance
    {
        manifold.normal = previous.separating_axis;
        return;
    }

    let d = b.center - a.center;
    let mut best_face = 0;
    let mut best_face_separation = -f32::MAX;
    for i in 0..6 {
        let n = if i < 3 { axis_of(&a.basis, i) } else { axis_of(&b.basis, i - 3) };
        let s = separation_along(a, b, n);
        if s > speculative_distance {
            manifold.separating_axis = n;
            manifold.normal = n;
            return;
        }
        if s > best_face_separation {
            best_face_separation = s;
            best_face = i;
        }
    }

    let mut best_edge: Option<(usize, usize)> = None;
    let mut best_edge_axis = Vec3::default();
    let mut best_edge_separation = -f32::MAX;
    for i in 0..3 {
        for j in 0..3 {
            let mut n = axis_of(&a.basis, i).cross(axis_of(&b.basis, j));
            let length_sq = n.dot(n);
            if length_sq < PARALLEL_EDGE_TOLERANCE * PARALLEL_EDGE_TOLERANCE {
                continue;
            }
            n = n / length_sq.sqrt();
            let s = separation_along(a, b, n);
            if s > speculative_distance {
                manifold.separating_axis = n;
                manifold.normal = n;
                return;
            }
            if s > best_edge_separation {
                best_edge_separation = s;
                best_edge = Some((i, j));
                best_edge_axis = if n.dot(d) >= 0.0 { n } else { -n };
            }
        }
    }

    let normal;
    match best_edge {
        Some((edge_a, edge_b)) if best_edge_separation > best_face_separation + linear_slop => {
            normal = best_edge_axis;
            let (pa, pb) = closest_points(&support_edge(a, edge_a, normal), &support_edge(b, edge_b, -normal));
            let point = &mut manifold.points[0];
            point.point = (pa + pb) * 0.5;
            point.separation = (pb - pa).dot(normal);
            point.feature_id = EDGE_CONTACT_FLAG | ((edge_a as u32) << 8) | edge_b as u32;
            manifold.point_count = 1;
        }
        _ => {
            let mut candidates = [Candidate::default(); MAX_CLIP];
            let count;
            if best_face < 3 {
                let positive = axis_of(&a.basis, best_face).dot(d) >= 0.0;
                let face = ReferenceFace { reference: a, incident: b, axis: best_face, positive, flag: 0 };
                let (n, reference_normal) = face_candidates(&face, speculative_distance, &mut candidates);
                count = n;
                normal = reference_normal;
            } else {
                let axis = best_face - 3;
                let positive = axis_of(&b.basis, axis).dot(d) <= 0.0;
                let face = ReferenceFace { reference: b, incident: a, axis, positive, flag: REFERENCE_B_FLAG };
                let (n, reference_normal) = face_candidates(&face, speculative_distance, &mut candidates);
                count = n;
                normal = -reference_normal;
            }
            if count == 0 {
                manifold.separating_axis = normal;
                manifold.normal = normal;
                return;
            }
            reduce_candidates(&mut candidates, count, normal, speculative_distance, manifold);
        }
    }

    manifold.normal = normal;
    manifold.separating_axis = normal;
    warm_start(&previous, manifold);
}
