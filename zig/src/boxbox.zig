//! Box-box narrowphase: 8 frames over 8192 pairs of oriented boxes, a
//! quarter of them stacked face to face. Separating-axis tests over 15 axes,
//! incident-face clipping against the reference face, reduction to four
//! points, and warm starting from the previous frame's manifold.
const std = @import("std");
const hash = @import("hash.zig");
const vm = @import("vecmath.zig");

const Vec3 = vm.Vec3;
const BoxPose = vm.BoxPose;

const BoxBox = @This();

pub const name = "boxbox";

const pair_count: usize = 8192;
const frames: usize = 8;
const max_points: usize = 4;
const max_clip: usize = 16;

const reference_b_flag: u32 = 0x80000000;
const edge_contact_flag: u32 = 0x40000000;
const parallel_edge_tolerance: f32 = 0.005;
const reduction_bias: f32 = 0.95;
const speculative_distance: f32 = 0.02;
const linear_slop: f32 = 0.005;
const float_max = std.math.floatMax(f32);

pub const ManifoldPoint = struct {
    point: Vec3 = .{},
    separation: f32 = 0.0,
    normal_impulse: f32 = 0.0,
    feature_id: u32 = 0,
    persisted: bool = false,
};

pub const Manifold = struct {
    normal: Vec3 = .{},
    separating_axis: Vec3 = .{},
    points: [max_points]ManifoldPoint = @splat(.{}),
    point_count: usize = 0,

    /// Append a candidate.
    fn emit(self: *Manifold, candidate: Candidate) void {
        const point = &self.points[self.point_count];
        point.point = candidate.point;
        point.separation = candidate.separation;
        point.feature_id = candidate.feature_id;
        self.point_count += 1;
    }
};

const ClipVertex = struct { point: Vec3 = .{}, id: u32 = 0, edge: u32 = 0 };
const Candidate = struct { point: Vec3 = .{}, separation: f32 = 0.0, feature_id: u32 = 0 };
const ClipPlane = struct { normal: Vec3, offset: f32, index: u32 };
const ReferenceFace = struct { reference: *const BoxPose, incident: *const BoxPose, axis: usize, positive: bool, flag: u32 };
const EdgeSegment = struct { center: Vec3, direction: Vec3, half_length: f32 };
const ClipPolygon = [max_clip]ClipVertex;
const Candidates = [max_clip]Candidate;

pub const Pair = struct { a: BoxPose, b: BoxPose, velocity: Vec3 };

pairs: []Pair,
manifolds: []Manifold,

pub fn init(gpa: std.mem.Allocator) !BoxBox {
    const pairs = try gpa.alloc(Pair, pair_count);
    errdefer gpa.free(pairs);
    const manifolds = try gpa.alloc(Manifold, pair_count);
    var rng: hash.Rng = .{ .s = 0xb0b0 };
    for (pairs, 0..) |*p, i| {
        const rotation = vm.Quat.random(&rng);
        p.a.half_extents = Vec3.random(&rng, 0.25, 1.5);
        p.a.center = Vec3.random(&rng, 0.0, 1000.0);
        p.a.basis = vm.Basis.fromQuat(rotation);
        p.b.half_extents = Vec3.random(&rng, 0.25, 1.5);
        if (i % 4 == 0) {
            const raw_yaw: vm.Quat = .{ .x = 0.0, .y = rng.unit() - 0.5, .z = 0.0, .w = 1.0 };
            p.b.basis = vm.Basis.fromQuat(rotation.mul(raw_yaw.normalize()));
            const slide = Vec3.random(&rng, -0.3, 0.3);
            const lift = p.a.half_extents.y + p.b.half_extents.y - 0.01;
            p.b.center = p.a.center.add(p.a.basis.y.scale(lift)).add(p.a.basis.x.scale(slide.x).add(p.a.basis.z.scale(slide.z)));
        } else {
            p.b.basis = vm.Basis.fromQuat(vm.Quat.random(&rng));
            const reach = (p.a.half_extents.length() + p.b.half_extents.length()) * 0.6;
            p.b.center = p.a.center.add(Vec3.random(&rng, -1.0, 1.0).scale(reach));
        }
        p.velocity = Vec3.random(&rng, -0.02, 0.02);
    }
    return .{ .pairs = pairs, .manifolds = manifolds };
}

pub fn deinit(self: *BoxBox, gpa: std.mem.Allocator) void {
    gpa.free(self.pairs);
    gpa.free(self.manifolds);
}

/// Column `index` of a basis.
fn axisOf(basis: vm.Basis, index: usize) Vec3 {
    return switch (index) {
        0 => basis.x,
        1 => basis.y,
        else => basis.z,
    };
}

/// Component `index` of the half extents.
fn extentOf(half_extents: Vec3, index: usize) f32 {
    return switch (index) {
        0 => half_extents.x,
        1 => half_extents.y,
        else => half_extents.z,
    };
}

/// Half the box's width along `n`.
fn projectedRadius(box: *const BoxPose, n: Vec3) f32 {
    return @abs(n.dot(box.basis.x)) * box.half_extents.x + @abs(n.dot(box.basis.y)) * box.half_extents.y +
        @abs(n.dot(box.basis.z)) * box.half_extents.z;
}

/// Gap between the boxes along `n`; negative when they overlap.
fn separationAlong(a: *const BoxPose, b: *const BoxPose, n: Vec3) f32 {
    return @abs(b.center.sub(a.center).dot(n)) - projectedRadius(a, n) - projectedRadius(b, n);
}

/// Some unit vector perpendicular to the unit vector `n`.
fn perpendicular(n: Vec3) Vec3 {
    if (@abs(n.x) > 0.57735) return Vec3.init(n.y, -n.x, 0.0).normalize();
    return Vec3.init(0.0, n.z, -n.y).normalize();
}

/// Feature id of a box corner from which side of each axis it lies on.
fn vertexId(face_axis: usize, positive: bool, u_axis: usize, u_positive: bool, v_axis: usize, v_positive: bool) u32 {
    var id: u32 = 0;
    if (positive) id |= @as(u32, 1) << @intCast(face_axis);
    if (u_positive) id |= @as(u32, 1) << @intCast(u_axis);
    if (v_positive) id |= @as(u32, 1) << @intCast(v_axis);
    return id;
}

/// The face of `box` most anti-parallel to `reference_normal`, as four corners.
fn incidentFace(box: *const BoxPose, reference_normal: Vec3, polygon: *ClipPolygon) usize {
    var best_axis: usize = 0;
    var best: f32 = float_max;
    var positive = false;
    for (0..3) |axis| {
        const d = axisOf(box.basis, axis).dot(reference_normal);
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
    const u_axis = (best_axis + 1) % 3;
    const v_axis = (best_axis + 2) % 3;
    const n = axisOf(box.basis, best_axis).scale(if (positive) 1.0 else -1.0);
    const u = axisOf(box.basis, u_axis);
    const v = axisOf(box.basis, v_axis);
    const hn = extentOf(box.half_extents, best_axis);
    const hu = extentOf(box.half_extents, u_axis);
    const hv = extentOf(box.half_extents, v_axis);
    const face_center = box.center.add(n.scale(hn));

    const corners = [4][2]f32{ .{ 1.0, 1.0 }, .{ -1.0, 1.0 }, .{ -1.0, -1.0 }, .{ 1.0, -1.0 } };
    for (corners, 0..) |corner, i| {
        const su = corner[0];
        const sv = corner[1];
        polygon[i] = .{
            .point = face_center.add(u.scale(su * hu)).add(v.scale(sv * hv)),
            .id = vertexId(best_axis, positive, u_axis, su > 0.0, v_axis, sv > 0.0),
            .edge = @intCast(i),
        };
    }
    return 4;
}

/// Sutherland-Hodgman clip of a polygon against one plane, tracking the
/// feature id of every vertex it creates.
fn clipPolygon(input: *const ClipPolygon, count: usize, plane: ClipPlane, output: *ClipPolygon) usize {
    const plane_index = plane.index;
    var out: usize = 0;
    for (0..count) |i| {
        const current = input[i];
        const next = input[(i + 1) % count];
        const d_current = current.point.dot(plane.normal) - plane.offset;
        const d_next = next.point.dot(plane.normal) - plane.offset;
        if (d_current <= 0.0 and out < max_clip) {
            output[out] = current;
            out += 1;
        }
        if ((d_current < 0.0 and d_next > 0.0) or (d_current > 0.0 and d_next < 0.0)) {
            const t = d_current / (d_current - d_next);
            const leaving = d_current < 0.0;
            const vertex: ClipVertex = .{
                .point = current.point.add(next.point.sub(current.point).scale(t)),
                .id = if (current.edge < 4) 8 + current.edge * 4 + plane_index else 24 + (current.edge - 4) * 4 + plane_index,
                .edge = if (leaving) 4 + plane_index else current.edge,
            };
            if (out < max_clip) {
                output[out] = vertex;
                out += 1;
            }
        }
    }
    return out;
}

/// Clip the incident face to the reference face's side planes and keep the
/// points within the speculative distance of the reference face.
fn faceCandidates(face: ReferenceFace, speculative: f32, candidates: *Candidates, reference_normal: *Vec3) usize {
    const reference = face.reference;
    const reference_axis = face.axis;
    const reference_positive = face.positive;
    reference_normal.* = axisOf(reference.basis, reference_axis).scale(if (reference_positive) 1.0 else -1.0);
    const u_axis = (reference_axis + 1) % 3;
    const v_axis = (reference_axis + 2) % 3;
    const u = axisOf(reference.basis, u_axis);
    const v = axisOf(reference.basis, v_axis);
    const hn = extentOf(reference.half_extents, reference_axis);
    const hu = extentOf(reference.half_extents, u_axis);
    const hv = extentOf(reference.half_extents, v_axis);

    var buffer_a: ClipPolygon = @splat(.{});
    var buffer_b: ClipPolygon = @splat(.{});
    var count = incidentFace(face.incident, reference_normal.*, &buffer_a);
    const cu = reference.center.dot(u);
    const cv = reference.center.dot(v);
    count = clipPolygon(&buffer_a, count, .{ .normal = u, .offset = cu + hu, .index = 0 }, &buffer_b);
    count = clipPolygon(&buffer_b, count, .{ .normal = u.neg(), .offset = -cu + hu, .index = 1 }, &buffer_a);
    count = clipPolygon(&buffer_a, count, .{ .normal = v, .offset = cv + hv, .index = 2 }, &buffer_b);
    count = clipPolygon(&buffer_b, count, .{ .normal = v.neg(), .offset = -cv + hv, .index = 3 }, &buffer_a);

    const face_offset = reference.center.dot(reference_normal.*) + hn;
    const face_bits = @as(u32, @intCast(reference_axis * 2 + @as(usize, if (reference_positive) 0 else 1))) << 16;
    var out: usize = 0;
    for (buffer_a[0..count]) |vertex| {
        const separation = vertex.point.dot(reference_normal.*) - face_offset;
        if (separation >= speculative) continue;
        candidates[out] = .{
            .point = vertex.point.sub(reference_normal.scale(0.5 * separation)),
            .separation = separation,
            .feature_id = face.flag | face_bits | vertex.id,
        };
        out += 1;
    }
    return out;
}

/// Remove candidate `index` by moving the last one into its place.
fn take(candidates: *Candidates, count: *usize, index: usize) Candidate {
    const chosen = candidates[index];
    candidates[index] = candidates[count.* - 1];
    count.* -= 1;
    return chosen;
}

/// Keep at most four candidates: an extreme point, the one farthest from it,
/// the one spanning the largest triangle, and the one farthest outside it.
fn reduceCandidates(candidates: *Candidates, initial_count: usize, normal: Vec3, speculative: f32, manifold: *Manifold) void {
    var count = initial_count;
    if (count <= max_points) {
        for (candidates[0..count]) |candidate| manifold.emit(candidate);
        return;
    }

    const perp = perpendicular(normal);
    var best_index: usize = 0;
    var best_score: f32 = -float_max;
    for (candidates[0..count], 0..) |candidate, i| {
        const score = -candidate.separation + perp.dot(candidate.point);
        if (reduction_bias * score > best_score) {
            best_score = score;
            best_index = i;
        }
    }
    const a = take(candidates, &count, best_index);

    best_index = 0;
    best_score = -float_max;
    for (candidates[0..count], 0..) |candidate, i| {
        const d = candidate.point.sub(a.point);
        const tangential = d.sub(normal.scale(d.dot(normal)));
        const depth = @max(0.0, -candidate.separation);
        const score = tangential.dot(tangential) + 4.0 * depth * depth;
        if (reduction_bias * score > best_score) {
            best_score = score;
            best_index = i;
        }
    }
    const b = take(candidates, &count, best_index);

    const tolerance_sq = speculative * speculative;
    best_index = count;
    best_score = tolerance_sq;
    for (candidates[0..count], 0..) |candidate, i| {
        const area = b.point.sub(a.point).cross(candidate.point.sub(a.point));
        const score = @abs(area.dot(normal));
        if (reduction_bias * score > best_score) {
            best_score = score;
            best_index = i;
        }
    }
    if (best_index == count) {
        manifold.emit(a);
        manifold.emit(b);
        return;
    }
    const c = take(candidates, &count, best_index);
    const orientation: f32 = if (b.point.sub(a.point).cross(c.point.sub(a.point)).dot(normal) >= 0.0) 1.0 else -1.0;

    best_index = count;
    best_score = tolerance_sq;
    for (candidates[0..count], 0..) |candidate, i| {
        const p = candidate.point;
        const area_ab = -orientation * b.point.sub(a.point).cross(p.sub(a.point)).dot(normal);
        const area_bc = -orientation * c.point.sub(b.point).cross(p.sub(b.point)).dot(normal);
        const area_ca = -orientation * a.point.sub(c.point).cross(p.sub(c.point)).dot(normal);
        const score = @max(area_ab, area_bc, area_ca);
        if (reduction_bias * score > best_score) {
            best_score = score;
            best_index = i;
        }
    }
    manifold.emit(a);
    manifold.emit(b);
    manifold.emit(c);
    if (best_index < count) manifold.emit(candidates[best_index]);
}

/// The edge of `box` along `axis` that lies furthest along `direction`.
fn supportEdge(box: *const BoxPose, axis: usize, direction: Vec3) EdgeSegment {
    var center = box.center;
    for (0..3) |k| {
        if (k == axis) continue;
        const a = axisOf(box.basis, k);
        const sign: f32 = if (a.dot(direction) >= 0.0) 1.0 else -1.0;
        center = center.add(a.scale(sign * extentOf(box.half_extents, k)));
    }
    return .{ .center = center, .direction = axisOf(box.basis, axis), .half_length = extentOf(box.half_extents, axis) };
}

/// Closest points between two edge segments.
fn closestPoints(a: EdgeSegment, b: EdgeSegment, pa: *Vec3, pb: *Vec3) void {
    const r = a.center.sub(b.center);
    const dd = a.direction.dot(b.direction);
    const ra = a.direction.dot(r);
    const rb = b.direction.dot(r);
    const denominator = 1.0 - dd * dd;
    var s: f32 = 0.0;
    if (denominator > 1e-6) s = std.math.clamp((dd * rb - ra) / denominator, -a.half_length, a.half_length);
    const t = std.math.clamp(dd * s + rb, -b.half_length, b.half_length);
    s = std.math.clamp(dd * t - ra, -a.half_length, a.half_length);
    pa.* = a.center.add(a.direction.scale(s));
    pb.* = b.center.add(b.direction.scale(t));
}

/// Carry impulses over from points whose feature id persisted.
fn warmStart(previous: *const Manifold, manifold: *Manifold) void {
    for (manifold.points[0..manifold.point_count]) |*point| {
        for (previous.points[0..previous.point_count]) |old| {
            if (old.feature_id == point.feature_id) {
                point.normal_impulse = old.normal_impulse;
                point.persisted = true;
                break;
            }
        }
    }
}

/// Rebuild the manifold between two boxes, reusing last frame's separating
/// axis as an early out and its points for warm starting.
fn collideBoxes(a: *const BoxPose, b: *const BoxPose, manifold: *Manifold) void {
    const previous = manifold.*;
    manifold.point_count = 0;
    manifold.points = @splat(.{});

    if (previous.separating_axis.dot(previous.separating_axis) > 0.5) {
        if (separationAlong(a, b, previous.separating_axis) > speculative_distance) {
            manifold.normal = previous.separating_axis;
            return;
        }
    }

    const d = b.center.sub(a.center);
    var best_face: usize = 0;
    var best_face_separation: f32 = -float_max;
    for (0..6) |i| {
        const n = if (i < 3) axisOf(a.basis, i) else axisOf(b.basis, i - 3);
        const s = separationAlong(a, b, n);
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

    var best_edge: ?[2]usize = null;
    var best_edge_axis: Vec3 = .{};
    var best_edge_separation: f32 = -float_max;
    for (0..3) |i| {
        for (0..3) |j| {
            var n = axisOf(a.basis, i).cross(axisOf(b.basis, j));
            const length_sq = n.dot(n);
            if (length_sq < parallel_edge_tolerance * parallel_edge_tolerance) continue;
            n = n.div(@sqrt(length_sq));
            const s = separationAlong(a, b, n);
            if (s > speculative_distance) {
                manifold.separating_axis = n;
                manifold.normal = n;
                return;
            }
            if (s > best_edge_separation) {
                best_edge_separation = s;
                best_edge = .{ i, j };
                best_edge_axis = if (n.dot(d) >= 0.0) n else n.neg();
            }
        }
    }

    var normal: Vec3 = undefined;
    if (best_edge != null and best_edge_separation > best_face_separation + linear_slop) {
        const edge_a, const edge_b = best_edge.?;
        normal = best_edge_axis;
        var pa: Vec3 = undefined;
        var pb: Vec3 = undefined;
        closestPoints(supportEdge(a, edge_a, normal), supportEdge(b, edge_b, normal.neg()), &pa, &pb);
        const point = &manifold.points[0];
        point.point = pa.add(pb).scale(0.5);
        point.separation = pb.sub(pa).dot(normal);
        point.feature_id = edge_contact_flag | (@as(u32, @intCast(edge_a)) << 8) | @as(u32, @intCast(edge_b));
        manifold.point_count = 1;
    } else {
        var candidates: Candidates = @splat(.{});
        var count: usize = 0;
        var reference_normal: Vec3 = undefined;
        if (best_face < 3) {
            const positive = axisOf(a.basis, best_face).dot(d) >= 0.0;
            const face: ReferenceFace = .{ .reference = a, .incident = b, .axis = best_face, .positive = positive, .flag = 0 };
            count = faceCandidates(face, speculative_distance, &candidates, &reference_normal);
            normal = reference_normal;
        } else {
            const axis = best_face - 3;
            const positive = axisOf(b.basis, axis).dot(d) <= 0.0;
            const face: ReferenceFace = .{ .reference = b, .incident = a, .axis = axis, .positive = positive, .flag = reference_b_flag };
            count = faceCandidates(face, speculative_distance, &candidates, &reference_normal);
            normal = reference_normal.neg();
        }
        if (count == 0) {
            manifold.separating_axis = normal;
            manifold.normal = normal;
            return;
        }
        reduceCandidates(&candidates, count, normal, speculative_distance, manifold);
    }

    manifold.normal = normal;
    manifold.separating_axis = normal;
    warmStart(&previous, manifold);
}

pub fn run(self: *BoxBox) u64 {
    @memset(self.manifolds, .{});
    for (0..frames) |f| {
        const t: f32 = @floatFromInt(f);
        for (self.pairs, self.manifolds) |*p, *manifold| {
            var b = p.b;
            b.center = b.center.add(p.velocity.scale(t));
            collideBoxes(&p.a, &b, manifold);
        }
    }
    var h: u64 = 0;
    for (self.manifolds) |*m| {
        h = hash.add(h, @as(u64, hash.f32Bits(m.normal.x)) | (@as(u64, hash.f32Bits(m.normal.z)) << 32));
        h = hash.add(h, m.point_count);
        for (m.points[0..m.point_count]) |p| {
            h = hash.add(h, @as(u64, hash.f32Bits(p.point.y)) | (@as(u64, hash.f32Bits(p.separation)) << 32));
            h = hash.add(h, @as(u64, p.feature_id) | (@as(u64, @intFromBool(p.persisted)) << 32));
        }
    }
    return h;
}
