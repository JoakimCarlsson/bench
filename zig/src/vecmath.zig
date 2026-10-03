//! The voxel engine's math types, with the engine's operation order: the
//! vectors, quaternions, bases, transforms and matrices of math.hpp and
//! math.cpp, as methods since Zig has no operator overloading.
const hash = @import("hash.zig");

pub const Vec3 = struct {
    x: f32 = 0.0,
    y: f32 = 0.0,
    z: f32 = 0.0,

    /// A vector from its components.
    pub fn init(x: f32, y: f32, z: f32) Vec3 {
        return .{ .x = x, .y = y, .z = z };
    }

    /// Uniform in [lo, hi) on each axis, drawn x, then y, then z.
    pub fn random(rng: *hash.Rng, lo: f32, hi: f32) Vec3 {
        const x = lo + rng.unit() * (hi - lo);
        const y = lo + rng.unit() * (hi - lo);
        const z = lo + rng.unit() * (hi - lo);
        return .{ .x = x, .y = y, .z = z };
    }

    /// Component-wise sum.
    pub fn add(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = a.x + b.x, .y = a.y + b.y, .z = a.z + b.z };
    }

    /// Component-wise difference.
    pub fn sub(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = a.x - b.x, .y = a.y - b.y, .z = a.z - b.z };
    }

    /// Negation.
    pub fn neg(a: Vec3) Vec3 {
        return .{ .x = -a.x, .y = -a.y, .z = -a.z };
    }

    /// Scale by a scalar.
    pub fn scale(a: Vec3, s: f32) Vec3 {
        return .{ .x = a.x * s, .y = a.y * s, .z = a.z * s };
    }

    /// Divide by a scalar.
    pub fn div(a: Vec3, s: f32) Vec3 {
        return .{ .x = a.x / s, .y = a.y / s, .z = a.z / s };
    }

    /// Dot product.
    pub fn dot(a: Vec3, b: Vec3) f32 {
        return a.x * b.x + a.y * b.y + a.z * b.z;
    }

    /// Cross product.
    pub fn cross(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = a.y * b.z - a.z * b.y, .y = a.z * b.x - a.x * b.z, .z = a.x * b.y - a.y * b.x };
    }

    /// Euclidean length.
    pub fn length(a: Vec3) f32 {
        return @sqrt(a.dot(a));
    }

    /// Scaled to unit length; a zero vector is not guarded.
    pub fn normalize(a: Vec3) Vec3 {
        return a.div(a.length());
    }

    /// Component-wise minimum.
    pub fn min(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = if (a.x < b.x) a.x else b.x, .y = if (a.y < b.y) a.y else b.y, .z = if (a.z < b.z) a.z else b.z };
    }

    /// Component-wise maximum.
    pub fn max(a: Vec3, b: Vec3) Vec3 {
        return .{ .x = if (a.x > b.x) a.x else b.x, .y = if (a.y > b.y) a.y else b.y, .z = if (a.z > b.z) a.z else b.z };
    }

    /// Component-wise absolute value.
    pub fn abs(a: Vec3) Vec3 {
        return .{ .x = if (a.x < 0.0) -a.x else a.x, .y = if (a.y < 0.0) -a.y else a.y, .z = if (a.z < 0.0) -a.z else a.z };
    }
};

/// Three column vectors; the identity by default.
pub const Basis = struct {
    x: Vec3 = .{ .x = 1.0 },
    y: Vec3 = .{ .y = 1.0 },
    z: Vec3 = .{ .z = 1.0 },

    /// Transform a vector by the basis.
    pub fn apply(b: Basis, v: Vec3) Vec3 {
        return b.x.scale(v.x).add(b.y.scale(v.y)).add(b.z.scale(v.z));
    }

    /// Compose two bases; the result applies `b` first, then `a`.
    pub fn mul(a: Basis, b: Basis) Basis {
        return .{ .x = a.apply(b.x), .y = a.apply(b.y), .z = a.apply(b.z) };
    }

    /// Column-wise sum.
    pub fn add(a: Basis, b: Basis) Basis {
        return .{ .x = a.x.add(b.x), .y = a.y.add(b.y), .z = a.z.add(b.z) };
    }

    /// Every column scaled.
    pub fn scale(a: Basis, s: f32) Basis {
        return .{ .x = a.x.scale(s), .y = a.y.scale(s), .z = a.z.scale(s) };
    }

    /// Swap rows and columns.
    pub fn transposed(b: Basis) Basis {
        return .{
            .x = Vec3.init(b.x.x, b.y.x, b.z.x),
            .y = Vec3.init(b.x.y, b.y.y, b.z.y),
            .z = Vec3.init(b.x.z, b.y.z, b.z.z),
        };
    }

    /// Determinant.
    pub fn determinant(b: Basis) f32 {
        return b.x.dot(b.y.cross(b.z));
    }

    /// Inverse, or the identity if the basis is singular.
    pub fn inverse(b: Basis) Basis {
        const r0 = b.y.cross(b.z);
        const r1 = b.z.cross(b.x);
        const r2 = b.x.cross(b.y);
        const det = b.x.dot(r0);
        if (det == 0.0) return .{};
        const s = 1.0 / det;
        const rows: Basis = .{ .x = r0.scale(s), .y = r1.scale(s), .z = r2.scale(s) };
        return rows.transposed();
    }

    /// Rotation basis of a unit quaternion.
    pub fn fromQuat(q: Quat) Basis {
        const xx = q.x * q.x;
        const yy = q.y * q.y;
        const zz = q.z * q.z;
        const xy = q.x * q.y;
        const xz = q.x * q.z;
        const yz = q.y * q.z;
        const wx = q.w * q.x;
        const wy = q.w * q.y;
        const wz = q.w * q.z;
        return .{
            .x = Vec3.init(1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy)),
            .y = Vec3.init(2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx)),
            .z = Vec3.init(2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)),
        };
    }

    /// Rotation basis with per-axis scale applied to its columns.
    pub fn fromRotationScale(rotation: Quat, s: Vec3) Basis {
        const b = fromQuat(rotation);
        return .{ .x = b.x.scale(s.x), .y = b.y.scale(s.y), .z = b.z.scale(s.z) };
    }

    /// Column lengths, with x negated when the basis is mirrored.
    pub fn scaleOf(b: Basis) Vec3 {
        const s = Vec3.init(b.x.length(), b.y.length(), b.z.length());
        return if (b.determinant() < 0.0) Vec3.init(-s.x, s.y, s.z) else s;
    }

    /// Rotation with the scale removed.
    pub fn rotationOf(b: Basis) Quat {
        const s = b.scaleOf();
        const unit: Basis = .{
            .x = if (s.x != 0.0) b.x.div(s.x) else Vec3.init(1.0, 0.0, 0.0),
            .y = if (s.y != 0.0) b.y.div(s.y) else Vec3.init(0.0, 1.0, 0.0),
            .z = if (s.z != 0.0) b.z.div(s.z) else Vec3.init(0.0, 0.0, 1.0),
        };
        return Quat.fromBasis(unit);
    }
};

/// Rotation quaternion; the identity by default.
pub const Quat = struct {
    x: f32 = 0.0,
    y: f32 = 0.0,
    z: f32 = 0.0,
    w: f32 = 1.0,

    /// Hamilton product; the result applies `b` first, then `a`.
    pub fn mul(a: Quat, b: Quat) Quat {
        return .{
            .x = a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            .y = a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            .z = a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            .w = a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        };
    }

    /// Four-component dot product.
    pub fn dot(a: Quat, b: Quat) f32 {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }

    /// Scaled to unit length, or the identity if its length is zero.
    pub fn normalize(q: Quat) Quat {
        const len = @sqrt(q.dot(q));
        if (len <= 0.0) return .{};
        const inv = 1.0 / len;
        return .{ .x = q.x * inv, .y = q.y * inv, .z = q.z * inv, .w = q.w * inv };
    }

    /// Normalised quaternion of four components uniform in [-1, 1).
    pub fn random(rng: *hash.Rng) Quat {
        const x = rng.unit() * 2.0 - 1.0;
        const y = rng.unit() * 2.0 - 1.0;
        const z = rng.unit() * 2.0 - 1.0;
        const w = rng.unit() * 2.0 - 1.0;
        const q: Quat = .{ .x = x, .y = y, .z = z, .w = w };
        return q.normalize();
    }

    /// Rotate a vector by a unit quaternion.
    pub fn rotate(q: Quat, v: Vec3) Vec3 {
        const u = Vec3.init(q.x, q.y, q.z);
        const t = u.cross(v).scale(2.0);
        return v.add(t.scale(q.w)).add(u.cross(t));
    }

    /// Unit quaternion of a rotation basis.
    pub fn fromBasis(b: Basis) Quat {
        const trace = b.x.x + b.y.y + b.z.z;
        var q: Quat = .{};
        if (trace > 0.0) {
            const s = @sqrt(trace + 1.0) * 2.0;
            q.w = 0.25 * s;
            q.x = (b.y.z - b.z.y) / s;
            q.y = (b.z.x - b.x.z) / s;
            q.z = (b.x.y - b.y.x) / s;
        } else if (b.x.x > b.y.y and b.x.x > b.z.z) {
            const s = @sqrt(1.0 + b.x.x - b.y.y - b.z.z) * 2.0;
            q.w = (b.y.z - b.z.y) / s;
            q.x = 0.25 * s;
            q.y = (b.y.x + b.x.y) / s;
            q.z = (b.z.x + b.x.z) / s;
        } else if (b.y.y > b.z.z) {
            const s = @sqrt(1.0 + b.y.y - b.x.x - b.z.z) * 2.0;
            q.w = (b.z.x - b.x.z) / s;
            q.x = (b.y.x + b.x.y) / s;
            q.y = 0.25 * s;
            q.z = (b.z.y + b.y.z) / s;
        } else {
            const s = @sqrt(1.0 + b.z.z - b.x.x - b.y.y) * 2.0;
            q.w = (b.x.y - b.y.x) / s;
            q.x = (b.z.x + b.x.z) / s;
            q.y = (b.z.y + b.y.z) / s;
            q.z = 0.25 * s;
        }
        return q.normalize();
    }

    /// Advance a rotation by a world-space axis-angle displacement.
    pub fn integrate(q: Quat, angular_delta: Vec3) Quat {
        const omega: Quat = .{ .x = angular_delta.x, .y = angular_delta.y, .z = angular_delta.z, .w = 0.0 };
        const d = omega.mul(q);
        const next: Quat = .{ .x = q.x + 0.5 * d.x, .y = q.y + 0.5 * d.y, .z = q.z + 0.5 * d.z, .w = q.w + 0.5 * d.w };
        return next.normalize();
    }
};

pub const Transform = struct {
    basis: Basis = .{},
    origin: Vec3 = .{},

    /// Compose two transforms; the result applies `b` first, then `a`.
    pub fn mul(a: Transform, b: Transform) Transform {
        return .{ .basis = a.basis.mul(b.basis), .origin = a.basis.apply(b.origin).add(a.origin) };
    }

    /// A point moved by the transform.
    pub fn transformPoint(t: Transform, p: Vec3) Vec3 {
        return t.basis.apply(p).add(t.origin);
    }

    /// Transform at `eye` whose negative z axis faces `target`.
    pub fn lookingAt(eye: Vec3, target: Vec3, up: Vec3) Transform {
        const back = eye.sub(target).normalize();
        const right = up.cross(eye.sub(target)).normalize();
        const true_up = back.cross(right);
        return .{ .basis = .{ .x = right, .y = true_up, .z = back }, .origin = eye };
    }

    /// Inverse of a transform with an orthonormal basis.
    pub fn inverseOrthonormal(t: Transform) Transform {
        const inv = t.basis.transposed();
        return .{ .basis = inv, .origin = inv.apply(t.origin).neg() };
    }

    /// Column-major matrix of the transform.
    pub fn toMat4(t: Transform) Mat4 {
        var r = Mat4.identity;
        r.m[0] = t.basis.x.x;
        r.m[1] = t.basis.x.y;
        r.m[2] = t.basis.x.z;
        r.m[4] = t.basis.y.x;
        r.m[5] = t.basis.y.y;
        r.m[6] = t.basis.y.z;
        r.m[8] = t.basis.z.x;
        r.m[9] = t.basis.z.y;
        r.m[10] = t.basis.z.z;
        r.m[12] = t.origin.x;
        r.m[13] = t.origin.y;
        r.m[14] = t.origin.z;
        return r;
    }
};

/// Axis-aligned box by its corners.
pub const Aabb = struct {
    min: Vec3 = .{},
    max: Vec3 = .{},

    /// Whether two boxes overlap, touching included.
    pub fn overlaps(a: Aabb, b: Aabb) bool {
        return a.min.x <= b.max.x and a.max.x >= b.min.x and a.min.y <= b.max.y and a.max.y >= b.min.y and
            a.min.z <= b.max.z and a.max.z >= b.min.z;
    }

    /// Whether `outer` fully contains `inner`.
    pub fn contains(outer: Aabb, inner: Aabb) bool {
        return outer.min.x <= inner.min.x and outer.min.y <= inner.min.y and outer.min.z <= inner.min.z and
            outer.max.x >= inner.max.x and outer.max.y >= inner.max.y and outer.max.z >= inner.max.z;
    }

    /// Smallest box containing both.
    pub fn merge(a: Aabb, b: Aabb) Aabb {
        return .{ .min = a.min.min(b.min), .max = a.max.max(b.max) };
    }

    /// The box expanded by `margin` on every side.
    pub fn grow(box: Aabb, margin: f32) Aabb {
        const m = Vec3.init(margin, margin, margin);
        return .{ .min = box.min.sub(m), .max = box.max.add(m) };
    }

    /// Total surface area.
    pub fn surfaceArea(box: Aabb) f32 {
        const d = box.max.sub(box.min);
        return 2.0 * (d.x * d.y + d.y * d.z + d.z * d.x);
    }
};

/// Oriented box: half extents along the basis columns, around a centre.
pub const BoxPose = struct {
    half_extents: Vec3 = .{},
    center: Vec3 = .{},
    basis: Basis = .{},
};

/// Column-major 4x4 matrix.
pub const Mat4 = struct {
    m: [16]f32 = @splat(0.0),

    pub const identity: Mat4 = .{ .m = .{ 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0 } };

    /// Perspective projection from the tangent of half the vertical field of
    /// view: the engine's formula with the tangent precomputed, since `tan`
    /// differs in the last bit between C libraries.
    pub fn perspective(tan_half_fov: f32, aspect: f32, z_near: f32, z_far: f32) Mat4 {
        const depth = z_far - z_near;
        var r: Mat4 = .{};
        r.m[0] = 1.0 / (tan_half_fov * aspect);
        r.m[5] = 1.0 / tan_half_fov;
        r.m[10] = -(z_far + z_near) / depth;
        r.m[11] = -1.0;
        r.m[14] = -2.0 * z_far * z_near / depth;
        return r;
    }

    /// Matrix product; the result applies `b` first, then `a`.
    pub fn mul(a: Mat4, b: Mat4) Mat4 {
        var r: Mat4 = .{};
        for (0..4) |column| {
            for (0..4) |row| {
                var sum: f32 = 0.0;
                for (0..4) |k| sum += a.m[k * 4 + row] * b.m[column * 4 + k];
                r.m[column * 4 + row] = sum;
            }
        }
        return r;
    }

    /// Cofactor inverse, or the identity if the matrix is singular.
    pub fn inverse(self: Mat4) Mat4 {
        const m = &self.m;
        var inv: [16]f32 = undefined;
        inv[0] = m[5] * m[10] * m[15] - m[5] * m[11] * m[14] - m[9] * m[6] * m[15] + m[9] * m[7] * m[14] + m[13] * m[6] * m[11] - m[13] * m[7] * m[10];
        inv[4] = -m[4] * m[10] * m[15] + m[4] * m[11] * m[14] + m[8] * m[6] * m[15] - m[8] * m[7] * m[14] - m[12] * m[6] * m[11] + m[12] * m[7] * m[10];
        inv[8] = m[4] * m[9] * m[15] - m[4] * m[11] * m[13] - m[8] * m[5] * m[15] + m[8] * m[7] * m[13] + m[12] * m[5] * m[11] - m[12] * m[7] * m[9];
        inv[12] = -m[4] * m[9] * m[14] + m[4] * m[10] * m[13] + m[8] * m[5] * m[14] - m[8] * m[6] * m[13] - m[12] * m[5] * m[10] + m[12] * m[6] * m[9];
        inv[1] = -m[1] * m[10] * m[15] + m[1] * m[11] * m[14] + m[9] * m[2] * m[15] - m[9] * m[3] * m[14] - m[13] * m[2] * m[11] + m[13] * m[3] * m[10];
        inv[5] = m[0] * m[10] * m[15] - m[0] * m[11] * m[14] - m[8] * m[2] * m[15] + m[8] * m[3] * m[14] + m[12] * m[2] * m[11] - m[12] * m[3] * m[10];
        inv[9] = -m[0] * m[9] * m[15] + m[0] * m[11] * m[13] + m[8] * m[1] * m[15] - m[8] * m[3] * m[13] - m[12] * m[1] * m[11] + m[12] * m[3] * m[9];
        inv[13] = m[0] * m[9] * m[14] - m[0] * m[10] * m[13] - m[8] * m[1] * m[14] + m[8] * m[2] * m[13] + m[12] * m[1] * m[10] - m[12] * m[2] * m[9];
        inv[2] = m[1] * m[6] * m[15] - m[1] * m[7] * m[14] - m[5] * m[2] * m[15] + m[5] * m[3] * m[14] + m[13] * m[2] * m[7] - m[13] * m[3] * m[6];
        inv[6] = -m[0] * m[6] * m[15] + m[0] * m[7] * m[14] + m[4] * m[2] * m[15] - m[4] * m[3] * m[14] - m[12] * m[2] * m[7] + m[12] * m[3] * m[6];
        inv[10] = m[0] * m[5] * m[15] - m[0] * m[7] * m[13] - m[4] * m[1] * m[15] + m[4] * m[3] * m[13] + m[12] * m[1] * m[7] - m[12] * m[3] * m[5];
        inv[14] = -m[0] * m[5] * m[14] + m[0] * m[6] * m[13] + m[4] * m[1] * m[14] - m[4] * m[2] * m[13] - m[12] * m[1] * m[6] + m[12] * m[2] * m[5];
        inv[3] = -m[1] * m[6] * m[11] + m[1] * m[7] * m[10] + m[5] * m[2] * m[11] - m[5] * m[3] * m[10] - m[9] * m[2] * m[7] + m[9] * m[3] * m[6];
        inv[7] = m[0] * m[6] * m[11] - m[0] * m[7] * m[10] - m[4] * m[2] * m[11] + m[4] * m[3] * m[10] + m[8] * m[2] * m[7] - m[8] * m[3] * m[6];
        inv[11] = -m[0] * m[5] * m[11] + m[0] * m[7] * m[9] + m[4] * m[1] * m[11] - m[4] * m[3] * m[9] - m[8] * m[1] * m[7] + m[8] * m[3] * m[5];
        inv[15] = m[0] * m[5] * m[10] - m[0] * m[6] * m[9] - m[4] * m[1] * m[10] + m[4] * m[2] * m[9] + m[8] * m[1] * m[6] - m[8] * m[2] * m[5];
        const det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
        if (det == 0.0) return identity;
        const s = 1.0 / det;
        var r: Mat4 = .{};
        for (&r.m, inv) |*out, v| out.* = v * s;
        return r;
    }

    /// Transform a point and divide by the resulting w.
    pub fn transformPoint(self: Mat4, p: Vec3) Vec3 {
        const m = &self.m;
        const x = m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12];
        const y = m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13];
        const z = m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14];
        const w = m[3] * p.x + m[7] * p.y + m[11] * p.z + m[15];
        return Vec3.init(x, y, z).div(w);
    }
};

/// `std::max`: `b` when a < b, otherwise `a`.
pub fn maxf(a: f32, b: f32) f32 {
    return if (a < b) b else a;
}

/// `std::min`: `b` when b < a, otherwise `a`.
pub fn minf(a: f32, b: f32) f32 {
    return if (b < a) b else a;
}

/// `std::clamp`: `low` below it, `high` above it, otherwise `value`.
pub fn clampf(value: f32, low: f32, high: f32) f32 {
    if (value < low) return low;
    if (high < value) return high;
    return value;
}

/// `std::max` over three values, the first largest kept.
pub fn max3(a: f32, b: f32, c: f32) f32 {
    return maxf(maxf(a, b), c);
}
