//! The voxel engine's math types, with the engine's operation order: the
//! vectors, quaternions, bases, transforms and matrices of math.hpp and
//! math.cpp.
use crate::hash::Rng;
use std::ops::{Add, Div, Mul, Neg, Sub};

#[derive(Clone, Copy, Default)]
pub struct Vec3 {
    pub x: f32,
    pub y: f32,
    pub z: f32,
}

impl Vec3 {
    /// A vector from its components.
    pub const fn new(x: f32, y: f32, z: f32) -> Vec3 {
        Vec3 { x, y, z }
    }

    /// Uniform in [lo, hi) on each axis, drawn x, then y, then z.
    pub fn random(rng: &mut Rng, lo: f32, hi: f32) -> Vec3 {
        let x = lo + rng.unit() * (hi - lo);
        let y = lo + rng.unit() * (hi - lo);
        let z = lo + rng.unit() * (hi - lo);
        Vec3 { x, y, z }
    }

    /// Dot product.
    pub fn dot(self, b: Vec3) -> f32 {
        self.x * b.x + self.y * b.y + self.z * b.z
    }

    /// Cross product.
    pub fn cross(self, b: Vec3) -> Vec3 {
        Vec3::new(self.y * b.z - self.z * b.y, self.z * b.x - self.x * b.z, self.x * b.y - self.y * b.x)
    }

    /// Euclidean length.
    pub fn length(self) -> f32 {
        self.dot(self).sqrt()
    }

    /// Scaled to unit length; a zero vector is not guarded.
    pub fn normalize(self) -> Vec3 {
        self / self.length()
    }

    /// Component-wise minimum.
    pub fn min(self, b: Vec3) -> Vec3 {
        Vec3::new(if self.x < b.x { self.x } else { b.x }, if self.y < b.y { self.y } else { b.y }, if self.z < b.z { self.z } else { b.z })
    }

    /// Component-wise maximum.
    pub fn max(self, b: Vec3) -> Vec3 {
        Vec3::new(if self.x > b.x { self.x } else { b.x }, if self.y > b.y { self.y } else { b.y }, if self.z > b.z { self.z } else { b.z })
    }

    /// Component-wise absolute value, keeping the sign of a negative zero.
    pub fn abs(self) -> Vec3 {
        Vec3::new(if self.x < 0.0 { -self.x } else { self.x }, if self.y < 0.0 { -self.y } else { self.y }, if self.z < 0.0 { -self.z } else { self.z })
    }
}

/// The larger of two floats as `std::max` picks it: `a` unless `a < b`.
pub fn max(a: f32, b: f32) -> f32 {
    if a < b { b } else { a }
}

/// The smaller of two floats as `std::min` picks it: `a` unless `b < a`.
pub fn min(a: f32, b: f32) -> f32 {
    if b < a { b } else { a }
}

/// The largest of three floats, the first on a tie, as `std::max` over a list.
pub fn max3(a: f32, b: f32, c: f32) -> f32 {
    max(max(a, b), c)
}

/// `value` limited to [lo, hi] as `std::clamp` does it.
pub fn clamp(value: f32, lo: f32, hi: f32) -> f32 {
    if value < lo {
        lo
    } else if hi < value {
        hi
    } else {
        value
    }
}

impl Add for Vec3 {
    type Output = Vec3;

    /// Component-wise sum.
    fn add(self, b: Vec3) -> Vec3 {
        Vec3::new(self.x + b.x, self.y + b.y, self.z + b.z)
    }
}

impl Sub for Vec3 {
    type Output = Vec3;

    /// Component-wise difference.
    fn sub(self, b: Vec3) -> Vec3 {
        Vec3::new(self.x - b.x, self.y - b.y, self.z - b.z)
    }
}

impl Neg for Vec3 {
    type Output = Vec3;

    /// Negation.
    fn neg(self) -> Vec3 {
        Vec3::new(-self.x, -self.y, -self.z)
    }
}

impl Mul<f32> for Vec3 {
    type Output = Vec3;

    /// Scale by a scalar.
    fn mul(self, s: f32) -> Vec3 {
        Vec3::new(self.x * s, self.y * s, self.z * s)
    }
}

impl Div<f32> for Vec3 {
    type Output = Vec3;

    /// Divide by a scalar.
    fn div(self, s: f32) -> Vec3 {
        Vec3::new(self.x / s, self.y / s, self.z / s)
    }
}

/// Three column vectors.
#[derive(Clone, Copy)]
pub struct Basis {
    pub x: Vec3,
    pub y: Vec3,
    pub z: Vec3,
}

impl Default for Basis {
    /// The identity.
    fn default() -> Basis {
        Basis { x: Vec3::new(1.0, 0.0, 0.0), y: Vec3::new(0.0, 1.0, 0.0), z: Vec3::new(0.0, 0.0, 1.0) }
    }
}

impl Basis {
    /// Swap rows and columns.
    pub fn transposed(&self) -> Basis {
        Basis {
            x: Vec3::new(self.x.x, self.y.x, self.z.x),
            y: Vec3::new(self.x.y, self.y.y, self.z.y),
            z: Vec3::new(self.x.z, self.y.z, self.z.z),
        }
    }

    /// Determinant.
    pub fn determinant(&self) -> f32 {
        self.x.dot(self.y.cross(self.z))
    }

    /// Inverse, or the identity if the basis is singular.
    pub fn inverse(&self) -> Basis {
        let r0 = self.y.cross(self.z);
        let r1 = self.z.cross(self.x);
        let r2 = self.x.cross(self.y);
        let det = self.x.dot(r0);
        if det == 0.0 {
            return Basis::default();
        }
        let scale = 1.0 / det;
        Basis { x: r0 * scale, y: r1 * scale, z: r2 * scale }.transposed()
    }

    /// Rotation basis of a unit quaternion.
    pub fn from_quat(q: Quat) -> Basis {
        let (xx, yy, zz) = (q.x * q.x, q.y * q.y, q.z * q.z);
        let (xy, xz, yz) = (q.x * q.y, q.x * q.z, q.y * q.z);
        let (wx, wy, wz) = (q.w * q.x, q.w * q.y, q.w * q.z);
        Basis {
            x: Vec3::new(1.0 - 2.0 * (yy + zz), 2.0 * (xy + wz), 2.0 * (xz - wy)),
            y: Vec3::new(2.0 * (xy - wz), 1.0 - 2.0 * (xx + zz), 2.0 * (yz + wx)),
            z: Vec3::new(2.0 * (xz + wy), 2.0 * (yz - wx), 1.0 - 2.0 * (xx + yy)),
        }
    }

    /// Rotation basis with per-axis scale applied to its columns.
    pub fn from_rotation_scale(rotation: Quat, scale: Vec3) -> Basis {
        let b = Basis::from_quat(rotation);
        Basis { x: b.x * scale.x, y: b.y * scale.y, z: b.z * scale.z }
    }

    /// Column lengths, with x negated when the basis is mirrored.
    pub fn scale(&self) -> Vec3 {
        let scale = Vec3::new(self.x.length(), self.y.length(), self.z.length());
        if self.determinant() < 0.0 { Vec3::new(-scale.x, scale.y, scale.z) } else { scale }
    }

    /// Rotation with the scale removed.
    pub fn rotation(&self) -> Quat {
        let scale = self.scale();
        Quat::from_basis(&Basis {
            x: if scale.x != 0.0 { self.x / scale.x } else { Vec3::new(1.0, 0.0, 0.0) },
            y: if scale.y != 0.0 { self.y / scale.y } else { Vec3::new(0.0, 1.0, 0.0) },
            z: if scale.z != 0.0 { self.z / scale.z } else { Vec3::new(0.0, 0.0, 1.0) },
        })
    }
}

impl Add for Basis {
    type Output = Basis;

    /// Column-wise sum.
    fn add(self, b: Basis) -> Basis {
        Basis { x: self.x + b.x, y: self.y + b.y, z: self.z + b.z }
    }
}

impl Mul<f32> for Basis {
    type Output = Basis;

    /// Every column scaled.
    fn mul(self, s: f32) -> Basis {
        Basis { x: self.x * s, y: self.y * s, z: self.z * s }
    }
}

impl Mul<Vec3> for Basis {
    type Output = Vec3;

    /// Transform a vector by the basis.
    fn mul(self, v: Vec3) -> Vec3 {
        self.x * v.x + self.y * v.y + self.z * v.z
    }
}

impl Mul for Basis {
    type Output = Basis;

    /// Compose two bases; the result applies `b` first, then `self`.
    fn mul(self, b: Basis) -> Basis {
        Basis { x: self * b.x, y: self * b.y, z: self * b.z }
    }
}

#[derive(Clone, Copy)]
pub struct Quat {
    pub x: f32,
    pub y: f32,
    pub z: f32,
    pub w: f32,
}

impl Default for Quat {
    /// The identity rotation.
    fn default() -> Quat {
        Quat { x: 0.0, y: 0.0, z: 0.0, w: 1.0 }
    }
}

impl Quat {
    /// Four-component dot product.
    pub fn dot(self, b: Quat) -> f32 {
        self.x * b.x + self.y * b.y + self.z * b.z + self.w * b.w
    }

    /// Scaled to unit length, or the identity if its length is zero.
    pub fn normalize(self) -> Quat {
        let len = self.dot(self).sqrt();
        if len <= 0.0 {
            return Quat::default();
        }
        let inv = 1.0 / len;
        Quat { x: self.x * inv, y: self.y * inv, z: self.z * inv, w: self.w * inv }
    }

    /// Normalised quaternion of four components uniform in [-1, 1).
    pub fn random(rng: &mut Rng) -> Quat {
        let x = rng.unit() * 2.0 - 1.0;
        let y = rng.unit() * 2.0 - 1.0;
        let z = rng.unit() * 2.0 - 1.0;
        let w = rng.unit() * 2.0 - 1.0;
        Quat { x, y, z, w }.normalize()
    }

    /// Rotate a vector by a unit quaternion.
    pub fn rotate(self, v: Vec3) -> Vec3 {
        let u = Vec3::new(self.x, self.y, self.z);
        let t = u.cross(v) * 2.0;
        v + t * self.w + u.cross(t)
    }

    /// Unit quaternion of a rotation basis.
    pub fn from_basis(b: &Basis) -> Quat {
        let trace = b.x.x + b.y.y + b.z.z;
        let q = if trace > 0.0 {
            let s = (trace + 1.0).sqrt() * 2.0;
            Quat { w: 0.25 * s, x: (b.y.z - b.z.y) / s, y: (b.z.x - b.x.z) / s, z: (b.x.y - b.y.x) / s }
        } else if b.x.x > b.y.y && b.x.x > b.z.z {
            let s = (1.0 + b.x.x - b.y.y - b.z.z).sqrt() * 2.0;
            Quat { w: (b.y.z - b.z.y) / s, x: 0.25 * s, y: (b.y.x + b.x.y) / s, z: (b.z.x + b.x.z) / s }
        } else if b.y.y > b.z.z {
            let s = (1.0 + b.y.y - b.x.x - b.z.z).sqrt() * 2.0;
            Quat { w: (b.z.x - b.x.z) / s, x: (b.y.x + b.x.y) / s, y: 0.25 * s, z: (b.z.y + b.y.z) / s }
        } else {
            let s = (1.0 + b.z.z - b.x.x - b.y.y).sqrt() * 2.0;
            Quat { w: (b.x.y - b.y.x) / s, x: (b.z.x + b.x.z) / s, y: (b.z.y + b.y.z) / s, z: 0.25 * s }
        };
        q.normalize()
    }

    /// Advance a rotation by a world-space axis-angle displacement.
    pub fn integrate(self, angular_delta: Vec3) -> Quat {
        let omega = Quat { x: angular_delta.x, y: angular_delta.y, z: angular_delta.z, w: 0.0 };
        let d = omega * self;
        Quat { x: self.x + 0.5 * d.x, y: self.y + 0.5 * d.y, z: self.z + 0.5 * d.z, w: self.w + 0.5 * d.w }.normalize()
    }
}

impl Mul for Quat {
    type Output = Quat;

    /// Hamilton product; the result applies `b` first, then `self`.
    fn mul(self, b: Quat) -> Quat {
        let a = self;
        Quat {
            x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
        }
    }
}

#[derive(Clone, Copy, Default)]
pub struct Transform {
    pub basis: Basis,
    pub origin: Vec3,
}

impl Transform {
    /// Transform at `eye` whose negative z axis faces `target`.
    pub fn looking_at(eye: Vec3, target: Vec3, up: Vec3) -> Transform {
        let back = (eye - target).normalize();
        let right = up.cross(eye - target).normalize();
        let true_up = back.cross(right);
        Transform { basis: Basis { x: right, y: true_up, z: back }, origin: eye }
    }

    /// A point moved by the transform.
    pub fn transform_point(&self, p: Vec3) -> Vec3 {
        self.basis * p + self.origin
    }

    /// Inverse of a transform with an orthonormal basis.
    pub fn inverse_orthonormal(&self) -> Transform {
        let inv = self.basis.transposed();
        Transform { basis: inv, origin: -(inv * self.origin) }
    }

    /// Column-major matrix of the transform.
    pub fn to_mat4(&self) -> Mat4 {
        let mut r = Mat4::identity();
        r.m[0] = self.basis.x.x;
        r.m[1] = self.basis.x.y;
        r.m[2] = self.basis.x.z;
        r.m[4] = self.basis.y.x;
        r.m[5] = self.basis.y.y;
        r.m[6] = self.basis.y.z;
        r.m[8] = self.basis.z.x;
        r.m[9] = self.basis.z.y;
        r.m[10] = self.basis.z.z;
        r.m[12] = self.origin.x;
        r.m[13] = self.origin.y;
        r.m[14] = self.origin.z;
        r
    }
}

impl Mul for Transform {
    type Output = Transform;

    /// Compose two transforms; the result applies `b` first, then `self`.
    fn mul(self, b: Transform) -> Transform {
        Transform { basis: self.basis * b.basis, origin: self.basis * b.origin + self.origin }
    }
}

/// Axis-aligned box by its corners.
#[derive(Clone, Copy, Default)]
pub struct Aabb {
    pub min: Vec3,
    pub max: Vec3,
}

impl Aabb {
    /// Whether two boxes overlap, touching included.
    pub fn overlaps(&self, b: &Aabb) -> bool {
        self.min.x <= b.max.x && self.max.x >= b.min.x && self.min.y <= b.max.y && self.max.y >= b.min.y && self.min.z <= b.max.z && self.max.z >= b.min.z
    }

    /// Whether this box fully contains `inner`.
    pub fn contains(&self, inner: &Aabb) -> bool {
        self.min.x <= inner.min.x
            && self.min.y <= inner.min.y
            && self.min.z <= inner.min.z
            && self.max.x >= inner.max.x
            && self.max.y >= inner.max.y
            && self.max.z >= inner.max.z
    }

    /// Smallest box containing both.
    pub fn merge(&self, b: &Aabb) -> Aabb {
        Aabb { min: self.min.min(b.min), max: self.max.max(b.max) }
    }

    /// The box expanded by `margin` on every side.
    pub fn grow(&self, margin: f32) -> Aabb {
        let m = Vec3::new(margin, margin, margin);
        Aabb { min: self.min - m, max: self.max + m }
    }

    /// Total surface area.
    pub fn surface_area(&self) -> f32 {
        let d = self.max - self.min;
        2.0 * (d.x * d.y + d.y * d.z + d.z * d.x)
    }
}

/// Oriented box: half extents along the basis columns, around a centre.
#[derive(Clone, Copy, Default)]
pub struct BoxPose {
    pub half_extents: Vec3,
    pub center: Vec3,
    pub basis: Basis,
}

/// Column-major 4x4 matrix.
#[derive(Clone, Copy, Default)]
pub struct Mat4 {
    pub m: [f32; 16],
}

impl Mat4 {
    /// Identity matrix.
    pub fn identity() -> Mat4 {
        let mut r = Mat4::default();
        r.m[0] = 1.0;
        r.m[5] = 1.0;
        r.m[10] = 1.0;
        r.m[15] = 1.0;
        r
    }

    /// Perspective projection from the tangent of half the vertical field of
    /// view: the engine's formula with the tangent precomputed, since `tan`
    /// differs in the last bit between C libraries.
    pub fn perspective(tan_half_fov: f32, aspect: f32, z_near: f32, z_far: f32) -> Mat4 {
        let depth = z_far - z_near;
        let mut m = Mat4::default();
        m.m[0] = 1.0 / (tan_half_fov * aspect);
        m.m[5] = 1.0 / tan_half_fov;
        m.m[10] = -(z_far + z_near) / depth;
        m.m[11] = -1.0;
        m.m[14] = -2.0 * z_far * z_near / depth;
        m
    }

    /// Cofactor inverse, or the identity if the matrix is singular.
    pub fn inverse(&self) -> Mat4 {
        let m = &self.m;
        let mut inv = [0.0f32; 16];
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
        let det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
        if det == 0.0 {
            return Mat4::identity();
        }
        let scale = 1.0 / det;
        let mut r = Mat4::default();
        for (out, v) in r.m.iter_mut().zip(inv.iter()) {
            *out = v * scale;
        }
        r
    }

    /// Transform a point and divide by the resulting w.
    pub fn transform_point(&self, p: Vec3) -> Vec3 {
        let m = &self.m;
        let x = m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12];
        let y = m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13];
        let z = m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14];
        let w = m[3] * p.x + m[7] * p.y + m[11] * p.z + m[15];
        Vec3::new(x, y, z) / w
    }
}

impl Mul for Mat4 {
    type Output = Mat4;

    /// Matrix product; the result applies `b` first, then `self`.
    fn mul(self, b: Mat4) -> Mat4 {
        let mut r = Mat4::default();
        for column in 0..4 {
            for row in 0..4 {
                let mut sum = 0.0f32;
                for k in 0..4 {
                    sum += self.m[k * 4 + row] * b.m[column * 4 + k];
                }
                r.m[column * 4 + row] = sum;
            }
        }
        r
    }
}
