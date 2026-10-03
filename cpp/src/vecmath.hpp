#pragma once

#include <array>
#include <cmath>

#include "hash.hpp"

/// The voxel engine's math types, with the engine's operation order. Small
/// operations are inline here; the larger ones live in vecmath.cpp, the way
/// the engine splits math.hpp and math.cpp.
namespace bench::vm {

struct Vec3 {
    float x{};
    float y{};
    float z{};
};

/// Component-wise sum.
constexpr Vec3 operator+(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
/// Component-wise difference.
constexpr Vec3 operator-(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
/// Negation.
constexpr Vec3 operator-(Vec3 a) { return {-a.x, -a.y, -a.z}; }
/// Scale by a scalar.
constexpr Vec3 operator*(Vec3 a, float s) { return {a.x * s, a.y * s, a.z * s}; }
/// Divide by a scalar.
constexpr Vec3 operator/(Vec3 a, float s) { return {a.x / s, a.y / s, a.z / s}; }
/// Dot product.
constexpr float dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

/// Cross product.
constexpr Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

/// Euclidean length.
inline float length(Vec3 a) { return std::sqrt(dot(a, a)); }
/// Scaled to unit length; a zero vector is not guarded.
inline Vec3 normalize(Vec3 a) { return a / length(a); }

/// Three column vectors.
struct Basis {
    Vec3 x{1.0f, 0.0f, 0.0f};
    Vec3 y{0.0f, 1.0f, 0.0f};
    Vec3 z{0.0f, 0.0f, 1.0f};
};

/// Transform a vector by a basis.
constexpr Vec3 operator*(const Basis& b, Vec3 v) { return b.x * v.x + b.y * v.y + b.z * v.z; }
/// Every column scaled.
constexpr Basis operator*(const Basis& a, float s) { return {a.x * s, a.y * s, a.z * s}; }
/// Column-wise sum.
constexpr Basis operator+(const Basis& a, const Basis& b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
/// Compose two bases; the result applies `b` first, then `a`.
constexpr Basis operator*(const Basis& a, const Basis& b) { return {a * b.x, a * b.y, a * b.z}; }

/// Swap rows and columns.
constexpr Basis transposed(const Basis& b) {
    return {{b.x.x, b.y.x, b.z.x}, {b.x.y, b.y.y, b.z.y}, {b.x.z, b.y.z, b.z.z}};
}

/// Determinant.
constexpr float determinant(const Basis& b) { return dot(b.x, cross(b.y, b.z)); }

struct Quat {
    float x{};
    float y{};
    float z{};
    float w{1.0f};
};

/// Hamilton product; the result applies `b` first, then `a`.
constexpr Quat operator*(Quat a, Quat b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
            a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
            a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}

/// Four-component dot product.
constexpr float dot(Quat a, Quat b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

struct Transform {
    Basis basis{};
    Vec3 origin{};
};

/// Compose two transforms; the result applies `b` first, then `a`.
constexpr Transform operator*(const Transform& a, const Transform& b) {
    return {a.basis * b.basis, a.basis * b.origin + a.origin};
}

/// Component-wise minimum.
constexpr Vec3 min(Vec3 a, Vec3 b) { return {a.x < b.x ? a.x : b.x, a.y < b.y ? a.y : b.y, a.z < b.z ? a.z : b.z}; }
/// Component-wise maximum.
constexpr Vec3 max(Vec3 a, Vec3 b) { return {a.x > b.x ? a.x : b.x, a.y > b.y ? a.y : b.y, a.z > b.z ? a.z : b.z}; }
/// Component-wise absolute value.
constexpr Vec3 abs(Vec3 a) { return {a.x < 0.0f ? -a.x : a.x, a.y < 0.0f ? -a.y : a.y, a.z < 0.0f ? -a.z : a.z}; }

/// Axis-aligned box by its corners.
struct Aabb {
    Vec3 min{};
    Vec3 max{};
};

/// Whether two boxes overlap, touching included.
constexpr bool overlaps(const Aabb& a, const Aabb& b) {
    return a.min.x <= b.max.x && a.max.x >= b.min.x && a.min.y <= b.max.y && a.max.y >= b.min.y && a.min.z <= b.max.z &&
           a.max.z >= b.min.z;
}

/// Whether `outer` fully contains `inner`.
constexpr bool contains(const Aabb& outer, const Aabb& inner) {
    return outer.min.x <= inner.min.x && outer.min.y <= inner.min.y && outer.min.z <= inner.min.z &&
           outer.max.x >= inner.max.x && outer.max.y >= inner.max.y && outer.max.z >= inner.max.z;
}

/// Smallest box containing both.
constexpr Aabb merge(const Aabb& a, const Aabb& b) { return {min(a.min, b.min), max(a.max, b.max)}; }

/// The box expanded by `margin` on every side.
constexpr Aabb grow(const Aabb& box, float margin) {
    const Vec3 m{margin, margin, margin};
    return {box.min - m, box.max + m};
}

/// Total surface area.
constexpr float surface_area(const Aabb& box) {
    const Vec3 d = box.max - box.min;
    return 2.0f * (d.x * d.y + d.y * d.z + d.z * d.x);
}

/// A point moved by a transform.
constexpr Vec3 transform_point(const Transform& t, Vec3 p) { return t.basis * p + t.origin; }

/// Oriented box: half extents along the basis columns, around a centre.
struct BoxPose {
    Vec3 half_extents{};
    Vec3 center{};
    Basis basis{};
};

/// Column-major 4x4 matrix.
struct Mat4 {
    std::array<float, 16> m{};
};

/// Uniform in [lo, hi) on each axis, drawn x, then y, then z.
inline Vec3 random_vec3(Rng& rng, float lo, float hi) {
    Vec3 v;
    v.x = lo + rng.unit() * (hi - lo);
    v.y = lo + rng.unit() * (hi - lo);
    v.z = lo + rng.unit() * (hi - lo);
    return v;
}

/// Inverse of a basis, or the identity if it is singular.
Basis inverse(const Basis& b);
/// Scaled to unit length, or the identity if its length is zero.
Quat normalize(Quat q);
/// Normalised quaternion of four components uniform in [-1, 1).
Quat random_quat(Rng& rng);
/// Rotate a vector by a unit quaternion.
Vec3 rotate(Quat q, Vec3 v);
/// Rotation basis of a unit quaternion.
Basis basis_from_quat(Quat q);
/// Unit quaternion of a rotation basis.
Quat quat_from_basis(const Basis& b);
/// Column lengths, with x negated when the basis is mirrored.
Vec3 basis_scale(const Basis& b);
/// Rotation of a basis with its scale removed.
Quat basis_rotation(const Basis& b);
/// Rotation basis with per-axis scale applied to its columns.
Basis basis_from_rotation_scale(Quat rotation, Vec3 scale);
/// Advance a rotation by a world-space axis-angle displacement.
Quat integrate_rotation(Quat q, Vec3 angular_delta);
/// Transform at `eye` whose negative z axis faces `target`.
Transform looking_at(Vec3 eye, Vec3 target, Vec3 up);
/// Inverse of a transform with an orthonormal basis.
Transform inverse_orthonormal(const Transform& t);
/// Column-major matrix of a transform.
Mat4 to_mat4(const Transform& t);
/// Perspective projection from the tangent of half the vertical field of
/// view: the engine's formula with the tangent precomputed, since `tan`
/// differs in the last bit between C libraries.
Mat4 perspective(float tan_half_fov, float aspect, float z_near, float z_far);
/// Identity matrix.
Mat4 identity();
/// Matrix product; the result applies `b` first, then `a`.
Mat4 operator*(const Mat4& a, const Mat4& b);
/// Cofactor inverse, or the identity if the matrix is singular.
Mat4 inverse(const Mat4& matrix);
/// Transform a point and divide by the resulting w.
Vec3 transform_point(const Mat4& matrix, Vec3 p);

} // namespace bench::vm
