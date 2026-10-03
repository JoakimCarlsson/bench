#ifndef BENCH_VECMATH_H
#define BENCH_VECMATH_H

#include <math.h>

#include "hash.h"

/// The voxel engine's math types, with the engine's operation order. Small
/// operations are inline here; the larger ones live in vecmath.c, the way the
/// engine splits math.hpp and math.cpp.

typedef struct { float x, y, z; } Vec3;
typedef struct { float x, y, z, w; } Quat;
/// Three column vectors.
typedef struct { Vec3 x, y, z; } Basis;
typedef struct { Basis basis; Vec3 origin; } Transform;
/// Oriented box: half extents along the basis columns, around a centre.
typedef struct { Vec3 half_extents; Vec3 center; Basis basis; } BoxPose;
/// Column-major 4x4 matrix.
typedef struct { float m[16]; } Mat4;

/// Component-wise sum.
static inline Vec3 v3_add(Vec3 a, Vec3 b) { return (Vec3){ a.x + b.x, a.y + b.y, a.z + b.z }; }
/// Component-wise difference.
static inline Vec3 v3_sub(Vec3 a, Vec3 b) { return (Vec3){ a.x - b.x, a.y - b.y, a.z - b.z }; }
/// Negation.
static inline Vec3 v3_neg(Vec3 a) { return (Vec3){ -a.x, -a.y, -a.z }; }
/// Scale by a scalar.
static inline Vec3 v3_scale(Vec3 a, float s) { return (Vec3){ a.x * s, a.y * s, a.z * s }; }
/// Divide by a scalar.
static inline Vec3 v3_div(Vec3 a, float s) { return (Vec3){ a.x / s, a.y / s, a.z / s }; }
/// Dot product.
static inline float v3_dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

/// Cross product.
static inline Vec3 v3_cross(Vec3 a, Vec3 b) {
    return (Vec3){ a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x };
}

/// Euclidean length.
static inline float v3_length(Vec3 a) { return sqrtf(v3_dot(a, a)); }
/// Scaled to unit length; a zero vector is not guarded.
static inline Vec3 v3_normalize(Vec3 a) { return v3_div(a, v3_length(a)); }

/// Transform a vector by a basis.
static inline Vec3 basis_apply(const Basis* b, Vec3 v) {
    return v3_add(v3_add(v3_scale(b->x, v.x), v3_scale(b->y, v.y)), v3_scale(b->z, v.z));
}

/// Swap rows and columns.
static inline Basis basis_transposed(const Basis* b) {
    return (Basis){ { b->x.x, b->y.x, b->z.x }, { b->x.y, b->y.y, b->z.y }, { b->x.z, b->y.z, b->z.z } };
}

/// Compose two bases; the result applies `b` first, then `a`.
static inline Basis basis_mul(const Basis* a, const Basis* b) {
    return (Basis){ basis_apply(a, b->x), basis_apply(a, b->y), basis_apply(a, b->z) };
}

/// Determinant.
static inline float basis_determinant(const Basis* b) { return v3_dot(b->x, v3_cross(b->y, b->z)); }

/// Hamilton product; the result applies `b` first, then `a`.
static inline Quat quat_mul(Quat a, Quat b) {
    return (Quat){ a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
                   a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
                   a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w,
                   a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z };
}

/// Four-component dot product.
static inline float quat_dot(Quat a, Quat b) { return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w; }

/// Compose two transforms; the result applies `b` first, then `a`.
static inline Transform transform_mul(const Transform* a, const Transform* b) {
    return (Transform){ basis_mul(&a->basis, &b->basis), v3_add(basis_apply(&a->basis, b->origin), a->origin) };
}

/// Uniform in [lo, hi) on each axis, drawn x, then y, then z.
static inline Vec3 v3_random(Rng* r, float lo, float hi) {
    Vec3 v;
    v.x = lo + rng_unit(r) * (hi - lo);
    v.y = lo + rng_unit(r) * (hi - lo);
    v.z = lo + rng_unit(r) * (hi - lo);
    return v;
}

/// Inverse of a basis, or the identity if it is singular.
Basis basis_inverse(const Basis* b);
/// Scaled to unit length, or the identity if its length is zero.
Quat quat_normalize(Quat q);
/// Normalised quaternion of four components uniform in [-1, 1).
Quat quat_random(Rng* r);
/// Rotate a vector by a unit quaternion.
Vec3 quat_rotate(Quat q, Vec3 v);
/// Rotation basis of a unit quaternion.
Basis basis_from_quat(Quat q);
/// Unit quaternion of a rotation basis.
Quat quat_from_basis(const Basis* b);
/// Column lengths, with x negated when the basis is mirrored.
Vec3 basis_scale(const Basis* b);
/// Rotation of a basis with its scale removed.
Quat basis_rotation(const Basis* b);
/// Rotation basis with per-axis scale applied to its columns.
Basis basis_from_rotation_scale(Quat rotation, Vec3 scale);
/// Advance a rotation by a world-space axis-angle displacement.
Quat integrate_rotation(Quat q, Vec3 angular_delta);
/// Transform at `eye` whose negative z axis faces `target`.
Transform looking_at(Vec3 eye, Vec3 target, Vec3 up);
/// Inverse of a transform with an orthonormal basis.
Transform inverse_orthonormal(const Transform* t);
/// Column-major matrix of a transform.
Mat4 to_mat4(const Transform* t);
/// Perspective projection from the tangent of half the vertical field of
/// view: the engine's formula with the tangent precomputed, since `tan`
/// differs in the last bit between C libraries.
Mat4 perspective(float tan_half_fov, float aspect, float z_near, float z_far);
/// Identity matrix.
Mat4 mat4_identity(void);
/// Matrix product; the result applies `b` first, then `a`.
Mat4 mat4_mul(const Mat4* a, const Mat4* b);
/// Cofactor inverse, or the identity if the matrix is singular.
Mat4 mat4_inverse(const Mat4* matrix);
/// Transform a point and divide by the resulting w.
Vec3 mat4_transform_point(const Mat4* matrix, Vec3 p);

#endif
