#include "vecmath.hpp"

namespace bench::vm {

Basis inverse(const Basis& b) {
    const Vec3 r0 = cross(b.y, b.z);
    const Vec3 r1 = cross(b.z, b.x);
    const Vec3 r2 = cross(b.x, b.y);
    const float det = dot(b.x, r0);
    if (det == 0.0f) return Basis{};
    const float scale = 1.0f / det;
    return transposed(Basis{r0 * scale, r1 * scale, r2 * scale});
}

Quat normalize(Quat q) {
    const float len = std::sqrt(dot(q, q));
    if (len <= 0.0f) return Quat{};
    const float inv = 1.0f / len;
    return {q.x * inv, q.y * inv, q.z * inv, q.w * inv};
}

Quat random_quat(Rng& rng) {
    Quat q;
    q.x = rng.unit() * 2.0f - 1.0f;
    q.y = rng.unit() * 2.0f - 1.0f;
    q.z = rng.unit() * 2.0f - 1.0f;
    q.w = rng.unit() * 2.0f - 1.0f;
    return normalize(q);
}

Vec3 rotate(Quat q, Vec3 v) {
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = cross(u, v) * 2.0f;
    return v + t * q.w + cross(u, t);
}

Basis basis_from_quat(Quat q) {
    const float xx = q.x * q.x, yy = q.y * q.y, zz = q.z * q.z;
    const float xy = q.x * q.y, xz = q.x * q.z, yz = q.y * q.z;
    const float wx = q.w * q.x, wy = q.w * q.y, wz = q.w * q.z;
    return {{1.0f - 2.0f * (yy + zz), 2.0f * (xy + wz), 2.0f * (xz - wy)},
            {2.0f * (xy - wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz + wx)},
            {2.0f * (xz + wy), 2.0f * (yz - wx), 1.0f - 2.0f * (xx + yy)}};
}

Quat quat_from_basis(const Basis& b) {
    const float trace = b.x.x + b.y.y + b.z.z;
    Quat q{};
    if (trace > 0.0f) {
        const float s = std::sqrt(trace + 1.0f) * 2.0f;
        q.w = 0.25f * s;
        q.x = (b.y.z - b.z.y) / s;
        q.y = (b.z.x - b.x.z) / s;
        q.z = (b.x.y - b.y.x) / s;
    } else if (b.x.x > b.y.y && b.x.x > b.z.z) {
        const float s = std::sqrt(1.0f + b.x.x - b.y.y - b.z.z) * 2.0f;
        q.w = (b.y.z - b.z.y) / s;
        q.x = 0.25f * s;
        q.y = (b.y.x + b.x.y) / s;
        q.z = (b.z.x + b.x.z) / s;
    } else if (b.y.y > b.z.z) {
        const float s = std::sqrt(1.0f + b.y.y - b.x.x - b.z.z) * 2.0f;
        q.w = (b.z.x - b.x.z) / s;
        q.x = (b.y.x + b.x.y) / s;
        q.y = 0.25f * s;
        q.z = (b.z.y + b.y.z) / s;
    } else {
        const float s = std::sqrt(1.0f + b.z.z - b.x.x - b.y.y) * 2.0f;
        q.w = (b.x.y - b.y.x) / s;
        q.x = (b.z.x + b.x.z) / s;
        q.y = (b.z.y + b.y.z) / s;
        q.z = 0.25f * s;
    }
    return normalize(q);
}

Vec3 basis_scale(const Basis& b) {
    const Vec3 scale{length(b.x), length(b.y), length(b.z)};
    return determinant(b) < 0.0f ? Vec3{-scale.x, scale.y, scale.z} : scale;
}

Quat basis_rotation(const Basis& b) {
    const Vec3 scale = basis_scale(b);
    return quat_from_basis(Basis{scale.x != 0.0f ? b.x / scale.x : Vec3{1.0f, 0.0f, 0.0f},
                                 scale.y != 0.0f ? b.y / scale.y : Vec3{0.0f, 1.0f, 0.0f},
                                 scale.z != 0.0f ? b.z / scale.z : Vec3{0.0f, 0.0f, 1.0f}});
}

Basis basis_from_rotation_scale(Quat rotation, Vec3 scale) {
    const Basis b = basis_from_quat(rotation);
    return Basis{b.x * scale.x, b.y * scale.y, b.z * scale.z};
}

Quat integrate_rotation(Quat q, Vec3 angular_delta) {
    const Quat omega{angular_delta.x, angular_delta.y, angular_delta.z, 0.0f};
    const Quat d = omega * q;
    return normalize(Quat{q.x + 0.5f * d.x, q.y + 0.5f * d.y, q.z + 0.5f * d.z, q.w + 0.5f * d.w});
}

Transform looking_at(Vec3 eye, Vec3 target, Vec3 up) {
    const Vec3 back = normalize(eye - target);
    const Vec3 right = normalize(cross(up, eye - target));
    const Vec3 true_up = cross(back, right);
    return Transform{Basis{right, true_up, back}, eye};
}

Transform inverse_orthonormal(const Transform& t) {
    const Basis inv = transposed(t.basis);
    return Transform{inv, -(inv * t.origin)};
}

Mat4 to_mat4(const Transform& t) {
    Mat4 r = identity();
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

Mat4 perspective(float tan_half_fov, float aspect, float z_near, float z_far) {
    const float depth = z_far - z_near;
    Mat4 m{};
    m.m[0] = 1.0f / (tan_half_fov * aspect);
    m.m[5] = 1.0f / tan_half_fov;
    m.m[10] = -(z_far + z_near) / depth;
    m.m[11] = -1.0f;
    m.m[14] = -2.0f * z_far * z_near / depth;
    return m;
}

Mat4 identity() {
    Mat4 r{};
    r.m[0] = 1.0f;
    r.m[5] = 1.0f;
    r.m[10] = 1.0f;
    r.m[15] = 1.0f;
    return r;
}

Mat4 operator*(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int column = 0; column < 4; column++) {
        for (int row = 0; row < 4; row++) {
            float sum = 0.0f;
            for (int k = 0; k < 4; k++) sum += a.m[k * 4 + row] * b.m[column * 4 + k];
            r.m[column * 4 + row] = sum;
        }
    }
    return r;
}

Mat4 inverse(const Mat4& matrix) {
    const auto& m = matrix.m;
    std::array<float, 16> inv{};
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
    const float det = m[0] * inv[0] + m[1] * inv[4] + m[2] * inv[8] + m[3] * inv[12];
    if (det == 0.0f) return identity();
    const float scale = 1.0f / det;
    Mat4 r{};
    for (int i = 0; i < 16; i++) r.m[i] = inv[i] * scale;
    return r;
}

Vec3 transform_point(const Mat4& matrix, Vec3 p) {
    const auto& m = matrix.m;
    const float x = m[0] * p.x + m[4] * p.y + m[8] * p.z + m[12];
    const float y = m[1] * p.x + m[5] * p.y + m[9] * p.z + m[13];
    const float z = m[2] * p.x + m[6] * p.y + m[10] * p.z + m[14];
    const float w = m[3] * p.x + m[7] * p.y + m[11] * p.z + m[15];
    return Vec3{x, y, z} / w;
}

} // namespace bench::vm
