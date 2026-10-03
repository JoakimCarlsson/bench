#pragma once

#include <cmath>
#include <cstddef>

/// The engine's simd.hpp, trimmed to what the contact solver uses: four- and
/// eight-lane float vectors on the GCC and Clang vector extensions, with
/// selects instead of min and max so every lane rounds the same as scalar
/// code.
namespace bench::simd {

using RawFloats4 = float __attribute__((vector_size(16)));
using RawMask4 = int __attribute__((vector_size(16)));
using RawFloats8 = float __attribute__((vector_size(32)));
using RawMask8 = int __attribute__((vector_size(32)));

/// Per-lane comparison result for FloatW.
struct MaskW {
    RawMask4 v;
};

/// Four floats processed together.
struct FloatW {
    RawFloats4 v;

    FloatW() : v{} {}
    explicit FloatW(float s) : v{s, s, s, s} {}
    FloatW(float a, float b, float c, float d) : v{a, b, c, d} {}
};

/// Per-lane comparison result for FloatW8.
struct alignas(32) MaskW8 {
    RawMask8 v;
};

/// Eight floats processed together.
struct alignas(32) FloatW8 {
    RawFloats8 v;

    FloatW8() : v{} {}
    explicit FloatW8(float s) : v{s, s, s, s, s, s, s, s} {}
    explicit FloatW8(const RawFloats8& raw) : v(raw) {}
    /// Joins two four-lane vectors, lo in lanes 0 to 3 and hi in lanes 4 to 7.
    FloatW8(FloatW lo, FloatW hi)
        : v{lo.v[0], lo.v[1], lo.v[2], lo.v[3], hi.v[0], hi.v[1], hi.v[2], hi.v[3]} {}
};

/// Reads one lane.
inline float get_lane(const FloatW& value, std::size_t index) { return value.v[index]; }
/// Reads one lane.
inline float get_lane(const FloatW8& value, std::size_t index) { return value.v[index]; }
/// Writes one lane.
inline void set_lane(FloatW8& value, std::size_t index, float x) { value.v[index] = x; }

/// Lane-wise sum.
inline FloatW8 operator+(FloatW8 a, FloatW8 b) { return FloatW8{a.v + b.v}; }
/// Lane-wise difference.
inline FloatW8 operator-(FloatW8 a, FloatW8 b) { return FloatW8{a.v - b.v}; }
/// Lane-wise product.
inline FloatW8 operator*(FloatW8 a, FloatW8 b) { return FloatW8{a.v * b.v}; }
/// Lane-wise quotient.
inline FloatW8 operator/(FloatW8 a, FloatW8 b) { return FloatW8{a.v / b.v}; }
/// Lane-wise negation.
inline FloatW8 operator-(FloatW8 a) { return FloatW8{-a.v}; }
/// Mask of a > b.
inline MaskW8 greater(FloatW8 a, FloatW8 b) { return MaskW8{a.v > b.v}; }
/// Mask of a < b.
inline MaskW8 less(FloatW8 a, FloatW8 b) { return MaskW8{a.v < b.v}; }
/// Mask of a <= b.
inline MaskW8 less_equal(FloatW8 a, FloatW8 b) { return MaskW8{a.v <= b.v}; }
/// Mask of a != b.
inline MaskW8 not_equal(FloatW8 a, FloatW8 b) { return MaskW8{a.v != b.v}; }
/// Lanes set in both masks.
inline MaskW8 both(MaskW8 a, MaskW8 b) { return MaskW8{a.v & b.v}; }
/// `yes` where the mask is set, `no` elsewhere.
inline FloatW8 select(MaskW8 mask, FloatW8 yes, FloatW8 no) { return FloatW8{mask.v ? yes.v : no.v}; }

/// Whether any lane of the mask is set.
inline bool any(MaskW8 mask) {
    return (mask.v[0] | mask.v[1] | mask.v[2] | mask.v[3] | mask.v[4] | mask.v[5] | mask.v[6] | mask.v[7]) != 0;
}

/// Lane-wise square root.
inline FloatW8 sqrt(FloatW8 a) {
    FloatW8 out;
    for (std::size_t i = 0; i < 8; ++i) out.v[i] = std::sqrt(a.v[i]);
    return out;
}

/// `a` where a > b, otherwise `b`.
inline FloatW8 max(FloatW8 a, FloatW8 b) { return select(greater(a, b), a, b); }
/// `a` where a < b, otherwise `b`.
inline FloatW8 min(FloatW8 a, FloatW8 b) { return select(less(a, b), a, b); }

/// Loads four consecutive floats; alignment is not required.
inline FloatW load4(const float* source) { return FloatW{source[0], source[1], source[2], source[3]}; }

/// Stores four lanes as consecutive floats; alignment is not required.
inline void store4(float* destination, FloatW value) {
    for (std::size_t i = 0; i < 4; ++i) destination[i] = value.v[i];
}

/// Transposes a 4x4 block held as four row vectors into four column vectors.
inline void transpose4(FloatW& r0, FloatW& r1, FloatW& r2, FloatW& r3) {
    const FloatW c0{get_lane(r0, 0), get_lane(r1, 0), get_lane(r2, 0), get_lane(r3, 0)};
    const FloatW c1{get_lane(r0, 1), get_lane(r1, 1), get_lane(r2, 1), get_lane(r3, 1)};
    const FloatW c2{get_lane(r0, 2), get_lane(r1, 2), get_lane(r2, 2), get_lane(r3, 2)};
    const FloatW c3{get_lane(r0, 3), get_lane(r1, 3), get_lane(r2, 3), get_lane(r3, 3)};
    r0 = c0;
    r1 = c1;
    r2 = c2;
    r3 = c3;
}

} // namespace bench::simd
