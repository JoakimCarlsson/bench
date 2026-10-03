#ifndef BENCH_SIMD_H
#define BENCH_SIMD_H

#include <immintrin.h>
#include <stdbool.h>

/// The engine's simd.hpp, trimmed to what the contact solver uses: eight-lane
/// float vectors on AVX intrinsics, where MAXPS and MINPS are exactly the
/// selects `a > b ? a : b` and `a < b ? a : b`, so every lane rounds the same
/// as scalar code.

typedef __m256 W;

/// Every lane set to `s`.
static inline W w_splat(float s) { return _mm256_set1_ps(s); }
/// Lane-wise sum.
static inline W w_add(W a, W b) { return _mm256_add_ps(a, b); }
/// Lane-wise difference.
static inline W w_sub(W a, W b) { return _mm256_sub_ps(a, b); }
/// Lane-wise product.
static inline W w_mul(W a, W b) { return _mm256_mul_ps(a, b); }
/// Lane-wise quotient.
static inline W w_div(W a, W b) { return _mm256_div_ps(a, b); }
/// Lane-wise negation.
static inline W w_neg(W a) { return _mm256_xor_ps(a, _mm256_set1_ps(-0.0f)); }
/// Lane-wise square root.
static inline W w_sqrt(W a) { return _mm256_sqrt_ps(a); }
/// Mask of a > b.
static inline W w_greater(W a, W b) { return _mm256_cmp_ps(a, b, _CMP_GT_OQ); }
/// Mask of a <= b.
static inline W w_less_equal(W a, W b) { return _mm256_cmp_ps(a, b, _CMP_LE_OQ); }
/// Mask of a != b.
static inline W w_not_equal(W a, W b) { return _mm256_cmp_ps(a, b, _CMP_NEQ_UQ); }
/// Lanes set in both masks.
static inline W w_both(W a, W b) { return _mm256_and_ps(a, b); }
/// `yes` where the mask is set, `no` elsewhere.
static inline W w_select(W mask, W yes, W no) { return _mm256_blendv_ps(no, yes, mask); }
/// Whether any lane of the mask is set.
static inline bool w_any(W mask) { return _mm256_movemask_ps(mask) != 0; }
/// `a` where a > b, otherwise `b`: exactly what MAXPS does.
static inline W w_max(W a, W b) { return _mm256_max_ps(a, b); }
/// `a` where a < b, otherwise `b`: exactly what MINPS does.
static inline W w_min(W a, W b) { return _mm256_min_ps(a, b); }
/// Reads one lane.
static inline float w_lane(W a, int lane) { return ((const float*)&a)[lane]; }
/// Writes one lane.
static inline void w_set_lane(W* a, int lane, float x) { ((float*)a)[lane] = x; }

#endif
