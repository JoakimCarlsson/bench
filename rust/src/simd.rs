//! The engine's simd.hpp for eight lanes, on stable Rust: a wrapper over the
//! AVX intrinsics in `core::arch`, since `std::simd` is still nightly-only.
//! Selects stand in for min and max so every lane rounds the same as scalar
//! code, and MAXPS and MINPS are exactly those selects.
use std::arch::x86_64::*;
use std::ops::{Add, Div, Mul, Neg, Sub};

#[cfg(not(target_feature = "avx"))]
compile_error!("the wide kernel needs AVX; build with -C target-cpu=native");

/// Eight floats processed together.
#[derive(Clone, Copy)]
#[repr(transparent)]
pub struct F8(__m256);

/// Per-lane comparison result for F8.
#[derive(Clone, Copy)]
#[repr(transparent)]
pub struct M8(__m256);

impl Default for F8 {
    /// All lanes zero.
    fn default() -> F8 {
        F8::splat(0.0)
    }
}

impl F8 {
    /// Every lane set to `s`.
    #[inline(always)]
    pub fn splat(s: f32) -> F8 {
        // SAFETY: AVX is enabled for the whole crate, checked above.
        F8(unsafe { _mm256_set1_ps(s) })
    }

    /// Joins two four-lane vectors, lo in lanes 0 to 3 and hi in lanes 4 to 7.
    #[inline(always)]
    pub fn join(lo: __m128, hi: __m128) -> F8 {
        // SAFETY: AVX is enabled for the whole crate.
        F8(unsafe { _mm256_set_m128(hi, lo) })
    }

    /// The lanes as an array.
    #[inline(always)]
    pub fn to_array(self) -> [f32; 8] {
        // SAFETY: __m256 and [f32; 8] have the same size and every bit pattern is valid for both.
        unsafe { std::mem::transmute(self.0) }
    }

    /// Reads one lane.
    #[inline(always)]
    pub fn lane(&self, index: usize) -> f32 {
        self.as_array()[index]
    }

    /// Writes one lane.
    #[inline(always)]
    pub fn set_lane(&mut self, index: usize, x: f32) {
        // SAFETY: __m256 is 32 bytes of eight f32 lanes with no invalid bit patterns.
        let lanes = unsafe { &mut *(self as *mut F8 as *mut [f32; 8]) };
        lanes[index] = x;
    }

    /// The lanes viewed in place.
    #[inline(always)]
    fn as_array(&self) -> &[f32; 8] {
        // SAFETY: __m256 is 32 bytes of eight f32 lanes with no invalid bit patterns.
        unsafe { &*(self as *const F8 as *const [f32; 8]) }
    }

    /// Mask of self > b.
    #[inline(always)]
    pub fn greater(self, b: F8) -> M8 {
        // SAFETY: AVX is enabled for the whole crate.
        M8(unsafe { _mm256_cmp_ps::<_CMP_GT_OQ>(self.0, b.0) })
    }

    /// Mask of self <= b.
    #[inline(always)]
    pub fn less_equal(self, b: F8) -> M8 {
        // SAFETY: AVX is enabled for the whole crate.
        M8(unsafe { _mm256_cmp_ps::<_CMP_LE_OQ>(self.0, b.0) })
    }

    /// Mask of self != b.
    #[inline(always)]
    pub fn not_equal(self, b: F8) -> M8 {
        // SAFETY: AVX is enabled for the whole crate.
        M8(unsafe { _mm256_cmp_ps::<_CMP_NEQ_UQ>(self.0, b.0) })
    }

    /// Lane-wise square root.
    #[inline(always)]
    pub fn sqrt(self) -> F8 {
        // SAFETY: AVX is enabled for the whole crate.
        F8(unsafe { _mm256_sqrt_ps(self.0) })
    }

    /// `self` where self > b, otherwise `b`.
    #[inline(always)]
    pub fn max(self, b: F8) -> F8 {
        // SAFETY: AVX is enabled for the whole crate.
        F8(unsafe { _mm256_max_ps(self.0, b.0) })
    }

    /// `self` where self < b, otherwise `b`.
    #[inline(always)]
    pub fn min(self, b: F8) -> F8 {
        // SAFETY: AVX is enabled for the whole crate.
        F8(unsafe { _mm256_min_ps(self.0, b.0) })
    }
}

impl M8 {
    /// Lanes set in both masks.
    #[inline(always)]
    pub fn and(self, b: M8) -> M8 {
        // SAFETY: AVX is enabled for the whole crate.
        M8(unsafe { _mm256_and_ps(self.0, b.0) })
    }

    /// Whether any lane is set.
    #[inline(always)]
    pub fn any(self) -> bool {
        // SAFETY: AVX is enabled for the whole crate.
        unsafe { _mm256_movemask_ps(self.0) != 0 }
    }

    /// `yes` where the mask is set, `no` elsewhere.
    #[inline(always)]
    pub fn select(self, yes: F8, no: F8) -> F8 {
        // SAFETY: AVX is enabled for the whole crate.
        F8(unsafe { _mm256_blendv_ps(no.0, yes.0, self.0) })
    }
}

impl Add for F8 {
    type Output = F8;

    /// Lane-wise sum.
    #[inline(always)]
    fn add(self, b: F8) -> F8 {
        // SAFETY: AVX is enabled for the whole crate.
        F8(unsafe { _mm256_add_ps(self.0, b.0) })
    }
}

impl Sub for F8 {
    type Output = F8;

    /// Lane-wise difference.
    #[inline(always)]
    fn sub(self, b: F8) -> F8 {
        // SAFETY: AVX is enabled for the whole crate.
        F8(unsafe { _mm256_sub_ps(self.0, b.0) })
    }
}

impl Mul for F8 {
    type Output = F8;

    /// Lane-wise product.
    #[inline(always)]
    fn mul(self, b: F8) -> F8 {
        // SAFETY: AVX is enabled for the whole crate.
        F8(unsafe { _mm256_mul_ps(self.0, b.0) })
    }
}

impl Div for F8 {
    type Output = F8;

    /// Lane-wise quotient.
    #[inline(always)]
    fn div(self, b: F8) -> F8 {
        // SAFETY: AVX is enabled for the whole crate.
        F8(unsafe { _mm256_div_ps(self.0, b.0) })
    }
}

impl Neg for F8 {
    type Output = F8;

    /// Lane-wise negation: a sign flip, like `-x`.
    #[inline(always)]
    fn neg(self) -> F8 {
        // SAFETY: AVX is enabled for the whole crate.
        F8(unsafe { _mm256_xor_ps(self.0, _mm256_set1_ps(-0.0)) })
    }
}

/// Loads four consecutive floats.
#[inline(always)]
pub fn load4(source: &[f32; 4]) -> __m128 {
    // SAFETY: the reference covers four readable floats; the load is unaligned.
    unsafe { _mm_loadu_ps(source.as_ptr()) }
}

/// Transposes a 4x4 block held as four row vectors into four columns.
#[inline(always)]
pub fn transpose4(rows: &mut [__m128; 4]) {
    let [r0, r1, r2, r3] = rows;
    // SAFETY: SSE is part of x86-64.
    unsafe { _MM_TRANSPOSE4_PS(r0, r1, r2, r3) }
}
