//! Eight lanes of `f32` as a plain array, with the interface of `simd::F8`.
//! Every operation is a per-lane scalar operation the compiler is free to
//! turn into AVX; no intrinsics. Min, max and select are the
//! same comparisons the hardware instructions make, so every lane rounds and
//! picks exactly as the intrinsic wrapper does.
use std::ops::{Add, Div, Mul, Neg, Sub};

const LANES: usize = 8;

/// Eight floats processed together.
#[derive(Clone, Copy)]
#[repr(C, align(32))]
pub struct F8([f32; LANES]);

/// Per-lane comparison result for F8.
#[derive(Clone, Copy)]
pub struct M8([bool; LANES]);

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
        F8([s; LANES])
    }

    /// The lanes as an array.
    #[inline(always)]
    pub fn to_array(self) -> [f32; LANES] {
        self.0
    }

    /// Reads one lane.
    #[inline(always)]
    pub fn lane(&self, index: usize) -> f32 {
        self.0[index]
    }

    /// Writes one lane.
    #[inline(always)]
    pub fn set_lane(&mut self, index: usize, x: f32) {
        self.0[index] = x;
    }

    /// Applies `f` to every lane.
    #[inline(always)]
    fn map(self, f: impl Fn(f32) -> f32) -> F8 {
        F8(self.0.map(f))
    }

    /// Combines the lanes of two vectors with `f`.
    #[inline(always)]
    fn zip_with(self, b: F8, f: impl Fn(f32, f32) -> f32) -> F8 {
        F8(std::array::from_fn(|lane| f(self.0[lane], b.0[lane])))
    }

    /// Compares the lanes of two vectors with `f`.
    #[inline(always)]
    fn compare(self, b: F8, f: impl Fn(f32, f32) -> bool) -> M8 {
        M8(std::array::from_fn(|lane| f(self.0[lane], b.0[lane])))
    }

    /// Mask of self > b.
    #[inline(always)]
    pub fn greater(self, b: F8) -> M8 {
        self.compare(b, |x, y| x > y)
    }

    /// Mask of self <= b.
    #[inline(always)]
    pub fn less_equal(self, b: F8) -> M8 {
        self.compare(b, |x, y| x <= y)
    }

    /// Mask of self != b.
    #[inline(always)]
    pub fn not_equal(self, b: F8) -> M8 {
        self.compare(b, |x, y| x != y)
    }

    /// Lane-wise square root.
    #[inline(always)]
    pub fn sqrt(self) -> F8 {
        self.map(f32::sqrt)
    }

    /// `self` where self > b, otherwise `b`.
    #[inline(always)]
    pub fn max(self, b: F8) -> F8 {
        self.zip_with(b, |x, y| if x > y { x } else { y })
    }

    /// `self` where self < b, otherwise `b`.
    #[inline(always)]
    pub fn min(self, b: F8) -> F8 {
        self.zip_with(b, |x, y| if x < y { x } else { y })
    }
}

impl M8 {
    /// Lanes set in both masks.
    #[inline(always)]
    pub fn and(self, b: M8) -> M8 {
        M8(std::array::from_fn(|lane| self.0[lane] & b.0[lane]))
    }

    /// Whether any lane is set.
    #[inline(always)]
    pub fn any(self) -> bool {
        self.0.iter().fold(false, |any, &set| any | set)
    }

    /// `yes` where the mask is set, `no` elsewhere.
    #[inline(always)]
    pub fn select(self, yes: F8, no: F8) -> F8 {
        F8(std::array::from_fn(|lane| if self.0[lane] { yes.0[lane] } else { no.0[lane] }))
    }
}

impl Add for F8 {
    type Output = F8;

    /// Lane-wise sum.
    #[inline(always)]
    fn add(self, b: F8) -> F8 {
        self.zip_with(b, |x, y| x + y)
    }
}

impl Sub for F8 {
    type Output = F8;

    /// Lane-wise difference.
    #[inline(always)]
    fn sub(self, b: F8) -> F8 {
        self.zip_with(b, |x, y| x - y)
    }
}

impl Mul for F8 {
    type Output = F8;

    /// Lane-wise product.
    #[inline(always)]
    fn mul(self, b: F8) -> F8 {
        self.zip_with(b, |x, y| x * y)
    }
}

impl Div for F8 {
    type Output = F8;

    /// Lane-wise quotient.
    #[inline(always)]
    fn div(self, b: F8) -> F8 {
        self.zip_with(b, |x, y| x / y)
    }
}

impl Neg for F8 {
    type Output = F8;

    /// Lane-wise negation: a sign flip.
    #[inline(always)]
    fn neg(self) -> F8 {
        self.map(|x| -x)
    }
}
