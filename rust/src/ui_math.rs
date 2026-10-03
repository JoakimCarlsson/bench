//! Two-component vectors, rectangles and colours of the UI kernel, with the
//! float selects written the way `std::max`, `std::min` and `std::clamp`
//! pick their results.
use std::ops::{Add, Mul, Sub};

/// The larger of two floats as `std::max` picks it: `a` unless `a < b`.
pub fn max(a: f32, b: f32) -> f32 {
    if a < b { b } else { a }
}

/// The smaller of two floats as `std::min` picks it: `a` unless `b < a`.
pub fn min(a: f32, b: f32) -> f32 {
    if b < a { b } else { a }
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

/// The larger of two doubles as `std::max` picks it.
pub fn max64(a: f64, b: f64) -> f64 {
    if a < b { b } else { a }
}

/// `value` limited to [lo, hi] as `std::clamp` does it, in double precision.
pub fn clamp64(value: f64, lo: f64, hi: f64) -> f64 {
    if value < lo {
        lo
    } else if hi < value {
        hi
    } else {
        value
    }
}

/// Two-component float vector.
#[derive(Clone, Copy, Default, PartialEq)]
pub struct Vec2 {
    pub x: f32,
    pub y: f32,
}

impl Vec2 {
    /// A vector from its components.
    pub fn new(x: f32, y: f32) -> Vec2 {
        Vec2 { x, y }
    }

    /// Component-wise `std::max`.
    pub fn max(self, b: Vec2) -> Vec2 {
        Vec2::new(max(self.x, b.x), max(self.y, b.y))
    }

    /// Component-wise `std::min`.
    pub fn min(self, b: Vec2) -> Vec2 {
        Vec2::new(min(self.x, b.x), min(self.y, b.y))
    }
}

impl Add for Vec2 {
    type Output = Vec2;

    /// Component-wise sum.
    fn add(self, b: Vec2) -> Vec2 {
        Vec2::new(self.x + b.x, self.y + b.y)
    }
}

impl Sub for Vec2 {
    type Output = Vec2;

    /// Component-wise difference.
    fn sub(self, b: Vec2) -> Vec2 {
        Vec2::new(self.x - b.x, self.y - b.y)
    }
}

impl Mul<f32> for Vec2 {
    type Output = Vec2;

    /// Both components scaled by `s`.
    fn mul(self, s: f32) -> Vec2 {
        Vec2::new(self.x * s, self.y * s)
    }
}

/// Axis-aligned rectangle given by its position and size.
#[derive(Clone, Copy, Default, PartialEq)]
pub struct Rect2 {
    pub position: Vec2,
    pub size: Vec2,
}

impl Rect2 {
    /// A rectangle from its position and size.
    pub fn new(position: Vec2, size: Vec2) -> Rect2 {
        Rect2 { position, size }
    }

    /// The minimum x edge.
    pub fn left(&self) -> f32 {
        self.position.x
    }

    /// The minimum y edge.
    pub fn top(&self) -> f32 {
        self.position.y
    }

    /// The maximum x edge.
    pub fn right(&self) -> f32 {
        self.position.x + self.size.x
    }

    /// The maximum y edge.
    pub fn bottom(&self) -> f32 {
        self.position.y + self.size.y
    }

    /// The corner opposite the position.
    pub fn end(&self) -> Vec2 {
        self.position + self.size
    }

    /// The centre point.
    pub fn center(&self) -> Vec2 {
        self.position + self.size * 0.5
    }
}

/// The overlap of two rectangles, with zero size where they do not overlap.
pub fn intersection(a: &Rect2, b: &Rect2) -> Rect2 {
    let lower = a.position.max(b.position);
    let upper = a.end().min(b.end());
    Rect2::new(lower, (upper - lower).max(Vec2::default()))
}

/// A rectangle expanded by `amount` on every side.
pub fn grow(rect: &Rect2, amount: f32) -> Rect2 {
    Rect2::new(rect.position - Vec2::new(amount, amount), rect.size + Vec2::new(amount * 2.0, amount * 2.0))
}

/// Straight RGBA colour, white by default.
#[derive(Clone, Copy, PartialEq)]
pub struct Color4 {
    pub r: f32,
    pub g: f32,
    pub b: f32,
    pub a: f32,
}

impl Default for Color4 {
    /// Opaque white.
    fn default() -> Color4 {
        Color4::new(1.0, 1.0, 1.0, 1.0)
    }
}

impl Color4 {
    /// A colour from its channels.
    pub const fn new(r: f32, g: f32, b: f32, a: f32) -> Color4 {
        Color4 { r, g, b, a }
    }

    /// This colour with its alpha replaced.
    pub fn with_alpha(self, alpha: f32) -> Color4 {
        Color4::new(self.r, self.g, self.b, alpha)
    }

    /// This colour multiplied channel by channel with `by`.
    pub fn modulated(self, by: Color4) -> Color4 {
        Color4::new(self.r * by.r, self.g * by.g, self.b * by.b, self.a * by.a)
    }
}
