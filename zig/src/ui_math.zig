//! The two-dimensional float types of the UI: vectors, rectangles and colours.
const vm = @import("vecmath.zig");

/// Two-component float vector.
pub const Vec2 = struct {
    x: f32 = 0.0,
    y: f32 = 0.0,

    /// Compares both components for exact equality.
    pub fn eql(a: Vec2, b: Vec2) bool {
        return a.x == b.x and a.y == b.y;
    }

    /// Component-wise sum.
    pub fn add(a: Vec2, b: Vec2) Vec2 {
        return .{ .x = a.x + b.x, .y = a.y + b.y };
    }

    /// Component-wise difference.
    pub fn sub(a: Vec2, b: Vec2) Vec2 {
        return .{ .x = a.x - b.x, .y = a.y - b.y };
    }

    /// Both components scaled by `s`.
    pub fn scale(a: Vec2, s: f32) Vec2 {
        return .{ .x = a.x * s, .y = a.y * s };
    }

    /// Component-wise `std::max`.
    pub fn max(a: Vec2, b: Vec2) Vec2 {
        return .{ .x = vm.maxf(a.x, b.x), .y = vm.maxf(a.y, b.y) };
    }

    /// Component-wise `std::min`.
    pub fn min(a: Vec2, b: Vec2) Vec2 {
        return .{ .x = vm.minf(a.x, b.x), .y = vm.minf(a.y, b.y) };
    }
};

/// Axis-aligned rectangle given by its position and size.
pub const Rect2 = struct {
    position: Vec2 = .{},
    size: Vec2 = .{},

    /// Compares position and size for exact equality.
    pub fn eql(a: Rect2, b: Rect2) bool {
        return a.position.eql(b.position) and a.size.eql(b.size);
    }

    /// The minimum x edge.
    pub fn left(r: Rect2) f32 {
        return r.position.x;
    }

    /// The minimum y edge.
    pub fn top(r: Rect2) f32 {
        return r.position.y;
    }

    /// The maximum x edge.
    pub fn right(r: Rect2) f32 {
        return r.position.x + r.size.x;
    }

    /// The maximum y edge.
    pub fn bottom(r: Rect2) f32 {
        return r.position.y + r.size.y;
    }

    /// The corner opposite the position.
    pub fn end(r: Rect2) Vec2 {
        return r.position.add(r.size);
    }

    /// The centre point.
    pub fn center(r: Rect2) Vec2 {
        return r.position.add(r.size.scale(0.5));
    }
};

/// The overlap of two rectangles, with zero size where they do not overlap.
pub fn intersection(a: Rect2, b: Rect2) Rect2 {
    const lower = a.position.max(b.position);
    const upper = a.end().min(b.end());
    return .{ .position = lower, .size = upper.sub(lower).max(.{}) };
}

/// A rectangle expanded by `amount` on every side.
pub fn grow(rect: Rect2, amount: f32) Rect2 {
    const margin: Vec2 = .{ .x = amount, .y = amount };
    const doubled: Vec2 = .{ .x = amount * 2.0, .y = amount * 2.0 };
    return .{ .position = rect.position.sub(margin), .size = rect.size.add(doubled) };
}

/// Straight RGBA colour, white by default.
pub const Color4 = struct {
    r: f32 = 1.0,
    g: f32 = 1.0,
    b: f32 = 1.0,
    a: f32 = 1.0,

    /// A colour from its channels.
    pub fn init(r: f32, g: f32, b: f32, a: f32) Color4 {
        return .{ .r = r, .g = g, .b = b, .a = a };
    }

    /// The colour with its alpha replaced.
    pub fn withAlpha(color: Color4, alpha: f32) Color4 {
        return .{ .r = color.r, .g = color.g, .b = color.b, .a = alpha };
    }

    /// The colour multiplied channel by channel with `by`.
    pub fn modulated(color: Color4, by: Color4) Color4 {
        return .{ .r = color.r * by.r, .g = color.g * by.g, .b = color.b * by.b, .a = color.a * by.a };
    }
};
