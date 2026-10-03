#pragma once

#include <algorithm>

namespace bench::ui {

/// Two-component float vector.
struct Vec2 {
    float x{};
    float y{};

    /// Compares both components for exact equality.
    constexpr bool operator==(const Vec2&) const noexcept = default;
};

/// Component-wise sum.
constexpr Vec2 operator+(Vec2 a, Vec2 b) noexcept {
    return {a.x + b.x, a.y + b.y};
}

/// Component-wise difference.
constexpr Vec2 operator-(Vec2 a, Vec2 b) noexcept {
    return {a.x - b.x, a.y - b.y};
}

/// Both components scaled by `s`.
constexpr Vec2 operator*(Vec2 a, float s) noexcept {
    return {a.x * s, a.y * s};
}

/// Component-wise `std::max`.
constexpr Vec2 max(Vec2 a, Vec2 b) noexcept {
    return {std::max(a.x, b.x), std::max(a.y, b.y)};
}

/// Component-wise `std::min`.
constexpr Vec2 min(Vec2 a, Vec2 b) noexcept {
    return {std::min(a.x, b.x), std::min(a.y, b.y)};
}

/// Axis-aligned rectangle given by its position and size.
struct Rect2 {
    Vec2 position{};
    Vec2 size{};

    /// Compares position and size for exact equality.
    constexpr bool operator==(const Rect2&) const noexcept = default;

    /// The minimum x edge.
    constexpr float left() const noexcept { return position.x; }
    /// The minimum y edge.
    constexpr float top() const noexcept { return position.y; }
    /// The maximum x edge.
    constexpr float right() const noexcept { return position.x + size.x; }
    /// The maximum y edge.
    constexpr float bottom() const noexcept { return position.y + size.y; }
    /// The corner opposite the position.
    constexpr Vec2 end() const noexcept { return position + size; }
    /// The centre point.
    constexpr Vec2 center() const noexcept { return position + size * 0.5f; }
};

/// The overlap of two rectangles, with zero size where they do not overlap.
constexpr Rect2 intersection(const Rect2& a, const Rect2& b) noexcept {
    const Vec2 lower = max(a.position, b.position);
    const Vec2 upper = min(a.end(), b.end());
    return Rect2{lower, max(upper - lower, Vec2{})};
}

/// A rectangle expanded by `amount` on every side.
constexpr Rect2 grow(const Rect2& rect, float amount) noexcept {
    return Rect2{rect.position - Vec2{amount, amount}, rect.size + Vec2{amount * 2.0f, amount * 2.0f}};
}

/// Straight RGBA colour, white by default.
struct Color4 {
    float r{1.0f};
    float g{1.0f};
    float b{1.0f};
    float a{1.0f};
};

/// `color` with its alpha replaced.
constexpr Color4 with_alpha(Color4 color, float alpha) noexcept {
    return {color.r, color.g, color.b, alpha};
}

/// `color` multiplied channel by channel with `by`.
constexpr Color4 modulated(Color4 color, Color4 by) noexcept {
    return {color.r * by.r, color.g * by.g, color.b * by.b, color.a * by.a};
}

} // namespace bench::ui
