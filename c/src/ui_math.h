#ifndef BENCH_UI_MATH_H
#define BENCH_UI_MATH_H

#include <stdbool.h>

#include "vecmath.h"

/// The UI kernel's value types. Operations are plain `+ - *` on floats, and
/// every min or max is written through `f32_min` and `f32_max` so each one
/// picks the same operand as the C++ `std::min` and `std::max`.

/// Two-component float vector.
typedef struct {
    float x;
    float y;
} Vec2;

/// Axis-aligned rectangle given by its position and size.
typedef struct {
    Vec2 position;
    Vec2 size;
} Rect2;

/// Straight RGBA colour.
typedef struct {
    float r;
    float g;
    float b;
    float a;
} Color4;

/// A vector from its components.
static inline Vec2 ui_vec2(float x, float y) { return (Vec2){ x, y }; }

/// A rectangle from its position and size.
static inline Rect2 ui_rect2(Vec2 position, Vec2 size) { return (Rect2){ position, size }; }

/// Opaque white, the default colour.
static inline Color4 ui_white(void) { return (Color4){ 1.0f, 1.0f, 1.0f, 1.0f }; }

/// Component-wise sum.
static inline Vec2 ui_vec2_add(Vec2 a, Vec2 b) { return (Vec2){ a.x + b.x, a.y + b.y }; }

/// Component-wise difference.
static inline Vec2 ui_vec2_sub(Vec2 a, Vec2 b) { return (Vec2){ a.x - b.x, a.y - b.y }; }

/// Both components scaled by `s`.
static inline Vec2 ui_vec2_scale(Vec2 a, float s) { return (Vec2){ a.x * s, a.y * s }; }

/// Component-wise `std::max`.
static inline Vec2 ui_vec2_max(Vec2 a, Vec2 b) { return (Vec2){ f32_max(a.x, b.x), f32_max(a.y, b.y) }; }

/// Component-wise `std::min`.
static inline Vec2 ui_vec2_min(Vec2 a, Vec2 b) { return (Vec2){ f32_min(a.x, b.x), f32_min(a.y, b.y) }; }

/// Exact component-wise equality.
static inline bool ui_vec2_equal(Vec2 a, Vec2 b) { return a.x == b.x && a.y == b.y; }

/// Exact equality of position and size.
static inline bool ui_rect2_equal(const Rect2* a, const Rect2* b) {
    return ui_vec2_equal(a->position, b->position) && ui_vec2_equal(a->size, b->size);
}

/// The minimum x edge.
static inline float ui_rect2_left(const Rect2* rect) { return rect->position.x; }

/// The minimum y edge.
static inline float ui_rect2_top(const Rect2* rect) { return rect->position.y; }

/// The maximum x edge.
static inline float ui_rect2_right(const Rect2* rect) { return rect->position.x + rect->size.x; }

/// The maximum y edge.
static inline float ui_rect2_bottom(const Rect2* rect) { return rect->position.y + rect->size.y; }

/// The corner opposite the position.
static inline Vec2 ui_rect2_end(const Rect2* rect) { return ui_vec2_add(rect->position, rect->size); }

/// The centre point.
static inline Vec2 ui_rect2_center(const Rect2* rect) {
    return ui_vec2_add(rect->position, ui_vec2_scale(rect->size, 0.5f));
}

/// The overlap of two rectangles, with zero size where they do not overlap.
static inline Rect2 ui_rect2_intersection(const Rect2* a, const Rect2* b) {
    Vec2 lower = ui_vec2_max(a->position, b->position);
    Vec2 upper = ui_vec2_min(ui_rect2_end(a), ui_rect2_end(b));
    return ui_rect2(lower, ui_vec2_max(ui_vec2_sub(upper, lower), ui_vec2(0.0f, 0.0f)));
}

/// A rectangle expanded by `amount` on every side.
static inline Rect2 ui_rect2_grow(const Rect2* rect, float amount) {
    return ui_rect2(ui_vec2_sub(rect->position, ui_vec2(amount, amount)),
            ui_vec2_add(rect->size, ui_vec2(amount * 2.0f, amount * 2.0f)));
}

/// `color` with its alpha replaced.
static inline Color4 ui_color_with_alpha(Color4 color, float alpha) { return (Color4){ color.r, color.g, color.b, alpha }; }

/// `color` multiplied channel by channel with `by`.
static inline Color4 ui_color_modulated(Color4 color, Color4 by) {
    return (Color4){ color.r * by.r, color.g * by.g, color.b * by.b, color.a * by.a };
}

#endif
