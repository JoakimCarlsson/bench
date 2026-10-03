#ifndef BENCH_UI_THEME_H
#define BENCH_UI_THEME_H

#include "ui_draw.h"
#include "ui_math.h"
#include "ui_strmap.h"

/// Per-side distances.
typedef struct {
    float left;
    float top;
    float right;
    float bottom;
} Margins;

/// The summed horizontal and vertical margins.
static inline Vec2 ui_margins_size(Margins margins) {
    return ui_vec2(margins.left + margins.right, margins.top + margins.bottom);
}

/// A theme type name and item name pair.
typedef struct {
    StrView type;
    StrView name;
} ThemeKey;

/// The key of the item `name` of type `type`, both string literals.
#define UI_KEY(type, name) ((ThemeKey){ UI_SV(type), UI_SV(name) })

/// How a style box draws.
typedef enum { UI_STYLEBOX_EMPTY, UI_STYLEBOX_FLAT } StyleBoxKind;

/// A drawable background with content margins.
typedef struct {
    StyleBoxKind kind;
    Color4 color;
    Color4 border_color;
    float border_width;
    float corner_radius;
    Margins content_margins;
    Margins expand_margins;
} StyleBox;

/// A style box that draws nothing, with white colours and no margins.
StyleBox ui_stylebox_default(void);

/// `rect` shrunk by the content margins of `style`.
Rect2 ui_stylebox_content_rect(const StyleBox* style, Rect2 rect);

/// Draws `style` into `rect` grown by its expand margins, tinted by `modulate`.
void ui_stylebox_draw(const StyleBox* style, Canvas* canvas, Rect2 rect, Color4 modulate);

/// A flat style box of `color` with rounded corners and content margins.
StyleBox ui_flat_style(Color4 color, float corner_radius, Margins content);

/// A style box that draws nothing, with content margins.
StyleBox ui_empty_style(Margins content);

/// The four kinds of theme item, each with its own map per type.
typedef enum {
    UI_THEME_STYLEBOX,
    UI_THEME_COLOR,
    UI_THEME_CONSTANT,
    UI_THEME_FONT_SIZE,
    UI_THEME_ITEM_KINDS
} ThemeItemKind;

/// Style boxes, colours, constants and font sizes grouped by type name,
/// with type inheritance: a hash map from type name to its entry, and in each
/// entry a base type name and one string-keyed map per item kind.
typedef struct {
    StrMap types;
} Theme;

/// An empty theme.
Theme* ui_theme_new(void);

/// Frees a theme and everything it holds.
void ui_theme_free(Theme* theme);

/// Sets a style box for a type and name.
void ui_theme_set_stylebox(Theme* theme, ThemeKey key, StyleBox value);

/// Sets a colour for a type and name.
void ui_theme_set_color(Theme* theme, ThemeKey key, Color4 value);

/// Sets a constant for a type and name.
void ui_theme_set_constant(Theme* theme, ThemeKey key, float value);

/// Sets a font size for a type and name.
void ui_theme_set_font_size(Theme* theme, ThemeKey key, float value);

/// Sets the base type of `inheritance.type` to `inheritance.name`.
void ui_theme_set_type_base(Theme* theme, ThemeKey inheritance);

/// The item of `kind` for `key` as a pointer to a `StyleBox`, `Color4` or `float`, or null.
const void* ui_theme_item(const Theme* theme, ThemeItemKind kind, ThemeKey key);

/// The base type of `type`, or an empty view when it has none.
StrView ui_theme_type_base(const Theme* theme, StrView type);

/// The built-in dark theme.
Theme* ui_default_theme(void);

/// A theme for an inspector panel: tighter spacing, dimmer property labels and
/// two button variations that inherit from `Button` through type bases.
Theme* ui_inspector_theme(void);

#endif
