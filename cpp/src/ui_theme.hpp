#pragma once

#include <functional>
#include <memory>
#include <string>
#include <string_view>
#include <unordered_map>

#include "ui_draw.hpp"

namespace bench::ui {

/// Transparent string hash that allows `string_view` lookups.
struct StringHash {
    using is_transparent = void;

    /// Hashes a string view.
    size_t operator()(std::string_view value) const noexcept {
        return std::hash<std::string_view>{}(value);
    }
};

/// String-keyed hash map that accepts `string_view` lookups without allocating.
template <typename T>
using StringMap = std::unordered_map<std::string, T, StringHash, std::equal_to<>>;

/// Per-side distances.
struct Margins {
    float left{};
    float top{};
    float right{};
    float bottom{};

    /// The summed horizontal and vertical margins.
    constexpr Vec2 size() const noexcept {
        return {left + right, top + bottom};
    }
};

/// A theme type name and item name pair.
struct ThemeKey {
    std::string_view type;
    std::string_view name;
};

/// How a style box draws.
enum class StyleBoxKind : uint8_t { Empty, Flat };

/// A drawable background with content margins.
struct StyleBox {
    StyleBoxKind kind{StyleBoxKind::Empty};
    Color4 color{};
    Color4 border_color{};
    float border_width{};
    float corner_radius{};
    Margins content_margins{};
    Margins expand_margins{};

    /// `rect` shrunk by the content margins.
    Rect2 content_rect(const Rect2& rect) const noexcept;
    /// Draws the style into `rect` grown by the expand margins, tinted by `modulate`.
    void draw(Canvas& canvas, const Rect2& rect, Color4 modulate) const;
};

/// A flat style box of `color` with rounded corners and content margins.
StyleBox flat_style(Color4 color, float corner_radius = 0.0f, Margins content = {});

/// A style box that draws nothing, with content margins.
StyleBox empty_style(Margins content = {});

/// Style boxes, colours, constants and font sizes grouped by type name, with type inheritance.
class Theme {
public:
    /// Sets a style box for a type and name.
    void set_stylebox(ThemeKey key, StyleBox value);
    /// Sets a colour for a type and name.
    void set_color(ThemeKey key, Color4 value);
    /// Sets a constant for a type and name.
    void set_constant(ThemeKey key, float value);
    /// Sets a font size for a type and name.
    void set_font_size(ThemeKey key, float value);
    /// Sets the base type of `inheritance.type` to `inheritance.name`.
    void set_type_base(ThemeKey inheritance);

    /// The style box for `key`, or null.
    const StyleBox* stylebox(ThemeKey key) const;
    /// The colour for `key`, or null.
    const Color4* color(ThemeKey key) const;
    /// The constant for `key`, or null.
    const float* constant(ThemeKey key) const;
    /// The font size for `key`, or null.
    const float* font_size(ThemeKey key) const;
    /// The base type of `type`, or an empty view when it has none.
    std::string_view type_base(std::string_view type) const;

private:
    struct TypeEntry {
        std::string base;
        StringMap<StyleBox> styleboxes;
        StringMap<Color4> colors;
        StringMap<float> constants;
        StringMap<float> font_sizes;
    };

    /// The entry for `type`, or null.
    const TypeEntry* find(std::string_view type) const;
    /// The entry for `type`, created when absent.
    TypeEntry& entry(std::string_view type);

    StringMap<TypeEntry> types_;
};

/// The built-in dark theme.
std::shared_ptr<Theme> default_theme();

/// A theme for an inspector panel: tighter spacing, dimmer property labels and
/// two button variations that inherit from `Button` through type bases.
std::shared_ptr<Theme> inspector_theme();

} // namespace bench::ui
