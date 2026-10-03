#include "ui_theme.hpp"

#include <utility>

namespace bench::ui {

namespace {

constexpr Color4 surface{0.12f, 0.13f, 0.15f, 0.94f};
constexpr Color4 surface_raised{0.17f, 0.19f, 0.22f, 1.0f};
constexpr Color4 surface_hover{0.22f, 0.25f, 0.29f, 1.0f};
constexpr Color4 surface_pressed{0.10f, 0.11f, 0.13f, 1.0f};
constexpr Color4 surface_sunken{0.07f, 0.08f, 0.09f, 1.0f};
constexpr Color4 outline{0.30f, 0.33f, 0.38f, 1.0f};
constexpr Color4 accent{0.29f, 0.56f, 0.89f, 1.0f};
constexpr Color4 accent_dim{0.29f, 0.56f, 0.89f, 0.35f};
constexpr Color4 text{0.88f, 0.90f, 0.93f, 1.0f};
constexpr Color4 text_dim{0.58f, 0.61f, 0.66f, 1.0f};
constexpr Color4 text_disabled{0.40f, 0.42f, 0.45f, 1.0f};

constexpr float base_font_size = 16.0f;
constexpr float base_radius = 4.0f;

/// Margins with the same `amount` on every side.
Margins uniform(float amount) {
    return Margins{amount, amount, amount, amount};
}

/// Margins with `horizontal` on the left and right and `vertical` on top and bottom.
Margins padded(float horizontal, float vertical) {
    return Margins{horizontal, vertical, horizontal, vertical};
}

/// A text-field frame: sunken, outlined, with the given content margins.
StyleBox edit_style(Margins content) {
    StyleBox style = flat_style(surface_sunken, base_radius, content);
    style.border_width = 1.0f;
    style.border_color = outline;
    return style;
}

/// A button frame of `color`, outlined, with the given content margins.
StyleBox button_style(Color4 color, Margins content) {
    StyleBox style = flat_style(color, base_radius, content);
    style.border_width = 1.0f;
    style.border_color = outline;
    return style;
}

} // namespace

Rect2 StyleBox::content_rect(const Rect2& rect) const noexcept {
    return Rect2{rect.position + Vec2{content_margins.left, content_margins.top}, max(rect.size - content_margins.size(), Vec2{})};
}

void StyleBox::draw(Canvas& canvas, const Rect2& rect, Color4 modulate) const {
    if (kind == StyleBoxKind::Empty) return;
    const Rect2 target{rect.position - Vec2{expand_margins.left, expand_margins.top}, rect.size + expand_margins.size()};
    if (color.a > 0.0f) canvas.fill_rect(target, modulated(color, modulate), corner_radius);
    if (border_width > 0.0f && border_color.a > 0.0f) canvas.stroke_rect(target, modulated(border_color, modulate), Stroke{border_width, corner_radius});
}

StyleBox flat_style(Color4 color, float corner_radius, Margins content) {
    StyleBox style;
    style.kind = StyleBoxKind::Flat;
    style.color = color;
    style.corner_radius = corner_radius;
    style.content_margins = content;
    return style;
}

StyleBox empty_style(Margins content) {
    StyleBox style;
    style.kind = StyleBoxKind::Empty;
    style.content_margins = content;
    return style;
}

Theme::TypeEntry& Theme::entry(std::string_view type) {
    const auto found = types_.find(type);
    if (found != types_.end()) return found->second;
    return types_.emplace(std::string{type}, TypeEntry{}).first->second;
}

const Theme::TypeEntry* Theme::find(std::string_view type) const {
    const auto found = types_.find(type);
    return found == types_.end() ? nullptr : &found->second;
}

void Theme::set_stylebox(ThemeKey key, StyleBox value) {
    entry(key.type).styleboxes.insert_or_assign(std::string{key.name}, value);
}

void Theme::set_color(ThemeKey key, Color4 value) {
    entry(key.type).colors.insert_or_assign(std::string{key.name}, value);
}

void Theme::set_constant(ThemeKey key, float value) {
    entry(key.type).constants.insert_or_assign(std::string{key.name}, value);
}

void Theme::set_font_size(ThemeKey key, float value) {
    entry(key.type).font_sizes.insert_or_assign(std::string{key.name}, value);
}

void Theme::set_type_base(ThemeKey inheritance) {
    entry(inheritance.type).base = std::string{inheritance.name};
}

const StyleBox* Theme::stylebox(ThemeKey key) const {
    const TypeEntry* found = find(key.type);
    if (found == nullptr) return nullptr;
    const auto item = found->styleboxes.find(key.name);
    return item == found->styleboxes.end() ? nullptr : &item->second;
}

const Color4* Theme::color(ThemeKey key) const {
    const TypeEntry* found = find(key.type);
    if (found == nullptr) return nullptr;
    const auto item = found->colors.find(key.name);
    return item == found->colors.end() ? nullptr : &item->second;
}

const float* Theme::constant(ThemeKey key) const {
    const TypeEntry* found = find(key.type);
    if (found == nullptr) return nullptr;
    const auto item = found->constants.find(key.name);
    return item == found->constants.end() ? nullptr : &item->second;
}

const float* Theme::font_size(ThemeKey key) const {
    const TypeEntry* found = find(key.type);
    if (found == nullptr) return nullptr;
    const auto item = found->font_sizes.find(key.name);
    return item == found->font_sizes.end() ? nullptr : &item->second;
}

std::string_view Theme::type_base(std::string_view type) const {
    const TypeEntry* found = find(type);
    return found == nullptr ? std::string_view{} : found->base;
}

std::shared_ptr<Theme> default_theme() {
    auto theme = std::make_shared<Theme>();

    theme->set_font_size({"Control", "font_size"}, base_font_size);
    theme->set_color({"Control", "font_color"}, text);

    theme->set_stylebox({"Panel", "panel"}, flat_style(surface, base_radius, uniform(8.0f)));
    theme->set_stylebox({"PanelContainer", "panel"}, flat_style(surface, base_radius, uniform(8.0f)));

    theme->set_color({"Label", "font_color"}, text);
    theme->set_color({"Label", "font_shadow_color"}, {0.0f, 0.0f, 0.0f, 0.0f});
    theme->set_font_size({"Label", "font_size"}, base_font_size);
    theme->set_constant({"Label", "line_spacing"}, 2.0f);

    theme->set_constant({"BoxContainer", "separation"}, 6.0f);
    theme->set_constant({"GridContainer", "h_separation"}, 6.0f);
    theme->set_constant({"GridContainer", "v_separation"}, 6.0f);
    theme->set_constant({"MarginContainer", "margin_left"}, 0.0f);
    theme->set_constant({"MarginContainer", "margin_top"}, 0.0f);
    theme->set_constant({"MarginContainer", "margin_right"}, 0.0f);
    theme->set_constant({"MarginContainer", "margin_bottom"}, 0.0f);

    StyleBox button_normal = button_style(surface_raised, padded(12.0f, 6.0f));
    StyleBox button_hover = button_normal;
    button_hover.color = surface_hover;
    StyleBox button_pressed = button_normal;
    button_pressed.color = surface_pressed;
    StyleBox button_disabled = button_normal;
    button_disabled.color = surface_sunken;
    button_disabled.border_color = with_alpha(outline, 0.4f);
    StyleBox button_focus = empty_style(padded(12.0f, 6.0f));
    button_focus.kind = StyleBoxKind::Flat;
    button_focus.color = {0.0f, 0.0f, 0.0f, 0.0f};
    button_focus.border_width = 1.0f;
    button_focus.border_color = accent;
    button_focus.corner_radius = base_radius;

    theme->set_stylebox({"Button", "normal"}, button_normal);
    theme->set_stylebox({"Button", "hover"}, button_hover);
    theme->set_stylebox({"Button", "pressed"}, button_pressed);
    theme->set_stylebox({"Button", "disabled"}, button_disabled);
    theme->set_stylebox({"Button", "focus"}, button_focus);
    theme->set_color({"Button", "font_color"}, text);
    theme->set_color({"Button", "font_hover_color"}, text);
    theme->set_color({"Button", "font_pressed_color"}, text);
    theme->set_color({"Button", "font_disabled_color"}, text_disabled);
    theme->set_font_size({"Button", "font_size"}, base_font_size);
    theme->set_constant({"Button", "h_separation"}, 6.0f);

    theme->set_stylebox({"CheckBox", "normal"}, empty_style(padded(4.0f, 4.0f)));
    theme->set_stylebox({"CheckBox", "focus"}, button_focus);
    theme->set_color({"CheckBox", "font_color"}, text);
    theme->set_color({"CheckBox", "font_disabled_color"}, text_disabled);
    theme->set_color({"CheckBox", "box_color"}, surface_sunken);
    theme->set_color({"CheckBox", "box_border_color"}, outline);
    theme->set_color({"CheckBox", "check_color"}, accent);
    theme->set_font_size({"CheckBox", "font_size"}, base_font_size);
    theme->set_constant({"CheckBox", "h_separation"}, 8.0f);
    theme->set_constant({"CheckBox", "box_size"}, 16.0f);

    theme->set_stylebox({"Slider", "slider"}, flat_style(surface_sunken, 3.0f));
    theme->set_stylebox({"Slider", "grabber_area"}, flat_style(accent, 3.0f));
    theme->set_color({"Slider", "grabber_color"}, text);
    theme->set_color({"Slider", "grabber_hover_color"}, accent);
    theme->set_constant({"Slider", "grabber_size"}, 14.0f);
    theme->set_constant({"Slider", "thickness"}, 6.0f);

    StyleBox edit_normal = edit_style(padded(8.0f, 5.0f));
    StyleBox edit_focus = edit_normal;
    edit_focus.border_color = accent;
    StyleBox edit_read_only = edit_normal;
    edit_read_only.color = surface;
    theme->set_stylebox({"LineEdit", "normal"}, edit_normal);
    theme->set_stylebox({"LineEdit", "focus"}, edit_focus);
    theme->set_stylebox({"LineEdit", "read_only"}, edit_read_only);
    theme->set_color({"LineEdit", "font_color"}, text);
    theme->set_color({"LineEdit", "placeholder_color"}, text_disabled);
    theme->set_color({"LineEdit", "selection_color"}, accent_dim);
    theme->set_color({"LineEdit", "caret_color"}, text);
    theme->set_font_size({"LineEdit", "font_size"}, base_font_size);
    theme->set_constant({"LineEdit", "minimum_width"}, 80.0f);
    theme->set_constant({"LineEdit", "caret_width"}, 1.0f);

    theme->set_stylebox({"TabContainer", "panel"}, flat_style(surface, base_radius, uniform(8.0f)));
    theme->set_stylebox({"TabContainer", "tab_selected"}, flat_style(surface, base_radius, padded(12.0f, 6.0f)));
    theme->set_stylebox({"TabContainer", "tab_unselected"}, flat_style(surface_sunken, base_radius, padded(12.0f, 6.0f)));
    theme->set_color({"TabContainer", "font_selected_color"}, text);
    theme->set_color({"TabContainer", "font_unselected_color"}, text_dim);
    theme->set_font_size({"TabContainer", "font_size"}, base_font_size);
    theme->set_constant({"TabContainer", "h_separation"}, 2.0f);

    return theme;
}

std::shared_ptr<Theme> inspector_theme() {
    auto theme = std::make_shared<Theme>();

    theme->set_constant({"BoxContainer", "separation"}, 4.0f);
    theme->set_constant({"GridContainer", "h_separation"}, 8.0f);
    theme->set_constant({"GridContainer", "v_separation"}, 3.0f);
    theme->set_font_size({"Label", "font_size"}, 14.0f);

    theme->set_type_base({"PropertyLabel", "Label"});
    theme->set_color({"PropertyLabel", "font_color"}, text_dim);

    theme->set_type_base({"SectionHeader", "Button"});
    theme->set_stylebox({"SectionHeader", "normal"}, button_style(surface_raised, padded(8.0f, 4.0f)));
    theme->set_stylebox({"SectionHeader", "hover"}, button_style(surface_hover, padded(8.0f, 4.0f)));
    theme->set_color({"SectionHeader", "font_color"}, accent);

    theme->set_stylebox({"LineEdit", "normal"}, edit_style(padded(6.0f, 3.0f)));
    theme->set_font_size({"LineEdit", "font_size"}, 14.0f);
    theme->set_constant({"LineEdit", "minimum_width"}, 48.0f);

    theme->set_constant({"Slider", "grabber_size"}, 12.0f);
    return theme;
}

} // namespace bench::ui
