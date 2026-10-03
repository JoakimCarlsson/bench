#include "ui_widgets.hpp"

#include <algorithm>
#include <utility>

namespace bench::ui {

namespace {

/// The offset that places `used` within `available` for a horizontal alignment.
float aligned_offset(HorizontalAlignment alignment, float available, float used) {
    switch (alignment) {
    case HorizontalAlignment::Left:
        return 0.0f;
    case HorizontalAlignment::Center:
        return (available - used) * 0.5f;
    case HorizontalAlignment::Right:
        return available - used;
    }
    return 0.0f;
}

/// The offset that places `used` within `available` for a vertical alignment.
float aligned_offset(VerticalAlignment alignment, float available, float used) {
    switch (alignment) {
    case VerticalAlignment::Top:
        return 0.0f;
    case VerticalAlignment::Center:
        return (available - used) * 0.5f;
    case VerticalAlignment::Bottom:
        return available - used;
    }
    return 0.0f;
}

} // namespace

void Panel::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("Panel");
    Control::collect_theme_types(out);
}

void Panel::draw(Canvas& canvas) const {
    theme_stylebox("panel").draw(canvas, Rect2{{}, size()}, effective_modulate());
}

Label::Label(std::string text) : text_{std::move(text)} {}

void Label::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("Label");
    Control::collect_theme_types(out);
}

std::vector<std::string_view> Label::lines() const {
    std::vector<std::string_view> result;
    std::string_view remaining{text_};
    while (true) {
        const size_t position = remaining.find('\n');
        if (position == std::string_view::npos) {
            result.push_back(remaining);
            break;
        }
        result.push_back(remaining.substr(0, position));
        remaining = remaining.substr(position + 1);
    }
    return result;
}

Vec2 Label::get_minimum_size() const {
    const float font_size = theme_font_size("font_size");
    const float spacing = theme_constant("line_spacing");
    const Canvas& target = canvas();
    Vec2 result{};
    const std::vector<std::string_view> entries = lines();
    for (size_t index = 0; index < entries.size(); ++index) {
        const Vec2 measured = target.measure_text(entries[index], font_size);
        result.x = std::max(result.x, measured.x);
        result.y += measured.y;
        if (index + 1 < entries.size()) result.y += spacing;
    }
    return result;
}

void Label::draw(Canvas& canvas) const {
    if (text_.empty()) return;
    const float font_size = theme_font_size("font_size");
    const float spacing = theme_constant("line_spacing");
    const Color4 color = modulated(theme_color("font_color"), effective_modulate());
    const std::vector<std::string_view> entries = lines();
    const float line_height = canvas.metrics(font_size).line_height;
    const float total = line_height * static_cast<float>(entries.size()) + spacing * static_cast<float>(entries.size() - 1);
    float y = aligned_offset(vertical_, size().y, total);
    for (const std::string_view line : entries) {
        const Vec2 measured = canvas.measure_text(line, font_size);
        const float x = aligned_offset(horizontal_, size().x, measured.x);
        canvas.draw_text({x, y}, line, font_size, color);
        y += line_height + spacing;
    }
}

void Label::set_text(std::string text) {
    if (text_ == text) return;
    text_ = std::move(text);
    update_minimum_size();
}

void Label::set_horizontal_alignment(HorizontalAlignment alignment) {
    horizontal_ = alignment;
}

void Label::set_vertical_alignment(VerticalAlignment alignment) {
    vertical_ = alignment;
}

void BaseButton::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("BaseButton");
    Control::collect_theme_types(out);
}

void BaseButton::set_disabled(bool disabled) {
    disabled_ = disabled;
    if (disabled_) held_ = false;
}

void BaseButton::set_toggle_mode(bool toggle) {
    toggle_mode_ = toggle;
}

void BaseButton::set_pressed(bool pressed) {
    pressed_ = pressed;
}

void BaseButton::set_held(bool held) {
    held_ = held;
}

bool BaseButton::pressed() const noexcept {
    return pressed_;
}

bool BaseButton::disabled() const noexcept {
    return disabled_;
}

ButtonState BaseButton::state() const noexcept {
    if (disabled_) return ButtonState::Disabled;
    if (held_ || (toggle_mode_ && pressed_)) return ButtonState::Pressed;
    if (hovered()) return ButtonState::Hover;
    return ButtonState::Normal;
}

Button::Button(std::string text) : text_{std::move(text)} {}

void Button::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("Button");
    BaseButton::collect_theme_types(out);
}

void Button::set_text(std::string text) {
    if (text_ == text) return;
    text_ = std::move(text);
    update_minimum_size();
}

void Button::set_icon(TextureId icon, Vec2 size) {
    icon_ = icon;
    icon_size_ = size;
    update_minimum_size();
}

void Button::set_horizontal_alignment(HorizontalAlignment alignment) {
    horizontal_ = alignment;
}

std::string_view Button::style_name() const {
    switch (state()) {
    case ButtonState::Disabled:
        return "disabled";
    case ButtonState::Pressed:
        return "pressed";
    case ButtonState::Hover:
        return "hover";
    case ButtonState::Normal:
        return "normal";
    }
    return "normal";
}

std::string_view Button::color_name() const {
    switch (state()) {
    case ButtonState::Disabled:
        return "font_disabled_color";
    case ButtonState::Pressed:
        return "font_pressed_color";
    case ButtonState::Hover:
        return "font_hover_color";
    case ButtonState::Normal:
        return "font_color";
    }
    return "font_color";
}

Vec2 Button::get_minimum_size() const {
    const float font_size = theme_font_size("font_size");
    const Vec2 measured = canvas().measure_text(text_, font_size);
    Vec2 content = measured;
    if (icon_ != no_texture) {
        content.x += icon_size_.x;
        content.y = std::max(content.y, icon_size_.y);
        if (!text_.empty()) content.x += theme_constant("h_separation");
    }
    return content + theme_stylebox("normal").content_margins.size();
}

void Button::draw(Canvas& canvas) const {
    const Color4 tint = effective_modulate();
    const Rect2 area{{}, size()};
    theme_stylebox(style_name()).draw(canvas, area, tint);
    if (has_focus()) theme_stylebox("focus").draw(canvas, area, tint);

    const Rect2 content = theme_stylebox(style_name()).content_rect(area);
    const float font_size = theme_font_size("font_size");
    const Vec2 measured = canvas.measure_text(text_, font_size);
    const float separation = (icon_ != no_texture && !text_.empty()) ? theme_constant("h_separation") : 0.0f;
    const float used = measured.x + separation + (icon_ != no_texture ? icon_size_.x : 0.0f);

    float x = content.position.x;
    switch (horizontal_) {
    case HorizontalAlignment::Left:
        break;
    case HorizontalAlignment::Center:
        x += (content.size.x - used) * 0.5f;
        break;
    case HorizontalAlignment::Right:
        x += content.size.x - used;
        break;
    }

    if (icon_ != no_texture) {
        const Rect2 icon_rect{{x, content.position.y + (content.size.y - icon_size_.y) * 0.5f}, icon_size_};
        canvas.fill_texture_rect(icon_rect, icon_, Rect2{{0.0f, 0.0f}, {1.0f, 1.0f}}, tint);
        x += icon_size_.x + separation;
    }

    if (text_.empty()) return;
    const float y = content.position.y + (content.size.y - measured.y) * 0.5f;
    canvas.draw_text({x, y}, text_, font_size, modulated(theme_color(color_name()), tint));
}

CheckBox::CheckBox() {
    set_toggle_mode(true);
}

CheckBox::CheckBox(std::string text) : text_{std::move(text)} {
    set_toggle_mode(true);
}

void CheckBox::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("CheckBox");
    BaseButton::collect_theme_types(out);
}

void CheckBox::set_text(std::string text) {
    if (text_ == text) return;
    text_ = std::move(text);
    update_minimum_size();
}

Vec2 CheckBox::get_minimum_size() const {
    const float font_size = theme_font_size("font_size");
    const float box = theme_constant("box_size");
    const Vec2 measured = canvas().measure_text(text_, font_size);
    Vec2 content{box, std::max(box, measured.y)};
    if (!text_.empty()) content.x += theme_constant("h_separation") + measured.x;
    return content + theme_stylebox("normal").content_margins.size();
}

void CheckBox::draw(Canvas& canvas) const {
    const Color4 tint = effective_modulate();
    const Rect2 area{{}, size()};
    const StyleBox& normal = theme_stylebox("normal");
    normal.draw(canvas, area, tint);
    if (has_focus()) theme_stylebox("focus").draw(canvas, area, tint);

    const Rect2 content = normal.content_rect(area);
    const float box = theme_constant("box_size");
    const Rect2 box_rect{{content.position.x, content.position.y + (content.size.y - box) * 0.5f}, {box, box}};
    canvas.fill_rect(box_rect, modulated(theme_color("box_color"), tint), 3.0f);
    canvas.stroke_rect(box_rect, modulated(theme_color("box_border_color"), tint), Stroke{1.0f, 3.0f});
    if (pressed()) {
        const Rect2 mark = grow(box_rect, -box * 0.25f);
        canvas.fill_rect(mark, modulated(theme_color("check_color"), tint), 2.0f);
    }

    if (text_.empty()) return;
    const float font_size = theme_font_size("font_size");
    const Vec2 measured = canvas.measure_text(text_, font_size);
    const std::string_view color_key = disabled() ? "font_disabled_color" : "font_color";
    canvas.draw_text({box_rect.right() + theme_constant("h_separation"), content.position.y + (content.size.y - measured.y) * 0.5f}, text_, font_size, modulated(theme_color(color_key), tint));
}

void Range::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("Range");
    Control::collect_theme_types(out);
}

void Range::set_range(double minimum, double maximum, double value) {
    minimum_ = minimum;
    maximum_ = std::max(maximum, minimum_);
    set_value(value);
}

void Range::set_value(double value) {
    const double upper = std::max(minimum_, maximum_ - page_);
    value_ = std::clamp(value, minimum_, upper);
}

double Range::ratio() const noexcept {
    const double span = maximum_ - page_ - minimum_;
    if (span <= 0.0) return 0.0;
    return std::clamp((value_ - minimum_) / span, 0.0, 1.0);
}

Slider::Slider(bool vertical) : vertical_{vertical} {}

void Slider::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("Slider");
    Range::collect_theme_types(out);
}

float Slider::grabber_size() const {
    return theme_constant("grabber_size");
}

Vec2 Slider::get_minimum_size() const {
    const float grabber = grabber_size();
    return vertical_ ? Vec2{grabber, grabber * 2.0f} : Vec2{grabber * 2.0f, grabber};
}

Rect2 Slider::grabber_rect() const {
    const float grabber = grabber_size();
    const auto amount = static_cast<float>(ratio());
    if (vertical_) {
        const float travel = std::max(size().y - grabber, 0.0f);
        const float y = travel * (1.0f - amount);
        return Rect2{{(size().x - grabber) * 0.5f, y}, {grabber, grabber}};
    }
    const float travel = std::max(size().x - grabber, 0.0f);
    return Rect2{{travel * amount, (size().y - grabber) * 0.5f}, {grabber, grabber}};
}

void Slider::draw(Canvas& canvas) const {
    const Color4 tint = effective_modulate();
    const float thickness = theme_constant("thickness");
    const float grabber = grabber_size();
    Rect2 track;
    if (vertical_) {
        track = Rect2{{(size().x - thickness) * 0.5f, grabber * 0.5f}, {thickness, std::max(size().y - grabber, 0.0f)}};
    } else {
        track = Rect2{{grabber * 0.5f, (size().y - thickness) * 0.5f}, {std::max(size().x - grabber, 0.0f), thickness}};
    }
    theme_stylebox("slider").draw(canvas, track, tint);

    Rect2 filled = track;
    const auto amount = static_cast<float>(ratio());
    if (vertical_) {
        filled.size.y = track.size.y * amount;
        filled.position.y = track.position.y + track.size.y - filled.size.y;
    } else {
        filled.size.x = track.size.x * amount;
    }
    theme_stylebox("grabber_area").draw(canvas, filled, tint);

    const Color4 knob = hovered() ? theme_color("grabber_hover_color") : theme_color("grabber_color");
    const Rect2 knob_rect = grabber_rect();
    canvas.fill_rect(knob_rect, modulated(knob, tint), knob_rect.size.x * 0.5f);
}

void LineEdit::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("LineEdit");
    Control::collect_theme_types(out);
}

void LineEdit::update_scroll() {
    const float font_size = theme_font_size("font_size");
    const Canvas& target = canvas();
    const Rect2 content = theme_stylebox("normal").content_rect(Rect2{{}, size()});
    const std::string_view prefix = std::string_view{text_}.substr(0, caret_);
    const float caret_x = target.measure_text(prefix, font_size).x;
    if (caret_x - scroll_ > content.size.x) scroll_ = caret_x - content.size.x;
    if (caret_x - scroll_ < 0.0f) scroll_ = caret_x;
    const float width = target.measure_text(text_, font_size).x;
    scroll_ = std::clamp(scroll_, 0.0f, std::max(width - content.size.x, 0.0f));
}

void LineEdit::set_text(std::string text) {
    text_ = std::move(text);
    caret_ = text_.size();
    update_scroll();
}

void LineEdit::set_placeholder(std::string placeholder) {
    placeholder_ = std::move(placeholder);
}

void LineEdit::set_editable(bool editable) {
    editable_ = editable;
}

Vec2 LineEdit::get_minimum_size() const {
    const float font_size = theme_font_size("font_size");
    const Vec2 line{theme_constant("minimum_width"), canvas().metrics(font_size).line_height};
    return line + theme_stylebox("normal").content_margins.size();
}

void LineEdit::draw(Canvas& canvas) const {
    const Color4 tint = effective_modulate();
    const Rect2 area{{}, size()};
    const std::string_view style = !editable_ ? "read_only" : (has_focus() ? "focus" : "normal");
    theme_stylebox(style).draw(canvas, area, tint);

    const StyleBox& frame = theme_stylebox("normal");
    const Rect2 content = frame.content_rect(area);
    const float font_size = theme_font_size("font_size");
    const std::string_view shown{text_};
    const float line_height = canvas.metrics(font_size).line_height;
    const float baseline = content.position.y + (content.size.y - line_height) * 0.5f;

    canvas.push_clip(Rect2{canvas.offset() + content.position, content.size});

    if (shown.empty() && !placeholder_.empty()) {
        canvas.draw_text({content.position.x, baseline}, placeholder_, font_size, modulated(theme_color("placeholder_color"), tint));
    }

    canvas.draw_text({content.position.x - scroll_, baseline}, shown, font_size, modulated(theme_color("font_color"), tint));

    if (has_focus() && editable_) {
        const std::string_view prefix = shown.substr(0, caret_);
        const float caret_x = canvas.measure_text(prefix, font_size).x;
        canvas.fill_rect(Rect2{{content.position.x + caret_x - scroll_, baseline}, {std::max(theme_constant("caret_width"), 1.0f), line_height}}, modulated(theme_color("caret_color"), tint));
    }

    canvas.pop_clip();
}

void LineEdit::resized() {
    update_scroll();
}

} // namespace bench::ui
