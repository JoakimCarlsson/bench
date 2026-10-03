#pragma once

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "ui_control.hpp"

namespace bench::ui {

/// Where text sits along a row.
enum class HorizontalAlignment : uint8_t { Left, Center, Right };

/// Where text sits along a column.
enum class VerticalAlignment : uint8_t { Top, Center, Bottom };

/// A control that draws only its themed panel style box.
class Panel final : public Control {
public:
    void collect_theme_types(std::vector<std::string_view>& out) const override;
    void draw(Canvas& canvas) const override;
};

/// Static text of one or more lines.
class Label final : public Control {
public:
    Label() = default;
    /// A label showing `text`.
    explicit Label(std::string text);

    void collect_theme_types(std::vector<std::string_view>& out) const override;
    Vec2 get_minimum_size() const override;
    void draw(Canvas& canvas) const override;

    /// Replaces the text, invalidating the minimum size when it differs.
    void set_text(std::string text);
    /// Sets the horizontal alignment.
    void set_horizontal_alignment(HorizontalAlignment alignment);
    /// Sets the vertical alignment.
    void set_vertical_alignment(VerticalAlignment alignment);

private:
    /// The lines of the text, split at newlines.
    std::vector<std::string_view> lines() const;

    std::string text_;
    HorizontalAlignment horizontal_{HorizontalAlignment::Left};
    VerticalAlignment vertical_{VerticalAlignment::Top};
};

/// How a button looks.
enum class ButtonState : uint8_t { Normal, Hover, Pressed, Disabled };

/// Shared state of pressable controls.
class BaseButton : public Control {
public:
    void collect_theme_types(std::vector<std::string_view>& out) const override;

    /// Sets whether the button ignores input.
    void set_disabled(bool disabled);
    /// Sets whether pressing toggles the button.
    void set_toggle_mode(bool toggle);
    /// Sets the toggled state.
    void set_pressed(bool pressed);
    /// Sets whether the button is being held down.
    void set_held(bool held);
    /// Whether the button is toggled on.
    bool pressed() const noexcept;
    /// Whether the button ignores input.
    bool disabled() const noexcept;
    /// The look the button currently has.
    ButtonState state() const noexcept;

private:
    bool disabled_{};
    bool toggle_mode_{};
    bool pressed_{};
    bool held_{};
};

/// A push button with a text and an optional icon.
class Button final : public BaseButton {
public:
    Button() = default;
    /// A button labelled `text`.
    explicit Button(std::string text);

    void collect_theme_types(std::vector<std::string_view>& out) const override;
    Vec2 get_minimum_size() const override;
    void draw(Canvas& canvas) const override;

    /// Replaces the text, invalidating the minimum size when it differs.
    void set_text(std::string text);
    /// Sets the icon texture and its size.
    void set_icon(TextureId icon, Vec2 size);
    /// Sets the horizontal alignment of the content.
    void set_horizontal_alignment(HorizontalAlignment alignment);

private:
    /// The style box name for the current state.
    std::string_view style_name() const;
    /// The font colour name for the current state.
    std::string_view color_name() const;

    std::string text_;
    TextureId icon_{no_texture};
    Vec2 icon_size_{};
    HorizontalAlignment horizontal_{HorizontalAlignment::Center};
};

/// A toggle button drawn as a box with a label.
class CheckBox final : public BaseButton {
public:
    CheckBox();
    /// A check box labelled `text`.
    explicit CheckBox(std::string text);

    void collect_theme_types(std::vector<std::string_view>& out) const override;
    Vec2 get_minimum_size() const override;
    void draw(Canvas& canvas) const override;

    /// Replaces the text, invalidating the minimum size when it differs.
    void set_text(std::string text);

private:
    std::string text_;
};

/// A value between a minimum and a maximum.
class Range : public Control {
public:
    void collect_theme_types(std::vector<std::string_view>& out) const override;

    /// Sets the bounds and the value directly.
    void set_range(double minimum, double maximum, double value);
    /// Sets the value, clamped to the bounds.
    void set_value(double value);
    /// The value as a fraction of the range, in [0, 1].
    double ratio() const noexcept;

private:
    double minimum_{0.0};
    double maximum_{100.0};
    double page_{0.0};
    double value_{0.0};
};

/// A draggable grabber along a track.
class Slider final : public Range {
public:
    /// A vertical slider when `vertical`, else a horizontal one.
    explicit Slider(bool vertical = false);

    void collect_theme_types(std::vector<std::string_view>& out) const override;
    Vec2 get_minimum_size() const override;
    void draw(Canvas& canvas) const override;

private:
    /// The themed grabber size.
    float grabber_size() const;
    /// The rectangle of the grabber.
    Rect2 grabber_rect() const;

    bool vertical_{};
};

/// A single-line text field.
class LineEdit final : public Control {
public:
    void collect_theme_types(std::vector<std::string_view>& out) const override;
    Vec2 get_minimum_size() const override;
    void draw(Canvas& canvas) const override;
    void resized() override;

    /// Replaces the text, putting the caret at its end.
    void set_text(std::string text);
    /// Sets the text shown while the field is empty.
    void set_placeholder(std::string placeholder);
    /// Sets whether the field can be edited.
    void set_editable(bool editable);

private:
    /// Scrolls so that the caret is inside the content area.
    void update_scroll();

    std::string text_;
    std::string placeholder_;
    bool editable_{true};
    size_t caret_{};
    float scroll_{};
};

} // namespace bench::ui
