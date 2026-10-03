#ifndef BENCH_UI_WIDGETS_H
#define BENCH_UI_WIDGETS_H

#include <stdbool.h>
#include <stdint.h>

#include "ui_control.h"

/// The leaf widgets. Each class is a struct embedding its base class first
/// (`Control`, `BaseButton` or `Range`), so `(Control*)widget` and the
/// downcast `(Label*)control` are plain casts, and each has its own vtable.
/// Text is owned by the widget as a malloc'd copy.

/// Where text sits along a row.
typedef enum { UI_ALIGN_LEFT, UI_ALIGN_CENTER, UI_ALIGN_RIGHT } HorizontalAlignment;

/// Where text sits along a column.
typedef enum { UI_VALIGN_TOP, UI_VALIGN_CENTER, UI_VALIGN_BOTTOM } VerticalAlignment;

/// A control that draws only its themed panel style box.
typedef struct {
    Control base;
} Panel;

/// Static text of one or more lines.
typedef struct {
    Control base;
    OwnedStr text;
    HorizontalAlignment horizontal;
    VerticalAlignment vertical;
} Label;

/// How a button looks.
typedef enum { UI_BUTTON_NORMAL, UI_BUTTON_HOVER, UI_BUTTON_PRESSED, UI_BUTTON_DISABLED } ButtonState;

/// Shared state of pressable controls.
typedef struct {
    Control base;
    bool disabled;
    bool toggle_mode;
    bool pressed;
    bool held;
} BaseButton;

/// A push button with a text and an optional icon.
typedef struct {
    BaseButton base;
    OwnedStr text;
    TextureId icon;
    Vec2 icon_size;
    HorizontalAlignment horizontal;
} Button;

/// A toggle button drawn as a box with a label.
typedef struct {
    BaseButton base;
    OwnedStr text;
} CheckBox;

/// A value between a minimum and a maximum.
typedef struct {
    Control base;
    double minimum;
    double maximum;
    double page;
    double value;
} Range;

/// A draggable grabber along a track.
typedef struct {
    Range base;
    bool vertical;
} Slider;

/// A single-line text field.
typedef struct {
    Control base;
    OwnedStr text;
    OwnedStr placeholder;
    bool editable;
    size_t caret;
    float scroll;
} LineEdit;

/// A pointer to the `Control` base of any widget.
#define UI_CONTROL(widget) ((Control*)(widget))

/// Adds a panel.
Panel* ui_panel_add(Control* parent);

/// Adds a label showing `text`.
Label* ui_label_add(Control* parent, StrView text);

/// Replaces the text, invalidating the minimum size when it differs.
void ui_label_set_text(Label* label, StrView text);

/// Sets the horizontal alignment.
void ui_label_set_horizontal_alignment(Label* label, HorizontalAlignment alignment);

/// Sets the vertical alignment.
void ui_label_set_vertical_alignment(Label* label, VerticalAlignment alignment);

/// Sets whether the button ignores input.
void ui_base_button_set_disabled(BaseButton* button, bool disabled);

/// Sets whether pressing toggles the button.
void ui_base_button_set_toggle_mode(BaseButton* button, bool toggle);

/// Sets the toggled state.
void ui_base_button_set_pressed(BaseButton* button, bool pressed);

/// Sets whether the button is being held down.
void ui_base_button_set_held(BaseButton* button, bool held);

/// The look the button currently has.
ButtonState ui_base_button_state(const BaseButton* button);

/// Adds a button labelled `text`.
Button* ui_button_add(Control* parent, StrView text);

/// Replaces the text, invalidating the minimum size when it differs.
void ui_button_set_text(Button* button, StrView text);

/// Sets the icon texture and its size.
void ui_button_set_icon(Button* button, TextureId icon, Vec2 size);

/// Sets the horizontal alignment of the content.
void ui_button_set_horizontal_alignment(Button* button, HorizontalAlignment alignment);

/// Adds a check box labelled `text`.
CheckBox* ui_check_box_add(Control* parent, StrView text);

/// Replaces the text, invalidating the minimum size when it differs.
void ui_check_box_set_text(CheckBox* box, StrView text);

/// Sets the bounds and the value directly.
void ui_range_set_range(Range* range, double minimum, double maximum, double value);

/// Sets the value, clamped to the bounds.
void ui_range_set_value(Range* range, double value);

/// The value as a fraction of the range, in [0, 1].
double ui_range_ratio(const Range* range);

/// Adds a vertical slider when `vertical`, else a horizontal one.
Slider* ui_slider_add(Control* parent, bool vertical);

/// Adds an empty text field.
LineEdit* ui_line_edit_add(Control* parent);

/// Replaces the text, putting the caret at its end.
void ui_line_edit_set_text(LineEdit* edit, StrView text);

/// Sets the text shown while the field is empty.
void ui_line_edit_set_placeholder(LineEdit* edit, StrView placeholder);

/// Sets whether the field can be edited.
void ui_line_edit_set_editable(LineEdit* edit, bool editable);

#endif
