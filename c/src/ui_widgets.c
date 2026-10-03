#include "ui_widgets.h"

#include <stdlib.h>

/// Growable array of string views.
typedef ARRAY_OF(StrView) StrViewArray;

/// Larger of two doubles as `std::max` picks it: the first on a tie.
static double f64_max(double a, double b) { return a < b ? b : a; }

/// `v` limited to [lo, hi] as `std::clamp` does it.
static double f64_clamp(double v, double lo, double hi) { return v < lo ? lo : hi < v ? hi : v; }

/// The offset that places `used` within `available` for a horizontal alignment.
static float aligned_offset_horizontal(HorizontalAlignment alignment, float available, float used) {
    switch (alignment) {
    case UI_ALIGN_LEFT:
        return 0.0f;
    case UI_ALIGN_CENTER:
        return (available - used) * 0.5f;
    case UI_ALIGN_RIGHT:
        return available - used;
    }
    return 0.0f;
}

/// The offset that places `used` within `available` for a vertical alignment.
static float aligned_offset_vertical(VerticalAlignment alignment, float available, float used) {
    switch (alignment) {
    case UI_VALIGN_TOP:
        return 0.0f;
    case UI_VALIGN_CENTER:
        return (available - used) * 0.5f;
    case UI_VALIGN_BOTTOM:
        return available - used;
    }
    return 0.0f;
}

/// Replaces `target` with `text` and invalidates the minimum size of `control` when they differ.
static void replace_text(Control* control, OwnedStr* target, StrView text) {
    if (ui_sv_equal(ui_str_view(target), text)) return;
    ui_str_assign(target, text);
    ui_control_update_minimum_size(control);
}

/// Appends `Panel` and then `Control` to the theme types.
static void panel_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("Panel"));
    ui_control_collect_theme_types(control, out);
}

/// Draws the themed panel style box.
static void panel_draw(const Control* control, Canvas* canvas) {
    ui_stylebox_draw(UI_STYLEBOX(control, "panel"), canvas, ui_rect2(ui_vec2(0.0f, 0.0f), control->size), ui_control_effective_modulate(control));
}

/// Appends `Label` and then `Control` to the theme types.
static void label_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("Label"));
    ui_control_collect_theme_types(control, out);
}

/// The lines of the text, split at newlines, in an array the caller frees.
static StrViewArray label_lines(const Label* label) {
    StrViewArray result = { 0 };
    StrView remaining = ui_str_view(&label->text);
    while (true) {
        const char* newline = remaining.size == 0 ? NULL : memchr(remaining.data, '\n', remaining.size);
        if (newline == NULL) {
            ARRAY_PUSH(result, remaining);
            break;
        }
        size_t length = (size_t)(newline - remaining.data);
        StrView line = { remaining.data, length };
        ARRAY_PUSH(result, line);
        remaining = (StrView){ remaining.data + length + 1, remaining.size - length - 1 };
    }
    return result;
}

/// The width of the widest line and the summed line heights with spacing.
static Vec2 label_get_minimum_size(const Control* control) {
    float font_size = UI_FONT_SIZE(control, "font_size");
    float spacing = UI_CONSTANT(control, "line_spacing");
    const Canvas* target = ui_control_canvas(control);
    Vec2 result = ui_vec2(0.0f, 0.0f);
    StrViewArray entries = label_lines((const Label*)control);
    for (size_t index = 0; index < entries.len; ++index) {
        Vec2 measured = target->vt->measure_text(target, entries.data[index], font_size);
        result.x = f32_max(result.x, measured.x);
        result.y += measured.y;
        if (index + 1 < entries.len) result.y += spacing;
    }
    ARRAY_FREE(entries);
    return result;
}

/// Draws each line aligned inside the control rectangle.
static void label_draw(const Control* control, Canvas* canvas) {
    const Label* label = (const Label*)control;
    if (label->text.size == 0) return;
    float font_size = UI_FONT_SIZE(control, "font_size");
    float spacing = UI_CONSTANT(control, "line_spacing");
    Color4 color = ui_color_modulated(UI_COLOR(control, "font_color"), ui_control_effective_modulate(control));
    StrViewArray entries = label_lines(label);
    float line_height = canvas->vt->metrics(canvas, font_size).line_height;
    float total = line_height * (float)entries.len + spacing * (float)(entries.len - 1);
    float y = aligned_offset_vertical(label->vertical, control->size.y, total);
    for (size_t index = 0; index < entries.len; ++index) {
        StrView line = entries.data[index];
        Vec2 measured = canvas->vt->measure_text(canvas, line, font_size);
        float x = aligned_offset_horizontal(label->horizontal, control->size.x, measured.x);
        canvas->vt->draw_text(canvas, ui_vec2(x, y), line, font_size, color);
        y += line_height + spacing;
    }
    ARRAY_FREE(entries);
}

/// Releases the text, then the base control.
static void label_destroy(Control* control) {
    ui_str_free(&((Label*)control)->text);
    ui_control_destroy(control);
}

/// Appends `BaseButton` and then `Control` to the theme types.
static void base_button_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("BaseButton"));
    ui_control_collect_theme_types(control, out);
}

/// Appends `Button` and its bases to the theme types.
static void button_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("Button"));
    base_button_collect_theme_types(control, out);
}

/// The look a button control currently has.
static ButtonState button_state_of(const Control* control) {
    const BaseButton* button = (const BaseButton*)control;
    if (button->disabled) return UI_BUTTON_DISABLED;
    if (button->held || (button->toggle_mode && button->pressed)) return UI_BUTTON_PRESSED;
    if (control->hovered) return UI_BUTTON_HOVER;
    return UI_BUTTON_NORMAL;
}

/// The style box name for the current state.
static StrView button_style_name(const Control* control) {
    switch (button_state_of(control)) {
    case UI_BUTTON_DISABLED:
        return UI_SV("disabled");
    case UI_BUTTON_PRESSED:
        return UI_SV("pressed");
    case UI_BUTTON_HOVER:
        return UI_SV("hover");
    case UI_BUTTON_NORMAL:
        return UI_SV("normal");
    }
    return UI_SV("normal");
}

/// The font colour name for the current state.
static StrView button_color_name(const Control* control) {
    switch (button_state_of(control)) {
    case UI_BUTTON_DISABLED:
        return UI_SV("font_disabled_color");
    case UI_BUTTON_PRESSED:
        return UI_SV("font_pressed_color");
    case UI_BUTTON_HOVER:
        return UI_SV("font_hover_color");
    case UI_BUTTON_NORMAL:
        return UI_SV("font_color");
    }
    return UI_SV("font_color");
}

/// The text size plus the icon and the style content margins.
static Vec2 button_get_minimum_size(const Control* control) {
    const Button* button = (const Button*)control;
    float font_size = UI_FONT_SIZE(control, "font_size");
    const Canvas* target = ui_control_canvas(control);
    Vec2 measured = target->vt->measure_text(target, ui_str_view(&button->text), font_size);
    Vec2 content = measured;
    if (button->icon != UI_NO_TEXTURE) {
        content.x += button->icon_size.x;
        content.y = f32_max(content.y, button->icon_size.y);
        if (button->text.size != 0) content.x += UI_CONSTANT(control, "h_separation");
    }
    return ui_vec2_add(content, ui_margins_size(UI_STYLEBOX(control, "normal")->content_margins));
}

/// Draws the frame, the icon and the text.
static void button_draw(const Control* control, Canvas* canvas) {
    const Button* button = (const Button*)control;
    StrView text = ui_str_view(&button->text);
    Color4 tint = ui_control_effective_modulate(control);
    Rect2 area = ui_rect2(ui_vec2(0.0f, 0.0f), control->size);
    ui_stylebox_draw(ui_control_theme_stylebox(control, button_style_name(control)), canvas, area, tint);
    if (ui_control_has_focus(control)) ui_stylebox_draw(UI_STYLEBOX(control, "focus"), canvas, area, tint);

    Rect2 content = ui_stylebox_content_rect(ui_control_theme_stylebox(control, button_style_name(control)), area);
    float font_size = UI_FONT_SIZE(control, "font_size");
    Vec2 measured = canvas->vt->measure_text(canvas, text, font_size);
    bool has_icon = button->icon != UI_NO_TEXTURE;
    float separation = (has_icon && text.size != 0) ? UI_CONSTANT(control, "h_separation") : 0.0f;
    float used = measured.x + separation + (has_icon ? button->icon_size.x : 0.0f);

    float x = content.position.x;
    switch (button->horizontal) {
    case UI_ALIGN_LEFT:
        break;
    case UI_ALIGN_CENTER:
        x += (content.size.x - used) * 0.5f;
        break;
    case UI_ALIGN_RIGHT:
        x += content.size.x - used;
        break;
    }

    if (has_icon) {
        Rect2 icon_rect = ui_rect2(ui_vec2(x, content.position.y + (content.size.y - button->icon_size.y) * 0.5f), button->icon_size);
        Rect2 whole = ui_rect2(ui_vec2(0.0f, 0.0f), ui_vec2(1.0f, 1.0f));
        canvas->vt->fill_texture_rect(canvas, icon_rect, button->icon, whole, tint);
        x += button->icon_size.x + separation;
    }

    if (text.size == 0) return;
    float y = content.position.y + (content.size.y - measured.y) * 0.5f;
    canvas->vt->draw_text(canvas, ui_vec2(x, y), text, font_size, ui_color_modulated(ui_control_theme_color(control, button_color_name(control)), tint));
}

/// Releases the text, then the base control.
static void button_destroy(Control* control) {
    ui_str_free(&((Button*)control)->text);
    ui_control_destroy(control);
}

/// Appends `CheckBox` and its bases to the theme types.
static void check_box_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("CheckBox"));
    base_button_collect_theme_types(control, out);
}

/// The box and the text plus the style content margins.
static Vec2 check_box_get_minimum_size(const Control* control) {
    const CheckBox* box_control = (const CheckBox*)control;
    float font_size = UI_FONT_SIZE(control, "font_size");
    float box = UI_CONSTANT(control, "box_size");
    const Canvas* target = ui_control_canvas(control);
    Vec2 measured = target->vt->measure_text(target, ui_str_view(&box_control->text), font_size);
    Vec2 content = ui_vec2(box, f32_max(box, measured.y));
    if (box_control->text.size != 0) content.x += UI_CONSTANT(control, "h_separation") + measured.x;
    return ui_vec2_add(content, ui_margins_size(UI_STYLEBOX(control, "normal")->content_margins));
}

/// Draws the frame, the box with its check mark and the text.
static void check_box_draw(const Control* control, Canvas* canvas) {
    const CheckBox* box_control = (const CheckBox*)control;
    StrView text = ui_str_view(&box_control->text);
    Color4 tint = ui_control_effective_modulate(control);
    Rect2 area = ui_rect2(ui_vec2(0.0f, 0.0f), control->size);
    const StyleBox* normal = UI_STYLEBOX(control, "normal");
    ui_stylebox_draw(normal, canvas, area, tint);
    if (ui_control_has_focus(control)) ui_stylebox_draw(UI_STYLEBOX(control, "focus"), canvas, area, tint);

    Rect2 content = ui_stylebox_content_rect(normal, area);
    float box = UI_CONSTANT(control, "box_size");
    Rect2 box_rect = ui_rect2(ui_vec2(content.position.x, content.position.y + (content.size.y - box) * 0.5f), ui_vec2(box, box));
    canvas->vt->fill_rect(canvas, box_rect, ui_color_modulated(UI_COLOR(control, "box_color"), tint), 3.0f);
    Stroke outline = { 1.0f, 3.0f };
    canvas->vt->stroke_rect(canvas, box_rect, ui_color_modulated(UI_COLOR(control, "box_border_color"), tint), outline);
    if (box_control->base.pressed) {
        Rect2 mark = ui_rect2_grow(&box_rect, -box * 0.25f);
        canvas->vt->fill_rect(canvas, mark, ui_color_modulated(UI_COLOR(control, "check_color"), tint), 2.0f);
    }

    if (text.size == 0) return;
    float font_size = UI_FONT_SIZE(control, "font_size");
    Vec2 measured = canvas->vt->measure_text(canvas, text, font_size);
    StrView color_key = box_control->base.disabled ? UI_SV("font_disabled_color") : UI_SV("font_color");
    Vec2 origin = ui_vec2(ui_rect2_right(&box_rect) + UI_CONSTANT(control, "h_separation"), content.position.y + (content.size.y - measured.y) * 0.5f);
    canvas->vt->draw_text(canvas, origin, text, font_size, ui_color_modulated(ui_control_theme_color(control, color_key), tint));
}

/// Releases the text, then the base control.
static void check_box_destroy(Control* control) {
    ui_str_free(&((CheckBox*)control)->text);
    ui_control_destroy(control);
}

/// Appends `Range` and then `Control` to the theme types.
static void range_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("Range"));
    ui_control_collect_theme_types(control, out);
}

/// Appends `Slider` and its bases to the theme types.
static void slider_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("Slider"));
    range_collect_theme_types(control, out);
}

/// The themed grabber size.
static float slider_grabber_size(const Control* control) { return UI_CONSTANT(control, "grabber_size"); }

/// Twice the grabber along the track and one grabber across it.
static Vec2 slider_get_minimum_size(const Control* control) {
    float grabber = slider_grabber_size(control);
    return ((const Slider*)control)->vertical ? ui_vec2(grabber, grabber * 2.0f) : ui_vec2(grabber * 2.0f, grabber);
}

/// The rectangle of the grabber.
static Rect2 slider_grabber_rect(const Control* control) {
    const Slider* slider = (const Slider*)control;
    float grabber = slider_grabber_size(control);
    float amount = (float)ui_range_ratio(&slider->base);
    if (slider->vertical) {
        float travel = f32_max(control->size.y - grabber, 0.0f);
        float y = travel * (1.0f - amount);
        return ui_rect2(ui_vec2((control->size.x - grabber) * 0.5f, y), ui_vec2(grabber, grabber));
    }
    float travel = f32_max(control->size.x - grabber, 0.0f);
    return ui_rect2(ui_vec2(travel * amount, (control->size.y - grabber) * 0.5f), ui_vec2(grabber, grabber));
}

/// Draws the track, the filled part and the grabber.
static void slider_draw(const Control* control, Canvas* canvas) {
    const Slider* slider = (const Slider*)control;
    bool vertical = slider->vertical;
    Color4 tint = ui_control_effective_modulate(control);
    float thickness = UI_CONSTANT(control, "thickness");
    float grabber = slider_grabber_size(control);
    Rect2 track = { { 0 }, { 0 } };
    if (vertical) {
        track = ui_rect2(ui_vec2((control->size.x - thickness) * 0.5f, grabber * 0.5f), ui_vec2(thickness, f32_max(control->size.y - grabber, 0.0f)));
    } else {
        track = ui_rect2(ui_vec2(grabber * 0.5f, (control->size.y - thickness) * 0.5f), ui_vec2(f32_max(control->size.x - grabber, 0.0f), thickness));
    }
    ui_stylebox_draw(UI_STYLEBOX(control, "slider"), canvas, track, tint);

    Rect2 filled = track;
    float amount = (float)ui_range_ratio(&slider->base);
    if (vertical) {
        filled.size.y = track.size.y * amount;
        filled.position.y = track.position.y + track.size.y - filled.size.y;
    } else {
        filled.size.x = track.size.x * amount;
    }
    ui_stylebox_draw(UI_STYLEBOX(control, "grabber_area"), canvas, filled, tint);

    Color4 knob = control->hovered ? UI_COLOR(control, "grabber_hover_color") : UI_COLOR(control, "grabber_color");
    Rect2 knob_rect = slider_grabber_rect(control);
    canvas->vt->fill_rect(canvas, knob_rect, ui_color_modulated(knob, tint), knob_rect.size.x * 0.5f);
}

/// Appends `LineEdit` and then `Control` to the theme types.
static void line_edit_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("LineEdit"));
    ui_control_collect_theme_types(control, out);
}

/// The first `caret` bytes of `text`, or all of it when it is shorter.
static StrView prefix_of(StrView text, size_t caret) { return (StrView){ text.data, caret < text.size ? caret : text.size }; }

/// Scrolls so that the caret is inside the content area.
static void line_edit_update_scroll(LineEdit* edit) {
    const Control* control = &edit->base;
    float font_size = UI_FONT_SIZE(control, "font_size");
    const Canvas* target = ui_control_canvas(control);
    Rect2 content = ui_stylebox_content_rect(UI_STYLEBOX(control, "normal"), ui_rect2(ui_vec2(0.0f, 0.0f), control->size));
    StrView text = ui_str_view(&edit->text);
    float caret_x = target->vt->measure_text(target, prefix_of(text, edit->caret), font_size).x;
    if (caret_x - edit->scroll > content.size.x) edit->scroll = caret_x - content.size.x;
    if (caret_x - edit->scroll < 0.0f) edit->scroll = caret_x;
    float width = target->vt->measure_text(target, text, font_size).x;
    edit->scroll = f32_clamp(edit->scroll, 0.0f, f32_max(width - content.size.x, 0.0f));
}

/// One line of text height plus the style content margins, at the themed minimum width.
static Vec2 line_edit_get_minimum_size(const Control* control) {
    float font_size = UI_FONT_SIZE(control, "font_size");
    const Canvas* target = ui_control_canvas(control);
    Vec2 line = ui_vec2(UI_CONSTANT(control, "minimum_width"), target->vt->metrics(target, font_size).line_height);
    return ui_vec2_add(line, ui_margins_size(UI_STYLEBOX(control, "normal")->content_margins));
}

/// Draws the frame, the placeholder or text clipped to the content, and the caret.
static void line_edit_draw(const Control* control, Canvas* canvas) {
    const LineEdit* edit = (const LineEdit*)control;
    Color4 tint = ui_control_effective_modulate(control);
    Rect2 area = ui_rect2(ui_vec2(0.0f, 0.0f), control->size);
    bool focused = ui_control_has_focus(control);
    StrView style = !edit->editable ? UI_SV("read_only") : (focused ? UI_SV("focus") : UI_SV("normal"));
    ui_stylebox_draw(ui_control_theme_stylebox(control, style), canvas, area, tint);

    const StyleBox* frame = UI_STYLEBOX(control, "normal");
    Rect2 content = ui_stylebox_content_rect(frame, area);
    float font_size = UI_FONT_SIZE(control, "font_size");
    StrView shown = ui_str_view(&edit->text);
    float line_height = canvas->vt->metrics(canvas, font_size).line_height;
    float baseline = content.position.y + (content.size.y - line_height) * 0.5f;

    canvas->vt->push_clip(canvas, ui_rect2(ui_vec2_add(canvas->offset, content.position), content.size));

    if (shown.size == 0 && edit->placeholder.size != 0) {
        canvas->vt->draw_text(canvas, ui_vec2(content.position.x, baseline), ui_str_view(&edit->placeholder), font_size,
                ui_color_modulated(UI_COLOR(control, "placeholder_color"), tint));
    }

    canvas->vt->draw_text(canvas, ui_vec2(content.position.x - edit->scroll, baseline), shown, font_size,
            ui_color_modulated(UI_COLOR(control, "font_color"), tint));

    if (focused && edit->editable) {
        StrView prefix = prefix_of(shown, edit->caret);
        float caret_x = canvas->vt->measure_text(canvas, prefix, font_size).x;
        Rect2 caret = ui_rect2(ui_vec2(content.position.x + caret_x - edit->scroll, baseline),
                ui_vec2(f32_max(UI_CONSTANT(control, "caret_width"), 1.0f), line_height));
        canvas->vt->fill_rect(canvas, caret, ui_color_modulated(UI_COLOR(control, "caret_color"), tint), 0.0f);
    }

    canvas->vt->pop_clip(canvas);
}

/// Rescrolls after the size changes.
static void line_edit_resized(Control* control) { line_edit_update_scroll((LineEdit*)control); }

/// Releases the text and the placeholder, then the base control.
static void line_edit_destroy(Control* control) {
    LineEdit* edit = (LineEdit*)control;
    ui_str_free(&edit->text);
    ui_str_free(&edit->placeholder);
    ui_control_destroy(control);
}

static const ControlVTable panel_vtable = {
    .destroy = ui_control_destroy,
    .collect_theme_types = panel_collect_theme_types,
    .get_minimum_size = ui_control_get_minimum_size,
    .draw = panel_draw,
    .layout_children = ui_control_layout_children,
    .resized = ui_control_resized,
    .sort_children = NULL,
};

static const ControlVTable label_vtable = {
    .destroy = label_destroy,
    .collect_theme_types = label_collect_theme_types,
    .get_minimum_size = label_get_minimum_size,
    .draw = label_draw,
    .layout_children = ui_control_layout_children,
    .resized = ui_control_resized,
    .sort_children = NULL,
};

static const ControlVTable button_vtable = {
    .destroy = button_destroy,
    .collect_theme_types = button_collect_theme_types,
    .get_minimum_size = button_get_minimum_size,
    .draw = button_draw,
    .layout_children = ui_control_layout_children,
    .resized = ui_control_resized,
    .sort_children = NULL,
};

static const ControlVTable check_box_vtable = {
    .destroy = check_box_destroy,
    .collect_theme_types = check_box_collect_theme_types,
    .get_minimum_size = check_box_get_minimum_size,
    .draw = check_box_draw,
    .layout_children = ui_control_layout_children,
    .resized = ui_control_resized,
    .sort_children = NULL,
};

static const ControlVTable slider_vtable = {
    .destroy = ui_control_destroy,
    .collect_theme_types = slider_collect_theme_types,
    .get_minimum_size = slider_get_minimum_size,
    .draw = slider_draw,
    .layout_children = ui_control_layout_children,
    .resized = ui_control_resized,
    .sort_children = NULL,
};

static const ControlVTable line_edit_vtable = {
    .destroy = line_edit_destroy,
    .collect_theme_types = line_edit_collect_theme_types,
    .get_minimum_size = line_edit_get_minimum_size,
    .draw = line_edit_draw,
    .layout_children = ui_control_layout_children,
    .resized = line_edit_resized,
    .sort_children = NULL,
};

Panel* ui_panel_add(Control* parent) { return (Panel*)ui_control_add_new(parent, sizeof(Panel), &panel_vtable); }

Label* ui_label_add(Control* parent, StrView text) {
    Label* label = (Label*)ui_control_add_new(parent, sizeof(Label), &label_vtable);
    ui_str_assign(&label->text, text);
    return label;
}

void ui_label_set_text(Label* label, StrView text) { replace_text(&label->base, &label->text, text); }

void ui_label_set_horizontal_alignment(Label* label, HorizontalAlignment alignment) { label->horizontal = alignment; }

void ui_label_set_vertical_alignment(Label* label, VerticalAlignment alignment) { label->vertical = alignment; }

void ui_base_button_set_disabled(BaseButton* button, bool disabled) {
    button->disabled = disabled;
    if (button->disabled) button->held = false;
}

void ui_base_button_set_toggle_mode(BaseButton* button, bool toggle) { button->toggle_mode = toggle; }

void ui_base_button_set_pressed(BaseButton* button, bool pressed) { button->pressed = pressed; }

void ui_base_button_set_held(BaseButton* button, bool held) { button->held = held; }

ButtonState ui_base_button_state(const BaseButton* button) { return button_state_of(&button->base); }

Button* ui_button_add(Control* parent, StrView text) {
    Button* button = (Button*)ui_control_add_new(parent, sizeof(Button), &button_vtable);
    button->horizontal = UI_ALIGN_CENTER;
    ui_str_assign(&button->text, text);
    return button;
}

void ui_button_set_text(Button* button, StrView text) { replace_text(&button->base.base, &button->text, text); }

void ui_button_set_icon(Button* button, TextureId icon, Vec2 size) {
    button->icon = icon;
    button->icon_size = size;
    ui_control_update_minimum_size(&button->base.base);
}

void ui_button_set_horizontal_alignment(Button* button, HorizontalAlignment alignment) { button->horizontal = alignment; }

CheckBox* ui_check_box_add(Control* parent, StrView text) {
    CheckBox* box = (CheckBox*)ui_control_add_new(parent, sizeof(CheckBox), &check_box_vtable);
    ui_base_button_set_toggle_mode(&box->base, true);
    ui_str_assign(&box->text, text);
    return box;
}

void ui_check_box_set_text(CheckBox* box, StrView text) { replace_text(&box->base.base, &box->text, text); }

void ui_range_set_range(Range* range, double minimum, double maximum, double value) {
    range->minimum = minimum;
    range->maximum = f64_max(maximum, range->minimum);
    ui_range_set_value(range, value);
}

void ui_range_set_value(Range* range, double value) {
    double upper = f64_max(range->minimum, range->maximum - range->page);
    range->value = f64_clamp(value, range->minimum, upper);
}

double ui_range_ratio(const Range* range) {
    double span = range->maximum - range->page - range->minimum;
    if (span <= 0.0) return 0.0;
    return f64_clamp((range->value - range->minimum) / span, 0.0, 1.0);
}

Slider* ui_slider_add(Control* parent, bool vertical) {
    Slider* slider = (Slider*)ui_control_add_new(parent, sizeof(Slider), &slider_vtable);
    slider->base.maximum = 100.0;
    slider->vertical = vertical;
    return slider;
}

LineEdit* ui_line_edit_add(Control* parent) {
    LineEdit* edit = (LineEdit*)ui_control_add_new(parent, sizeof(LineEdit), &line_edit_vtable);
    edit->editable = true;
    return edit;
}

void ui_line_edit_set_text(LineEdit* edit, StrView text) {
    ui_str_assign(&edit->text, text);
    edit->caret = edit->text.size;
    line_edit_update_scroll(edit);
}

void ui_line_edit_set_placeholder(LineEdit* edit, StrView placeholder) { ui_str_assign(&edit->placeholder, placeholder); }

void ui_line_edit_set_editable(LineEdit* edit, bool editable) { edit->editable = editable; }
