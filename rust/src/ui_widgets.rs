//! The widgets that draw themselves: panel, label, button, check box,
//! slider and line edit. Text and values that change after the control is
//! built sit in `Cell` and `RefCell` fields; the rest is fixed when the
//! widget is constructed.
use crate::ui_control::{Control, Widget, control_types};
use crate::ui_draw::{Canvas, NO_TEXTURE, Stroke, TextureId};
use crate::ui_math::{self, Rect2, Vec2};
use std::cell::{Cell, RefCell};

/// Where text sits along a row.
#[derive(Clone, Copy)]
pub enum HorizontalAlignment {
    Left,
    Center,
    Right,
}

/// Where text sits along a column.
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum VerticalAlignment {
    Top,
    Center,
    Bottom,
}

/// The offset that places `used` within `available` for a horizontal alignment.
fn horizontal_offset(alignment: HorizontalAlignment, available: f32, used: f32) -> f32 {
    match alignment {
        HorizontalAlignment::Left => 0.0,
        HorizontalAlignment::Center => (available - used) * 0.5,
        HorizontalAlignment::Right => available - used,
    }
}

/// The offset that places `used` within `available` for a vertical alignment.
fn vertical_offset(alignment: VerticalAlignment, available: f32, used: f32) -> f32 {
    match alignment {
        VerticalAlignment::Top => 0.0,
        VerticalAlignment::Center => (available - used) * 0.5,
        VerticalAlignment::Bottom => available - used,
    }
}

/// A control that draws only its themed panel style box.
pub struct Panel;

impl Widget for Panel {
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>) {
        out.push("Panel");
        control_types(out);
    }

    fn draw(&self, control: &Control, canvas: &mut dyn Canvas) {
        control.theme_stylebox("panel").draw(canvas, &Rect2::new(Vec2::default(), control.size()), control.effective_modulate());
    }
}

/// Static text of one or more lines.
pub struct Label {
    text: RefCell<String>,
    horizontal: HorizontalAlignment,
    vertical: VerticalAlignment,
}

impl Label {
    /// A label showing `text`, aligned top left.
    pub fn new(text: String) -> Label {
        Label { text: RefCell::new(text), horizontal: HorizontalAlignment::Left, vertical: VerticalAlignment::Top }
    }

    /// This label with its horizontal alignment replaced.
    pub fn with_horizontal_alignment(mut self, alignment: HorizontalAlignment) -> Label {
        self.horizontal = alignment;
        self
    }

    /// Replaces the text, invalidating the minimum size when it differs.
    pub fn set_text(&self, control: &Control, text: String) {
        if *self.text.borrow() == text {
            return;
        }
        *self.text.borrow_mut() = text;
        control.update_minimum_size();
    }
}

impl Widget for Label {
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>) {
        out.push("Label");
        control_types(out);
    }

    fn get_minimum_size(&self, control: &Control) -> Vec2 {
        let font_size = control.theme_font_size("font_size");
        let spacing = control.theme_constant("line_spacing");
        let mut result = Vec2::default();
        let text = self.text.borrow();
        let entries: Vec<&str> = text.split('\n').collect();
        for index in 0..entries.len() {
            let measured = control.measure_text(entries[index], font_size);
            result.x = ui_math::max(result.x, measured.x);
            result.y += measured.y;
            if index + 1 < entries.len() {
                result.y += spacing;
            }
        }
        result
    }

    fn draw(&self, control: &Control, canvas: &mut dyn Canvas) {
        let text = self.text.borrow();
        if text.is_empty() {
            return;
        }
        let font_size = control.theme_font_size("font_size");
        let spacing = control.theme_constant("line_spacing");
        let color = control.theme_color("font_color").modulated(control.effective_modulate());
        let entries: Vec<&str> = text.split('\n').collect();
        let line_height = canvas.metrics(font_size).line_height;
        let total = line_height * entries.len() as f32 + spacing * (entries.len() - 1) as f32;
        let size = control.size();
        let mut y = vertical_offset(self.vertical, size.y, total);
        for line in entries {
            let measured = canvas.measure_text(line, font_size);
            let x = horizontal_offset(self.horizontal, size.x, measured.x);
            canvas.draw_text(Vec2::new(x, y), line, font_size, color);
            y += line_height + spacing;
        }
    }
}

/// How a button looks.
#[derive(Clone, Copy)]
enum ButtonState {
    Normal,
    Hover,
    Pressed,
    Disabled,
}

/// Shared state of pressable controls, the engine's `BaseButton`.
#[derive(Default)]
struct ButtonCore {
    disabled: bool,
    toggle_mode: bool,
    pressed: bool,
    held: bool,
}

impl ButtonCore {
    /// The look the button currently has, given whether the pointer is over it.
    fn state(&self, hovered: bool) -> ButtonState {
        if self.disabled {
            return ButtonState::Disabled;
        }
        if self.held || (self.toggle_mode && self.pressed) {
            return ButtonState::Pressed;
        }
        if hovered {
            return ButtonState::Hover;
        }
        ButtonState::Normal
    }
}

/// A push button with a text and an optional icon.
pub struct Button {
    core: ButtonCore,
    text: RefCell<String>,
    icon: TextureId,
    icon_size: Vec2,
    horizontal: HorizontalAlignment,
}

impl Button {
    /// A button labelled `text`, with its content centred.
    pub fn new(text: String) -> Button {
        Button { core: ButtonCore::default(), text: RefCell::new(text), icon: NO_TEXTURE, icon_size: Vec2::default(), horizontal: HorizontalAlignment::Center }
    }

    /// This button with an icon texture of the given size.
    pub fn with_icon(mut self, icon: TextureId, size: Vec2) -> Button {
        self.icon = icon;
        self.icon_size = size;
        self
    }

    /// This button with the horizontal alignment of its content replaced.
    pub fn with_horizontal_alignment(mut self, alignment: HorizontalAlignment) -> Button {
        self.horizontal = alignment;
        self
    }

    /// Sets whether the button ignores input.
    pub fn set_disabled(&mut self, disabled: bool) {
        self.core.disabled = disabled;
        if disabled {
            self.core.held = false;
        }
    }

    /// Replaces the text, invalidating the minimum size when it differs.
    pub fn set_text(&self, control: &Control, text: String) {
        if *self.text.borrow() == text {
            return;
        }
        *self.text.borrow_mut() = text;
        control.update_minimum_size();
    }

    /// The style box name for the current state.
    fn style_name(&self, control: &Control) -> &'static str {
        match self.core.state(control.hovered()) {
            ButtonState::Disabled => "disabled",
            ButtonState::Pressed => "pressed",
            ButtonState::Hover => "hover",
            ButtonState::Normal => "normal",
        }
    }

    /// The font colour name for the current state.
    fn color_name(&self, control: &Control) -> &'static str {
        match self.core.state(control.hovered()) {
            ButtonState::Disabled => "font_disabled_color",
            ButtonState::Pressed => "font_pressed_color",
            ButtonState::Hover => "font_hover_color",
            ButtonState::Normal => "font_color",
        }
    }
}

impl Widget for Button {
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>) {
        out.push("Button");
        out.push("BaseButton");
        control_types(out);
    }

    fn get_minimum_size(&self, control: &Control) -> Vec2 {
        let font_size = control.theme_font_size("font_size");
        let text = self.text.borrow();
        let mut content = control.measure_text(&text, font_size);
        if self.icon != NO_TEXTURE {
            content.x += self.icon_size.x;
            content.y = ui_math::max(content.y, self.icon_size.y);
            if !text.is_empty() {
                content.x += control.theme_constant("h_separation");
            }
        }
        content + control.theme_stylebox("normal").content_margins.size()
    }

    fn draw(&self, control: &Control, canvas: &mut dyn Canvas) {
        let tint = control.effective_modulate();
        let area = Rect2::new(Vec2::default(), control.size());
        control.theme_stylebox(self.style_name(control)).draw(canvas, &area, tint);
        if control.has_focus() {
            control.theme_stylebox("focus").draw(canvas, &area, tint);
        }

        let content = control.theme_stylebox(self.style_name(control)).content_rect(&area);
        let font_size = control.theme_font_size("font_size");
        let text = self.text.borrow();
        let measured = canvas.measure_text(&text, font_size);
        let separation = if self.icon != NO_TEXTURE && !text.is_empty() { control.theme_constant("h_separation") } else { 0.0 };
        let used = measured.x + separation + if self.icon != NO_TEXTURE { self.icon_size.x } else { 0.0 };

        let mut x = content.position.x;
        match self.horizontal {
            HorizontalAlignment::Left => {}
            HorizontalAlignment::Center => x += (content.size.x - used) * 0.5,
            HorizontalAlignment::Right => x += content.size.x - used,
        }

        if self.icon != NO_TEXTURE {
            let icon_rect = Rect2::new(Vec2::new(x, content.position.y + (content.size.y - self.icon_size.y) * 0.5), self.icon_size);
            canvas.fill_texture_rect(&icon_rect, self.icon, &Rect2::new(Vec2::new(0.0, 0.0), Vec2::new(1.0, 1.0)), tint);
            x += self.icon_size.x + separation;
        }

        if text.is_empty() {
            return;
        }
        let y = content.position.y + (content.size.y - measured.y) * 0.5;
        canvas.draw_text(Vec2::new(x, y), &text, font_size, control.theme_color(self.color_name(control)).modulated(tint));
    }
}

/// A toggle button drawn as a box with a label.
pub struct CheckBox {
    core: ButtonCore,
    text: RefCell<String>,
}

impl CheckBox {
    /// A check box labelled `text`, toggled on when `pressed`.
    pub fn new(text: String, pressed: bool) -> CheckBox {
        CheckBox { core: ButtonCore { toggle_mode: true, pressed, ..ButtonCore::default() }, text: RefCell::new(text) }
    }
}

impl Widget for CheckBox {
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>) {
        out.push("CheckBox");
        out.push("BaseButton");
        control_types(out);
    }

    fn get_minimum_size(&self, control: &Control) -> Vec2 {
        let font_size = control.theme_font_size("font_size");
        let box_size = control.theme_constant("box_size");
        let text = self.text.borrow();
        let measured = control.measure_text(&text, font_size);
        let mut content = Vec2::new(box_size, ui_math::max(box_size, measured.y));
        if !text.is_empty() {
            content.x += control.theme_constant("h_separation") + measured.x;
        }
        content + control.theme_stylebox("normal").content_margins.size()
    }

    fn draw(&self, control: &Control, canvas: &mut dyn Canvas) {
        let tint = control.effective_modulate();
        let area = Rect2::new(Vec2::default(), control.size());
        let normal = control.theme_stylebox("normal");
        normal.draw(canvas, &area, tint);
        if control.has_focus() {
            control.theme_stylebox("focus").draw(canvas, &area, tint);
        }

        let content = normal.content_rect(&area);
        let box_size = control.theme_constant("box_size");
        let box_rect = Rect2::new(
            Vec2::new(content.position.x, content.position.y + (content.size.y - box_size) * 0.5),
            Vec2::new(box_size, box_size),
        );
        canvas.fill_rect(&box_rect, control.theme_color("box_color").modulated(tint), 3.0);
        canvas.stroke_rect(&box_rect, control.theme_color("box_border_color").modulated(tint), Stroke { width: 1.0, corner_radius: 3.0 });
        if self.core.pressed {
            let mark = ui_math::grow(&box_rect, -box_size * 0.25);
            canvas.fill_rect(&mark, control.theme_color("check_color").modulated(tint), 2.0);
        }

        let text = self.text.borrow();
        if text.is_empty() {
            return;
        }
        let font_size = control.theme_font_size("font_size");
        let measured = canvas.measure_text(&text, font_size);
        let color_key = if self.core.disabled { "font_disabled_color" } else { "font_color" };
        canvas.draw_text(
            Vec2::new(box_rect.right() + control.theme_constant("h_separation"), content.position.y + (content.size.y - measured.y) * 0.5),
            &text,
            font_size,
            control.theme_color(color_key).modulated(tint),
        );
    }
}

/// A value between a minimum and a maximum, the engine's `Range`.
struct Range {
    minimum: f64,
    maximum: f64,
    page: f64,
    value: Cell<f64>,
}

impl Range {
    /// A range from 0 to 100 with value 0.
    fn new() -> Range {
        Range { minimum: 0.0, maximum: 100.0, page: 0.0, value: Cell::new(0.0) }
    }

    /// Sets the value, clamped to the bounds.
    fn set_value(&self, value: f64) {
        let upper = ui_math::max64(self.minimum, self.maximum - self.page);
        self.value.set(ui_math::clamp64(value, self.minimum, upper));
    }

    /// The value as a fraction of the range, in [0, 1].
    fn ratio(&self) -> f64 {
        let span = self.maximum - self.page - self.minimum;
        if span <= 0.0 {
            return 0.0;
        }
        ui_math::clamp64((self.value.get() - self.minimum) / span, 0.0, 1.0)
    }
}

/// A draggable grabber along a track.
pub struct Slider {
    range: Range,
    vertical: bool,
}

impl Slider {
    /// A horizontal slider from 0 to 100 with value 0.
    pub fn new() -> Slider {
        Slider { range: Range::new(), vertical: false }
    }

    /// This slider with the bounds and the value replaced.
    pub fn with_range(mut self, minimum: f64, maximum: f64, value: f64) -> Slider {
        self.range.minimum = minimum;
        self.range.maximum = ui_math::max64(maximum, minimum);
        self.range.set_value(value);
        self
    }

    /// Sets the value, clamped to the bounds.
    pub fn set_value(&self, value: f64) {
        self.range.set_value(value);
    }

    /// The themed grabber size.
    fn grabber_size(control: &Control) -> f32 {
        control.theme_constant("grabber_size")
    }

    /// The rectangle of the grabber.
    fn grabber_rect(&self, control: &Control) -> Rect2 {
        let grabber = Slider::grabber_size(control);
        let amount = self.range.ratio() as f32;
        let size = control.size();
        if self.vertical {
            let travel = ui_math::max(size.y - grabber, 0.0);
            let y = travel * (1.0 - amount);
            return Rect2::new(Vec2::new((size.x - grabber) * 0.5, y), Vec2::new(grabber, grabber));
        }
        let travel = ui_math::max(size.x - grabber, 0.0);
        Rect2::new(Vec2::new(travel * amount, (size.y - grabber) * 0.5), Vec2::new(grabber, grabber))
    }
}

impl Widget for Slider {
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>) {
        out.push("Slider");
        out.push("Range");
        control_types(out);
    }

    fn get_minimum_size(&self, control: &Control) -> Vec2 {
        let grabber = Slider::grabber_size(control);
        if self.vertical { Vec2::new(grabber, grabber * 2.0) } else { Vec2::new(grabber * 2.0, grabber) }
    }

    fn draw(&self, control: &Control, canvas: &mut dyn Canvas) {
        let tint = control.effective_modulate();
        let thickness = control.theme_constant("thickness");
        let grabber = Slider::grabber_size(control);
        let size = control.size();
        let track = if self.vertical {
            Rect2::new(Vec2::new((size.x - thickness) * 0.5, grabber * 0.5), Vec2::new(thickness, ui_math::max(size.y - grabber, 0.0)))
        } else {
            Rect2::new(Vec2::new(grabber * 0.5, (size.y - thickness) * 0.5), Vec2::new(ui_math::max(size.x - grabber, 0.0), thickness))
        };
        control.theme_stylebox("slider").draw(canvas, &track, tint);

        let mut filled = track;
        let amount = self.range.ratio() as f32;
        if self.vertical {
            filled.size.y = track.size.y * amount;
            filled.position.y = track.position.y + track.size.y - filled.size.y;
        } else {
            filled.size.x = track.size.x * amount;
        }
        control.theme_stylebox("grabber_area").draw(canvas, &filled, tint);

        let knob = if control.hovered() { control.theme_color("grabber_hover_color") } else { control.theme_color("grabber_color") };
        let knob_rect = self.grabber_rect(control);
        canvas.fill_rect(&knob_rect, knob.modulated(tint), knob_rect.size.x * 0.5);
    }
}

/// A single-line text field.
pub struct LineEdit {
    text: RefCell<String>,
    placeholder: RefCell<String>,
    editable: bool,
    caret: Cell<usize>,
    scroll: Cell<f32>,
}

impl LineEdit {
    /// An empty field, editable unless `read_only`.
    pub fn new(read_only: bool) -> LineEdit {
        LineEdit { text: RefCell::new(String::new()), placeholder: RefCell::new(String::new()), editable: !read_only, caret: Cell::new(0), scroll: Cell::new(0.0) }
    }

    /// Replaces the text, putting the caret at its end.
    pub fn set_text(&self, control: &Control, text: String) {
        self.caret.set(text.len());
        *self.text.borrow_mut() = text;
        self.update_scroll(control);
    }

    /// Sets the text shown while the field is empty.
    pub fn set_placeholder(&self, placeholder: &str) {
        *self.placeholder.borrow_mut() = placeholder.to_string();
    }

    /// Scrolls so that the caret is inside the content area.
    fn update_scroll(&self, control: &Control) {
        let font_size = control.theme_font_size("font_size");
        let content = control.theme_stylebox("normal").content_rect(&Rect2::new(Vec2::default(), control.size()));
        let text = self.text.borrow();
        let caret_x = control.measure_text(&text[..self.caret.get()], font_size).x;
        let mut scroll = self.scroll.get();
        if caret_x - scroll > content.size.x {
            scroll = caret_x - content.size.x;
        }
        if caret_x - scroll < 0.0 {
            scroll = caret_x;
        }
        let width = control.measure_text(&text, font_size).x;
        self.scroll.set(ui_math::clamp(scroll, 0.0, ui_math::max(width - content.size.x, 0.0)));
    }
}

impl Widget for LineEdit {
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>) {
        out.push("LineEdit");
        control_types(out);
    }

    fn get_minimum_size(&self, control: &Control) -> Vec2 {
        let font_size = control.theme_font_size("font_size");
        let line = Vec2::new(control.theme_constant("minimum_width"), control.font_metrics(font_size).line_height);
        line + control.theme_stylebox("normal").content_margins.size()
    }

    fn draw(&self, control: &Control, canvas: &mut dyn Canvas) {
        let tint = control.effective_modulate();
        let area = Rect2::new(Vec2::default(), control.size());
        let style = if !self.editable {
            "read_only"
        } else if control.has_focus() {
            "focus"
        } else {
            "normal"
        };
        control.theme_stylebox(style).draw(canvas, &area, tint);

        let frame = control.theme_stylebox("normal");
        let content = frame.content_rect(&area);
        let font_size = control.theme_font_size("font_size");
        let shown = self.text.borrow();
        let line_height = canvas.metrics(font_size).line_height;
        let baseline = content.position.y + (content.size.y - line_height) * 0.5;

        canvas.push_clip(&Rect2::new(canvas.offset() + content.position, content.size));

        let placeholder = self.placeholder.borrow();
        if shown.is_empty() && !placeholder.is_empty() {
            canvas.draw_text(Vec2::new(content.position.x, baseline), &placeholder, font_size, control.theme_color("placeholder_color").modulated(tint));
        }

        canvas.draw_text(
            Vec2::new(content.position.x - self.scroll.get(), baseline),
            &shown,
            font_size,
            control.theme_color("font_color").modulated(tint),
        );

        if control.has_focus() && self.editable {
            let caret_x = canvas.measure_text(&shown[..self.caret.get()], font_size).x;
            canvas.fill_rect(
                &Rect2::new(
                    Vec2::new(content.position.x + caret_x - self.scroll.get(), baseline),
                    Vec2::new(ui_math::max(control.theme_constant("caret_width"), 1.0), line_height),
                ),
                control.theme_color("caret_color").modulated(tint),
                0.0,
            );
        }

        canvas.pop_clip();
    }

    fn resized(&self, control: &Control) {
        self.update_scroll(control);
    }
}
