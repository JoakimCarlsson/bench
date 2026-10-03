//! The leaf controls: panel, label, buttons, slider and line edit.
const std = @import("std");
const vm = @import("vecmath.zig");
const math = @import("ui_math.zig");
const ui_control = @import("ui_control.zig");
const ui_draw = @import("ui_draw.zig");

const Allocator = std.mem.Allocator;
const Vec2 = math.Vec2;
const Rect2 = math.Rect2;
const Canvas = ui_draw.Canvas;
const Stroke = ui_draw.Stroke;
const TextureId = ui_draw.TextureId;
const Control = ui_control.Control;
const ThemeTypes = ui_control.ThemeTypes;
const Error = ui_control.Error;

/// Where content sits along one axis of its area.
pub const Alignment = enum(u8) {
    begin,
    center,
    end,

    /// The offset that places `used` within `available`.
    pub fn offset(self: Alignment, available: f32, used: f32) f32 {
        return switch (self) {
            .begin => 0.0,
            .center => (available - used) * 0.5,
            .end => available - used,
        };
    }
};

/// Where text sits along a row.
pub const HorizontalAlignment = Alignment;

/// Where text sits along a column.
pub const VerticalAlignment = Alignment;

/// Allocates a `T` with a copy of `text` in its `text` field and appends it to `parent`.
fn spawnWithText(parent: *Control, comptime T: type, value: T, text: []const u8) Error!*T {
    const gpa = parent.allocator();
    const owned = try gpa.dupe(u8, text);
    errdefer gpa.free(owned);
    var made = value;
    made.text = owned;
    return parent.spawn(T, made);
}

/// Replaces the text in `slot` and invalidates the minimum size when it differs.
fn setMeasuredText(control: *Control, slot: *[]const u8, text: []const u8) Error!void {
    if (std.mem.eql(u8, slot.*, text)) return;
    try ui_control.replaceString(control.allocator(), slot, text);
    control.updateMinimumSize();
}

/// Releases the subtree and the text of a `T` that embeds its control as `control`, and frees it.
fn destroyWithText(comptime T: type) fn (*Control, Allocator) void {
    return struct {
        /// Releases the subtree, frees the text and then the `T`.
        fn destroy(control: *Control, gpa: Allocator) void {
            const self: *T = @fieldParentPtr("control", control);
            gpa.free(self.text);
            control.deinit(gpa);
            gpa.destroy(self);
        }
    }.destroy;
}

/// A control that draws only its themed panel style box.
pub const Panel = struct {
    control: Control,

    const vtable: Control.VTable = .{
        .destroy = ui_control.destroyFor(Panel),
        .collectThemeTypes = collectThemeTypes,
        .draw = draw,
    };

    /// A panel added to `parent`.
    pub fn create(parent: *Control) Error!*Panel {
        return parent.spawn(Panel, .{ .control = Control.init(parent.context, &vtable) });
    }

    /// Appends the type names of a panel.
    fn collectThemeTypes(_: *const Control, types: *ThemeTypes) void {
        types.appendAssumeCapacity("Panel");
        Control.collectBaseThemeTypes(types);
    }

    /// Draws the panel style box.
    fn draw(control: *const Control, canvas: *Canvas) Error!void {
        try control.themeStylebox("panel").draw(canvas, .{ .size = control.size }, control.effectiveModulate());
    }
};

/// Static text of one or more lines.
pub const Label = struct {
    control: Control,
    text: []const u8 = "",
    horizontal: HorizontalAlignment = .begin,
    vertical: VerticalAlignment = .begin,

    const vtable: Control.VTable = .{
        .destroy = destroyWithText(Label),
        .collectThemeTypes = collectThemeTypes,
        .getMinimumSize = getMinimumSize,
        .draw = draw,
    };

    /// A label showing `text`, added to `parent`.
    pub fn create(parent: *Control, text: []const u8) Error!*Label {
        return spawnWithText(parent, Label, .{ .control = Control.init(parent.context, &vtable) }, text);
    }

    /// Appends the type names of a label.
    fn collectThemeTypes(_: *const Control, types: *ThemeTypes) void {
        types.appendAssumeCapacity("Label");
        Control.collectBaseThemeTypes(types);
    }

    /// The lines of the text, split at newlines; the caller frees the list.
    fn lines(self: *const Label) Error!std.ArrayList([]const u8) {
        const gpa = self.control.allocator();
        var result: std.ArrayList([]const u8) = .empty;
        errdefer result.deinit(gpa);
        var remaining = self.text;
        while (std.mem.indexOfScalar(u8, remaining, '\n')) |position| {
            try result.append(gpa, remaining[0..position]);
            remaining = remaining[position + 1 ..];
        }
        try result.append(gpa, remaining);
        return result;
    }

    /// The widest line, and the line heights plus the spacings between them.
    fn getMinimumSize(control: *const Control) Error!Vec2 {
        const self: *const Label = @fieldParentPtr("control", control);
        const font_size = control.themeFontSize("font_size");
        const spacing = control.themeConstant("line_spacing");
        const target = control.canvas();
        var result: Vec2 = .{};
        var entries = try self.lines();
        defer entries.deinit(control.allocator());
        for (entries.items, 0..) |line, index| {
            const measured = target.measureText(line, font_size);
            result.x = vm.maxf(result.x, measured.x);
            result.y += measured.y;
            if (index + 1 < entries.items.len) result.y += spacing;
        }
        return result;
    }

    /// Draws every line, aligned inside the control.
    fn draw(control: *const Control, canvas: *Canvas) Error!void {
        const self: *const Label = @fieldParentPtr("control", control);
        if (self.text.len == 0) return;
        const font_size = control.themeFontSize("font_size");
        const spacing = control.themeConstant("line_spacing");
        const color = control.themeColor("font_color").modulated(control.effectiveModulate());
        var entries = try self.lines();
        defer entries.deinit(control.allocator());
        const count: f32 = @floatFromInt(entries.items.len);
        const line_height = canvas.metrics(font_size).line_height;
        const total = line_height * count + spacing * @as(f32, @floatFromInt(entries.items.len - 1));
        var y = self.vertical.offset(control.size.y, total);
        for (entries.items) |line| {
            const measured = canvas.measureText(line, font_size);
            const x = self.horizontal.offset(control.size.x, measured.x);
            try canvas.drawText(.{ .x = x, .y = y }, line, font_size, color);
            y += line_height + spacing;
        }
    }

    /// Replaces the text, invalidating the minimum size when it differs.
    pub fn setText(self: *Label, text: []const u8) Error!void {
        try setMeasuredText(&self.control, &self.text, text);
    }
};

/// How a button looks.
pub const ButtonState = enum(u8) { normal, hover, pressed, disabled };

/// The state shared by pressable controls.
pub const BaseButton = struct {
    disabled: bool = false,
    toggle_mode: bool = false,
    pressed: bool = false,
    held: bool = false,

    /// Appends the type name of a pressable control and then the base control type.
    fn collectThemeTypes(types: *ThemeTypes) void {
        types.appendAssumeCapacity("BaseButton");
        Control.collectBaseThemeTypes(types);
    }

    /// Sets whether the button ignores input.
    pub fn setDisabled(self: *BaseButton, disabled: bool) void {
        self.disabled = disabled;
        if (self.disabled) self.held = false;
    }

    /// The look the button currently has.
    pub fn state(self: *const BaseButton, hovered: bool) ButtonState {
        if (self.disabled) return .disabled;
        if (self.held or (self.toggle_mode and self.pressed)) return .pressed;
        if (hovered) return .hover;
        return .normal;
    }
};

/// A push button with a text and an optional icon.
pub const Button = struct {
    control: Control,
    base: BaseButton = .{},
    text: []const u8 = "",
    icon: TextureId = ui_draw.no_texture,
    icon_size: Vec2 = .{},
    horizontal: HorizontalAlignment = .center,

    const vtable: Control.VTable = .{
        .destroy = destroyWithText(Button),
        .collectThemeTypes = collectThemeTypes,
        .getMinimumSize = getMinimumSize,
        .draw = draw,
    };

    /// A button labelled `text`, added to `parent`.
    pub fn create(parent: *Control, text: []const u8) Error!*Button {
        return spawnWithText(parent, Button, .{ .control = Control.init(parent.context, &vtable) }, text);
    }

    /// Appends the type names of a button.
    fn collectThemeTypes(_: *const Control, types: *ThemeTypes) void {
        types.appendAssumeCapacity("Button");
        BaseButton.collectThemeTypes(types);
    }

    /// Replaces the text, invalidating the minimum size when it differs.
    pub fn setText(self: *Button, text: []const u8) Error!void {
        try setMeasuredText(&self.control, &self.text, text);
    }

    /// Sets the icon texture and its size.
    pub fn setIcon(self: *Button, icon: TextureId, size: Vec2) void {
        self.icon = icon;
        self.icon_size = size;
        self.control.updateMinimumSize();
    }

    /// The style box name for the current state.
    fn styleName(self: *const Button) []const u8 {
        return switch (self.base.state(self.control.hovered)) {
            .disabled => "disabled",
            .pressed => "pressed",
            .hover => "hover",
            .normal => "normal",
        };
    }

    /// The font colour name for the current state.
    fn colorName(self: *const Button) []const u8 {
        return switch (self.base.state(self.control.hovered)) {
            .disabled => "font_disabled_color",
            .pressed => "font_pressed_color",
            .hover => "font_hover_color",
            .normal => "font_color",
        };
    }

    /// The text and icon sizes plus the normal style's content margins.
    fn getMinimumSize(control: *const Control) Error!Vec2 {
        const self: *const Button = @fieldParentPtr("control", control);
        const font_size = control.themeFontSize("font_size");
        const measured = control.canvas().measureText(self.text, font_size);
        var content = measured;
        if (self.icon != ui_draw.no_texture) {
            content.x += self.icon_size.x;
            content.y = vm.maxf(content.y, self.icon_size.y);
            if (self.text.len > 0) content.x += control.themeConstant("h_separation");
        }
        return content.add(control.themeStylebox("normal").content_margins.size());
    }

    /// Draws the frame, the icon and the text.
    fn draw(control: *const Control, canvas: *Canvas) Error!void {
        const self: *const Button = @fieldParentPtr("control", control);
        const tint = control.effectiveModulate();
        const area: Rect2 = .{ .size = control.size };
        try control.themeStylebox(self.styleName()).draw(canvas, area, tint);
        if (control.hasFocus()) try control.themeStylebox("focus").draw(canvas, area, tint);

        const content = control.themeStylebox(self.styleName()).contentRect(area);
        const font_size = control.themeFontSize("font_size");
        const measured = canvas.measureText(self.text, font_size);
        const has_icon = self.icon != ui_draw.no_texture;
        const separation: f32 = if (has_icon and self.text.len > 0) control.themeConstant("h_separation") else 0.0;
        const used = measured.x + separation + (if (has_icon) self.icon_size.x else 0.0);

        var x = content.position.x;
        switch (self.horizontal) {
            .begin => {},
            .center => x += (content.size.x - used) * 0.5,
            .end => x += content.size.x - used,
        }

        if (has_icon) {
            const icon_rect: Rect2 = .{
                .position = .{ .x = x, .y = content.position.y + (content.size.y - self.icon_size.y) * 0.5 },
                .size = self.icon_size,
            };
            try canvas.fillTextureRect(icon_rect, self.icon, .{ .size = .{ .x = 1.0, .y = 1.0 } }, tint);
            x += self.icon_size.x + separation;
        }

        if (self.text.len == 0) return;
        const y = content.position.y + (content.size.y - measured.y) * 0.5;
        try canvas.drawText(.{ .x = x, .y = y }, self.text, font_size, control.themeColor(self.colorName()).modulated(tint));
    }
};

/// A toggle button drawn as a box with a label.
pub const CheckBox = struct {
    control: Control,
    base: BaseButton = .{ .toggle_mode = true },
    text: []const u8 = "",

    const vtable: Control.VTable = .{
        .destroy = destroyWithText(CheckBox),
        .collectThemeTypes = collectThemeTypes,
        .getMinimumSize = getMinimumSize,
        .draw = draw,
    };

    /// A check box labelled `text`, added to `parent`.
    pub fn create(parent: *Control, text: []const u8) Error!*CheckBox {
        return spawnWithText(parent, CheckBox, .{ .control = Control.init(parent.context, &vtable) }, text);
    }

    /// Appends the type names of a check box.
    fn collectThemeTypes(_: *const Control, types: *ThemeTypes) void {
        types.appendAssumeCapacity("CheckBox");
        BaseButton.collectThemeTypes(types);
    }

    /// The box and the text plus the normal style's content margins.
    fn getMinimumSize(control: *const Control) Error!Vec2 {
        const self: *const CheckBox = @fieldParentPtr("control", control);
        const font_size = control.themeFontSize("font_size");
        const box = control.themeConstant("box_size");
        const measured = control.canvas().measureText(self.text, font_size);
        var content: Vec2 = .{ .x = box, .y = vm.maxf(box, measured.y) };
        if (self.text.len > 0) content.x += control.themeConstant("h_separation") + measured.x;
        return content.add(control.themeStylebox("normal").content_margins.size());
    }

    /// Draws the frame, the box, the check mark and the text.
    fn draw(control: *const Control, canvas: *Canvas) Error!void {
        const self: *const CheckBox = @fieldParentPtr("control", control);
        const tint = control.effectiveModulate();
        const area: Rect2 = .{ .size = control.size };
        const normal = control.themeStylebox("normal");
        try normal.draw(canvas, area, tint);
        if (control.hasFocus()) try control.themeStylebox("focus").draw(canvas, area, tint);

        const content = normal.contentRect(area);
        const box = control.themeConstant("box_size");
        const box_rect: Rect2 = .{
            .position = .{ .x = content.position.x, .y = content.position.y + (content.size.y - box) * 0.5 },
            .size = .{ .x = box, .y = box },
        };
        try canvas.fillRect(box_rect, control.themeColor("box_color").modulated(tint), 3.0);
        try canvas.strokeRect(box_rect, control.themeColor("box_border_color").modulated(tint), Stroke{ .width = 1.0, .corner_radius = 3.0 });
        if (self.base.pressed) {
            const mark = math.grow(box_rect, -box * 0.25);
            try canvas.fillRect(mark, control.themeColor("check_color").modulated(tint), 2.0);
        }

        if (self.text.len == 0) return;
        const font_size = control.themeFontSize("font_size");
        const measured = canvas.measureText(self.text, font_size);
        const color_key: []const u8 = if (self.base.disabled) "font_disabled_color" else "font_color";
        try canvas.drawText(
            .{ .x = box_rect.right() + control.themeConstant("h_separation"), .y = content.position.y + (content.size.y - measured.y) * 0.5 },
            self.text,
            font_size,
            control.themeColor(color_key).modulated(tint),
        );
    }

    /// Replaces the text, invalidating the minimum size when it differs.
    pub fn setText(self: *CheckBox, text: []const u8) Error!void {
        try setMeasuredText(&self.control, &self.text, text);
    }
};

/// `std::max` on doubles.
fn maxd(a: f64, b: f64) f64 {
    return if (a < b) b else a;
}

/// `std::clamp` on doubles.
fn clampd(value: f64, low: f64, high: f64) f64 {
    if (value < low) return low;
    if (high < value) return high;
    return value;
}

/// A value between a minimum and a maximum.
pub const Range = struct {
    minimum: f64 = 0.0,
    maximum: f64 = 100.0,
    page: f64 = 0.0,
    value: f64 = 0.0,

    /// Appends the type name of a range and then the base control type.
    fn collectThemeTypes(types: *ThemeTypes) void {
        types.appendAssumeCapacity("Range");
        Control.collectBaseThemeTypes(types);
    }

    /// Sets the bounds and the value directly.
    pub fn setRange(self: *Range, minimum: f64, maximum: f64, value: f64) void {
        self.minimum = minimum;
        self.maximum = maxd(maximum, self.minimum);
        self.setValue(value);
    }

    /// Sets the value, clamped to the bounds.
    pub fn setValue(self: *Range, value: f64) void {
        const upper = maxd(self.minimum, self.maximum - self.page);
        self.value = clampd(value, self.minimum, upper);
    }

    /// The value as a fraction of the range, in [0, 1].
    pub fn ratio(self: *const Range) f64 {
        const span = self.maximum - self.page - self.minimum;
        if (span <= 0.0) return 0.0;
        return clampd((self.value - self.minimum) / span, 0.0, 1.0);
    }
};

/// A draggable grabber along a track.
pub const Slider = struct {
    control: Control,
    range: Range = .{},
    vertical: bool = false,

    const vtable: Control.VTable = .{
        .destroy = ui_control.destroyFor(Slider),
        .collectThemeTypes = collectThemeTypes,
        .getMinimumSize = getMinimumSize,
        .draw = draw,
    };

    /// A vertical slider when `vertical`, else a horizontal one, added to `parent`.
    pub fn create(parent: *Control, vertical: bool) Error!*Slider {
        return parent.spawn(Slider, .{ .control = Control.init(parent.context, &vtable), .vertical = vertical });
    }

    /// Appends the type names of a slider.
    fn collectThemeTypes(_: *const Control, types: *ThemeTypes) void {
        types.appendAssumeCapacity("Slider");
        Range.collectThemeTypes(types);
    }

    /// The themed grabber size.
    fn grabberSize(control: *const Control) f32 {
        return control.themeConstant("grabber_size");
    }

    /// Twice the grabber along the track and one grabber across it.
    fn getMinimumSize(control: *const Control) Error!Vec2 {
        const self: *const Slider = @fieldParentPtr("control", control);
        const grabber = grabberSize(control);
        return if (self.vertical)
            .{ .x = grabber, .y = grabber * 2.0 }
        else
            .{ .x = grabber * 2.0, .y = grabber };
    }

    /// The rectangle of the grabber.
    fn grabberRect(self: *const Slider) Rect2 {
        const control = &self.control;
        const grabber = grabberSize(control);
        const amount: f32 = @floatCast(self.range.ratio());
        if (self.vertical) {
            const travel = vm.maxf(control.size.y - grabber, 0.0);
            const y = travel * (1.0 - amount);
            return .{ .position = .{ .x = (control.size.x - grabber) * 0.5, .y = y }, .size = .{ .x = grabber, .y = grabber } };
        }
        const travel = vm.maxf(control.size.x - grabber, 0.0);
        return .{ .position = .{ .x = travel * amount, .y = (control.size.y - grabber) * 0.5 }, .size = .{ .x = grabber, .y = grabber } };
    }

    /// Draws the track, the filled part and the grabber.
    fn draw(control: *const Control, canvas: *Canvas) Error!void {
        const self: *const Slider = @fieldParentPtr("control", control);
        const tint = control.effectiveModulate();
        const thickness = control.themeConstant("thickness");
        const grabber = grabberSize(control);
        const track: Rect2 = if (self.vertical) .{
            .position = .{ .x = (control.size.x - thickness) * 0.5, .y = grabber * 0.5 },
            .size = .{ .x = thickness, .y = vm.maxf(control.size.y - grabber, 0.0) },
        } else .{
            .position = .{ .x = grabber * 0.5, .y = (control.size.y - thickness) * 0.5 },
            .size = .{ .x = vm.maxf(control.size.x - grabber, 0.0), .y = thickness },
        };
        try control.themeStylebox("slider").draw(canvas, track, tint);

        var filled = track;
        const amount: f32 = @floatCast(self.range.ratio());
        if (self.vertical) {
            filled.size.y = track.size.y * amount;
            filled.position.y = track.position.y + track.size.y - filled.size.y;
        } else {
            filled.size.x = track.size.x * amount;
        }
        try control.themeStylebox("grabber_area").draw(canvas, filled, tint);

        const knob = if (control.hovered) control.themeColor("grabber_hover_color") else control.themeColor("grabber_color");
        const knob_rect = self.grabberRect();
        try canvas.fillRect(knob_rect, knob.modulated(tint), knob_rect.size.x * 0.5);
    }
};

/// A single-line text field.
pub const LineEdit = struct {
    control: Control,
    text: []const u8 = "",
    placeholder: []const u8 = "",
    editable: bool = true,
    caret: usize = 0,
    scroll: f32 = 0.0,

    const vtable: Control.VTable = .{
        .destroy = destroy,
        .collectThemeTypes = collectThemeTypes,
        .getMinimumSize = getMinimumSize,
        .draw = draw,
        .resized = resized,
    };

    /// An empty text field added to `parent`.
    pub fn create(parent: *Control) Error!*LineEdit {
        return parent.spawn(LineEdit, .{ .control = Control.init(parent.context, &vtable) });
    }

    /// Releases the subtree and both texts, and frees the field.
    fn destroy(control: *Control, gpa: Allocator) void {
        const self: *LineEdit = @fieldParentPtr("control", control);
        gpa.free(self.text);
        gpa.free(self.placeholder);
        control.deinit(gpa);
        gpa.destroy(self);
    }

    /// Appends the type names of a line edit.
    fn collectThemeTypes(_: *const Control, types: *ThemeTypes) void {
        types.appendAssumeCapacity("LineEdit");
        Control.collectBaseThemeTypes(types);
    }

    /// Scrolls so that the caret is inside the content area.
    fn updateScroll(self: *LineEdit) void {
        const control = &self.control;
        const font_size = control.themeFontSize("font_size");
        const target = control.canvas();
        const content = control.themeStylebox("normal").contentRect(.{ .size = control.size });
        const prefix = self.text[0..self.caret];
        const caret_x = target.measureText(prefix, font_size).x;
        if (caret_x - self.scroll > content.size.x) self.scroll = caret_x - content.size.x;
        if (caret_x - self.scroll < 0.0) self.scroll = caret_x;
        const width = target.measureText(self.text, font_size).x;
        self.scroll = vm.clampf(self.scroll, 0.0, vm.maxf(width - content.size.x, 0.0));
    }

    /// Replaces the text, putting the caret at its end.
    pub fn setText(self: *LineEdit, text: []const u8) Error!void {
        try ui_control.replaceString(self.control.allocator(), &self.text, text);
        self.caret = self.text.len;
        self.updateScroll();
    }

    /// Sets the text shown while the field is empty.
    pub fn setPlaceholder(self: *LineEdit, placeholder: []const u8) Error!void {
        try ui_control.replaceString(self.control.allocator(), &self.placeholder, placeholder);
    }

    /// The minimum width and one line plus the normal style's content margins.
    fn getMinimumSize(control: *const Control) Error!Vec2 {
        const font_size = control.themeFontSize("font_size");
        const line: Vec2 = .{ .x = control.themeConstant("minimum_width"), .y = control.canvas().metrics(font_size).line_height };
        return line.add(control.themeStylebox("normal").content_margins.size());
    }

    /// Draws the frame, the text or placeholder, and the caret, clipped to the content area.
    fn draw(control: *const Control, canvas: *Canvas) Error!void {
        const self: *const LineEdit = @fieldParentPtr("control", control);
        const tint = control.effectiveModulate();
        const area: Rect2 = .{ .size = control.size };
        const style: []const u8 = if (!self.editable) "read_only" else if (control.hasFocus()) "focus" else "normal";
        try control.themeStylebox(style).draw(canvas, area, tint);

        const frame = control.themeStylebox("normal");
        const content = frame.contentRect(area);
        const font_size = control.themeFontSize("font_size");
        const shown = self.text;
        const line_height = canvas.metrics(font_size).line_height;
        const baseline = content.position.y + (content.size.y - line_height) * 0.5;

        try canvas.pushClip(.{ .position = canvas.offset().add(content.position), .size = content.size });

        if (shown.len == 0 and self.placeholder.len > 0) {
            try canvas.drawText(.{ .x = content.position.x, .y = baseline }, self.placeholder, font_size, control.themeColor("placeholder_color").modulated(tint));
        }

        try canvas.drawText(.{ .x = content.position.x - self.scroll, .y = baseline }, shown, font_size, control.themeColor("font_color").modulated(tint));

        if (control.hasFocus() and self.editable) {
            const prefix = shown[0..self.caret];
            const caret_x = canvas.measureText(prefix, font_size).x;
            const caret: Rect2 = .{
                .position = .{ .x = content.position.x + caret_x - self.scroll, .y = baseline },
                .size = .{ .x = vm.maxf(control.themeConstant("caret_width"), 1.0), .y = line_height },
            };
            try canvas.fillRect(caret, control.themeColor("caret_color").modulated(tint), 0.0);
        }

        canvas.popClip();
    }

    /// Keeps the caret in view after the size changes.
    fn resized(control: *Control) void {
        const self: *LineEdit = @fieldParentPtr("control", control);
        self.updateScroll();
    }
};
