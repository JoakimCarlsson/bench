//! Themes: style boxes, colours, constants and font sizes grouped by type
//! name with type inheritance, looked up by string keys.
const std = @import("std");
const math = @import("ui_math.zig");
const ui_draw = @import("ui_draw.zig");

const Allocator = std.mem.Allocator;
const Vec2 = math.Vec2;
const Rect2 = math.Rect2;
const Color4 = math.Color4;
const Canvas = ui_draw.Canvas;
const Stroke = ui_draw.Stroke;

/// A string-keyed hash map whose keys the caller owns through `putOwned`.
pub fn StringMap(comptime T: type) type {
    return std.StringHashMapUnmanaged(T);
}

/// Sets `name` to `value` in `map`, copying the key when it is new.
pub fn putOwned(comptime T: type, map: *StringMap(T), gpa: Allocator, name: []const u8, value: T) Allocator.Error!void {
    if (map.getPtr(name)) |existing| {
        existing.* = value;
        return;
    }
    const key = try gpa.dupe(u8, name);
    errdefer gpa.free(key);
    try map.put(gpa, key, value);
}

/// Frees the keys of `map` and the map itself.
pub fn deinitOwned(comptime T: type, map: *StringMap(T), gpa: Allocator) void {
    var keys = map.keyIterator();
    while (keys.next()) |key| gpa.free(key.*);
    map.deinit(gpa);
}

/// Per-side distances.
pub const Margins = struct {
    left: f32 = 0.0,
    top: f32 = 0.0,
    right: f32 = 0.0,
    bottom: f32 = 0.0,

    /// The summed horizontal and vertical margins.
    pub fn size(self: Margins) Vec2 {
        return .{ .x = self.left + self.right, .y = self.top + self.bottom };
    }
};

/// How a style box draws.
pub const StyleBoxKind = enum(u8) { empty, flat };

/// A drawable background with content margins.
pub const StyleBox = struct {
    kind: StyleBoxKind = .empty,
    color: Color4 = .{},
    border_color: Color4 = .{},
    border_width: f32 = 0.0,
    corner_radius: f32 = 0.0,
    content_margins: Margins = .{},
    expand_margins: Margins = .{},

    /// `rect` shrunk by the content margins.
    pub fn contentRect(self: *const StyleBox, rect: Rect2) Rect2 {
        const inset: Vec2 = .{ .x = self.content_margins.left, .y = self.content_margins.top };
        return .{
            .position = rect.position.add(inset),
            .size = rect.size.sub(self.content_margins.size()).max(.{}),
        };
    }

    /// Draws the style into `rect` grown by the expand margins, tinted by `modulate`.
    pub fn draw(self: *const StyleBox, canvas: *Canvas, rect: Rect2, modulate: Color4) ui_draw.Error!void {
        if (self.kind == .empty) return;
        const target: Rect2 = .{
            .position = rect.position.sub(.{ .x = self.expand_margins.left, .y = self.expand_margins.top }),
            .size = rect.size.add(self.expand_margins.size()),
        };
        if (self.color.a > 0.0) try canvas.fillRect(target, self.color.modulated(modulate), self.corner_radius);
        if (self.border_width > 0.0 and self.border_color.a > 0.0) {
            try canvas.strokeRect(target, self.border_color.modulated(modulate), Stroke{ .width = self.border_width, .corner_radius = self.corner_radius });
        }
    }
};

/// A flat style box of `color` with rounded corners and content margins.
pub fn flatStyle(color: Color4, corner_radius: f32, content: Margins) StyleBox {
    return .{ .kind = .flat, .color = color, .corner_radius = corner_radius, .content_margins = content };
}

/// A style box that draws nothing, with content margins.
pub fn emptyStyle(content: Margins) StyleBox {
    return .{ .kind = .empty, .content_margins = content };
}

/// Style boxes, colours, constants and font sizes grouped by type name, with type inheritance.
pub const Theme = struct {
    gpa: Allocator,
    types: StringMap(TypeEntry) = .empty,

    /// Everything a theme holds for one type name.
    const TypeEntry = struct {
        base: []const u8 = &.{},
        styleboxes: StringMap(StyleBox) = .empty,
        colors: StringMap(Color4) = .empty,
        constants: StringMap(f32) = .empty,
        font_sizes: StringMap(f32) = .empty,
    };

    /// An empty heap-allocated theme.
    pub fn create(gpa: Allocator) Allocator.Error!*Theme {
        const self = try gpa.create(Theme);
        self.* = .{ .gpa = gpa };
        return self;
    }

    /// Frees the theme and every entry it holds.
    pub fn destroy(self: *Theme) void {
        const gpa = self.gpa;
        var entries = self.types.iterator();
        while (entries.next()) |item| {
            gpa.free(item.key_ptr.*);
            const found = item.value_ptr;
            gpa.free(found.base);
            deinitOwned(StyleBox, &found.styleboxes, gpa);
            deinitOwned(Color4, &found.colors, gpa);
            deinitOwned(f32, &found.constants, gpa);
            deinitOwned(f32, &found.font_sizes, gpa);
        }
        self.types.deinit(gpa);
        gpa.destroy(self);
    }

    /// The entry for `type_name`, created when absent.
    fn entry(self: *Theme, type_name: []const u8) Allocator.Error!*TypeEntry {
        if (self.types.getPtr(type_name)) |found| return found;
        const key = try self.gpa.dupe(u8, type_name);
        errdefer self.gpa.free(key);
        try self.types.put(self.gpa, key, .{});
        return self.types.getPtr(key).?;
    }

    /// Sets a style box for a type and name.
    pub fn setStylebox(self: *Theme, type_name: []const u8, name: []const u8, value: StyleBox) Allocator.Error!void {
        try putOwned(StyleBox, &(try self.entry(type_name)).styleboxes, self.gpa, name, value);
    }

    /// Sets a colour for a type and name.
    pub fn setColor(self: *Theme, type_name: []const u8, name: []const u8, value: Color4) Allocator.Error!void {
        try putOwned(Color4, &(try self.entry(type_name)).colors, self.gpa, name, value);
    }

    /// Sets a constant for a type and name.
    pub fn setConstant(self: *Theme, type_name: []const u8, name: []const u8, value: f32) Allocator.Error!void {
        try putOwned(f32, &(try self.entry(type_name)).constants, self.gpa, name, value);
    }

    /// Sets a font size for a type and name.
    pub fn setFontSize(self: *Theme, type_name: []const u8, name: []const u8, value: f32) Allocator.Error!void {
        try putOwned(f32, &(try self.entry(type_name)).font_sizes, self.gpa, name, value);
    }

    /// Sets the base type of `type_name` to `base`.
    pub fn setTypeBase(self: *Theme, type_name: []const u8, base: []const u8) Allocator.Error!void {
        const found = try self.entry(type_name);
        const owned = try self.gpa.dupe(u8, base);
        self.gpa.free(found.base);
        found.base = owned;
    }

    /// The item `name` of the kind `field` for `type_name`, or null.
    fn lookup(self: *const Theme, comptime T: type, comptime field: []const u8, type_name: []const u8, name: []const u8) ?*const T {
        const found = self.types.getPtr(type_name) orelse return null;
        return @field(found, field).getPtr(name);
    }

    /// The style box for `type_name` and `name`, or null.
    pub fn stylebox(self: *const Theme, type_name: []const u8, name: []const u8) ?*const StyleBox {
        return self.lookup(StyleBox, "styleboxes", type_name, name);
    }

    /// The colour for `type_name` and `name`, or null.
    pub fn color(self: *const Theme, type_name: []const u8, name: []const u8) ?*const Color4 {
        return self.lookup(Color4, "colors", type_name, name);
    }

    /// The constant for `type_name` and `name`, or null.
    pub fn constant(self: *const Theme, type_name: []const u8, name: []const u8) ?*const f32 {
        return self.lookup(f32, "constants", type_name, name);
    }

    /// The font size for `type_name` and `name`, or null.
    pub fn fontSize(self: *const Theme, type_name: []const u8, name: []const u8) ?*const f32 {
        return self.lookup(f32, "font_sizes", type_name, name);
    }

    /// The base type of `type_name`, or an empty slice when it has none.
    pub fn typeBase(self: *const Theme, type_name: []const u8) []const u8 {
        const found = self.types.getPtr(type_name) orelse return "";
        return found.base;
    }
};

const surface = Color4.init(0.12, 0.13, 0.15, 0.94);
const surface_raised = Color4.init(0.17, 0.19, 0.22, 1.0);
const surface_hover = Color4.init(0.22, 0.25, 0.29, 1.0);
const surface_pressed = Color4.init(0.10, 0.11, 0.13, 1.0);
const surface_sunken = Color4.init(0.07, 0.08, 0.09, 1.0);
const outline = Color4.init(0.30, 0.33, 0.38, 1.0);
const accent = Color4.init(0.29, 0.56, 0.89, 1.0);
const accent_dim = Color4.init(0.29, 0.56, 0.89, 0.35);
const text = Color4.init(0.88, 0.90, 0.93, 1.0);
const text_dim = Color4.init(0.58, 0.61, 0.66, 1.0);
const text_disabled = Color4.init(0.40, 0.42, 0.45, 1.0);
const transparent = Color4.init(0.0, 0.0, 0.0, 0.0);

const base_font_size: f32 = 16.0;
const base_radius: f32 = 4.0;

/// Margins with the same `amount` on every side.
fn uniform(amount: f32) Margins {
    return .{ .left = amount, .top = amount, .right = amount, .bottom = amount };
}

/// Margins with `horizontal` on the left and right and `vertical` on top and bottom.
fn padded(horizontal: f32, vertical: f32) Margins {
    return .{ .left = horizontal, .top = vertical, .right = horizontal, .bottom = vertical };
}

/// A text-field frame: sunken, outlined, with the given content margins.
fn editStyle(content: Margins) StyleBox {
    var style = flatStyle(surface_sunken, base_radius, content);
    style.border_width = 1.0;
    style.border_color = outline;
    return style;
}

/// A button frame of `color`, outlined, with the given content margins.
fn buttonStyle(color: Color4, content: Margins) StyleBox {
    var style = flatStyle(color, base_radius, content);
    style.border_width = 1.0;
    style.border_color = outline;
    return style;
}

/// The built-in dark theme, heap-allocated.
pub fn defaultTheme(gpa: Allocator) Allocator.Error!*Theme {
    const theme = try Theme.create(gpa);
    errdefer theme.destroy();

    try theme.setFontSize("Control", "font_size", base_font_size);
    try theme.setColor("Control", "font_color", text);

    try theme.setStylebox("Panel", "panel", flatStyle(surface, base_radius, uniform(8.0)));
    try theme.setStylebox("PanelContainer", "panel", flatStyle(surface, base_radius, uniform(8.0)));

    try theme.setColor("Label", "font_color", text);
    try theme.setColor("Label", "font_shadow_color", transparent);
    try theme.setFontSize("Label", "font_size", base_font_size);
    try theme.setConstant("Label", "line_spacing", 2.0);

    try theme.setConstant("BoxContainer", "separation", 6.0);
    try theme.setConstant("GridContainer", "h_separation", 6.0);
    try theme.setConstant("GridContainer", "v_separation", 6.0);
    try theme.setConstant("MarginContainer", "margin_left", 0.0);
    try theme.setConstant("MarginContainer", "margin_top", 0.0);
    try theme.setConstant("MarginContainer", "margin_right", 0.0);
    try theme.setConstant("MarginContainer", "margin_bottom", 0.0);

    const button_normal = buttonStyle(surface_raised, padded(12.0, 6.0));
    var button_hover = button_normal;
    button_hover.color = surface_hover;
    var button_pressed = button_normal;
    button_pressed.color = surface_pressed;
    var button_disabled = button_normal;
    button_disabled.color = surface_sunken;
    button_disabled.border_color = outline.withAlpha(0.4);
    var button_focus = emptyStyle(padded(12.0, 6.0));
    button_focus.kind = .flat;
    button_focus.color = transparent;
    button_focus.border_width = 1.0;
    button_focus.border_color = accent;
    button_focus.corner_radius = base_radius;

    try theme.setStylebox("Button", "normal", button_normal);
    try theme.setStylebox("Button", "hover", button_hover);
    try theme.setStylebox("Button", "pressed", button_pressed);
    try theme.setStylebox("Button", "disabled", button_disabled);
    try theme.setStylebox("Button", "focus", button_focus);
    try theme.setColor("Button", "font_color", text);
    try theme.setColor("Button", "font_hover_color", text);
    try theme.setColor("Button", "font_pressed_color", text);
    try theme.setColor("Button", "font_disabled_color", text_disabled);
    try theme.setFontSize("Button", "font_size", base_font_size);
    try theme.setConstant("Button", "h_separation", 6.0);

    try theme.setStylebox("CheckBox", "normal", emptyStyle(padded(4.0, 4.0)));
    try theme.setStylebox("CheckBox", "focus", button_focus);
    try theme.setColor("CheckBox", "font_color", text);
    try theme.setColor("CheckBox", "font_disabled_color", text_disabled);
    try theme.setColor("CheckBox", "box_color", surface_sunken);
    try theme.setColor("CheckBox", "box_border_color", outline);
    try theme.setColor("CheckBox", "check_color", accent);
    try theme.setFontSize("CheckBox", "font_size", base_font_size);
    try theme.setConstant("CheckBox", "h_separation", 8.0);
    try theme.setConstant("CheckBox", "box_size", 16.0);

    try theme.setStylebox("Slider", "slider", flatStyle(surface_sunken, 3.0, .{}));
    try theme.setStylebox("Slider", "grabber_area", flatStyle(accent, 3.0, .{}));
    try theme.setColor("Slider", "grabber_color", text);
    try theme.setColor("Slider", "grabber_hover_color", accent);
    try theme.setConstant("Slider", "grabber_size", 14.0);
    try theme.setConstant("Slider", "thickness", 6.0);

    const edit_normal = editStyle(padded(8.0, 5.0));
    var edit_focus = edit_normal;
    edit_focus.border_color = accent;
    var edit_read_only = edit_normal;
    edit_read_only.color = surface;
    try theme.setStylebox("LineEdit", "normal", edit_normal);
    try theme.setStylebox("LineEdit", "focus", edit_focus);
    try theme.setStylebox("LineEdit", "read_only", edit_read_only);
    try theme.setColor("LineEdit", "font_color", text);
    try theme.setColor("LineEdit", "placeholder_color", text_disabled);
    try theme.setColor("LineEdit", "selection_color", accent_dim);
    try theme.setColor("LineEdit", "caret_color", text);
    try theme.setFontSize("LineEdit", "font_size", base_font_size);
    try theme.setConstant("LineEdit", "minimum_width", 80.0);
    try theme.setConstant("LineEdit", "caret_width", 1.0);

    try theme.setStylebox("TabContainer", "panel", flatStyle(surface, base_radius, uniform(8.0)));
    try theme.setStylebox("TabContainer", "tab_selected", flatStyle(surface, base_radius, padded(12.0, 6.0)));
    try theme.setStylebox("TabContainer", "tab_unselected", flatStyle(surface_sunken, base_radius, padded(12.0, 6.0)));
    try theme.setColor("TabContainer", "font_selected_color", text);
    try theme.setColor("TabContainer", "font_unselected_color", text_dim);
    try theme.setFontSize("TabContainer", "font_size", base_font_size);
    try theme.setConstant("TabContainer", "h_separation", 2.0);

    return theme;
}

/// A theme for an inspector panel, heap-allocated: tighter spacing, dimmer
/// property labels and two button variations that inherit from `Button`
/// through type bases.
pub fn inspectorTheme(gpa: Allocator) Allocator.Error!*Theme {
    const theme = try Theme.create(gpa);
    errdefer theme.destroy();

    try theme.setConstant("BoxContainer", "separation", 4.0);
    try theme.setConstant("GridContainer", "h_separation", 8.0);
    try theme.setConstant("GridContainer", "v_separation", 3.0);
    try theme.setFontSize("Label", "font_size", 14.0);

    try theme.setTypeBase("PropertyLabel", "Label");
    try theme.setColor("PropertyLabel", "font_color", text_dim);

    try theme.setTypeBase("SectionHeader", "Button");
    try theme.setStylebox("SectionHeader", "normal", buttonStyle(surface_raised, padded(8.0, 4.0)));
    try theme.setStylebox("SectionHeader", "hover", buttonStyle(surface_hover, padded(8.0, 4.0)));
    try theme.setColor("SectionHeader", "font_color", accent);

    try theme.setStylebox("LineEdit", "normal", editStyle(padded(6.0, 3.0)));
    try theme.setFontSize("LineEdit", "font_size", 14.0);
    try theme.setConstant("LineEdit", "minimum_width", 48.0);

    try theme.setConstant("Slider", "grabber_size", 12.0);
    return theme;
}
