//! The base of the UI tree: `Control`, a rectangle with anchors, a cached
//! minimum size, string-keyed theme lookup through the control and theme
//! chains, and virtual layout and drawing through a vtable; and `Context`,
//! the owner of a tree.
const std = @import("std");
const vm = @import("vecmath.zig");
const math = @import("ui_math.zig");
const ui_draw = @import("ui_draw.zig");
const ui_theme = @import("ui_theme.zig");

const Allocator = std.mem.Allocator;
const Vec2 = math.Vec2;
const Rect2 = math.Rect2;
const Color4 = math.Color4;
const Canvas = ui_draw.Canvas;
const Theme = ui_theme.Theme;
const StyleBox = ui_theme.StyleBox;
const StringMap = ui_theme.StringMap;

pub const Error = ui_draw.Error;

/// The most theme type names a control reports, its type variation included.
pub const max_theme_types = 8;

/// The depth of the base-type chain followed from each theme type name.
const max_type_depth = 8;

/// The theme type names of a control, most specific first.
pub const ThemeTypes = std.ArrayList([]const u8);

/// A side of a rectangle.
pub const Side = enum(u8) { left, top, right, bottom };

/// Named anchor layouts such as corners, edges and full rect.
pub const LayoutPreset = enum(u8) {
    top_left,
    top_right,
    bottom_left,
    bottom_right,
    center_left,
    center_top,
    center_right,
    center_bottom,
    center,
    left_wide,
    top_wide,
    right_wide,
    bottom_wide,
    v_center_wide,
    h_center_wide,
    full_rect,
};

/// The four anchor values of a layout preset.
pub const PresetAnchors = struct {
    left: f32 = 0.0,
    top: f32 = 0.0,
    right: f32 = 0.0,
    bottom: f32 = 0.0,
};

/// A set of anchors from its four values.
fn anchors(left: f32, top: f32, right: f32, bottom: f32) PresetAnchors {
    return .{ .left = left, .top = top, .right = right, .bottom = bottom };
}

/// The anchors that `preset` places on each side.
pub fn presetAnchors(preset: LayoutPreset) PresetAnchors {
    return switch (preset) {
        .top_left => anchors(0.0, 0.0, 0.0, 0.0),
        .top_right => anchors(1.0, 0.0, 1.0, 0.0),
        .bottom_left => anchors(0.0, 1.0, 0.0, 1.0),
        .bottom_right => anchors(1.0, 1.0, 1.0, 1.0),
        .center_left => anchors(0.0, 0.5, 0.0, 0.5),
        .center_top => anchors(0.5, 0.0, 0.5, 0.0),
        .center_right => anchors(1.0, 0.5, 1.0, 0.5),
        .center_bottom => anchors(0.5, 1.0, 0.5, 1.0),
        .center => anchors(0.5, 0.5, 0.5, 0.5),
        .left_wide => anchors(0.0, 0.0, 0.0, 1.0),
        .top_wide => anchors(0.0, 0.0, 1.0, 0.0),
        .right_wide => anchors(1.0, 0.0, 1.0, 1.0),
        .bottom_wide => anchors(0.0, 1.0, 1.0, 1.0),
        .v_center_wide => anchors(0.0, 0.5, 1.0, 0.5),
        .h_center_wide => anchors(0.5, 0.0, 0.5, 1.0),
        .full_rect => anchors(0.0, 0.0, 1.0, 1.0),
    };
}

/// Which way a control grows when its minimum size exceeds its rect.
pub const GrowDirection = enum(u8) { begin, end, both };

/// How a container sizes and places a child on one axis.
pub const SizeFlags = enum(u8) {
    none = 0,
    fill = 1,
    expand = 2,
    shrink_center = 4,
    shrink_end = 8,
    expand_fill = 3,
};

/// Whether any bit of `flag` is set in `value`.
pub fn hasFlag(value: SizeFlags, flag: SizeFlags) bool {
    return (@intFromEnum(value) & @intFromEnum(flag)) != 0;
}

/// Replaces the owned string in `slot` with a copy of `text`.
pub fn replaceString(gpa: Allocator, slot: *[]const u8, text: []const u8) Allocator.Error!void {
    const owned = try gpa.dupe(u8, text);
    gpa.free(slot.*);
    slot.* = owned;
}

/// A `destroy` for a `T` that embeds its control as `control` and owns nothing else.
pub fn destroyFor(comptime T: type) fn (*Control, Allocator) void {
    return struct {
        /// Releases the control's subtree and frees the `T`.
        fn destroy(control: *Control, gpa: Allocator) void {
            const self: *T = @fieldParentPtr("control", control);
            control.deinit(gpa);
            gpa.destroy(self);
        }
    }.destroy;
}

const fallback_stylebox: StyleBox = .{};

/// Walks `type_name` and its base types in `theme`, returning the first item `lookup` finds, or null.
fn findInTypeChain(
    comptime Value: type,
    comptime lookup: fn (*const Theme, []const u8, []const u8) ?*const Value,
    theme: *const Theme,
    type_name: []const u8,
    name: []const u8,
) ?*const Value {
    var current = type_name;
    for (0..max_type_depth) |_| {
        if (lookup(theme, current, name)) |found| return found;
        const base = theme.typeBase(current);
        if (base.len == 0) break;
        current = base;
    }
    return null;
}

/// Looks up a theme item for a control, trying its type variation and theme
/// types up each base chain. Searches the control's own theme and its
/// ancestors' themes before the context's fallback theme.
fn findInThemes(
    comptime Value: type,
    comptime lookup: fn (*const Theme, []const u8, []const u8) ?*const Value,
    control: *const Control,
    name: []const u8,
) ?*const Value {
    var buffer: [max_theme_types][]const u8 = undefined;
    var types = ThemeTypes.initBuffer(&buffer);
    if (control.theme_type_variation.len > 0) types.appendAssumeCapacity(control.theme_type_variation);
    control.vtable.collectThemeTypes(control, &types);

    var owner: ?*const Control = control;
    while (owner) |current| : (owner = current.parent) {
        const theme = current.theme orelse continue;
        for (types.items) |type_name| {
            if (findInTypeChain(Value, lookup, theme, type_name, name)) |found| return found;
        }
    }

    for (types.items) |type_name| {
        if (findInTypeChain(Value, lookup, control.context.theme, type_name, name)) |found| return found;
    }
    return null;
}

/// Base of the UI tree. Concrete controls embed it as their `control` field
/// and recover themselves with `@fieldParentPtr`. Children are owned.
pub const Control = struct {
    vtable: *const VTable,
    context: *Context,
    parent: ?*Control = null,
    children: std.ArrayList(*Control) = .empty,

    anchor: [4]f32 = @splat(0.0),
    offset: [4]f32 = @splat(0.0),
    h_grow: GrowDirection = .end,
    v_grow: GrowDirection = .end,

    position: Vec2 = .{},
    size: Vec2 = .{},
    custom_minimum_size: Vec2 = .{},
    minimum_size_cache: Vec2 = .{},
    minimum_size_valid: bool = false,

    h_size_flags: SizeFlags = .fill,
    v_size_flags: SizeFlags = .fill,
    stretch_ratio: f32 = 1.0,

    visible: bool = true,
    clip_contents: bool = false,
    modulate: Color4 = .{},
    hovered: bool = false,

    theme: ?*const Theme = null,
    theme_type_variation: []const u8 = "",
    color_overrides: StringMap(Color4) = .empty,
    constant_overrides: StringMap(f32) = .empty,

    /// The virtual functions of a control. Only `destroy` has no default.
    pub const VTable = struct {
        destroy: *const fn (*Control, Allocator) void,
        collectThemeTypes: *const fn (*const Control, *ThemeTypes) void = defaultCollectThemeTypes,
        getMinimumSize: *const fn (*const Control) Error!Vec2 = defaultMinimumSize,
        draw: *const fn (*const Control, *Canvas) Error!void = defaultDraw,
        layoutChildren: *const fn (*Control) Error!void = defaultLayoutChildren,
        resized: *const fn (*Control) void = defaultResized,
        sortChildren: *const fn (*Control) Error!void = defaultSortChildren,
    };

    /// A control of `context` with the virtual functions `vtable`.
    pub fn init(context: *Context, vtable: *const VTable) Control {
        return .{ .vtable = vtable, .context = context };
    }

    /// Releases the subtree below the control and what the control itself owns.
    pub fn deinit(self: *Control, gpa: Allocator) void {
        for (self.children.items) |child| child.vtable.destroy(child, gpa);
        self.children.deinit(gpa);
        ui_theme.deinitOwned(Color4, &self.color_overrides, gpa);
        ui_theme.deinitOwned(f32, &self.constant_overrides, gpa);
    }

    /// The allocator of the owning context.
    pub fn allocator(self: *const Control) Allocator {
        return self.context.gpa;
    }

    /// Allocates a `T` from `value`, appends its `control` field as a child and returns it.
    pub fn spawn(self: *Control, comptime T: type, value: T) Error!*T {
        const gpa = self.allocator();
        const node = try gpa.create(T);
        errdefer gpa.destroy(node);
        node.* = value;
        try self.addChild(&node.control);
        return node;
    }

    /// Appends a child and takes ownership of it.
    pub fn addChild(self: *Control, child: *Control) Error!void {
        try self.children.append(self.allocator(), child);
        child.parent = self;
        self.updateMinimumSize();
        self.queueLayout();
    }

    /// Sets one anchor, clamped to [0, 1], keeping the edge in place when `keep_offset` is set.
    pub fn setAnchor(self: *Control, side: Side, value: f32, keep_offset: bool) void {
        const index = @intFromEnum(side);
        const previous = self.anchor[index];
        self.anchor[index] = vm.clampf(value, 0.0, 1.0);
        if (keep_offset) {
            if (self.parent) |parent| {
                const area = if (index % 2 == 0) parent.size.x else parent.size.y;
                self.offset[index] -= (self.anchor[index] - previous) * area;
            }
        }
        self.queueLayout();
    }

    /// Sets the distance of one edge from its anchor.
    pub fn setOffset(self: *Control, side: Side, value: f32) void {
        self.offset[@intFromEnum(side)] = value;
        self.queueLayout();
    }

    /// Sets all four anchors from `preset`.
    pub fn setAnchorsPreset(self: *Control, preset: LayoutPreset, keep_offsets: bool) void {
        const set = presetAnchors(preset);
        self.setAnchor(.left, set.left, keep_offsets);
        self.setAnchor(.top, set.top, keep_offsets);
        self.setAnchor(.right, set.right, keep_offsets);
        self.setAnchor(.bottom, set.bottom, keep_offsets);
    }

    /// Sets all four offsets so the control fits `preset` at its current or minimum size.
    pub fn setOffsetsPreset(self: *Control, preset: LayoutPreset, margin: f32) Error!void {
        const set = presetAnchors(preset);
        const minimum = try self.combinedMinimumSize();
        const target = self.size.max(minimum);

        const stretch_h = set.left != set.right;
        const stretch_v = set.top != set.bottom;

        if (stretch_h) {
            self.offset[0] = margin;
            self.offset[2] = -margin;
        } else if (set.left == 0.0) {
            self.offset[0] = margin;
            self.offset[2] = margin + target.x;
        } else if (set.left == 1.0) {
            self.offset[0] = -margin - target.x;
            self.offset[2] = -margin;
        } else {
            self.offset[0] = -target.x * 0.5;
            self.offset[2] = target.x * 0.5;
        }

        if (stretch_v) {
            self.offset[1] = margin;
            self.offset[3] = -margin;
        } else if (set.top == 0.0) {
            self.offset[1] = margin;
            self.offset[3] = margin + target.y;
        } else if (set.top == 1.0) {
            self.offset[1] = -margin - target.y;
            self.offset[3] = -margin;
        } else {
            self.offset[1] = -target.y * 0.5;
            self.offset[3] = target.y * 0.5;
        }
        self.queueLayout();
    }

    /// Applies `preset` to both anchors and offsets.
    pub fn setAnchorsAndOffsetsPreset(self: *Control, preset: LayoutPreset, margin: f32) Error!void {
        self.setAnchorsPreset(preset, false);
        try self.setOffsetsPreset(preset, margin);
    }

    /// Sets position and size, not below the minimum size, without queuing a layout.
    pub fn setRect(self: *Control, rect: Rect2) Error!void {
        const target = rect.size.max(try self.combinedMinimumSize());
        self.position = rect.position;
        self.offset[0] = rect.position.x;
        self.offset[1] = rect.position.y;
        self.offset[2] = rect.position.x + target.x;
        self.offset[3] = rect.position.y + target.y;
        if (!self.size.eql(target)) {
            self.size = target;
            self.vtable.resized(self);
        }
    }

    /// Sets a floor for the minimum size.
    pub fn setCustomMinimumSize(self: *Control, size: Vec2) void {
        if (self.custom_minimum_size.eql(size)) return;
        self.custom_minimum_size = size;
        self.updateMinimumSize();
    }

    /// The larger of the custom and intrinsic minimum sizes, cached until invalidated.
    pub fn combinedMinimumSize(self: *Control) Error!Vec2 {
        if (!self.minimum_size_valid) {
            self.minimum_size_cache = self.custom_minimum_size.max(try self.vtable.getMinimumSize(self));
            self.minimum_size_valid = true;
        }
        return self.minimum_size_cache;
    }

    /// Invalidates the cached minimum size here and in every ancestor, and queues a layout.
    pub fn updateMinimumSize(self: *Control) void {
        self.minimum_size_valid = false;
        var current = self.parent;
        while (current) |ancestor| : (current = ancestor.parent) ancestor.minimum_size_valid = false;
        self.queueLayout();
    }

    /// Sets how a container sizes this control horizontally.
    pub fn setHSizeFlags(self: *Control, flags: SizeFlags) void {
        self.h_size_flags = flags;
        self.queueLayout();
    }

    /// Sets how a container sizes this control vertically.
    pub fn setVSizeFlags(self: *Control, flags: SizeFlags) void {
        self.v_size_flags = flags;
        self.queueLayout();
    }

    /// Sets the share of spare space among expanding siblings, floored at zero.
    pub fn setStretchRatio(self: *Control, ratio: f32) void {
        self.stretch_ratio = vm.maxf(ratio, 0.0);
        self.queueLayout();
    }

    /// Shows or hides the control and its subtree.
    pub fn setVisible(self: *Control, visible: bool) void {
        if (self.visible == visible) return;
        self.visible = visible;
        self.updateMinimumSize();
    }

    /// The colour multiplied through every ancestor.
    pub fn effectiveModulate(self: *const Control) Color4 {
        var result = self.modulate;
        var current = self.parent;
        while (current) |ancestor| : (current = ancestor.parent) result = result.modulated(ancestor.modulate);
        return result;
    }

    /// Whether this control holds keyboard focus.
    pub fn hasFocus(self: *const Control) bool {
        const focused = self.context.focus orelse return false;
        return focused == self;
    }

    /// Sets a theme for this control and its descendants; it must outlive the control.
    pub fn setTheme(self: *Control, theme: *const Theme) void {
        self.theme = theme;
        self.updateMinimumSize();
    }

    /// Sets a theme type searched before the control types; the name must outlive the control.
    pub fn setThemeTypeVariation(self: *Control, type_name: []const u8) void {
        self.theme_type_variation = type_name;
        self.updateMinimumSize();
    }

    /// Overrides a colour by name for this control only.
    pub fn addColorOverride(self: *Control, name: []const u8, value: Color4) Error!void {
        try ui_theme.putOwned(Color4, &self.color_overrides, self.allocator(), name, value);
        self.updateMinimumSize();
    }

    /// Overrides a constant by name for this control only.
    pub fn addConstantOverride(self: *Control, name: []const u8, value: f32) Error!void {
        try ui_theme.putOwned(f32, &self.constant_overrides, self.allocator(), name, value);
        self.updateMinimumSize();
    }

    /// Resolves a style box by name through ancestor themes and the context theme.
    pub fn themeStylebox(self: *const Control, name: []const u8) *const StyleBox {
        return findInThemes(StyleBox, Theme.stylebox, self, name) orelse &fallback_stylebox;
    }

    /// Resolves a colour by name through overrides, ancestor themes and the context theme.
    pub fn themeColor(self: *const Control, name: []const u8) Color4 {
        if (self.color_overrides.get(name)) |override| return override;
        const found = findInThemes(Color4, Theme.color, self, name) orelse return .{};
        return found.*;
    }

    /// Resolves a constant by name through overrides, ancestor themes and the context theme.
    pub fn themeConstant(self: *const Control, name: []const u8) f32 {
        if (self.constant_overrides.get(name)) |override| return override;
        const found = findInThemes(f32, Theme.constant, self, name) orelse return 0.0;
        return found.*;
    }

    /// Resolves a font size by name through ancestor themes and the context theme.
    pub fn themeFontSize(self: *const Control, name: []const u8) f32 {
        const found = findInThemes(f32, Theme.fontSize, self, name) orelse return 16.0;
        return found.*;
    }

    /// The canvas of the owning context.
    pub fn canvas(self: *const Control) *Canvas {
        return self.context.canvas;
    }

    /// Marks the owning context for layout on its next update.
    pub fn queueLayout(self: *const Control) void {
        self.context.queueLayout();
    }

    /// Computes position and size from anchors and offsets inside `parent_size`.
    pub fn applyAnchors(self: *Control, parent_size: Vec2) Error!void {
        var edges: [4]f32 = undefined;
        for (&edges, self.offset, self.anchor, 0..) |*edge, offset, anchor, i| {
            const area = if (i % 2 == 0) parent_size.x else parent_size.y;
            edge.* = offset + anchor * area;
        }

        var new_position: Vec2 = .{ .x = edges[0], .y = edges[1] };
        var new_size: Vec2 = .{ .x = edges[2] - edges[0], .y = edges[3] - edges[1] };
        const minimum = try self.combinedMinimumSize();

        if (minimum.x > new_size.x) {
            if (self.h_grow == .begin) {
                new_position.x += new_size.x - minimum.x;
            } else if (self.h_grow == .both) {
                new_position.x += (new_size.x - minimum.x) * 0.5;
            }
            new_size.x = minimum.x;
        }
        if (minimum.y > new_size.y) {
            if (self.v_grow == .begin) {
                new_position.y += new_size.y - minimum.y;
            } else if (self.v_grow == .both) {
                new_position.y += (new_size.y - minimum.y) * 0.5;
            }
            new_size.y = minimum.y;
        }

        self.position = new_position;
        if (!self.size.eql(new_size)) {
            self.size = new_size;
            self.vtable.resized(self);
        }
    }

    /// Appends the theme type name of the base control.
    pub fn collectBaseThemeTypes(types: *ThemeTypes) void {
        types.appendAssumeCapacity("Control");
    }

    /// Default `collectThemeTypes`: just the base control type.
    fn defaultCollectThemeTypes(_: *const Control, types: *ThemeTypes) void {
        collectBaseThemeTypes(types);
    }

    /// Default `getMinimumSize`: zero.
    fn defaultMinimumSize(_: *const Control) Error!Vec2 {
        return .{};
    }

    /// Default `draw`: nothing.
    fn defaultDraw(_: *const Control, _: *Canvas) Error!void {}

    /// Default `layoutChildren`: applies the anchors of every child.
    fn defaultLayoutChildren(self: *Control) Error!void {
        for (self.children.items) |child| try child.applyAnchors(self.size);
    }

    /// Default `resized`: nothing.
    fn defaultResized(_: *Control) void {}

    /// Default `sortChildren`: nothing.
    fn defaultSortChildren(_: *Control) Error!void {}

    /// Destroys a plain control that was allocated on its own.
    fn destroyPlain(self: *Control, gpa: Allocator) void {
        self.deinit(gpa);
        gpa.destroy(self);
    }
};

const root_vtable: Control.VTable = .{ .destroy = Control.destroyPlain };

/// The owner of a control tree: the root control, the fallback theme, the
/// canvas, the viewport and the dirty flag the per-frame update checks.
pub const Context = struct {
    gpa: Allocator,
    canvas: *Canvas,
    root: *Control,
    theme: *Theme,
    focus: ?*Control = null,
    viewport_rect: Rect2 = .{},
    layout_dirty: bool = true,

    /// A heap-allocated context with an empty root and the default theme that paints into `canvas`.
    pub fn create(gpa: Allocator, canvas: *Canvas) Error!*Context {
        const self = try gpa.create(Context);
        errdefer gpa.destroy(self);
        const theme = try ui_theme.defaultTheme(gpa);
        errdefer theme.destroy();
        const root = try gpa.create(Control);
        errdefer gpa.destroy(root);
        self.* = .{ .gpa = gpa, .canvas = canvas, .root = root, .theme = theme };
        root.* = Control.init(self, &root_vtable);
        return self;
    }

    /// Frees the context, its theme and the whole tree.
    pub fn destroy(self: *Context) void {
        const gpa = self.gpa;
        self.root.vtable.destroy(self.root, gpa);
        self.theme.destroy();
        gpa.destroy(self);
    }

    /// Moves the keyboard focus to `control`, or clears it.
    pub fn setFocus(self: *Context, control: ?*Control) void {
        self.focus = control;
    }

    /// Sets the viewport rectangle, queuing a layout when it changes.
    pub fn setViewportRect(self: *Context, rect: Rect2) void {
        if (self.viewport_rect.eql(rect)) return;
        self.viewport_rect = rect;
        self.queueLayout();
    }

    /// Marks the tree for layout on the next update.
    pub fn queueLayout(self: *Context) void {
        self.layout_dirty = true;
    }

    /// Lays out `control`, then its visible children.
    fn layout(self: *Context, control: *Control) Error!void {
        try control.vtable.layoutChildren(control);
        for (control.children.items) |child| {
            if (child.visible) try self.layout(child);
        }
    }

    /// Fits the root to the viewport and, when a layout is queued, lays out the tree.
    pub fn update(self: *Context) Error!void {
        try self.root.setRect(self.viewport_rect);
        if (!self.layout_dirty) return;
        self.layout_dirty = false;
        try self.layout(self.root);
    }

    /// Paints `control` and its subtree with `origin` as its parent's global position.
    fn paint(self: *Context, control: *const Control, origin: Vec2) Error!void {
        if (!control.visible) return;
        const target = self.canvas;
        const position = origin.add(control.position);
        try target.pushOffset(control.position);
        if (control.clip_contents) try target.pushClip(.{ .position = position, .size = control.size });
        try control.vtable.draw(control, target);
        for (control.children.items) |child| try self.paint(child, position);
        if (control.clip_contents) target.popClip();
        target.popOffset();
    }

    /// Paints the tree into the canvas.
    pub fn draw(self: *Context) Error!void {
        try self.paint(self.root, .{});
    }
};
