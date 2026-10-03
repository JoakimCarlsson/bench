//! Containers: controls that position their visible children themselves.
const std = @import("std");
const vm = @import("vecmath.zig");
const math = @import("ui_math.zig");
const ui_control = @import("ui_control.zig");
const ui_draw = @import("ui_draw.zig");
const ui_theme = @import("ui_theme.zig");

const Allocator = std.mem.Allocator;
const Vec2 = math.Vec2;
const Rect2 = math.Rect2;
const Canvas = ui_draw.Canvas;
const Margins = ui_theme.Margins;
const Control = ui_control.Control;
const ThemeTypes = ui_control.ThemeTypes;
const Error = ui_control.Error;
const hasFlag = ui_control.hasFlag;

/// The behaviour every container shares: it positions its visible children
/// through the `sortChildren` slot of its vtable.
pub const Container = struct {
    /// Appends the container type name and then the base control type.
    pub fn collectThemeTypes(types: *ThemeTypes) void {
        types.appendAssumeCapacity("Container");
        Control.collectBaseThemeTypes(types);
    }

    /// The visible children of `control`, in order; the caller frees the list.
    pub fn sortableChildren(control: *const Control) Error!std.ArrayList(*Control) {
        var result = try std.ArrayList(*Control).initCapacity(control.allocator(), control.children.items.len);
        for (control.children.items) |child| {
            if (child.visible) result.appendAssumeCapacity(child);
        }
        return result;
    }

    /// Places every sortable child.
    pub fn layoutChildren(control: *Control) Error!void {
        try control.vtable.sortChildren(control);
    }

    /// The largest minimum size among the visible children of `control`.
    fn largestChild(control: *const Control) Error!Vec2 {
        var entries = try sortableChildren(control);
        defer entries.deinit(control.allocator());
        var result: Vec2 = .{};
        for (entries.items) |child| result = result.max(try child.combinedMinimumSize());
        return result;
    }

    /// Places `child` in `rect`, shrinking and aligning it by its size flags.
    pub fn fitChildInRect(child: *Control, rect: Rect2) Error!void {
        const minimum = try child.combinedMinimumSize();
        var target = rect;

        if (!hasFlag(child.h_size_flags, .fill)) {
            const width = vm.minf(vm.maxf(minimum.x, 0.0), rect.size.x);
            target.size.x = vm.maxf(width, minimum.x);
            if (hasFlag(child.h_size_flags, .shrink_end)) {
                target.position.x += rect.size.x - target.size.x;
            } else if (hasFlag(child.h_size_flags, .shrink_center)) {
                target.position.x += (rect.size.x - target.size.x) * 0.5;
            }
        }

        if (!hasFlag(child.v_size_flags, .fill)) {
            const height = vm.minf(vm.maxf(minimum.y, 0.0), rect.size.y);
            target.size.y = vm.maxf(height, minimum.y);
            if (hasFlag(child.v_size_flags, .shrink_end)) {
                target.position.y += rect.size.y - target.size.y;
            } else if (hasFlag(child.v_size_flags, .shrink_center)) {
                target.position.y += (rect.size.y - target.size.y) * 0.5;
            }
        }

        try child.setRect(target);
    }
};

/// Lays children out in a row or a column with a themed separation. Created
/// as a `VBoxContainer` or an `HBoxContainer`, which differ in their theme
/// type name.
pub const BoxContainer = struct {
    control: Control,
    vertical: bool,

    const vbox_vtable: Control.VTable = .{
        .destroy = ui_control.destroyFor(BoxContainer),
        .collectThemeTypes = collectVertical,
        .getMinimumSize = getMinimumSize,
        .layoutChildren = Container.layoutChildren,
        .sortChildren = sortChildren,
    };

    const hbox_vtable: Control.VTable = .{
        .destroy = ui_control.destroyFor(BoxContainer),
        .collectThemeTypes = collectHorizontal,
        .getMinimumSize = getMinimumSize,
        .layoutChildren = Container.layoutChildren,
        .sortChildren = sortChildren,
    };

    /// A column added to `parent`.
    pub fn createVertical(parent: *Control) Error!*BoxContainer {
        return parent.spawn(BoxContainer, .{ .control = Control.init(parent.context, &vbox_vtable), .vertical = true });
    }

    /// A row added to `parent`.
    pub fn createHorizontal(parent: *Control) Error!*BoxContainer {
        return parent.spawn(BoxContainer, .{ .control = Control.init(parent.context, &hbox_vtable), .vertical = false });
    }

    /// Appends the box type name and then the container types.
    fn collectBox(types: *ThemeTypes) void {
        types.appendAssumeCapacity("BoxContainer");
        Container.collectThemeTypes(types);
    }

    /// Theme types of a column.
    fn collectVertical(_: *const Control, types: *ThemeTypes) void {
        types.appendAssumeCapacity("VBoxContainer");
        collectBox(types);
    }

    /// Theme types of a row.
    fn collectHorizontal(_: *const Control, types: *ThemeTypes) void {
        types.appendAssumeCapacity("HBoxContainer");
        collectBox(types);
    }

    /// The extent of the children along the box plus the separations, and the largest across it.
    fn getMinimumSize(control: *const Control) Error!Vec2 {
        const self: *const BoxContainer = @fieldParentPtr("control", control);
        var entries = try Container.sortableChildren(control);
        defer entries.deinit(control.allocator());
        const separation = control.themeConstant("separation");
        var result: Vec2 = .{};
        for (entries.items, 0..) |child, index| {
            const minimum = try child.combinedMinimumSize();
            if (self.vertical) {
                result.x = vm.maxf(result.x, minimum.x);
                result.y += minimum.y;
                if (index + 1 < entries.items.len) result.y += separation;
            } else {
                result.y = vm.maxf(result.y, minimum.y);
                result.x += minimum.x;
                if (index + 1 < entries.items.len) result.x += separation;
            }
        }
        return result;
    }

    /// Places the children one after another, sharing spare space among the expanding ones.
    fn sortChildren(control: *Control) Error!void {
        const self: *const BoxContainer = @fieldParentPtr("control", control);
        var entries = try Container.sortableChildren(control);
        defer entries.deinit(control.allocator());
        if (entries.items.len == 0) return;
        const vertical = self.vertical;
        const separation = control.themeConstant("separation");
        const extent = if (vertical) control.size.y else control.size.x;
        const cross = if (vertical) control.size.x else control.size.y;

        var used = separation * @as(f32, @floatFromInt(entries.items.len - 1));
        var total_ratio: f32 = 0.0;
        for (entries.items) |child| {
            const minimum = try child.combinedMinimumSize();
            used += if (vertical) minimum.y else minimum.x;
            const flags = if (vertical) child.v_size_flags else child.h_size_flags;
            if (hasFlag(flags, .expand)) total_ratio += child.stretch_ratio;
        }

        const remaining = vm.maxf(extent - used, 0.0);
        var cursor: f32 = 0.0;
        for (entries.items, 0..) |child, index| {
            const minimum = try child.combinedMinimumSize();
            var length = if (vertical) minimum.y else minimum.x;
            const flags = if (vertical) child.v_size_flags else child.h_size_flags;
            if (hasFlag(flags, .expand) and total_ratio > 0.0) length += remaining * (child.stretch_ratio / total_ratio);
            const slot: Rect2 = if (vertical)
                .{ .position = .{ .x = 0.0, .y = cursor }, .size = .{ .x = cross, .y = length } }
            else
                .{ .position = .{ .x = cursor, .y = 0.0 }, .size = .{ .x = length, .y = cross } };
            try Container.fitChildInRect(child, slot);
            cursor += length;
            if (index + 1 < entries.items.len) cursor += separation;
        }
    }
};

/// Insets its children by the themed `margin_*` constants.
pub const MarginContainer = struct {
    control: Control,

    const vtable: Control.VTable = .{
        .destroy = ui_control.destroyFor(MarginContainer),
        .collectThemeTypes = collectThemeTypes,
        .getMinimumSize = getMinimumSize,
        .layoutChildren = Container.layoutChildren,
        .sortChildren = sortChildren,
    };

    /// A margin container added to `parent`.
    pub fn create(parent: *Control) Error!*MarginContainer {
        return parent.spawn(MarginContainer, .{ .control = Control.init(parent.context, &vtable) });
    }

    /// Appends the type names of a margin container.
    fn collectThemeTypes(_: *const Control, types: *ThemeTypes) void {
        types.appendAssumeCapacity("MarginContainer");
        Container.collectThemeTypes(types);
    }

    /// The four themed margins.
    fn margins(control: *const Control) Margins {
        return .{
            .left = control.themeConstant("margin_left"),
            .top = control.themeConstant("margin_top"),
            .right = control.themeConstant("margin_right"),
            .bottom = control.themeConstant("margin_bottom"),
        };
    }

    /// The largest child plus the margins.
    fn getMinimumSize(control: *const Control) Error!Vec2 {
        const largest = try Container.largestChild(control);
        return largest.add(margins(control).size());
    }

    /// Places every child inside the margins.
    fn sortChildren(control: *Control) Error!void {
        const margin = margins(control);
        const slot: Rect2 = .{
            .position = .{ .x = margin.left, .y = margin.top },
            .size = control.size.sub(margin.size()).max(.{}),
        };
        var entries = try Container.sortableChildren(control);
        defer entries.deinit(control.allocator());
        for (entries.items) |child| try Container.fitChildInRect(child, slot);
    }
};

/// Centres each child at its minimum size.
pub const CenterContainer = struct {
    control: Control,

    const vtable: Control.VTable = .{
        .destroy = ui_control.destroyFor(CenterContainer),
        .collectThemeTypes = collectThemeTypes,
        .getMinimumSize = getMinimumSize,
        .layoutChildren = Container.layoutChildren,
        .sortChildren = sortChildren,
    };

    /// A centring container added to `parent`.
    pub fn create(parent: *Control) Error!*CenterContainer {
        return parent.spawn(CenterContainer, .{ .control = Control.init(parent.context, &vtable) });
    }

    /// Appends the type names of a centring container.
    fn collectThemeTypes(_: *const Control, types: *ThemeTypes) void {
        types.appendAssumeCapacity("CenterContainer");
        Container.collectThemeTypes(types);
    }

    /// The largest child.
    fn getMinimumSize(control: *const Control) Error!Vec2 {
        return Container.largestChild(control);
    }

    /// Centres every child at its minimum size.
    fn sortChildren(control: *Control) Error!void {
        var entries = try Container.sortableChildren(control);
        defer entries.deinit(control.allocator());
        for (entries.items) |child| {
            const minimum = try child.combinedMinimumSize();
            const position = control.size.sub(minimum).scale(0.5);
            try child.setRect(.{ .position = position, .size = minimum });
        }
    }
};

/// Draws the themed panel style box and insets its children by its content margins.
pub const PanelContainer = struct {
    control: Control,

    const vtable: Control.VTable = .{
        .destroy = ui_control.destroyFor(PanelContainer),
        .collectThemeTypes = collectThemeTypes,
        .getMinimumSize = getMinimumSize,
        .draw = draw,
        .layoutChildren = Container.layoutChildren,
        .sortChildren = sortChildren,
    };

    /// A panel container added to `parent`.
    pub fn create(parent: *Control) Error!*PanelContainer {
        return parent.spawn(PanelContainer, .{ .control = Control.init(parent.context, &vtable) });
    }

    /// Appends the type names of a panel container.
    fn collectThemeTypes(_: *const Control, types: *ThemeTypes) void {
        types.appendAssumeCapacity("PanelContainer");
        Container.collectThemeTypes(types);
    }

    /// The largest child plus the panel's content margins.
    fn getMinimumSize(control: *const Control) Error!Vec2 {
        const largest = try Container.largestChild(control);
        return largest.add(control.themeStylebox("panel").content_margins.size());
    }

    /// Draws the panel style box.
    fn draw(control: *const Control, canvas: *Canvas) Error!void {
        try control.themeStylebox("panel").draw(canvas, .{ .size = control.size }, control.effectiveModulate());
    }

    /// Places every child inside the panel's content rectangle.
    fn sortChildren(control: *Control) Error!void {
        const slot = control.themeStylebox("panel").contentRect(.{ .size = control.size });
        var entries = try Container.sortableChildren(control);
        defer entries.deinit(control.allocator());
        for (entries.items) |child| try Container.fitChildInRect(child, slot);
    }
};

/// Lays children out in a grid with a fixed column count.
pub const GridContainer = struct {
    control: Control,
    columns: u32 = 1,

    const vtable: Control.VTable = .{
        .destroy = ui_control.destroyFor(GridContainer),
        .collectThemeTypes = collectThemeTypes,
        .getMinimumSize = getMinimumSize,
        .layoutChildren = Container.layoutChildren,
        .sortChildren = sortChildren,
    };

    /// Column widths and row heights of a grid.
    const Metrics = struct {
        columns: []f32,
        rows: std.ArrayList(f32),

        /// Frees both lists.
        fn deinit(self: *Metrics, gpa: Allocator) void {
            gpa.free(self.columns);
            self.rows.deinit(gpa);
        }
    };

    /// A grid with one column added to `parent`.
    pub fn create(parent: *Control) Error!*GridContainer {
        return parent.spawn(GridContainer, .{ .control = Control.init(parent.context, &vtable) });
    }

    /// Appends the type names of a grid container.
    fn collectThemeTypes(_: *const Control, types: *ThemeTypes) void {
        types.appendAssumeCapacity("GridContainer");
        Container.collectThemeTypes(types);
    }

    /// Sets the column count, at least one.
    pub fn setColumns(self: *GridContainer, columns: u32) void {
        const target = @max(columns, 1);
        if (self.columns == target) return;
        self.columns = target;
        self.control.updateMinimumSize();
    }

    /// The widest cell of each column and the tallest of each row.
    fn measure(control: *const Control, columns: u32) Error!Metrics {
        const gpa = control.allocator();
        var entries = try Container.sortableChildren(control);
        defer entries.deinit(gpa);
        const widths = try gpa.alloc(f32, columns);
        errdefer gpa.free(widths);
        @memset(widths, 0.0);
        var rows: std.ArrayList(f32) = .empty;
        errdefer rows.deinit(gpa);
        for (entries.items, 0..) |child, index| {
            const column = index % columns;
            const row = index / columns;
            if (rows.items.len <= row) try rows.append(gpa, 0.0);
            const minimum = try child.combinedMinimumSize();
            widths[column] = vm.maxf(widths[column], minimum.x);
            rows.items[row] = vm.maxf(rows.items[row], minimum.y);
        }
        return .{ .columns = widths, .rows = rows };
    }

    /// The summed columns and rows plus the themed separations.
    fn getMinimumSize(control: *const Control) Error!Vec2 {
        const self: *const GridContainer = @fieldParentPtr("control", control);
        var metrics = try measure(control, self.columns);
        defer metrics.deinit(control.allocator());
        const horizontal = control.themeConstant("h_separation");
        const vertical = control.themeConstant("v_separation");

        var result: Vec2 = .{};
        for (metrics.columns) |width| result.x += width;
        for (metrics.rows.items) |height| result.y += height;
        if (metrics.columns.len > 0) result.x += horizontal * @as(f32, @floatFromInt(metrics.columns.len - 1));
        if (metrics.rows.items.len > 0) result.y += vertical * @as(f32, @floatFromInt(metrics.rows.items.len - 1));
        return result;
    }

    /// Places every child in its cell, sharing spare space evenly among the columns and rows.
    fn sortChildren(control: *Control) Error!void {
        const self: *const GridContainer = @fieldParentPtr("control", control);
        const gpa = control.allocator();
        var entries = try Container.sortableChildren(control);
        defer entries.deinit(gpa);
        if (entries.items.len == 0) return;
        var metrics = try measure(control, self.columns);
        defer metrics.deinit(gpa);
        const column_widths = metrics.columns;
        const row_heights = metrics.rows.items;
        const horizontal = control.themeConstant("h_separation");
        const vertical = control.themeConstant("v_separation");

        const minimum = try getMinimumSize(control);
        const extra = control.size.sub(minimum).max(.{});
        if (column_widths.len > 0 and extra.x > 0.0) {
            const share = extra.x / @as(f32, @floatFromInt(column_widths.len));
            for (column_widths) |*width| width.* += share;
        }
        if (row_heights.len > 0 and extra.y > 0.0) {
            const share = extra.y / @as(f32, @floatFromInt(row_heights.len));
            for (row_heights) |*height| height.* += share;
        }

        for (entries.items, 0..) |child, index| {
            const column = index % self.columns;
            const row = index / self.columns;
            var x: f32 = 0.0;
            for (column_widths[0..column]) |width| x += width + horizontal;
            var y: f32 = 0.0;
            for (row_heights[0..row]) |height| y += height + vertical;
            try Container.fitChildInRect(child, .{
                .position = .{ .x = x, .y = y },
                .size = .{ .x = column_widths[column], .y = row_heights[row] },
            });
        }
    }
};

/// Shows one of its children under a bar of titled tabs.
pub const TabContainer = struct {
    control: Control,
    titles: std.ArrayList([]const u8) = .empty,
    current: usize = 0,

    const vtable: Control.VTable = .{
        .destroy = destroy,
        .collectThemeTypes = collectThemeTypes,
        .getMinimumSize = getMinimumSize,
        .draw = draw,
        .layoutChildren = Container.layoutChildren,
        .sortChildren = sortChildren,
    };

    /// A tab container added to `parent`.
    pub fn create(parent: *Control) Error!*TabContainer {
        return parent.spawn(TabContainer, .{ .control = Control.init(parent.context, &vtable) });
    }

    /// Releases the subtree and the titles, and frees the container.
    fn destroy(control: *Control, gpa: Allocator) void {
        const self: *TabContainer = @fieldParentPtr("control", control);
        for (self.titles.items) |title| gpa.free(title);
        self.titles.deinit(gpa);
        control.deinit(gpa);
        gpa.destroy(self);
    }

    /// Appends the type names of a tab container.
    fn collectThemeTypes(_: *const Control, types: *ThemeTypes) void {
        types.appendAssumeCapacity("TabContainer");
        Container.collectThemeTypes(types);
    }

    /// Sets the title of tab `index`.
    pub fn setTabTitle(self: *TabContainer, index: usize, title: []const u8) Error!void {
        const gpa = self.control.allocator();
        while (self.titles.items.len <= index) try self.titles.append(gpa, &.{});
        try ui_control.replaceString(gpa, &self.titles.items[index], title);
        self.control.updateMinimumSize();
    }

    /// The title of tab `index`, or empty.
    fn tabTitle(self: *const TabContainer, index: usize) []const u8 {
        return if (index < self.titles.items.len) self.titles.items[index] else "";
    }

    /// Shows tab `index`, clamped to the last one.
    pub fn setCurrentTab(self: *TabContainer, index: usize) void {
        const count = self.control.children.items.len;
        if (count == 0) {
            self.current = 0;
            return;
        }
        const target = @min(index, count - 1);
        if (self.current == target) return;
        self.current = target;
        self.control.queueLayout();
    }

    /// The height of the tab bar.
    fn tabBarHeight(control: *const Control) f32 {
        const font_size = control.themeFontSize("font_size");
        const content = control.themeStylebox("tab_selected").content_margins;
        return control.canvas().metrics(font_size).line_height + content.size().y;
    }

    /// The rectangle of every tab in the bar; the caller frees the list.
    fn tabRects(self: *const TabContainer) Error!std.ArrayList(Rect2) {
        const control = &self.control;
        const gpa = control.allocator();
        const font_size = control.themeFontSize("font_size");
        const separation = control.themeConstant("h_separation");
        const content = control.themeStylebox("tab_selected").content_margins;
        const height = tabBarHeight(control);
        const target = control.canvas();

        const count = control.children.items.len;
        var result = try std.ArrayList(Rect2).initCapacity(gpa, count);
        errdefer result.deinit(gpa);
        var x: f32 = 0.0;
        for (0..count) |index| {
            const width = target.measureText(self.tabTitle(index), font_size).x + content.size().x;
            result.appendAssumeCapacity(.{ .position = .{ .x = x, .y = 0.0 }, .size = .{ .x = width, .y = height } });
            x += width + separation;
        }
        return result;
    }

    /// The largest child plus the panel margins, widened to the tab bar, plus the bar height.
    fn getMinimumSize(control: *const Control) Error!Vec2 {
        const self: *const TabContainer = @fieldParentPtr("control", control);
        var content = try Container.largestChild(control);
        const panel = control.themeStylebox("panel").content_margins;
        content = content.add(panel.size());

        var tabs = try self.tabRects();
        defer tabs.deinit(control.allocator());
        var bar_width: f32 = 0.0;
        for (tabs.items) |tab| bar_width = vm.maxf(bar_width, tab.right());
        return .{ .x = vm.maxf(content.x, bar_width), .y = content.y + tabBarHeight(control) };
    }

    /// Shows the current child and hides the others.
    fn syncVisibility(self: *TabContainer) void {
        for (self.control.children.items, 0..) |child, index| child.setVisible(index == self.current);
    }

    /// Shows the current tab and fits it under the tab bar.
    fn sortChildren(control: *Control) Error!void {
        const self: *TabContainer = @fieldParentPtr("control", control);
        const count = control.children.items.len;
        if (count == 0) return;
        self.current = @min(self.current, count - 1);
        self.syncVisibility();

        const bar = tabBarHeight(control);
        const body: Rect2 = .{
            .position = .{ .x = 0.0, .y = bar },
            .size = .{ .x = control.size.x, .y = vm.maxf(control.size.y - bar, 0.0) },
        };
        const slot = control.themeStylebox("panel").contentRect(body);

        for (control.children.items, 0..) |child, index| {
            if (index == self.current) try Container.fitChildInRect(child, slot);
        }
    }

    /// Draws the panel and the tab bar.
    fn draw(control: *const Control, canvas: *Canvas) Error!void {
        const self: *const TabContainer = @fieldParentPtr("control", control);
        const tint = control.effectiveModulate();
        const bar = tabBarHeight(control);
        const body: Rect2 = .{
            .position = .{ .x = 0.0, .y = bar },
            .size = .{ .x = control.size.x, .y = vm.maxf(control.size.y - bar, 0.0) },
        };
        try control.themeStylebox("panel").draw(canvas, body, tint);

        const font_size = control.themeFontSize("font_size");
        var tabs = try self.tabRects();
        defer tabs.deinit(control.allocator());
        for (tabs.items, 0..) |tab, index| {
            const selected = index == self.current;
            const style: []const u8 = if (selected) "tab_selected" else "tab_unselected";
            try control.themeStylebox(style).draw(canvas, tab, tint);

            const color = if (selected) control.themeColor("font_selected_color") else control.themeColor("font_unselected_color");
            const content = control.themeStylebox(style).contentRect(tab);
            const measured = canvas.measureText(self.tabTitle(index), font_size);
            try canvas.drawText(
                .{ .x = content.position.x, .y = content.position.y + (content.size.y - measured.y) * 0.5 },
                self.tabTitle(index),
                font_size,
                color.modulated(tint),
            );
        }
    }
};
