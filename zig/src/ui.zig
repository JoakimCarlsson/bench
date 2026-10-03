//! The engine's game-facing UI: a tree of about sixteen thousand controls
//! (an editor screen with a toolbar, a long hierarchy, a tabbed asset grid
//! and an inspector with long property lists) laid out and drawn into a
//! batched vertex, index and command list. Each of the frames resizes the
//! root, changes the text and minimum size of some controls so the
//! invalidation climbs the tree, runs the layout pass, draws the whole
//! tree, and folds every control rect and all draw data into the checksum.
const std = @import("std");
const hash = @import("hash.zig");
const math = @import("ui_math.zig");
const ui_container = @import("ui_container.zig");
const ui_control = @import("ui_control.zig");
const ui_draw = @import("ui_draw.zig");
const ui_theme = @import("ui_theme.zig");
const ui_widgets = @import("ui_widgets.zig");

const Allocator = std.mem.Allocator;
const Vec2 = math.Vec2;
const Rect2 = math.Rect2;
const Color4 = math.Color4;
const Control = ui_control.Control;
const Context = ui_control.Context;
const Error = ui_control.Error;
const DrawData = ui_draw.DrawData;
const BoxContainer = ui_container.BoxContainer;
const CenterContainer = ui_container.CenterContainer;
const GridContainer = ui_container.GridContainer;
const MarginContainer = ui_container.MarginContainer;
const PanelContainer = ui_container.PanelContainer;
const TabContainer = ui_container.TabContainer;
const Button = ui_widgets.Button;
const CheckBox = ui_widgets.CheckBox;
const Label = ui_widgets.Label;
const LineEdit = ui_widgets.LineEdit;
const Panel = ui_widgets.Panel;
const Slider = ui_widgets.Slider;

const Ui = @This();

pub const name = "ui";

const frames: u32 = 3;
const toolbar_buttons: u32 = 28;
const hierarchy_rows: u32 = 1300;
const asset_cards: u32 = 300;
const log_lines: u32 = 200;
const inspector_sections: u32 = 130;
const section_rows: u32 = 14;

const label_period: u32 = 8;
const button_period: u32 = 8;
const edit_period: u32 = 6;
const resizable_period: u32 = 12;
const slider_period: u32 = 3;

const fold_multiplier: u64 = 0x9E3779B97F4A7C15;

const property_names = [24][]const u8{
    "Position",       "Rotation",       "Scale",           "Mass",           "Friction",      "Restitution",
    "Linear Damping", "Angular Damping", "Gravity Scale",   "Collision Layer", "Collision Mask", "Sleep Threshold",
    "Cast Shadows",   "Receive Shadows", "Material",        "Mesh",           "Texture Region", "Tint",
    "Opacity",        "Z Index",         "Visible",         "Name",           "Tag",            "Script",
};

const section_names = [12][]const u8{
    "Transform",        "Rigid Body",       "Collider",         "Mesh Renderer", "Material", "Audio Source",
    "Particle Emitter", "Animation Player", "Script",           "Light",         "Camera",   "Navigation",
};

const tool_names = [16][]const u8{
    "New", "Open", "Save",  "Undo",  "Redo",   "Cut",    "Copy",   "Paste",
    "Play", "Pause", "Step", "Build", "Select", "Move",   "Rotate", "Scale",
};

const entity_names = [10][]const u8{
    "Player", "Crate", "Barrel", "Light", "Camera", "Spawner", "Trigger", "Door", "Terrain Chunk", "Emitter",
};

const text_pool = [16][]const u8{
    "",                         "Mass",                   "Linear Velocity",        "A rather long label for a property",
    "Two\nlines",               "Collision Layer Mask",   "x",                      "Position",
    "Angular Damping Factor",   "Three\nline\nlabel",     "Cast Shadows",           "ID",
    "Texture Atlas Region",     "Wi",                     "Material Override Slot", "Sleep Threshold Linear",
};

/// The controls the frames of the `ui` kernel change, picked while the screen is built.
const Targets = struct {
    labels: std.ArrayList(*Label) = .empty,
    buttons: std.ArrayList(*Button) = .empty,
    edits: std.ArrayList(*LineEdit) = .empty,
    resizable: std.ArrayList(*Control) = .empty,
    sliders: std.ArrayList(*Slider) = .empty,

    /// Releases the five lists.
    fn deinit(self: *Targets, gpa: Allocator) void {
        self.labels.deinit(gpa);
        self.buttons.deinit(gpa);
        self.edits.deinit(gpa);
        self.resizable.deinit(gpa);
        self.sliders.deinit(gpa);
    }
};

/// The `index`-th entry of `table`, wrapping around.
fn named(table: []const []const u8, index: usize) []const u8 {
    return table[index % table.len];
}

/// The `index`-th entry of `table` followed by a space and `number`, written into `buffer`.
fn numbered(buffer: *[48]u8, table: []const []const u8, index: usize, number: u32) []const u8 {
    return std.fmt.bufPrint(buffer, "{s} {d}", .{ named(table, index), number }) catch unreachable;
}

/// Builds the editor screen and records the controls the frames change.
const Builder = struct {
    gpa: Allocator,
    rng: *hash.Rng,
    targets: *Targets,
    inspector: *const ui_theme.Theme,

    /// Whether the next draw from the generator lands on one in `period`.
    fn pick(self: *Builder, period: u32) bool {
        return self.rng.next() % period == 0;
    }

    /// A decimal text such as `12.50` from the next draw of the generator, written into `buffer`.
    fn decimalText(self: *Builder, buffer: *[24]u8) []const u8 {
        const value = self.rng.next() % 100000;
        return std.fmt.bufPrint(buffer, "{d}.{d}", .{ value % 1000, value / 1000 }) catch unreachable;
    }

    /// Adds a label to `parent`, maybe recording it as a target.
    fn label(self: *Builder, parent: *Control, text: []const u8) Error!*Label {
        const made = try Label.create(parent, text);
        if (self.pick(label_period)) try self.targets.labels.append(self.gpa, made);
        if (self.pick(resizable_period)) try self.targets.resizable.append(self.gpa, &made.control);
        return made;
    }

    /// Adds a button to `parent`, maybe recording it as a target.
    fn button(self: *Builder, parent: *Control, text: []const u8) Error!*Button {
        const made = try Button.create(parent, text);
        if (self.pick(button_period)) try self.targets.buttons.append(self.gpa, made);
        if (self.pick(11)) made.control.hovered = true;
        if (self.pick(23)) made.base.setDisabled(true);
        return made;
    }

    /// Adds a text field to `parent`, maybe recording it as a target.
    fn lineEdit(self: *Builder, parent: *Control, text: []const u8) Error!*LineEdit {
        const made = try LineEdit.create(parent);
        try made.setText(text);
        made.control.setHSizeFlags(.expand_fill);
        if (self.pick(edit_period)) try self.targets.edits.append(self.gpa, made);
        if (self.pick(17)) made.editable = false;
        return made;
    }

    /// Adds a slider to `parent` with a random value, maybe recording it as a target.
    fn slider(self: *Builder, parent: *Control) Error!*Slider {
        const made = try Slider.create(parent, false);
        const fraction = self.rng.unit();
        made.range.setRange(0.0, 100.0, fraction * 100.0);
        made.control.setHSizeFlags(.expand_fill);
        if (self.pick(slider_period)) try self.targets.sliders.append(self.gpa, made);
        if (self.pick(resizable_period)) try self.targets.resizable.append(self.gpa, &made.control);
        return made;
    }

    /// Adds a check box to `parent` with a random state.
    fn checkBox(self: *Builder, parent: *Control, text: []const u8) Error!*CheckBox {
        const made = try CheckBox.create(parent, text);
        made.base.pressed = self.pick(2);
        return made;
    }

    /// Adds the toolbar to `screen`.
    fn toolbar(self: *Builder, screen: *Control) Error!void {
        var buffer: [48]u8 = undefined;
        const bar = try PanelContainer.create(screen);
        const margin = try MarginContainer.create(&bar.control);
        try margin.control.addConstantOverride("margin_left", 6.0);
        try margin.control.addConstantOverride("margin_top", 4.0);
        try margin.control.addConstantOverride("margin_right", 6.0);
        try margin.control.addConstantOverride("margin_bottom", 4.0);
        const row = try BoxContainer.createHorizontal(&margin.control);
        for (0..toolbar_buttons) |index| {
            const tool = try self.button(&row.control, named(&tool_names, index));
            tool.setIcon(1 + index % 3, .{ .x = 16.0, .y = 16.0 });
            if (index % 7 == 6) _ = try self.label(&row.control, "|");
        }
        for (0..6) |index| _ = try self.checkBox(&row.control, numbered(&buffer, &tool_names, index + 8, @intCast(index)));
        for (0..4) |_| {
            const zoom = try self.slider(&row.control);
            zoom.control.setCustomMinimumSize(.{ .x = 120.0, .y = 0.0 });
        }
    }

    /// Adds the status bar to `screen`.
    fn statusBar(self: *Builder, screen: *Control) Error!void {
        var buffer: [48]u8 = undefined;
        const row = try BoxContainer.createHorizontal(screen);
        for (0..8) |index| _ = try self.label(&row.control, numbered(&buffer, &entity_names, index, @intCast(index * 13)));
        const fill = try self.label(&row.control, "Ready");
        fill.control.setHSizeFlags(.expand_fill);
        fill.horizontal = .end;
    }

    /// Adds the entity hierarchy to `body`.
    fn hierarchy(self: *Builder, body: *Control) Error!void {
        var buffer: [48]u8 = undefined;
        const panel = try PanelContainer.create(body);
        panel.control.setCustomMinimumSize(.{ .x = 300.0, .y = 0.0 });
        const margin = try MarginContainer.create(&panel.control);
        try margin.control.addConstantOverride("margin_left", 4.0);
        try margin.control.addConstantOverride("margin_right", 4.0);
        const rows = try BoxContainer.createVertical(&margin.control);
        try rows.control.addConstantOverride("separation", 2.0);
        for (0..hierarchy_rows) |index| {
            const row = try BoxContainer.createHorizontal(&rows.control);
            if (index % 2 == 1) row.control.modulate = Color4.init(0.9, 0.9, 0.95, 1.0);
            const depth = (index / 3 + index / 11) % 5;
            const indent = try MarginContainer.create(&row.control);
            try indent.control.addConstantOverride("margin_left", 14.0 * @as(f32, @floatFromInt(depth)));
            const arrow = try self.button(&indent.control, if (index % 4 == 0) ">" else "v");
            arrow.horizontal = .begin;
            const entity = try self.label(&row.control, numbered(&buffer, &entity_names, index, @intCast(index)));
            entity.control.setHSizeFlags(.expand_fill);
            _ = try self.checkBox(&row.control, "");
            _ = try self.button(&row.control, "..");
        }
    }

    /// Adds the asset grid to `tabs`.
    fn assets(self: *Builder, tabs: *TabContainer) Error!void {
        var buffer: [48]u8 = undefined;
        const grid = try GridContainer.create(&tabs.control);
        grid.setColumns(8);
        for (0..asset_cards) |index| {
            const card = try PanelContainer.create(&grid.control);
            const column = try BoxContainer.createVertical(&card.control);
            const centre = try CenterContainer.create(&column.control);
            const thumb = try self.button(&centre.control, named(&tool_names, index));
            thumb.setIcon(1 + index % 3, .{ .x = 32.0, .y = 32.0 });
            _ = try self.label(&column.control, numbered(&buffer, &entity_names, index, @intCast(index)));
        }
    }

    /// Adds the log, profiler and settings tabs to `tabs`.
    fn minorTabs(self: *Builder, tabs: *TabContainer) Error!void {
        var buffer: [48]u8 = undefined;
        var text: [24]u8 = undefined;
        const log = try BoxContainer.createVertical(&tabs.control);
        for (0..log_lines) |index| _ = try self.label(&log.control, numbered(&buffer, &property_names, index, @intCast(index)));

        const profiler = try BoxContainer.createVertical(&tabs.control);
        for (0..10) |index| {
            const row = try BoxContainer.createHorizontal(&profiler.control);
            _ = try self.label(&row.control, named(&section_names, index));
            _ = try self.slider(&row.control);
        }

        const settings = try GridContainer.create(&tabs.control);
        settings.setColumns(2);
        for (0..20) |index| {
            _ = try self.checkBox(&settings.control, named(&property_names, index));
            _ = try self.lineEdit(&settings.control, self.decimalText(&text));
        }
    }

    /// Adds the tab container to `body`.
    fn addTabs(self: *Builder, body: *Control) Error!void {
        const container = try TabContainer.create(body);
        container.control.setHSizeFlags(.expand_fill);
        container.control.setStretchRatio(2.0);
        try self.assets(container);
        try self.minorTabs(container);
        try container.setTabTitle(0, "Assets");
        try container.setTabTitle(1, "Console");
        try container.setTabTitle(2, "Profiler");
        try container.setTabTitle(3, "Settings");
        for (container.control.children.items[1..]) |child| child.setVisible(false);
    }

    /// Adds one property, a label and an editor, to `grid`.
    fn propertyRow(self: *Builder, grid: *GridContainer, section_index: usize, row: usize) Error!void {
        var text: [24]u8 = undefined;
        const property = try self.label(&grid.control, named(&property_names, section_index * 7 + row));
        property.control.setThemeTypeVariation("PropertyLabel");
        switch (row % 6) {
            0 => {
                const edit = try self.lineEdit(&grid.control, self.decimalText(&text));
                try edit.setPlaceholder("empty");
            },
            1 => _ = try self.slider(&grid.control),
            2 => _ = try self.checkBox(&grid.control, "Enabled"),
            3 => {
                const vector = try BoxContainer.createHorizontal(&grid.control);
                for (0..3) |_| _ = try self.lineEdit(&vector.control, self.decimalText(&text));
            },
            4 => {
                const browse = try self.button(&grid.control, "Browse");
                browse.setIcon(1 + row % 3, .{ .x = 16.0, .y = 16.0 });
            },
            else => {
                const pair = try BoxContainer.createHorizontal(&grid.control);
                _ = try self.slider(&pair.control);
                _ = try self.lineEdit(&pair.control, self.decimalText(&text));
            },
        }
    }

    /// Adds inspector section `index` to `list`.
    fn section(self: *Builder, list: *Control, index: usize) Error!void {
        var buffer: [48]u8 = undefined;
        const header = try self.button(list, numbered(&buffer, &section_names, index, @intCast(index)));
        header.control.setThemeTypeVariation("SectionHeader");
        header.horizontal = .begin;
        const body = try MarginContainer.create(list);
        try body.control.addConstantOverride("margin_left", 10.0);
        try body.control.addConstantOverride("margin_bottom", 6.0);
        body.control.clip_contents = true;
        const grid = try GridContainer.create(&body.control);
        grid.setColumns(2);
        for (0..section_rows) |row| try self.propertyRow(grid, index, row);
    }

    /// Adds the inspector to `body`.
    fn buildInspector(self: *Builder, body: *Control) Error!void {
        const panel = try PanelContainer.create(body);
        panel.control.setTheme(self.inspector);
        panel.control.setCustomMinimumSize(.{ .x = 420.0, .y = 0.0 });
        const margin = try MarginContainer.create(&panel.control);
        try margin.control.addConstantOverride("margin_left", 6.0);
        try margin.control.addConstantOverride("margin_top", 6.0);
        try margin.control.addConstantOverride("margin_right", 6.0);
        try margin.control.addConstantOverride("margin_bottom", 6.0);
        const list = try BoxContainer.createVertical(&margin.control);
        for (0..inspector_sections) |index| try self.section(&list.control, index);
    }

    /// Fills `root` with the backdrop, toolbar, body and status bar.
    fn build(self: *Builder, root: *Control) Error!void {
        const backdrop = try Panel.create(root);
        try backdrop.control.setAnchorsAndOffsetsPreset(.full_rect, 0.0);
        const screen = try BoxContainer.createVertical(root);
        try screen.control.setAnchorsAndOffsetsPreset(.full_rect, 0.0);
        try self.toolbar(&screen.control);
        const body = try BoxContainer.createHorizontal(&screen.control);
        body.control.setVSizeFlags(.expand_fill);
        try self.hierarchy(&body.control);
        try self.addTabs(&body.control);
        try self.buildInspector(&body.control);
        try self.statusBar(&screen.control);
    }
};

/// The viewport rectangle of `frame`.
fn viewportOf(frame: u32) Rect2 {
    const step: f32 = @floatFromInt(frame);
    return .{ .size = .{ .x = 2560.0 + step * 24.0, .y = 65536.0 + step * 128.0 } };
}

/// Folds `value` into the running checksum `h`; cheaper than `hash.add` for bulk data.
fn fold(h: u64, value: u64) u64 {
    const mixed = (h ^ value) *% fold_multiplier;
    return mixed ^ (mixed >> 29);
}

/// Two floats as one 64-bit word.
fn pack(low: f32, high: f32) u64 {
    return @as(u64, hash.f32Bits(low)) | (@as(u64, hash.f32Bits(high)) << 32);
}

/// Two 32-bit words as one 64-bit word.
fn join(low: u32, high: u32) u64 {
    return @as(u64, low) | (@as(u64, high) << 32);
}

/// Folds the position and size of `control` and of every control below it.
fn foldTree(control: *const Control, h0: u64) u64 {
    var h = fold(h0, pack(control.position.x, control.position.y));
    h = fold(h, pack(control.size.x, control.size.y));
    for (control.children.items) |child| h = foldTree(child, h);
    return h;
}

/// Folds every vertex, index and command of `data`.
fn foldDraw(data: *const DrawData, h0: u64) u64 {
    var h = h0;
    for (data.vertices.items) |vertex| {
        h = fold(h, pack(vertex.position.x, vertex.position.y));
        h = fold(h, pack(vertex.uv.x, vertex.uv.y));
        h = fold(h, pack(vertex.rect_center.x, vertex.rect_center.y));
        h = fold(h, pack(vertex.rect_half.x, vertex.rect_half.y));
        h = fold(h, join(vertex.color, hash.f32Bits(vertex.radius)));
        h = fold(h, join(hash.f32Bits(vertex.stroke), vertex.mode));
    }
    var index: usize = 0;
    while (index + 1 < data.indices.items.len) : (index += 2) {
        h = fold(h, join(data.indices.items[index], data.indices.items[index + 1]));
    }
    for (data.commands.items) |command| {
        h = fold(h, join(command.first_index, command.index_count));
        h = fold(h, pack(command.clip.position.x, command.clip.position.y));
        h = fold(h, pack(command.clip.size.x, command.clip.size.y));
        h = fold(h, command.texture);
    }
    return h;
}

gpa: Allocator,
font: *ui_draw.Font,
canvas: *ui_draw.DrawCanvas,
context: *Context,
inspector: *ui_theme.Theme,
targets: Targets,

/// Builds the screen and lays out and draws its first frame.
pub fn init(gpa: Allocator) !Ui {
    const font = try gpa.create(ui_draw.Font);
    errdefer gpa.destroy(font);
    font.* = .{};
    const canvas = try ui_draw.DrawCanvas.create(gpa, font);
    errdefer canvas.destroy();
    const context = try Context.create(gpa, &canvas.canvas);
    errdefer context.destroy();
    const inspector = try ui_theme.inspectorTheme(gpa);
    errdefer inspector.destroy();

    var self: Ui = .{
        .gpa = gpa,
        .font = font,
        .canvas = canvas,
        .context = context,
        .inspector = inspector,
        .targets = .{},
    };
    errdefer self.targets.deinit(gpa);

    var rng: hash.Rng = .{ .s = 0x1b0 };
    var builder: Builder = .{ .gpa = gpa, .rng = &rng, .targets = &self.targets, .inspector = inspector };
    try builder.build(context.root);
    if (self.targets.edits.items.len > 0) context.setFocus(&self.targets.edits.items[0].control);
    context.setViewportRect(viewportOf(0));
    try context.update();
    canvas.begin(context.viewport_rect);
    try context.draw();
    return self;
}

/// Frees the tree, the themes, the canvas and the target lists.
pub fn deinit(self: *Ui, gpa: Allocator) void {
    self.targets.deinit(gpa);
    self.context.destroy();
    self.inspector.destroy();
    self.canvas.destroy();
    gpa.destroy(self.font);
}

/// Changes the text, minimum sizes and slider values of the targets for `frame`.
fn mutate(self: *Ui, frame: u32) Error!void {
    const targets = &self.targets;
    const step: usize = frame;
    for (targets.labels.items, 0..) |target, k| try target.setText(text_pool[(step * 5 + k * 3) % text_pool.len]);
    for (targets.buttons.items, 0..) |target, k| try target.setText(text_pool[(step * 3 + k * 7) % text_pool.len]);
    for (targets.edits.items, 0..) |target, k| try target.setText(text_pool[(step * 11 + k * 5) % text_pool.len]);
    for (targets.resizable.items, 0..) |target, k| {
        const width = 90.0 + 20.0 * @as(f32, @floatFromInt((step + k) % 4));
        target.setCustomMinimumSize(.{ .x = width, .y = 0.0 });
    }
    for (targets.sliders.items, 0..) |target, k| target.range.setValue(@floatFromInt((step * 37 + k * 11) % 100));
}

/// One pass over every frame: mutate, lay out, draw and fold into the checksum.
fn runFrames(self: *Ui) Error!u64 {
    var h: u64 = 0;
    for (0..frames) |frame| {
        const index: u32 = @intCast(frame);
        try self.mutate(index);
        self.context.setViewportRect(viewportOf(index));
        try self.context.update();
        self.canvas.begin(self.context.viewport_rect);
        try self.context.draw();
        var frame_hash = foldTree(self.context.root, 0);
        frame_hash = foldDraw(&self.canvas.data, frame_hash);
        h = hash.add(h, frame_hash);
    }
    return h;
}

/// Runs the frames and returns the checksum of every control rect and all draw data.
pub fn run(self: *Ui) u64 {
    return self.runFrames() catch @panic("out of memory");
}
