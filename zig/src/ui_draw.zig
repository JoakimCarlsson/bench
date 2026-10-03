//! What the UI paints with: a deterministic stand-in for the font atlas, the
//! `Canvas` interface the controls draw into, and the `DrawCanvas` that
//! records batched vertices, indices and commands.
const std = @import("std");
const vm = @import("vecmath.zig");
const math = @import("ui_math.zig");

const Allocator = std.mem.Allocator;
const Vec2 = math.Vec2;
const Rect2 = math.Rect2;
const Color4 = math.Color4;

/// The only error drawing and layout can raise.
pub const Error = Allocator.Error;

/// Identifier of a texture; zero is the font atlas, which also holds the white pixel.
pub const TextureId = u64;

pub const no_texture: TextureId = 0;

const sdf_bleed: f32 = 1.5;
const atlas_extent: f32 = 512.0;

/// Outline parameters of `Canvas.strokeRect`.
pub const Stroke = struct {
    width: f32 = 1.0,
    corner_radius: f32 = 0.0,
};

/// Vertical metrics of the font at one size.
pub const FontFace = struct {
    ascent: f32 = 0.0,
    descent: f32 = 0.0,
    line_height: f32 = 0.0,
};

/// One glyph of the fake atlas: its UV rectangle, offset from the pen, size and advance.
pub const Glyph = struct {
    uv: Rect2 = .{},
    offset: Vec2 = .{},
    size: Vec2 = .{},
    advance: f32 = 0.0,
};

/// Advance and height of a glyph as fractions of the font size.
const GlyphClass = struct {
    advance: f32,
    height: f32,
};

const glyph_classes = [7]GlyphClass{
    .{ .advance = 0.3, .height = 0.0 },
    .{ .advance = 0.28, .height = 0.7 },
    .{ .advance = 0.55, .height = 0.7 },
    .{ .advance = 0.52, .height = 0.55 },
    .{ .advance = 0.64, .height = 0.72 },
    .{ .advance = 0.85, .height = 0.72 },
    .{ .advance = 0.5, .height = 0.7 },
};

/// The index into `glyph_classes` of the character class of `code`.
fn classify(code: u8) usize {
    switch (code) {
        ' ' => return 0,
        'i', 'l', 'j', '.', ',', ':', ';', '!', '\'', '|', '(', ')', '[', ']' => return 1,
        'm', 'w', 'M', 'W', '@', '%' => return 5,
        else => {},
    }
    if (code >= '0' and code <= '9') return 2;
    if (code >= 'a' and code <= 'z') return 3;
    if (code >= 'A' and code <= 'Z') return 4;
    return 6;
}

/// One colour channel in [0, 1] as 8 bits, rounded half up.
fn channel(value: f32) u32 {
    return @intFromFloat(@floor(vm.clampf(value, 0.0, 1.0) * 255.0 + 0.5));
}

/// A colour packed into 8-bit RGBA with red in the lowest byte.
fn packed_color(color: Color4) u32 {
    return channel(color.r) | (channel(color.g) << 8) | (channel(color.b) << 16) | (channel(color.a) << 24);
}

/// Deterministic stand-in for the engine's TrueType atlas: the advance and
/// height of a glyph come from its character class, and the UVs from a
/// 16 by 8 grid of 32 pixel cells in a 512 pixel atlas.
pub const Font = struct {
    /// Ascent, descent and line height at `size`.
    pub fn face(_: *const Font, size: f32) FontFace {
        return .{ .ascent = size * 0.75, .descent = size * 0.25, .line_height = size * 1.25 };
    }

    /// The pen advance of `code` at `size`.
    pub fn advance(_: *const Font, code: u8, size: f32) f32 {
        return size * glyph_classes[classify(code)].advance;
    }

    /// The glyph for `code` at `size`.
    pub fn glyph(_: *const Font, code: u8, size: f32) Glyph {
        const entry = glyph_classes[classify(code)];
        var made: Glyph = .{ .advance = size * entry.advance };
        if (entry.height <= 0.0) return made;
        const width = @floor(made.advance * 0.75) + 1.0;
        const height = @floor(size * entry.height) + 1.0;
        made.size = .{ .x = width, .y = height };
        made.offset = .{ .x = @floor(made.advance * 0.125), .y = -height };
        const cell: u32 = code & 127;
        const u = @as(f32, @floatFromInt((cell & 15) * 32)) / atlas_extent;
        const v = @as(f32, @floatFromInt((cell >> 4) * 32)) / atlas_extent;
        made.uv = .{ .position = .{ .x = u, .y = v }, .size = .{ .x = width / atlas_extent, .y = height / atlas_extent } };
        return made;
    }

    /// Width of `text` by walking its glyphs, and the line height.
    pub fn measure(self: *const Font, text: []const u8, size: f32) Vec2 {
        const metrics = self.face(size);
        var width: f32 = 0.0;
        for (text) |code| width += self.advance(code, size);
        return .{ .x = width, .y = metrics.line_height };
    }

    /// The solid-fill block of the atlas.
    pub fn whitePixel(_: *const Font) Rect2 {
        const texel: f32 = 1.0 / atlas_extent;
        return .{ .position = .{ .x = texel, .y = texel }, .size = .{ .x = texel, .y = texel } };
    }
};

/// Drawing surface the controls paint into, polymorphic through its vtable.
/// Implementations embed it as their `canvas` field.
pub const Canvas = struct {
    vtable: *const VTable,
    gpa: Allocator,
    offsets: std.ArrayList(Vec2) = .empty,
    current: Vec2 = .{},

    pub const VTable = struct {
        pushClip: *const fn (*Canvas, Rect2) Error!void,
        popClip: *const fn (*Canvas) void,
        fillRect: *const fn (*Canvas, Rect2, Color4, f32) Error!void,
        strokeRect: *const fn (*Canvas, Rect2, Color4, Stroke) Error!void,
        fillTextureRect: *const fn (*Canvas, Rect2, TextureId, Rect2, Color4) Error!void,
        drawText: *const fn (*Canvas, Vec2, []const u8, f32, Color4) Error!void,
        measureText: *const fn (*const Canvas, []const u8, f32) Vec2,
        metrics: *const fn (*const Canvas, f32) FontFace,
    };

    /// Releases the offset stack.
    pub fn deinit(self: *Canvas) void {
        self.offsets.deinit(self.gpa);
    }

    /// Shifts the origin by `delta` until the matching `popOffset`.
    pub fn pushOffset(self: *Canvas, delta: Vec2) Error!void {
        try self.offsets.append(self.gpa, self.current);
        self.current = self.current.add(delta);
    }

    /// Restores the origin before the last `pushOffset`.
    pub fn popOffset(self: *Canvas) void {
        self.current = self.offsets.pop() orelse .{};
    }

    /// The accumulated origin.
    pub fn offset(self: *const Canvas) Vec2 {
        return self.current;
    }

    /// Narrows the clip rectangle to its intersection with `rect`.
    pub fn pushClip(self: *Canvas, rect: Rect2) Error!void {
        return self.vtable.pushClip(self, rect);
    }

    /// Restores the previous clip rectangle.
    pub fn popClip(self: *Canvas) void {
        self.vtable.popClip(self);
    }

    /// Fills `rect`, rounded when `corner_radius` is positive.
    pub fn fillRect(self: *Canvas, rect: Rect2, color: Color4, corner_radius: f32) Error!void {
        return self.vtable.fillRect(self, rect, color, corner_radius);
    }

    /// Outlines `rect`.
    pub fn strokeRect(self: *Canvas, rect: Rect2, color: Color4, stroke: Stroke) Error!void {
        return self.vtable.strokeRect(self, rect, color, stroke);
    }

    /// Draws `texture` over `rect` using `uv`, tinted by `modulate`.
    pub fn fillTextureRect(self: *Canvas, rect: Rect2, texture: TextureId, uv: Rect2, modulate: Color4) Error!void {
        return self.vtable.fillTextureRect(self, rect, texture, uv, modulate);
    }

    /// Draws `text` with its top-left at `position`, one quad per glyph.
    pub fn drawText(self: *Canvas, position: Vec2, text: []const u8, font_size: f32, color: Color4) Error!void {
        return self.vtable.drawText(self, position, text, font_size, color);
    }

    /// Size of `text` at `font_size`.
    pub fn measureText(self: *const Canvas, text: []const u8, font_size: f32) Vec2 {
        return self.vtable.measureText(self, text, font_size);
    }

    /// Vertical metrics at `font_size`.
    pub fn metrics(self: *const Canvas, font_size: f32) FontFace {
        return self.vtable.metrics(self, font_size);
    }
};

/// How the UI shader interprets a vertex.
pub const DrawMode = enum(u8) {
    textured = 0,
    rounded_fill = 1,
    rounded_stroke = 2,
    glyph = 3,
};

/// One UI vertex: position, atlas UV, the SDF rectangle and a packed RGBA colour.
pub const DrawVertex = struct {
    position: Vec2 = .{},
    uv: Vec2 = .{},
    rect_center: Vec2 = .{},
    rect_half: Vec2 = .{},
    color: u32 = 0,
    radius: f32 = 0.0,
    stroke: f32 = 0.0,
    mode: u32 = 0,
};

/// A run of indices drawn with one texture under one clip rectangle.
pub const DrawCommand = struct {
    first_index: u32 = 0,
    index_count: u32 = 0,
    clip: Rect2 = .{},
    texture: TextureId = no_texture,
};

/// The vertices, indices and commands a `DrawCanvas` records for one frame.
pub const DrawData = struct {
    vertices: std.ArrayList(DrawVertex) = .empty,
    indices: std.ArrayList(u32) = .empty,
    commands: std.ArrayList(DrawCommand) = .empty,

    /// Releases the three arrays.
    pub fn deinit(self: *DrawData, gpa: Allocator) void {
        self.vertices.deinit(gpa);
        self.indices.deinit(gpa);
        self.commands.deinit(gpa);
    }

    /// Empties the three arrays, keeping their capacity.
    pub fn clear(self: *DrawData) void {
        self.vertices.clearRetainingCapacity();
        self.indices.clearRetainingCapacity();
        self.commands.clearRetainingCapacity();
    }
};

/// Parameters of one axis-aligned quad.
const QuadDesc = struct {
    rect: Rect2 = .{},
    uv: Rect2 = .{},
    color: Color4 = .{},
    mode: DrawMode = .textured,
    radius: f32 = 0.0,
    stroke: f32 = 0.0,
};

const quad_indices = [6]u32{ 0, 1, 2, 0, 2, 3 };

/// Canvas that records `DrawData`, batching quads by texture and clip rectangle.
pub const DrawCanvas = struct {
    canvas: Canvas,
    font: *const Font,
    data: DrawData = .{},
    clips: std.ArrayList(Rect2) = .empty,
    viewport: Rect2 = .{},
    texture: TextureId = no_texture,

    const vtable: Canvas.VTable = .{
        .pushClip = pushClip,
        .popClip = popClip,
        .fillRect = fillRect,
        .strokeRect = strokeRect,
        .fillTextureRect = fillTextureRect,
        .drawText = drawText,
        .measureText = measureText,
        .metrics = metrics,
    };

    /// A heap-allocated canvas that measures and rasterises text through `font`, which must outlive it.
    pub fn create(gpa: Allocator, font: *const Font) Allocator.Error!*DrawCanvas {
        const self = try gpa.create(DrawCanvas);
        self.* = .{ .canvas = .{ .vtable = &vtable, .gpa = gpa }, .font = font };
        return self;
    }

    /// Frees the canvas and everything it recorded.
    pub fn destroy(self: *DrawCanvas) void {
        const gpa = self.canvas.gpa;
        self.data.deinit(gpa);
        self.clips.deinit(gpa);
        self.canvas.deinit();
        gpa.destroy(self);
    }

    /// Discards the previous frame and starts recording against `viewport`.
    pub fn begin(self: *DrawCanvas, viewport: Rect2) void {
        self.data.clear();
        self.clips.clearRetainingCapacity();
        self.viewport = viewport;
        self.texture = no_texture;
    }

    /// The innermost clip rectangle, or the viewport when none is pushed.
    fn clip(self: *const DrawCanvas) Rect2 {
        return self.clips.getLastOrNull() orelse self.viewport;
    }

    /// Recovers the `DrawCanvas` that embeds `canvas`.
    fn from(canvas: *Canvas) *DrawCanvas {
        return @alignCast(@fieldParentPtr("canvas", canvas));
    }

    /// Recovers the `DrawCanvas` that embeds `canvas`, read-only.
    fn fromConst(canvas: *const Canvas) *const DrawCanvas {
        return @alignCast(@fieldParentPtr("canvas", canvas));
    }

    /// Narrows the clip rectangle to its intersection with `rect`.
    fn pushClip(canvas: *Canvas, rect: Rect2) Error!void {
        const self = from(canvas);
        try self.clips.append(canvas.gpa, math.intersection(self.clip(), rect));
    }

    /// Restores the previous clip rectangle.
    fn popClip(canvas: *Canvas) void {
        _ = from(canvas).clips.pop();
    }

    /// Makes the last command match the current clip and `texture`, starting a new one when it does not.
    fn use(self: *DrawCanvas, texture: TextureId) Error!void {
        const area = self.clip();
        const commands = &self.data.commands;
        if (commands.items.len > 0) {
            const last = &commands.items[commands.items.len - 1];
            if (last.texture == texture and last.clip.eql(area)) return;
            if (last.index_count == 0) {
                last.texture = texture;
                last.clip = area;
                return;
            }
        }
        try commands.append(self.canvas.gpa, .{
            .first_index = @intCast(self.data.indices.items.len),
            .texture = texture,
            .clip = area,
        });
    }

    /// Appends the quad `desc`, skipping it when empty, transparent or fully clipped.
    fn quad(self: *DrawCanvas, desc: QuadDesc) Error!void {
        if (desc.rect.size.x <= 0.0 or desc.rect.size.y <= 0.0 or desc.color.a <= 0.0) return;
        const area = self.clip();
        const visible = math.intersection(area, desc.rect);
        if (visible.size.x <= 0.0 or visible.size.y <= 0.0) return;
        try self.use(self.texture);

        var target = desc.rect;
        const coords = desc.uv;
        if (desc.mode == .rounded_fill or desc.mode == .rounded_stroke) target = math.grow(desc.rect, sdf_bleed);

        const gpa = self.canvas.gpa;
        const base: u32 = @intCast(self.data.vertices.items.len);
        const tint = packed_color(desc.color);
        const centre = desc.rect.center();
        const half = desc.rect.size.scale(0.5);
        const kind: u32 = @intFromEnum(desc.mode);

        const corners = [4]Vec2{
            target.position,
            .{ .x = target.right(), .y = target.top() },
            target.end(),
            .{ .x = target.left(), .y = target.bottom() },
        };
        const texels = [4]Vec2{
            coords.position,
            .{ .x = coords.right(), .y = coords.top() },
            coords.end(),
            .{ .x = coords.left(), .y = coords.bottom() },
        };
        for (corners, texels) |corner, texel| {
            try self.data.vertices.append(gpa, .{
                .position = corner,
                .uv = texel,
                .rect_center = centre,
                .rect_half = half,
                .color = tint,
                .radius = desc.radius,
                .stroke = desc.stroke,
                .mode = kind,
            });
        }
        for (quad_indices) |step| try self.data.indices.append(gpa, base + step);
        self.data.commands.items[self.data.commands.items.len - 1].index_count += 6;
    }

    /// Fills `rect`, rounded when `corner_radius` is positive.
    fn fillRect(canvas: *Canvas, rect: Rect2, color: Color4, corner_radius: f32) Error!void {
        const self = from(canvas);
        self.texture = no_texture;
        try self.quad(.{
            .rect = .{ .position = canvas.offset().add(rect.position), .size = rect.size },
            .uv = self.font.whitePixel(),
            .color = color,
            .mode = if (corner_radius <= 0.0) .textured else .rounded_fill,
            .radius = corner_radius,
        });
    }

    /// Outlines `rect`.
    fn strokeRect(canvas: *Canvas, rect: Rect2, color: Color4, stroke: Stroke) Error!void {
        const self = from(canvas);
        self.texture = no_texture;
        try self.quad(.{
            .rect = .{ .position = canvas.offset().add(rect.position), .size = rect.size },
            .uv = self.font.whitePixel(),
            .color = color,
            .mode = .rounded_stroke,
            .radius = stroke.corner_radius,
            .stroke = vm.maxf(stroke.width, 1.0),
        });
    }

    /// Draws `texture` over `rect` using `uv`, tinted by `modulate`.
    fn fillTextureRect(canvas: *Canvas, rect: Rect2, texture: TextureId, uv: Rect2, modulate: Color4) Error!void {
        if (texture == no_texture) return;
        const self = from(canvas);
        self.texture = texture;
        try self.quad(.{
            .rect = .{ .position = canvas.offset().add(rect.position), .size = rect.size },
            .uv = uv,
            .color = modulate,
        });
        self.texture = no_texture;
    }

    /// Draws `text` with its top-left at `position`, one quad per glyph.
    fn drawText(canvas: *Canvas, position: Vec2, text: []const u8, font_size: f32, color: Color4) Error!void {
        if (text.len == 0 or color.a <= 0.0) return;
        const self = from(canvas);
        const face = self.font.face(font_size);
        var pen = canvas.offset().add(position);
        pen.y += face.ascent;

        self.texture = no_texture;
        for (text) |code| {
            const entry = self.font.glyph(code, font_size);
            if (entry.size.x > 0.0 and entry.size.y > 0.0) {
                try self.quad(.{
                    .rect = .{ .position = pen.add(entry.offset), .size = entry.size },
                    .uv = entry.uv,
                    .color = color,
                    .mode = .glyph,
                });
            }
            pen.x += entry.advance;
        }
    }

    /// Size of `text` at `font_size`.
    fn measureText(canvas: *const Canvas, text: []const u8, font_size: f32) Vec2 {
        return fromConst(canvas).font.measure(text, font_size);
    }

    /// Vertical metrics at `font_size`.
    fn metrics(canvas: *const Canvas, font_size: f32) FontFace {
        return fromConst(canvas).font.face(font_size);
    }
};
