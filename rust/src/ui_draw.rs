//! The UI's drawing surface: a deterministic fake font standing in for the
//! engine's TrueType atlas, the `Canvas` trait the controls paint into, and
//! `DrawCanvas`, which records quads as batched vertex, index and command
//! arrays.
use crate::ui_math::{self, Color4, Rect2, Vec2};

/// Identifier of a texture; zero is the font atlas, which also holds the white pixel.
pub type TextureId = u64;

pub const NO_TEXTURE: TextureId = 0;

const SDF_BLEED: f32 = 1.5;
const ATLAS_EXTENT: f32 = 512.0;

/// Outline parameters of `Canvas::stroke_rect`.
#[derive(Clone, Copy)]
pub struct Stroke {
    pub width: f32,
    pub corner_radius: f32,
}

/// Vertical metrics of the font at one size.
#[derive(Clone, Copy)]
pub struct FontFace {
    pub ascent: f32,
    #[allow(dead_code)]
    pub descent: f32,
    pub line_height: f32,
}

/// One glyph of the fake atlas: its UV rectangle, offset from the pen, size and advance.
#[derive(Clone, Copy, Default)]
pub struct Glyph {
    pub uv: Rect2,
    pub offset: Vec2,
    pub size: Vec2,
    pub advance: f32,
}

/// Advance and height of a glyph as fractions of the font size.
struct GlyphClass {
    advance: f32,
    height: f32,
}

const GLYPH_CLASSES: [GlyphClass; 7] = [
    GlyphClass { advance: 0.3, height: 0.0 },
    GlyphClass { advance: 0.28, height: 0.7 },
    GlyphClass { advance: 0.55, height: 0.7 },
    GlyphClass { advance: 0.52, height: 0.55 },
    GlyphClass { advance: 0.64, height: 0.72 },
    GlyphClass { advance: 0.85, height: 0.72 },
    GlyphClass { advance: 0.5, height: 0.7 },
];

/// The index into `GLYPH_CLASSES` of the character class of `code`.
fn classify(code: u8) -> usize {
    match code {
        b' ' => 0,
        b'i' | b'l' | b'j' | b'.' | b',' | b':' | b';' | b'!' | b'\'' | b'|' | b'(' | b')' | b'[' | b']' => 1,
        b'm' | b'w' | b'M' | b'W' | b'@' | b'%' => 5,
        b'0'..=b'9' => 2,
        b'a'..=b'z' => 3,
        b'A'..=b'Z' => 4,
        _ => 6,
    }
}

/// One colour channel in [0, 1] as 8 bits, rounded half up.
fn channel(value: f32) -> u32 {
    (ui_math::clamp(value, 0.0, 1.0) * 255.0 + 0.5).floor() as u32
}

/// A colour packed into 8-bit RGBA with red in the lowest byte.
fn packed(color: Color4) -> u32 {
    channel(color.r) | (channel(color.g) << 8) | (channel(color.b) << 16) | (channel(color.a) << 24)
}

/// Deterministic stand-in for the engine's TrueType atlas: the advance and
/// height of a glyph come from its character class, and the UVs from a 16 by
/// 8 grid of 32 pixel cells in a 512 pixel atlas.
#[derive(Clone, Copy, Default)]
pub struct Font;

impl Font {
    /// Ascent, descent and line height at `size`.
    pub fn face(&self, size: f32) -> FontFace {
        FontFace { ascent: size * 0.75, descent: size * 0.25, line_height: size * 1.25 }
    }

    /// The pen advance of `code` at `size`.
    pub fn advance(&self, code: u8, size: f32) -> f32 {
        size * GLYPH_CLASSES[classify(code)].advance
    }

    /// The glyph for `code` at `size`.
    pub fn glyph(&self, code: u8, size: f32) -> Glyph {
        let entry = &GLYPH_CLASSES[classify(code)];
        let mut made = Glyph { advance: size * entry.advance, ..Glyph::default() };
        if entry.height <= 0.0 {
            return made;
        }
        let width = (made.advance * 0.75).floor() + 1.0;
        let height = (size * entry.height).floor() + 1.0;
        made.size = Vec2::new(width, height);
        made.offset = Vec2::new((made.advance * 0.125).floor(), -height);
        let cell = (code & 127) as u32;
        let u = ((cell & 15) * 32) as f32 / ATLAS_EXTENT;
        let v = ((cell >> 4) * 32) as f32 / ATLAS_EXTENT;
        made.uv = Rect2::new(Vec2::new(u, v), Vec2::new(width / ATLAS_EXTENT, height / ATLAS_EXTENT));
        made
    }

    /// Width of `text` by walking its glyphs, and the line height.
    pub fn measure(&self, text: &str, size: f32) -> Vec2 {
        let metrics = self.face(size);
        let mut width = 0.0;
        for code in text.bytes() {
            width += self.advance(code, size);
        }
        Vec2::new(width, metrics.line_height)
    }

    /// The solid-fill block of the atlas.
    pub fn white_pixel(&self) -> Rect2 {
        let texel = 1.0 / ATLAS_EXTENT;
        Rect2::new(Vec2::new(texel, texel), Vec2::new(texel, texel))
    }
}

/// The stack of origins `push_offset` and `pop_offset` maintain.
#[derive(Default)]
pub struct Offsets {
    saved: Vec<Vec2>,
    current: Vec2,
}

impl Offsets {
    /// Shifts the origin by `delta`.
    fn push(&mut self, delta: Vec2) {
        self.saved.push(self.current);
        self.current = self.current + delta;
    }

    /// Restores the origin before the last push.
    fn pop(&mut self) {
        match self.saved.pop() {
            Some(previous) => self.current = previous,
            None => self.current = Vec2::default(),
        }
    }
}

/// Drawing surface the controls paint into.
pub trait Canvas {
    /// Shifts the origin by `delta` until the matching `pop_offset`.
    fn push_offset(&mut self, delta: Vec2);
    /// Restores the origin before the last `push_offset`.
    fn pop_offset(&mut self);
    /// The accumulated origin.
    fn offset(&self) -> Vec2;
    /// Narrows the clip rectangle to its intersection with `rect`.
    fn push_clip(&mut self, rect: &Rect2);
    /// Restores the previous clip rectangle.
    fn pop_clip(&mut self);
    /// Fills `rect`, rounded when `corner_radius` is positive.
    fn fill_rect(&mut self, rect: &Rect2, color: Color4, corner_radius: f32);
    /// Outlines `rect`.
    fn stroke_rect(&mut self, rect: &Rect2, color: Color4, stroke: Stroke);
    /// Draws `texture` over `rect` using `uv`, tinted by `modulate`.
    fn fill_texture_rect(&mut self, rect: &Rect2, texture: TextureId, uv: &Rect2, modulate: Color4);
    /// Draws `text` with its top-left at `position`, one quad per glyph.
    fn draw_text(&mut self, position: Vec2, text: &str, font_size: f32, color: Color4);
    /// Size of `text` at `font_size`.
    fn measure_text(&self, text: &str, font_size: f32) -> Vec2;
    /// Vertical metrics at `font_size`.
    fn metrics(&self, font_size: f32) -> FontFace;
}

/// How the UI shader interprets a vertex.
#[derive(Clone, Copy, PartialEq)]
pub enum DrawMode {
    Textured = 0,
    RoundedFill = 1,
    RoundedStroke = 2,
    Glyph = 3,
}

/// One UI vertex: position, atlas UV, the SDF rectangle and a packed RGBA colour.
#[derive(Clone, Copy, Default)]
pub struct DrawVertex {
    pub position: Vec2,
    pub uv: Vec2,
    pub rect_center: Vec2,
    pub rect_half: Vec2,
    pub color: u32,
    pub radius: f32,
    pub stroke: f32,
    pub mode: u32,
}

/// A run of indices drawn with one texture under one clip rectangle.
#[derive(Clone, Copy, Default)]
pub struct DrawCommand {
    pub first_index: u32,
    pub index_count: u32,
    pub clip: Rect2,
    pub texture: TextureId,
}

/// The vertices, indices and commands a `DrawCanvas` records for one frame.
#[derive(Default)]
pub struct DrawData {
    pub vertices: Vec<DrawVertex>,
    pub indices: Vec<u32>,
    pub commands: Vec<DrawCommand>,
}

impl DrawData {
    /// Empties the three arrays, keeping their capacity.
    fn clear(&mut self) {
        self.vertices.clear();
        self.indices.clear();
        self.commands.clear();
    }
}

/// Parameters of one axis-aligned quad.
struct QuadDesc {
    rect: Rect2,
    uv: Rect2,
    color: Color4,
    mode: DrawMode,
    radius: f32,
    stroke: f32,
}

/// Canvas that records `DrawData`, batching quads by texture and clip rectangle.
pub struct DrawCanvas {
    font: Font,
    data: DrawData,
    clips: Vec<Rect2>,
    viewport: Rect2,
    texture: TextureId,
    offsets: Offsets,
}

impl DrawCanvas {
    /// A canvas that measures and rasterises text through `font`.
    pub fn new(font: Font) -> DrawCanvas {
        DrawCanvas { font, data: DrawData::default(), clips: Vec::new(), viewport: Rect2::default(), texture: NO_TEXTURE, offsets: Offsets::default() }
    }

    /// Discards the previous frame and starts recording against `viewport`.
    pub fn begin(&mut self, viewport: &Rect2) {
        self.data.clear();
        self.clips.clear();
        self.viewport = *viewport;
        self.texture = NO_TEXTURE;
    }

    /// The data recorded since the last `begin`.
    pub fn data(&self) -> &DrawData {
        &self.data
    }

    /// The innermost clip rectangle, or the viewport when none is pushed.
    fn clip(&self) -> Rect2 {
        match self.clips.last() {
            Some(rect) => *rect,
            None => self.viewport,
        }
    }

    /// Makes the last command match the current clip and `texture`, starting a new one when it does not.
    fn use_texture(&mut self, texture: TextureId) {
        let area = self.clip();
        if let Some(last) = self.data.commands.last_mut() {
            if last.texture == texture && last.clip == area {
                return;
            }
            if last.index_count == 0 {
                last.texture = texture;
                last.clip = area;
                return;
            }
        }
        self.data.commands.push(DrawCommand { first_index: self.data.indices.len() as u32, index_count: 0, clip: area, texture });
    }

    /// Appends the quad `desc`, skipping it when empty, transparent or fully clipped.
    fn quad(&mut self, desc: &QuadDesc) {
        if desc.rect.size.x <= 0.0 || desc.rect.size.y <= 0.0 || desc.color.a <= 0.0 {
            return;
        }
        let area = self.clip();
        let visible = ui_math::intersection(&area, &desc.rect);
        if visible.size.x <= 0.0 || visible.size.y <= 0.0 {
            return;
        }
        self.use_texture(self.texture);

        let mut target = desc.rect;
        let coords = desc.uv;
        if desc.mode == DrawMode::RoundedFill || desc.mode == DrawMode::RoundedStroke {
            target = ui_math::grow(&desc.rect, SDF_BLEED);
        }

        let base = self.data.vertices.len() as u32;
        let tint = packed(desc.color);
        let centre = desc.rect.center();
        let half = desc.rect.size * 0.5;
        let kind = desc.mode as u32;

        let corners = [target.position, Vec2::new(target.right(), target.top()), target.end(), Vec2::new(target.left(), target.bottom())];
        let texels = [coords.position, Vec2::new(coords.right(), coords.top()), coords.end(), Vec2::new(coords.left(), coords.bottom())];
        for index in 0..4 {
            self.data.vertices.push(DrawVertex {
                position: corners[index],
                uv: texels[index],
                rect_center: centre,
                rect_half: half,
                color: tint,
                radius: desc.radius,
                stroke: desc.stroke,
                mode: kind,
            });
        }
        for offset in [0u32, 1, 2, 0, 2, 3] {
            self.data.indices.push(base + offset);
        }
        if let Some(command) = self.data.commands.last_mut() {
            command.index_count += 6;
        }
    }
}

impl Canvas for DrawCanvas {
    fn push_offset(&mut self, delta: Vec2) {
        self.offsets.push(delta);
    }

    fn pop_offset(&mut self) {
        self.offsets.pop();
    }

    fn offset(&self) -> Vec2 {
        self.offsets.current
    }

    fn push_clip(&mut self, rect: &Rect2) {
        let clipped = ui_math::intersection(&self.clip(), rect);
        self.clips.push(clipped);
    }

    fn pop_clip(&mut self) {
        self.clips.pop();
    }

    fn fill_rect(&mut self, rect: &Rect2, color: Color4, corner_radius: f32) {
        let target = Rect2::new(self.offset() + rect.position, rect.size);
        self.texture = NO_TEXTURE;
        let mode = if corner_radius <= 0.0 { DrawMode::Textured } else { DrawMode::RoundedFill };
        self.quad(&QuadDesc { rect: target, uv: self.font.white_pixel(), color, mode, radius: corner_radius, stroke: 0.0 });
    }

    fn stroke_rect(&mut self, rect: &Rect2, color: Color4, stroke: Stroke) {
        let target = Rect2::new(self.offset() + rect.position, rect.size);
        self.texture = NO_TEXTURE;
        self.quad(&QuadDesc {
            rect: target,
            uv: self.font.white_pixel(),
            color,
            mode: DrawMode::RoundedStroke,
            radius: stroke.corner_radius,
            stroke: ui_math::max(stroke.width, 1.0),
        });
    }

    fn fill_texture_rect(&mut self, rect: &Rect2, texture: TextureId, uv: &Rect2, modulate: Color4) {
        if texture == NO_TEXTURE {
            return;
        }
        self.texture = texture;
        let target = Rect2::new(self.offset() + rect.position, rect.size);
        self.quad(&QuadDesc { rect: target, uv: *uv, color: modulate, mode: DrawMode::Textured, radius: 0.0, stroke: 0.0 });
        self.texture = NO_TEXTURE;
    }

    fn draw_text(&mut self, position: Vec2, text: &str, font_size: f32, color: Color4) {
        if text.is_empty() || color.a <= 0.0 {
            return;
        }
        let metrics = self.font.face(font_size);
        let mut pen = self.offset() + position;
        pen.y += metrics.ascent;

        self.texture = NO_TEXTURE;
        for code in text.bytes() {
            let entry = self.font.glyph(code, font_size);
            if entry.size.x > 0.0 && entry.size.y > 0.0 {
                self.quad(&QuadDesc {
                    rect: Rect2::new(pen + entry.offset, entry.size),
                    uv: entry.uv,
                    color,
                    mode: DrawMode::Glyph,
                    radius: 0.0,
                    stroke: 0.0,
                });
            }
            pen.x += entry.advance;
        }
    }

    fn measure_text(&self, text: &str, font_size: f32) -> Vec2 {
        self.font.measure(text, font_size)
    }

    fn metrics(&self, font_size: f32) -> FontFace {
        self.font.face(font_size)
    }
}
