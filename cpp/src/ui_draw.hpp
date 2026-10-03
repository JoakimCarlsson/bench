#pragma once

#include <cstdint>
#include <string_view>
#include <vector>

#include "ui_math.hpp"

namespace bench::ui {

/// Identifier of a texture; zero is the font atlas, which also holds the white pixel.
using TextureId = uint64_t;

inline constexpr TextureId no_texture = 0;

/// Outline parameters of `Canvas::stroke_rect`.
struct Stroke {
    float width{1.0f};
    float corner_radius{};
};

/// Vertical metrics of the font at one size.
struct FontFace {
    float ascent{};
    float descent{};
    float line_height{};
};

/// One glyph of the fake atlas: its UV rectangle, offset from the pen, size and advance.
struct Glyph {
    Rect2 uv{};
    Vec2 offset{};
    Vec2 size{};
    float advance{};
};

/// Deterministic stand-in for the engine's TrueType atlas: the advance and
/// height of a glyph come from its character class, and the UVs from a
/// 16 by 8 grid of 32 pixel cells in a 512 pixel atlas.
class Font {
public:
    /// Ascent, descent and line height at `size`.
    FontFace face(float size) const;
    /// The glyph for `code` at `size`.
    Glyph glyph(unsigned char code, float size) const;
    /// The pen advance of `code` at `size`.
    float advance(unsigned char code, float size) const;
    /// Width of `text` by walking its glyphs, and the line height.
    Vec2 measure(std::string_view text, float size) const;
    /// The solid-fill block of the atlas.
    Rect2 white_pixel() const;
};

/// Drawing surface the controls paint into.
class Canvas {
public:
    Canvas() = default;
    virtual ~Canvas() = default;

    Canvas(const Canvas&) = delete;
    Canvas& operator=(const Canvas&) = delete;

    /// Shifts the origin by `delta` until the matching `pop_offset`.
    void push_offset(Vec2 delta);
    /// Restores the origin before the last `push_offset`.
    void pop_offset();
    /// The accumulated origin.
    Vec2 offset() const noexcept;

    /// Narrows the clip rectangle to its intersection with `rect`.
    virtual void push_clip(const Rect2& rect) = 0;
    /// Restores the previous clip rectangle.
    virtual void pop_clip() = 0;
    /// Fills `rect`, rounded when `corner_radius` is positive.
    virtual void fill_rect(const Rect2& rect, Color4 color, float corner_radius = 0.0f) = 0;
    /// Outlines `rect`.
    virtual void stroke_rect(const Rect2& rect, Color4 color, Stroke stroke) = 0;
    /// Draws `texture` over `rect` using `uv`, tinted by `modulate`.
    virtual void fill_texture_rect(const Rect2& rect, TextureId texture, const Rect2& uv, Color4 modulate) = 0;
    /// Draws `text` with its top-left at `position`, one quad per glyph.
    virtual void draw_text(Vec2 position, std::string_view text, float font_size, Color4 color) = 0;
    /// Size of `text` at `font_size`.
    virtual Vec2 measure_text(std::string_view text, float font_size) const = 0;
    /// Vertical metrics at `font_size`.
    virtual FontFace metrics(float font_size) const = 0;

private:
    std::vector<Vec2> offsets_;
    Vec2 offset_{};
};

/// How the UI shader interprets a vertex.
enum class DrawMode : uint8_t {
    Textured = 0,
    RoundedFill = 1,
    RoundedStroke = 2,
    Glyph = 3,
};

/// One UI vertex: position, atlas UV, the SDF rectangle and a packed RGBA colour.
struct DrawVertex {
    Vec2 position{};
    Vec2 uv{};
    Vec2 rect_center{};
    Vec2 rect_half{};
    uint32_t color{};
    float radius{};
    float stroke{};
    uint32_t mode{};
};

/// A run of indices drawn with one texture under one clip rectangle.
struct DrawCommand {
    uint32_t first_index{};
    uint32_t index_count{};
    Rect2 clip{};
    TextureId texture{no_texture};
};

/// The vertices, indices and commands a `DrawCanvas` records for one frame.
struct DrawData {
    std::vector<DrawVertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<DrawCommand> commands;

    /// Empties the three arrays, keeping their capacity.
    void clear();
};

/// Parameters of one axis-aligned quad.
struct QuadDesc {
    Rect2 rect{};
    Rect2 uv{};
    Color4 color{};
    DrawMode mode{DrawMode::Textured};
    float radius{};
    float stroke{};
};

/// Canvas that records `DrawData`, batching quads by texture and clip rectangle.
class DrawCanvas final : public Canvas {
public:
    /// A canvas that measures and rasterises text through `font`, which must outlive it.
    explicit DrawCanvas(const Font& font);

    /// Discards the previous frame and starts recording against `viewport`.
    void begin(const Rect2& viewport);
    /// The data recorded since the last `begin`.
    const DrawData& data() const noexcept;

    void push_clip(const Rect2& rect) override;
    void pop_clip() override;
    void fill_rect(const Rect2& rect, Color4 color, float corner_radius) override;
    void stroke_rect(const Rect2& rect, Color4 color, Stroke stroke) override;
    void fill_texture_rect(const Rect2& rect, TextureId texture, const Rect2& uv, Color4 modulate) override;
    void draw_text(Vec2 position, std::string_view text, float font_size, Color4 color) override;
    Vec2 measure_text(std::string_view text, float font_size) const override;
    FontFace metrics(float font_size) const override;

private:
    /// Makes the last command match the current clip and `texture`, starting a new one when it does not.
    void use(TextureId texture);
    /// Appends the quad `desc`, skipping it when empty, transparent or fully clipped.
    void quad(const QuadDesc& desc);
    /// The innermost clip rectangle, or the viewport when none is pushed.
    Rect2 clip() const noexcept;

    const Font* font_{};
    DrawData data_;
    std::vector<Rect2> clips_;
    Rect2 viewport_{};
    TextureId texture_{no_texture};
};

} // namespace bench::ui
