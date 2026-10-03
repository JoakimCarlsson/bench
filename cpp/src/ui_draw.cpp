#include "ui_draw.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace bench::ui {

namespace {

constexpr float sdf_bleed = 1.5f;
constexpr float atlas_extent = 512.0f;

/// Advance and height of a glyph as fractions of the font size.
struct GlyphClass {
    float advance;
    float height;
};

constexpr std::array<GlyphClass, 7> glyph_classes{{
    {0.3f, 0.0f},
    {0.28f, 0.7f},
    {0.55f, 0.7f},
    {0.52f, 0.55f},
    {0.64f, 0.72f},
    {0.85f, 0.72f},
    {0.5f, 0.7f},
}};

/// The index into `glyph_classes` of the character class of `code`.
constexpr size_t classify(unsigned char code) {
    switch (code) {
    case ' ':
        return 0;
    case 'i':
    case 'l':
    case 'j':
    case '.':
    case ',':
    case ':':
    case ';':
    case '!':
    case '\'':
    case '|':
    case '(':
    case ')':
    case '[':
    case ']':
        return 1;
    case 'm':
    case 'w':
    case 'M':
    case 'W':
    case '@':
    case '%':
        return 5;
    default:
        break;
    }
    if (code >= '0' && code <= '9') return 2;
    if (code >= 'a' && code <= 'z') return 3;
    if (code >= 'A' && code <= 'Z') return 4;
    return 6;
}

/// One colour channel in [0, 1] as 8 bits, rounded half up.
uint32_t channel(float value) {
    return static_cast<uint32_t>(std::floor(std::clamp(value, 0.0f, 1.0f) * 255.0f + 0.5f));
}

/// A colour packed into 8-bit RGBA with red in the lowest byte.
uint32_t packed(Color4 color) {
    return channel(color.r) | (channel(color.g) << 8u) | (channel(color.b) << 16u) | (channel(color.a) << 24u);
}

} // namespace

FontFace Font::face(float size) const {
    return FontFace{size * 0.75f, size * 0.25f, size * 1.25f};
}

float Font::advance(unsigned char code, float size) const {
    return size * glyph_classes[classify(code)].advance;
}

Glyph Font::glyph(unsigned char code, float size) const {
    const GlyphClass& entry = glyph_classes[classify(code)];
    Glyph made;
    made.advance = size * entry.advance;
    if (entry.height <= 0.0f) return made;
    const float width = std::floor(made.advance * 0.75f) + 1.0f;
    const float height = std::floor(size * entry.height) + 1.0f;
    made.size = Vec2{width, height};
    made.offset = Vec2{std::floor(made.advance * 0.125f), -height};
    const uint32_t cell = code & 127u;
    const float u = static_cast<float>((cell & 15u) * 32u) / atlas_extent;
    const float v = static_cast<float>((cell >> 4u) * 32u) / atlas_extent;
    made.uv = Rect2{{u, v}, {width / atlas_extent, height / atlas_extent}};
    return made;
}

Vec2 Font::measure(std::string_view text, float size) const {
    const FontFace metrics = face(size);
    float width = 0.0f;
    for (const char code : text) width += advance(static_cast<unsigned char>(code), size);
    return {width, metrics.line_height};
}

Rect2 Font::white_pixel() const {
    return Rect2{{1.0f / atlas_extent, 1.0f / atlas_extent}, {1.0f / atlas_extent, 1.0f / atlas_extent}};
}

void Canvas::push_offset(Vec2 delta) {
    offsets_.push_back(offset_);
    offset_ = offset_ + delta;
}

void Canvas::pop_offset() {
    if (offsets_.empty()) {
        offset_ = Vec2{};
        return;
    }
    offset_ = offsets_.back();
    offsets_.pop_back();
}

Vec2 Canvas::offset() const noexcept {
    return offset_;
}

void DrawData::clear() {
    vertices.clear();
    indices.clear();
    commands.clear();
}

DrawCanvas::DrawCanvas(const Font& font) : font_{&font} {}

void DrawCanvas::begin(const Rect2& viewport) {
    data_.clear();
    clips_.clear();
    viewport_ = viewport;
    texture_ = no_texture;
}

const DrawData& DrawCanvas::data() const noexcept {
    return data_;
}

Rect2 DrawCanvas::clip() const noexcept {
    return clips_.empty() ? viewport_ : clips_.back();
}

void DrawCanvas::push_clip(const Rect2& rect) {
    clips_.push_back(intersection(clip(), rect));
}

void DrawCanvas::pop_clip() {
    if (!clips_.empty()) clips_.pop_back();
}

void DrawCanvas::use(TextureId texture) {
    const Rect2 area = clip();
    if (!data_.commands.empty()) {
        DrawCommand& last = data_.commands.back();
        if (last.texture == texture && last.clip == area) return;
        if (last.index_count == 0) {
            last.texture = texture;
            last.clip = area;
            return;
        }
    }
    DrawCommand next;
    next.first_index = static_cast<uint32_t>(data_.indices.size());
    next.texture = texture;
    next.clip = area;
    data_.commands.push_back(next);
}

void DrawCanvas::quad(const QuadDesc& desc) {
    if (desc.rect.size.x <= 0.0f || desc.rect.size.y <= 0.0f || desc.color.a <= 0.0f) return;
    const Rect2 area = clip();
    const Rect2 visible = intersection(area, desc.rect);
    if (visible.size.x <= 0.0f || visible.size.y <= 0.0f) return;
    use(texture_);

    Rect2 target = desc.rect;
    const Rect2 coords = desc.uv;
    if (desc.mode == DrawMode::RoundedFill || desc.mode == DrawMode::RoundedStroke) target = grow(desc.rect, sdf_bleed);

    const auto base = static_cast<uint32_t>(data_.vertices.size());
    const uint32_t tint = packed(desc.color);
    const Vec2 centre = desc.rect.center();
    const Vec2 half = desc.rect.size * 0.5f;
    const auto kind = static_cast<uint32_t>(desc.mode);

    const std::array<Vec2, 4> corners{target.position, Vec2{target.right(), target.top()}, target.end(), Vec2{target.left(), target.bottom()}};
    const std::array<Vec2, 4> texels{coords.position, Vec2{coords.right(), coords.top()}, coords.end(), Vec2{coords.left(), coords.bottom()}};
    for (size_t index = 0; index < 4; ++index) {
        DrawVertex vertex;
        vertex.position = corners[index];
        vertex.uv = texels[index];
        vertex.rect_center = centre;
        vertex.rect_half = half;
        vertex.color = tint;
        vertex.radius = desc.radius;
        vertex.stroke = desc.stroke;
        vertex.mode = kind;
        data_.vertices.push_back(vertex);
    }
    for (const uint32_t offset : {0u, 1u, 2u, 0u, 2u, 3u}) data_.indices.push_back(base + offset);
    data_.commands.back().index_count += 6;
}

void DrawCanvas::fill_rect(const Rect2& rect, Color4 color, float corner_radius) {
    const Rect2 target{offset() + rect.position, rect.size};
    texture_ = no_texture;
    QuadDesc desc;
    desc.rect = target;
    desc.uv = font_->white_pixel();
    desc.color = color;
    desc.mode = corner_radius <= 0.0f ? DrawMode::Textured : DrawMode::RoundedFill;
    desc.radius = corner_radius;
    quad(desc);
}

void DrawCanvas::stroke_rect(const Rect2& rect, Color4 color, Stroke stroke) {
    const Rect2 target{offset() + rect.position, rect.size};
    texture_ = no_texture;
    QuadDesc desc;
    desc.rect = target;
    desc.uv = font_->white_pixel();
    desc.color = color;
    desc.mode = DrawMode::RoundedStroke;
    desc.radius = stroke.corner_radius;
    desc.stroke = std::max(stroke.width, 1.0f);
    quad(desc);
}

void DrawCanvas::fill_texture_rect(const Rect2& rect, TextureId texture, const Rect2& uv, Color4 modulate) {
    if (texture == no_texture) return;
    texture_ = texture;
    QuadDesc desc;
    desc.rect = Rect2{offset() + rect.position, rect.size};
    desc.uv = uv;
    desc.color = modulate;
    quad(desc);
    texture_ = no_texture;
}

void DrawCanvas::draw_text(Vec2 position, std::string_view text, float font_size, Color4 color) {
    if (text.empty() || color.a <= 0.0f) return;
    const FontFace metrics = font_->face(font_size);
    Vec2 pen = offset() + position;
    pen.y += metrics.ascent;

    texture_ = no_texture;
    for (const char code : text) {
        const Glyph entry = font_->glyph(static_cast<unsigned char>(code), font_size);
        if (entry.size.x > 0.0f && entry.size.y > 0.0f) {
            QuadDesc desc;
            desc.rect = Rect2{pen + entry.offset, entry.size};
            desc.uv = entry.uv;
            desc.color = color;
            desc.mode = DrawMode::Glyph;
            quad(desc);
        }
        pen.x += entry.advance;
    }
}

Vec2 DrawCanvas::measure_text(std::string_view text, float font_size) const {
    return font_->measure(text, font_size);
}

FontFace DrawCanvas::metrics(float font_size) const {
    return font_->face(font_size);
}

} // namespace bench::ui
