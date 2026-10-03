#include "ui_draw.h"

#include <math.h>

static const float sdf_bleed = 1.5f;
static const float default_atlas_extent = 512.0f;

/// Advance and height of a glyph as fractions of the font size.
typedef struct {
    float advance;
    float height;
} GlyphClass;

/// Parameters of one axis-aligned quad.
typedef struct {
    Rect2 rect;
    Rect2 uv;
    Color4 color;
    DrawMode mode;
    float radius;
    float stroke;
} QuadDesc;

static const GlyphClass glyph_classes[7] = {
    { 0.3f, 0.0f },
    { 0.28f, 0.7f },
    { 0.55f, 0.7f },
    { 0.52f, 0.55f },
    { 0.64f, 0.72f },
    { 0.85f, 0.72f },
    { 0.5f, 0.7f },
};

/// The index into `glyph_classes` of the character class of `code`.
static size_t classify(unsigned char code) {
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
static uint32_t channel(float value) {
    return (uint32_t)floorf(f32_clamp(value, 0.0f, 1.0f) * 255.0f + 0.5f);
}

/// A colour packed into 8-bit RGBA with red in the lowest byte.
static uint32_t packed(Color4 color) {
    return channel(color.r) | (channel(color.g) << 8u) | (channel(color.b) << 16u) | (channel(color.a) << 24u);
}

Font ui_font_make(void) { return (Font){ default_atlas_extent }; }

FontFace ui_font_face(const Font* font, float size) {
    (void)font;
    return (FontFace){ size * 0.75f, size * 0.25f, size * 1.25f };
}

float ui_font_advance(const Font* font, unsigned char code, float size) {
    (void)font;
    return size * glyph_classes[classify(code)].advance;
}

Glyph ui_font_glyph(const Font* font, unsigned char code, float size) {
    const GlyphClass* entry = &glyph_classes[classify(code)];
    Glyph made = { 0 };
    made.advance = size * entry->advance;
    if (entry->height <= 0.0f) return made;
    float width = floorf(made.advance * 0.75f) + 1.0f;
    float height = floorf(size * entry->height) + 1.0f;
    made.size = ui_vec2(width, height);
    made.offset = ui_vec2(floorf(made.advance * 0.125f), -height);
    uint32_t cell = code & 127u;
    float u = (float)((cell & 15u) * 32u) / font->atlas_extent;
    float v = (float)((cell >> 4u) * 32u) / font->atlas_extent;
    made.uv = ui_rect2(ui_vec2(u, v), ui_vec2(width / font->atlas_extent, height / font->atlas_extent));
    return made;
}

Vec2 ui_font_measure(const Font* font, StrView text, float size) {
    FontFace metrics = ui_font_face(font, size);
    float width = 0.0f;
    for (size_t index = 0; index < text.size; ++index) width += ui_font_advance(font, (unsigned char)text.data[index], size);
    return ui_vec2(width, metrics.line_height);
}

Rect2 ui_font_white_pixel(const Font* font) {
    float texel = 1.0f / font->atlas_extent;
    return ui_rect2(ui_vec2(texel, texel), ui_vec2(texel, texel));
}

void ui_canvas_init(Canvas* canvas, const CanvasVTable* vt) {
    canvas->vt = vt;
    canvas->offsets.data = NULL;
    canvas->offsets.len = 0;
    canvas->offsets.cap = 0;
    canvas->offset = ui_vec2(0.0f, 0.0f);
}

void ui_canvas_destroy(Canvas* canvas) { ARRAY_FREE(canvas->offsets); }

void ui_canvas_push_offset(Canvas* canvas, Vec2 delta) {
    Vec2 previous = canvas->offset;
    ARRAY_PUSH(canvas->offsets, previous);
    canvas->offset = ui_vec2_add(canvas->offset, delta);
}

void ui_canvas_pop_offset(Canvas* canvas) {
    if (canvas->offsets.len == 0) {
        canvas->offset = ui_vec2(0.0f, 0.0f);
        return;
    }
    canvas->offset = ARRAY_POP(canvas->offsets);
}

/// The innermost clip rectangle, or the viewport when none is pushed.
static Rect2 current_clip(const DrawCanvas* canvas) {
    return canvas->clips.len == 0 ? canvas->viewport : canvas->clips.data[canvas->clips.len - 1];
}

/// Makes the last command match the current clip and `texture`, starting a new one when it does not.
static void use_texture(DrawCanvas* canvas, TextureId texture) {
    Rect2 area = current_clip(canvas);
    DrawData* data = &canvas->data;
    if (data->commands.len != 0) {
        DrawCommand* last = &ARRAY_BACK(data->commands);
        if (last->texture == texture && ui_rect2_equal(&last->clip, &area)) return;
        if (last->index_count == 0) {
            last->texture = texture;
            last->clip = area;
            return;
        }
    }
    DrawCommand next = { 0 };
    next.first_index = (uint32_t)data->indices.len;
    next.texture = texture;
    next.clip = area;
    ARRAY_PUSH(data->commands, next);
}

/// Appends the quad `desc`, skipping it when empty, transparent or fully clipped.
static void quad(DrawCanvas* canvas, const QuadDesc* desc) {
    if (desc->rect.size.x <= 0.0f || desc->rect.size.y <= 0.0f || desc->color.a <= 0.0f) return;
    Rect2 area = current_clip(canvas);
    Rect2 visible = ui_rect2_intersection(&area, &desc->rect);
    if (visible.size.x <= 0.0f || visible.size.y <= 0.0f) return;
    use_texture(canvas, canvas->texture);

    DrawData* data = &canvas->data;
    Rect2 target = desc->rect;
    Rect2 coords = desc->uv;
    if (desc->mode == UI_DRAW_ROUNDED_FILL || desc->mode == UI_DRAW_ROUNDED_STROKE) target = ui_rect2_grow(&desc->rect, sdf_bleed);

    uint32_t base = (uint32_t)data->vertices.len;
    uint32_t tint = packed(desc->color);
    Vec2 centre = ui_rect2_center(&desc->rect);
    Vec2 half = ui_vec2_scale(desc->rect.size, 0.5f);
    uint32_t kind = (uint32_t)desc->mode;

    Vec2 corners[4] = { target.position, ui_vec2(ui_rect2_right(&target), ui_rect2_top(&target)), ui_rect2_end(&target),
        ui_vec2(ui_rect2_left(&target), ui_rect2_bottom(&target)) };
    Vec2 texels[4] = { coords.position, ui_vec2(ui_rect2_right(&coords), ui_rect2_top(&coords)), ui_rect2_end(&coords),
        ui_vec2(ui_rect2_left(&coords), ui_rect2_bottom(&coords)) };
    for (size_t index = 0; index < 4; ++index) {
        DrawVertex vertex = { 0 };
        vertex.position = corners[index];
        vertex.uv = texels[index];
        vertex.rect_center = centre;
        vertex.rect_half = half;
        vertex.color = tint;
        vertex.radius = desc->radius;
        vertex.stroke = desc->stroke;
        vertex.mode = kind;
        ARRAY_PUSH(data->vertices, vertex);
    }
    static const uint32_t quad_indices[6] = { 0u, 1u, 2u, 0u, 2u, 3u };
    for (size_t index = 0; index < 6; ++index) {
        uint32_t value = base + quad_indices[index];
        ARRAY_PUSH(data->indices, value);
    }
    ARRAY_BACK(data->commands).index_count += 6;
}

/// A quad over `rect` offset by the canvas origin, sampling the white pixel, with the given colour.
static QuadDesc solid_quad(const DrawCanvas* canvas, Rect2 rect, Color4 color) {
    QuadDesc desc = { 0 };
    desc.rect = ui_rect2(ui_vec2_add(canvas->base.offset, rect.position), rect.size);
    desc.uv = ui_font_white_pixel(canvas->font);
    desc.color = color;
    return desc;
}

/// Narrows the clip rectangle to its intersection with `rect`.
static void draw_push_clip(Canvas* base, Rect2 rect) {
    DrawCanvas* canvas = (DrawCanvas*)base;
    Rect2 area = current_clip(canvas);
    Rect2 narrowed = ui_rect2_intersection(&area, &rect);
    ARRAY_PUSH(canvas->clips, narrowed);
}

/// Restores the previous clip rectangle.
static void draw_pop_clip(Canvas* base) {
    DrawCanvas* canvas = (DrawCanvas*)base;
    if (canvas->clips.len != 0) canvas->clips.len -= 1;
}

/// Fills `rect`, rounded when `corner_radius` is positive.
static void draw_fill_rect(Canvas* base, Rect2 rect, Color4 color, float corner_radius) {
    DrawCanvas* canvas = (DrawCanvas*)base;
    canvas->texture = UI_NO_TEXTURE;
    QuadDesc desc = solid_quad(canvas, rect, color);
    desc.mode = corner_radius <= 0.0f ? UI_DRAW_TEXTURED : UI_DRAW_ROUNDED_FILL;
    desc.radius = corner_radius;
    quad(canvas, &desc);
}

/// Outlines `rect`.
static void draw_stroke_rect(Canvas* base, Rect2 rect, Color4 color, Stroke stroke) {
    DrawCanvas* canvas = (DrawCanvas*)base;
    canvas->texture = UI_NO_TEXTURE;
    QuadDesc desc = solid_quad(canvas, rect, color);
    desc.mode = UI_DRAW_ROUNDED_STROKE;
    desc.radius = stroke.corner_radius;
    desc.stroke = f32_max(stroke.width, 1.0f);
    quad(canvas, &desc);
}

/// Draws `texture` over `rect` using `uv`, tinted by `modulate`.
static void draw_fill_texture_rect(Canvas* base, Rect2 rect, TextureId texture, Rect2 uv, Color4 modulate) {
    DrawCanvas* canvas = (DrawCanvas*)base;
    if (texture == UI_NO_TEXTURE) return;
    canvas->texture = texture;
    QuadDesc desc = { 0 };
    desc.rect = ui_rect2(ui_vec2_add(base->offset, rect.position), rect.size);
    desc.uv = uv;
    desc.color = modulate;
    quad(canvas, &desc);
    canvas->texture = UI_NO_TEXTURE;
}

/// Draws `text` with its top-left at `position`, one quad per glyph.
static void draw_text_glyphs(Canvas* base, Vec2 position, StrView text, float font_size, Color4 color) {
    DrawCanvas* canvas = (DrawCanvas*)base;
    if (text.size == 0 || color.a <= 0.0f) return;
    FontFace metrics = ui_font_face(canvas->font, font_size);
    Vec2 pen = ui_vec2_add(base->offset, position);
    pen.y += metrics.ascent;

    canvas->texture = UI_NO_TEXTURE;
    for (size_t index = 0; index < text.size; ++index) {
        Glyph entry = ui_font_glyph(canvas->font, (unsigned char)text.data[index], font_size);
        if (entry.size.x > 0.0f && entry.size.y > 0.0f) {
            QuadDesc desc = { 0 };
            desc.rect = ui_rect2(ui_vec2_add(pen, entry.offset), entry.size);
            desc.uv = entry.uv;
            desc.color = color;
            desc.mode = UI_DRAW_GLYPH;
            quad(canvas, &desc);
        }
        pen.x += entry.advance;
    }
}

/// Size of `text` at `font_size`.
static Vec2 draw_measure_text(const Canvas* base, StrView text, float font_size) {
    return ui_font_measure(((const DrawCanvas*)base)->font, text, font_size);
}

/// Vertical metrics at `font_size`.
static FontFace draw_metrics(const Canvas* base, float font_size) {
    return ui_font_face(((const DrawCanvas*)base)->font, font_size);
}

static const CanvasVTable draw_canvas_vtable = {
    .push_clip = draw_push_clip,
    .pop_clip = draw_pop_clip,
    .fill_rect = draw_fill_rect,
    .stroke_rect = draw_stroke_rect,
    .fill_texture_rect = draw_fill_texture_rect,
    .draw_text = draw_text_glyphs,
    .measure_text = draw_measure_text,
    .metrics = draw_metrics,
};

void ui_draw_canvas_init(DrawCanvas* canvas, const Font* font) {
    *canvas = (DrawCanvas){ 0 };
    ui_canvas_init(&canvas->base, &draw_canvas_vtable);
    canvas->font = font;
}

void ui_draw_canvas_destroy(DrawCanvas* canvas) {
    ARRAY_FREE(canvas->data.vertices);
    ARRAY_FREE(canvas->data.indices);
    ARRAY_FREE(canvas->data.commands);
    ARRAY_FREE(canvas->clips);
    ui_canvas_destroy(&canvas->base);
}

void ui_draw_canvas_begin(DrawCanvas* canvas, Rect2 viewport) {
    canvas->data.vertices.len = 0;
    canvas->data.indices.len = 0;
    canvas->data.commands.len = 0;
    canvas->clips.len = 0;
    canvas->viewport = viewport;
    canvas->texture = UI_NO_TEXTURE;
}
