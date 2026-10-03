#ifndef BENCH_UI_DRAW_H
#define BENCH_UI_DRAW_H

#include <stdint.h>

#include "array.h"
#include "ui_math.h"
#include "ui_strmap.h"

/// The drawing side of the UI kernel. `Canvas` is the engine's abstract
/// drawing surface as a struct with a vtable of function pointers; the
/// concrete `DrawCanvas` embeds it as its first member, so a `DrawCanvas*`
/// converts to a `Canvas*` by a plain cast.

/// Identifier of a texture; zero is the font atlas, which also holds the white pixel.
typedef uint64_t TextureId;

enum { UI_NO_TEXTURE = 0 };

/// Outline parameters of `CanvasVTable::stroke_rect`.
typedef struct {
    float width;
    float corner_radius;
} Stroke;

/// Vertical metrics of the font at one size.
typedef struct {
    float ascent;
    float descent;
    float line_height;
} FontFace;

/// One glyph of the fake atlas: its UV rectangle, offset from the pen, size and advance.
typedef struct {
    Rect2 uv;
    Vec2 offset;
    Vec2 size;
    float advance;
} Glyph;

/// Deterministic stand-in for the engine's TrueType atlas: the advance and
/// height of a glyph come from its character class, and the UVs from a
/// 16 by 8 grid of 32 pixel cells in an atlas of `atlas_extent` pixels.
typedef struct {
    float atlas_extent;
} Font;

/// A font with the 512 pixel atlas.
Font ui_font_make(void);

/// Ascent, descent and line height at `size`.
FontFace ui_font_face(const Font* font, float size);

/// The glyph for `code` at `size`.
Glyph ui_font_glyph(const Font* font, unsigned char code, float size);

/// The pen advance of `code` at `size`.
float ui_font_advance(const Font* font, unsigned char code, float size);

/// Width of `text` by walking its glyphs, and the line height.
Vec2 ui_font_measure(const Font* font, StrView text, float size);

/// The solid-fill block of the atlas.
Rect2 ui_font_white_pixel(const Font* font);

typedef struct Canvas Canvas;

/// The operations a drawing surface provides.
typedef struct {
    /// Narrows the clip rectangle to its intersection with `rect`.
    void (*push_clip)(Canvas* canvas, Rect2 rect);
    /// Restores the previous clip rectangle.
    void (*pop_clip)(Canvas* canvas);
    /// Fills `rect`, rounded when `corner_radius` is positive.
    void (*fill_rect)(Canvas* canvas, Rect2 rect, Color4 color, float corner_radius);
    /// Outlines `rect`.
    void (*stroke_rect)(Canvas* canvas, Rect2 rect, Color4 color, Stroke stroke);
    /// Draws `texture` over `rect` using `uv`, tinted by `modulate`.
    void (*fill_texture_rect)(Canvas* canvas, Rect2 rect, TextureId texture, Rect2 uv, Color4 modulate);
    /// Draws `text` with its top-left at `position`, one quad per glyph.
    void (*draw_text)(Canvas* canvas, Vec2 position, StrView text, float font_size, Color4 color);
    /// Size of `text` at `font_size`.
    Vec2 (*measure_text)(const Canvas* canvas, StrView text, float font_size);
    /// Vertical metrics at `font_size`.
    FontFace (*metrics)(const Canvas* canvas, float font_size);
} CanvasVTable;

/// Drawing surface the controls paint into: a vtable and the offset stack
/// that every canvas shares.
struct Canvas {
    const CanvasVTable* vt;
    ARRAY_OF(Vec2) offsets;
    Vec2 offset;
};

/// Sets up the base part of a canvas with the operations `vt`.
void ui_canvas_init(Canvas* canvas, const CanvasVTable* vt);

/// Releases the base part of a canvas.
void ui_canvas_destroy(Canvas* canvas);

/// Shifts the origin by `delta` until the matching `ui_canvas_pop_offset`.
void ui_canvas_push_offset(Canvas* canvas, Vec2 delta);

/// Restores the origin before the last `ui_canvas_push_offset`.
void ui_canvas_pop_offset(Canvas* canvas);

/// How the UI shader interprets a vertex.
typedef enum {
    UI_DRAW_TEXTURED = 0,
    UI_DRAW_ROUNDED_FILL = 1,
    UI_DRAW_ROUNDED_STROKE = 2,
    UI_DRAW_GLYPH = 3,
} DrawMode;

/// One UI vertex: position, atlas UV, the SDF rectangle and a packed RGBA colour.
typedef struct {
    Vec2 position;
    Vec2 uv;
    Vec2 rect_center;
    Vec2 rect_half;
    uint32_t color;
    float radius;
    float stroke;
    uint32_t mode;
} DrawVertex;

/// A run of indices drawn with one texture under one clip rectangle.
typedef struct {
    uint32_t first_index;
    uint32_t index_count;
    Rect2 clip;
    TextureId texture;
} DrawCommand;

/// The vertices, indices and commands a `DrawCanvas` records for one frame.
typedef struct {
    ARRAY_OF(DrawVertex) vertices;
    ARRAY_OF(uint32_t) indices;
    ARRAY_OF(DrawCommand) commands;
} DrawData;

/// Canvas that records `DrawData`, batching quads by texture and clip rectangle.
typedef struct {
    Canvas base;
    const Font* font;
    DrawData data;
    ARRAY_OF(Rect2) clips;
    Rect2 viewport;
    TextureId texture;
} DrawCanvas;

/// A canvas that measures and rasterises text through `font`, which must outlive it.
void ui_draw_canvas_init(DrawCanvas* canvas, const Font* font);

/// Releases everything the canvas holds.
void ui_draw_canvas_destroy(DrawCanvas* canvas);

/// Discards the previous frame, keeping capacity, and starts recording against `viewport`.
void ui_draw_canvas_begin(DrawCanvas* canvas, Rect2 viewport);

#endif
