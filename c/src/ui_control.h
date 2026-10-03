#ifndef BENCH_UI_CONTROL_H
#define BENCH_UI_CONTROL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "array.h"
#include "ui_draw.h"
#include "ui_math.h"
#include "ui_strmap.h"
#include "ui_theme.h"

/// The base of the UI tree as an object-oriented C struct. A `Control`
/// carries a pointer to a `ControlVTable` of function pointers, the way the
/// C++ class has virtual functions. A subclass is a struct whose first member
/// is its base, so a `Label*` converts to a `Control*` by a plain cast and
/// back, and its `destroy` slot chains to the base destroy like a destructor.
/// A parent owns its children, which are malloc'd; `ui_control_free`
/// destroys a node and its whole subtree.

typedef struct Control Control;
typedef struct Context Context;

/// A side of a rectangle.
typedef enum { UI_SIDE_LEFT, UI_SIDE_TOP, UI_SIDE_RIGHT, UI_SIDE_BOTTOM } Side;

/// Named anchor layouts such as corners, edges and full rect.
typedef enum {
    UI_PRESET_TOP_LEFT,
    UI_PRESET_TOP_RIGHT,
    UI_PRESET_BOTTOM_LEFT,
    UI_PRESET_BOTTOM_RIGHT,
    UI_PRESET_CENTER_LEFT,
    UI_PRESET_CENTER_TOP,
    UI_PRESET_CENTER_RIGHT,
    UI_PRESET_CENTER_BOTTOM,
    UI_PRESET_CENTER,
    UI_PRESET_LEFT_WIDE,
    UI_PRESET_TOP_WIDE,
    UI_PRESET_RIGHT_WIDE,
    UI_PRESET_BOTTOM_WIDE,
    UI_PRESET_VCENTER_WIDE,
    UI_PRESET_HCENTER_WIDE,
    UI_PRESET_FULL_RECT
} LayoutPreset;

/// The four anchor values of a layout preset.
typedef struct {
    float left;
    float top;
    float right;
    float bottom;
} PresetAnchors;

/// The anchors that `preset` places on each side.
PresetAnchors ui_preset_anchors(LayoutPreset preset);

/// Which way a control grows when its minimum size exceeds its rect.
typedef enum { UI_GROW_BEGIN, UI_GROW_END, UI_GROW_BOTH } GrowDirection;

/// How a container sizes and places a child on one axis, as combinable bits.
typedef uint8_t SizeFlags;

enum {
    UI_SIZE_NONE = 0,
    UI_SIZE_FILL = 1,
    UI_SIZE_EXPAND = 2,
    UI_SIZE_SHRINK_CENTER = 4,
    UI_SIZE_SHRINK_END = 8,
    UI_SIZE_EXPAND_FILL = UI_SIZE_FILL | UI_SIZE_EXPAND
};

/// Whether any bit of `flag` is set in `value`.
static inline bool ui_has_flag(SizeFlags value, SizeFlags flag) { return (value & flag) != 0; }

/// Capacity of the theme type list of one control, bounded by the deepest class chain plus a variation.
enum { UI_MAX_THEME_TYPES = 8 };

/// The theme type names that look up a control's items, most specific first.
typedef struct {
    StrView items[UI_MAX_THEME_TYPES];
    size_t count;
} ThemeTypes;

/// Appends `type` to `types`.
void ui_theme_types_add(ThemeTypes* types, StrView type);

/// The virtual functions of a control class.
typedef struct {
    /// Releases what the class owns and chains to its base class; the node itself is freed by the caller.
    void (*destroy)(Control* control);
    /// Appends the theme type names used to look up this control's theme items, most specific first.
    void (*collect_theme_types)(const Control* control, ThemeTypes* out);
    /// The intrinsic minimum size.
    Vec2 (*get_minimum_size)(const Control* control);
    /// Draws the control.
    void (*draw)(const Control* control, Canvas* canvas);
    /// Positions the children.
    void (*layout_children)(Control* control);
    /// Called after the size changes.
    void (*resized)(Control* control);
    /// Places the sortable children; set by containers, whose `layout_children` calls it, and null otherwise.
    void (*sort_children)(Control* control);
} ControlVTable;

/// Growable array of owned children.
typedef ARRAY_OF(Control*) ControlArray;

/// A rectangle with anchors, a cached minimum size, string-keyed theme lookup
/// through the control and theme chains, and virtual layout and drawing.
/// Fields are public, as in any C struct; outside code reads them and changes
/// them only through the functions below.
struct Control {
    const ControlVTable* vt;
    Control* parent;
    Context* ui;
    ControlArray children;

    float anchor[4];
    float offset[4];
    GrowDirection h_grow;
    GrowDirection v_grow;

    Vec2 position;
    Vec2 size;
    Vec2 custom_minimum_size;
    Vec2 minimum_size_cache;
    bool minimum_size_valid;

    SizeFlags h_size_flags;
    SizeFlags v_size_flags;
    float stretch_ratio;

    bool visible;
    bool clip_contents;
    Color4 modulate;
    bool hovered;

    Theme* theme;
    OwnedStr theme_type_variation;
    StrMap color_overrides;
    StrMap constant_overrides;
};

/// The vtable of a plain `Control`.
extern const ControlVTable ui_control_vtable;

/// Sets up the base part of a zeroed control of the class with `vt`.
void ui_control_init(Control* control, const ControlVTable* vt);

/// The base destroy: frees the children, theme, variation and overrides, but not the node.
void ui_control_destroy(Control* control);

/// Destroys `control` through its vtable and frees the node.
void ui_control_free(Control* control);

/// Appends `child` and takes ownership of it.
Control* ui_control_add_child(Control* parent, Control* child);

/// Allocates a zeroed node of `size` bytes of the class with `vt`, appends it to `parent` and returns it.
Control* ui_control_add_new(Control* parent, size_t size, const ControlVTable* vt);

/// Sets one anchor, clamped to [0, 1], keeping the edge in place when `keep_offset` is set.
void ui_control_set_anchor(Control* control, Side side, float value, bool keep_offset);

/// Sets the distance of one edge from its anchor.
void ui_control_set_offset(Control* control, Side side, float value);

/// Sets all four anchors from `preset`.
void ui_control_set_anchors_preset(Control* control, LayoutPreset preset, bool keep_offsets);

/// Sets all four offsets so the control fits `preset` at its current or minimum size.
void ui_control_set_offsets_preset(Control* control, LayoutPreset preset, float margin);

/// Applies `preset` to both anchors and offsets.
void ui_control_set_anchors_and_offsets_preset(Control* control, LayoutPreset preset, float margin);

/// Sets position and size, not below the minimum size, without queuing a layout.
void ui_control_set_rect(Control* control, Rect2 rect);

/// Sets a floor for the minimum size.
void ui_control_set_custom_minimum_size(Control* control, Vec2 size);

/// The larger of the custom and intrinsic minimum sizes, cached until invalidated.
Vec2 ui_control_combined_minimum_size(Control* control);

/// Invalidates the cached minimum size here and in every ancestor, and queues a layout.
void ui_control_update_minimum_size(Control* control);

/// Sets how a container sizes this control horizontally.
void ui_control_set_h_size_flags(Control* control, SizeFlags flags);

/// Sets how a container sizes this control vertically.
void ui_control_set_v_size_flags(Control* control, SizeFlags flags);

/// Sets the share of spare space among expanding siblings, floored at zero.
void ui_control_set_stretch_ratio(Control* control, float ratio);

/// Sets the horizontal grow direction.
void ui_control_set_h_grow_direction(Control* control, GrowDirection direction);

/// Sets the vertical grow direction.
void ui_control_set_v_grow_direction(Control* control, GrowDirection direction);

/// Shows or hides the control and its subtree.
void ui_control_set_visible(Control* control, bool visible);

/// Enables clipping of drawing to the control rect.
void ui_control_set_clip_contents(Control* control, bool clip);

/// Sets the colour multiplied into this control and its descendants.
void ui_control_set_modulate(Control* control, Color4 modulate);

/// The colour multiplied through every ancestor.
Color4 ui_control_effective_modulate(const Control* control);

/// Marks the pointer as over this control.
void ui_control_set_hovered(Control* control, bool hovered);

/// Whether this control holds keyboard focus.
bool ui_control_has_focus(const Control* control);

/// Sets a theme for this control and its descendants, taking ownership of it.
void ui_control_set_theme(Control* control, Theme* theme);

/// Sets a theme type searched before the control types.
void ui_control_set_theme_type_variation(Control* control, StrView type);

/// Overrides a colour by name for this control only.
void ui_control_add_color_override(Control* control, StrView name, Color4 value);

/// Overrides a constant by name for this control only.
void ui_control_add_constant_override(Control* control, StrView name, float value);

/// Resolves a style box by name through overrides, ancestor themes and the context theme.
const StyleBox* ui_control_theme_stylebox(const Control* control, StrView name);

/// Resolves a colour by name through overrides, ancestor themes and the context theme.
Color4 ui_control_theme_color(const Control* control, StrView name);

/// Resolves a constant by name through overrides, ancestor themes and the context theme.
float ui_control_theme_constant(const Control* control, StrView name);

/// Resolves a font size by name through overrides, ancestor themes and the context theme.
float ui_control_theme_font_size(const Control* control, StrView name);

/// The style box `name` of `control`, with `name` a string literal.
#define UI_STYLEBOX(control, name) ui_control_theme_stylebox((control), UI_SV(name))

/// The colour `name` of `control`, with `name` a string literal.
#define UI_COLOR(control, name) ui_control_theme_color((control), UI_SV(name))

/// The constant `name` of `control`, with `name` a string literal.
#define UI_CONSTANT(control, name) ui_control_theme_constant((control), UI_SV(name))

/// The font size `name` of `control`, with `name` a string literal.
#define UI_FONT_SIZE(control, name) ui_control_theme_font_size((control), UI_SV(name))

/// The canvas of the owning context.
Canvas* ui_control_canvas(const Control* control);

/// Marks the owning context for layout on its next update.
void ui_control_queue_layout(Control* control);

/// Default `collect_theme_types`: appends `Control`.
void ui_control_collect_theme_types(const Control* control, ThemeTypes* out);

/// Default `get_minimum_size`: zero.
Vec2 ui_control_get_minimum_size(const Control* control);

/// Default `draw`: draws nothing.
void ui_control_draw(const Control* control, Canvas* canvas);

/// Default `layout_children`: applies the anchors of each child.
void ui_control_layout_children(Control* control);

/// Default `resized`: does nothing.
void ui_control_resized(Control* control);

/// The owner of a control tree: the root control, the fallback theme, the
/// canvas, the viewport and the dirty flag the per-frame update checks.
struct Context {
    Canvas* canvas;
    Control* root;
    Theme* theme;
    Control* focus;
    Rect2 viewport_rect;
    bool layout_dirty;
};

/// Sets up a context with an empty root and the default theme that paints into `canvas`.
void ui_context_init(Context* context, Canvas* canvas);

/// Frees the root tree and the fallback theme.
void ui_context_destroy(Context* context);

/// Moves the keyboard focus to `control`, or clears it.
void ui_context_set_focus(Context* context, Control* control);

/// Sets the viewport rectangle, queuing a layout when it changes.
void ui_context_set_viewport_rect(Context* context, Rect2 rect);

/// Marks the tree for layout on the next update.
void ui_context_queue_layout(Context* context);

/// Fits the root to the viewport and, when a layout is queued, lays out the tree.
void ui_context_update(Context* context);

/// Paints the tree into the canvas.
void ui_context_draw(Context* context);

#endif
