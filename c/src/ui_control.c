#include "ui_control.h"

#include <stdlib.h>

#include "alloc.h"

enum { MAX_TYPE_DEPTH = 8 };

static const StyleBox fallback_stylebox = {
    UI_STYLEBOX_EMPTY, { 1.0f, 1.0f, 1.0f, 1.0f }, { 1.0f, 1.0f, 1.0f, 1.0f }, 0.0f, 0.0f, { 0.0f, 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f, 0.0f }
};

/// Walks `type` and its base types in `theme`, returning the first item of `kind` named `name` that it finds, or null.
static const void* find_in_type_chain(const Theme* theme, ThemeItemKind kind, StrView type, StrView name) {
    StrView current = type;
    for (size_t depth = 0; depth < MAX_TYPE_DEPTH; ++depth) {
        const void* found = ui_theme_item(theme, kind, (ThemeKey){ current, name });
        if (found != NULL) return found;
        StrView base = ui_theme_type_base(theme, current);
        if (base.size == 0) break;
        current = base;
    }
    return NULL;
}

/// Tries every type of `types` in `theme`, each up its base chain, and returns the first item found, or null.
static const void* find_in_theme(const Theme* theme, ThemeItemKind kind, const ThemeTypes* types, StrView name) {
    for (size_t index = 0; index < types->count; ++index) {
        const void* found = find_in_type_chain(theme, kind, types->items[index], name);
        if (found != NULL) return found;
    }
    return NULL;
}

/// Looks up a theme item for a control, trying its type variation and theme
/// types up each base chain. Searches the control's own theme and its
/// ancestors' themes before the context's fallback theme.
static const void* find_in_themes(const Control* control, ThemeItemKind kind, StrView name) {
    ThemeTypes types = { { { NULL, 0 } }, 0 };
    if (control->theme_type_variation.size != 0) ui_theme_types_add(&types, ui_str_view(&control->theme_type_variation));
    control->vt->collect_theme_types(control, &types);

    for (const Control* owner = control; owner != NULL; owner = owner->parent) {
        if (owner->theme == NULL) continue;
        const void* found = find_in_theme(owner->theme, kind, &types, name);
        if (found != NULL) return found;
    }

    if (control->ui == NULL || control->ui->theme == NULL) return NULL;
    return find_in_theme(control->ui->theme, kind, &types, name);
}

/// Sets the owning context on this subtree.
static void propagate_ui(Control* control, Context* ui) {
    if (control->ui == ui) return;
    control->ui = ui;
    for (size_t index = 0; index < control->children.len; ++index) propagate_ui(control->children.data[index], ui);
}

/// Links the control to a parent and sets the owning context on its subtree.
static void attach(Control* control, Control* parent, Context* ui) {
    control->parent = parent;
    propagate_ui(control, ui);
}

/// The size of the parent along the axis of edge `index`: width for left and right, height for top and bottom.
static float axis_extent(Vec2 size, size_t index) { return (index % 2 == 0) ? size.x : size.y; }

/// Computes position and size from anchors and offsets inside `parent_size`.
static void apply_anchors(Control* control, Vec2 parent_size) {
    float edges[4] = { 0 };
    for (size_t i = 0; i < 4; ++i) {
        float area = axis_extent(parent_size, i);
        edges[i] = control->offset[i] + control->anchor[i] * area;
    }

    Vec2 new_position = ui_vec2(edges[0], edges[1]);
    Vec2 new_size = ui_vec2(edges[2] - edges[0], edges[3] - edges[1]);
    Vec2 minimum = ui_control_combined_minimum_size(control);

    if (minimum.x > new_size.x) {
        if (control->h_grow == UI_GROW_BEGIN) {
            new_position.x += new_size.x - minimum.x;
        } else if (control->h_grow == UI_GROW_BOTH) {
            new_position.x += (new_size.x - minimum.x) * 0.5f;
        }
        new_size.x = minimum.x;
    }
    if (minimum.y > new_size.y) {
        if (control->v_grow == UI_GROW_BEGIN) {
            new_position.y += new_size.y - minimum.y;
        } else if (control->v_grow == UI_GROW_BOTH) {
            new_position.y += (new_size.y - minimum.y) * 0.5f;
        }
        new_size.y = minimum.y;
    }

    control->position = new_position;
    if (!ui_vec2_equal(control->size, new_size)) {
        control->size = new_size;
        control->vt->resized(control);
    }
}

/// Sets the offsets of one axis so the control fits an anchor pair at extent `target`.
static void fit_axis(Control* control, size_t low, size_t high, float anchor_low, float anchor_high, float target, float margin) {
    if (anchor_low != anchor_high) {
        control->offset[low] = margin;
        control->offset[high] = -margin;
    } else if (anchor_low == 0.0f) {
        control->offset[low] = margin;
        control->offset[high] = margin + target;
    } else if (anchor_low == 1.0f) {
        control->offset[low] = -margin - target;
        control->offset[high] = -margin;
    } else {
        control->offset[low] = -target * 0.5f;
        control->offset[high] = target * 0.5f;
    }
}

PresetAnchors ui_preset_anchors(LayoutPreset preset) {
    switch (preset) {
    case UI_PRESET_TOP_LEFT:
        return (PresetAnchors){ 0.0f, 0.0f, 0.0f, 0.0f };
    case UI_PRESET_TOP_RIGHT:
        return (PresetAnchors){ 1.0f, 0.0f, 1.0f, 0.0f };
    case UI_PRESET_BOTTOM_LEFT:
        return (PresetAnchors){ 0.0f, 1.0f, 0.0f, 1.0f };
    case UI_PRESET_BOTTOM_RIGHT:
        return (PresetAnchors){ 1.0f, 1.0f, 1.0f, 1.0f };
    case UI_PRESET_CENTER_LEFT:
        return (PresetAnchors){ 0.0f, 0.5f, 0.0f, 0.5f };
    case UI_PRESET_CENTER_TOP:
        return (PresetAnchors){ 0.5f, 0.0f, 0.5f, 0.0f };
    case UI_PRESET_CENTER_RIGHT:
        return (PresetAnchors){ 1.0f, 0.5f, 1.0f, 0.5f };
    case UI_PRESET_CENTER_BOTTOM:
        return (PresetAnchors){ 0.5f, 1.0f, 0.5f, 1.0f };
    case UI_PRESET_CENTER:
        return (PresetAnchors){ 0.5f, 0.5f, 0.5f, 0.5f };
    case UI_PRESET_LEFT_WIDE:
        return (PresetAnchors){ 0.0f, 0.0f, 0.0f, 1.0f };
    case UI_PRESET_TOP_WIDE:
        return (PresetAnchors){ 0.0f, 0.0f, 1.0f, 0.0f };
    case UI_PRESET_RIGHT_WIDE:
        return (PresetAnchors){ 1.0f, 0.0f, 1.0f, 1.0f };
    case UI_PRESET_BOTTOM_WIDE:
        return (PresetAnchors){ 0.0f, 1.0f, 1.0f, 1.0f };
    case UI_PRESET_VCENTER_WIDE:
        return (PresetAnchors){ 0.0f, 0.5f, 1.0f, 0.5f };
    case UI_PRESET_HCENTER_WIDE:
        return (PresetAnchors){ 0.5f, 0.0f, 0.5f, 1.0f };
    case UI_PRESET_FULL_RECT:
        return (PresetAnchors){ 0.0f, 0.0f, 1.0f, 1.0f };
    }
    return (PresetAnchors){ 0.0f, 0.0f, 0.0f, 0.0f };
}

void ui_theme_types_add(ThemeTypes* types, StrView type) {
    if (types->count < UI_MAX_THEME_TYPES) types->items[types->count++] = type;
}

const ControlVTable ui_control_vtable = {
    .destroy = ui_control_destroy,
    .collect_theme_types = ui_control_collect_theme_types,
    .get_minimum_size = ui_control_get_minimum_size,
    .draw = ui_control_draw,
    .layout_children = ui_control_layout_children,
    .resized = ui_control_resized,
    .sort_children = NULL,
};

void ui_control_init(Control* control, const ControlVTable* vt) {
    control->vt = vt;
    control->h_grow = UI_GROW_END;
    control->v_grow = UI_GROW_END;
    control->h_size_flags = UI_SIZE_FILL;
    control->v_size_flags = UI_SIZE_FILL;
    control->stretch_ratio = 1.0f;
    control->visible = true;
    control->modulate = ui_white();
    ui_strmap_init(&control->color_overrides, sizeof(Color4));
    ui_strmap_init(&control->constant_overrides, sizeof(float));
}

void ui_control_destroy(Control* control) {
    for (size_t index = 0; index < control->children.len; ++index) ui_control_free(control->children.data[index]);
    ARRAY_FREE(control->children);
    if (control->theme != NULL) ui_theme_free(control->theme);
    ui_str_free(&control->theme_type_variation);
    ui_strmap_clear(&control->color_overrides, NULL);
    ui_strmap_clear(&control->constant_overrides, NULL);
}

void ui_control_free(Control* control) {
    control->vt->destroy(control);
    free(control);
}

Control* ui_control_add_child(Control* parent, Control* child) {
    ARRAY_PUSH(parent->children, child);
    attach(child, parent, parent->ui);
    ui_control_update_minimum_size(parent);
    ui_control_queue_layout(parent);
    return child;
}

Control* ui_control_add_new(Control* parent, size_t size, const ControlVTable* vt) {
    Control* child = xalloc(size);
    ui_control_init(child, vt);
    return ui_control_add_child(parent, child);
}

void ui_control_set_anchor(Control* control, Side side, float value, bool keep_offset) {
    size_t index = (size_t)side;
    float previous = control->anchor[index];
    control->anchor[index] = f32_clamp(value, 0.0f, 1.0f);
    if (keep_offset && control->parent != NULL) {
        float area = axis_extent(control->parent->size, index);
        control->offset[index] -= (control->anchor[index] - previous) * area;
    }
    ui_control_queue_layout(control);
}

void ui_control_set_offset(Control* control, Side side, float value) {
    control->offset[(size_t)side] = value;
    ui_control_queue_layout(control);
}

void ui_control_set_anchors_preset(Control* control, LayoutPreset preset, bool keep_offsets) {
    PresetAnchors anchors = ui_preset_anchors(preset);
    ui_control_set_anchor(control, UI_SIDE_LEFT, anchors.left, keep_offsets);
    ui_control_set_anchor(control, UI_SIDE_TOP, anchors.top, keep_offsets);
    ui_control_set_anchor(control, UI_SIDE_RIGHT, anchors.right, keep_offsets);
    ui_control_set_anchor(control, UI_SIDE_BOTTOM, anchors.bottom, keep_offsets);
}

void ui_control_set_offsets_preset(Control* control, LayoutPreset preset, float margin) {
    PresetAnchors anchors = ui_preset_anchors(preset);
    Vec2 minimum = ui_control_combined_minimum_size(control);
    Vec2 target = ui_vec2_max(control->size, minimum);
    fit_axis(control, 0, 2, anchors.left, anchors.right, target.x, margin);
    fit_axis(control, 1, 3, anchors.top, anchors.bottom, target.y, margin);
    ui_control_queue_layout(control);
}

void ui_control_set_anchors_and_offsets_preset(Control* control, LayoutPreset preset, float margin) {
    ui_control_set_anchors_preset(control, preset, false);
    ui_control_set_offsets_preset(control, preset, margin);
}

void ui_control_set_rect(Control* control, Rect2 rect) {
    Vec2 target = ui_vec2_max(rect.size, ui_control_combined_minimum_size(control));
    control->position = rect.position;
    control->offset[0] = rect.position.x;
    control->offset[1] = rect.position.y;
    control->offset[2] = rect.position.x + target.x;
    control->offset[3] = rect.position.y + target.y;
    if (!ui_vec2_equal(control->size, target)) {
        control->size = target;
        control->vt->resized(control);
    }
}

void ui_control_set_custom_minimum_size(Control* control, Vec2 size) {
    if (ui_vec2_equal(control->custom_minimum_size, size)) return;
    control->custom_minimum_size = size;
    ui_control_update_minimum_size(control);
}

Vec2 ui_control_combined_minimum_size(Control* control) {
    if (!control->minimum_size_valid) {
        control->minimum_size_cache = ui_vec2_max(control->custom_minimum_size, control->vt->get_minimum_size(control));
        control->minimum_size_valid = true;
    }
    return control->minimum_size_cache;
}

void ui_control_update_minimum_size(Control* control) {
    control->minimum_size_valid = false;
    for (Control* current = control->parent; current != NULL; current = current->parent) current->minimum_size_valid = false;
    ui_control_queue_layout(control);
}

void ui_control_set_h_size_flags(Control* control, SizeFlags flags) {
    control->h_size_flags = flags;
    ui_control_queue_layout(control);
}

void ui_control_set_v_size_flags(Control* control, SizeFlags flags) {
    control->v_size_flags = flags;
    ui_control_queue_layout(control);
}

void ui_control_set_stretch_ratio(Control* control, float ratio) {
    control->stretch_ratio = f32_max(ratio, 0.0f);
    ui_control_queue_layout(control);
}

void ui_control_set_h_grow_direction(Control* control, GrowDirection direction) {
    control->h_grow = direction;
    ui_control_queue_layout(control);
}

void ui_control_set_v_grow_direction(Control* control, GrowDirection direction) {
    control->v_grow = direction;
    ui_control_queue_layout(control);
}

void ui_control_set_visible(Control* control, bool visible) {
    if (control->visible == visible) return;
    control->visible = visible;
    ui_control_update_minimum_size(control);
}

void ui_control_set_clip_contents(Control* control, bool clip) { control->clip_contents = clip; }

void ui_control_set_modulate(Control* control, Color4 modulate) { control->modulate = modulate; }

Color4 ui_control_effective_modulate(const Control* control) {
    Color4 result = control->modulate;
    for (const Control* current = control->parent; current != NULL; current = current->parent) {
        result = ui_color_modulated(result, current->modulate);
    }
    return result;
}

void ui_control_set_hovered(Control* control, bool hovered) { control->hovered = hovered; }

bool ui_control_has_focus(const Control* control) { return control->ui != NULL && control->ui->focus == control; }

void ui_control_set_theme(Control* control, Theme* theme) {
    if (control->theme != NULL && control->theme != theme) ui_theme_free(control->theme);
    control->theme = theme;
    ui_control_update_minimum_size(control);
}

void ui_control_set_theme_type_variation(Control* control, StrView type) {
    ui_str_assign(&control->theme_type_variation, type);
    ui_control_update_minimum_size(control);
}

void ui_control_add_color_override(Control* control, StrView name, Color4 value) {
    ui_strmap_set(&control->color_overrides, name, &value);
    ui_control_update_minimum_size(control);
}

void ui_control_add_constant_override(Control* control, StrView name, float value) {
    ui_strmap_set(&control->constant_overrides, name, &value);
    ui_control_update_minimum_size(control);
}

const StyleBox* ui_control_theme_stylebox(const Control* control, StrView name) {
    const StyleBox* found = find_in_themes(control, UI_THEME_STYLEBOX, name);
    return found == NULL ? &fallback_stylebox : found;
}

Color4 ui_control_theme_color(const Control* control, StrView name) {
    const Color4* override_found = ui_strmap_find(&control->color_overrides, name);
    if (override_found != NULL) return *override_found;
    const Color4* found = find_in_themes(control, UI_THEME_COLOR, name);
    return found == NULL ? ui_white() : *found;
}

float ui_control_theme_constant(const Control* control, StrView name) {
    const float* override_found = ui_strmap_find(&control->constant_overrides, name);
    if (override_found != NULL) return *override_found;
    const float* found = find_in_themes(control, UI_THEME_CONSTANT, name);
    return found == NULL ? 0.0f : *found;
}

float ui_control_theme_font_size(const Control* control, StrView name) {
    const float* found = find_in_themes(control, UI_THEME_FONT_SIZE, name);
    return found == NULL ? 16.0f : *found;
}

Canvas* ui_control_canvas(const Control* control) { return control->ui->canvas; }

void ui_control_queue_layout(Control* control) {
    if (control->ui != NULL) ui_context_queue_layout(control->ui);
}

void ui_control_collect_theme_types(const Control* control, ThemeTypes* out) {
    (void)control;
    ui_theme_types_add(out, UI_SV("Control"));
}

Vec2 ui_control_get_minimum_size(const Control* control) {
    (void)control;
    return ui_vec2(0.0f, 0.0f);
}

void ui_control_draw(const Control* control, Canvas* canvas) {
    (void)control;
    (void)canvas;
}

void ui_control_layout_children(Control* control) {
    for (size_t index = 0; index < control->children.len; ++index) apply_anchors(control->children.data[index], control->size);
}

void ui_control_resized(Control* control) { (void)control; }

void ui_context_init(Context* context, Canvas* canvas) {
    context->canvas = canvas;
    context->root = xalloc(sizeof(Control));
    ui_control_init(context->root, &ui_control_vtable);
    context->theme = ui_default_theme();
    context->focus = NULL;
    context->viewport_rect = ui_rect2(ui_vec2(0.0f, 0.0f), ui_vec2(0.0f, 0.0f));
    context->layout_dirty = true;
    attach(context->root, NULL, context);
}

void ui_context_destroy(Context* context) {
    ui_control_free(context->root);
    ui_theme_free(context->theme);
}

void ui_context_set_focus(Context* context, Control* control) { context->focus = control; }

void ui_context_set_viewport_rect(Context* context, Rect2 rect) {
    if (ui_rect2_equal(&context->viewport_rect, &rect)) return;
    context->viewport_rect = rect;
    ui_context_queue_layout(context);
}

void ui_context_queue_layout(Context* context) { context->layout_dirty = true; }

/// Lays out `control`, then its visible children.
static void layout(Control* control) {
    control->vt->layout_children(control);
    for (size_t index = 0; index < control->children.len; ++index) {
        Control* child = control->children.data[index];
        if (child->visible) layout(child);
    }
}

void ui_context_update(Context* context) {
    ui_control_set_rect(context->root, context->viewport_rect);
    if (!context->layout_dirty) return;
    context->layout_dirty = false;
    layout(context->root);
}

/// Paints `control` and its subtree with `origin` as its parent's global position.
static void paint(Context* context, Control* control, Vec2 origin) {
    if (!control->visible) return;
    Canvas* target = context->canvas;
    Vec2 position = ui_vec2_add(origin, control->position);
    ui_canvas_push_offset(target, control->position);
    if (control->clip_contents) target->vt->push_clip(target, ui_rect2(position, control->size));
    control->vt->draw(control, target);
    for (size_t index = 0; index < control->children.len; ++index) paint(context, control->children.data[index], position);
    if (control->clip_contents) target->vt->pop_clip(target);
    ui_canvas_pop_offset(target);
}

void ui_context_draw(Context* context) { paint(context, context->root, ui_vec2(0.0f, 0.0f)); }
