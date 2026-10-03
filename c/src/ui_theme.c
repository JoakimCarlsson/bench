#include "ui_theme.h"

#include <stdlib.h>

#include "alloc.h"

static const Color4 surface = { 0.12f, 0.13f, 0.15f, 0.94f };
static const Color4 surface_raised = { 0.17f, 0.19f, 0.22f, 1.0f };
static const Color4 surface_hover = { 0.22f, 0.25f, 0.29f, 1.0f };
static const Color4 surface_pressed = { 0.10f, 0.11f, 0.13f, 1.0f };
static const Color4 surface_sunken = { 0.07f, 0.08f, 0.09f, 1.0f };
static const Color4 outline = { 0.30f, 0.33f, 0.38f, 1.0f };
static const Color4 accent = { 0.29f, 0.56f, 0.89f, 1.0f };
static const Color4 accent_dim = { 0.29f, 0.56f, 0.89f, 0.35f };
static const Color4 text = { 0.88f, 0.90f, 0.93f, 1.0f };
static const Color4 text_dim = { 0.58f, 0.61f, 0.66f, 1.0f };
static const Color4 text_disabled = { 0.40f, 0.42f, 0.45f, 1.0f };
static const Color4 clear = { 0.0f, 0.0f, 0.0f, 0.0f };

static const float base_font_size = 16.0f;
static const float base_radius = 4.0f;

/// The entry of one theme type: its base type name and one map per item kind.
typedef struct {
    OwnedStr base;
    StrMap items[UI_THEME_ITEM_KINDS];
} TypeEntry;

static const size_t item_sizes[UI_THEME_ITEM_KINDS] = { sizeof(StyleBox), sizeof(Color4), sizeof(float), sizeof(float) };

/// Margins with the same `amount` on every side.
static Margins uniform(float amount) { return (Margins){ amount, amount, amount, amount }; }

/// Margins with `horizontal` on the left and right and `vertical` on top and bottom.
static Margins padded(float horizontal, float vertical) { return (Margins){ horizontal, vertical, horizontal, vertical }; }

/// A text-field frame: sunken, outlined, with the given content margins.
static StyleBox edit_style(Margins content) {
    StyleBox style = ui_flat_style(surface_sunken, base_radius, content);
    style.border_width = 1.0f;
    style.border_color = outline;
    return style;
}

/// A button frame of `color`, outlined, with the given content margins.
static StyleBox button_style(Color4 color, Margins content) {
    StyleBox style = ui_flat_style(color, base_radius, content);
    style.border_width = 1.0f;
    style.border_color = outline;
    return style;
}

/// Prepares the empty maps of a freshly created entry.
static void type_entry_init(TypeEntry* entry) {
    for (size_t kind = 0; kind < UI_THEME_ITEM_KINDS; ++kind) ui_strmap_init(&entry->items[kind], item_sizes[kind]);
}

/// Releases the strings and maps of an entry; used as the destructor of the type map.
static void type_entry_destroy(void* value) {
    TypeEntry* entry = value;
    ui_str_free(&entry->base);
    for (size_t kind = 0; kind < UI_THEME_ITEM_KINDS; ++kind) ui_strmap_clear(&entry->items[kind], NULL);
}

/// The entry for `type`, or null.
static TypeEntry* find_entry(const Theme* theme, StrView type) { return ui_strmap_find(&theme->types, type); }

/// The entry for `type`, created when absent.
static TypeEntry* entry_for(Theme* theme, StrView type) {
    bool created = false;
    TypeEntry* entry = ui_strmap_emplace(&theme->types, type, &created);
    if (created) type_entry_init(entry);
    return entry;
}

/// Stores `value` as the item of `kind` under `key`.
static void set_item(Theme* theme, ThemeItemKind kind, ThemeKey key, const void* value) {
    ui_strmap_set(&entry_for(theme, key.type)->items[kind], key.name, value);
}

StyleBox ui_stylebox_default(void) {
    StyleBox style = { 0 };
    style.kind = UI_STYLEBOX_EMPTY;
    style.color = ui_white();
    style.border_color = ui_white();
    return style;
}

Rect2 ui_stylebox_content_rect(const StyleBox* style, Rect2 rect) {
    Vec2 inset = ui_vec2(style->content_margins.left, style->content_margins.top);
    return ui_rect2(ui_vec2_add(rect.position, inset),
            ui_vec2_max(ui_vec2_sub(rect.size, ui_margins_size(style->content_margins)), ui_vec2(0.0f, 0.0f)));
}

void ui_stylebox_draw(const StyleBox* style, Canvas* canvas, Rect2 rect, Color4 modulate) {
    if (style->kind == UI_STYLEBOX_EMPTY) return;
    Rect2 target = ui_rect2(ui_vec2_sub(rect.position, ui_vec2(style->expand_margins.left, style->expand_margins.top)),
            ui_vec2_add(rect.size, ui_margins_size(style->expand_margins)));
    if (style->color.a > 0.0f) canvas->vt->fill_rect(canvas, target, ui_color_modulated(style->color, modulate), style->corner_radius);
    if (style->border_width > 0.0f && style->border_color.a > 0.0f) {
        Stroke stroke = { style->border_width, style->corner_radius };
        canvas->vt->stroke_rect(canvas, target, ui_color_modulated(style->border_color, modulate), stroke);
    }
}

StyleBox ui_flat_style(Color4 color, float corner_radius, Margins content) {
    StyleBox style = ui_stylebox_default();
    style.kind = UI_STYLEBOX_FLAT;
    style.color = color;
    style.corner_radius = corner_radius;
    style.content_margins = content;
    return style;
}

StyleBox ui_empty_style(Margins content) {
    StyleBox style = ui_stylebox_default();
    style.kind = UI_STYLEBOX_EMPTY;
    style.content_margins = content;
    return style;
}

Theme* ui_theme_new(void) {
    Theme* theme = xalloc(sizeof(Theme));
    ui_strmap_init(&theme->types, sizeof(TypeEntry));
    return theme;
}

void ui_theme_free(Theme* theme) {
    ui_strmap_clear(&theme->types, type_entry_destroy);
    free(theme);
}

void ui_theme_set_stylebox(Theme* theme, ThemeKey key, StyleBox value) { set_item(theme, UI_THEME_STYLEBOX, key, &value); }

void ui_theme_set_color(Theme* theme, ThemeKey key, Color4 value) { set_item(theme, UI_THEME_COLOR, key, &value); }

void ui_theme_set_constant(Theme* theme, ThemeKey key, float value) { set_item(theme, UI_THEME_CONSTANT, key, &value); }

void ui_theme_set_font_size(Theme* theme, ThemeKey key, float value) { set_item(theme, UI_THEME_FONT_SIZE, key, &value); }

void ui_theme_set_type_base(Theme* theme, ThemeKey inheritance) {
    ui_str_assign(&entry_for(theme, inheritance.type)->base, inheritance.name);
}

const void* ui_theme_item(const Theme* theme, ThemeItemKind kind, ThemeKey key) {
    const TypeEntry* found = find_entry(theme, key.type);
    if (found == NULL) return NULL;
    return ui_strmap_find(&found->items[kind], key.name);
}

StrView ui_theme_type_base(const Theme* theme, StrView type) {
    const TypeEntry* found = find_entry(theme, type);
    if (found == NULL) return (StrView){ NULL, 0 };
    return ui_str_view(&found->base);
}

Theme* ui_default_theme(void) {
    Theme* theme = ui_theme_new();

    ui_theme_set_font_size(theme, UI_KEY("Control", "font_size"), base_font_size);
    ui_theme_set_color(theme, UI_KEY("Control", "font_color"), text);

    ui_theme_set_stylebox(theme, UI_KEY("Panel", "panel"), ui_flat_style(surface, base_radius, uniform(8.0f)));
    ui_theme_set_stylebox(theme, UI_KEY("PanelContainer", "panel"), ui_flat_style(surface, base_radius, uniform(8.0f)));

    ui_theme_set_color(theme, UI_KEY("Label", "font_color"), text);
    ui_theme_set_color(theme, UI_KEY("Label", "font_shadow_color"), clear);
    ui_theme_set_font_size(theme, UI_KEY("Label", "font_size"), base_font_size);
    ui_theme_set_constant(theme, UI_KEY("Label", "line_spacing"), 2.0f);

    ui_theme_set_constant(theme, UI_KEY("BoxContainer", "separation"), 6.0f);
    ui_theme_set_constant(theme, UI_KEY("GridContainer", "h_separation"), 6.0f);
    ui_theme_set_constant(theme, UI_KEY("GridContainer", "v_separation"), 6.0f);
    ui_theme_set_constant(theme, UI_KEY("MarginContainer", "margin_left"), 0.0f);
    ui_theme_set_constant(theme, UI_KEY("MarginContainer", "margin_top"), 0.0f);
    ui_theme_set_constant(theme, UI_KEY("MarginContainer", "margin_right"), 0.0f);
    ui_theme_set_constant(theme, UI_KEY("MarginContainer", "margin_bottom"), 0.0f);

    StyleBox button_normal = button_style(surface_raised, padded(12.0f, 6.0f));
    StyleBox button_hover = button_normal;
    button_hover.color = surface_hover;
    StyleBox button_pressed = button_normal;
    button_pressed.color = surface_pressed;
    StyleBox button_disabled = button_normal;
    button_disabled.color = surface_sunken;
    button_disabled.border_color = ui_color_with_alpha(outline, 0.4f);
    StyleBox button_focus = ui_empty_style(padded(12.0f, 6.0f));
    button_focus.kind = UI_STYLEBOX_FLAT;
    button_focus.color = clear;
    button_focus.border_width = 1.0f;
    button_focus.border_color = accent;
    button_focus.corner_radius = base_radius;

    ui_theme_set_stylebox(theme, UI_KEY("Button", "normal"), button_normal);
    ui_theme_set_stylebox(theme, UI_KEY("Button", "hover"), button_hover);
    ui_theme_set_stylebox(theme, UI_KEY("Button", "pressed"), button_pressed);
    ui_theme_set_stylebox(theme, UI_KEY("Button", "disabled"), button_disabled);
    ui_theme_set_stylebox(theme, UI_KEY("Button", "focus"), button_focus);
    ui_theme_set_color(theme, UI_KEY("Button", "font_color"), text);
    ui_theme_set_color(theme, UI_KEY("Button", "font_hover_color"), text);
    ui_theme_set_color(theme, UI_KEY("Button", "font_pressed_color"), text);
    ui_theme_set_color(theme, UI_KEY("Button", "font_disabled_color"), text_disabled);
    ui_theme_set_font_size(theme, UI_KEY("Button", "font_size"), base_font_size);
    ui_theme_set_constant(theme, UI_KEY("Button", "h_separation"), 6.0f);

    ui_theme_set_stylebox(theme, UI_KEY("CheckBox", "normal"), ui_empty_style(padded(4.0f, 4.0f)));
    ui_theme_set_stylebox(theme, UI_KEY("CheckBox", "focus"), button_focus);
    ui_theme_set_color(theme, UI_KEY("CheckBox", "font_color"), text);
    ui_theme_set_color(theme, UI_KEY("CheckBox", "font_disabled_color"), text_disabled);
    ui_theme_set_color(theme, UI_KEY("CheckBox", "box_color"), surface_sunken);
    ui_theme_set_color(theme, UI_KEY("CheckBox", "box_border_color"), outline);
    ui_theme_set_color(theme, UI_KEY("CheckBox", "check_color"), accent);
    ui_theme_set_font_size(theme, UI_KEY("CheckBox", "font_size"), base_font_size);
    ui_theme_set_constant(theme, UI_KEY("CheckBox", "h_separation"), 8.0f);
    ui_theme_set_constant(theme, UI_KEY("CheckBox", "box_size"), 16.0f);

    ui_theme_set_stylebox(theme, UI_KEY("Slider", "slider"), ui_flat_style(surface_sunken, 3.0f, (Margins){ 0 }));
    ui_theme_set_stylebox(theme, UI_KEY("Slider", "grabber_area"), ui_flat_style(accent, 3.0f, (Margins){ 0 }));
    ui_theme_set_color(theme, UI_KEY("Slider", "grabber_color"), text);
    ui_theme_set_color(theme, UI_KEY("Slider", "grabber_hover_color"), accent);
    ui_theme_set_constant(theme, UI_KEY("Slider", "grabber_size"), 14.0f);
    ui_theme_set_constant(theme, UI_KEY("Slider", "thickness"), 6.0f);

    StyleBox edit_normal = edit_style(padded(8.0f, 5.0f));
    StyleBox edit_focus = edit_normal;
    edit_focus.border_color = accent;
    StyleBox edit_read_only = edit_normal;
    edit_read_only.color = surface;
    ui_theme_set_stylebox(theme, UI_KEY("LineEdit", "normal"), edit_normal);
    ui_theme_set_stylebox(theme, UI_KEY("LineEdit", "focus"), edit_focus);
    ui_theme_set_stylebox(theme, UI_KEY("LineEdit", "read_only"), edit_read_only);
    ui_theme_set_color(theme, UI_KEY("LineEdit", "font_color"), text);
    ui_theme_set_color(theme, UI_KEY("LineEdit", "placeholder_color"), text_disabled);
    ui_theme_set_color(theme, UI_KEY("LineEdit", "selection_color"), accent_dim);
    ui_theme_set_color(theme, UI_KEY("LineEdit", "caret_color"), text);
    ui_theme_set_font_size(theme, UI_KEY("LineEdit", "font_size"), base_font_size);
    ui_theme_set_constant(theme, UI_KEY("LineEdit", "minimum_width"), 80.0f);
    ui_theme_set_constant(theme, UI_KEY("LineEdit", "caret_width"), 1.0f);

    ui_theme_set_stylebox(theme, UI_KEY("TabContainer", "panel"), ui_flat_style(surface, base_radius, uniform(8.0f)));
    ui_theme_set_stylebox(theme, UI_KEY("TabContainer", "tab_selected"), ui_flat_style(surface, base_radius, padded(12.0f, 6.0f)));
    ui_theme_set_stylebox(theme, UI_KEY("TabContainer", "tab_unselected"), ui_flat_style(surface_sunken, base_radius, padded(12.0f, 6.0f)));
    ui_theme_set_color(theme, UI_KEY("TabContainer", "font_selected_color"), text);
    ui_theme_set_color(theme, UI_KEY("TabContainer", "font_unselected_color"), text_dim);
    ui_theme_set_font_size(theme, UI_KEY("TabContainer", "font_size"), base_font_size);
    ui_theme_set_constant(theme, UI_KEY("TabContainer", "h_separation"), 2.0f);

    return theme;
}

Theme* ui_inspector_theme(void) {
    Theme* theme = ui_theme_new();

    ui_theme_set_constant(theme, UI_KEY("BoxContainer", "separation"), 4.0f);
    ui_theme_set_constant(theme, UI_KEY("GridContainer", "h_separation"), 8.0f);
    ui_theme_set_constant(theme, UI_KEY("GridContainer", "v_separation"), 3.0f);
    ui_theme_set_font_size(theme, UI_KEY("Label", "font_size"), 14.0f);

    ui_theme_set_type_base(theme, UI_KEY("PropertyLabel", "Label"));
    ui_theme_set_color(theme, UI_KEY("PropertyLabel", "font_color"), text_dim);

    ui_theme_set_type_base(theme, UI_KEY("SectionHeader", "Button"));
    ui_theme_set_stylebox(theme, UI_KEY("SectionHeader", "normal"), button_style(surface_raised, padded(8.0f, 4.0f)));
    ui_theme_set_stylebox(theme, UI_KEY("SectionHeader", "hover"), button_style(surface_hover, padded(8.0f, 4.0f)));
    ui_theme_set_color(theme, UI_KEY("SectionHeader", "font_color"), accent);

    ui_theme_set_stylebox(theme, UI_KEY("LineEdit", "normal"), edit_style(padded(6.0f, 3.0f)));
    ui_theme_set_font_size(theme, UI_KEY("LineEdit", "font_size"), 14.0f);
    ui_theme_set_constant(theme, UI_KEY("LineEdit", "minimum_width"), 48.0f);

    ui_theme_set_constant(theme, UI_KEY("Slider", "grabber_size"), 12.0f);
    return theme;
}
