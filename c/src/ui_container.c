#include "ui_container.h"

#include <stdlib.h>

/// Growable array of rectangles.
typedef ARRAY_OF(Rect2) RectArray;

/// Column widths and row heights of a grid.
typedef struct {
    ARRAY_OF(float) columns;
    ARRAY_OF(float) rows;
} GridMetrics;

/// Appends `Container` and then `Control` to the theme types.
static void container_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("Container"));
    ui_control_collect_theme_types(control, out);
}

/// Appends `BoxContainer` and its bases to the theme types.
static void box_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("BoxContainer"));
    container_collect_theme_types(control, out);
}

/// Appends `VBoxContainer` and its bases to the theme types.
static void vbox_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("VBoxContainer"));
    box_collect_theme_types(control, out);
}

/// Appends `HBoxContainer` and its bases to the theme types.
static void hbox_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("HBoxContainer"));
    box_collect_theme_types(control, out);
}

/// Appends `MarginContainer` and its bases to the theme types.
static void margin_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("MarginContainer"));
    container_collect_theme_types(control, out);
}

/// Appends `CenterContainer` and its bases to the theme types.
static void center_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("CenterContainer"));
    container_collect_theme_types(control, out);
}

/// Appends `PanelContainer` and its bases to the theme types.
static void panel_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("PanelContainer"));
    container_collect_theme_types(control, out);
}

/// Appends `GridContainer` and its bases to the theme types.
static void grid_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("GridContainer"));
    container_collect_theme_types(control, out);
}

/// Appends `TabContainer` and its bases to the theme types.
static void tab_collect_theme_types(const Control* control, ThemeTypes* out) {
    ui_theme_types_add(out, UI_SV("TabContainer"));
    container_collect_theme_types(control, out);
}

/// The visible children of `container`, in order, in a fresh array the caller frees.
static ControlArray sortable_children(const Control* container) {
    ControlArray result = { 0 };
    ARRAY_RESERVE(result, container->children.len);
    for (size_t index = 0; index < container->children.len; ++index) {
        Control* child = container->children.data[index];
        if (child->visible) ARRAY_PUSH(result, child);
    }
    return result;
}

/// Places every sortable child by calling the `sort_children` slot.
static void container_layout_children(Control* control) { control->vt->sort_children(control); }

/// The largest minimum size among the visible children.
static Vec2 largest_child_minimum(const Control* container) {
    ControlArray entries = sortable_children(container);
    Vec2 result = ui_vec2(0.0f, 0.0f);
    for (size_t index = 0; index < entries.len; ++index) {
        result = ui_vec2_max(result, ui_control_combined_minimum_size(entries.data[index]));
    }
    ARRAY_FREE(entries);
    return result;
}

/// The component of `value` along the main axis of a box.
static float axis_of(Vec2 value, bool vertical) { return vertical ? value.y : value.x; }

/// The size flags of `child` along the main axis of a box.
static SizeFlags axis_flags(const Control* child, bool vertical) { return vertical ? child->v_size_flags : child->h_size_flags; }

/// Positions one axis of `target` inside `rect` for a child of minimum extent `minimum` and the given flags.
static void fit_axis(float* position, float* extent, float rect_position, float rect_extent, float minimum, SizeFlags flags) {
    *position = rect_position;
    *extent = rect_extent;
    if (ui_has_flag(flags, UI_SIZE_FILL)) return;
    float width = f32_min(f32_max(minimum, 0.0f), rect_extent);
    *extent = f32_max(width, minimum);
    if (ui_has_flag(flags, UI_SIZE_SHRINK_END)) {
        *position += rect_extent - *extent;
    } else if (ui_has_flag(flags, UI_SIZE_SHRINK_CENTER)) {
        *position += (rect_extent - *extent) * 0.5f;
    }
}

void ui_container_fit_child_in_rect(Control* child, Rect2 rect) {
    Vec2 minimum = ui_control_combined_minimum_size(child);
    Rect2 target = rect;
    fit_axis(&target.position.x, &target.size.x, rect.position.x, rect.size.x, minimum.x, child->h_size_flags);
    fit_axis(&target.position.y, &target.size.y, rect.position.y, rect.size.y, minimum.y, child->v_size_flags);
    ui_control_set_rect(child, target);
}

/// The minimum size of a box: children summed along its axis with separations, the widest across it.
static Vec2 box_get_minimum_size(const Control* control) {
    bool vertical = ((const BoxContainer*)control)->vertical;
    ControlArray entries = sortable_children(control);
    float separation = UI_CONSTANT(control, "separation");
    Vec2 result = ui_vec2(0.0f, 0.0f);
    for (size_t index = 0; index < entries.len; ++index) {
        Vec2 minimum = ui_control_combined_minimum_size(entries.data[index]);
        if (vertical) {
            result.x = f32_max(result.x, minimum.x);
            result.y += minimum.y;
            if (index + 1 < entries.len) result.y += separation;
        } else {
            result.y = f32_max(result.y, minimum.y);
            result.x += minimum.x;
            if (index + 1 < entries.len) result.x += separation;
        }
    }
    ARRAY_FREE(entries);
    return result;
}

/// Places the children of a box, sharing spare space among the expanding ones by stretch ratio.
static void box_sort_children(Control* control) {
    bool vertical = ((BoxContainer*)control)->vertical;
    ControlArray entries = sortable_children(control);
    if (entries.len == 0) {
        ARRAY_FREE(entries);
        return;
    }
    float separation = UI_CONSTANT(control, "separation");
    float extent = axis_of(control->size, vertical);
    float cross = vertical ? control->size.x : control->size.y;

    float used = separation * (float)(entries.len - 1);
    float total_ratio = 0.0f;
    for (size_t index = 0; index < entries.len; ++index) {
        Control* child = entries.data[index];
        Vec2 minimum = ui_control_combined_minimum_size(child);
        used += axis_of(minimum, vertical);
        SizeFlags flags = axis_flags(child, vertical);
        if (ui_has_flag(flags, UI_SIZE_EXPAND)) total_ratio += child->stretch_ratio;
    }

    float remaining = f32_max(extent - used, 0.0f);
    float cursor = 0.0f;
    for (size_t index = 0; index < entries.len; ++index) {
        Control* child = entries.data[index];
        Vec2 minimum = ui_control_combined_minimum_size(child);
        float length = axis_of(minimum, vertical);
        SizeFlags flags = axis_flags(child, vertical);
        if (ui_has_flag(flags, UI_SIZE_EXPAND) && total_ratio > 0.0f) length += remaining * (child->stretch_ratio / total_ratio);
        Rect2 slot = vertical ? ui_rect2(ui_vec2(0.0f, cursor), ui_vec2(cross, length))
                              : ui_rect2(ui_vec2(cursor, 0.0f), ui_vec2(length, cross));
        ui_container_fit_child_in_rect(child, slot);
        cursor += length;
        if (index + 1 < entries.len) cursor += separation;
    }
    ARRAY_FREE(entries);
}

/// The four themed margins of a margin container.
static Margins margin_margins(const Control* control) {
    Margins margins = { 0 };
    margins.left = UI_CONSTANT(control, "margin_left");
    margins.top = UI_CONSTANT(control, "margin_top");
    margins.right = UI_CONSTANT(control, "margin_right");
    margins.bottom = UI_CONSTANT(control, "margin_bottom");
    return margins;
}

/// The largest child minimum plus the margins.
static Vec2 margin_get_minimum_size(const Control* control) {
    return ui_vec2_add(largest_child_minimum(control), ui_margins_size(margin_margins(control)));
}

/// Fits every child into the rectangle inside the margins.
static void margin_sort_children(Control* control) {
    Margins margin = margin_margins(control);
    Rect2 slot = ui_rect2(ui_vec2(margin.left, margin.top),
            ui_vec2_max(ui_vec2_sub(control->size, ui_margins_size(margin)), ui_vec2(0.0f, 0.0f)));
    ControlArray entries = sortable_children(control);
    for (size_t index = 0; index < entries.len; ++index) ui_container_fit_child_in_rect(entries.data[index], slot);
    ARRAY_FREE(entries);
}

/// The largest child minimum.
static Vec2 center_get_minimum_size(const Control* control) { return largest_child_minimum(control); }

/// Centres each child at its minimum size.
static void center_sort_children(Control* control) {
    ControlArray entries = sortable_children(control);
    for (size_t index = 0; index < entries.len; ++index) {
        Control* child = entries.data[index];
        Vec2 minimum = ui_control_combined_minimum_size(child);
        Vec2 position = ui_vec2_scale(ui_vec2_sub(control->size, minimum), 0.5f);
        ui_control_set_rect(child, ui_rect2(position, minimum));
    }
    ARRAY_FREE(entries);
}

/// The largest child minimum plus the content margins of the panel style.
static Vec2 panel_get_minimum_size(const Control* control) {
    return ui_vec2_add(largest_child_minimum(control), ui_margins_size(UI_STYLEBOX(control, "panel")->content_margins));
}

/// Draws the themed panel style box.
static void panel_draw(const Control* control, Canvas* canvas) {
    ui_stylebox_draw(UI_STYLEBOX(control, "panel"), canvas, ui_rect2(ui_vec2(0.0f, 0.0f), control->size), ui_control_effective_modulate(control));
}

/// Fits every child into the content rectangle of the panel style.
static void panel_sort_children(Control* control) {
    Rect2 slot = ui_stylebox_content_rect(UI_STYLEBOX(control, "panel"), ui_rect2(ui_vec2(0.0f, 0.0f), control->size));
    ControlArray entries = sortable_children(control);
    for (size_t index = 0; index < entries.len; ++index) ui_container_fit_child_in_rect(entries.data[index], slot);
    ARRAY_FREE(entries);
}

/// The widest cell of each column and the tallest of each row of `entries`, in arrays the caller frees.
static GridMetrics grid_measure(const GridContainer* grid, const ControlArray* entries) {
    GridMetrics metrics = { { 0 }, { 0 } };
    ARRAY_RESIZE(metrics.columns, grid->columns);
    for (size_t index = 0; index < entries->len; ++index) {
        size_t column = index % grid->columns;
        size_t row = index / grid->columns;
        if (metrics.rows.len <= row) ARRAY_PUSH(metrics.rows, 0.0f);
        Vec2 minimum = ui_control_combined_minimum_size(entries->data[index]);
        metrics.columns.data[column] = f32_max(metrics.columns.data[column], minimum.x);
        metrics.rows.data[row] = f32_max(metrics.rows.data[row], minimum.y);
    }
    return metrics;
}

/// Releases the arrays of `metrics`.
static void grid_metrics_free(GridMetrics* metrics) {
    ARRAY_FREE(metrics->columns);
    ARRAY_FREE(metrics->rows);
}

/// The size of a grid from its metrics and themed separations.
static Vec2 grid_extent(const Control* control, const GridMetrics* metrics) {
    float horizontal = UI_CONSTANT(control, "h_separation");
    float vertical = UI_CONSTANT(control, "v_separation");
    Vec2 result = ui_vec2(0.0f, 0.0f);
    for (size_t index = 0; index < metrics->columns.len; ++index) result.x += metrics->columns.data[index];
    for (size_t index = 0; index < metrics->rows.len; ++index) result.y += metrics->rows.data[index];
    if (metrics->columns.len != 0) result.x += horizontal * (float)(metrics->columns.len - 1);
    if (metrics->rows.len != 0) result.y += vertical * (float)(metrics->rows.len - 1);
    return result;
}

/// The minimum size of a grid.
static Vec2 grid_get_minimum_size(const Control* control) {
    ControlArray entries = sortable_children(control);
    GridMetrics metrics = grid_measure((const GridContainer*)control, &entries);
    Vec2 result = grid_extent(control, &metrics);
    grid_metrics_free(&metrics);
    ARRAY_FREE(entries);
    return result;
}

/// Adds an equal share of `extra` to every entry of `cells`.
static void share_extra(float* cells, size_t count, float extra) {
    if (count == 0 || !(extra > 0.0f)) return;
    float share = extra / (float)count;
    for (size_t index = 0; index < count; ++index) cells[index] += share;
}

/// Places the children of a grid in cells grown to fill the spare size.
static void grid_sort_children(Control* control) {
    const GridContainer* grid = (const GridContainer*)control;
    ControlArray entries = sortable_children(control);
    if (entries.len == 0) {
        ARRAY_FREE(entries);
        return;
    }
    GridMetrics metrics = grid_measure(grid, &entries);
    float horizontal = UI_CONSTANT(control, "h_separation");
    float vertical = UI_CONSTANT(control, "v_separation");

    Vec2 minimum = grid_get_minimum_size(control);
    Vec2 extra = ui_vec2_max(ui_vec2_sub(control->size, minimum), ui_vec2(0.0f, 0.0f));
    share_extra(metrics.columns.data, metrics.columns.len, extra.x);
    share_extra(metrics.rows.data, metrics.rows.len, extra.y);

    for (size_t index = 0; index < entries.len; ++index) {
        size_t column = index % grid->columns;
        size_t row = index / grid->columns;
        float x = 0.0f;
        for (size_t i = 0; i < column; ++i) x += metrics.columns.data[i] + horizontal;
        float y = 0.0f;
        for (size_t i = 0; i < row; ++i) y += metrics.rows.data[i] + vertical;
        ui_container_fit_child_in_rect(entries.data[index], ui_rect2(ui_vec2(x, y), ui_vec2(metrics.columns.data[column], metrics.rows.data[row])));
    }
    grid_metrics_free(&metrics);
    ARRAY_FREE(entries);
}

/// The title of tab `index`, or an empty view.
static StrView tab_title(const TabContainer* tabs, size_t index) {
    if (index >= tabs->titles.len) return (StrView){ NULL, 0 };
    return ui_str_view(&tabs->titles.data[index]);
}

/// The height of the tab bar.
static float tab_bar_height(const Control* control) {
    float font_size = UI_FONT_SIZE(control, "font_size");
    Margins content = UI_STYLEBOX(control, "tab_selected")->content_margins;
    const Canvas* canvas = ui_control_canvas(control);
    return canvas->vt->metrics(canvas, font_size).line_height + ui_margins_size(content).y;
}

/// The rectangle of every tab in the bar, in an array the caller frees.
static RectArray tab_rects(const Control* control) {
    const TabContainer* tabs = (const TabContainer*)control;
    float font_size = UI_FONT_SIZE(control, "font_size");
    float separation = UI_CONSTANT(control, "h_separation");
    Margins content = UI_STYLEBOX(control, "tab_selected")->content_margins;
    float height = tab_bar_height(control);
    const Canvas* target = ui_control_canvas(control);

    RectArray result = { 0 };
    float x = 0.0f;
    size_t count = control->children.len;
    for (size_t index = 0; index < count; ++index) {
        float width = target->vt->measure_text(target, tab_title(tabs, index), font_size).x + ui_margins_size(content).x;
        Rect2 tab = ui_rect2(ui_vec2(x, 0.0f), ui_vec2(width, height));
        ARRAY_PUSH(result, tab);
        x += width + separation;
    }
    return result;
}

/// The minimum size of a tab container: the larger of the content and the bar widths.
static Vec2 tab_get_minimum_size(const Control* control) {
    Vec2 content = largest_child_minimum(control);
    Margins panel = UI_STYLEBOX(control, "panel")->content_margins;
    content = ui_vec2_add(content, ui_margins_size(panel));

    float bar_width = 0.0f;
    RectArray tabs = tab_rects(control);
    for (size_t index = 0; index < tabs.len; ++index) bar_width = f32_max(bar_width, ui_rect2_right(&tabs.data[index]));
    ARRAY_FREE(tabs);
    return ui_vec2(f32_max(content.x, bar_width), content.y + tab_bar_height(control));
}

/// Shows the current child and hides the others.
static void tab_sync_visibility(Control* control) {
    const TabContainer* tabs = (const TabContainer*)control;
    for (size_t index = 0; index < control->children.len; ++index) {
        ui_control_set_visible(control->children.data[index], index == tabs->current);
    }
}

/// The rectangle under the tab bar.
static Rect2 tab_body(const Control* control) {
    float bar = tab_bar_height(control);
    return ui_rect2(ui_vec2(0.0f, bar), ui_vec2(control->size.x, f32_max(control->size.y - bar, 0.0f)));
}

/// Shows the current tab and fits its child into the panel content rectangle.
static void tab_sort_children(Control* control) {
    TabContainer* tabs = (TabContainer*)control;
    size_t count = control->children.len;
    if (count == 0) return;
    tabs->current = tabs->current < count - 1 ? tabs->current : count - 1;
    tab_sync_visibility(control);

    Rect2 body = tab_body(control);
    Rect2 slot = ui_stylebox_content_rect(UI_STYLEBOX(control, "panel"), body);

    for (size_t index = 0; index < count; ++index) {
        if (index == tabs->current) ui_container_fit_child_in_rect(control->children.data[index], slot);
    }
}

/// Draws the panel under the bar and the bar of tabs.
static void tab_draw(const Control* control, Canvas* canvas) {
    const TabContainer* tabs = (const TabContainer*)control;
    Color4 tint = ui_control_effective_modulate(control);
    Rect2 body = tab_body(control);
    ui_stylebox_draw(UI_STYLEBOX(control, "panel"), canvas, body, tint);

    float font_size = UI_FONT_SIZE(control, "font_size");
    RectArray rects = tab_rects(control);
    for (size_t index = 0; index < rects.len; ++index) {
        bool selected = index == tabs->current;
        StrView style = selected ? UI_SV("tab_selected") : UI_SV("tab_unselected");
        ui_stylebox_draw(ui_control_theme_stylebox(control, style), canvas, rects.data[index], tint);

        Color4 color = selected ? UI_COLOR(control, "font_selected_color") : UI_COLOR(control, "font_unselected_color");
        Rect2 content = ui_stylebox_content_rect(ui_control_theme_stylebox(control, style), rects.data[index]);
        StrView title = tab_title(tabs, index);
        Vec2 measured = canvas->vt->measure_text(canvas, title, font_size);
        Vec2 origin = ui_vec2(content.position.x, content.position.y + (content.size.y - measured.y) * 0.5f);
        canvas->vt->draw_text(canvas, origin, title, font_size, ui_color_modulated(color, tint));
    }
    ARRAY_FREE(rects);
}

/// Releases the tab titles, then the base control.
static void tab_destroy(Control* control) {
    TabContainer* tabs = (TabContainer*)control;
    for (size_t index = 0; index < tabs->titles.len; ++index) ui_str_free(&tabs->titles.data[index]);
    ARRAY_FREE(tabs->titles);
    ui_control_destroy(control);
}

static const ControlVTable vbox_vtable = {
    .destroy = ui_control_destroy,
    .collect_theme_types = vbox_collect_theme_types,
    .get_minimum_size = box_get_minimum_size,
    .draw = ui_control_draw,
    .layout_children = container_layout_children,
    .resized = ui_control_resized,
    .sort_children = box_sort_children,
};

static const ControlVTable hbox_vtable = {
    .destroy = ui_control_destroy,
    .collect_theme_types = hbox_collect_theme_types,
    .get_minimum_size = box_get_minimum_size,
    .draw = ui_control_draw,
    .layout_children = container_layout_children,
    .resized = ui_control_resized,
    .sort_children = box_sort_children,
};

static const ControlVTable margin_vtable = {
    .destroy = ui_control_destroy,
    .collect_theme_types = margin_collect_theme_types,
    .get_minimum_size = margin_get_minimum_size,
    .draw = ui_control_draw,
    .layout_children = container_layout_children,
    .resized = ui_control_resized,
    .sort_children = margin_sort_children,
};

static const ControlVTable center_vtable = {
    .destroy = ui_control_destroy,
    .collect_theme_types = center_collect_theme_types,
    .get_minimum_size = center_get_minimum_size,
    .draw = ui_control_draw,
    .layout_children = container_layout_children,
    .resized = ui_control_resized,
    .sort_children = center_sort_children,
};

static const ControlVTable panel_vtable = {
    .destroy = ui_control_destroy,
    .collect_theme_types = panel_collect_theme_types,
    .get_minimum_size = panel_get_minimum_size,
    .draw = panel_draw,
    .layout_children = container_layout_children,
    .resized = ui_control_resized,
    .sort_children = panel_sort_children,
};

static const ControlVTable grid_vtable = {
    .destroy = ui_control_destroy,
    .collect_theme_types = grid_collect_theme_types,
    .get_minimum_size = grid_get_minimum_size,
    .draw = ui_control_draw,
    .layout_children = container_layout_children,
    .resized = ui_control_resized,
    .sort_children = grid_sort_children,
};

static const ControlVTable tab_vtable = {
    .destroy = tab_destroy,
    .collect_theme_types = tab_collect_theme_types,
    .get_minimum_size = tab_get_minimum_size,
    .draw = tab_draw,
    .layout_children = container_layout_children,
    .resized = ui_control_resized,
    .sort_children = tab_sort_children,
};

BoxContainer* ui_vbox_container_add(Control* parent) {
    BoxContainer* box = (BoxContainer*)ui_control_add_new(parent, sizeof(BoxContainer), &vbox_vtable);
    box->vertical = true;
    return box;
}

BoxContainer* ui_hbox_container_add(Control* parent) {
    BoxContainer* box = (BoxContainer*)ui_control_add_new(parent, sizeof(BoxContainer), &hbox_vtable);
    box->vertical = false;
    return box;
}

MarginContainer* ui_margin_container_add(Control* parent) {
    return (MarginContainer*)ui_control_add_new(parent, sizeof(MarginContainer), &margin_vtable);
}

CenterContainer* ui_center_container_add(Control* parent) {
    return (CenterContainer*)ui_control_add_new(parent, sizeof(CenterContainer), &center_vtable);
}

PanelContainer* ui_panel_container_add(Control* parent) {
    return (PanelContainer*)ui_control_add_new(parent, sizeof(PanelContainer), &panel_vtable);
}

GridContainer* ui_grid_container_add(Control* parent) {
    GridContainer* grid = (GridContainer*)ui_control_add_new(parent, sizeof(GridContainer), &grid_vtable);
    grid->columns = 1;
    return grid;
}

void ui_grid_container_set_columns(GridContainer* grid, uint32_t columns) {
    uint32_t target = columns > 1u ? columns : 1u;
    if (grid->columns == target) return;
    grid->columns = target;
    ui_control_update_minimum_size(&grid->base);
}

TabContainer* ui_tab_container_add(Control* parent) {
    return (TabContainer*)ui_control_add_new(parent, sizeof(TabContainer), &tab_vtable);
}

void ui_tab_container_set_tab_title(TabContainer* tabs, size_t index, StrView title) {
    if (tabs->titles.len <= index) ARRAY_RESIZE(tabs->titles, index + 1);
    ui_str_assign(&tabs->titles.data[index], title);
    ui_control_update_minimum_size(&tabs->base);
}

void ui_tab_container_set_current_tab(TabContainer* tabs, size_t index) {
    size_t count = tabs->base.children.len;
    if (count == 0) {
        tabs->current = 0;
        return;
    }
    size_t target = index < count - 1 ? index : count - 1;
    if (tabs->current == target) return;
    tabs->current = target;
    ui_control_queue_layout(&tabs->base);
}
