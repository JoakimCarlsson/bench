#ifndef BENCH_UI_CONTAINER_H
#define BENCH_UI_CONTAINER_H

#include <stdint.h>

#include "ui_control.h"

/// The containers: controls that position their visible children themselves.
/// Each class is a struct embedding `Control` first, with its own vtable. The
/// shared `layout_children` of a container calls its `sort_children` slot, and
/// `Container`-level behaviour is the shared functions in `ui_container.c`.

/// A box container laid out top to bottom or left to right.
typedef struct {
    Control base;
    bool vertical;
} BoxContainer;

/// Insets its children by the themed `margin_*` constants.
typedef struct {
    Control base;
} MarginContainer;

/// Centres each child at its minimum size.
typedef struct {
    Control base;
} CenterContainer;

/// Draws the themed panel style box and insets its children by its content margins.
typedef struct {
    Control base;
} PanelContainer;

/// Lays children out in a grid with a fixed column count.
typedef struct {
    Control base;
    uint32_t columns;
} GridContainer;

/// Shows one of its children under a bar of titled tabs.
typedef struct {
    Control base;
    ARRAY_OF(OwnedStr) titles;
    size_t current;
} TabContainer;

/// Places `child` in `rect`, shrinking and aligning it by its size flags.
void ui_container_fit_child_in_rect(Control* child, Rect2 rect);

/// Adds a column that lays its children out top to bottom.
BoxContainer* ui_vbox_container_add(Control* parent);

/// Adds a row that lays its children out left to right.
BoxContainer* ui_hbox_container_add(Control* parent);

/// Adds a margin container.
MarginContainer* ui_margin_container_add(Control* parent);

/// Adds a centring container.
CenterContainer* ui_center_container_add(Control* parent);

/// Adds a panel container.
PanelContainer* ui_panel_container_add(Control* parent);

/// Adds a grid container with one column.
GridContainer* ui_grid_container_add(Control* parent);

/// Sets the column count of `grid`, at least one.
void ui_grid_container_set_columns(GridContainer* grid, uint32_t columns);

/// Adds a tab container.
TabContainer* ui_tab_container_add(Control* parent);

/// Sets the title of tab `index`.
void ui_tab_container_set_tab_title(TabContainer* tabs, size_t index, StrView title);

/// Shows tab `index`, clamped to the last one.
void ui_tab_container_set_current_tab(TabContainer* tabs, size_t index);

#endif
