//! Containers: box, margin, centre, panel, grid and tab. Each is a `Widget`
//! whose `layout_children` is the engine's `sort_children`; the shared
//! `Container` behaviour is the free functions `sortable_children` and
//! `fit_child_in_rect`, and the engine's chain of base classes is the chain
//! of `*_types` functions.
use crate::ui_control::{Control, SizeFlags, Widget, control_types};
use crate::ui_draw::Canvas;
use crate::ui_math::{self, Rect2, Vec2};
use crate::ui_theme::Margins;
use std::cell::Cell;
use std::rc::Rc;

/// Appends the type names of the `Container` base class.
fn container_types<'a>(out: &mut Vec<&'a str>) {
    out.push("Container");
    control_types(out);
}

/// The visible children of `control`, in order.
fn sortable_children(control: &Control) -> Vec<Rc<Control>> {
    let children = control.children();
    let mut result = Vec::with_capacity(children.len());
    for child in children.iter() {
        if child.visible() {
            result.push(Rc::clone(child));
        }
    }
    result
}

/// The largest combined minimum size among the visible children of `control`.
fn largest_child_minimum(control: &Control) -> Vec2 {
    let mut result = Vec2::default();
    for child in sortable_children(control) {
        result = result.max(child.combined_minimum_size());
    }
    result
}

/// Places `child` in `rect`, shrinking and aligning it by its size flags.
fn fit_child_in_rect(child: &Control, rect: &Rect2) {
    let minimum = child.combined_minimum_size();
    let mut target = *rect;

    if !child.h_size_flags().has(SizeFlags::FILL) {
        let width = ui_math::min(ui_math::max(minimum.x, 0.0), rect.size.x);
        target.size.x = ui_math::max(width, minimum.x);
        if child.h_size_flags().has(SizeFlags::SHRINK_END) {
            target.position.x += rect.size.x - target.size.x;
        } else if child.h_size_flags().has(SizeFlags::SHRINK_CENTER) {
            target.position.x += (rect.size.x - target.size.x) * 0.5;
        }
    }

    if !child.v_size_flags().has(SizeFlags::FILL) {
        let height = ui_math::min(ui_math::max(minimum.y, 0.0), rect.size.y);
        target.size.y = ui_math::max(height, minimum.y);
        if child.v_size_flags().has(SizeFlags::SHRINK_END) {
            target.position.y += rect.size.y - target.size.y;
        } else if child.v_size_flags().has(SizeFlags::SHRINK_CENTER) {
            target.position.y += (rect.size.y - target.size.y) * 0.5;
        }
    }

    child.set_rect(&target);
}

/// Lays children out in a row or a column with a themed separation; the
/// engine's `VBoxContainer` and `HBoxContainer` are the two values of `vertical`.
pub struct BoxContainer {
    vertical: bool,
}

impl BoxContainer {
    /// A column when `vertical`, else a row.
    pub fn new(vertical: bool) -> BoxContainer {
        BoxContainer { vertical }
    }
}

impl Widget for BoxContainer {
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>) {
        out.push(if self.vertical { "VBoxContainer" } else { "HBoxContainer" });
        out.push("BoxContainer");
        container_types(out);
    }

    fn get_minimum_size(&self, control: &Control) -> Vec2 {
        let entries = sortable_children(control);
        let separation = control.theme_constant("separation");
        let mut result = Vec2::default();
        for index in 0..entries.len() {
            let minimum = entries[index].combined_minimum_size();
            if self.vertical {
                result.x = ui_math::max(result.x, minimum.x);
                result.y += minimum.y;
                if index + 1 < entries.len() {
                    result.y += separation;
                }
            } else {
                result.y = ui_math::max(result.y, minimum.y);
                result.x += minimum.x;
                if index + 1 < entries.len() {
                    result.x += separation;
                }
            }
        }
        result
    }

    fn layout_children(&self, control: &Control) {
        let entries = sortable_children(control);
        if entries.is_empty() {
            return;
        }
        let separation = control.theme_constant("separation");
        let size = control.size();
        let extent = if self.vertical { size.y } else { size.x };
        let cross = if self.vertical { size.x } else { size.y };

        let mut used = separation * (entries.len() - 1) as f32;
        let mut total_ratio = 0.0f32;
        for child in &entries {
            let minimum = child.combined_minimum_size();
            used += if self.vertical { minimum.y } else { minimum.x };
            let flags = if self.vertical { child.v_size_flags() } else { child.h_size_flags() };
            if flags.has(SizeFlags::EXPAND) {
                total_ratio += child.stretch_ratio();
            }
        }

        let remaining = ui_math::max(extent - used, 0.0);
        let mut cursor = 0.0f32;
        for index in 0..entries.len() {
            let child = &entries[index];
            let minimum = child.combined_minimum_size();
            let mut length = if self.vertical { minimum.y } else { minimum.x };
            let flags = if self.vertical { child.v_size_flags() } else { child.h_size_flags() };
            if flags.has(SizeFlags::EXPAND) && total_ratio > 0.0 {
                length += remaining * (child.stretch_ratio() / total_ratio);
            }
            let slot = if self.vertical {
                Rect2::new(Vec2::new(0.0, cursor), Vec2::new(cross, length))
            } else {
                Rect2::new(Vec2::new(cursor, 0.0), Vec2::new(length, cross))
            };
            fit_child_in_rect(child, &slot);
            cursor += length;
            if index + 1 < entries.len() {
                cursor += separation;
            }
        }
    }
}

/// Insets its children by the themed `margin_*` constants.
pub struct MarginContainer;

impl MarginContainer {
    /// The four themed margins of `control`.
    fn margins(control: &Control) -> Margins {
        Margins {
            left: control.theme_constant("margin_left"),
            top: control.theme_constant("margin_top"),
            right: control.theme_constant("margin_right"),
            bottom: control.theme_constant("margin_bottom"),
        }
    }
}

impl Widget for MarginContainer {
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>) {
        out.push("MarginContainer");
        container_types(out);
    }

    fn get_minimum_size(&self, control: &Control) -> Vec2 {
        largest_child_minimum(control) + MarginContainer::margins(control).size()
    }

    fn layout_children(&self, control: &Control) {
        let margin = MarginContainer::margins(control);
        let slot = Rect2::new(Vec2::new(margin.left, margin.top), (control.size() - margin.size()).max(Vec2::default()));
        for child in sortable_children(control) {
            fit_child_in_rect(&child, &slot);
        }
    }
}

/// Centres each child at its minimum size.
pub struct CenterContainer;

impl Widget for CenterContainer {
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>) {
        out.push("CenterContainer");
        container_types(out);
    }

    fn get_minimum_size(&self, control: &Control) -> Vec2 {
        largest_child_minimum(control)
    }

    fn layout_children(&self, control: &Control) {
        for child in sortable_children(control) {
            let minimum = child.combined_minimum_size();
            let position = (control.size() - minimum) * 0.5;
            child.set_rect(&Rect2::new(position, minimum));
        }
    }
}

/// Draws the themed panel style box and insets its children by its content margins.
pub struct PanelContainer;

impl Widget for PanelContainer {
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>) {
        out.push("PanelContainer");
        container_types(out);
    }

    fn get_minimum_size(&self, control: &Control) -> Vec2 {
        largest_child_minimum(control) + control.theme_stylebox("panel").content_margins.size()
    }

    fn draw(&self, control: &Control, canvas: &mut dyn Canvas) {
        control.theme_stylebox("panel").draw(canvas, &Rect2::new(Vec2::default(), control.size()), control.effective_modulate());
    }

    fn layout_children(&self, control: &Control) {
        let slot = control.theme_stylebox("panel").content_rect(&Rect2::new(Vec2::default(), control.size()));
        for child in sortable_children(control) {
            fit_child_in_rect(&child, &slot);
        }
    }
}

/// Column widths and row heights of a grid.
struct GridMetrics {
    columns: Vec<f32>,
    rows: Vec<f32>,
}

/// Lays children out in a grid with a fixed column count.
pub struct GridContainer {
    columns: usize,
}

impl GridContainer {
    /// A grid of `columns` columns, at least one.
    pub fn new(columns: usize) -> GridContainer {
        GridContainer { columns: columns.max(1) }
    }

    /// The widest cell of each column and the tallest of each row.
    fn measure(&self, control: &Control) -> GridMetrics {
        let entries = sortable_children(control);
        let mut metrics = GridMetrics { columns: vec![0.0; self.columns], rows: Vec::new() };
        for index in 0..entries.len() {
            let column = index % self.columns;
            let row = index / self.columns;
            if metrics.rows.len() <= row {
                metrics.rows.push(0.0);
            }
            let minimum = entries[index].combined_minimum_size();
            metrics.columns[column] = ui_math::max(metrics.columns[column], minimum.x);
            metrics.rows[row] = ui_math::max(metrics.rows[row], minimum.y);
        }
        metrics
    }
}

impl Widget for GridContainer {
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>) {
        out.push("GridContainer");
        container_types(out);
    }

    fn get_minimum_size(&self, control: &Control) -> Vec2 {
        let metrics = self.measure(control);
        let horizontal = control.theme_constant("h_separation");
        let vertical = control.theme_constant("v_separation");

        let mut result = Vec2::default();
        for width in &metrics.columns {
            result.x += *width;
        }
        for height in &metrics.rows {
            result.y += *height;
        }
        if !metrics.columns.is_empty() {
            result.x += horizontal * (metrics.columns.len() - 1) as f32;
        }
        if !metrics.rows.is_empty() {
            result.y += vertical * (metrics.rows.len() - 1) as f32;
        }
        result
    }

    fn layout_children(&self, control: &Control) {
        let entries = sortable_children(control);
        if entries.is_empty() {
            return;
        }
        let mut metrics = self.measure(control);
        let horizontal = control.theme_constant("h_separation");
        let vertical = control.theme_constant("v_separation");

        let minimum = self.get_minimum_size(control);
        let extra = (control.size() - minimum).max(Vec2::default());
        if !metrics.columns.is_empty() && extra.x > 0.0 {
            let share = extra.x / metrics.columns.len() as f32;
            for width in metrics.columns.iter_mut() {
                *width += share;
            }
        }
        if !metrics.rows.is_empty() && extra.y > 0.0 {
            let share = extra.y / metrics.rows.len() as f32;
            for height in metrics.rows.iter_mut() {
                *height += share;
            }
        }

        for index in 0..entries.len() {
            let column = index % self.columns;
            let row = index / self.columns;
            let mut x = 0.0f32;
            for i in 0..column {
                x += metrics.columns[i] + horizontal;
            }
            let mut y = 0.0f32;
            for i in 0..row {
                y += metrics.rows[i] + vertical;
            }
            fit_child_in_rect(&entries[index], &Rect2::new(Vec2::new(x, y), Vec2::new(metrics.columns[column], metrics.rows[row])));
        }
    }
}

/// Shows one of its children under a bar of titled tabs.
pub struct TabContainer {
    titles: Vec<String>,
    current: Cell<usize>,
}

impl TabContainer {
    /// A tab container whose tabs carry `titles`, showing the first.
    pub fn new(titles: Vec<String>) -> TabContainer {
        TabContainer { titles, current: Cell::new(0) }
    }

    /// The title of tab `index`, or empty.
    fn tab_title(&self, index: usize) -> &str {
        match self.titles.get(index) {
            Some(title) => title,
            None => "",
        }
    }

    /// The height of the tab bar.
    fn tab_bar_height(control: &Control) -> f32 {
        let font_size = control.theme_font_size("font_size");
        let content = control.theme_stylebox("tab_selected").content_margins;
        control.font_metrics(font_size).line_height + content.size().y
    }

    /// The rectangle of every tab in the bar.
    fn tab_rects(&self, control: &Control) -> Vec<Rect2> {
        let font_size = control.theme_font_size("font_size");
        let separation = control.theme_constant("h_separation");
        let content = control.theme_stylebox("tab_selected").content_margins;
        let height = TabContainer::tab_bar_height(control);

        let mut result = Vec::new();
        let mut x = 0.0f32;
        let count = control.children().len();
        for index in 0..count {
            let width = control.measure_text(self.tab_title(index), font_size).x + content.size().x;
            result.push(Rect2::new(Vec2::new(x, 0.0), Vec2::new(width, height)));
            x += width + separation;
        }
        result
    }

    /// Shows the current child and hides the others.
    fn sync_visibility(&self, control: &Control) {
        for (index, child) in control.children().iter().enumerate() {
            child.set_visible(index == self.current.get());
        }
    }
}

impl Widget for TabContainer {
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>) {
        out.push("TabContainer");
        container_types(out);
    }

    fn get_minimum_size(&self, control: &Control) -> Vec2 {
        let mut content = largest_child_minimum(control);
        let panel = control.theme_stylebox("panel").content_margins;
        content = content + panel.size();

        let mut bar_width = 0.0f32;
        for tab in self.tab_rects(control) {
            bar_width = ui_math::max(bar_width, tab.right());
        }
        Vec2::new(ui_math::max(content.x, bar_width), content.y + TabContainer::tab_bar_height(control))
    }

    fn layout_children(&self, control: &Control) {
        let count = control.children().len();
        if count == 0 {
            return;
        }
        self.current.set(self.current.get().min(count - 1));
        self.sync_visibility(control);

        let bar = TabContainer::tab_bar_height(control);
        let size = control.size();
        let body = Rect2::new(Vec2::new(0.0, bar), Vec2::new(size.x, ui_math::max(size.y - bar, 0.0)));
        let slot = control.theme_stylebox("panel").content_rect(&body);

        for (index, child) in control.children().iter().enumerate() {
            if index == self.current.get() {
                fit_child_in_rect(child, &slot);
            }
        }
    }

    fn draw(&self, control: &Control, canvas: &mut dyn Canvas) {
        let tint = control.effective_modulate();
        let bar = TabContainer::tab_bar_height(control);
        let size = control.size();
        let body = Rect2::new(Vec2::new(0.0, bar), Vec2::new(size.x, ui_math::max(size.y - bar, 0.0)));
        control.theme_stylebox("panel").draw(canvas, &body, tint);

        let font_size = control.theme_font_size("font_size");
        let tabs = self.tab_rects(control);
        for (index, tab) in tabs.iter().enumerate() {
            let selected = index == self.current.get();
            let style = if selected { "tab_selected" } else { "tab_unselected" };
            control.theme_stylebox(style).draw(canvas, tab, tint);

            let color = if selected { control.theme_color("font_selected_color") } else { control.theme_color("font_unselected_color") };
            let content = control.theme_stylebox(style).content_rect(tab);
            let measured = canvas.measure_text(self.tab_title(index), font_size);
            canvas.draw_text(
                Vec2::new(content.position.x, content.position.y + (content.size.y - measured.y) * 0.5),
                self.tab_title(index),
                font_size,
                color.modulated(tint),
            );
        }
    }
}
