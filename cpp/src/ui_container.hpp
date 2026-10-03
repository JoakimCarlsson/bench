#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include "ui_control.hpp"

namespace bench::ui {

/// A control that positions its visible children itself.
class Container : public Control {
public:
    Container() = default;

    void collect_theme_types(std::vector<std::string_view>& out) const override;
    void layout_children() override;

    /// Places `child` in `rect`, shrinking and aligning it by its size flags.
    void fit_child_in_rect(Control& child, const Rect2& rect) const;

protected:
    /// The visible children, in order.
    std::vector<Control*> sortable_children() const;

    /// Places every sortable child.
    virtual void sort_children() = 0;
};

/// Lays children out in a row or a column with a themed separation.
class BoxContainer : public Container {
public:
    /// A column when `vertical`, else a row.
    explicit BoxContainer(bool vertical = true);

    void collect_theme_types(std::vector<std::string_view>& out) const override;
    Vec2 get_minimum_size() const override;

protected:
    void sort_children() override;

private:
    bool vertical_{};
};

/// A box container laid out top to bottom.
class VBoxContainer final : public BoxContainer {
public:
    VBoxContainer();
    void collect_theme_types(std::vector<std::string_view>& out) const override;
};

/// A box container laid out left to right.
class HBoxContainer final : public BoxContainer {
public:
    HBoxContainer();
    void collect_theme_types(std::vector<std::string_view>& out) const override;
};

/// Insets its children by the themed `margin_*` constants.
class MarginContainer final : public Container {
public:
    void collect_theme_types(std::vector<std::string_view>& out) const override;
    Vec2 get_minimum_size() const override;

protected:
    void sort_children() override;

private:
    /// The four themed margins.
    Margins margins() const;
};

/// Centres each child at its minimum size.
class CenterContainer final : public Container {
public:
    void collect_theme_types(std::vector<std::string_view>& out) const override;
    Vec2 get_minimum_size() const override;

protected:
    void sort_children() override;
};

/// Draws the themed panel style box and insets its children by its content margins.
class PanelContainer final : public Container {
public:
    void collect_theme_types(std::vector<std::string_view>& out) const override;
    Vec2 get_minimum_size() const override;
    void draw(Canvas& canvas) const override;

protected:
    void sort_children() override;
};

/// Column widths and row heights of a grid.
struct GridMetrics {
    std::vector<float> columns;
    std::vector<float> rows;
};

/// Lays children out in a grid with a fixed column count.
class GridContainer final : public Container {
public:
    void collect_theme_types(std::vector<std::string_view>& out) const override;
    Vec2 get_minimum_size() const override;

    /// Sets the column count, at least one.
    void set_columns(uint32_t columns);

protected:
    void sort_children() override;

private:
    /// The widest cell of each column and the tallest of each row.
    GridMetrics measure() const;

    uint32_t columns_{1};
};

/// Shows one of its children under a bar of titled tabs.
class TabContainer final : public Container {
public:
    void collect_theme_types(std::vector<std::string_view>& out) const override;
    Vec2 get_minimum_size() const override;
    void draw(Canvas& canvas) const override;

    /// Sets the title of tab `index`.
    void set_tab_title(size_t index, std::string title);
    /// Shows tab `index`, clamped to the last one.
    void set_current_tab(size_t index);

protected:
    void sort_children() override;

private:
    /// The title of tab `index`, or empty.
    const std::string& tab_title(size_t index) const;
    /// The height of the tab bar.
    float tab_bar_height() const;
    /// The rectangle of every tab in the bar.
    std::vector<Rect2> tab_rects() const;
    /// Shows the current child and hides the others.
    void sync_visibility();

    std::vector<std::string> titles_;
    size_t current_{};
};

} // namespace bench::ui
