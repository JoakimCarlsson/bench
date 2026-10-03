#include "ui_container.hpp"

#include <algorithm>
#include <utility>

namespace bench::ui {

void Container::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("Container");
    Control::collect_theme_types(out);
}

std::vector<Control*> Container::sortable_children() const {
    std::vector<Control*> result;
    result.reserve(children().size());
    for (const std::unique_ptr<Control>& child : children()) {
        if (child->visible()) result.push_back(child.get());
    }
    return result;
}

void Container::layout_children() {
    sort_children();
}

void Container::fit_child_in_rect(Control& child, const Rect2& rect) const {
    const Vec2 minimum = child.combined_minimum_size();
    Rect2 target = rect;

    if (!has_flag(child.h_size_flags(), SizeFlags::Fill)) {
        const float width = std::min(std::max(minimum.x, 0.0f), rect.size.x);
        target.size.x = std::max(width, minimum.x);
        if (has_flag(child.h_size_flags(), SizeFlags::ShrinkEnd)) {
            target.position.x += rect.size.x - target.size.x;
        } else if (has_flag(child.h_size_flags(), SizeFlags::ShrinkCenter)) {
            target.position.x += (rect.size.x - target.size.x) * 0.5f;
        }
    }

    if (!has_flag(child.v_size_flags(), SizeFlags::Fill)) {
        const float height = std::min(std::max(minimum.y, 0.0f), rect.size.y);
        target.size.y = std::max(height, minimum.y);
        if (has_flag(child.v_size_flags(), SizeFlags::ShrinkEnd)) {
            target.position.y += rect.size.y - target.size.y;
        } else if (has_flag(child.v_size_flags(), SizeFlags::ShrinkCenter)) {
            target.position.y += (rect.size.y - target.size.y) * 0.5f;
        }
    }

    child.set_rect(target);
}

BoxContainer::BoxContainer(bool vertical) : vertical_{vertical} {}

void BoxContainer::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("BoxContainer");
    Container::collect_theme_types(out);
}

Vec2 BoxContainer::get_minimum_size() const {
    const std::vector<Control*> entries = sortable_children();
    const float separation = theme_constant("separation");
    Vec2 result{};
    for (size_t index = 0; index < entries.size(); ++index) {
        const Vec2 minimum = entries[index]->combined_minimum_size();
        if (vertical_) {
            result.x = std::max(result.x, minimum.x);
            result.y += minimum.y;
            if (index + 1 < entries.size()) result.y += separation;
        } else {
            result.y = std::max(result.y, minimum.y);
            result.x += minimum.x;
            if (index + 1 < entries.size()) result.x += separation;
        }
    }
    return result;
}

void BoxContainer::sort_children() {
    const std::vector<Control*> entries = sortable_children();
    if (entries.empty()) return;
    const float separation = theme_constant("separation");
    const float extent = vertical_ ? size().y : size().x;
    const float cross = vertical_ ? size().x : size().y;

    float used = separation * static_cast<float>(entries.size() - 1);
    float total_ratio = 0.0f;
    for (const Control* child : entries) {
        const Vec2 minimum = child->combined_minimum_size();
        used += vertical_ ? minimum.y : minimum.x;
        const SizeFlags flags = vertical_ ? child->v_size_flags() : child->h_size_flags();
        if (has_flag(flags, SizeFlags::Expand)) total_ratio += child->stretch_ratio();
    }

    const float remaining = std::max(extent - used, 0.0f);
    float cursor = 0.0f;
    for (size_t index = 0; index < entries.size(); ++index) {
        Control* child = entries[index];
        const Vec2 minimum = child->combined_minimum_size();
        float length = vertical_ ? minimum.y : minimum.x;
        const SizeFlags flags = vertical_ ? child->v_size_flags() : child->h_size_flags();
        if (has_flag(flags, SizeFlags::Expand) && total_ratio > 0.0f) length += remaining * (child->stretch_ratio() / total_ratio);
        const Rect2 slot = vertical_ ? Rect2{{0.0f, cursor}, {cross, length}} : Rect2{{cursor, 0.0f}, {length, cross}};
        fit_child_in_rect(*child, slot);
        cursor += length;
        if (index + 1 < entries.size()) cursor += separation;
    }
}

VBoxContainer::VBoxContainer() : BoxContainer{true} {}

void VBoxContainer::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("VBoxContainer");
    BoxContainer::collect_theme_types(out);
}

HBoxContainer::HBoxContainer() : BoxContainer{false} {}

void HBoxContainer::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("HBoxContainer");
    BoxContainer::collect_theme_types(out);
}

void MarginContainer::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("MarginContainer");
    Container::collect_theme_types(out);
}

Margins MarginContainer::margins() const {
    return Margins{theme_constant("margin_left"), theme_constant("margin_top"), theme_constant("margin_right"), theme_constant("margin_bottom")};
}

Vec2 MarginContainer::get_minimum_size() const {
    Vec2 result{};
    for (const Control* child : sortable_children()) result = max(result, child->combined_minimum_size());
    return result + margins().size();
}

void MarginContainer::sort_children() {
    const Margins margin = margins();
    const Rect2 slot{{margin.left, margin.top}, max(size() - margin.size(), Vec2{})};
    for (Control* child : sortable_children()) fit_child_in_rect(*child, slot);
}

void CenterContainer::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("CenterContainer");
    Container::collect_theme_types(out);
}

Vec2 CenterContainer::get_minimum_size() const {
    Vec2 result{};
    for (const Control* child : sortable_children()) result = max(result, child->combined_minimum_size());
    return result;
}

void CenterContainer::sort_children() {
    for (Control* child : sortable_children()) {
        const Vec2 minimum = child->combined_minimum_size();
        const Vec2 position = (size() - minimum) * 0.5f;
        child->set_rect(Rect2{position, minimum});
    }
}

void PanelContainer::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("PanelContainer");
    Container::collect_theme_types(out);
}

Vec2 PanelContainer::get_minimum_size() const {
    Vec2 result{};
    for (const Control* child : sortable_children()) result = max(result, child->combined_minimum_size());
    return result + theme_stylebox("panel").content_margins.size();
}

void PanelContainer::draw(Canvas& canvas) const {
    theme_stylebox("panel").draw(canvas, Rect2{{}, size()}, effective_modulate());
}

void PanelContainer::sort_children() {
    const Rect2 slot = theme_stylebox("panel").content_rect(Rect2{{}, size()});
    for (Control* child : sortable_children()) fit_child_in_rect(*child, slot);
}

void GridContainer::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("GridContainer");
    Container::collect_theme_types(out);
}

void GridContainer::set_columns(uint32_t columns) {
    const uint32_t target = std::max(columns, 1u);
    if (columns_ == target) return;
    columns_ = target;
    update_minimum_size();
}

GridMetrics GridContainer::measure() const {
    const std::vector<Control*> entries = sortable_children();
    GridMetrics metrics;
    metrics.columns.assign(columns_, 0.0f);
    for (size_t index = 0; index < entries.size(); ++index) {
        const size_t column = index % columns_;
        const size_t row = index / columns_;
        if (metrics.rows.size() <= row) metrics.rows.push_back(0.0f);
        const Vec2 minimum = entries[index]->combined_minimum_size();
        metrics.columns[column] = std::max(metrics.columns[column], minimum.x);
        metrics.rows[row] = std::max(metrics.rows[row], minimum.y);
    }
    return metrics;
}

Vec2 GridContainer::get_minimum_size() const {
    const GridMetrics metrics = measure();
    const float horizontal = theme_constant("h_separation");
    const float vertical = theme_constant("v_separation");

    Vec2 result{};
    for (const float width : metrics.columns) result.x += width;
    for (const float height : metrics.rows) result.y += height;
    if (!metrics.columns.empty()) result.x += horizontal * static_cast<float>(metrics.columns.size() - 1);
    if (!metrics.rows.empty()) result.y += vertical * static_cast<float>(metrics.rows.size() - 1);
    return result;
}

void GridContainer::sort_children() {
    const std::vector<Control*> entries = sortable_children();
    if (entries.empty()) return;
    GridMetrics metrics = measure();
    std::vector<float>& column_widths = metrics.columns;
    std::vector<float>& row_heights = metrics.rows;
    const float horizontal = theme_constant("h_separation");
    const float vertical = theme_constant("v_separation");

    const Vec2 minimum = get_minimum_size();
    const Vec2 extra = max(size() - minimum, Vec2{});
    if (!column_widths.empty() && extra.x > 0.0f) {
        const float share = extra.x / static_cast<float>(column_widths.size());
        for (float& width : column_widths) width += share;
    }
    if (!row_heights.empty() && extra.y > 0.0f) {
        const float share = extra.y / static_cast<float>(row_heights.size());
        for (float& height : row_heights) height += share;
    }

    for (size_t index = 0; index < entries.size(); ++index) {
        const size_t column = index % columns_;
        const size_t row = index / columns_;
        float x = 0.0f;
        for (size_t i = 0; i < column; ++i) x += column_widths[i] + horizontal;
        float y = 0.0f;
        for (size_t i = 0; i < row; ++i) y += row_heights[i] + vertical;
        fit_child_in_rect(*entries[index], Rect2{{x, y}, {column_widths[column], row_heights[row]}});
    }
}

void TabContainer::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("TabContainer");
    Container::collect_theme_types(out);
}

void TabContainer::set_tab_title(size_t index, std::string title) {
    if (titles_.size() <= index) titles_.resize(index + 1);
    titles_[index] = std::move(title);
    update_minimum_size();
}

const std::string& TabContainer::tab_title(size_t index) const {
    static const std::string empty;
    return index < titles_.size() ? titles_[index] : empty;
}

void TabContainer::set_current_tab(size_t index) {
    const size_t count = children().size();
    if (count == 0) {
        current_ = 0;
        return;
    }
    const size_t target = std::min(index, count - 1);
    if (current_ == target) return;
    current_ = target;
    queue_layout();
}

float TabContainer::tab_bar_height() const {
    const float font_size = theme_font_size("font_size");
    const Margins content = theme_stylebox("tab_selected").content_margins;
    return canvas().metrics(font_size).line_height + content.size().y;
}

std::vector<Rect2> TabContainer::tab_rects() const {
    const float font_size = theme_font_size("font_size");
    const float separation = theme_constant("h_separation");
    const Margins content = theme_stylebox("tab_selected").content_margins;
    const float height = tab_bar_height();
    const Canvas& target = canvas();

    std::vector<Rect2> result;
    float x = 0.0f;
    const size_t count = children().size();
    for (size_t index = 0; index < count; ++index) {
        const float width = target.measure_text(tab_title(index), font_size).x + content.size().x;
        result.push_back(Rect2{{x, 0.0f}, {width, height}});
        x += width + separation;
    }
    return result;
}

Vec2 TabContainer::get_minimum_size() const {
    Vec2 content{};
    for (const Control* child : sortable_children()) content = max(content, child->combined_minimum_size());
    const Margins panel = theme_stylebox("panel").content_margins;
    content = content + panel.size();

    float bar_width = 0.0f;
    for (const Rect2& tab : tab_rects()) bar_width = std::max(bar_width, tab.right());
    return {std::max(content.x, bar_width), content.y + tab_bar_height()};
}

void TabContainer::sync_visibility() {
    size_t index = 0;
    for (const std::unique_ptr<Control>& child : children()) {
        child->set_visible(index == current_);
        ++index;
    }
}

void TabContainer::sort_children() {
    const size_t count = children().size();
    if (count == 0) return;
    current_ = std::min(current_, count - 1);
    sync_visibility();

    const float bar = tab_bar_height();
    const Rect2 body{{0.0f, bar}, {size().x, std::max(size().y - bar, 0.0f)}};
    const Rect2 slot = theme_stylebox("panel").content_rect(body);

    size_t index = 0;
    for (const std::unique_ptr<Control>& child : children()) {
        if (index == current_) fit_child_in_rect(*child, slot);
        ++index;
    }
}

void TabContainer::draw(Canvas& canvas) const {
    const Color4 tint = effective_modulate();
    const float bar = tab_bar_height();
    const Rect2 body{{0.0f, bar}, {size().x, std::max(size().y - bar, 0.0f)}};
    theme_stylebox("panel").draw(canvas, body, tint);

    const float font_size = theme_font_size("font_size");
    const std::vector<Rect2> tabs = tab_rects();
    for (size_t index = 0; index < tabs.size(); ++index) {
        const bool selected = index == current_;
        const std::string_view style = selected ? "tab_selected" : "tab_unselected";
        theme_stylebox(style).draw(canvas, tabs[index], tint);

        const Color4 color = selected ? theme_color("font_selected_color") : theme_color("font_unselected_color");
        const Rect2 content = theme_stylebox(style).content_rect(tabs[index]);
        const Vec2 measured = canvas.measure_text(tab_title(index), font_size);
        canvas.draw_text({content.position.x, content.position.y + (content.size.y - measured.y) * 0.5f}, tab_title(index), font_size, modulated(color, tint));
    }
}

} // namespace bench::ui
