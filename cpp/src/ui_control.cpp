#include "ui_control.hpp"

#include <algorithm>

namespace bench::ui {

namespace {

constexpr size_t max_type_depth = 8;

/// Walks `type` and its base types in `theme`, returning the first item `lookup` finds, or null.
template <typename Value, typename Lookup>
const Value* find_in_type_chain(const Theme& theme, std::string_view type, std::string_view name, Lookup lookup) {
    std::string_view current = type;
    for (size_t depth = 0; depth < max_type_depth; ++depth) {
        if (const Value* found = lookup(theme, ThemeKey{current, name}); found != nullptr) return found;
        const std::string_view base = theme.type_base(current);
        if (base.empty()) break;
        current = base;
    }
    return nullptr;
}

/// Looks up a theme item for a control, trying its type variation and theme
/// types up each base chain. Searches the control's own theme and its
/// ancestors' themes before the context's fallback theme.
template <typename Value, typename Lookup>
const Value* find_in_themes(const Control& control, std::string_view name, Lookup lookup) {
    std::vector<std::string_view> types;
    if (!control.theme_type_variation().empty()) types.push_back(control.theme_type_variation());
    control.collect_theme_types(types);

    const Control* owner = &control;
    while (owner != nullptr) {
        const std::shared_ptr<Theme>& theme = owner->theme();
        if (theme) {
            for (const std::string_view type : types) {
                if (const Value* found = find_in_type_chain<Value>(*theme, type, name, lookup); found != nullptr) return found;
            }
        }
        owner = owner->parent();
    }

    const Context* ui = control.ui();
    if (ui == nullptr || !ui->theme()) return nullptr;
    for (const std::string_view type : types) {
        if (const Value* found = find_in_type_chain<Value>(*ui->theme(), type, name, lookup); found != nullptr) return found;
    }
    return nullptr;
}

} // namespace

PresetAnchors preset_anchors(LayoutPreset preset) noexcept {
    switch (preset) {
    case LayoutPreset::TopLeft:
        return {0.0f, 0.0f, 0.0f, 0.0f};
    case LayoutPreset::TopRight:
        return {1.0f, 0.0f, 1.0f, 0.0f};
    case LayoutPreset::BottomLeft:
        return {0.0f, 1.0f, 0.0f, 1.0f};
    case LayoutPreset::BottomRight:
        return {1.0f, 1.0f, 1.0f, 1.0f};
    case LayoutPreset::CenterLeft:
        return {0.0f, 0.5f, 0.0f, 0.5f};
    case LayoutPreset::CenterTop:
        return {0.5f, 0.0f, 0.5f, 0.0f};
    case LayoutPreset::CenterRight:
        return {1.0f, 0.5f, 1.0f, 0.5f};
    case LayoutPreset::CenterBottom:
        return {0.5f, 1.0f, 0.5f, 1.0f};
    case LayoutPreset::Center:
        return {0.5f, 0.5f, 0.5f, 0.5f};
    case LayoutPreset::LeftWide:
        return {0.0f, 0.0f, 0.0f, 1.0f};
    case LayoutPreset::TopWide:
        return {0.0f, 0.0f, 1.0f, 0.0f};
    case LayoutPreset::RightWide:
        return {1.0f, 0.0f, 1.0f, 1.0f};
    case LayoutPreset::BottomWide:
        return {0.0f, 1.0f, 1.0f, 1.0f};
    case LayoutPreset::VCenterWide:
        return {0.0f, 0.5f, 1.0f, 0.5f};
    case LayoutPreset::HCenterWide:
        return {0.5f, 0.0f, 0.5f, 1.0f};
    case LayoutPreset::FullRect:
        return {0.0f, 0.0f, 1.0f, 1.0f};
    }
    return {};
}

Control::Control() = default;

Control& Control::add_child(std::unique_ptr<Control> child) {
    Control& reference = *child;
    children_.push_back(std::move(child));
    reference.attach(this, ui_);
    update_minimum_size();
    queue_layout();
    return reference;
}

const std::vector<std::unique_ptr<Control>>& Control::children() const noexcept {
    return children_;
}

Control* Control::parent() const noexcept {
    return parent_;
}

Context* Control::ui() const noexcept {
    return ui_;
}

void Control::attach(Control* parent, Context* ui) {
    parent_ = parent;
    propagate_ui(ui);
}

void Control::propagate_ui(Context* ui) {
    if (ui_ == ui) return;
    ui_ = ui;
    for (const std::unique_ptr<Control>& child : children_) child->propagate_ui(ui);
}

void Control::set_anchor(Side side, float value, bool keep_offset) {
    const auto index = static_cast<size_t>(side);
    const float previous = anchor_[index];
    anchor_[index] = std::clamp(value, 0.0f, 1.0f);
    if (keep_offset && parent_ != nullptr) {
        const float area = (index % 2 == 0) ? parent_->size_.x : parent_->size_.y;
        offset_[index] -= (anchor_[index] - previous) * area;
    }
    queue_layout();
}

void Control::set_offset(Side side, float value) {
    offset_[static_cast<size_t>(side)] = value;
    queue_layout();
}

void Control::set_anchors_preset(LayoutPreset preset, bool keep_offsets) {
    const PresetAnchors anchors = preset_anchors(preset);
    set_anchor(Side::Left, anchors.left, keep_offsets);
    set_anchor(Side::Top, anchors.top, keep_offsets);
    set_anchor(Side::Right, anchors.right, keep_offsets);
    set_anchor(Side::Bottom, anchors.bottom, keep_offsets);
}

void Control::set_offsets_preset(LayoutPreset preset, float margin) {
    const PresetAnchors anchors = preset_anchors(preset);
    const Vec2 minimum = combined_minimum_size();
    const Vec2 target = max(size_, minimum);

    const bool stretch_h = anchors.left != anchors.right;
    const bool stretch_v = anchors.top != anchors.bottom;

    if (stretch_h) {
        offset_[0] = margin;
        offset_[2] = -margin;
    } else if (anchors.left == 0.0f) {
        offset_[0] = margin;
        offset_[2] = margin + target.x;
    } else if (anchors.left == 1.0f) {
        offset_[0] = -margin - target.x;
        offset_[2] = -margin;
    } else {
        offset_[0] = -target.x * 0.5f;
        offset_[2] = target.x * 0.5f;
    }

    if (stretch_v) {
        offset_[1] = margin;
        offset_[3] = -margin;
    } else if (anchors.top == 0.0f) {
        offset_[1] = margin;
        offset_[3] = margin + target.y;
    } else if (anchors.top == 1.0f) {
        offset_[1] = -margin - target.y;
        offset_[3] = -margin;
    } else {
        offset_[1] = -target.y * 0.5f;
        offset_[3] = target.y * 0.5f;
    }
    queue_layout();
}

void Control::set_anchors_and_offsets_preset(LayoutPreset preset, float margin) {
    set_anchors_preset(preset, false);
    set_offsets_preset(preset, margin);
}

void Control::set_rect(const Rect2& rect) {
    const Vec2 target = max(rect.size, combined_minimum_size());
    position_ = rect.position;
    offset_[0] = rect.position.x;
    offset_[1] = rect.position.y;
    offset_[2] = rect.position.x + target.x;
    offset_[3] = rect.position.y + target.y;
    if (size_ != target) {
        size_ = target;
        resized();
    }
}

Vec2 Control::position() const noexcept {
    return position_;
}

Vec2 Control::size() const noexcept {
    return size_;
}

void Control::set_custom_minimum_size(Vec2 size) {
    if (custom_minimum_size_ == size) return;
    custom_minimum_size_ = size;
    update_minimum_size();
}

Vec2 Control::combined_minimum_size() const {
    if (!minimum_size_valid_) {
        minimum_size_cache_ = max(custom_minimum_size_, get_minimum_size());
        minimum_size_valid_ = true;
    }
    return minimum_size_cache_;
}

void Control::update_minimum_size() {
    minimum_size_valid_ = false;
    Control* current = parent_;
    while (current != nullptr) {
        current->minimum_size_valid_ = false;
        current = current->parent_;
    }
    queue_layout();
}

void Control::set_h_size_flags(SizeFlags flags) {
    h_size_flags_ = flags;
    queue_layout();
}

void Control::set_v_size_flags(SizeFlags flags) {
    v_size_flags_ = flags;
    queue_layout();
}

SizeFlags Control::h_size_flags() const noexcept {
    return h_size_flags_;
}

SizeFlags Control::v_size_flags() const noexcept {
    return v_size_flags_;
}

void Control::set_stretch_ratio(float ratio) {
    stretch_ratio_ = std::max(ratio, 0.0f);
    queue_layout();
}

float Control::stretch_ratio() const noexcept {
    return stretch_ratio_;
}

void Control::set_h_grow_direction(GrowDirection direction) {
    h_grow_ = direction;
    queue_layout();
}

void Control::set_v_grow_direction(GrowDirection direction) {
    v_grow_ = direction;
    queue_layout();
}

void Control::set_visible(bool visible) {
    if (visible_ == visible) return;
    visible_ = visible;
    update_minimum_size();
}

bool Control::visible() const noexcept {
    return visible_;
}

void Control::set_clip_contents(bool clip) {
    clip_contents_ = clip;
}

bool Control::clip_contents() const noexcept {
    return clip_contents_;
}

void Control::set_modulate(Color4 modulate) {
    modulate_ = modulate;
}

Color4 Control::effective_modulate() const noexcept {
    Color4 result = modulate_;
    const Control* current = parent_;
    while (current != nullptr) {
        result = modulated(result, current->modulate_);
        current = current->parent_;
    }
    return result;
}

void Control::set_hovered(bool hovered) {
    hovered_ = hovered;
}

bool Control::hovered() const noexcept {
    return hovered_;
}

bool Control::has_focus() const noexcept {
    return ui_ != nullptr && ui_->focused() == this;
}

void Control::set_theme(std::shared_ptr<Theme> theme) {
    theme_ = std::move(theme);
    update_minimum_size();
}

const std::shared_ptr<Theme>& Control::theme() const noexcept {
    return theme_;
}

void Control::set_theme_type_variation(std::string type) {
    theme_type_variation_ = std::move(type);
    update_minimum_size();
}

const std::string& Control::theme_type_variation() const noexcept {
    return theme_type_variation_;
}

void Control::add_color_override(std::string_view name, Color4 value) {
    color_overrides_.insert_or_assign(std::string{name}, value);
    update_minimum_size();
}

void Control::add_constant_override(std::string_view name, float value) {
    constant_overrides_.insert_or_assign(std::string{name}, value);
    update_minimum_size();
}

const StyleBox& Control::theme_stylebox(std::string_view name) const {
    static const StyleBox fallback{};
    const StyleBox* found = find_in_themes<StyleBox>(*this, name, [](const Theme& theme, ThemeKey key) { return theme.stylebox(key); });
    return found == nullptr ? fallback : *found;
}

Color4 Control::theme_color(std::string_view name) const {
    const auto override_found = color_overrides_.find(name);
    if (override_found != color_overrides_.end()) return override_found->second;
    const Color4* found = find_in_themes<Color4>(*this, name, [](const Theme& theme, ThemeKey key) { return theme.color(key); });
    return found == nullptr ? Color4{} : *found;
}

float Control::theme_constant(std::string_view name) const {
    const auto override_found = constant_overrides_.find(name);
    if (override_found != constant_overrides_.end()) return override_found->second;
    const float* found = find_in_themes<float>(*this, name, [](const Theme& theme, ThemeKey key) { return theme.constant(key); });
    return found == nullptr ? 0.0f : *found;
}

float Control::theme_font_size(std::string_view name) const {
    const float* found = find_in_themes<float>(*this, name, [](const Theme& theme, ThemeKey key) { return theme.font_size(key); });
    return found == nullptr ? 16.0f : *found;
}

const Canvas& Control::canvas() const {
    return ui_->canvas();
}

void Control::queue_layout() {
    if (ui_ != nullptr) ui_->queue_layout();
}

void Control::apply_anchors(Vec2 parent_size) {
    float edges[4]{};
    for (size_t i = 0; i < 4; ++i) {
        const float area = (i % 2 == 0) ? parent_size.x : parent_size.y;
        edges[i] = offset_[i] + anchor_[i] * area;
    }

    Vec2 new_position{edges[0], edges[1]};
    Vec2 new_size{edges[2] - edges[0], edges[3] - edges[1]};
    const Vec2 minimum = combined_minimum_size();

    if (minimum.x > new_size.x) {
        if (h_grow_ == GrowDirection::Begin) {
            new_position.x += new_size.x - minimum.x;
        } else if (h_grow_ == GrowDirection::Both) {
            new_position.x += (new_size.x - minimum.x) * 0.5f;
        }
        new_size.x = minimum.x;
    }
    if (minimum.y > new_size.y) {
        if (v_grow_ == GrowDirection::Begin) {
            new_position.y += new_size.y - minimum.y;
        } else if (v_grow_ == GrowDirection::Both) {
            new_position.y += (new_size.y - minimum.y) * 0.5f;
        }
        new_size.y = minimum.y;
    }

    position_ = new_position;
    if (size_ != new_size) {
        size_ = new_size;
        resized();
    }
}

void Control::collect_theme_types(std::vector<std::string_view>& out) const {
    out.emplace_back("Control");
}

Vec2 Control::get_minimum_size() const {
    return {};
}

void Control::draw(Canvas&) const {}

void Control::layout_children() {
    for (const std::unique_ptr<Control>& child : children_) child->apply_anchors(size_);
}

void Control::resized() {}

Context::Context(Canvas& canvas) : canvas_{&canvas}, root_{std::make_unique<Control>()}, theme_{default_theme()} {
    root_->attach(nullptr, this);
}

Canvas& Context::canvas() const noexcept {
    return *canvas_;
}

Control& Context::root() noexcept {
    return *root_;
}

const std::shared_ptr<Theme>& Context::theme() const noexcept {
    return theme_;
}

void Context::set_focus(Control* control) noexcept {
    focus_ = control;
}

Control* Context::focused() const noexcept {
    return focus_;
}

void Context::set_viewport_rect(const Rect2& rect) {
    if (viewport_rect_ == rect) return;
    viewport_rect_ = rect;
    queue_layout();
}

const Rect2& Context::viewport_rect() const noexcept {
    return viewport_rect_;
}

void Context::queue_layout() noexcept {
    layout_dirty_ = true;
}

void Context::layout(Control& control) {
    control.layout_children();
    for (const std::unique_ptr<Control>& child : control.children()) {
        if (child->visible()) layout(*child);
    }
}

void Context::update() {
    root_->set_rect(viewport_rect_);
    if (!layout_dirty_) return;
    layout_dirty_ = false;
    layout(*root_);
}

void Context::paint(Control& control, Vec2 origin) {
    if (!control.visible()) return;
    Canvas& target = canvas();
    const Vec2 position = origin + control.position();
    target.push_offset(control.position());
    if (control.clip_contents()) target.push_clip(Rect2{position, control.size()});
    control.draw(target);
    for (const std::unique_ptr<Control>& child : control.children()) paint(*child, position);
    if (control.clip_contents()) target.pop_clip();
    target.pop_offset();
}

void Context::draw() {
    paint(*root_, Vec2{});
}

} // namespace bench::ui
