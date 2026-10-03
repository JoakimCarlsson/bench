#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include "ui_draw.hpp"
#include "ui_math.hpp"
#include "ui_theme.hpp"

namespace bench::ui {

class Context;

/// A side of a rectangle.
enum class Side : uint8_t { Left, Top, Right, Bottom };

/// Named anchor layouts such as corners, edges and full rect.
enum class LayoutPreset : uint8_t {
    TopLeft,
    TopRight,
    BottomLeft,
    BottomRight,
    CenterLeft,
    CenterTop,
    CenterRight,
    CenterBottom,
    Center,
    LeftWide,
    TopWide,
    RightWide,
    BottomWide,
    VCenterWide,
    HCenterWide,
    FullRect
};

/// The four anchor values of a layout preset.
struct PresetAnchors {
    float left{};
    float top{};
    float right{};
    float bottom{};
};

/// The anchors that `preset` places on each side.
PresetAnchors preset_anchors(LayoutPreset preset) noexcept;

/// Which way a control grows when its minimum size exceeds its rect.
enum class GrowDirection : uint8_t { Begin, End, Both };

/// How a container sizes and places a child on one axis.
enum class SizeFlags : uint8_t {
    None = 0,
    Fill = 1,
    Expand = 2,
    ShrinkCenter = 4,
    ShrinkEnd = 8,
    ExpandFill = Fill | Expand
};

/// Two size flag sets combined.
constexpr SizeFlags operator|(SizeFlags a, SizeFlags b) noexcept {
    return static_cast<SizeFlags>(static_cast<uint8_t>(a) | static_cast<uint8_t>(b));
}

/// Whether any bit of `flag` is set in `value`.
constexpr bool has_flag(SizeFlags value, SizeFlags flag) noexcept {
    return (static_cast<uint8_t>(value) & static_cast<uint8_t>(flag)) != 0;
}

/// Base of the UI tree: a rectangle with anchors, a cached minimum size,
/// string-keyed theme lookup through the control and theme chains, and
/// virtual layout and drawing. Children are owned.
class Control {
public:
    Control();
    virtual ~Control() = default;

    Control(const Control&) = delete;
    Control& operator=(const Control&) = delete;

    /// Appends a child and takes ownership of it.
    Control& add_child(std::unique_ptr<Control> child);
    /// Constructs a child of type `T` in place and appends it.
    template <typename T, typename... Args> T& add(Args&&... args) {
        auto child = std::make_unique<T>(std::forward<Args>(args)...);
        T& reference = *child;
        add_child(std::move(child));
        return reference;
    }

    /// The children in draw order.
    const std::vector<std::unique_ptr<Control>>& children() const noexcept;
    /// The parent, or null for a root.
    Control* parent() const noexcept;
    /// The owning context, or null when detached.
    Context* ui() const noexcept;

    /// Sets one anchor, clamped to [0, 1], keeping the edge in place when `keep_offset` is set.
    void set_anchor(Side side, float value, bool keep_offset = true);
    /// Sets the distance of one edge from its anchor.
    void set_offset(Side side, float value);
    /// Sets all four anchors from `preset`.
    void set_anchors_preset(LayoutPreset preset, bool keep_offsets = true);
    /// Sets all four offsets so the control fits `preset` at its current or minimum size.
    void set_offsets_preset(LayoutPreset preset, float margin = 0.0f);
    /// Applies `preset` to both anchors and offsets.
    void set_anchors_and_offsets_preset(LayoutPreset preset, float margin = 0.0f);

    /// Sets position and size, not below the minimum size, without queuing a layout.
    void set_rect(const Rect2& rect);
    /// The position relative to the parent.
    Vec2 position() const noexcept;
    /// The size.
    Vec2 size() const noexcept;

    /// Sets a floor for the minimum size.
    void set_custom_minimum_size(Vec2 size);
    /// The larger of the custom and intrinsic minimum sizes, cached until invalidated.
    Vec2 combined_minimum_size() const;
    /// Invalidates the cached minimum size here and in every ancestor, and queues a layout.
    void update_minimum_size();

    /// Sets how a container sizes this control horizontally.
    void set_h_size_flags(SizeFlags flags);
    /// Sets how a container sizes this control vertically.
    void set_v_size_flags(SizeFlags flags);
    /// How a container sizes this control horizontally.
    SizeFlags h_size_flags() const noexcept;
    /// How a container sizes this control vertically.
    SizeFlags v_size_flags() const noexcept;
    /// Sets the share of spare space among expanding siblings, floored at zero.
    void set_stretch_ratio(float ratio);
    /// The share of spare space among expanding siblings.
    float stretch_ratio() const noexcept;
    /// Sets the horizontal grow direction.
    void set_h_grow_direction(GrowDirection direction);
    /// Sets the vertical grow direction.
    void set_v_grow_direction(GrowDirection direction);

    /// Shows or hides the control and its subtree.
    void set_visible(bool visible);
    /// Whether the control itself is visible.
    bool visible() const noexcept;
    /// Enables clipping of drawing to the control rect.
    void set_clip_contents(bool clip);
    /// Whether contents are clipped.
    bool clip_contents() const noexcept;
    /// Sets the colour multiplied into this control and its descendants.
    void set_modulate(Color4 modulate);
    /// The colour multiplied through every ancestor.
    Color4 effective_modulate() const noexcept;
    /// Marks the pointer as over this control.
    void set_hovered(bool hovered);
    /// Whether the pointer is over this control.
    bool hovered() const noexcept;
    /// Whether this control holds keyboard focus.
    bool has_focus() const noexcept;

    /// Sets a theme for this control and its descendants.
    void set_theme(std::shared_ptr<Theme> theme);
    /// The theme set on this control, if any.
    const std::shared_ptr<Theme>& theme() const noexcept;
    /// Sets a theme type searched before the control types.
    void set_theme_type_variation(std::string type);
    /// The theme type variation.
    const std::string& theme_type_variation() const noexcept;
    /// Overrides a colour by name for this control only.
    void add_color_override(std::string_view name, Color4 value);
    /// Overrides a constant by name for this control only.
    void add_constant_override(std::string_view name, float value);

    /// Resolves a style box by name through overrides, ancestor themes and the context theme.
    const StyleBox& theme_stylebox(std::string_view name) const;
    /// Resolves a colour by name through overrides, ancestor themes and the context theme.
    Color4 theme_color(std::string_view name) const;
    /// Resolves a constant by name through overrides, ancestor themes and the context theme.
    float theme_constant(std::string_view name) const;
    /// Resolves a font size by name through overrides, ancestor themes and the context theme.
    float theme_font_size(std::string_view name) const;

    /// The canvas of the owning context.
    const Canvas& canvas() const;
    /// Marks the owning context for layout on its next update.
    void queue_layout();

    /// Appends the theme type names used to look up this control's theme items, most specific first.
    virtual void collect_theme_types(std::vector<std::string_view>& out) const;
    /// The intrinsic minimum size; zero by default.
    virtual Vec2 get_minimum_size() const;
    /// Draws the control; does nothing by default.
    virtual void draw(Canvas& canvas) const;
    /// Positions the children; applies their anchors by default.
    virtual void layout_children();
    /// Called after the size changes.
    virtual void resized();

protected:
    /// Computes position and size from anchors and offsets inside `parent_size`.
    void apply_anchors(Vec2 parent_size);

private:
    friend class Context;

    /// Links the control to a parent and a context.
    void attach(Control* parent, Context* ui);
    /// Sets the owning context on this subtree.
    void propagate_ui(Context* ui);

    Control* parent_{};
    Context* ui_{};
    std::vector<std::unique_ptr<Control>> children_;

    float anchor_[4]{};
    float offset_[4]{};
    GrowDirection h_grow_{GrowDirection::End};
    GrowDirection v_grow_{GrowDirection::End};

    Vec2 position_{};
    Vec2 size_{};
    Vec2 custom_minimum_size_{};
    mutable Vec2 minimum_size_cache_{};
    mutable bool minimum_size_valid_{};

    SizeFlags h_size_flags_{SizeFlags::Fill};
    SizeFlags v_size_flags_{SizeFlags::Fill};
    float stretch_ratio_{1.0f};

    bool visible_{true};
    bool clip_contents_{};
    Color4 modulate_{};
    bool hovered_{};

    std::shared_ptr<Theme> theme_;
    std::string theme_type_variation_;
    StringMap<Color4> color_overrides_;
    StringMap<float> constant_overrides_;
};

/// The owner of a control tree: the root control, the fallback theme, the
/// canvas, the viewport and the dirty flag the per-frame update checks.
class Context {
public:
    /// A context with an empty root and the default theme that paints into `canvas`.
    explicit Context(Canvas& canvas);

    Context(const Context&) = delete;
    Context& operator=(const Context&) = delete;

    /// The canvas controls measure and draw with.
    Canvas& canvas() const noexcept;
    /// The root control.
    Control& root() noexcept;
    /// The fallback theme.
    const std::shared_ptr<Theme>& theme() const noexcept;
    /// Moves the keyboard focus to `control`, or clears it.
    void set_focus(Control* control) noexcept;
    /// The control holding keyboard focus, or null.
    Control* focused() const noexcept;
    /// Sets the viewport rectangle, queuing a layout when it changes.
    void set_viewport_rect(const Rect2& rect);
    /// The viewport rectangle.
    const Rect2& viewport_rect() const noexcept;
    /// Marks the tree for layout on the next update.
    void queue_layout() noexcept;
    /// Fits the root to the viewport and, when a layout is queued, lays out the tree.
    void update();
    /// Paints the tree into the canvas.
    void draw();

private:
    /// Lays out `control`, then its visible children.
    void layout(Control& control);
    /// Paints `control` and its subtree with `origin` as its parent's global position.
    void paint(Control& control, Vec2 origin);

    Canvas* canvas_{};
    std::unique_ptr<Control> root_;
    std::shared_ptr<Theme> theme_;
    Control* focus_{};
    Rect2 viewport_rect_{};
    bool layout_dirty_{true};
};

} // namespace bench::ui
