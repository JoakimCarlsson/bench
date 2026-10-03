#include "ui.hpp"

#include <array>
#include <string>
#include <utility>

#include "hash.hpp"

namespace bench {

namespace {

using namespace ui;

constexpr uint32_t frames = 3;
constexpr uint32_t toolbar_buttons = 28;
constexpr uint32_t hierarchy_rows = 1300;
constexpr uint32_t asset_cards = 300;
constexpr uint32_t log_lines = 200;
constexpr uint32_t inspector_sections = 130;
constexpr uint32_t section_rows = 14;

constexpr uint32_t label_period = 8;
constexpr uint32_t button_period = 8;
constexpr uint32_t edit_period = 6;
constexpr uint32_t resizable_period = 12;
constexpr uint32_t slider_period = 3;

constexpr uint64_t fold_multiplier = 0x9E3779B97F4A7C15ull;

constexpr std::array<const char*, 24> property_names{
    "Position", "Rotation", "Scale", "Mass", "Friction", "Restitution", "Linear Damping", "Angular Damping",
    "Gravity Scale", "Collision Layer", "Collision Mask", "Sleep Threshold", "Cast Shadows", "Receive Shadows",
    "Material", "Mesh", "Texture Region", "Tint", "Opacity", "Z Index", "Visible", "Name", "Tag", "Script"};

constexpr std::array<const char*, 12> section_names{
    "Transform", "Rigid Body", "Collider", "Mesh Renderer", "Material", "Audio Source",
    "Particle Emitter", "Animation Player", "Script", "Light", "Camera", "Navigation"};

constexpr std::array<const char*, 16> tool_names{
    "New", "Open", "Save", "Undo", "Redo", "Cut", "Copy", "Paste",
    "Play", "Pause", "Step", "Build", "Select", "Move", "Rotate", "Scale"};

constexpr std::array<const char*, 10> entity_names{
    "Player", "Crate", "Barrel", "Light", "Camera", "Spawner", "Trigger", "Door", "Terrain Chunk", "Emitter"};

constexpr std::array<const char*, 16> text_pool{
    "", "Mass", "Linear Velocity", "A rather long label for a property", "Two\nlines", "Collision Layer Mask",
    "x", "Position", "Angular Damping Factor", "Three\nline\nlabel", "Cast Shadows", "ID",
    "Texture Atlas Region", "Wi", "Material Override Slot", "Sleep Threshold Linear"};

/// The `index`-th entry of `table`, wrapping around.
template <size_t N> std::string named(const std::array<const char*, N>& table, size_t index) {
    return std::string{table[index % N]};
}

/// The `index`-th entry of `table` followed by a space and `number`.
template <size_t N> std::string numbered(const std::array<const char*, N>& table, size_t index, uint32_t number) {
    return std::string{table[index % N]} + " " + std::to_string(number);
}

/// Builds the editor screen and records the controls the frames change.
class Builder {
public:
    /// A builder drawing its choices from `rng` and recording targets in `targets`.
    Builder(Rng& rng, UiTargets& targets) : rng_{rng}, targets_{targets} {}

    /// Fills `root` with the backdrop, toolbar, body and status bar.
    void build(Control& root);

private:
    /// Whether the next draw from the generator lands on one in `period`.
    bool pick(uint32_t period);
    /// A decimal text such as `12.50` from the next draw of the generator.
    std::string decimal_text();
    /// Adds a label to `parent`, maybe recording it as a target.
    Label& label(Control& parent, std::string text);
    /// Adds a button to `parent`, maybe recording it as a target.
    Button& button(Control& parent, std::string text);
    /// Adds a text field to `parent`, maybe recording it as a target.
    LineEdit& line_edit(Control& parent, std::string text);
    /// Adds a slider to `parent` with a random value, maybe recording it as a target.
    Slider& slider(Control& parent);
    /// Adds a check box to `parent` with a random state.
    CheckBox& check_box(Control& parent, std::string text);
    /// Adds the toolbar to `screen`.
    void toolbar(Control& screen);
    /// Adds the status bar to `screen`.
    void status_bar(Control& screen);
    /// Adds the entity hierarchy to `body`.
    void hierarchy(Control& body);
    /// Adds the tab container to `body`.
    void tabs(Control& body);
    /// Adds the asset grid to `tabs`.
    void assets(TabContainer& tabs);
    /// Adds the log, profiler and settings tabs to `tabs`.
    void minor_tabs(TabContainer& tabs);
    /// Adds the inspector to `body`.
    void inspector(Control& body);
    /// Adds inspector section `index` to `list`.
    void section(Control& list, uint32_t index);
    /// Adds one property, a label and an editor, to `grid`.
    void property_row(GridContainer& grid, uint32_t section, uint32_t row);

    Rng& rng_;
    UiTargets& targets_;
};

bool Builder::pick(uint32_t period) {
    return rng_.next() % period == 0;
}

std::string Builder::decimal_text() {
    const uint64_t value = rng_.next() % 100000;
    return std::to_string(value % 1000) + "." + std::to_string(value / 1000);
}

Label& Builder::label(Control& parent, std::string text) {
    Label& made = parent.add<Label>(std::move(text));
    if (pick(label_period)) targets_.labels.push_back(&made);
    if (pick(resizable_period)) targets_.resizable.push_back(&made);
    return made;
}

Button& Builder::button(Control& parent, std::string text) {
    Button& made = parent.add<Button>(std::move(text));
    if (pick(button_period)) targets_.buttons.push_back(&made);
    if (pick(11)) made.set_hovered(true);
    if (pick(23)) made.set_disabled(true);
    return made;
}

LineEdit& Builder::line_edit(Control& parent, std::string text) {
    LineEdit& made = parent.add<LineEdit>();
    made.set_text(std::move(text));
    made.set_h_size_flags(SizeFlags::ExpandFill);
    if (pick(edit_period)) targets_.edits.push_back(&made);
    if (pick(17)) made.set_editable(false);
    return made;
}

Slider& Builder::slider(Control& parent) {
    Slider& made = parent.add<Slider>();
    const float fraction = rng_.unit();
    made.set_range(0.0, 100.0, static_cast<double>(fraction * 100.0f));
    made.set_h_size_flags(SizeFlags::ExpandFill);
    if (pick(slider_period)) targets_.sliders.push_back(&made);
    if (pick(resizable_period)) targets_.resizable.push_back(&made);
    return made;
}

CheckBox& Builder::check_box(Control& parent, std::string text) {
    CheckBox& made = parent.add<CheckBox>(std::move(text));
    made.set_pressed(pick(2));
    return made;
}

void Builder::toolbar(Control& screen) {
    PanelContainer& bar = screen.add<PanelContainer>();
    MarginContainer& margin = bar.add<MarginContainer>();
    margin.add_constant_override("margin_left", 6.0f);
    margin.add_constant_override("margin_top", 4.0f);
    margin.add_constant_override("margin_right", 6.0f);
    margin.add_constant_override("margin_bottom", 4.0f);
    HBoxContainer& row = margin.add<HBoxContainer>();
    for (uint32_t index = 0; index < toolbar_buttons; ++index) {
        Button& tool = button(row, named(tool_names, index));
        tool.set_icon(1 + index % 3, Vec2{16.0f, 16.0f});
        if (index % 7 == 6) label(row, "|");
    }
    for (uint32_t index = 0; index < 6; ++index) check_box(row, numbered(tool_names, index + 8, index));
    for (uint32_t index = 0; index < 4; ++index) {
        Slider& zoom = slider(row);
        zoom.set_custom_minimum_size(Vec2{120.0f, 0.0f});
    }
}

void Builder::status_bar(Control& screen) {
    HBoxContainer& row = screen.add<HBoxContainer>();
    for (uint32_t index = 0; index < 8; ++index) label(row, numbered(entity_names, index, index * 13));
    Label& fill = label(row, "Ready");
    fill.set_h_size_flags(SizeFlags::ExpandFill);
    fill.set_horizontal_alignment(HorizontalAlignment::Right);
}

void Builder::hierarchy(Control& body) {
    PanelContainer& panel = body.add<PanelContainer>();
    panel.set_custom_minimum_size(Vec2{300.0f, 0.0f});
    MarginContainer& margin = panel.add<MarginContainer>();
    margin.add_constant_override("margin_left", 4.0f);
    margin.add_constant_override("margin_right", 4.0f);
    VBoxContainer& rows = margin.add<VBoxContainer>();
    rows.add_constant_override("separation", 2.0f);
    for (uint32_t index = 0; index < hierarchy_rows; ++index) {
        HBoxContainer& row = rows.add<HBoxContainer>();
        if (index % 2 == 1) row.set_modulate(Color4{0.9f, 0.9f, 0.95f, 1.0f});
        const uint32_t depth = (index / 3 + index / 11) % 5;
        MarginContainer& indent = row.add<MarginContainer>();
        indent.add_constant_override("margin_left", 14.0f * static_cast<float>(depth));
        Button& arrow = button(indent, index % 4 == 0 ? ">" : "v");
        arrow.set_horizontal_alignment(HorizontalAlignment::Left);
        Label& name = label(row, numbered(entity_names, index, index));
        name.set_h_size_flags(SizeFlags::ExpandFill);
        check_box(row, "");
        button(row, "..");
    }
}

void Builder::assets(TabContainer& tabs) {
    GridContainer& grid = tabs.add<GridContainer>();
    grid.set_columns(8);
    for (uint32_t index = 0; index < asset_cards; ++index) {
        PanelContainer& card = grid.add<PanelContainer>();
        VBoxContainer& column = card.add<VBoxContainer>();
        CenterContainer& centre = column.add<CenterContainer>();
        Button& thumb = button(centre, named(tool_names, index));
        thumb.set_icon(1 + index % 3, Vec2{32.0f, 32.0f});
        label(column, numbered(entity_names, index, index));
    }
}

void Builder::minor_tabs(TabContainer& tabs) {
    VBoxContainer& log = tabs.add<VBoxContainer>();
    for (uint32_t index = 0; index < log_lines; ++index) label(log, numbered(property_names, index, index));

    VBoxContainer& profiler = tabs.add<VBoxContainer>();
    for (uint32_t index = 0; index < 10; ++index) {
        HBoxContainer& row = profiler.add<HBoxContainer>();
        label(row, named(section_names, index));
        slider(row);
    }

    GridContainer& settings = tabs.add<GridContainer>();
    settings.set_columns(2);
    for (uint32_t index = 0; index < 20; ++index) {
        check_box(settings, named(property_names, index));
        line_edit(settings, decimal_text());
    }
}

void Builder::tabs(Control& body) {
    TabContainer& container = body.add<TabContainer>();
    container.set_h_size_flags(SizeFlags::ExpandFill);
    container.set_stretch_ratio(2.0f);
    assets(container);
    minor_tabs(container);
    container.set_tab_title(0, "Assets");
    container.set_tab_title(1, "Console");
    container.set_tab_title(2, "Profiler");
    container.set_tab_title(3, "Settings");
    for (size_t index = 1; index < container.children().size(); ++index) container.children()[index]->set_visible(false);
}

void Builder::property_row(GridContainer& grid, uint32_t section, uint32_t row) {
    Label& name = label(grid, named(property_names, section * 7 + row));
    name.set_theme_type_variation("PropertyLabel");
    switch (row % 6) {
    case 0: {
        LineEdit& edit = line_edit(grid, decimal_text());
        edit.set_placeholder("empty");
        break;
    }
    case 1:
        slider(grid);
        break;
    case 2:
        check_box(grid, "Enabled");
        break;
    case 3: {
        HBoxContainer& vector = grid.add<HBoxContainer>();
        for (uint32_t axis = 0; axis < 3; ++axis) line_edit(vector, decimal_text());
        break;
    }
    case 4: {
        Button& browse = button(grid, "Browse");
        browse.set_icon(1 + row % 3, Vec2{16.0f, 16.0f});
        break;
    }
    default: {
        HBoxContainer& pair = grid.add<HBoxContainer>();
        slider(pair);
        line_edit(pair, decimal_text());
        break;
    }
    }
}

void Builder::section(Control& list, uint32_t index) {
    Button& header = button(list, numbered(section_names, index, index));
    header.set_theme_type_variation("SectionHeader");
    header.set_horizontal_alignment(HorizontalAlignment::Left);
    MarginContainer& body = list.add<MarginContainer>();
    body.add_constant_override("margin_left", 10.0f);
    body.add_constant_override("margin_bottom", 6.0f);
    body.set_clip_contents(true);
    GridContainer& grid = body.add<GridContainer>();
    grid.set_columns(2);
    for (uint32_t row = 0; row < section_rows; ++row) property_row(grid, index, row);
}

void Builder::inspector(Control& body) {
    PanelContainer& panel = body.add<PanelContainer>();
    panel.set_theme(inspector_theme());
    panel.set_custom_minimum_size(Vec2{420.0f, 0.0f});
    MarginContainer& margin = panel.add<MarginContainer>();
    margin.add_constant_override("margin_left", 6.0f);
    margin.add_constant_override("margin_top", 6.0f);
    margin.add_constant_override("margin_right", 6.0f);
    margin.add_constant_override("margin_bottom", 6.0f);
    VBoxContainer& list = margin.add<VBoxContainer>();
    for (uint32_t index = 0; index < inspector_sections; ++index) section(list, index);
}

void Builder::build(Control& root) {
    Panel& backdrop = root.add<Panel>();
    backdrop.set_anchors_and_offsets_preset(LayoutPreset::FullRect);
    VBoxContainer& screen = root.add<VBoxContainer>();
    screen.set_anchors_and_offsets_preset(LayoutPreset::FullRect);
    toolbar(screen);
    HBoxContainer& body = screen.add<HBoxContainer>();
    body.set_v_size_flags(SizeFlags::ExpandFill);
    hierarchy(body);
    tabs(body);
    inspector(body);
    status_bar(screen);
}

/// The viewport rectangle of `frame`.
Rect2 viewport_of(uint32_t frame) {
    const float step = static_cast<float>(frame);
    return Rect2{{0.0f, 0.0f}, {2560.0f + step * 24.0f, 65536.0f + step * 128.0f}};
}

/// Folds `value` into the running checksum `h`; cheaper than `hash_add` for bulk data.
constexpr uint64_t fold(uint64_t h, uint64_t value) {
    const uint64_t mixed = (h ^ value) * fold_multiplier;
    return mixed ^ (mixed >> 29);
}

/// Two floats as one 64-bit word.
uint64_t pack(float low, float high) {
    return static_cast<uint64_t>(f32_bits(low)) | (static_cast<uint64_t>(f32_bits(high)) << 32);
}

/// Folds the position and size of `control` and of every control below it.
uint64_t fold_tree(const Control& control, uint64_t h) {
    h = fold(h, pack(control.position().x, control.position().y));
    h = fold(h, pack(control.size().x, control.size().y));
    for (const std::unique_ptr<Control>& child : control.children()) h = fold_tree(*child, h);
    return h;
}

/// Folds every vertex, index and command of `data`.
uint64_t fold_draw(const DrawData& data, uint64_t h) {
    for (const DrawVertex& vertex : data.vertices) {
        h = fold(h, pack(vertex.position.x, vertex.position.y));
        h = fold(h, pack(vertex.uv.x, vertex.uv.y));
        h = fold(h, pack(vertex.rect_center.x, vertex.rect_center.y));
        h = fold(h, pack(vertex.rect_half.x, vertex.rect_half.y));
        h = fold(h, static_cast<uint64_t>(vertex.color) | (static_cast<uint64_t>(f32_bits(vertex.radius)) << 32));
        h = fold(h, static_cast<uint64_t>(f32_bits(vertex.stroke)) | (static_cast<uint64_t>(vertex.mode) << 32));
    }
    for (size_t index = 0; index + 1 < data.indices.size(); index += 2) {
        h = fold(h, static_cast<uint64_t>(data.indices[index]) | (static_cast<uint64_t>(data.indices[index + 1]) << 32));
    }
    for (const DrawCommand& command : data.commands) {
        h = fold(h, static_cast<uint64_t>(command.first_index) | (static_cast<uint64_t>(command.index_count) << 32));
        h = fold(h, pack(command.clip.position.x, command.clip.position.y));
        h = fold(h, pack(command.clip.size.x, command.clip.size.y));
        h = fold(h, command.texture);
    }
    return h;
}

} // namespace

Ui::Ui() : canvas_{font_}, context_{canvas_} {
    Rng rng{0x1b0};
    Builder builder{rng, targets_};
    builder.build(context_.root());
    if (!targets_.edits.empty()) context_.set_focus(targets_.edits.front());
    context_.set_viewport_rect(viewport_of(0));
    context_.update();
    canvas_.begin(context_.viewport_rect());
    context_.draw();
}

void Ui::mutate(uint32_t frame) {
    for (size_t k = 0; k < targets_.labels.size(); ++k) targets_.labels[k]->set_text(std::string{text_pool[(frame * 5 + k * 3) % text_pool.size()]});
    for (size_t k = 0; k < targets_.buttons.size(); ++k) targets_.buttons[k]->set_text(std::string{text_pool[(frame * 3 + k * 7) % text_pool.size()]});
    for (size_t k = 0; k < targets_.edits.size(); ++k) targets_.edits[k]->set_text(std::string{text_pool[(frame * 11 + k * 5) % text_pool.size()]});
    for (size_t k = 0; k < targets_.resizable.size(); ++k) {
        const float width = 90.0f + 20.0f * static_cast<float>((frame + k) % 4);
        targets_.resizable[k]->set_custom_minimum_size(Vec2{width, 0.0f});
    }
    for (size_t k = 0; k < targets_.sliders.size(); ++k) targets_.sliders[k]->set_value(static_cast<double>((frame * 37 + k * 11) % 100));
}

uint64_t Ui::run() {
    uint64_t h = 0;
    for (uint32_t frame = 0; frame < frames; ++frame) {
        mutate(frame);
        context_.set_viewport_rect(viewport_of(frame));
        context_.update();
        canvas_.begin(context_.viewport_rect());
        context_.draw();
        uint64_t frame_hash = fold_tree(context_.root(), 0);
        frame_hash = fold_draw(canvas_.data(), frame_hash);
        h = hash_add(h, frame_hash);
    }
    return h;
}

} // namespace bench
