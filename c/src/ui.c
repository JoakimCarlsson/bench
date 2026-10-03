#include "ui.h"

#include <stdlib.h>

#include "array.h"
#include "hash.h"
#include "ui_container.h"
#include "ui_control.h"
#include "ui_draw.h"
#include "ui_widgets.h"

enum {
    FRAMES = 3,
    TOOLBAR_BUTTONS = 28,
    HIERARCHY_ROWS = 1300,
    ASSET_CARDS = 300,
    LOG_LINES = 200,
    INSPECTOR_SECTIONS = 130,
    SECTION_ROWS = 14,
    LABEL_PERIOD = 8,
    BUTTON_PERIOD = 8,
    EDIT_PERIOD = 6,
    RESIZABLE_PERIOD = 12,
    SLIDER_PERIOD = 3,
    TEXT_CAPACITY = 96
};

static const uint64_t fold_multiplier = 0x9E3779B97F4A7C15ull;

static const char* const property_names[24] = {
    "Position", "Rotation", "Scale", "Mass", "Friction", "Restitution", "Linear Damping", "Angular Damping",
    "Gravity Scale", "Collision Layer", "Collision Mask", "Sleep Threshold", "Cast Shadows", "Receive Shadows",
    "Material", "Mesh", "Texture Region", "Tint", "Opacity", "Z Index", "Visible", "Name", "Tag", "Script"
};

static const char* const section_names[12] = {
    "Transform", "Rigid Body", "Collider", "Mesh Renderer", "Material", "Audio Source",
    "Particle Emitter", "Animation Player", "Script", "Light", "Camera", "Navigation"
};

static const char* const tool_names[16] = {
    "New", "Open", "Save", "Undo", "Redo", "Cut", "Copy", "Paste",
    "Play", "Pause", "Step", "Build", "Select", "Move", "Rotate", "Scale"
};

static const char* const entity_names[10] = {
    "Player", "Crate", "Barrel", "Light", "Camera", "Spawner", "Trigger", "Door", "Terrain Chunk", "Emitter"
};

static const char* const text_pool[16] = {
    "", "Mass", "Linear Velocity", "A rather long label for a property", "Two\nlines", "Collision Layer Mask",
    "x", "Position", "Angular Damping Factor", "Three\nline\nlabel", "Cast Shadows", "ID",
    "Texture Atlas Region", "Wi", "Material Override Slot", "Sleep Threshold Linear"
};

/// The number of entries of a table.
#define COUNT(table) (sizeof(table) / sizeof((table)[0]))

/// The `index`-th entry of `table`, wrapping around.
#define NAMED(table, index) named((table), COUNT(table), (index))

/// The `index`-th entry of `table` followed by a space and `number`, written into `buffer`.
#define NUMBERED(buffer, table, index, number) numbered((buffer), (table), COUNT(table), (index), (number))

/// The controls the frames of the `ui` kernel change, picked while the screen is built.
typedef struct {
    ARRAY_OF(Label*) labels;
    ARRAY_OF(Button*) buttons;
    ARRAY_OF(LineEdit*) edits;
    ARRAY_OF(Control*) resizable;
    ARRAY_OF(Slider*) sliders;
} UiTargets;

/// The whole kernel state: font, canvas, context and the targets.
typedef struct {
    Font font;
    DrawCanvas canvas;
    Context context;
    UiTargets targets;
} Ui;

/// Builds the editor screen from a generator and records the controls the frames change.
typedef struct {
    Rng* rng;
    UiTargets* targets;
} Builder;

/// A small fixed buffer that texts such as `Mass 12` are assembled in.
typedef struct {
    char data[TEXT_CAPACITY];
    size_t size;
} TextBuffer;

/// The `index`-th entry of `table`, wrapping around.
static StrView named(const char* const* table, size_t count, size_t index) { return ui_sv_of(table[index % count]); }

/// Appends `text` to `buffer`.
static void text_append(TextBuffer* buffer, StrView text) {
    memcpy(buffer->data + buffer->size, text.data, text.size);
    buffer->size += text.size;
}

/// Appends the decimal digits of `value` to `buffer`.
static void text_append_number(TextBuffer* buffer, uint64_t value) {
    char digits[20];
    size_t count = 0;
    do {
        digits[count++] = (char)('0' + value % 10);
        value /= 10;
    } while (value != 0);
    while (count != 0) buffer->data[buffer->size++] = digits[--count];
}

/// A view of the text in `buffer`.
static StrView text_view(const TextBuffer* buffer) { return (StrView){ buffer->data, buffer->size }; }

/// The `index`-th entry of `table` followed by a space and `number`, written into `buffer`.
static StrView numbered(TextBuffer* buffer, const char* const* table, size_t count, size_t index, uint32_t number) {
    buffer->size = 0;
    text_append(buffer, named(table, count, index));
    text_append(buffer, UI_SV(" "));
    text_append_number(buffer, number);
    return text_view(buffer);
}

/// Whether the next draw from the generator lands on one in `period`.
static bool pick(Builder* builder, uint32_t period) { return rng_next(builder->rng) % period == 0; }

/// A decimal text such as `12.50` from the next draw of the generator, written into `buffer`.
static StrView decimal_text(Builder* builder, TextBuffer* buffer) {
    uint64_t value = rng_next(builder->rng) % 100000;
    buffer->size = 0;
    text_append_number(buffer, value % 1000);
    text_append(buffer, UI_SV("."));
    text_append_number(buffer, value / 1000);
    return text_view(buffer);
}

/// Adds a label to `parent`, maybe recording it as a target.
static Label* builder_label(Builder* builder, Control* parent, StrView text) {
    Label* made = ui_label_add(parent, text);
    if (pick(builder, LABEL_PERIOD)) ARRAY_PUSH(builder->targets->labels, made);
    if (pick(builder, RESIZABLE_PERIOD)) ARRAY_PUSH(builder->targets->resizable, UI_CONTROL(made));
    return made;
}

/// Adds a button to `parent`, maybe recording it as a target.
static Button* builder_button(Builder* builder, Control* parent, StrView text) {
    Button* made = ui_button_add(parent, text);
    if (pick(builder, BUTTON_PERIOD)) ARRAY_PUSH(builder->targets->buttons, made);
    if (pick(builder, 11)) ui_control_set_hovered(UI_CONTROL(made), true);
    if (pick(builder, 23)) ui_base_button_set_disabled(&made->base, true);
    return made;
}

/// Adds a text field to `parent`, maybe recording it as a target.
static LineEdit* builder_line_edit(Builder* builder, Control* parent, StrView text) {
    LineEdit* made = ui_line_edit_add(parent);
    ui_line_edit_set_text(made, text);
    ui_control_set_h_size_flags(UI_CONTROL(made), UI_SIZE_EXPAND_FILL);
    if (pick(builder, EDIT_PERIOD)) ARRAY_PUSH(builder->targets->edits, made);
    if (pick(builder, 17)) ui_line_edit_set_editable(made, false);
    return made;
}

/// Adds a text field showing the next decimal text to `parent`.
static LineEdit* builder_decimal_edit(Builder* builder, Control* parent) {
    TextBuffer buffer;
    StrView text = decimal_text(builder, &buffer);
    return builder_line_edit(builder, parent, text);
}

/// Adds a slider to `parent` with a random value, maybe recording it as a target.
static Slider* builder_slider(Builder* builder, Control* parent) {
    Slider* made = ui_slider_add(parent, false);
    float fraction = rng_unit(builder->rng);
    ui_range_set_range(&made->base, 0.0, 100.0, (double)(fraction * 100.0f));
    ui_control_set_h_size_flags(UI_CONTROL(made), UI_SIZE_EXPAND_FILL);
    if (pick(builder, SLIDER_PERIOD)) ARRAY_PUSH(builder->targets->sliders, made);
    if (pick(builder, RESIZABLE_PERIOD)) ARRAY_PUSH(builder->targets->resizable, UI_CONTROL(made));
    return made;
}

/// Adds a check box to `parent` with a random state.
static CheckBox* builder_check_box(Builder* builder, Control* parent, StrView text) {
    CheckBox* made = ui_check_box_add(parent, text);
    bool pressed = pick(builder, 2);
    ui_base_button_set_pressed(&made->base, pressed);
    return made;
}

/// Adds the toolbar to `screen`.
static void builder_toolbar(Builder* builder, Control* screen) {
    PanelContainer* bar = ui_panel_container_add(screen);
    MarginContainer* margin = ui_margin_container_add(UI_CONTROL(bar));
    ui_control_add_constant_override(UI_CONTROL(margin), UI_SV("margin_left"), 6.0f);
    ui_control_add_constant_override(UI_CONTROL(margin), UI_SV("margin_top"), 4.0f);
    ui_control_add_constant_override(UI_CONTROL(margin), UI_SV("margin_right"), 6.0f);
    ui_control_add_constant_override(UI_CONTROL(margin), UI_SV("margin_bottom"), 4.0f);
    Control* row = UI_CONTROL(ui_hbox_container_add(UI_CONTROL(margin)));
    for (uint32_t index = 0; index < TOOLBAR_BUTTONS; ++index) {
        Button* tool = builder_button(builder, row, NAMED(tool_names, index));
        ui_button_set_icon(tool, 1 + index % 3, ui_vec2(16.0f, 16.0f));
        if (index % 7 == 6) builder_label(builder, row, UI_SV("|"));
    }
    for (uint32_t index = 0; index < 6; ++index) {
        TextBuffer buffer;
        builder_check_box(builder, row, NUMBERED(&buffer, tool_names, index + 8, index));
    }
    for (uint32_t index = 0; index < 4; ++index) {
        Slider* zoom = builder_slider(builder, row);
        ui_control_set_custom_minimum_size(UI_CONTROL(zoom), ui_vec2(120.0f, 0.0f));
    }
}

/// Adds the status bar to `screen`.
static void builder_status_bar(Builder* builder, Control* screen) {
    Control* row = UI_CONTROL(ui_hbox_container_add(screen));
    for (uint32_t index = 0; index < 8; ++index) {
        TextBuffer buffer;
        builder_label(builder, row, NUMBERED(&buffer, entity_names, index, index * 13));
    }
    Label* fill = builder_label(builder, row, UI_SV("Ready"));
    ui_control_set_h_size_flags(UI_CONTROL(fill), UI_SIZE_EXPAND_FILL);
    ui_label_set_horizontal_alignment(fill, UI_ALIGN_RIGHT);
}

/// Adds the entity hierarchy to `body`.
static void builder_hierarchy(Builder* builder, Control* body) {
    PanelContainer* panel = ui_panel_container_add(body);
    ui_control_set_custom_minimum_size(UI_CONTROL(panel), ui_vec2(300.0f, 0.0f));
    MarginContainer* margin = ui_margin_container_add(UI_CONTROL(panel));
    ui_control_add_constant_override(UI_CONTROL(margin), UI_SV("margin_left"), 4.0f);
    ui_control_add_constant_override(UI_CONTROL(margin), UI_SV("margin_right"), 4.0f);
    BoxContainer* rows = ui_vbox_container_add(UI_CONTROL(margin));
    ui_control_add_constant_override(UI_CONTROL(rows), UI_SV("separation"), 2.0f);
    for (uint32_t index = 0; index < HIERARCHY_ROWS; ++index) {
        Control* row = UI_CONTROL(ui_hbox_container_add(UI_CONTROL(rows)));
        if (index % 2 == 1) ui_control_set_modulate(row, (Color4){ 0.9f, 0.9f, 0.95f, 1.0f });
        uint32_t depth = (index / 3 + index / 11) % 5;
        MarginContainer* indent = ui_margin_container_add(row);
        ui_control_add_constant_override(UI_CONTROL(indent), UI_SV("margin_left"), 14.0f * (float)depth);
        Button* arrow = builder_button(builder, UI_CONTROL(indent), index % 4 == 0 ? UI_SV(">") : UI_SV("v"));
        ui_button_set_horizontal_alignment(arrow, UI_ALIGN_LEFT);
        TextBuffer buffer;
        Label* name = builder_label(builder, row, NUMBERED(&buffer, entity_names, index, index));
        ui_control_set_h_size_flags(UI_CONTROL(name), UI_SIZE_EXPAND_FILL);
        builder_check_box(builder, row, UI_SV(""));
        builder_button(builder, row, UI_SV(".."));
    }
}

/// Adds the asset grid to `tabs`.
static void builder_assets(Builder* builder, TabContainer* tabs) {
    GridContainer* grid = ui_grid_container_add(UI_CONTROL(tabs));
    ui_grid_container_set_columns(grid, 8);
    for (uint32_t index = 0; index < ASSET_CARDS; ++index) {
        PanelContainer* card = ui_panel_container_add(UI_CONTROL(grid));
        Control* column = UI_CONTROL(ui_vbox_container_add(UI_CONTROL(card)));
        CenterContainer* centre = ui_center_container_add(column);
        Button* thumb = builder_button(builder, UI_CONTROL(centre), NAMED(tool_names, index));
        ui_button_set_icon(thumb, 1 + index % 3, ui_vec2(32.0f, 32.0f));
        TextBuffer buffer;
        builder_label(builder, column, NUMBERED(&buffer, entity_names, index, index));
    }
}

/// Adds the log, profiler and settings tabs to `tabs`.
static void builder_minor_tabs(Builder* builder, TabContainer* tabs) {
    Control* log = UI_CONTROL(ui_vbox_container_add(UI_CONTROL(tabs)));
    for (uint32_t index = 0; index < LOG_LINES; ++index) {
        TextBuffer buffer;
        builder_label(builder, log, NUMBERED(&buffer, property_names, index, index));
    }

    Control* profiler = UI_CONTROL(ui_vbox_container_add(UI_CONTROL(tabs)));
    for (uint32_t index = 0; index < 10; ++index) {
        Control* row = UI_CONTROL(ui_hbox_container_add(profiler));
        builder_label(builder, row, NAMED(section_names, index));
        builder_slider(builder, row);
    }

    GridContainer* settings = ui_grid_container_add(UI_CONTROL(tabs));
    ui_grid_container_set_columns(settings, 2);
    for (uint32_t index = 0; index < 20; ++index) {
        builder_check_box(builder, UI_CONTROL(settings), NAMED(property_names, index));
        builder_decimal_edit(builder, UI_CONTROL(settings));
    }
}

/// Adds the tab container to `body`.
static void builder_tabs(Builder* builder, Control* body) {
    TabContainer* container = ui_tab_container_add(body);
    ui_control_set_h_size_flags(UI_CONTROL(container), UI_SIZE_EXPAND_FILL);
    ui_control_set_stretch_ratio(UI_CONTROL(container), 2.0f);
    builder_assets(builder, container);
    builder_minor_tabs(builder, container);
    ui_tab_container_set_tab_title(container, 0, UI_SV("Assets"));
    ui_tab_container_set_tab_title(container, 1, UI_SV("Console"));
    ui_tab_container_set_tab_title(container, 2, UI_SV("Profiler"));
    ui_tab_container_set_tab_title(container, 3, UI_SV("Settings"));
    for (size_t index = 1; index < container->base.children.len; ++index) {
        ui_control_set_visible(container->base.children.data[index], false);
    }
}

/// Adds one property, a label and an editor, to `grid`.
static void builder_property_row(Builder* builder, GridContainer* grid, uint32_t section, uint32_t row) {
    Control* parent = UI_CONTROL(grid);
    Label* name = builder_label(builder, parent, NAMED(property_names, section * 7 + row));
    ui_control_set_theme_type_variation(UI_CONTROL(name), UI_SV("PropertyLabel"));
    switch (row % 6) {
    case 0: {
        LineEdit* edit = builder_decimal_edit(builder, parent);
        ui_line_edit_set_placeholder(edit, UI_SV("empty"));
        break;
    }
    case 1:
        builder_slider(builder, parent);
        break;
    case 2:
        builder_check_box(builder, parent, UI_SV("Enabled"));
        break;
    case 3: {
        Control* vector = UI_CONTROL(ui_hbox_container_add(parent));
        for (uint32_t axis = 0; axis < 3; ++axis) builder_decimal_edit(builder, vector);
        break;
    }
    case 4: {
        Button* browse = builder_button(builder, parent, UI_SV("Browse"));
        ui_button_set_icon(browse, 1 + row % 3, ui_vec2(16.0f, 16.0f));
        break;
    }
    default: {
        Control* pair = UI_CONTROL(ui_hbox_container_add(parent));
        builder_slider(builder, pair);
        builder_decimal_edit(builder, pair);
        break;
    }
    }
}

/// Adds inspector section `index` to `list`.
static void builder_section(Builder* builder, Control* list, uint32_t index) {
    TextBuffer buffer;
    Button* header = builder_button(builder, list, NUMBERED(&buffer, section_names, index, index));
    ui_control_set_theme_type_variation(UI_CONTROL(header), UI_SV("SectionHeader"));
    ui_button_set_horizontal_alignment(header, UI_ALIGN_LEFT);
    MarginContainer* body = ui_margin_container_add(list);
    ui_control_add_constant_override(UI_CONTROL(body), UI_SV("margin_left"), 10.0f);
    ui_control_add_constant_override(UI_CONTROL(body), UI_SV("margin_bottom"), 6.0f);
    ui_control_set_clip_contents(UI_CONTROL(body), true);
    GridContainer* grid = ui_grid_container_add(UI_CONTROL(body));
    ui_grid_container_set_columns(grid, 2);
    for (uint32_t row = 0; row < SECTION_ROWS; ++row) builder_property_row(builder, grid, index, row);
}

/// Adds the inspector to `body`.
static void builder_inspector(Builder* builder, Control* body) {
    PanelContainer* panel = ui_panel_container_add(body);
    ui_control_set_theme(UI_CONTROL(panel), ui_inspector_theme());
    ui_control_set_custom_minimum_size(UI_CONTROL(panel), ui_vec2(420.0f, 0.0f));
    MarginContainer* margin = ui_margin_container_add(UI_CONTROL(panel));
    ui_control_add_constant_override(UI_CONTROL(margin), UI_SV("margin_left"), 6.0f);
    ui_control_add_constant_override(UI_CONTROL(margin), UI_SV("margin_top"), 6.0f);
    ui_control_add_constant_override(UI_CONTROL(margin), UI_SV("margin_right"), 6.0f);
    ui_control_add_constant_override(UI_CONTROL(margin), UI_SV("margin_bottom"), 6.0f);
    Control* list = UI_CONTROL(ui_vbox_container_add(UI_CONTROL(margin)));
    for (uint32_t index = 0; index < INSPECTOR_SECTIONS; ++index) builder_section(builder, list, index);
}

/// Fills `root` with the backdrop, toolbar, body and status bar.
static void builder_build(Builder* builder, Control* root) {
    Panel* backdrop = ui_panel_add(root);
    ui_control_set_anchors_and_offsets_preset(UI_CONTROL(backdrop), UI_PRESET_FULL_RECT, 0.0f);
    Control* screen = UI_CONTROL(ui_vbox_container_add(root));
    ui_control_set_anchors_and_offsets_preset(screen, UI_PRESET_FULL_RECT, 0.0f);
    builder_toolbar(builder, screen);
    Control* body = UI_CONTROL(ui_hbox_container_add(screen));
    ui_control_set_v_size_flags(body, UI_SIZE_EXPAND_FILL);
    builder_hierarchy(builder, body);
    builder_tabs(builder, body);
    builder_inspector(builder, body);
    builder_status_bar(builder, screen);
}

/// The viewport rectangle of `frame`.
static Rect2 viewport_of(uint32_t frame) {
    float step = (float)frame;
    return ui_rect2(ui_vec2(0.0f, 0.0f), ui_vec2(2560.0f + step * 24.0f, 65536.0f + step * 128.0f));
}

/// Folds `value` into the running checksum `h`; cheaper than `hash_add` for bulk data.
static uint64_t fold(uint64_t h, uint64_t value) {
    uint64_t mixed = (h ^ value) * fold_multiplier;
    return mixed ^ (mixed >> 29);
}

/// Two floats as one 64-bit word.
static uint64_t pack(float low, float high) { return (uint64_t)f32_bits(low) | ((uint64_t)f32_bits(high) << 32); }

/// Folds the position and size of `control` and of every control below it.
static uint64_t fold_tree(const Control* control, uint64_t h) {
    h = fold(h, pack(control->position.x, control->position.y));
    h = fold(h, pack(control->size.x, control->size.y));
    for (size_t index = 0; index < control->children.len; ++index) h = fold_tree(control->children.data[index], h);
    return h;
}

/// Folds every vertex, index and command of `data`.
static uint64_t fold_draw(const DrawData* data, uint64_t h) {
    for (size_t index = 0; index < data->vertices.len; ++index) {
        const DrawVertex* vertex = &data->vertices.data[index];
        h = fold(h, pack(vertex->position.x, vertex->position.y));
        h = fold(h, pack(vertex->uv.x, vertex->uv.y));
        h = fold(h, pack(vertex->rect_center.x, vertex->rect_center.y));
        h = fold(h, pack(vertex->rect_half.x, vertex->rect_half.y));
        h = fold(h, (uint64_t)vertex->color | ((uint64_t)f32_bits(vertex->radius) << 32));
        h = fold(h, (uint64_t)f32_bits(vertex->stroke) | ((uint64_t)vertex->mode << 32));
    }
    for (size_t index = 0; index + 1 < data->indices.len; index += 2) {
        h = fold(h, (uint64_t)data->indices.data[index] | ((uint64_t)data->indices.data[index + 1] << 32));
    }
    for (size_t index = 0; index < data->commands.len; ++index) {
        const DrawCommand* command = &data->commands.data[index];
        h = fold(h, (uint64_t)command->first_index | ((uint64_t)command->index_count << 32));
        h = fold(h, pack(command->clip.position.x, command->clip.position.y));
        h = fold(h, pack(command->clip.size.x, command->clip.size.y));
        h = fold(h, command->texture);
    }
    return h;
}

/// Builds the screen, then lays out and draws it once.
static void setup(void* state) {
    Ui* ui = state;
    ui->font = ui_font_make();
    ui_draw_canvas_init(&ui->canvas, &ui->font);
    ui_context_init(&ui->context, &ui->canvas.base);
    Rng rng = { 0x1b0 };
    Builder builder = { &rng, &ui->targets };
    builder_build(&builder, ui->context.root);
    if (ui->targets.edits.len != 0) ui_context_set_focus(&ui->context, UI_CONTROL(ui->targets.edits.data[0]));
    ui_context_set_viewport_rect(&ui->context, viewport_of(0));
    ui_context_update(&ui->context);
    ui_draw_canvas_begin(&ui->canvas, ui->context.viewport_rect);
    ui_context_draw(&ui->context);
}

/// Changes the text, minimum sizes and slider values of the targets for `frame`.
static void mutate(Ui* ui, uint32_t frame) {
    UiTargets* targets = &ui->targets;
    for (size_t k = 0; k < targets->labels.len; ++k) {
        ui_label_set_text(targets->labels.data[k], ui_sv_of(text_pool[(frame * 5 + k * 3) % COUNT(text_pool)]));
    }
    for (size_t k = 0; k < targets->buttons.len; ++k) {
        ui_button_set_text(targets->buttons.data[k], ui_sv_of(text_pool[(frame * 3 + k * 7) % COUNT(text_pool)]));
    }
    for (size_t k = 0; k < targets->edits.len; ++k) {
        ui_line_edit_set_text(targets->edits.data[k], ui_sv_of(text_pool[(frame * 11 + k * 5) % COUNT(text_pool)]));
    }
    for (size_t k = 0; k < targets->resizable.len; ++k) {
        float width = 90.0f + 20.0f * (float)((frame + k) % 4);
        ui_control_set_custom_minimum_size(targets->resizable.data[k], ui_vec2(width, 0.0f));
    }
    for (size_t k = 0; k < targets->sliders.len; ++k) {
        ui_range_set_value(&targets->sliders.data[k]->base, (double)((frame * 37 + k * 11) % 100));
    }
}

/// Runs the frames and hashes the control rects and draw data of each.
static uint64_t run(void* state) {
    Ui* ui = state;
    uint64_t h = 0;
    for (uint32_t frame = 0; frame < FRAMES; ++frame) {
        mutate(ui, frame);
        ui_context_set_viewport_rect(&ui->context, viewport_of(frame));
        ui_context_update(&ui->context);
        ui_draw_canvas_begin(&ui->canvas, ui->context.viewport_rect);
        ui_context_draw(&ui->context);
        uint64_t frame_hash = fold_tree(ui->context.root, 0);
        frame_hash = fold_draw(&ui->canvas.data, frame_hash);
        h = hash_add(h, frame_hash);
    }
    return h;
}

/// Frees the tree, the context theme, the canvas and the target lists.
static void teardown(void* state) {
    Ui* ui = state;
    ui_context_destroy(&ui->context);
    ui_draw_canvas_destroy(&ui->canvas);
    ARRAY_FREE(ui->targets.labels);
    ARRAY_FREE(ui->targets.buttons);
    ARRAY_FREE(ui->targets.edits);
    ARRAY_FREE(ui->targets.resizable);
    ARRAY_FREE(ui->targets.sliders);
}

const Case ui_case = { "ui", sizeof(Ui), setup, run, teardown };
