//! The engine's game-facing UI: a tree of about sixteen thousand controls
//! (an editor screen with a toolbar, a long hierarchy, a tabbed asset grid
//! and an inspector with long property lists) laid out and drawn into a
//! batched vertex, index and command list. Each of the frames resizes the
//! root, changes the text and minimum size of some controls so the
//! invalidation climbs the tree, runs the layout pass, draws the whole
//! tree, and folds every control rect and all draw data into the checksum.
//!
//! The tree is `Rc<Control>` nodes with `Weak` parents and a `Box<dyn Widget>`
//! per node; see `ui_control`.
use crate::harness::Case;
use crate::hash::{self, Rng};
use crate::ui_container::{BoxContainer, CenterContainer, GridContainer, MarginContainer, PanelContainer, TabContainer};
use crate::ui_control::{Context, Control, LayoutPreset, SizeFlags};
use crate::ui_draw::{DrawCanvas, DrawData, Font};
use crate::ui_math::{Color4, Rect2, Vec2};
use crate::ui_theme::inspector_theme;
use crate::ui_widgets::{Button, CheckBox, HorizontalAlignment, Label, LineEdit, Panel, Slider};
use std::rc::Rc;

const FRAMES: u32 = 3;
const TOOLBAR_BUTTONS: u32 = 28;
const HIERARCHY_ROWS: u32 = 1300;
const ASSET_CARDS: u32 = 300;
const LOG_LINES: u32 = 200;
const INSPECTOR_SECTIONS: u32 = 130;
const SECTION_ROWS: u32 = 14;

const LABEL_PERIOD: u32 = 8;
const BUTTON_PERIOD: u32 = 8;
const EDIT_PERIOD: u32 = 6;
const RESIZABLE_PERIOD: u32 = 12;
const SLIDER_PERIOD: u32 = 3;

const FOLD_MULTIPLIER: u64 = 0x9E37_79B9_7F4A_7C15;

const PROPERTY_NAMES: [&str; 24] = [
    "Position", "Rotation", "Scale", "Mass", "Friction", "Restitution", "Linear Damping", "Angular Damping",
    "Gravity Scale", "Collision Layer", "Collision Mask", "Sleep Threshold", "Cast Shadows", "Receive Shadows",
    "Material", "Mesh", "Texture Region", "Tint", "Opacity", "Z Index", "Visible", "Name", "Tag", "Script",
];

const SECTION_NAMES: [&str; 12] = [
    "Transform", "Rigid Body", "Collider", "Mesh Renderer", "Material", "Audio Source",
    "Particle Emitter", "Animation Player", "Script", "Light", "Camera", "Navigation",
];

const TOOL_NAMES: [&str; 16] = [
    "New", "Open", "Save", "Undo", "Redo", "Cut", "Copy", "Paste",
    "Play", "Pause", "Step", "Build", "Select", "Move", "Rotate", "Scale",
];

const ENTITY_NAMES: [&str; 10] = [
    "Player", "Crate", "Barrel", "Light", "Camera", "Spawner", "Trigger", "Door", "Terrain Chunk", "Emitter",
];

const TEXT_POOL: [&str; 16] = [
    "", "Mass", "Linear Velocity", "A rather long label for a property", "Two\nlines", "Collision Layer Mask",
    "x", "Position", "Angular Damping Factor", "Three\nline\nlabel", "Cast Shadows", "ID",
    "Texture Atlas Region", "Wi", "Material Override Slot", "Sleep Threshold Linear",
];

/// The controls the frames change, picked while the screen is built.
#[derive(Default)]
struct Targets {
    labels: Vec<Rc<Control>>,
    buttons: Vec<Rc<Control>>,
    edits: Vec<Rc<Control>>,
    resizable: Vec<Rc<Control>>,
    sliders: Vec<Rc<Control>>,
}

/// The `index`-th entry of `table`, wrapping around.
fn named(table: &[&str], index: usize) -> String {
    table[index % table.len()].to_string()
}

/// The `index`-th entry of `table` followed by a space and `number`.
fn numbered(table: &[&str], index: usize, number: u32) -> String {
    format!("{} {}", table[index % table.len()], number)
}

/// Builds the editor screen and records the controls the frames change.
struct Builder<'a> {
    rng: Rng,
    targets: &'a mut Targets,
}

impl Builder<'_> {
    /// Whether the next draw from the generator lands on one in `period`.
    fn pick(&mut self, period: u32) -> bool {
        self.rng.next() % u64::from(period) == 0
    }

    /// A decimal text such as `12.50` from the next draw of the generator.
    fn decimal_text(&mut self) -> String {
        let value = self.rng.next() % 100000;
        format!("{}.{}", value % 1000, value / 1000)
    }

    /// Adds `widget` to `parent`, maybe recording it as a target.
    fn label(&mut self, parent: &Rc<Control>, widget: Label) -> Rc<Control> {
        let made = parent.add(widget);
        if self.pick(LABEL_PERIOD) {
            self.targets.labels.push(Rc::clone(&made));
        }
        if self.pick(RESIZABLE_PERIOD) {
            self.targets.resizable.push(Rc::clone(&made));
        }
        made
    }

    /// Adds `widget` to `parent`, maybe recording it as a target, hovering it or disabling it.
    fn button(&mut self, parent: &Rc<Control>, mut widget: Button) -> Rc<Control> {
        let target = self.pick(BUTTON_PERIOD);
        let hovered = self.pick(11);
        let disabled = self.pick(23);
        widget.set_disabled(disabled);
        let made = parent.add(widget);
        if target {
            self.targets.buttons.push(Rc::clone(&made));
        }
        if hovered {
            made.set_hovered(true);
        }
        made
    }

    /// Adds a text field showing `text` to `parent`, maybe recording it as a target or making it read only.
    fn line_edit(&mut self, parent: &Rc<Control>, text: String) -> Rc<Control> {
        let target = self.pick(EDIT_PERIOD);
        let read_only = self.pick(17);
        let made = parent.add(LineEdit::new(read_only));
        made.widget::<LineEdit>().set_text(&made, text);
        made.set_h_size_flags(SizeFlags::EXPAND_FILL);
        if target {
            self.targets.edits.push(Rc::clone(&made));
        }
        made
    }

    /// Adds a slider with a random value to `parent`, maybe recording it as a target.
    fn slider(&mut self, parent: &Rc<Control>) -> Rc<Control> {
        let fraction = self.rng.unit();
        let target = self.pick(SLIDER_PERIOD);
        let resizable = self.pick(RESIZABLE_PERIOD);
        let made = parent.add(Slider::new().with_range(0.0, 100.0, f64::from(fraction * 100.0)));
        made.set_h_size_flags(SizeFlags::EXPAND_FILL);
        if target {
            self.targets.sliders.push(Rc::clone(&made));
        }
        if resizable {
            self.targets.resizable.push(Rc::clone(&made));
        }
        made
    }

    /// Adds a check box labelled `text` with a random state to `parent`.
    fn check_box(&mut self, parent: &Rc<Control>, text: String) -> Rc<Control> {
        let pressed = self.pick(2);
        parent.add(CheckBox::new(text, pressed))
    }

    /// Adds the toolbar to `screen`.
    fn toolbar(&mut self, screen: &Rc<Control>) {
        let bar = screen.add(PanelContainer);
        let margin = bar.add(MarginContainer);
        margin.add_constant_override("margin_left", 6.0);
        margin.add_constant_override("margin_top", 4.0);
        margin.add_constant_override("margin_right", 6.0);
        margin.add_constant_override("margin_bottom", 4.0);
        let row = margin.add(BoxContainer::new(false));
        for index in 0..TOOLBAR_BUTTONS {
            let tool = Button::new(named(&TOOL_NAMES, index as usize)).with_icon(u64::from(1 + index % 3), Vec2::new(16.0, 16.0));
            self.button(&row, tool);
            if index % 7 == 6 {
                self.label(&row, Label::new("|".to_string()));
            }
        }
        for index in 0..6u32 {
            self.check_box(&row, numbered(&TOOL_NAMES, (index + 8) as usize, index));
        }
        for _ in 0..4 {
            let zoom = self.slider(&row);
            zoom.set_custom_minimum_size(Vec2::new(120.0, 0.0));
        }
    }

    /// Adds the status bar to `screen`.
    fn status_bar(&mut self, screen: &Rc<Control>) {
        let row = screen.add(BoxContainer::new(false));
        for index in 0..8u32 {
            self.label(&row, Label::new(numbered(&ENTITY_NAMES, index as usize, index * 13)));
        }
        let fill = self.label(&row, Label::new("Ready".to_string()).with_horizontal_alignment(HorizontalAlignment::Right));
        fill.set_h_size_flags(SizeFlags::EXPAND_FILL);
    }

    /// Adds the entity hierarchy to `body`.
    fn hierarchy(&mut self, body: &Rc<Control>) {
        let panel = body.add(PanelContainer);
        panel.set_custom_minimum_size(Vec2::new(300.0, 0.0));
        let margin = panel.add(MarginContainer);
        margin.add_constant_override("margin_left", 4.0);
        margin.add_constant_override("margin_right", 4.0);
        let rows = margin.add(BoxContainer::new(true));
        rows.add_constant_override("separation", 2.0);
        for index in 0..HIERARCHY_ROWS {
            let row = rows.add(BoxContainer::new(false));
            if index % 2 == 1 {
                row.set_modulate(Color4::new(0.9, 0.9, 0.95, 1.0));
            }
            let depth = (index / 3 + index / 11) % 5;
            let indent = row.add(MarginContainer);
            indent.add_constant_override("margin_left", 14.0 * depth as f32);
            let arrow = Button::new((if index % 4 == 0 { ">" } else { "v" }).to_string()).with_horizontal_alignment(HorizontalAlignment::Left);
            self.button(&indent, arrow);
            let name = self.label(&row, Label::new(numbered(&ENTITY_NAMES, index as usize, index)));
            name.set_h_size_flags(SizeFlags::EXPAND_FILL);
            self.check_box(&row, String::new());
            self.button(&row, Button::new("..".to_string()));
        }
    }

    /// Adds the asset grid to `tabs`.
    fn assets(&mut self, tabs: &Rc<Control>) {
        let grid = tabs.add(GridContainer::new(8));
        for index in 0..ASSET_CARDS {
            let card = grid.add(PanelContainer);
            let column = card.add(BoxContainer::new(true));
            let centre = column.add(CenterContainer);
            let thumb = Button::new(named(&TOOL_NAMES, index as usize)).with_icon(u64::from(1 + index % 3), Vec2::new(32.0, 32.0));
            self.button(&centre, thumb);
            self.label(&column, Label::new(numbered(&ENTITY_NAMES, index as usize, index)));
        }
    }

    /// Adds the log, profiler and settings tabs to `tabs`.
    fn minor_tabs(&mut self, tabs: &Rc<Control>) {
        let log = tabs.add(BoxContainer::new(true));
        for index in 0..LOG_LINES {
            self.label(&log, Label::new(numbered(&PROPERTY_NAMES, index as usize, index)));
        }

        let profiler = tabs.add(BoxContainer::new(true));
        for index in 0..10usize {
            let row = profiler.add(BoxContainer::new(false));
            self.label(&row, Label::new(named(&SECTION_NAMES, index)));
            self.slider(&row);
        }

        let settings = tabs.add(GridContainer::new(2));
        for index in 0..20usize {
            self.check_box(&settings, named(&PROPERTY_NAMES, index));
            let text = self.decimal_text();
            self.line_edit(&settings, text);
        }
    }

    /// Adds the tab container to `body`.
    fn tabs(&mut self, body: &Rc<Control>) {
        let titles = ["Assets", "Console", "Profiler", "Settings"].iter().map(|title| title.to_string()).collect();
        let container = body.add(TabContainer::new(titles));
        container.set_h_size_flags(SizeFlags::EXPAND_FILL);
        container.set_stretch_ratio(2.0);
        self.assets(&container);
        self.minor_tabs(&container);
        for child in container.children().iter().skip(1) {
            child.set_visible(false);
        }
    }

    /// Adds one property, a label and an editor, to `grid`.
    fn property_row(&mut self, grid: &Rc<Control>, section: u32, row: u32) {
        let name = self.label(grid, Label::new(named(&PROPERTY_NAMES, (section * 7 + row) as usize)));
        name.set_theme_type_variation("PropertyLabel");
        match row % 6 {
            0 => {
                let text = self.decimal_text();
                let edit = self.line_edit(grid, text);
                edit.widget::<LineEdit>().set_placeholder("empty");
            }
            1 => {
                self.slider(grid);
            }
            2 => {
                self.check_box(grid, "Enabled".to_string());
            }
            3 => {
                let vector = grid.add(BoxContainer::new(false));
                for _ in 0..3 {
                    let text = self.decimal_text();
                    self.line_edit(&vector, text);
                }
            }
            4 => {
                let browse = Button::new("Browse".to_string()).with_icon(u64::from(1 + row % 3), Vec2::new(16.0, 16.0));
                self.button(grid, browse);
            }
            _ => {
                let pair = grid.add(BoxContainer::new(false));
                self.slider(&pair);
                let text = self.decimal_text();
                self.line_edit(&pair, text);
            }
        }
    }

    /// Adds inspector section `index` to `list`.
    fn section(&mut self, list: &Rc<Control>, index: u32) {
        let header = Button::new(numbered(&SECTION_NAMES, index as usize, index)).with_horizontal_alignment(HorizontalAlignment::Left);
        let header = self.button(list, header);
        header.set_theme_type_variation("SectionHeader");
        let body = list.add(MarginContainer);
        body.add_constant_override("margin_left", 10.0);
        body.add_constant_override("margin_bottom", 6.0);
        body.set_clip_contents(true);
        let grid = body.add(GridContainer::new(2));
        for row in 0..SECTION_ROWS {
            self.property_row(&grid, index, row);
        }
    }

    /// Adds the inspector to `body`.
    fn inspector(&mut self, body: &Rc<Control>) {
        let panel = body.add(PanelContainer);
        panel.set_theme(inspector_theme());
        panel.set_custom_minimum_size(Vec2::new(420.0, 0.0));
        let margin = panel.add(MarginContainer);
        margin.add_constant_override("margin_left", 6.0);
        margin.add_constant_override("margin_top", 6.0);
        margin.add_constant_override("margin_right", 6.0);
        margin.add_constant_override("margin_bottom", 6.0);
        let list = margin.add(BoxContainer::new(true));
        for index in 0..INSPECTOR_SECTIONS {
            self.section(&list, index);
        }
    }

    /// Fills `root` with the backdrop, toolbar, body and status bar.
    fn build(&mut self, root: &Rc<Control>) {
        let backdrop = root.add(Panel);
        backdrop.set_anchors_and_offsets_preset(LayoutPreset::FullRect, 0.0);
        let screen = root.add(BoxContainer::new(true));
        screen.set_anchors_and_offsets_preset(LayoutPreset::FullRect, 0.0);
        self.toolbar(&screen);
        let body = screen.add(BoxContainer::new(false));
        body.set_v_size_flags(SizeFlags::EXPAND_FILL);
        self.hierarchy(&body);
        self.tabs(&body);
        self.inspector(&body);
        self.status_bar(&screen);
    }
}

/// The viewport rectangle of `frame`.
fn viewport_of(frame: u32) -> Rect2 {
    let step = frame as f32;
    Rect2::new(Vec2::new(0.0, 0.0), Vec2::new(2560.0 + step * 24.0, 65536.0 + step * 128.0))
}

/// Folds `value` into the running checksum `h`; cheaper than `hash::add` for bulk data.
fn fold(h: u64, value: u64) -> u64 {
    let mixed = (h ^ value).wrapping_mul(FOLD_MULTIPLIER);
    mixed ^ (mixed >> 29)
}

/// Two floats as one 64-bit word.
fn pack(low: f32, high: f32) -> u64 {
    u64::from(hash::f32_bits(low)) | (u64::from(hash::f32_bits(high)) << 32)
}

/// Folds the position and size of `control` and of every control below it.
fn fold_tree(control: &Control, mut h: u64) -> u64 {
    h = fold(h, pack(control.position().x, control.position().y));
    h = fold(h, pack(control.size().x, control.size().y));
    for child in control.children().iter() {
        h = fold_tree(child, h);
    }
    h
}

/// Folds every vertex, index and command of `data`.
fn fold_draw(data: &DrawData, mut h: u64) -> u64 {
    for vertex in &data.vertices {
        h = fold(h, pack(vertex.position.x, vertex.position.y));
        h = fold(h, pack(vertex.uv.x, vertex.uv.y));
        h = fold(h, pack(vertex.rect_center.x, vertex.rect_center.y));
        h = fold(h, pack(vertex.rect_half.x, vertex.rect_half.y));
        h = fold(h, u64::from(vertex.color) | (u64::from(hash::f32_bits(vertex.radius)) << 32));
        h = fold(h, u64::from(hash::f32_bits(vertex.stroke)) | (u64::from(vertex.mode) << 32));
    }
    for pair in data.indices.chunks_exact(2) {
        h = fold(h, u64::from(pair[0]) | (u64::from(pair[1]) << 32));
    }
    for command in &data.commands {
        h = fold(h, u64::from(command.first_index) | (u64::from(command.index_count) << 32));
        h = fold(h, pack(command.clip.position.x, command.clip.position.y));
        h = fold(h, pack(command.clip.size.x, command.clip.size.y));
        h = fold(h, command.texture);
    }
    h
}

/// The screen, its canvas and the controls the frames change.
pub struct Ui {
    canvas: DrawCanvas,
    context: Context,
    targets: Targets,
}

impl Ui {
    /// Changes the text, minimum sizes and slider values of the targets for `frame`.
    fn mutate(&self, frame: u32) {
        let step = frame as usize;
        for (k, label) in self.targets.labels.iter().enumerate() {
            label.widget::<Label>().set_text(label, TEXT_POOL[(step * 5 + k * 3) % TEXT_POOL.len()].to_string());
        }
        for (k, button) in self.targets.buttons.iter().enumerate() {
            button.widget::<Button>().set_text(button, TEXT_POOL[(step * 3 + k * 7) % TEXT_POOL.len()].to_string());
        }
        for (k, edit) in self.targets.edits.iter().enumerate() {
            edit.widget::<LineEdit>().set_text(edit, TEXT_POOL[(step * 11 + k * 5) % TEXT_POOL.len()].to_string());
        }
        for (k, control) in self.targets.resizable.iter().enumerate() {
            let width = 90.0 + 20.0 * ((step + k) % 4) as f32;
            control.set_custom_minimum_size(Vec2::new(width, 0.0));
        }
        for (k, slider) in self.targets.sliders.iter().enumerate() {
            slider.widget::<Slider>().set_value(((step * 37 + k * 11) % 100) as f64);
        }
    }
}

impl Case for Ui {
    const NAME: &'static str = "ui";

    /// Builds the screen and lays it out and draws it once.
    fn init() -> Ui {
        let context = Context::new(Font);
        let mut targets = Targets::default();
        Builder { rng: Rng::new(0x1b0), targets: &mut targets }.build(context.root());
        if let Some(edit) = targets.edits.first() {
            context.set_focus(edit);
        }
        let mut canvas = DrawCanvas::new(Font);
        context.set_viewport_rect(&viewport_of(0));
        context.update();
        canvas.begin(&context.viewport_rect());
        context.draw(&mut canvas);
        Ui { canvas, context, targets }
    }

    /// Runs every frame: mutate, resize, lay out, draw, fold.
    fn run(&mut self) -> u64 {
        let mut h = 0u64;
        for frame in 0..FRAMES {
            self.mutate(frame);
            self.context.set_viewport_rect(&viewport_of(frame));
            self.context.update();
            self.canvas.begin(&self.context.viewport_rect());
            self.context.draw(&mut self.canvas);
            let mut frame_hash = fold_tree(self.context.root(), 0);
            frame_hash = fold_draw(self.canvas.data(), frame_hash);
            h = hash::add(h, frame_hash);
        }
        h
    }
}
