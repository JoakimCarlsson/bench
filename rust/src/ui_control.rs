//! The UI control tree. The tree is `Rc<Control>` nodes: a parent owns its
//! children in a `Vec<Rc<Control>>` and each child holds a `Weak` back to its
//! parent, which is how safe Rust spells the engine's `unique_ptr` children
//! and raw parent pointer. All mutable state sits in `Cell` and `RefCell`
//! fields so layout can walk the tree through shared references. The engine's
//! virtual methods are the `Widget` trait, held as a `Box<dyn Widget>` inside
//! each node, so every control is a base struct plus a widget, not a class
//! hierarchy. The context-wide state (font, fallback theme, dirty flag,
//! focus) lives in a `Shared` that every node points to, and a control's
//! identity for focus is a small integer id rather than a pointer.
use crate::ui_draw::{Canvas, Font, FontFace};
use crate::ui_math::{self, Color4, Rect2, Vec2};
use crate::ui_theme::{StringMap, StyleBox, Theme, default_theme};
use std::any::Any;
use std::cell::{Cell, Ref, RefCell};
use std::rc::{Rc, Weak};

const MAX_TYPE_DEPTH: usize = 8;

/// A side of a rectangle.
#[derive(Clone, Copy)]
pub enum Side {
    Left = 0,
    Top = 1,
    Right = 2,
    Bottom = 3,
}

/// Named anchor layouts such as corners, edges and full rect.
#[derive(Clone, Copy)]
#[allow(dead_code)]
pub enum LayoutPreset {
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
    FullRect,
}

/// The four anchor values of a layout preset.
struct PresetAnchors {
    left: f32,
    top: f32,
    right: f32,
    bottom: f32,
}

/// The anchors that `preset` places on each side.
fn preset_anchors(preset: LayoutPreset) -> PresetAnchors {
    let (left, top, right, bottom) = match preset {
        LayoutPreset::TopLeft => (0.0, 0.0, 0.0, 0.0),
        LayoutPreset::TopRight => (1.0, 0.0, 1.0, 0.0),
        LayoutPreset::BottomLeft => (0.0, 1.0, 0.0, 1.0),
        LayoutPreset::BottomRight => (1.0, 1.0, 1.0, 1.0),
        LayoutPreset::CenterLeft => (0.0, 0.5, 0.0, 0.5),
        LayoutPreset::CenterTop => (0.5, 0.0, 0.5, 0.0),
        LayoutPreset::CenterRight => (1.0, 0.5, 1.0, 0.5),
        LayoutPreset::CenterBottom => (0.5, 1.0, 0.5, 1.0),
        LayoutPreset::Center => (0.5, 0.5, 0.5, 0.5),
        LayoutPreset::LeftWide => (0.0, 0.0, 0.0, 1.0),
        LayoutPreset::TopWide => (0.0, 0.0, 1.0, 0.0),
        LayoutPreset::RightWide => (1.0, 0.0, 1.0, 1.0),
        LayoutPreset::BottomWide => (0.0, 1.0, 1.0, 1.0),
        LayoutPreset::VCenterWide => (0.0, 0.5, 1.0, 0.5),
        LayoutPreset::HCenterWide => (0.5, 0.0, 0.5, 1.0),
        LayoutPreset::FullRect => (0.0, 0.0, 1.0, 1.0),
    };
    PresetAnchors { left, top, right, bottom }
}

/// Which way a control grows when its minimum size exceeds its rect.
#[derive(Clone, Copy, PartialEq)]
#[allow(dead_code)]
pub enum GrowDirection {
    Begin,
    End,
    Both,
}

/// How a container sizes and places a child on one axis, a set of bit flags.
#[derive(Clone, Copy)]
pub struct SizeFlags(u8);

impl SizeFlags {
    pub const FILL: SizeFlags = SizeFlags(1);
    pub const EXPAND: SizeFlags = SizeFlags(2);
    pub const SHRINK_CENTER: SizeFlags = SizeFlags(4);
    pub const SHRINK_END: SizeFlags = SizeFlags(8);
    pub const EXPAND_FILL: SizeFlags = SizeFlags(3);

    /// Whether any bit of `flag` is set in these flags.
    pub fn has(self, flag: SizeFlags) -> bool {
        (self.0 & flag.0) != 0
    }
}

/// The state every control shares with its context: the font, the fallback
/// theme, the layout dirty flag and the focused control.
pub struct Shared {
    font: Font,
    theme: Rc<Theme>,
    layout_dirty: Cell<bool>,
    focus: Cell<u32>,
    next_id: Cell<u32>,
}

/// The behaviour that differs between controls, the engine's virtual methods.
/// `control` is the node the widget belongs to.
pub trait Widget: Any {
    /// Appends the theme type names used to look up this control's theme items, most specific first.
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>);

    /// The intrinsic minimum size; zero by default.
    fn get_minimum_size(&self, _control: &Control) -> Vec2 {
        Vec2::default()
    }

    /// Draws the control; does nothing by default.
    fn draw(&self, _control: &Control, _canvas: &mut dyn Canvas) {}

    /// Positions the children; applies their anchors by default.
    fn layout_children(&self, control: &Control) {
        for child in control.children().iter() {
            child.apply_anchors(control.size());
        }
    }

    /// Called after the size changes.
    fn resized(&self, _control: &Control) {}
}

/// Appends the type name every control ends its chain with.
pub fn control_types<'a>(out: &mut Vec<&'a str>) {
    out.push("Control");
}

/// The widget of a plain control with no behaviour of its own.
struct Plain;

impl Widget for Plain {
    fn collect_theme_types<'a>(&self, out: &mut Vec<&'a str>) {
        control_types(out);
    }
}

/// A node of the UI tree: a rectangle with anchors, a cached minimum size,
/// string-keyed theme lookup through the control and theme chains, and a widget
/// for layout and drawing.
pub struct Control {
    id: u32,
    shared: Rc<Shared>,
    parent: RefCell<Weak<Control>>,
    children: RefCell<Vec<Rc<Control>>>,
    widget: Box<dyn Widget>,

    anchor: [Cell<f32>; 4],
    offset: [Cell<f32>; 4],
    h_grow: Cell<GrowDirection>,
    v_grow: Cell<GrowDirection>,

    position: Cell<Vec2>,
    size: Cell<Vec2>,
    custom_minimum_size: Cell<Vec2>,
    minimum_size_cache: Cell<Vec2>,
    minimum_size_valid: Cell<bool>,

    h_size_flags: Cell<SizeFlags>,
    v_size_flags: Cell<SizeFlags>,
    stretch_ratio: Cell<f32>,

    visible: Cell<bool>,
    clip_contents: Cell<bool>,
    modulate: Cell<Color4>,
    hovered: Cell<bool>,

    theme: RefCell<Option<Rc<Theme>>>,
    theme_type_variation: RefCell<String>,
    color_overrides: RefCell<StringMap<Color4>>,
    constant_overrides: RefCell<StringMap<f32>>,
}

/// Walks `type_name` and its base types in `theme`, returning the first item `lookup` finds.
fn find_in_type_chain<V: Copy>(theme: &Theme, type_name: &str, name: &str, lookup: &impl Fn(&Theme, &str, &str) -> Option<V>) -> Option<V> {
    let mut current = type_name;
    for _ in 0..MAX_TYPE_DEPTH {
        if let Some(found) = lookup(theme, current, name) {
            return Some(found);
        }
        let base = theme.type_base(current);
        if base.is_empty() {
            break;
        }
        current = base;
    }
    None
}

/// Searches `owner`'s own theme, if it has one, for `name` under each of `types`.
fn find_in_owner<V: Copy>(owner: &Control, types: &[&str], name: &str, lookup: &impl Fn(&Theme, &str, &str) -> Option<V>) -> Option<V> {
    let theme = owner.theme.borrow();
    let theme = theme.as_ref()?;
    for type_name in types {
        if let Some(found) = find_in_type_chain(theme, type_name, name, lookup) {
            return Some(found);
        }
    }
    None
}

/// Looks up a theme item for a control, trying its type variation and theme
/// types up each base chain. Searches the control's own theme and its
/// ancestors' themes before the context's fallback theme.
fn find_in_themes<V: Copy>(control: &Control, name: &str, lookup: impl Fn(&Theme, &str, &str) -> Option<V>) -> Option<V> {
    let variation = control.theme_type_variation.borrow();
    let mut types: Vec<&str> = Vec::new();
    if !variation.is_empty() {
        types.push(variation.as_str());
    }
    control.widget.collect_theme_types(&mut types);

    if let Some(found) = find_in_owner(control, &types, name, &lookup) {
        return Some(found);
    }
    let mut current = control.parent();
    while let Some(owner) = current {
        if let Some(found) = find_in_owner(&owner, &types, name, &lookup) {
            return Some(found);
        }
        current = owner.parent();
    }

    for type_name in &types {
        if let Some(found) = find_in_type_chain(&control.shared.theme, type_name, name, &lookup) {
            return Some(found);
        }
    }
    None
}

impl Control {
    /// A control with full-size flags, a visible state and no parent.
    fn new(shared: &Rc<Shared>, widget: Box<dyn Widget>) -> Rc<Control> {
        let id = shared.next_id.get();
        shared.next_id.set(id + 1);
        Rc::new(Control {
            id,
            shared: Rc::clone(shared),
            parent: RefCell::new(Weak::new()),
            children: RefCell::new(Vec::new()),
            widget,
            anchor: Default::default(),
            offset: Default::default(),
            h_grow: Cell::new(GrowDirection::End),
            v_grow: Cell::new(GrowDirection::End),
            position: Cell::new(Vec2::default()),
            size: Cell::new(Vec2::default()),
            custom_minimum_size: Cell::new(Vec2::default()),
            minimum_size_cache: Cell::new(Vec2::default()),
            minimum_size_valid: Cell::new(false),
            h_size_flags: Cell::new(SizeFlags::FILL),
            v_size_flags: Cell::new(SizeFlags::FILL),
            stretch_ratio: Cell::new(1.0),
            visible: Cell::new(true),
            clip_contents: Cell::new(false),
            modulate: Cell::new(Color4::default()),
            hovered: Cell::new(false),
            theme: RefCell::new(None),
            theme_type_variation: RefCell::new(String::new()),
            color_overrides: RefCell::new(StringMap::default()),
            constant_overrides: RefCell::new(StringMap::default()),
        })
    }

    /// Appends `child` and takes ownership of it.
    pub fn add_child(self: &Rc<Control>, child: Rc<Control>) {
        *child.parent.borrow_mut() = Rc::downgrade(self);
        self.children.borrow_mut().push(child);
        self.update_minimum_size();
        self.queue_layout();
    }

    /// Constructs a child around `widget`, appends it and returns it.
    pub fn add<W: Widget>(self: &Rc<Control>, widget: W) -> Rc<Control> {
        let child = Control::new(&self.shared, Box::new(widget));
        self.add_child(Rc::clone(&child));
        child
    }

    /// The widget as its concrete type, which the caller must know.
    pub fn widget<T: Widget>(&self) -> &T {
        let widget: &dyn Any = self.widget.as_ref();
        widget.downcast_ref::<T>().expect("widget has the requested type")
    }

    /// The children in draw order.
    pub fn children(&self) -> Ref<'_, Vec<Rc<Control>>> {
        self.children.borrow()
    }

    /// The parent, or `None` for a root.
    pub fn parent(&self) -> Option<Rc<Control>> {
        self.parent.borrow().upgrade()
    }

    /// Sets one anchor, clamped to [0, 1], keeping the edge in place when `keep_offset` is set.
    fn set_anchor(&self, side: Side, value: f32, keep_offset: bool) {
        let index = side as usize;
        let previous = self.anchor[index].get();
        self.anchor[index].set(ui_math::clamp(value, 0.0, 1.0));
        if keep_offset {
            if let Some(parent) = self.parent() {
                let parent_size = parent.size();
                let area = if index % 2 == 0 { parent_size.x } else { parent_size.y };
                self.offset[index].set(self.offset[index].get() - (self.anchor[index].get() - previous) * area);
            }
        }
        self.queue_layout();
    }

    /// Sets all four anchors from `preset`.
    fn set_anchors_preset(&self, preset: LayoutPreset, keep_offsets: bool) {
        let anchors = preset_anchors(preset);
        self.set_anchor(Side::Left, anchors.left, keep_offsets);
        self.set_anchor(Side::Top, anchors.top, keep_offsets);
        self.set_anchor(Side::Right, anchors.right, keep_offsets);
        self.set_anchor(Side::Bottom, anchors.bottom, keep_offsets);
    }

    /// Sets all four offsets so the control fits `preset` at its current or minimum size.
    fn set_offsets_preset(&self, preset: LayoutPreset, margin: f32) {
        let anchors = preset_anchors(preset);
        let minimum = self.combined_minimum_size();
        let target = self.size().max(minimum);

        let stretch_h = anchors.left != anchors.right;
        let stretch_v = anchors.top != anchors.bottom;

        if stretch_h {
            self.offset[0].set(margin);
            self.offset[2].set(-margin);
        } else if anchors.left == 0.0 {
            self.offset[0].set(margin);
            self.offset[2].set(margin + target.x);
        } else if anchors.left == 1.0 {
            self.offset[0].set(-margin - target.x);
            self.offset[2].set(-margin);
        } else {
            self.offset[0].set(-target.x * 0.5);
            self.offset[2].set(target.x * 0.5);
        }

        if stretch_v {
            self.offset[1].set(margin);
            self.offset[3].set(-margin);
        } else if anchors.top == 0.0 {
            self.offset[1].set(margin);
            self.offset[3].set(margin + target.y);
        } else if anchors.top == 1.0 {
            self.offset[1].set(-margin - target.y);
            self.offset[3].set(-margin);
        } else {
            self.offset[1].set(-target.y * 0.5);
            self.offset[3].set(target.y * 0.5);
        }
        self.queue_layout();
    }

    /// Applies `preset` to both anchors and offsets.
    pub fn set_anchors_and_offsets_preset(&self, preset: LayoutPreset, margin: f32) {
        self.set_anchors_preset(preset, false);
        self.set_offsets_preset(preset, margin);
    }

    /// Sets position and size, not below the minimum size, without queuing a layout.
    pub fn set_rect(&self, rect: &Rect2) {
        let target = rect.size.max(self.combined_minimum_size());
        self.position.set(rect.position);
        self.offset[0].set(rect.position.x);
        self.offset[1].set(rect.position.y);
        self.offset[2].set(rect.position.x + target.x);
        self.offset[3].set(rect.position.y + target.y);
        if self.size.get() != target {
            self.size.set(target);
            self.widget.resized(self);
        }
    }

    /// The position relative to the parent.
    pub fn position(&self) -> Vec2 {
        self.position.get()
    }

    /// The size.
    pub fn size(&self) -> Vec2 {
        self.size.get()
    }

    /// Sets a floor for the minimum size.
    pub fn set_custom_minimum_size(&self, size: Vec2) {
        if self.custom_minimum_size.get() == size {
            return;
        }
        self.custom_minimum_size.set(size);
        self.update_minimum_size();
    }

    /// The larger of the custom and intrinsic minimum sizes, cached until invalidated.
    pub fn combined_minimum_size(&self) -> Vec2 {
        if !self.minimum_size_valid.get() {
            self.minimum_size_cache.set(self.custom_minimum_size.get().max(self.widget.get_minimum_size(self)));
            self.minimum_size_valid.set(true);
        }
        self.minimum_size_cache.get()
    }

    /// Invalidates the cached minimum size here and in every ancestor, and queues a layout.
    pub fn update_minimum_size(&self) {
        self.minimum_size_valid.set(false);
        let mut current = self.parent();
        while let Some(node) = current {
            node.minimum_size_valid.set(false);
            current = node.parent();
        }
        self.queue_layout();
    }

    /// Sets how a container sizes this control horizontally.
    pub fn set_h_size_flags(&self, flags: SizeFlags) {
        self.h_size_flags.set(flags);
        self.queue_layout();
    }

    /// Sets how a container sizes this control vertically.
    pub fn set_v_size_flags(&self, flags: SizeFlags) {
        self.v_size_flags.set(flags);
        self.queue_layout();
    }

    /// How a container sizes this control horizontally.
    pub fn h_size_flags(&self) -> SizeFlags {
        self.h_size_flags.get()
    }

    /// How a container sizes this control vertically.
    pub fn v_size_flags(&self) -> SizeFlags {
        self.v_size_flags.get()
    }

    /// Sets the share of spare space among expanding siblings, floored at zero.
    pub fn set_stretch_ratio(&self, ratio: f32) {
        self.stretch_ratio.set(ui_math::max(ratio, 0.0));
        self.queue_layout();
    }

    /// The share of spare space among expanding siblings.
    pub fn stretch_ratio(&self) -> f32 {
        self.stretch_ratio.get()
    }

    /// Shows or hides the control and its subtree.
    pub fn set_visible(&self, visible: bool) {
        if self.visible.get() == visible {
            return;
        }
        self.visible.set(visible);
        self.update_minimum_size();
    }

    /// Whether the control itself is visible.
    pub fn visible(&self) -> bool {
        self.visible.get()
    }

    /// Enables clipping of drawing to the control rect.
    pub fn set_clip_contents(&self, clip: bool) {
        self.clip_contents.set(clip);
    }

    /// Sets the colour multiplied into this control and its descendants.
    pub fn set_modulate(&self, modulate: Color4) {
        self.modulate.set(modulate);
    }

    /// The colour multiplied through every ancestor.
    pub fn effective_modulate(&self) -> Color4 {
        let mut result = self.modulate.get();
        let mut current = self.parent();
        while let Some(node) = current {
            result = result.modulated(node.modulate.get());
            current = node.parent();
        }
        result
    }

    /// Marks the pointer as over this control.
    pub fn set_hovered(&self, hovered: bool) {
        self.hovered.set(hovered);
    }

    /// Whether the pointer is over this control.
    pub fn hovered(&self) -> bool {
        self.hovered.get()
    }

    /// Whether this control holds keyboard focus.
    pub fn has_focus(&self) -> bool {
        self.shared.focus.get() == self.id
    }

    /// Sets a theme for this control and its descendants.
    pub fn set_theme(&self, theme: Rc<Theme>) {
        *self.theme.borrow_mut() = Some(theme);
        self.update_minimum_size();
    }

    /// Sets a theme type searched before the control types.
    pub fn set_theme_type_variation(&self, type_name: &str) {
        *self.theme_type_variation.borrow_mut() = type_name.to_string();
        self.update_minimum_size();
    }

    /// Overrides a constant by name for this control only.
    pub fn add_constant_override(&self, name: &str, value: f32) {
        self.constant_overrides.borrow_mut().insert(name.to_string(), value);
        self.update_minimum_size();
    }

    /// Resolves a style box by name through ancestor themes and the context theme.
    pub fn theme_stylebox(&self, name: &str) -> StyleBox {
        find_in_themes(self, name, |theme, type_name, item| theme.stylebox(type_name, item).copied()).unwrap_or_default()
    }

    /// Resolves a colour by name through overrides, ancestor themes and the context theme.
    pub fn theme_color(&self, name: &str) -> Color4 {
        if let Some(found) = self.color_overrides.borrow().get(name) {
            return *found;
        }
        find_in_themes(self, name, |theme, type_name, item| theme.color(type_name, item).copied()).unwrap_or_default()
    }

    /// Resolves a constant by name through overrides, ancestor themes and the context theme.
    pub fn theme_constant(&self, name: &str) -> f32 {
        if let Some(found) = self.constant_overrides.borrow().get(name) {
            return *found;
        }
        find_in_themes(self, name, |theme, type_name, item| theme.constant(type_name, item).copied()).unwrap_or(0.0)
    }

    /// Resolves a font size by name through ancestor themes and the context theme.
    pub fn theme_font_size(&self, name: &str) -> f32 {
        find_in_themes(self, name, |theme, type_name, item| theme.font_size(type_name, item).copied()).unwrap_or(16.0)
    }

    /// Size of `text` at `font_size`. Controls measure through the shared font
    /// because the canvas is mutably borrowed while they draw.
    pub fn measure_text(&self, text: &str, font_size: f32) -> Vec2 {
        self.shared.font.measure(text, font_size)
    }

    /// Vertical metrics at `font_size`.
    pub fn font_metrics(&self, font_size: f32) -> FontFace {
        self.shared.font.face(font_size)
    }

    /// Marks the owning context for layout on its next update.
    pub fn queue_layout(&self) {
        self.shared.layout_dirty.set(true);
    }

    /// Computes position and size from anchors and offsets inside `parent_size`.
    fn apply_anchors(&self, parent_size: Vec2) {
        let mut edges = [0.0f32; 4];
        for (i, edge) in edges.iter_mut().enumerate() {
            let area = if i % 2 == 0 { parent_size.x } else { parent_size.y };
            *edge = self.offset[i].get() + self.anchor[i].get() * area;
        }

        let mut new_position = Vec2::new(edges[0], edges[1]);
        let mut new_size = Vec2::new(edges[2] - edges[0], edges[3] - edges[1]);
        let minimum = self.combined_minimum_size();

        if minimum.x > new_size.x {
            if self.h_grow.get() == GrowDirection::Begin {
                new_position.x += new_size.x - minimum.x;
            } else if self.h_grow.get() == GrowDirection::Both {
                new_position.x += (new_size.x - minimum.x) * 0.5;
            }
            new_size.x = minimum.x;
        }
        if minimum.y > new_size.y {
            if self.v_grow.get() == GrowDirection::Begin {
                new_position.y += new_size.y - minimum.y;
            } else if self.v_grow.get() == GrowDirection::Both {
                new_position.y += (new_size.y - minimum.y) * 0.5;
            }
            new_size.y = minimum.y;
        }

        self.position.set(new_position);
        if self.size.get() != new_size {
            self.size.set(new_size);
            self.widget.resized(self);
        }
    }
}

/// The owner of a control tree: the root control, the viewport and the dirty
/// flag the per-frame update checks.
pub struct Context {
    shared: Rc<Shared>,
    root: Rc<Control>,
    viewport_rect: Cell<Rect2>,
}

impl Context {
    /// A context with an empty root and the default theme, measuring text through `font`.
    pub fn new(font: Font) -> Context {
        let shared = Rc::new(Shared {
            font,
            theme: Rc::new(default_theme()),
            layout_dirty: Cell::new(true),
            focus: Cell::new(0),
            next_id: Cell::new(1),
        });
        let root = Control::new(&shared, Box::new(Plain));
        Context { shared, root, viewport_rect: Cell::new(Rect2::default()) }
    }

    /// The root control.
    pub fn root(&self) -> &Rc<Control> {
        &self.root
    }

    /// Moves the keyboard focus to `control`.
    pub fn set_focus(&self, control: &Control) {
        self.shared.focus.set(control.id);
    }

    /// Sets the viewport rectangle, queuing a layout when it changes.
    pub fn set_viewport_rect(&self, rect: &Rect2) {
        if self.viewport_rect.get() == *rect {
            return;
        }
        self.viewport_rect.set(*rect);
        self.shared.layout_dirty.set(true);
    }

    /// The viewport rectangle.
    pub fn viewport_rect(&self) -> Rect2 {
        self.viewport_rect.get()
    }

    /// Fits the root to the viewport and, when a layout is queued, lays out the tree.
    pub fn update(&self) {
        self.root.set_rect(&self.viewport_rect.get());
        if !self.shared.layout_dirty.get() {
            return;
        }
        self.shared.layout_dirty.set(false);
        Context::layout(&self.root);
    }

    /// Paints the tree into `canvas`.
    pub fn draw(&self, canvas: &mut dyn Canvas) {
        Context::paint(&self.root, Vec2::default(), canvas);
    }

    /// Lays out `control`, then its visible children.
    fn layout(control: &Control) {
        control.widget.layout_children(control);
        for child in control.children().iter() {
            if child.visible() {
                Context::layout(child);
            }
        }
    }

    /// Paints `control` and its subtree with `origin` as its parent's global position.
    fn paint(control: &Control, origin: Vec2, canvas: &mut dyn Canvas) {
        if !control.visible() {
            return;
        }
        let position = origin + control.position();
        canvas.push_offset(control.position());
        if control.clip_contents.get() {
            canvas.push_clip(&Rect2::new(position, control.size()));
        }
        control.widget.draw(control, canvas);
        for child in control.children().iter() {
            Context::paint(child, position, canvas);
        }
        if control.clip_contents.get() {
            canvas.pop_clip();
        }
        canvas.pop_offset();
    }
}
