//! String-keyed themes: style boxes, colours, constants and font sizes
//! grouped by type name, with type inheritance, plus the two themes the
//! kernel uses.
use crate::ui_draw::{Canvas, Stroke};
use crate::ui_math::{Color4, Rect2, Vec2};
use rustc_hash::FxHashMap;
use std::rc::Rc;

/// String-keyed hash map. `FxHashMap` rather than the default SipHash, as for the other kernels' maps.
pub type StringMap<T> = FxHashMap<String, T>;

const SURFACE: Color4 = Color4::new(0.12, 0.13, 0.15, 0.94);
const SURFACE_RAISED: Color4 = Color4::new(0.17, 0.19, 0.22, 1.0);
const SURFACE_HOVER: Color4 = Color4::new(0.22, 0.25, 0.29, 1.0);
const SURFACE_PRESSED: Color4 = Color4::new(0.10, 0.11, 0.13, 1.0);
const SURFACE_SUNKEN: Color4 = Color4::new(0.07, 0.08, 0.09, 1.0);
const OUTLINE: Color4 = Color4::new(0.30, 0.33, 0.38, 1.0);
const ACCENT: Color4 = Color4::new(0.29, 0.56, 0.89, 1.0);
const ACCENT_DIM: Color4 = Color4::new(0.29, 0.56, 0.89, 0.35);
const TEXT: Color4 = Color4::new(0.88, 0.90, 0.93, 1.0);
const TEXT_DIM: Color4 = Color4::new(0.58, 0.61, 0.66, 1.0);
const TEXT_DISABLED: Color4 = Color4::new(0.40, 0.42, 0.45, 1.0);

const BASE_FONT_SIZE: f32 = 16.0;
const BASE_RADIUS: f32 = 4.0;

/// Per-side distances.
#[derive(Clone, Copy, Default)]
pub struct Margins {
    pub left: f32,
    pub top: f32,
    pub right: f32,
    pub bottom: f32,
}

impl Margins {
    /// The summed horizontal and vertical margins.
    pub fn size(&self) -> Vec2 {
        Vec2::new(self.left + self.right, self.top + self.bottom)
    }
}

/// How a style box draws.
#[derive(Clone, Copy, Default, PartialEq)]
pub enum StyleBoxKind {
    #[default]
    Empty,
    Flat,
}

/// A drawable background with content margins.
#[derive(Clone, Copy, Default)]
pub struct StyleBox {
    pub kind: StyleBoxKind,
    pub color: Color4,
    pub border_color: Color4,
    pub border_width: f32,
    pub corner_radius: f32,
    pub content_margins: Margins,
    pub expand_margins: Margins,
}

impl StyleBox {
    /// `rect` shrunk by the content margins.
    pub fn content_rect(&self, rect: &Rect2) -> Rect2 {
        Rect2::new(
            rect.position + Vec2::new(self.content_margins.left, self.content_margins.top),
            (rect.size - self.content_margins.size()).max(Vec2::default()),
        )
    }

    /// Draws the style into `rect` grown by the expand margins, tinted by `modulate`.
    pub fn draw(&self, canvas: &mut dyn Canvas, rect: &Rect2, modulate: Color4) {
        if self.kind == StyleBoxKind::Empty {
            return;
        }
        let target = Rect2::new(
            rect.position - Vec2::new(self.expand_margins.left, self.expand_margins.top),
            rect.size + self.expand_margins.size(),
        );
        if self.color.a > 0.0 {
            canvas.fill_rect(&target, self.color.modulated(modulate), self.corner_radius);
        }
        if self.border_width > 0.0 && self.border_color.a > 0.0 {
            canvas.stroke_rect(&target, self.border_color.modulated(modulate), Stroke { width: self.border_width, corner_radius: self.corner_radius });
        }
    }
}

/// Margins with the same `amount` on every side.
fn uniform(amount: f32) -> Margins {
    Margins { left: amount, top: amount, right: amount, bottom: amount }
}

/// Margins with `horizontal` on the left and right and `vertical` on top and bottom.
fn padded(horizontal: f32, vertical: f32) -> Margins {
    Margins { left: horizontal, top: vertical, right: horizontal, bottom: vertical }
}

/// A flat style box of `color` with rounded corners and content margins.
fn flat_style(color: Color4, corner_radius: f32, content: Margins) -> StyleBox {
    StyleBox { kind: StyleBoxKind::Flat, color, corner_radius, content_margins: content, ..StyleBox::default() }
}

/// A style box that draws nothing, with content margins.
fn empty_style(content: Margins) -> StyleBox {
    StyleBox { kind: StyleBoxKind::Empty, content_margins: content, ..StyleBox::default() }
}

/// A text-field frame: sunken, outlined, with the given content margins.
fn edit_style(content: Margins) -> StyleBox {
    StyleBox { border_width: 1.0, border_color: OUTLINE, ..flat_style(SURFACE_SUNKEN, BASE_RADIUS, content) }
}

/// A button frame of `color`, outlined, with the given content margins.
fn button_style(color: Color4, content: Margins) -> StyleBox {
    StyleBox { border_width: 1.0, border_color: OUTLINE, ..flat_style(color, BASE_RADIUS, content) }
}

/// The items of one theme type.
#[derive(Default)]
struct TypeEntry {
    base: String,
    styleboxes: StringMap<StyleBox>,
    colors: StringMap<Color4>,
    constants: StringMap<f32>,
    font_sizes: StringMap<f32>,
}

/// Style boxes, colours, constants and font sizes grouped by type name, with type inheritance.
#[derive(Default)]
pub struct Theme {
    types: StringMap<TypeEntry>,
}

impl Theme {
    /// The entry for `type_name`, created when absent.
    fn entry(&mut self, type_name: &str) -> &mut TypeEntry {
        self.types.entry(type_name.to_string()).or_default()
    }

    /// Sets a style box for a type and name.
    pub fn set_stylebox(&mut self, type_name: &str, name: &str, value: StyleBox) {
        self.entry(type_name).styleboxes.insert(name.to_string(), value);
    }

    /// Sets a colour for a type and name.
    pub fn set_color(&mut self, type_name: &str, name: &str, value: Color4) {
        self.entry(type_name).colors.insert(name.to_string(), value);
    }

    /// Sets a constant for a type and name.
    pub fn set_constant(&mut self, type_name: &str, name: &str, value: f32) {
        self.entry(type_name).constants.insert(name.to_string(), value);
    }

    /// Sets a font size for a type and name.
    pub fn set_font_size(&mut self, type_name: &str, name: &str, value: f32) {
        self.entry(type_name).font_sizes.insert(name.to_string(), value);
    }

    /// Sets the base type of `type_name` to `base`.
    pub fn set_type_base(&mut self, type_name: &str, base: &str) {
        self.entry(type_name).base = base.to_string();
    }

    /// The style box for a type and name, if any.
    pub fn stylebox(&self, type_name: &str, name: &str) -> Option<&StyleBox> {
        self.types.get(type_name)?.styleboxes.get(name)
    }

    /// The colour for a type and name, if any.
    pub fn color(&self, type_name: &str, name: &str) -> Option<&Color4> {
        self.types.get(type_name)?.colors.get(name)
    }

    /// The constant for a type and name, if any.
    pub fn constant(&self, type_name: &str, name: &str) -> Option<&f32> {
        self.types.get(type_name)?.constants.get(name)
    }

    /// The font size for a type and name, if any.
    pub fn font_size(&self, type_name: &str, name: &str) -> Option<&f32> {
        self.types.get(type_name)?.font_sizes.get(name)
    }

    /// The base type of `type_name`, or an empty string when it has none.
    pub fn type_base(&self, type_name: &str) -> &str {
        match self.types.get(type_name) {
            Some(found) => &found.base,
            None => "",
        }
    }
}

/// The built-in dark theme.
pub fn default_theme() -> Theme {
    let mut theme = Theme::default();

    theme.set_font_size("Control", "font_size", BASE_FONT_SIZE);
    theme.set_color("Control", "font_color", TEXT);

    theme.set_stylebox("Panel", "panel", flat_style(SURFACE, BASE_RADIUS, uniform(8.0)));
    theme.set_stylebox("PanelContainer", "panel", flat_style(SURFACE, BASE_RADIUS, uniform(8.0)));

    theme.set_color("Label", "font_color", TEXT);
    theme.set_color("Label", "font_shadow_color", Color4::new(0.0, 0.0, 0.0, 0.0));
    theme.set_font_size("Label", "font_size", BASE_FONT_SIZE);
    theme.set_constant("Label", "line_spacing", 2.0);

    theme.set_constant("BoxContainer", "separation", 6.0);
    theme.set_constant("GridContainer", "h_separation", 6.0);
    theme.set_constant("GridContainer", "v_separation", 6.0);
    theme.set_constant("MarginContainer", "margin_left", 0.0);
    theme.set_constant("MarginContainer", "margin_top", 0.0);
    theme.set_constant("MarginContainer", "margin_right", 0.0);
    theme.set_constant("MarginContainer", "margin_bottom", 0.0);

    let button_normal = button_style(SURFACE_RAISED, padded(12.0, 6.0));
    let button_hover = StyleBox { color: SURFACE_HOVER, ..button_normal };
    let button_pressed = StyleBox { color: SURFACE_PRESSED, ..button_normal };
    let button_disabled = StyleBox { color: SURFACE_SUNKEN, border_color: OUTLINE.with_alpha(0.4), ..button_normal };
    let button_focus = StyleBox {
        kind: StyleBoxKind::Flat,
        color: Color4::new(0.0, 0.0, 0.0, 0.0),
        border_width: 1.0,
        border_color: ACCENT,
        corner_radius: BASE_RADIUS,
        ..empty_style(padded(12.0, 6.0))
    };

    theme.set_stylebox("Button", "normal", button_normal);
    theme.set_stylebox("Button", "hover", button_hover);
    theme.set_stylebox("Button", "pressed", button_pressed);
    theme.set_stylebox("Button", "disabled", button_disabled);
    theme.set_stylebox("Button", "focus", button_focus);
    theme.set_color("Button", "font_color", TEXT);
    theme.set_color("Button", "font_hover_color", TEXT);
    theme.set_color("Button", "font_pressed_color", TEXT);
    theme.set_color("Button", "font_disabled_color", TEXT_DISABLED);
    theme.set_font_size("Button", "font_size", BASE_FONT_SIZE);
    theme.set_constant("Button", "h_separation", 6.0);

    theme.set_stylebox("CheckBox", "normal", empty_style(padded(4.0, 4.0)));
    theme.set_stylebox("CheckBox", "focus", button_focus);
    theme.set_color("CheckBox", "font_color", TEXT);
    theme.set_color("CheckBox", "font_disabled_color", TEXT_DISABLED);
    theme.set_color("CheckBox", "box_color", SURFACE_SUNKEN);
    theme.set_color("CheckBox", "box_border_color", OUTLINE);
    theme.set_color("CheckBox", "check_color", ACCENT);
    theme.set_font_size("CheckBox", "font_size", BASE_FONT_SIZE);
    theme.set_constant("CheckBox", "h_separation", 8.0);
    theme.set_constant("CheckBox", "box_size", 16.0);

    theme.set_stylebox("Slider", "slider", flat_style(SURFACE_SUNKEN, 3.0, Margins::default()));
    theme.set_stylebox("Slider", "grabber_area", flat_style(ACCENT, 3.0, Margins::default()));
    theme.set_color("Slider", "grabber_color", TEXT);
    theme.set_color("Slider", "grabber_hover_color", ACCENT);
    theme.set_constant("Slider", "grabber_size", 14.0);
    theme.set_constant("Slider", "thickness", 6.0);

    let edit_normal = edit_style(padded(8.0, 5.0));
    let edit_focus = StyleBox { border_color: ACCENT, ..edit_normal };
    let edit_read_only = StyleBox { color: SURFACE, ..edit_normal };
    theme.set_stylebox("LineEdit", "normal", edit_normal);
    theme.set_stylebox("LineEdit", "focus", edit_focus);
    theme.set_stylebox("LineEdit", "read_only", edit_read_only);
    theme.set_color("LineEdit", "font_color", TEXT);
    theme.set_color("LineEdit", "placeholder_color", TEXT_DISABLED);
    theme.set_color("LineEdit", "selection_color", ACCENT_DIM);
    theme.set_color("LineEdit", "caret_color", TEXT);
    theme.set_font_size("LineEdit", "font_size", BASE_FONT_SIZE);
    theme.set_constant("LineEdit", "minimum_width", 80.0);
    theme.set_constant("LineEdit", "caret_width", 1.0);

    theme.set_stylebox("TabContainer", "panel", flat_style(SURFACE, BASE_RADIUS, uniform(8.0)));
    theme.set_stylebox("TabContainer", "tab_selected", flat_style(SURFACE, BASE_RADIUS, padded(12.0, 6.0)));
    theme.set_stylebox("TabContainer", "tab_unselected", flat_style(SURFACE_SUNKEN, BASE_RADIUS, padded(12.0, 6.0)));
    theme.set_color("TabContainer", "font_selected_color", TEXT);
    theme.set_color("TabContainer", "font_unselected_color", TEXT_DIM);
    theme.set_font_size("TabContainer", "font_size", BASE_FONT_SIZE);
    theme.set_constant("TabContainer", "h_separation", 2.0);

    theme
}

/// A theme for an inspector panel: tighter spacing, dimmer property labels
/// and two button variations that inherit from `Button` through type bases.
pub fn inspector_theme() -> Rc<Theme> {
    let mut theme = Theme::default();

    theme.set_constant("BoxContainer", "separation", 4.0);
    theme.set_constant("GridContainer", "h_separation", 8.0);
    theme.set_constant("GridContainer", "v_separation", 3.0);
    theme.set_font_size("Label", "font_size", 14.0);

    theme.set_type_base("PropertyLabel", "Label");
    theme.set_color("PropertyLabel", "font_color", TEXT_DIM);

    theme.set_type_base("SectionHeader", "Button");
    theme.set_stylebox("SectionHeader", "normal", button_style(SURFACE_RAISED, padded(8.0, 4.0)));
    theme.set_stylebox("SectionHeader", "hover", button_style(SURFACE_HOVER, padded(8.0, 4.0)));
    theme.set_color("SectionHeader", "font_color", ACCENT);

    theme.set_stylebox("LineEdit", "normal", edit_style(padded(6.0, 3.0)));
    theme.set_font_size("LineEdit", "font_size", 14.0);
    theme.set_constant("LineEdit", "minimum_width", 48.0);

    theme.set_constant("Slider", "grabber_size", 12.0);
    Rc::new(theme)
}
