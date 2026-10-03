//! Scene load: the engine's JSON reader over a generated scene document of a
//! few megabytes. Init writes the text with integer arithmetic only, so it is
//! the same bytes in every language. Each run parses it into a value tree
//! with the engine's strict recursive descent parser, walks the entities the
//! way the entity document code does (components looked up by key, vectors
//! and transforms read from number arrays), folds everything it reads into a
//! checksum, then drops the tree.
use crate::harness::Case;
use crate::hash::{self, Rng};
use crate::json_value::{self, Value};

const ROOT_ENTITIES: u32 = 520;
const MAX_ENTITY_LEVEL: u32 = 3;
const RNG_SEED: u64 = 0x15011;

const WORDS: [&str; 16] = [
    "stone", "iron", "lantern", "crate", "torch", "golem", "bridge", "tower", "grass", "water", "ember",
    "crystal", "door", "statue", "spawn", "beacon",
];

const ESCAPES: [&str; 8] = ["\\n", "\\\"", "\\\\", "\\u00e9", "\\u4e2d", "\\/", "\\t", "\\u20AC"];

const SHORT_NUMBERS: [&str; 4] = ["-0.5", "0.5", "0.25", "1.0"];

const PROPERTY_TYPES: [&str; 5] = ["string", "float", "vec3", "bool", "entity"];

pub struct Json {
    text: String,
}

/// The document text under construction: indentation depth and the layout
/// style of the entity being written (0 pretty, 1 pretty with inline number
/// arrays, 2 compact, 3 tabs and CRLF).
struct Writer {
    out: Vec<u8>,
    depth: u32,
    style: u32,
}

impl Writer {
    /// Appends text.
    fn put(&mut self, text: &str) {
        self.out.extend_from_slice(text.as_bytes());
    }

    /// Appends one character.
    fn put_char(&mut self, character: u8) {
        self.out.push(character);
    }

    /// Appends a line break and indentation as the current style writes them.
    fn newline(&mut self) {
        match self.style {
            0 | 1 => {
                self.put_char(b'\n');
                self.out.resize(self.out.len() + self.depth as usize * 2, b' ');
            }
            2 => {}
            _ => {
                self.put("\r\n");
                self.out.resize(self.out.len() + self.depth as usize, b'\t');
            }
        }
    }

    /// Opens an object or array.
    fn open(&mut self, bracket: u8) {
        self.put_char(bracket);
        self.depth += 1;
    }

    /// Writes the comma between members (not before the first) and a line
    /// break.
    fn sep(&mut self, first: bool) {
        if !first {
            self.put_char(b',');
        }
        self.newline();
    }

    /// Closes an object or array.
    fn close(&mut self, bracket: u8) {
        self.depth -= 1;
        self.newline();
        self.put_char(bracket);
    }

    /// Writes an object member name, and clears `first`.
    fn put_key(&mut self, name: &str, first: &mut bool) {
        self.sep(*first);
        *first = false;
        self.put_char(b'"');
        self.put(name);
        self.put_char(b'"');
        self.put_char(b':');
        if self.style != 2 {
            self.put_char(b' ');
        }
    }

    /// Appends an unsigned decimal.
    fn put_uint(&mut self, mut value: u64) {
        let mut digits = [0u8; 20];
        let mut count = 0;
        loop {
            digits[count] = b'0' + (value % 10) as u8;
            count += 1;
            value /= 10;
            if value == 0 {
                break;
            }
        }
        while count > 0 {
            count -= 1;
            self.put_char(digits[count]);
        }
    }

    /// Appends an unsigned decimal padded with zeros to `width` digits.
    fn put_padded(&mut self, mut value: u64, width: usize) {
        let mut digits = [0u8; 20];
        for i in 0..width {
            digits[width - 1 - i] = b'0' + (value % 10) as u8;
            value /= 10;
        }
        self.out.extend_from_slice(&digits[..width]);
    }

    /// Appends a signed decimal.
    fn put_signed(&mut self, value: i64) {
        if value < 0 {
            self.put_char(b'-');
        }
        self.put_uint(value.unsigned_abs());
    }

    /// Appends the low `digits` hex digits of `value`, lowercase.
    fn put_hex(&mut self, value: u64, digits: u32) {
        for i in 0..digits {
            let digit = ((value >> (4 * (digits - 1 - i))) & 15) as u8;
            self.put_char(if digit < 10 { b'0' + digit } else { b'a' + digit - 10 });
        }
    }

    /// Appends `value / scale` as a decimal with `digits` places.
    fn put_fixed(&mut self, value: i64, scale: u64, digits: usize) {
        if value < 0 {
            self.put_char(b'-');
        }
        let magnitude = value.unsigned_abs();
        self.put_uint(magnitude / scale);
        self.put_char(b'.');
        self.put_padded(magnitude % scale, digits);
    }

    /// Appends a mantissa-and-exponent number such as `1.05e-3` or `-7.50E+2`.
    fn put_exponent(&mut self, m: u64) {
        if (m >> 26) & 1 != 0 {
            self.put_char(b'-');
        }
        self.put_uint(m % 9 + 1);
        self.put_char(b'.');
        self.put_padded((m >> 8) % 100, 2);
        self.put_char(if (m >> 25) & 1 != 0 { b'E' } else { b'e' });
        if (m >> 24) & 1 != 0 {
            self.put_char(b'-');
        } else if (m >> 27) & 1 != 0 {
            self.put_char(b'+');
        }
        self.put_uint((m >> 16) % 6 + 1);
    }

    /// Appends a number in one of five spellings, from integer arithmetic only.
    fn put_number(&mut self, rng: &mut Rng) {
        let x = rng.next();
        let m = x >> 8;
        match x & 7 {
            0..=3 => self.put_fixed((m % 400000) as i64 - 200000, 1000, 3),
            4 => self.put_fixed((m % 2000) as i64 - 1000, 10, 1),
            5 => self.put_signed((m % 2001) as i64 - 1000),
            6 => self.put_exponent(m),
            _ => self.put(SHORT_NUMBERS[(m & 3) as usize]),
        }
    }

    /// Appends `true` or `false`.
    fn put_bool(&mut self, value: bool) {
        self.put(if value { "true" } else { "false" });
    }

    /// Appends a quoted string of one to three words, sometimes with an
    /// escape or a number.
    fn put_text(&mut self, rng: &mut Rng) {
        let x = rng.next();
        let count = 1 + x % 3;
        let separator = if (x >> 4) & 1 != 0 { b'_' } else { b' ' };
        self.put_char(b'"');
        for i in 0..count {
            if i > 0 {
                self.put_char(separator);
            }
            self.put(WORDS[((x >> (8 + 8 * i)) & 15) as usize]);
        }
        let escape = (x >> 32) & 15;
        if escape < 8 {
            self.put(ESCAPES[escape as usize]);
        }
        if (x >> 40) & 1 != 0 {
            self.put_uint((x >> 41) % 1000);
        }
        self.put_char(b'"');
    }

    /// Appends a quoted GUID in the 8-4-4-4-12 layout.
    fn put_guid(&mut self, rng: &mut Rng) {
        let a = rng.next();
        let b = rng.next();
        self.put_char(b'"');
        self.put_hex(a >> 32, 8);
        self.put_char(b'-');
        self.put_hex((a >> 16) & 0xffff, 4);
        self.put_char(b'-');
        self.put_hex(a & 0xffff, 4);
        self.put_char(b'-');
        self.put_hex(b >> 48, 4);
        self.put_char(b'-');
        self.put_hex(b & 0xffff_ffff_ffff, 12);
        self.put_char(b'"');
    }

    /// Appends a quoted asset path.
    fn put_path(&mut self, rng: &mut Rng) {
        let x = rng.next();
        self.put("\"assets/models/");
        self.put(WORDS[(x & 15) as usize]);
        self.put_char(b'_');
        self.put(WORDS[((x >> 8) & 15) as usize]);
        self.put(".vox\"");
    }

    /// Appends a quoted run of 16 to 255 hex digits.
    fn put_blob(&mut self, rng: &mut Rng) {
        let length = 16 + rng.next() % 240;
        self.put_char(b'"');
        for _ in 0..length {
            let digit = rng.next() & 15;
            self.put_hex(digit, 1);
        }
        self.put_char(b'"');
    }

    /// Appends an array of `count` numbers, on one line in styles 1 and 2.
    fn put_numbers(&mut self, rng: &mut Rng, count: u32) {
        let inline_array = self.style == 1 || self.style == 2;
        self.open(b'[');
        for i in 0..count {
            if inline_array {
                if i > 0 {
                    self.put(if self.style == 1 { ", " } else { "," });
                }
            } else {
                self.sep(i == 0);
            }
            self.put_number(rng);
        }
        if inline_array {
            self.depth -= 1;
            self.put_char(b']');
        } else {
            self.close(b']');
        }
    }

    /// Appends a transform object: a nine-number basis and a three-number
    /// origin.
    fn put_transform(&mut self, rng: &mut Rng) {
        let mut first = true;
        self.open(b'{');
        self.put_key("basis", &mut first);
        self.put_numbers(rng, 9);
        self.put_key("origin", &mut first);
        self.put_numbers(rng, 3);
        self.close(b'}');
    }

    /// Appends a voxel body component.
    fn put_mesh(&mut self, rng: &mut Rng) {
        let mut first = true;
        self.open(b'{');
        self.put_key("asset", &mut first);
        let mut inner_first = true;
        self.open(b'{');
        self.put_key("asset", &mut inner_first);
        self.put_guid(rng);
        self.put_key("path", &mut inner_first);
        self.put_path(rng);
        self.close(b'}');
        self.put_key("voxel_size", &mut first);
        self.put_number(rng);
        self.put_key("size", &mut first);
        self.put_numbers(rng, 3);
        self.put_key("palette", &mut first);
        let colours = 2 + rng.next() % 5;
        self.open(b'[');
        for i in 0..colours {
            self.sep(i == 0);
            self.put_numbers(rng, 3);
        }
        self.close(b']');
        self.put_key("cast_shadow", &mut first);
        let shadow = rng.next();
        self.put_bool(shadow % 4 != 0);
        self.put_key("occupancy", &mut first);
        self.put_blob(rng);
        self.close(b'}');
    }

    /// Appends one collision shape.
    fn put_shape(&mut self, rng: &mut Rng) {
        let mut first = true;
        self.open(b'{');
        self.put_key("kind", &mut first);
        self.put_text(rng);
        self.put_key("half_extents", &mut first);
        self.put_numbers(rng, 3);
        self.put_key("radius", &mut first);
        self.put_number(rng);
        self.put_key("offset", &mut first);
        self.put_transform(rng);
        self.close(b'}');
    }

    /// Appends a physics body component with nested shape arrays.
    fn put_physics(&mut self, rng: &mut Rng) {
        let mut first = true;
        self.open(b'{');
        self.put_key("type", &mut first);
        self.put_text(rng);
        self.put_key("mass", &mut first);
        self.put_number(rng);
        self.put_key("friction", &mut first);
        self.put_number(rng);
        self.put_key("restitution", &mut first);
        self.put_number(rng);
        self.put_key("velocity", &mut first);
        self.put_numbers(rng, 3);
        self.put_key("angular_velocity", &mut first);
        self.put_numbers(rng, 3);
        self.put_key("shapes", &mut first);
        let shapes = 1 + rng.next() % 3;
        self.open(b'[');
        for i in 0..shapes {
            self.sep(i == 0);
            self.put_shape(rng);
        }
        self.close(b']');
        self.put_key("sleeping", &mut first);
        let sleeping = rng.next();
        self.put_bool(sleeping % 4 == 0);
        self.close(b'}');
    }

    /// Appends a light component.
    fn put_light(&mut self, rng: &mut Rng) {
        let mut first = true;
        self.open(b'{');
        self.put_key("kind", &mut first);
        self.put_text(rng);
        self.put_key("color", &mut first);
        self.put_numbers(rng, 3);
        self.put_key("intensity", &mut first);
        self.put_number(rng);
        self.put_key("range", &mut first);
        self.put_number(rng);
        self.put_key("spot", &mut first);
        let mut spot_first = true;
        self.open(b'{');
        self.put_key("inner", &mut spot_first);
        self.put_number(rng);
        self.put_key("outer", &mut spot_first);
        self.put_number(rng);
        self.close(b'}');
        self.put_key("shadows", &mut first);
        let shadows = rng.next();
        self.put_bool(shadows % 2 == 0);
        self.close(b'}');
    }

    /// Appends a particle emitter component.
    fn put_emitter(&mut self, rng: &mut Rng) {
        let mut first = true;
        self.open(b'{');
        self.put_key("name", &mut first);
        self.put_text(rng);
        self.put_key("rate", &mut first);
        self.put_number(rng);
        self.put_key("lifetime", &mut first);
        self.put_number(rng);
        self.put_key("gas", &mut first);
        let gas = rng.next();
        self.put_bool(gas % 2 == 0);
        self.put_key("size", &mut first);
        self.put_number(rng);
        self.put_key("velocity", &mut first);
        self.put_numbers(rng, 3);
        self.put_key("colors", &mut first);
        let colours = 2 + rng.next() % 3;
        self.open(b'[');
        for i in 0..colours {
            self.sep(i == 0);
            self.put_numbers(rng, 3);
        }
        self.close(b']');
        self.close(b'}');
    }

    /// Appends an entity reference: a list of instance GUIDs and an entity
    /// GUID.
    fn put_entity_reference(&mut self, rng: &mut Rng) {
        let mut first = true;
        self.open(b'{');
        self.put_key("instances", &mut first);
        let instances = rng.next() % 3;
        self.open(b'[');
        for i in 0..instances {
            self.sep(i == 0);
            self.put_guid(rng);
        }
        self.close(b']');
        self.put_key("entity", &mut first);
        self.put_guid(rng);
        self.close(b'}');
    }

    /// Appends one typed property entry: a key, a type name and a value.
    fn put_property(&mut self, rng: &mut Rng) {
        let mut first = true;
        self.open(b'{');
        self.put_key("key", &mut first);
        self.put_text(rng);
        let kind = rng.next() % 5;
        self.put_key("type", &mut first);
        self.put_char(b'"');
        self.put(PROPERTY_TYPES[kind as usize]);
        self.put_char(b'"');
        self.put_key("value", &mut first);
        match kind {
            0 => self.put_text(rng),
            1 => self.put_number(rng),
            2 => self.put_numbers(rng, 3),
            3 => {
                let flag = rng.next();
                self.put_bool(flag & 1 != 0);
            }
            _ => self.put_entity_reference(rng),
        }
        self.close(b'}');
    }

    /// Appends a behavior component with string and typed properties.
    fn put_behavior(&mut self, rng: &mut Rng) {
        let mut first = true;
        self.open(b'{');
        self.put_key("script", &mut first);
        self.put_path(rng);
        self.put_key("enabled", &mut first);
        let enabled = rng.next();
        self.put_bool(enabled % 8 != 0);
        self.put_key("properties", &mut first);
        let properties = 2 + rng.next() % 4;
        self.open(b'[');
        for i in 0..properties {
            self.sep(i == 0);
            self.put_property(rng);
        }
        self.close(b']');
        self.close(b'}');
    }

    /// Appends the components object; each kind is present with its own odds.
    fn put_components(&mut self, rng: &mut Rng) {
        let flags = rng.next();
        let mut first = true;
        self.open(b'{');
        if flags & 3 != 0 {
            self.put_key("mesh", &mut first);
            self.put_mesh(rng);
        }
        if (flags >> 2) & 3 != 0 {
            self.put_key("physics", &mut first);
            self.put_physics(rng);
        }
        if (flags >> 4) & 3 == 0 {
            self.put_key("light", &mut first);
            self.put_light(rng);
        }
        if (flags >> 6) & 3 == 0 {
            self.put_key("emitter", &mut first);
            self.put_emitter(rng);
        }
        if (flags >> 8) & 3 != 0 {
            self.put_key("behavior", &mut first);
            self.put_behavior(rng);
        }
        self.close(b'}');
    }

    /// Appends an entity in a layout style of its own, with its children.
    fn put_entity(&mut self, rng: &mut Rng, level: u32) {
        let outer_style = self.style;
        self.style = (rng.next() & 3) as u32;
        let mut first = true;
        self.open(b'{');
        self.put_key("id", &mut first);
        self.put_guid(rng);
        self.put_key("name", &mut first);
        self.put_text(rng);
        self.put_key("active", &mut first);
        let active = rng.next();
        self.put_bool(active % 8 != 0);
        self.put_key("order", &mut first);
        let order = rng.next() % 1000;
        self.put_uint(order);
        self.put_key("transform", &mut first);
        self.put_transform(rng);
        self.put_key("tags", &mut first);
        let tags = rng.next() % 4;
        self.open(b'[');
        for i in 0..tags {
            self.sep(i == 0);
            self.put_text(rng);
        }
        self.close(b']');
        self.put_key("components", &mut first);
        self.put_components(rng);
        self.put_key("children", &mut first);
        let children = if level < MAX_ENTITY_LEVEL { rng.next() % 3 } else { 0 };
        self.open(b'[');
        for i in 0..children {
            self.sep(i == 0);
            self.put_entity(rng, level + 1);
        }
        self.close(b']');
        self.close(b'}');
        self.style = outer_style;
    }
}

/// Writes the whole scene document.
fn generate_scene() -> String {
    let mut rng = Rng::new(RNG_SEED);
    let mut w = Writer { out: Vec::with_capacity(6 << 20), depth: 0, style: 0 };
    let mut first = true;
    w.open(b'{');
    w.put_key("format", &mut first);
    w.put("\"voxel.scene\"");
    w.put_key("version", &mut first);
    w.put("3");
    w.put_key("name", &mut first);
    w.put_text(&mut rng);
    w.put_key("settings", &mut first);
    let mut settings_first = true;
    w.open(b'{');
    w.put_key("gravity", &mut settings_first);
    w.put_numbers(&mut rng, 3);
    w.put_key("ambient", &mut settings_first);
    w.put_numbers(&mut rng, 3);
    w.put_key("fog", &mut settings_first);
    let mut fog_first = true;
    w.open(b'{');
    w.put_key("enabled", &mut fog_first);
    w.put_bool(true);
    w.put_key("density", &mut fog_first);
    w.put_number(&mut rng);
    w.close(b'}');
    w.close(b'}');
    w.put_key("entities", &mut first);
    w.open(b'[');
    for i in 0..ROOT_ENTITIES {
        w.sep(i == 0);
        w.put_entity(&mut rng, 0);
    }
    w.close(b']');
    w.close(b'}');
    w.put_char(b'\n');
    String::from_utf8(w.out).expect("the generator writes ASCII")
}

/// Folds one value into the checksum.
fn fold(h: &mut u64, value: u64) {
    *h = hash::add(*h, value);
}

/// Folds a double by its bit pattern.
fn fold_f64(h: &mut u64, value: f64) {
    fold(h, value.to_bits());
}

/// Folds a float by its bit pattern.
fn fold_f32(h: &mut u64, value: f32) {
    fold(h, u64::from(hash::f32_bits(value)));
}

/// Folds a boolean as 0 or 1.
fn fold_bool(h: &mut u64, value: bool) {
    fold(h, u64::from(value));
}

/// Folds a string's length and a running hash of its bytes.
fn fold_text(h: &mut u64, text: &str) {
    let mut running = text.len() as u64;
    for &character in text.as_bytes() {
        running = (running ^ u64::from(character)).wrapping_mul(0x100000001b3);
    }
    fold(h, running);
}

/// Value of one hex digit.
fn nibble(digit: u8) -> Option<u64> {
    match digit {
        b'0'..=b'9' => Some(u64::from(digit - b'0')),
        b'a'..=b'f' => Some(u64::from(digit - b'a') + 10),
        b'A'..=b'F' => Some(u64::from(digit - b'A') + 10),
        _ => None,
    }
}

/// Parses an 8-4-4-4-12 GUID into two words. `None` when malformed.
fn parse_guid(text: &str) -> Option<(u64, u64)> {
    let bytes = text.as_bytes();
    if bytes.len() != 36 {
        return None;
    }
    let mut high = 0u64;
    let mut low = 0u64;
    let mut count = 0;
    for (i, &character) in bytes.iter().enumerate() {
        if i == 8 || i == 13 || i == 18 || i == 23 {
            if character != b'-' {
                return None;
            }
            continue;
        }
        let digit = nibble(character)?;
        if count < 16 {
            high = (high << 4) | digit;
        } else {
            low = (low << 4) | digit;
        }
        count += 1;
    }
    Some((high, low))
}

/// Folds a parsed GUID, or a marker when it is malformed.
fn fold_guid(h: &mut u64, text: &str) {
    match parse_guid(text) {
        Some((high, low)) => {
            fold(h, high);
            fold(h, low);
        }
        None => fold(h, !0u64),
    }
}

/// Reads the GUID member `key` of `parent`.
fn read_guid(h: &mut u64, parent: &Value, key: &str) {
    let text = parent.text_or(key, "");
    if text.is_empty() {
        fold(h, 0);
        return;
    }
    fold_guid(h, text);
}

/// Reads up to `N` numbers of an array as floats, zero for the rest, and
/// folds them.
fn fold_numbers<const N: usize>(h: &mut u64, value: Option<&Value>) {
    let mut result = [0.0f32; N];
    if let Some(array) = value {
        for (slot, item) in result.iter_mut().zip(array.items()) {
            *slot = match item {
                Value::Number(number) => *number as f32,
                _ => 0.0,
            };
        }
    }
    for number in result {
        fold_f32(h, number);
    }
}

/// Reads a transform: a nine-number basis and a three-number origin.
fn read_transform(h: &mut u64, value: Option<&Value>) {
    let Some(transform) = value else {
        fold(h, 0);
        return;
    };
    fold_numbers::<9>(h, transform.find("basis"));
    fold_numbers::<3>(h, transform.find("origin"));
}

/// Reads an asset reference: a GUID and the last known path.
fn read_asset_reference(h: &mut u64, value: Option<&Value>) {
    let Some(reference) = value else {
        fold(h, 0);
        return;
    };
    read_guid(h, reference, "asset");
    fold_text(h, reference.text_or("path", ""));
}

/// Reads an entity reference: instance GUIDs and the entity GUID.
fn read_entity_reference(h: &mut u64, value: Option<&Value>) {
    let Some(reference) = value else {
        fold(h, 0);
        return;
    };
    if let Some(instances @ Value::Array(_)) = reference.find("instances") {
        for item in instances.items() {
            fold_guid(h, item.as_str_or(""));
        }
        fold(h, instances.items().len() as u64);
    }
    read_guid(h, reference, "entity");
}

/// Reads one typed property entry.
fn read_property(h: &mut u64, entry: &Value) {
    fold_text(h, entry.text_or("key", ""));
    let kind = entry.text_or("type", "");
    fold_text(h, kind);
    let value = entry.find("value");
    match (kind, value) {
        ("string", Some(Value::String(text))) => fold_text(h, text),
        ("float", Some(Value::Number(number))) => fold_f64(h, *number),
        ("vec3", _) => fold_numbers::<3>(h, value),
        ("bool", Some(Value::Bool(flag))) => fold_bool(h, *flag),
        ("entity", _) => read_entity_reference(h, value),
        _ => {}
    }
}

/// Reads a behavior component.
fn read_behavior(h: &mut u64, source: &Value) {
    fold_text(h, source.text_or("script", ""));
    fold_bool(h, source.bool_or("enabled", true));
    if let Some(properties @ Value::Array(_)) = source.find("properties") {
        fold(h, properties.items().len() as u64);
        for entry in properties.items() {
            read_property(h, entry);
        }
    }
}

/// Reads every array element as three numbers.
fn read_colours(h: &mut u64, array: Option<&Value>) {
    let Some(colours @ Value::Array(_)) = array else { return };
    for colour in colours.items() {
        fold_numbers::<3>(h, Some(colour));
    }
    fold(h, colours.items().len() as u64);
}

/// Reads a voxel body component.
fn read_mesh(h: &mut u64, source: &Value) {
    read_asset_reference(h, source.find("asset"));
    fold_f64(h, source.number_or("voxel_size", 1.0));
    fold_numbers::<3>(h, source.find("size"));
    read_colours(h, source.find("palette"));
    fold_bool(h, source.bool_or("cast_shadow", true));
    fold_text(h, source.text_or("occupancy", ""));
}

/// Reads a physics body component.
fn read_physics(h: &mut u64, source: &Value) {
    fold_text(h, source.text_or("type", ""));
    fold_f64(h, source.number_or("mass", 1.0));
    fold_f64(h, source.number_or("friction", 0.5));
    fold_f64(h, source.number_or("restitution", 0.0));
    fold_numbers::<3>(h, source.find("velocity"));
    fold_numbers::<3>(h, source.find("angular_velocity"));
    if let Some(shapes @ Value::Array(_)) = source.find("shapes") {
        for shape in shapes.items() {
            fold_text(h, shape.text_or("kind", ""));
            fold_numbers::<3>(h, shape.find("half_extents"));
            fold_f64(h, shape.number_or("radius", 0.5));
            read_transform(h, shape.find("offset"));
        }
        fold(h, shapes.items().len() as u64);
    }
    fold_bool(h, source.bool_or("sleeping", false));
}

/// Reads a light component.
fn read_light(h: &mut u64, source: &Value) {
    fold_text(h, source.text_or("kind", ""));
    fold_numbers::<3>(h, source.find("color"));
    fold_f64(h, source.number_or("intensity", 1.0));
    fold_f64(h, source.number_or("range", 10.0));
    if let Some(spot) = source.find("spot") {
        fold_f64(h, spot.number_or("inner", 0.0));
        fold_f64(h, spot.number_or("outer", 0.0));
    }
    fold_bool(h, source.bool_or("shadows", false));
}

/// Reads a particle emitter component.
fn read_emitter(h: &mut u64, source: &Value) {
    fold_text(h, source.text_or("name", ""));
    fold_f64(h, source.number_or("rate", 0.0));
    fold_f64(h, source.number_or("lifetime", 1.0));
    fold_bool(h, source.bool_or("gas", false));
    fold_f64(h, source.number_or("size", 1.0));
    fold_numbers::<3>(h, source.find("velocity"));
    read_colours(h, source.find("colors"));
}

/// Looks up the component `kind` and reads it, or folds a marker when absent.
fn read_component(h: &mut u64, components: Option<&Value>, kind: &str, reader: fn(&mut u64, &Value)) {
    match components.and_then(|object| object.find(kind)) {
        Some(source) => reader(h, source),
        None => fold(h, 0),
    }
}

/// Reads an entity record and its children.
fn read_entity(h: &mut u64, source: &Value) {
    read_guid(h, source, "id");
    fold_text(h, source.text_or("name", ""));
    fold_bool(h, source.bool_or("active", true));
    fold_f64(h, source.number_or("order", 0.0));
    read_transform(h, source.find("transform"));
    if let Some(tags @ Value::Array(_)) = source.find("tags") {
        for tag in tags.items() {
            fold_text(h, tag.as_str_or(""));
        }
        fold(h, tags.items().len() as u64);
    }
    let components = source.find("components");
    if let Some(object) = components {
        fold(h, object.members().len() as u64);
        for member in object.members() {
            fold_text(h, &member.0);
        }
    }
    read_component(h, components, "mesh", read_mesh);
    read_component(h, components, "physics", read_physics);
    read_component(h, components, "light", read_light);
    read_component(h, components, "emitter", read_emitter);
    read_component(h, components, "behavior", read_behavior);
    read_component(h, components, "audio", read_behavior);
    if let Some(children @ Value::Array(_)) = source.find("children") {
        for child in children.items() {
            read_entity(h, child);
        }
        fold(h, children.items().len() as u64);
    }
}

/// Reads the scene document: header, settings and every entity.
fn read_document(h: &mut u64, root: &Value) {
    fold_text(h, root.text_or("format", ""));
    fold_f64(h, root.number_or("version", 0.0));
    fold_text(h, root.text_or("name", ""));
    if let Some(settings) = root.find("settings") {
        fold_numbers::<3>(h, settings.find("gravity"));
        fold_numbers::<3>(h, settings.find("ambient"));
        if let Some(fog) = settings.find("fog") {
            fold_bool(h, fog.bool_or("enabled", false));
            fold_f64(h, fog.number_or("density", 0.0));
        }
    }
    if let Some(entities @ Value::Array(_)) = root.find("entities") {
        for entity in entities.items() {
            read_entity(h, entity);
        }
        fold(h, entities.items().len() as u64);
    }
}

impl Case for Json {
    const NAME: &'static str = "json";

    /// Writes the scene document.
    fn init() -> Json {
        Json { text: generate_scene() }
    }

    /// Parses the document, reads every entity, and drops the tree.
    fn run(&mut self) -> u64 {
        let Ok(root) = json_value::parse(&self.text) else { return 0 };
        let mut h = 0u64;
        read_document(&mut h, &root);
        h
    }
}
