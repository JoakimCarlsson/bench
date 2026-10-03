//! Scene load: the engine's JSON reader over a generated scene document of a
//! few megabytes. Init writes the text with integer arithmetic only, so it is
//! the same bytes in every language. Each run parses it into a value tree
//! with the engine's strict recursive descent parser, walks the entities the
//! way the entity document code does (components looked up by key, vectors
//! and transforms read from number arrays), folds everything it reads into a
//! checksum, then drops the tree.
const std = @import("std");
const hash = @import("hash.zig");
const json_value = @import("json_value.zig");

const Allocator = std.mem.Allocator;
const Value = json_value.Value;

const Json = @This();

pub const name = "json";

const root_entities: u32 = 520;
const max_entity_level: u32 = 3;
const rng_seed: u64 = 0x15011;

const words = [16][]const u8{
    "stone", "iron",  "lantern", "crate",   "torch", "golem",  "bridge", "tower",
    "grass", "water", "ember",   "crystal", "door",  "statue", "spawn",  "beacon",
};

const escapes = [8][]const u8{ "\\n", "\\\"", "\\\\", "\\u00e9", "\\u4e2d", "\\/", "\\t", "\\u20AC" };

const short_numbers = [4][]const u8{ "-0.5", "0.5", "0.25", "1.0" };

const property_types = [5][]const u8{ "string", "float", "vec3", "bool", "entity" };

gpa: Allocator,
text: []u8,

/// The document text under construction: indentation depth and the layout
/// style of the entity being written (0 pretty, 1 pretty with inline number
/// arrays, 2 compact, 3 tabs and CRLF).
const Writer = struct {
    gpa: Allocator,
    out: std.ArrayList(u8) = .empty,
    depth: u32 = 0,
    style: u32 = 0,

    const Error = Allocator.Error;

    /// Appends text.
    fn put(self: *Writer, text: []const u8) Error!void {
        try self.out.appendSlice(self.gpa, text);
    }

    /// Appends one character.
    fn putChar(self: *Writer, character: u8) Error!void {
        try self.out.append(self.gpa, character);
    }

    /// Appends `count` copies of `character`.
    fn putRepeat(self: *Writer, character: u8, count: usize) Error!void {
        try self.out.appendNTimes(self.gpa, character, count);
    }

    /// Appends a line break and indentation as the current style writes them.
    fn newline(self: *Writer) Error!void {
        switch (self.style) {
            0, 1 => {
                try self.putChar('\n');
                try self.putRepeat(' ', @as(usize, self.depth) * 2);
            },
            2 => {},
            else => {
                try self.put("\r\n");
                try self.putRepeat('\t', self.depth);
            },
        }
    }

    /// Opens an object or array.
    fn open(self: *Writer, bracket: u8) Error!void {
        try self.putChar(bracket);
        self.depth += 1;
    }

    /// Writes the comma between members (not before the first) and a line
    /// break.
    fn sep(self: *Writer, first: bool) Error!void {
        if (!first) try self.putChar(',');
        try self.newline();
    }

    /// Closes an object or array.
    fn close(self: *Writer, bracket: u8) Error!void {
        self.depth -= 1;
        try self.newline();
        try self.putChar(bracket);
    }

    /// Writes an object member name, and clears `first`.
    fn putKey(self: *Writer, key: []const u8, first: *bool) Error!void {
        try self.sep(first.*);
        first.* = false;
        try self.putChar('"');
        try self.put(key);
        try self.putChar('"');
        try self.putChar(':');
        if (self.style != 2) try self.putChar(' ');
    }

    /// Appends an unsigned decimal.
    fn putUint(self: *Writer, value_in: u64) Error!void {
        var value = value_in;
        var digits: [20]u8 = undefined;
        var count: usize = 0;
        while (true) {
            digits[count] = '0' + @as(u8, @intCast(value % 10));
            count += 1;
            value /= 10;
            if (value == 0) break;
        }
        while (count > 0) {
            count -= 1;
            try self.putChar(digits[count]);
        }
    }

    /// Appends an unsigned decimal padded with zeros to `width` digits.
    fn putPadded(self: *Writer, value_in: u64, width: u32) Error!void {
        var value = value_in;
        var digits: [20]u8 = undefined;
        for (0..width) |i| {
            digits[width - 1 - i] = '0' + @as(u8, @intCast(value % 10));
            value /= 10;
        }
        try self.put(digits[0..width]);
    }

    /// Appends a signed decimal.
    fn putSigned(self: *Writer, value: i64) Error!void {
        if (value < 0) try self.putChar('-');
        try self.putUint(@abs(value));
    }

    /// Appends the low `digits` hex digits of `value`, lowercase.
    fn putHex(self: *Writer, value: u64, digits: u32) Error!void {
        for (0..digits) |i| {
            const shift: u6 = @intCast(4 * (digits - 1 - i));
            const digit: u8 = @intCast((value >> shift) & 15);
            try self.putChar(if (digit < 10) '0' + digit else 'a' + digit - 10);
        }
    }

    /// Appends `value / scale` as a decimal with `digits` places.
    fn putFixed(self: *Writer, value: i64, scale: u64, digits: u32) Error!void {
        if (value < 0) try self.putChar('-');
        const magnitude: u64 = @abs(value);
        try self.putUint(magnitude / scale);
        try self.putChar('.');
        try self.putPadded(magnitude % scale, digits);
    }

    /// Appends a mantissa-and-exponent number such as `1.05e-3` or `-7.50E+2`.
    fn putExponent(self: *Writer, m: u64) Error!void {
        if ((m >> 26) & 1 != 0) try self.putChar('-');
        try self.putUint(m % 9 + 1);
        try self.putChar('.');
        try self.putPadded((m >> 8) % 100, 2);
        try self.putChar(if ((m >> 25) & 1 != 0) 'E' else 'e');
        if ((m >> 24) & 1 != 0) {
            try self.putChar('-');
        } else if ((m >> 27) & 1 != 0) {
            try self.putChar('+');
        }
        try self.putUint((m >> 16) % 6 + 1);
    }

    /// Appends a number in one of five spellings, from integer arithmetic only.
    fn putNumber(self: *Writer, rng: *hash.Rng) Error!void {
        const x = rng.next();
        const m = x >> 8;
        switch (x & 7) {
            0...3 => try self.putFixed(@as(i64, @intCast(m % 400000)) - 200000, 1000, 3),
            4 => try self.putFixed(@as(i64, @intCast(m % 2000)) - 1000, 10, 1),
            5 => try self.putSigned(@as(i64, @intCast(m % 2001)) - 1000),
            6 => try self.putExponent(m),
            else => try self.put(short_numbers[m & 3]),
        }
    }

    /// Appends `true` or `false`.
    fn putBool(self: *Writer, value: bool) Error!void {
        try self.put(if (value) "true" else "false");
    }

    /// Appends a quoted string of one to three words, sometimes with an
    /// escape or a number.
    fn putText(self: *Writer, rng: *hash.Rng) Error!void {
        const x = rng.next();
        const count = 1 + x % 3;
        const separator: u8 = if ((x >> 4) & 1 != 0) '_' else ' ';
        try self.putChar('"');
        for (0..count) |i| {
            if (i > 0) try self.putChar(separator);
            const shift: u6 = @intCast(8 + 8 * i);
            try self.put(words[(x >> shift) & 15]);
        }
        const escape = (x >> 32) & 15;
        if (escape < 8) try self.put(escapes[escape]);
        if ((x >> 40) & 1 != 0) try self.putUint((x >> 41) % 1000);
        try self.putChar('"');
    }

    /// Appends a quoted GUID in the 8-4-4-4-12 layout.
    fn putGuid(self: *Writer, rng: *hash.Rng) Error!void {
        const a = rng.next();
        const b = rng.next();
        try self.putChar('"');
        try self.putHex(a >> 32, 8);
        try self.putChar('-');
        try self.putHex((a >> 16) & 0xffff, 4);
        try self.putChar('-');
        try self.putHex(a & 0xffff, 4);
        try self.putChar('-');
        try self.putHex(b >> 48, 4);
        try self.putChar('-');
        try self.putHex(b & 0xffffffffffff, 12);
        try self.putChar('"');
    }

    /// Appends a quoted asset path.
    fn putPath(self: *Writer, rng: *hash.Rng) Error!void {
        const x = rng.next();
        try self.put("\"assets/models/");
        try self.put(words[x & 15]);
        try self.putChar('_');
        try self.put(words[(x >> 8) & 15]);
        try self.put(".vox\"");
    }

    /// Appends a quoted run of 16 to 255 hex digits.
    fn putBlob(self: *Writer, rng: *hash.Rng) Error!void {
        const length = 16 + rng.next() % 240;
        try self.putChar('"');
        for (0..length) |_| {
            const digit = rng.next() & 15;
            try self.putHex(digit, 1);
        }
        try self.putChar('"');
    }

    /// Appends an array of `count` numbers, on one line in styles 1 and 2.
    fn putNumbers(self: *Writer, rng: *hash.Rng, count: u32) Error!void {
        const inline_array = self.style == 1 or self.style == 2;
        try self.open('[');
        for (0..count) |i| {
            if (inline_array) {
                if (i > 0) try self.put(if (self.style == 1) ", " else ",");
            } else {
                try self.sep(i == 0);
            }
            try self.putNumber(rng);
        }
        if (inline_array) {
            self.depth -= 1;
            try self.putChar(']');
        } else {
            try self.close(']');
        }
    }

    /// Appends a transform object: a nine-number basis and a three-number
    /// origin.
    fn putTransform(self: *Writer, rng: *hash.Rng) Error!void {
        var first = true;
        try self.open('{');
        try self.putKey("basis", &first);
        try self.putNumbers(rng, 9);
        try self.putKey("origin", &first);
        try self.putNumbers(rng, 3);
        try self.close('}');
    }

    /// Appends a voxel body component.
    fn putMesh(self: *Writer, rng: *hash.Rng) Error!void {
        var first = true;
        try self.open('{');
        try self.putKey("asset", &first);
        var inner_first = true;
        try self.open('{');
        try self.putKey("asset", &inner_first);
        try self.putGuid(rng);
        try self.putKey("path", &inner_first);
        try self.putPath(rng);
        try self.close('}');
        try self.putKey("voxel_size", &first);
        try self.putNumber(rng);
        try self.putKey("size", &first);
        try self.putNumbers(rng, 3);
        try self.putKey("palette", &first);
        const colours = 2 + rng.next() % 5;
        try self.open('[');
        for (0..colours) |i| {
            try self.sep(i == 0);
            try self.putNumbers(rng, 3);
        }
        try self.close(']');
        try self.putKey("cast_shadow", &first);
        const shadow = rng.next();
        try self.putBool(shadow % 4 != 0);
        try self.putKey("occupancy", &first);
        try self.putBlob(rng);
        try self.close('}');
    }

    /// Appends one collision shape.
    fn putShape(self: *Writer, rng: *hash.Rng) Error!void {
        var first = true;
        try self.open('{');
        try self.putKey("kind", &first);
        try self.putText(rng);
        try self.putKey("half_extents", &first);
        try self.putNumbers(rng, 3);
        try self.putKey("radius", &first);
        try self.putNumber(rng);
        try self.putKey("offset", &first);
        try self.putTransform(rng);
        try self.close('}');
    }

    /// Appends a physics body component with nested shape arrays.
    fn putPhysics(self: *Writer, rng: *hash.Rng) Error!void {
        var first = true;
        try self.open('{');
        try self.putKey("type", &first);
        try self.putText(rng);
        try self.putKey("mass", &first);
        try self.putNumber(rng);
        try self.putKey("friction", &first);
        try self.putNumber(rng);
        try self.putKey("restitution", &first);
        try self.putNumber(rng);
        try self.putKey("velocity", &first);
        try self.putNumbers(rng, 3);
        try self.putKey("angular_velocity", &first);
        try self.putNumbers(rng, 3);
        try self.putKey("shapes", &first);
        const shapes = 1 + rng.next() % 3;
        try self.open('[');
        for (0..shapes) |i| {
            try self.sep(i == 0);
            try self.putShape(rng);
        }
        try self.close(']');
        try self.putKey("sleeping", &first);
        const sleeping = rng.next();
        try self.putBool(sleeping % 4 == 0);
        try self.close('}');
    }

    /// Appends a light component.
    fn putLight(self: *Writer, rng: *hash.Rng) Error!void {
        var first = true;
        try self.open('{');
        try self.putKey("kind", &first);
        try self.putText(rng);
        try self.putKey("color", &first);
        try self.putNumbers(rng, 3);
        try self.putKey("intensity", &first);
        try self.putNumber(rng);
        try self.putKey("range", &first);
        try self.putNumber(rng);
        try self.putKey("spot", &first);
        var spot_first = true;
        try self.open('{');
        try self.putKey("inner", &spot_first);
        try self.putNumber(rng);
        try self.putKey("outer", &spot_first);
        try self.putNumber(rng);
        try self.close('}');
        try self.putKey("shadows", &first);
        const shadows = rng.next();
        try self.putBool(shadows % 2 == 0);
        try self.close('}');
    }

    /// Appends a particle emitter component.
    fn putEmitter(self: *Writer, rng: *hash.Rng) Error!void {
        var first = true;
        try self.open('{');
        try self.putKey("name", &first);
        try self.putText(rng);
        try self.putKey("rate", &first);
        try self.putNumber(rng);
        try self.putKey("lifetime", &first);
        try self.putNumber(rng);
        try self.putKey("gas", &first);
        const gas = rng.next();
        try self.putBool(gas % 2 == 0);
        try self.putKey("size", &first);
        try self.putNumber(rng);
        try self.putKey("velocity", &first);
        try self.putNumbers(rng, 3);
        try self.putKey("colors", &first);
        const colours = 2 + rng.next() % 3;
        try self.open('[');
        for (0..colours) |i| {
            try self.sep(i == 0);
            try self.putNumbers(rng, 3);
        }
        try self.close(']');
        try self.close('}');
    }

    /// Appends an entity reference: a list of instance GUIDs and an entity
    /// GUID.
    fn putEntityReference(self: *Writer, rng: *hash.Rng) Error!void {
        var first = true;
        try self.open('{');
        try self.putKey("instances", &first);
        const instances = rng.next() % 3;
        try self.open('[');
        for (0..instances) |i| {
            try self.sep(i == 0);
            try self.putGuid(rng);
        }
        try self.close(']');
        try self.putKey("entity", &first);
        try self.putGuid(rng);
        try self.close('}');
    }

    /// Appends one typed property entry: a key, a type name and a value.
    fn putProperty(self: *Writer, rng: *hash.Rng) Error!void {
        var first = true;
        try self.open('{');
        try self.putKey("key", &first);
        try self.putText(rng);
        const kind = rng.next() % 5;
        try self.putKey("type", &first);
        try self.putChar('"');
        try self.put(property_types[kind]);
        try self.putChar('"');
        try self.putKey("value", &first);
        switch (kind) {
            0 => try self.putText(rng),
            1 => try self.putNumber(rng),
            2 => try self.putNumbers(rng, 3),
            3 => {
                const flag = rng.next();
                try self.putBool(flag & 1 != 0);
            },
            else => try self.putEntityReference(rng),
        }
        try self.close('}');
    }

    /// Appends a behavior component with string and typed properties.
    fn putBehavior(self: *Writer, rng: *hash.Rng) Error!void {
        var first = true;
        try self.open('{');
        try self.putKey("script", &first);
        try self.putPath(rng);
        try self.putKey("enabled", &first);
        const enabled = rng.next();
        try self.putBool(enabled % 8 != 0);
        try self.putKey("properties", &first);
        const properties = 2 + rng.next() % 4;
        try self.open('[');
        for (0..properties) |i| {
            try self.sep(i == 0);
            try self.putProperty(rng);
        }
        try self.close(']');
        try self.close('}');
    }

    /// Appends the components object; each kind is present with its own odds.
    fn putComponents(self: *Writer, rng: *hash.Rng) Error!void {
        const flags = rng.next();
        var first = true;
        try self.open('{');
        if (flags & 3 != 0) {
            try self.putKey("mesh", &first);
            try self.putMesh(rng);
        }
        if ((flags >> 2) & 3 != 0) {
            try self.putKey("physics", &first);
            try self.putPhysics(rng);
        }
        if ((flags >> 4) & 3 == 0) {
            try self.putKey("light", &first);
            try self.putLight(rng);
        }
        if ((flags >> 6) & 3 == 0) {
            try self.putKey("emitter", &first);
            try self.putEmitter(rng);
        }
        if ((flags >> 8) & 3 != 0) {
            try self.putKey("behavior", &first);
            try self.putBehavior(rng);
        }
        try self.close('}');
    }

    /// Appends an entity in a layout style of its own, with its children.
    fn putEntity(self: *Writer, rng: *hash.Rng, level: u32) Error!void {
        const outer_style = self.style;
        self.style = @intCast(rng.next() & 3);
        var first = true;
        try self.open('{');
        try self.putKey("id", &first);
        try self.putGuid(rng);
        try self.putKey("name", &first);
        try self.putText(rng);
        try self.putKey("active", &first);
        const active = rng.next();
        try self.putBool(active % 8 != 0);
        try self.putKey("order", &first);
        const order = rng.next() % 1000;
        try self.putUint(order);
        try self.putKey("transform", &first);
        try self.putTransform(rng);
        try self.putKey("tags", &first);
        const tags = rng.next() % 4;
        try self.open('[');
        for (0..tags) |i| {
            try self.sep(i == 0);
            try self.putText(rng);
        }
        try self.close(']');
        try self.putKey("components", &first);
        try self.putComponents(rng);
        try self.putKey("children", &first);
        const children: u64 = if (level < max_entity_level) rng.next() % 3 else 0;
        try self.open('[');
        for (0..children) |i| {
            try self.sep(i == 0);
            try self.putEntity(rng, level + 1);
        }
        try self.close(']');
        try self.close('}');
        self.style = outer_style;
    }
};

/// Writes the whole scene document. The caller owns the text.
fn generateScene(gpa: Allocator) Allocator.Error![]u8 {
    var rng: hash.Rng = .{ .s = rng_seed };
    var w: Writer = .{ .gpa = gpa };
    errdefer w.out.deinit(gpa);
    try w.out.ensureTotalCapacity(gpa, 6 << 20);
    var first = true;
    try w.open('{');
    try w.putKey("format", &first);
    try w.put("\"voxel.scene\"");
    try w.putKey("version", &first);
    try w.put("3");
    try w.putKey("name", &first);
    try w.putText(&rng);
    try w.putKey("settings", &first);
    var settings_first = true;
    try w.open('{');
    try w.putKey("gravity", &settings_first);
    try w.putNumbers(&rng, 3);
    try w.putKey("ambient", &settings_first);
    try w.putNumbers(&rng, 3);
    try w.putKey("fog", &settings_first);
    var fog_first = true;
    try w.open('{');
    try w.putKey("enabled", &fog_first);
    try w.putBool(true);
    try w.putKey("density", &fog_first);
    try w.putNumber(&rng);
    try w.close('}');
    try w.close('}');
    try w.putKey("entities", &first);
    try w.open('[');
    for (0..root_entities) |i| {
        try w.sep(i == 0);
        try w.putEntity(&rng, 0);
    }
    try w.close(']');
    try w.close('}');
    try w.putChar('\n');
    return w.out.toOwnedSlice(gpa);
}

/// Folds one value into the checksum.
fn fold(h: *u64, value: u64) void {
    h.* = hash.add(h.*, value);
}

/// Folds a double by its bit pattern.
fn foldF64(h: *u64, value: f64) void {
    fold(h, @as(u64, @bitCast(value)));
}

/// Folds a float by its bit pattern.
fn foldF32(h: *u64, value: f32) void {
    fold(h, hash.f32Bits(value));
}

/// Folds a boolean as 0 or 1.
fn foldBool(h: *u64, value: bool) void {
    fold(h, @intFromBool(value));
}

/// Folds a string's length and a running hash of its bytes.
fn foldText(h: *u64, text: []const u8) void {
    var running: u64 = text.len;
    for (text) |character| {
        running = (running ^ character) *% 0x100000001b3;
    }
    fold(h, running);
}

/// Value of one hex digit, or null.
fn nibble(digit: u8) ?u64 {
    return switch (digit) {
        '0'...'9' => digit - '0',
        'a'...'f' => digit - 'a' + 10,
        'A'...'F' => digit - 'A' + 10,
        else => null,
    };
}

/// The two words of an 8-4-4-4-12 GUID.
const Guid = struct { high: u64, low: u64 };

/// Parses an 8-4-4-4-12 GUID. Null when malformed.
fn parseGuid(text: []const u8) ?Guid {
    if (text.len != 36) return null;
    var result: Guid = .{ .high = 0, .low = 0 };
    var count: u32 = 0;
    for (text, 0..) |character, i| {
        if (i == 8 or i == 13 or i == 18 or i == 23) {
            if (character != '-') return null;
            continue;
        }
        const digit = nibble(character) orelse return null;
        if (count < 16) {
            result.high = (result.high << 4) | digit;
        } else {
            result.low = (result.low << 4) | digit;
        }
        count += 1;
    }
    return result;
}

/// Folds a parsed GUID, or a marker when it is malformed.
fn foldGuid(h: *u64, text: []const u8) void {
    if (parseGuid(text)) |guid| {
        fold(h, guid.high);
        fold(h, guid.low);
    } else {
        fold(h, ~@as(u64, 0));
    }
}

/// Reads the GUID member `key` of `parent`.
fn readGuid(h: *u64, parent: *const Value, key: []const u8) void {
    const text = parent.textOr(key);
    if (text.len == 0) {
        fold(h, 0);
        return;
    }
    foldGuid(h, text);
}

/// Reads up to `n` numbers of an array as floats, zero for the rest, and
/// folds them.
fn foldNumbers(h: *u64, value: ?*const Value, comptime n: usize) void {
    var result = [_]f32{0} ** n;
    if (value) |array| {
        for (array.items(), 0..) |item, i| {
            if (i >= n) break;
            result[i] = switch (item) {
                .number => |number| @floatCast(number),
                else => 0,
            };
        }
    }
    for (result) |number| foldF32(h, number);
}

/// Reads a transform: a nine-number basis and a three-number origin.
fn readTransform(h: *u64, value: ?*const Value) void {
    const transform = value orelse {
        fold(h, 0);
        return;
    };
    foldNumbers(h, transform.find("basis"), 9);
    foldNumbers(h, transform.find("origin"), 3);
}

/// Reads an asset reference: a GUID and the last known path.
fn readAssetReference(h: *u64, value: ?*const Value) void {
    const reference = value orelse {
        fold(h, 0);
        return;
    };
    readGuid(h, reference, "asset");
    foldText(h, reference.textOr("path"));
}

/// Reads an entity reference: instance GUIDs and the entity GUID.
fn readEntityReference(h: *u64, value: ?*const Value) void {
    const reference = value orelse {
        fold(h, 0);
        return;
    };
    if (reference.find("instances")) |instances| {
        if (instances.isArray()) {
            for (instances.items()) |item| foldGuid(h, item.asString());
            fold(h, instances.items().len);
        }
    }
    readGuid(h, reference, "entity");
}

/// Reads one typed property entry.
fn readProperty(h: *u64, entry: *const Value) void {
    foldText(h, entry.textOr("key"));
    const kind = entry.textOr("type");
    foldText(h, kind);
    const value = entry.find("value");
    if (std.mem.eql(u8, kind, "string")) {
        if (value) |text| if (text.* == .string) foldText(h, text.asString());
    } else if (std.mem.eql(u8, kind, "float")) {
        if (value) |number| if (number.* == .number) foldF64(h, number.number);
    } else if (std.mem.eql(u8, kind, "vec3")) {
        foldNumbers(h, value, 3);
    } else if (std.mem.eql(u8, kind, "bool")) {
        if (value) |flag| if (flag.* == .bool) foldBool(h, flag.bool);
    } else if (std.mem.eql(u8, kind, "entity")) {
        readEntityReference(h, value);
    }
}

/// Reads a behavior component.
fn readBehavior(h: *u64, source: *const Value) void {
    foldText(h, source.textOr("script"));
    foldBool(h, source.boolOr("enabled", true));
    if (source.find("properties")) |properties| {
        if (properties.isArray()) {
            fold(h, properties.items().len);
            for (properties.items()) |*entry| readProperty(h, entry);
        }
    }
}

/// Reads every array element as three numbers.
fn readColours(h: *u64, array: ?*const Value) void {
    const colours = array orelse return;
    if (!colours.isArray()) return;
    for (colours.items()) |*colour| foldNumbers(h, colour, 3);
    fold(h, colours.items().len);
}

/// Reads a voxel body component.
fn readMesh(h: *u64, source: *const Value) void {
    readAssetReference(h, source.find("asset"));
    foldF64(h, source.numberOr("voxel_size", 1.0));
    foldNumbers(h, source.find("size"), 3);
    readColours(h, source.find("palette"));
    foldBool(h, source.boolOr("cast_shadow", true));
    foldText(h, source.textOr("occupancy"));
}

/// Reads a physics body component.
fn readPhysics(h: *u64, source: *const Value) void {
    foldText(h, source.textOr("type"));
    foldF64(h, source.numberOr("mass", 1.0));
    foldF64(h, source.numberOr("friction", 0.5));
    foldF64(h, source.numberOr("restitution", 0.0));
    foldNumbers(h, source.find("velocity"), 3);
    foldNumbers(h, source.find("angular_velocity"), 3);
    if (source.find("shapes")) |shapes| {
        if (shapes.isArray()) {
            for (shapes.items()) |*shape| {
                foldText(h, shape.textOr("kind"));
                foldNumbers(h, shape.find("half_extents"), 3);
                foldF64(h, shape.numberOr("radius", 0.5));
                readTransform(h, shape.find("offset"));
            }
            fold(h, shapes.items().len);
        }
    }
    foldBool(h, source.boolOr("sleeping", false));
}

/// Reads a light component.
fn readLight(h: *u64, source: *const Value) void {
    foldText(h, source.textOr("kind"));
    foldNumbers(h, source.find("color"), 3);
    foldF64(h, source.numberOr("intensity", 1.0));
    foldF64(h, source.numberOr("range", 10.0));
    if (source.find("spot")) |spot| {
        foldF64(h, spot.numberOr("inner", 0.0));
        foldF64(h, spot.numberOr("outer", 0.0));
    }
    foldBool(h, source.boolOr("shadows", false));
}

/// Reads a particle emitter component.
fn readEmitter(h: *u64, source: *const Value) void {
    foldText(h, source.textOr("name"));
    foldF64(h, source.numberOr("rate", 0.0));
    foldF64(h, source.numberOr("lifetime", 1.0));
    foldBool(h, source.boolOr("gas", false));
    foldF64(h, source.numberOr("size", 1.0));
    foldNumbers(h, source.find("velocity"), 3);
    readColours(h, source.find("colors"));
}

/// Looks up the component `kind` and reads it, or folds a marker when absent.
fn readComponent(h: *u64, components: ?*const Value, kind: []const u8, comptime reader: anytype) void {
    const source = if (components) |object| object.find(kind) else null;
    if (source) |found| {
        reader(h, found);
    } else {
        fold(h, 0);
    }
}

/// Reads an entity record and its children.
fn readEntity(h: *u64, source: *const Value) void {
    readGuid(h, source, "id");
    foldText(h, source.textOr("name"));
    foldBool(h, source.boolOr("active", true));
    foldF64(h, source.numberOr("order", 0.0));
    readTransform(h, source.find("transform"));
    if (source.find("tags")) |tags| {
        if (tags.isArray()) {
            for (tags.items()) |tag| foldText(h, tag.asString());
            fold(h, tags.items().len);
        }
    }
    const components = source.find("components");
    if (components) |object| {
        fold(h, object.members().len);
        for (object.members()) |member| foldText(h, member.key);
    }
    readComponent(h, components, "mesh", readMesh);
    readComponent(h, components, "physics", readPhysics);
    readComponent(h, components, "light", readLight);
    readComponent(h, components, "emitter", readEmitter);
    readComponent(h, components, "behavior", readBehavior);
    readComponent(h, components, "audio", readBehavior);
    if (source.find("children")) |children| {
        if (children.isArray()) {
            for (children.items()) |*child| readEntity(h, child);
            fold(h, children.items().len);
        }
    }
}

/// Reads the scene document: header, settings and every entity.
fn readDocument(h: *u64, root: *const Value) void {
    foldText(h, root.textOr("format"));
    foldF64(h, root.numberOr("version", 0.0));
    foldText(h, root.textOr("name"));
    if (root.find("settings")) |settings| {
        foldNumbers(h, settings.find("gravity"), 3);
        foldNumbers(h, settings.find("ambient"), 3);
        if (settings.find("fog")) |fog| {
            foldBool(h, fog.boolOr("enabled", false));
            foldF64(h, fog.numberOr("density", 0.0));
        }
    }
    if (root.find("entities")) |entities| {
        if (entities.isArray()) {
            for (entities.items()) |*entity| readEntity(h, entity);
            fold(h, entities.items().len);
        }
    }
}

/// Writes the scene document.
pub fn init(gpa: Allocator) !Json {
    return .{ .gpa = gpa, .text = try generateScene(gpa) };
}

/// Frees the document text.
pub fn deinit(self: *Json, gpa: Allocator) void {
    gpa.free(self.text);
}

/// Parses the document, reads every entity, and frees the tree.
pub fn run(self: *Json) !u64 {
    var root = json_value.parse(self.gpa, self.text) catch return 0;
    defer root.deinit(self.gpa);
    var h: u64 = 0;
    readDocument(&h, &root);
    return h;
}
