#include "json.h"

#include <stdlib.h>
#include <string.h>

#include "hash.h"
#include "json_value.h"

enum { ROOT_ENTITIES = 520, MAX_ENTITY_LEVEL = 3 };

static const uint64_t RNG_SEED = 0x15011;

static const char* const words[16] = {
    "stone", "iron",    "lantern", "crate", "torch",  "golem",  "bridge", "tower",
    "grass", "water",   "ember",   "crystal", "door", "statue", "spawn",  "beacon",
};

static const char* const escapes[8] = {
    "\\n", "\\\"", "\\\\", "\\u00e9", "\\u4e2d", "\\/", "\\t", "\\u20AC",
};

static const char* const short_numbers[4] = {"-0.5", "0.5", "0.25", "1.0"};

static const char* const property_types[5] = {"string", "float", "vec3", "bool", "entity"};

/// The document text under construction: indentation depth and the layout
/// style of the entity being written (0 pretty, 1 pretty with inline number
/// arrays, 2 compact, 3 tabs and CRLF).
typedef struct {
    char* data;
    size_t len;
    size_t cap;
    uint32_t depth;
    uint32_t style;
} Writer;

/// Appends `n` bytes.
static void put_bytes(Writer* w, const char* text, size_t n) {
    if (w->len + n > w->cap) {
        size_t grown = w->cap ? w->cap * 2 : 1u << 20;
        while (grown < w->len + n) grown *= 2;
        w->data = realloc(w->data, grown);
        if (!w->data) exit(1);
        w->cap = grown;
    }
    memcpy(w->data + w->len, text, n);
    w->len += n;
}

/// Appends a NUL-terminated string.
static void put(Writer* w, const char* text) {
    put_bytes(w, text, strlen(text));
}

/// Appends one character.
static void put_char(Writer* w, char c) {
    put_bytes(w, &c, 1);
}

/// Appends `count` copies of `c`.
static void put_repeat(Writer* w, char c, size_t count) {
    for (size_t i = 0; i < count; ++i) put_char(w, c);
}

/// Appends a line break and indentation as the current style writes them.
static void newline(Writer* w) {
    switch (w->style) {
    case 0:
    case 1:
        put_char(w, '\n');
        put_repeat(w, ' ', (size_t)w->depth * 2);
        break;
    case 2:
        break;
    default:
        put(w, "\r\n");
        put_repeat(w, '\t', w->depth);
        break;
    }
}

/// Opens an object or array.
static void open_bracket(Writer* w, char bracket) {
    put_char(w, bracket);
    ++w->depth;
}

/// Writes the comma between members (not before the first) and a line break.
static void sep(Writer* w, bool first) {
    if (!first) put_char(w, ',');
    newline(w);
}

/// Closes an object or array.
static void close_bracket(Writer* w, char bracket) {
    --w->depth;
    newline(w);
    put_char(w, bracket);
}

/// Writes an object member name, and clears `first`.
static void put_key(Writer* w, const char* name, bool* first) {
    sep(w, *first);
    *first = false;
    put_char(w, '"');
    put(w, name);
    put_char(w, '"');
    put_char(w, ':');
    if (w->style != 2) put_char(w, ' ');
}

/// Appends an unsigned decimal.
static void put_uint(Writer* w, uint64_t value) {
    char digits[20];
    size_t count = 0;
    do {
        digits[count++] = (char)('0' + value % 10);
        value /= 10;
    } while (value != 0);
    while (count > 0) put_char(w, digits[--count]);
}

/// Appends an unsigned decimal padded with zeros to `width` digits.
static void put_padded(Writer* w, uint64_t value, uint32_t width) {
    char digits[20];
    for (uint32_t i = 0; i < width; ++i) {
        digits[width - 1 - i] = (char)('0' + value % 10);
        value /= 10;
    }
    for (uint32_t i = 0; i < width; ++i) put_char(w, digits[i]);
}

/// Appends a signed decimal.
static void put_signed(Writer* w, int64_t value) {
    if (value < 0) put_char(w, '-');
    put_uint(w, value < 0 ? (uint64_t)(-value) : (uint64_t)value);
}

/// Appends the low `digits` hex digits of `value`, lowercase.
static void put_hex(Writer* w, uint64_t value, uint32_t digits) {
    for (uint32_t i = 0; i < digits; ++i) {
        uint64_t nibble = (value >> (4 * (digits - 1 - i))) & 15;
        put_char(w, (char)(nibble < 10 ? '0' + nibble : 'a' + nibble - 10));
    }
}

/// Appends `value / scale` as a decimal with `digits` places.
static void put_fixed(Writer* w, int64_t value, uint64_t scale, uint32_t digits) {
    if (value < 0) put_char(w, '-');
    uint64_t magnitude = value < 0 ? (uint64_t)(-value) : (uint64_t)value;
    put_uint(w, magnitude / scale);
    put_char(w, '.');
    put_padded(w, magnitude % scale, digits);
}

/// Appends a mantissa-and-exponent number such as `1.05e-3` or `-7.50E+2`.
static void put_exponent(Writer* w, uint64_t m) {
    if ((m >> 26) & 1) put_char(w, '-');
    put_uint(w, m % 9 + 1);
    put_char(w, '.');
    put_padded(w, (m >> 8) % 100, 2);
    put_char(w, ((m >> 25) & 1) ? 'E' : 'e');
    if ((m >> 24) & 1) {
        put_char(w, '-');
    } else if ((m >> 27) & 1) {
        put_char(w, '+');
    }
    put_uint(w, (m >> 16) % 6 + 1);
}

/// Appends a number in one of five spellings, from integer arithmetic only.
static void put_number(Writer* w, Rng* rng) {
    uint64_t x = rng_next(rng);
    uint64_t m = x >> 8;
    switch (x & 7) {
    case 0:
    case 1:
    case 2:
    case 3:
        put_fixed(w, (int64_t)(m % 400000) - 200000, 1000, 3);
        break;
    case 4:
        put_fixed(w, (int64_t)(m % 2000) - 1000, 10, 1);
        break;
    case 5:
        put_signed(w, (int64_t)(m % 2001) - 1000);
        break;
    case 6:
        put_exponent(w, m);
        break;
    default:
        put(w, short_numbers[m & 3]);
        break;
    }
}

/// Appends `true` or `false`.
static void put_bool(Writer* w, bool value) {
    put(w, value ? "true" : "false");
}

/// Appends a quoted string of one to three words, sometimes with an escape or
/// a number.
static void put_text(Writer* w, Rng* rng) {
    uint64_t x = rng_next(rng);
    uint64_t count = 1 + x % 3;
    char separator = ((x >> 4) & 1) ? '_' : ' ';
    put_char(w, '"');
    for (uint64_t i = 0; i < count; ++i) {
        if (i > 0) put_char(w, separator);
        put(w, words[(x >> (8 + 8 * i)) & 15]);
    }
    uint64_t escape = (x >> 32) & 15;
    if (escape < 8) put(w, escapes[escape]);
    if ((x >> 40) & 1) put_uint(w, (x >> 41) % 1000);
    put_char(w, '"');
}

/// Appends a quoted GUID in the 8-4-4-4-12 layout.
static void put_guid(Writer* w, Rng* rng) {
    uint64_t a = rng_next(rng);
    uint64_t b = rng_next(rng);
    put_char(w, '"');
    put_hex(w, a >> 32, 8);
    put_char(w, '-');
    put_hex(w, (a >> 16) & 0xffff, 4);
    put_char(w, '-');
    put_hex(w, a & 0xffff, 4);
    put_char(w, '-');
    put_hex(w, b >> 48, 4);
    put_char(w, '-');
    put_hex(w, b & 0xffffffffffffull, 12);
    put_char(w, '"');
}

/// Appends a quoted asset path.
static void put_path(Writer* w, Rng* rng) {
    uint64_t x = rng_next(rng);
    put(w, "\"assets/models/");
    put(w, words[x & 15]);
    put_char(w, '_');
    put(w, words[(x >> 8) & 15]);
    put(w, ".vox\"");
}

/// Appends a quoted run of 16 to 255 hex digits.
static void put_blob(Writer* w, Rng* rng) {
    uint64_t length = 16 + rng_next(rng) % 240;
    put_char(w, '"');
    for (uint64_t i = 0; i < length; ++i) {
        uint64_t digit = rng_next(rng) & 15;
        put_hex(w, digit, 1);
    }
    put_char(w, '"');
}

/// Appends an array of `count` numbers, on one line in styles 1 and 2.
static void put_numbers(Writer* w, Rng* rng, uint32_t count) {
    bool inline_array = w->style == 1 || w->style == 2;
    open_bracket(w, '[');
    for (uint32_t i = 0; i < count; ++i) {
        if (inline_array) {
            if (i > 0) put(w, w->style == 1 ? ", " : ",");
        } else {
            sep(w, i == 0);
        }
        put_number(w, rng);
    }
    if (inline_array) {
        --w->depth;
        put_char(w, ']');
    } else {
        close_bracket(w, ']');
    }
}

/// Appends a transform object: a nine-number basis and a three-number origin.
static void put_transform(Writer* w, Rng* rng) {
    bool first = true;
    open_bracket(w, '{');
    put_key(w, "basis", &first);
    put_numbers(w, rng, 9);
    put_key(w, "origin", &first);
    put_numbers(w, rng, 3);
    close_bracket(w, '}');
}

/// Appends a voxel body component.
static void put_mesh(Writer* w, Rng* rng) {
    bool first = true;
    open_bracket(w, '{');
    put_key(w, "asset", &first);
    bool inner_first = true;
    open_bracket(w, '{');
    put_key(w, "asset", &inner_first);
    put_guid(w, rng);
    put_key(w, "path", &inner_first);
    put_path(w, rng);
    close_bracket(w, '}');
    put_key(w, "voxel_size", &first);
    put_number(w, rng);
    put_key(w, "size", &first);
    put_numbers(w, rng, 3);
    put_key(w, "palette", &first);
    uint64_t colours = 2 + rng_next(rng) % 5;
    open_bracket(w, '[');
    for (uint64_t i = 0; i < colours; ++i) {
        sep(w, i == 0);
        put_numbers(w, rng, 3);
    }
    close_bracket(w, ']');
    put_key(w, "cast_shadow", &first);
    uint64_t shadow = rng_next(rng);
    put_bool(w, shadow % 4 != 0);
    put_key(w, "occupancy", &first);
    put_blob(w, rng);
    close_bracket(w, '}');
}

/// Appends one collision shape.
static void put_shape(Writer* w, Rng* rng) {
    bool first = true;
    open_bracket(w, '{');
    put_key(w, "kind", &first);
    put_text(w, rng);
    put_key(w, "half_extents", &first);
    put_numbers(w, rng, 3);
    put_key(w, "radius", &first);
    put_number(w, rng);
    put_key(w, "offset", &first);
    put_transform(w, rng);
    close_bracket(w, '}');
}

/// Appends a physics body component with nested shape arrays.
static void put_physics(Writer* w, Rng* rng) {
    bool first = true;
    open_bracket(w, '{');
    put_key(w, "type", &first);
    put_text(w, rng);
    put_key(w, "mass", &first);
    put_number(w, rng);
    put_key(w, "friction", &first);
    put_number(w, rng);
    put_key(w, "restitution", &first);
    put_number(w, rng);
    put_key(w, "velocity", &first);
    put_numbers(w, rng, 3);
    put_key(w, "angular_velocity", &first);
    put_numbers(w, rng, 3);
    put_key(w, "shapes", &first);
    uint64_t shapes = 1 + rng_next(rng) % 3;
    open_bracket(w, '[');
    for (uint64_t i = 0; i < shapes; ++i) {
        sep(w, i == 0);
        put_shape(w, rng);
    }
    close_bracket(w, ']');
    put_key(w, "sleeping", &first);
    uint64_t sleeping = rng_next(rng);
    put_bool(w, sleeping % 4 == 0);
    close_bracket(w, '}');
}

/// Appends a light component.
static void put_light(Writer* w, Rng* rng) {
    bool first = true;
    open_bracket(w, '{');
    put_key(w, "kind", &first);
    put_text(w, rng);
    put_key(w, "color", &first);
    put_numbers(w, rng, 3);
    put_key(w, "intensity", &first);
    put_number(w, rng);
    put_key(w, "range", &first);
    put_number(w, rng);
    put_key(w, "spot", &first);
    bool spot_first = true;
    open_bracket(w, '{');
    put_key(w, "inner", &spot_first);
    put_number(w, rng);
    put_key(w, "outer", &spot_first);
    put_number(w, rng);
    close_bracket(w, '}');
    put_key(w, "shadows", &first);
    uint64_t shadows = rng_next(rng);
    put_bool(w, shadows % 2 == 0);
    close_bracket(w, '}');
}

/// Appends a particle emitter component.
static void put_emitter(Writer* w, Rng* rng) {
    bool first = true;
    open_bracket(w, '{');
    put_key(w, "name", &first);
    put_text(w, rng);
    put_key(w, "rate", &first);
    put_number(w, rng);
    put_key(w, "lifetime", &first);
    put_number(w, rng);
    put_key(w, "gas", &first);
    uint64_t gas = rng_next(rng);
    put_bool(w, gas % 2 == 0);
    put_key(w, "size", &first);
    put_number(w, rng);
    put_key(w, "velocity", &first);
    put_numbers(w, rng, 3);
    put_key(w, "colors", &first);
    uint64_t colours = 2 + rng_next(rng) % 3;
    open_bracket(w, '[');
    for (uint64_t i = 0; i < colours; ++i) {
        sep(w, i == 0);
        put_numbers(w, rng, 3);
    }
    close_bracket(w, ']');
    close_bracket(w, '}');
}

/// Appends an entity reference: a list of instance GUIDs and an entity GUID.
static void put_entity_reference(Writer* w, Rng* rng) {
    bool first = true;
    open_bracket(w, '{');
    put_key(w, "instances", &first);
    uint64_t instances = rng_next(rng) % 3;
    open_bracket(w, '[');
    for (uint64_t i = 0; i < instances; ++i) {
        sep(w, i == 0);
        put_guid(w, rng);
    }
    close_bracket(w, ']');
    put_key(w, "entity", &first);
    put_guid(w, rng);
    close_bracket(w, '}');
}

/// Appends one typed property entry: a key, a type name and a value.
static void put_property(Writer* w, Rng* rng) {
    bool first = true;
    open_bracket(w, '{');
    put_key(w, "key", &first);
    put_text(w, rng);
    uint64_t kind = rng_next(rng) % 5;
    put_key(w, "type", &first);
    put_char(w, '"');
    put(w, property_types[kind]);
    put_char(w, '"');
    put_key(w, "value", &first);
    switch (kind) {
    case 0:
        put_text(w, rng);
        break;
    case 1:
        put_number(w, rng);
        break;
    case 2:
        put_numbers(w, rng, 3);
        break;
    case 3: {
        uint64_t flag = rng_next(rng);
        put_bool(w, (flag & 1) != 0);
        break;
    }
    default:
        put_entity_reference(w, rng);
        break;
    }
    close_bracket(w, '}');
}

/// Appends a behavior component with string and typed properties.
static void put_behavior(Writer* w, Rng* rng) {
    bool first = true;
    open_bracket(w, '{');
    put_key(w, "script", &first);
    put_path(w, rng);
    put_key(w, "enabled", &first);
    uint64_t enabled = rng_next(rng);
    put_bool(w, enabled % 8 != 0);
    put_key(w, "properties", &first);
    uint64_t properties = 2 + rng_next(rng) % 4;
    open_bracket(w, '[');
    for (uint64_t i = 0; i < properties; ++i) {
        sep(w, i == 0);
        put_property(w, rng);
    }
    close_bracket(w, ']');
    close_bracket(w, '}');
}

/// Appends the components object; each kind is present with its own odds.
static void put_components(Writer* w, Rng* rng) {
    uint64_t flags = rng_next(rng);
    bool first = true;
    open_bracket(w, '{');
    if ((flags & 3) != 0) {
        put_key(w, "mesh", &first);
        put_mesh(w, rng);
    }
    if (((flags >> 2) & 3) != 0) {
        put_key(w, "physics", &first);
        put_physics(w, rng);
    }
    if (((flags >> 4) & 3) == 0) {
        put_key(w, "light", &first);
        put_light(w, rng);
    }
    if (((flags >> 6) & 3) == 0) {
        put_key(w, "emitter", &first);
        put_emitter(w, rng);
    }
    if (((flags >> 8) & 3) != 0) {
        put_key(w, "behavior", &first);
        put_behavior(w, rng);
    }
    close_bracket(w, '}');
}

/// Appends an entity in a layout style of its own, with its children.
static void put_entity(Writer* w, Rng* rng, uint32_t level) {
    uint32_t outer_style = w->style;
    w->style = (uint32_t)(rng_next(rng) & 3);
    bool first = true;
    open_bracket(w, '{');
    put_key(w, "id", &first);
    put_guid(w, rng);
    put_key(w, "name", &first);
    put_text(w, rng);
    put_key(w, "active", &first);
    uint64_t active = rng_next(rng);
    put_bool(w, active % 8 != 0);
    put_key(w, "order", &first);
    uint64_t order = rng_next(rng) % 1000;
    put_uint(w, order);
    put_key(w, "transform", &first);
    put_transform(w, rng);
    put_key(w, "tags", &first);
    uint64_t tags = rng_next(rng) % 4;
    open_bracket(w, '[');
    for (uint64_t i = 0; i < tags; ++i) {
        sep(w, i == 0);
        put_text(w, rng);
    }
    close_bracket(w, ']');
    put_key(w, "components", &first);
    put_components(w, rng);
    put_key(w, "children", &first);
    uint64_t children = level < MAX_ENTITY_LEVEL ? rng_next(rng) % 3 : 0;
    open_bracket(w, '[');
    for (uint64_t i = 0; i < children; ++i) {
        sep(w, i == 0);
        put_entity(w, rng, level + 1);
    }
    close_bracket(w, ']');
    close_bracket(w, '}');
    w->style = outer_style;
}

/// Writes the whole scene document into `w`, NUL-terminated.
static void generate_scene(Writer* w) {
    Rng rng = {RNG_SEED};
    bool first = true;
    open_bracket(w, '{');
    put_key(w, "format", &first);
    put(w, "\"voxel.scene\"");
    put_key(w, "version", &first);
    put(w, "3");
    put_key(w, "name", &first);
    put_text(w, &rng);
    put_key(w, "settings", &first);
    bool settings_first = true;
    open_bracket(w, '{');
    put_key(w, "gravity", &settings_first);
    put_numbers(w, &rng, 3);
    put_key(w, "ambient", &settings_first);
    put_numbers(w, &rng, 3);
    put_key(w, "fog", &settings_first);
    bool fog_first = true;
    open_bracket(w, '{');
    put_key(w, "enabled", &fog_first);
    put_bool(w, true);
    put_key(w, "density", &fog_first);
    put_number(w, &rng);
    close_bracket(w, '}');
    close_bracket(w, '}');
    put_key(w, "entities", &first);
    open_bracket(w, '[');
    for (uint32_t i = 0; i < ROOT_ENTITIES; ++i) {
        sep(w, i == 0);
        put_entity(w, &rng, 0);
    }
    close_bracket(w, ']');
    close_bracket(w, '}');
    put_char(w, '\n');
    put_char(w, '\0');
    --w->len;
}

/// Folds one value into the checksum.
static void fold(uint64_t* h, uint64_t value) {
    *h = hash_add(*h, value);
}

/// Folds a double by its bit pattern.
static void fold_f64(uint64_t* h, double value) {
    uint64_t bits;
    memcpy(&bits, &value, sizeof bits);
    fold(h, bits);
}

/// Folds a float by its bit pattern.
static void fold_f32(uint64_t* h, float value) {
    fold(h, f32_bits(value));
}

/// Folds a string's length and a running hash of its bytes.
static void fold_text(uint64_t* h, JsonText text) {
    uint64_t running = text.len;
    for (size_t i = 0; i < text.len; ++i) {
        running = (running ^ (uint8_t)text.data[i]) * 0x100000001b3ull;
    }
    fold(h, running);
}

/// Value of one hex digit, or -1.
static int nibble(char digit) {
    if (digit >= '0' && digit <= '9') return digit - '0';
    if (digit >= 'a' && digit <= 'f') return digit - 'a' + 10;
    if (digit >= 'A' && digit <= 'F') return digit - 'A' + 10;
    return -1;
}

/// Parses an 8-4-4-4-12 GUID into two words. Returns false when malformed.
static bool parse_guid(JsonText text, uint64_t* high, uint64_t* low) {
    if (text.len != 36) return false;
    *high = 0;
    *low = 0;
    uint32_t count = 0;
    for (size_t i = 0; i < text.len; ++i) {
        if (i == 8 || i == 13 || i == 18 || i == 23) {
            if (text.data[i] != '-') return false;
            continue;
        }
        int digit = nibble(text.data[i]);
        if (digit < 0) return false;
        if (count < 16) {
            *high = (*high << 4) | (uint64_t)digit;
        } else {
            *low = (*low << 4) | (uint64_t)digit;
        }
        ++count;
    }
    return true;
}

/// Folds a parsed GUID, or a marker when it is malformed.
static void fold_guid(uint64_t* h, JsonText text) {
    uint64_t high = 0;
    uint64_t low = 0;
    if (parse_guid(text, &high, &low)) {
        fold(h, high);
        fold(h, low);
    } else {
        fold(h, ~(uint64_t)0);
    }
}

/// Reads the GUID member `key` of `parent`.
static void read_guid(uint64_t* h, const JsonValue* parent, const char* key, size_t key_len) {
    JsonText text = json_text_or(parent, key, key_len);
    if (text.len == 0) {
        fold(h, 0);
        return;
    }
    fold_guid(h, text);
}

#define READ_GUID(h, parent, key) read_guid((h), (parent), (key), sizeof(key) - 1)

/// Reads up to `count` numbers of an array as floats, zero for the rest, and
/// folds them.
static void fold_numbers(uint64_t* h, const JsonValue* value, size_t count) {
    for (size_t i = 0; i < count; ++i) {
        float number = 0.0f;
        if (value && value->kind == JSON_ARRAY && i < value->as.items.len) {
            const JsonValue* item = &value->as.items.data[i];
            number = (float)(item->kind == JSON_NUMBER ? item->as.number : 0.0);
        }
        fold_f32(h, number);
    }
}

/// Reads a transform: a nine-number basis and a three-number origin.
static void read_transform(uint64_t* h, const JsonValue* value) {
    if (!value) {
        fold(h, 0);
        return;
    }
    fold_numbers(h, JSON_FIND(value, "basis"), 9);
    fold_numbers(h, JSON_FIND(value, "origin"), 3);
}

/// Reads an asset reference: a GUID and the last known path.
static void read_asset_reference(uint64_t* h, const JsonValue* value) {
    if (!value) {
        fold(h, 0);
        return;
    }
    READ_GUID(h, value, "asset");
    fold_text(h, JSON_TEXT_OR(value, "path"));
}

/// Reads an entity reference: instance GUIDs and the entity GUID.
static void read_entity_reference(uint64_t* h, const JsonValue* value) {
    if (!value) {
        fold(h, 0);
        return;
    }
    const JsonValue* instances = JSON_FIND(value, "instances");
    if (instances && instances->kind == JSON_ARRAY) {
        for (size_t i = 0; i < instances->as.items.len; ++i) {
            fold_guid(h, json_as_text(&instances->as.items.data[i]));
        }
        fold(h, instances->as.items.len);
    }
    READ_GUID(h, value, "entity");
}

/// Reads one typed property entry.
static void read_property(uint64_t* h, const JsonValue* entry) {
    fold_text(h, JSON_TEXT_OR(entry, "key"));
    JsonText type = JSON_TEXT_OR(entry, "type");
    fold_text(h, type);
    const JsonValue* value = JSON_FIND(entry, "value");
#define TYPE_IS(name) (type.len == sizeof(name) - 1 && memcmp(type.data, name, sizeof(name) - 1) == 0)
    if (TYPE_IS("string")) {
        if (value && value->kind == JSON_STRING) fold_text(h, json_as_text(value));
    } else if (TYPE_IS("float")) {
        if (value && value->kind == JSON_NUMBER) fold_f64(h, value->as.number);
    } else if (TYPE_IS("vec3")) {
        fold_numbers(h, value, 3);
    } else if (TYPE_IS("bool")) {
        if (value && value->kind == JSON_BOOL) fold(h, value->as.boolean ? 1 : 0);
    } else if (TYPE_IS("entity")) {
        read_entity_reference(h, value);
    }
#undef TYPE_IS
}

/// Reads a behavior component.
static void read_behavior(uint64_t* h, const JsonValue* source) {
    fold_text(h, JSON_TEXT_OR(source, "script"));
    fold(h, JSON_BOOL_OR(source, "enabled", true) ? 1 : 0);
    const JsonValue* properties = JSON_FIND(source, "properties");
    if (properties && properties->kind == JSON_ARRAY) {
        fold(h, properties->as.items.len);
        for (size_t i = 0; i < properties->as.items.len; ++i) read_property(h, &properties->as.items.data[i]);
    }
}

/// Reads every array element as three numbers.
static void read_colours(uint64_t* h, const JsonValue* array) {
    if (!array || array->kind != JSON_ARRAY) return;
    for (size_t i = 0; i < array->as.items.len; ++i) fold_numbers(h, &array->as.items.data[i], 3);
    fold(h, array->as.items.len);
}

/// Reads a voxel body component.
static void read_mesh(uint64_t* h, const JsonValue* source) {
    read_asset_reference(h, JSON_FIND(source, "asset"));
    fold_f64(h, JSON_NUMBER_OR(source, "voxel_size", 1.0));
    fold_numbers(h, JSON_FIND(source, "size"), 3);
    read_colours(h, JSON_FIND(source, "palette"));
    fold(h, JSON_BOOL_OR(source, "cast_shadow", true) ? 1 : 0);
    fold_text(h, JSON_TEXT_OR(source, "occupancy"));
}

/// Reads a physics body component.
static void read_physics(uint64_t* h, const JsonValue* source) {
    fold_text(h, JSON_TEXT_OR(source, "type"));
    fold_f64(h, JSON_NUMBER_OR(source, "mass", 1.0));
    fold_f64(h, JSON_NUMBER_OR(source, "friction", 0.5));
    fold_f64(h, JSON_NUMBER_OR(source, "restitution", 0.0));
    fold_numbers(h, JSON_FIND(source, "velocity"), 3);
    fold_numbers(h, JSON_FIND(source, "angular_velocity"), 3);
    const JsonValue* shapes = JSON_FIND(source, "shapes");
    if (shapes && shapes->kind == JSON_ARRAY) {
        for (size_t i = 0; i < shapes->as.items.len; ++i) {
            const JsonValue* shape = &shapes->as.items.data[i];
            fold_text(h, JSON_TEXT_OR(shape, "kind"));
            fold_numbers(h, JSON_FIND(shape, "half_extents"), 3);
            fold_f64(h, JSON_NUMBER_OR(shape, "radius", 0.5));
            read_transform(h, JSON_FIND(shape, "offset"));
        }
        fold(h, shapes->as.items.len);
    }
    fold(h, JSON_BOOL_OR(source, "sleeping", false) ? 1 : 0);
}

/// Reads a light component.
static void read_light(uint64_t* h, const JsonValue* source) {
    fold_text(h, JSON_TEXT_OR(source, "kind"));
    fold_numbers(h, JSON_FIND(source, "color"), 3);
    fold_f64(h, JSON_NUMBER_OR(source, "intensity", 1.0));
    fold_f64(h, JSON_NUMBER_OR(source, "range", 10.0));
    const JsonValue* spot = JSON_FIND(source, "spot");
    if (spot) {
        fold_f64(h, JSON_NUMBER_OR(spot, "inner", 0.0));
        fold_f64(h, JSON_NUMBER_OR(spot, "outer", 0.0));
    }
    fold(h, JSON_BOOL_OR(source, "shadows", false) ? 1 : 0);
}

/// Reads a particle emitter component.
static void read_emitter(uint64_t* h, const JsonValue* source) {
    fold_text(h, JSON_TEXT_OR(source, "name"));
    fold_f64(h, JSON_NUMBER_OR(source, "rate", 0.0));
    fold_f64(h, JSON_NUMBER_OR(source, "lifetime", 1.0));
    fold(h, JSON_BOOL_OR(source, "gas", false) ? 1 : 0);
    fold_f64(h, JSON_NUMBER_OR(source, "size", 1.0));
    fold_numbers(h, JSON_FIND(source, "velocity"), 3);
    read_colours(h, JSON_FIND(source, "colors"));
}

/// A reader of one component kind.
typedef void (*ComponentReader)(uint64_t*, const JsonValue*);

/// Looks up the component `type` and reads it, or folds a marker when absent.
static void read_component(uint64_t* h, const JsonValue* components, const char* type, size_t type_len,
                           ComponentReader reader) {
    const JsonValue* source = json_find(components, type, type_len);
    if (!source) {
        fold(h, 0);
        return;
    }
    reader(h, source);
}

#define READ_COMPONENT(h, components, type, reader) read_component((h), (components), (type), sizeof(type) - 1, (reader))

/// Reads an entity record and its children.
static void read_entity(uint64_t* h, const JsonValue* source) {
    READ_GUID(h, source, "id");
    fold_text(h, JSON_TEXT_OR(source, "name"));
    fold(h, JSON_BOOL_OR(source, "active", true) ? 1 : 0);
    fold_f64(h, JSON_NUMBER_OR(source, "order", 0.0));
    read_transform(h, JSON_FIND(source, "transform"));
    const JsonValue* tags = JSON_FIND(source, "tags");
    if (tags && tags->kind == JSON_ARRAY) {
        for (size_t i = 0; i < tags->as.items.len; ++i) fold_text(h, json_as_text(&tags->as.items.data[i]));
        fold(h, tags->as.items.len);
    }
    const JsonValue* components = JSON_FIND(source, "components");
    if (components) {
        fold(h, components->as.members.len);
        for (size_t i = 0; i < components->as.members.len; ++i) {
            const JsonString* key = &components->as.members.data[i].key;
            fold_text(h, (JsonText){key->data, key->len});
        }
    }
    READ_COMPONENT(h, components, "mesh", read_mesh);
    READ_COMPONENT(h, components, "physics", read_physics);
    READ_COMPONENT(h, components, "light", read_light);
    READ_COMPONENT(h, components, "emitter", read_emitter);
    READ_COMPONENT(h, components, "behavior", read_behavior);
    READ_COMPONENT(h, components, "audio", read_behavior);
    const JsonValue* children = JSON_FIND(source, "children");
    if (children && children->kind == JSON_ARRAY) {
        for (size_t i = 0; i < children->as.items.len; ++i) read_entity(h, &children->as.items.data[i]);
        fold(h, children->as.items.len);
    }
}

/// Reads the scene document: header, settings and every entity.
static void read_document(uint64_t* h, const JsonValue* root) {
    fold_text(h, JSON_TEXT_OR(root, "format"));
    fold_f64(h, JSON_NUMBER_OR(root, "version", 0.0));
    fold_text(h, JSON_TEXT_OR(root, "name"));
    const JsonValue* settings = JSON_FIND(root, "settings");
    if (settings) {
        fold_numbers(h, JSON_FIND(settings, "gravity"), 3);
        fold_numbers(h, JSON_FIND(settings, "ambient"), 3);
        const JsonValue* fog = JSON_FIND(settings, "fog");
        if (fog) {
            fold(h, JSON_BOOL_OR(fog, "enabled", false) ? 1 : 0);
            fold_f64(h, JSON_NUMBER_OR(fog, "density", 0.0));
        }
    }
    const JsonValue* entities = JSON_FIND(root, "entities");
    if (entities && entities->kind == JSON_ARRAY) {
        for (size_t i = 0; i < entities->as.items.len; ++i) read_entity(h, &entities->as.items.data[i]);
        fold(h, entities->as.items.len);
    }
}

typedef struct {
    char* text;
    size_t len;
} Json;

/// Writes the scene document.
static void setup(void* state) {
    Json* s = state;
    Writer w = {0};
    generate_scene(&w);
    s->text = w.data;
    s->len = w.len;
}

/// Parses the document, reads every entity, and frees the tree.
static uint64_t run(void* state) {
    Json* s = state;
    JsonValue root;
    if (!json_parse(s->text, s->len, &root)) return 0;
    uint64_t h = 0;
    read_document(&h, &root);
    json_free(&root);
    return h;
}

/// Frees the document text.
static void teardown(void* state) {
    Json* s = state;
    free(s->text);
}

const Case json_case = { "json", sizeof(Json), setup, run, teardown };
