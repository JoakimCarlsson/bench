#include "json.hpp"

#include <array>
#include <bit>
#include <string_view>

#include "hash.hpp"
#include "json_value.hpp"

namespace bench {

namespace {

using json::Kind;
using json::Value;

constexpr uint32_t root_entities = 520;
constexpr uint32_t max_entity_level = 3;
constexpr uint64_t rng_seed = 0x15011;

constexpr std::array<std::string_view, 16> words = {
    "stone", "iron",   "lantern", "crate",  "torch",   "golem",  "bridge", "tower",
    "grass", "water",  "ember",   "crystal", "door",   "statue", "spawn",  "beacon",
};

constexpr std::array<std::string_view, 8> escapes = {
    "\\n", "\\\"", "\\\\", "\\u00e9", "\\u4e2d", "\\/", "\\t", "\\u20AC",
};

constexpr std::array<std::string_view, 4> short_numbers = {"-0.5", "0.5", "0.25", "1.0"};

constexpr std::array<std::string_view, 5> property_types = {"string", "float", "vec3", "bool", "entity"};

/// The document text under construction: indentation depth and the layout
/// style of the entity being written (0 pretty, 1 pretty with inline number
/// arrays, 2 compact, 3 tabs and CRLF).
struct Writer {
    std::string out;
    uint32_t depth = 0;
    uint32_t style = 0;
};

/// Appends text.
void put(Writer& w, std::string_view text) {
    w.out.append(text);
}

/// Appends one character.
void put_char(Writer& w, char character) {
    w.out.push_back(character);
}

/// Appends a line break and indentation as the current style writes them.
void newline(Writer& w) {
    switch (w.style) {
    case 0:
    case 1:
        w.out.push_back('\n');
        w.out.append(w.depth * 2, ' ');
        break;
    case 2:
        break;
    default:
        w.out.append("\r\n");
        w.out.append(w.depth, '\t');
        break;
    }
}

/// Opens an object or array.
void open(Writer& w, char bracket) {
    put_char(w, bracket);
    ++w.depth;
}

/// Writes the comma between members (not before the first) and a line break.
void sep(Writer& w, bool first) {
    if (!first) put_char(w, ',');
    newline(w);
}

/// Closes an object or array.
void close(Writer& w, char bracket) {
    --w.depth;
    newline(w);
    put_char(w, bracket);
}

/// Writes an object member name, and clears `first`.
void put_key(Writer& w, std::string_view name, bool& first) {
    sep(w, first);
    first = false;
    put_char(w, '"');
    put(w, name);
    put_char(w, '"');
    put_char(w, ':');
    if (w.style != 2) put_char(w, ' ');
}

/// Appends an unsigned decimal.
void put_uint(Writer& w, uint64_t value) {
    std::array<char, 20> digits{};
    size_t count = 0;
    do {
        digits[count++] = static_cast<char>('0' + value % 10);
        value /= 10;
    } while (value != 0);
    while (count > 0) put_char(w, digits[--count]);
}

/// Appends an unsigned decimal padded with zeros to `width` digits.
void put_padded(Writer& w, uint64_t value, uint32_t width) {
    std::array<char, 20> digits{};
    for (uint32_t i = 0; i < width; ++i) {
        digits[width - 1 - i] = static_cast<char>('0' + value % 10);
        value /= 10;
    }
    for (uint32_t i = 0; i < width; ++i) put_char(w, digits[i]);
}

/// Appends a signed decimal.
void put_signed(Writer& w, int64_t value) {
    if (value < 0) put_char(w, '-');
    put_uint(w, value < 0 ? static_cast<uint64_t>(-value) : static_cast<uint64_t>(value));
}

/// Appends the low `digits` hex digits of `value`, lowercase.
void put_hex(Writer& w, uint64_t value, uint32_t digits) {
    for (uint32_t i = 0; i < digits; ++i) {
        const uint64_t nibble = (value >> (4 * (digits - 1 - i))) & 15;
        put_char(w, static_cast<char>(nibble < 10 ? '0' + nibble : 'a' + nibble - 10));
    }
}

/// Appends `value / scale` as a decimal with `digits` places.
void put_fixed(Writer& w, int64_t value, uint64_t scale, uint32_t digits) {
    if (value < 0) put_char(w, '-');
    const uint64_t magnitude = value < 0 ? static_cast<uint64_t>(-value) : static_cast<uint64_t>(value);
    put_uint(w, magnitude / scale);
    put_char(w, '.');
    put_padded(w, magnitude % scale, digits);
}

/// Appends a mantissa-and-exponent number such as `1.05e-3` or `-7.50E+2`.
void put_exponent(Writer& w, uint64_t m) {
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
void put_number(Writer& w, Rng& rng) {
    const uint64_t x = rng.next();
    const uint64_t m = x >> 8;
    switch (x & 7) {
    case 0:
    case 1:
    case 2:
    case 3:
        put_fixed(w, static_cast<int64_t>(m % 400000) - 200000, 1000, 3);
        break;
    case 4:
        put_fixed(w, static_cast<int64_t>(m % 2000) - 1000, 10, 1);
        break;
    case 5:
        put_signed(w, static_cast<int64_t>(m % 2001) - 1000);
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
void put_bool(Writer& w, bool value) {
    put(w, value ? "true" : "false");
}

/// Appends a quoted string of one to three words, sometimes with an escape or
/// a number.
void put_text(Writer& w, Rng& rng) {
    const uint64_t x = rng.next();
    const uint64_t count = 1 + x % 3;
    const char separator = ((x >> 4) & 1) ? '_' : ' ';
    put_char(w, '"');
    for (uint64_t i = 0; i < count; ++i) {
        if (i > 0) put_char(w, separator);
        put(w, words[(x >> (8 + 8 * i)) & 15]);
    }
    const uint64_t escape = (x >> 32) & 15;
    if (escape < 8) put(w, escapes[escape]);
    if ((x >> 40) & 1) put_uint(w, (x >> 41) % 1000);
    put_char(w, '"');
}

/// Appends a quoted GUID in the 8-4-4-4-12 layout.
void put_guid(Writer& w, Rng& rng) {
    const uint64_t a = rng.next();
    const uint64_t b = rng.next();
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
void put_path(Writer& w, Rng& rng) {
    const uint64_t x = rng.next();
    put(w, "\"assets/models/");
    put(w, words[x & 15]);
    put_char(w, '_');
    put(w, words[(x >> 8) & 15]);
    put(w, ".vox\"");
}

/// Appends a quoted run of 16 to 255 hex digits.
void put_blob(Writer& w, Rng& rng) {
    const uint64_t length = 16 + rng.next() % 240;
    put_char(w, '"');
    for (uint64_t i = 0; i < length; ++i) {
        const uint64_t digit = rng.next() & 15;
        put_hex(w, digit, 1);
    }
    put_char(w, '"');
}

/// Appends an array of `count` numbers, on one line in styles 1 and 2.
void put_numbers(Writer& w, Rng& rng, uint32_t count) {
    const bool inline_array = w.style == 1 || w.style == 2;
    open(w, '[');
    for (uint32_t i = 0; i < count; ++i) {
        if (inline_array) {
            if (i > 0) put(w, w.style == 1 ? ", " : ",");
        } else {
            sep(w, i == 0);
        }
        put_number(w, rng);
    }
    if (inline_array) {
        --w.depth;
        put_char(w, ']');
    } else {
        close(w, ']');
    }
}

/// Appends a transform object: a nine-number basis and a three-number origin.
void put_transform(Writer& w, Rng& rng) {
    bool first = true;
    open(w, '{');
    put_key(w, "basis", first);
    put_numbers(w, rng, 9);
    put_key(w, "origin", first);
    put_numbers(w, rng, 3);
    close(w, '}');
}

/// Appends a voxel body component.
void put_mesh(Writer& w, Rng& rng) {
    bool first = true;
    open(w, '{');
    put_key(w, "asset", first);
    bool inner_first = true;
    open(w, '{');
    put_key(w, "asset", inner_first);
    put_guid(w, rng);
    put_key(w, "path", inner_first);
    put_path(w, rng);
    close(w, '}');
    put_key(w, "voxel_size", first);
    put_number(w, rng);
    put_key(w, "size", first);
    put_numbers(w, rng, 3);
    put_key(w, "palette", first);
    const uint64_t colours = 2 + rng.next() % 5;
    open(w, '[');
    for (uint64_t i = 0; i < colours; ++i) {
        sep(w, i == 0);
        put_numbers(w, rng, 3);
    }
    close(w, ']');
    put_key(w, "cast_shadow", first);
    const uint64_t shadow = rng.next();
    put_bool(w, shadow % 4 != 0);
    put_key(w, "occupancy", first);
    put_blob(w, rng);
    close(w, '}');
}

/// Appends one collision shape.
void put_shape(Writer& w, Rng& rng) {
    bool first = true;
    open(w, '{');
    put_key(w, "kind", first);
    put_text(w, rng);
    put_key(w, "half_extents", first);
    put_numbers(w, rng, 3);
    put_key(w, "radius", first);
    put_number(w, rng);
    put_key(w, "offset", first);
    put_transform(w, rng);
    close(w, '}');
}

/// Appends a physics body component with nested shape arrays.
void put_physics(Writer& w, Rng& rng) {
    bool first = true;
    open(w, '{');
    put_key(w, "type", first);
    put_text(w, rng);
    put_key(w, "mass", first);
    put_number(w, rng);
    put_key(w, "friction", first);
    put_number(w, rng);
    put_key(w, "restitution", first);
    put_number(w, rng);
    put_key(w, "velocity", first);
    put_numbers(w, rng, 3);
    put_key(w, "angular_velocity", first);
    put_numbers(w, rng, 3);
    put_key(w, "shapes", first);
    const uint64_t shapes = 1 + rng.next() % 3;
    open(w, '[');
    for (uint64_t i = 0; i < shapes; ++i) {
        sep(w, i == 0);
        put_shape(w, rng);
    }
    close(w, ']');
    put_key(w, "sleeping", first);
    const uint64_t sleeping = rng.next();
    put_bool(w, sleeping % 4 == 0);
    close(w, '}');
}

/// Appends a light component.
void put_light(Writer& w, Rng& rng) {
    bool first = true;
    open(w, '{');
    put_key(w, "kind", first);
    put_text(w, rng);
    put_key(w, "color", first);
    put_numbers(w, rng, 3);
    put_key(w, "intensity", first);
    put_number(w, rng);
    put_key(w, "range", first);
    put_number(w, rng);
    put_key(w, "spot", first);
    bool spot_first = true;
    open(w, '{');
    put_key(w, "inner", spot_first);
    put_number(w, rng);
    put_key(w, "outer", spot_first);
    put_number(w, rng);
    close(w, '}');
    put_key(w, "shadows", first);
    const uint64_t shadows = rng.next();
    put_bool(w, shadows % 2 == 0);
    close(w, '}');
}

/// Appends a particle emitter component.
void put_emitter(Writer& w, Rng& rng) {
    bool first = true;
    open(w, '{');
    put_key(w, "name", first);
    put_text(w, rng);
    put_key(w, "rate", first);
    put_number(w, rng);
    put_key(w, "lifetime", first);
    put_number(w, rng);
    put_key(w, "gas", first);
    const uint64_t gas = rng.next();
    put_bool(w, gas % 2 == 0);
    put_key(w, "size", first);
    put_number(w, rng);
    put_key(w, "velocity", first);
    put_numbers(w, rng, 3);
    put_key(w, "colors", first);
    const uint64_t colours = 2 + rng.next() % 3;
    open(w, '[');
    for (uint64_t i = 0; i < colours; ++i) {
        sep(w, i == 0);
        put_numbers(w, rng, 3);
    }
    close(w, ']');
    close(w, '}');
}

/// Appends an entity reference: a list of instance GUIDs and an entity GUID.
void put_entity_reference(Writer& w, Rng& rng) {
    bool first = true;
    open(w, '{');
    put_key(w, "instances", first);
    const uint64_t instances = rng.next() % 3;
    open(w, '[');
    for (uint64_t i = 0; i < instances; ++i) {
        sep(w, i == 0);
        put_guid(w, rng);
    }
    close(w, ']');
    put_key(w, "entity", first);
    put_guid(w, rng);
    close(w, '}');
}

/// Appends one typed property entry: a key, a type name and a value.
void put_property(Writer& w, Rng& rng) {
    bool first = true;
    open(w, '{');
    put_key(w, "key", first);
    put_text(w, rng);
    const uint64_t kind = rng.next() % 5;
    put_key(w, "type", first);
    put_char(w, '"');
    put(w, property_types[kind]);
    put_char(w, '"');
    put_key(w, "value", first);
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
        const uint64_t flag = rng.next();
        put_bool(w, (flag & 1) != 0);
        break;
    }
    default:
        put_entity_reference(w, rng);
        break;
    }
    close(w, '}');
}

/// Appends a behavior component with string and typed properties.
void put_behavior(Writer& w, Rng& rng) {
    bool first = true;
    open(w, '{');
    put_key(w, "script", first);
    put_path(w, rng);
    put_key(w, "enabled", first);
    const uint64_t enabled = rng.next();
    put_bool(w, enabled % 8 != 0);
    put_key(w, "properties", first);
    const uint64_t properties = 2 + rng.next() % 4;
    open(w, '[');
    for (uint64_t i = 0; i < properties; ++i) {
        sep(w, i == 0);
        put_property(w, rng);
    }
    close(w, ']');
    close(w, '}');
}

/// Appends the components object; each kind is present with its own odds.
void put_components(Writer& w, Rng& rng) {
    const uint64_t flags = rng.next();
    bool first = true;
    open(w, '{');
    if ((flags & 3) != 0) {
        put_key(w, "mesh", first);
        put_mesh(w, rng);
    }
    if (((flags >> 2) & 3) != 0) {
        put_key(w, "physics", first);
        put_physics(w, rng);
    }
    if (((flags >> 4) & 3) == 0) {
        put_key(w, "light", first);
        put_light(w, rng);
    }
    if (((flags >> 6) & 3) == 0) {
        put_key(w, "emitter", first);
        put_emitter(w, rng);
    }
    if (((flags >> 8) & 3) != 0) {
        put_key(w, "behavior", first);
        put_behavior(w, rng);
    }
    close(w, '}');
}

/// Appends an entity in a layout style of its own, with its children.
void put_entity(Writer& w, Rng& rng, uint32_t level) {
    const uint32_t outer_style = w.style;
    w.style = static_cast<uint32_t>(rng.next() & 3);
    bool first = true;
    open(w, '{');
    put_key(w, "id", first);
    put_guid(w, rng);
    put_key(w, "name", first);
    put_text(w, rng);
    put_key(w, "active", first);
    const uint64_t active = rng.next();
    put_bool(w, active % 8 != 0);
    put_key(w, "order", first);
    const uint64_t order = rng.next() % 1000;
    put_uint(w, order);
    put_key(w, "transform", first);
    put_transform(w, rng);
    put_key(w, "tags", first);
    const uint64_t tags = rng.next() % 4;
    open(w, '[');
    for (uint64_t i = 0; i < tags; ++i) {
        sep(w, i == 0);
        put_text(w, rng);
    }
    close(w, ']');
    put_key(w, "components", first);
    put_components(w, rng);
    put_key(w, "children", first);
    const uint64_t children = level < max_entity_level ? rng.next() % 3 : 0;
    open(w, '[');
    for (uint64_t i = 0; i < children; ++i) {
        sep(w, i == 0);
        put_entity(w, rng, level + 1);
    }
    close(w, ']');
    close(w, '}');
    w.style = outer_style;
}

/// Writes the whole scene document.
std::string generate_scene() {
    Rng rng{rng_seed};
    Writer w;
    w.out.reserve(6u << 20);
    bool first = true;
    open(w, '{');
    put_key(w, "format", first);
    put(w, "\"voxel.scene\"");
    put_key(w, "version", first);
    put(w, "3");
    put_key(w, "name", first);
    put_text(w, rng);
    put_key(w, "settings", first);
    bool settings_first = true;
    open(w, '{');
    put_key(w, "gravity", settings_first);
    put_numbers(w, rng, 3);
    put_key(w, "ambient", settings_first);
    put_numbers(w, rng, 3);
    put_key(w, "fog", settings_first);
    bool fog_first = true;
    open(w, '{');
    put_key(w, "enabled", fog_first);
    put_bool(w, true);
    put_key(w, "density", fog_first);
    put_number(w, rng);
    close(w, '}');
    close(w, '}');
    put_key(w, "entities", first);
    open(w, '[');
    for (uint32_t i = 0; i < root_entities; ++i) {
        sep(w, i == 0);
        put_entity(w, rng, 0);
    }
    close(w, ']');
    close(w, '}');
    put_char(w, '\n');
    return std::move(w.out);
}

/// Folds one value into the checksum.
void fold(uint64_t& h, uint64_t value) {
    h = hash_add(h, value);
}

/// Folds a double by its bit pattern.
void fold_f64(uint64_t& h, double value) {
    fold(h, std::bit_cast<uint64_t>(value));
}

/// Folds a float by its bit pattern.
void fold_f32(uint64_t& h, float value) {
    fold(h, f32_bits(value));
}

/// Folds a string's length and a running hash of its bytes.
void fold_text(uint64_t& h, std::string_view text) {
    uint64_t running = text.size();
    for (const char character : text) {
        running = (running ^ static_cast<uint8_t>(character)) * 0x100000001b3ull;
    }
    fold(h, running);
}

/// Value of one hex digit, or -1.
int nibble(char digit) {
    if (digit >= '0' && digit <= '9') return digit - '0';
    if (digit >= 'a' && digit <= 'f') return digit - 'a' + 10;
    if (digit >= 'A' && digit <= 'F') return digit - 'A' + 10;
    return -1;
}

/// Parses an 8-4-4-4-12 GUID into two words. Returns false when malformed.
bool parse_guid(std::string_view text, uint64_t& high, uint64_t& low) {
    if (text.size() != 36) return false;
    high = 0;
    low = 0;
    uint32_t count = 0;
    for (size_t i = 0; i < text.size(); ++i) {
        if (i == 8 || i == 13 || i == 18 || i == 23) {
            if (text[i] != '-') return false;
            continue;
        }
        const int digit = nibble(text[i]);
        if (digit < 0) return false;
        if (count < 16) {
            high = (high << 4) | static_cast<uint64_t>(digit);
        } else {
            low = (low << 4) | static_cast<uint64_t>(digit);
        }
        ++count;
    }
    return true;
}

/// Folds a parsed GUID, or a marker when it is malformed.
void fold_guid(uint64_t& h, std::string_view text) {
    uint64_t high = 0;
    uint64_t low = 0;
    if (parse_guid(text, high, low)) {
        fold(h, high);
        fold(h, low);
    } else {
        fold(h, ~uint64_t{0});
    }
}

/// Reads the GUID member `key` of `parent`.
void read_guid(uint64_t& h, const Value& parent, std::string_view key) {
    const std::string_view text = parent.text_or(key, "");
    if (text.empty()) {
        fold(h, 0);
        return;
    }
    fold_guid(h, text);
}

/// Reads up to `N` numbers of an array as floats, zero for the rest.
template <size_t N>
std::array<float, N> read_numbers(const Value* value) {
    std::array<float, N> result{};
    if (value == nullptr || !value->is(Kind::Array)) return result;
    for (size_t i = 0; i < N && i < value->items().size(); ++i) {
        result[i] = static_cast<float>(value->items()[i].as_number());
    }
    return result;
}

/// Reads `N` numbers and folds them.
template <size_t N>
void fold_numbers(uint64_t& h, const Value* value) {
    for (const float number : read_numbers<N>(value)) fold_f32(h, number);
}

/// Reads a transform: a nine-number basis and a three-number origin.
void read_transform(uint64_t& h, const Value* value) {
    if (value == nullptr) {
        fold(h, 0);
        return;
    }
    fold_numbers<9>(h, value->find("basis"));
    fold_numbers<3>(h, value->find("origin"));
}

/// Reads an asset reference: a GUID and the last known path.
void read_asset_reference(uint64_t& h, const Value* value) {
    if (value == nullptr) {
        fold(h, 0);
        return;
    }
    read_guid(h, *value, "asset");
    fold_text(h, value->text_or("path", ""));
}

/// Reads an entity reference: instance GUIDs and the entity GUID.
void read_entity_reference(uint64_t& h, const Value* value) {
    if (value == nullptr) {
        fold(h, 0);
        return;
    }
    const Value* instances = value->find("instances");
    if (instances != nullptr && instances->is(Kind::Array)) {
        for (const Value& item : instances->items()) fold_guid(h, item.as_string());
        fold(h, instances->items().size());
    }
    read_guid(h, *value, "entity");
}

/// Reads one typed property entry.
void read_property(uint64_t& h, const Value& entry) {
    fold_text(h, entry.text_or("key", ""));
    const std::string_view type = entry.text_or("type", "");
    fold_text(h, type);
    const Value* value = entry.find("value");
    if (type == "string") {
        if (value != nullptr && value->is(Kind::String)) fold_text(h, value->as_string());
    } else if (type == "float") {
        if (value != nullptr && value->is(Kind::Number)) fold_f64(h, value->as_number());
    } else if (type == "vec3") {
        fold_numbers<3>(h, value);
    } else if (type == "bool") {
        if (value != nullptr && value->is(Kind::Bool)) fold(h, value->as_bool() ? 1 : 0);
    } else if (type == "entity") {
        read_entity_reference(h, value);
    }
}

/// Reads a behavior component.
void read_behavior(uint64_t& h, const Value& source) {
    fold_text(h, source.text_or("script", ""));
    fold(h, source.bool_or("enabled", true) ? 1 : 0);
    const Value* properties = source.find("properties");
    if (properties != nullptr && properties->is(Kind::Array)) {
        fold(h, properties->items().size());
        for (const Value& entry : properties->items()) read_property(h, entry);
    }
}

/// Reads every array element as three numbers.
void read_colours(uint64_t& h, const Value* array) {
    if (array == nullptr || !array->is(Kind::Array)) return;
    for (const Value& colour : array->items()) fold_numbers<3>(h, &colour);
    fold(h, array->items().size());
}

/// Reads a voxel body component.
void read_mesh(uint64_t& h, const Value& source) {
    read_asset_reference(h, source.find("asset"));
    fold_f64(h, source.number_or("voxel_size", 1.0));
    fold_numbers<3>(h, source.find("size"));
    read_colours(h, source.find("palette"));
    fold(h, source.bool_or("cast_shadow", true) ? 1 : 0);
    fold_text(h, source.text_or("occupancy", ""));
}

/// Reads a physics body component.
void read_physics(uint64_t& h, const Value& source) {
    fold_text(h, source.text_or("type", ""));
    fold_f64(h, source.number_or("mass", 1.0));
    fold_f64(h, source.number_or("friction", 0.5));
    fold_f64(h, source.number_or("restitution", 0.0));
    fold_numbers<3>(h, source.find("velocity"));
    fold_numbers<3>(h, source.find("angular_velocity"));
    const Value* shapes = source.find("shapes");
    if (shapes != nullptr && shapes->is(Kind::Array)) {
        for (const Value& shape : shapes->items()) {
            fold_text(h, shape.text_or("kind", ""));
            fold_numbers<3>(h, shape.find("half_extents"));
            fold_f64(h, shape.number_or("radius", 0.5));
            read_transform(h, shape.find("offset"));
        }
        fold(h, shapes->items().size());
    }
    fold(h, source.bool_or("sleeping", false) ? 1 : 0);
}

/// Reads a light component.
void read_light(uint64_t& h, const Value& source) {
    fold_text(h, source.text_or("kind", ""));
    fold_numbers<3>(h, source.find("color"));
    fold_f64(h, source.number_or("intensity", 1.0));
    fold_f64(h, source.number_or("range", 10.0));
    const Value* spot = source.find("spot");
    if (spot != nullptr) {
        fold_f64(h, spot->number_or("inner", 0.0));
        fold_f64(h, spot->number_or("outer", 0.0));
    }
    fold(h, source.bool_or("shadows", false) ? 1 : 0);
}

/// Reads a particle emitter component.
void read_emitter(uint64_t& h, const Value& source) {
    fold_text(h, source.text_or("name", ""));
    fold_f64(h, source.number_or("rate", 0.0));
    fold_f64(h, source.number_or("lifetime", 1.0));
    fold(h, source.bool_or("gas", false) ? 1 : 0);
    fold_f64(h, source.number_or("size", 1.0));
    fold_numbers<3>(h, source.find("velocity"));
    read_colours(h, source.find("colors"));
}

/// Looks up the component `type` and reads it, or folds a marker when absent.
void read_component(uint64_t& h, const Value* components, std::string_view type,
                    void (*reader)(uint64_t&, const Value&)) {
    const Value* source = components != nullptr ? components->find(type) : nullptr;
    if (source == nullptr) {
        fold(h, 0);
        return;
    }
    reader(h, *source);
}

/// Reads an entity record and its children.
void read_entity(uint64_t& h, const Value& source) {
    read_guid(h, source, "id");
    fold_text(h, source.text_or("name", ""));
    fold(h, source.bool_or("active", true) ? 1 : 0);
    fold_f64(h, source.number_or("order", 0.0));
    read_transform(h, source.find("transform"));
    const Value* tags = source.find("tags");
    if (tags != nullptr && tags->is(Kind::Array)) {
        for (const Value& tag : tags->items()) fold_text(h, tag.as_string());
        fold(h, tags->items().size());
    }
    const Value* components = source.find("components");
    if (components != nullptr) {
        fold(h, components->members().size());
        for (const auto& member : components->members()) fold_text(h, member.first);
    }
    read_component(h, components, "mesh", read_mesh);
    read_component(h, components, "physics", read_physics);
    read_component(h, components, "light", read_light);
    read_component(h, components, "emitter", read_emitter);
    read_component(h, components, "behavior", read_behavior);
    read_component(h, components, "audio", read_behavior);
    const Value* children = source.find("children");
    if (children != nullptr && children->is(Kind::Array)) {
        for (const Value& child : children->items()) read_entity(h, child);
        fold(h, children->items().size());
    }
}

/// Reads the scene document: header, settings and every entity.
void read_document(uint64_t& h, const Value& root) {
    fold_text(h, root.text_or("format", ""));
    fold_f64(h, root.number_or("version", 0.0));
    fold_text(h, root.text_or("name", ""));
    const Value* settings = root.find("settings");
    if (settings != nullptr) {
        fold_numbers<3>(h, settings->find("gravity"));
        fold_numbers<3>(h, settings->find("ambient"));
        const Value* fog = settings->find("fog");
        if (fog != nullptr) {
            fold(h, fog->bool_or("enabled", false) ? 1 : 0);
            fold_f64(h, fog->number_or("density", 0.0));
        }
    }
    const Value* entities = root.find("entities");
    if (entities != nullptr && entities->is(Kind::Array)) {
        for (const Value& entity : entities->items()) read_entity(h, entity);
        fold(h, entities->items().size());
    }
}

} // namespace

Json::Json() : text_{generate_scene()} {}

uint64_t Json::run() {
    Value root;
    if (!json::parse(text_, root)) return 0;
    uint64_t h = 0;
    read_document(h, root);
    return h;
}

} // namespace bench
