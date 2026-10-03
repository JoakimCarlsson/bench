#include "json_value.h"

#include <stdlib.h>
#include <string.h>

enum { MAX_DEPTH = 128, STRING_MIN_CAP = 16, LIST_MIN_CAP = 4 };

/// A recursive descent parser over a NUL-terminated buffer.
typedef struct {
    const char* source;
    size_t size;
    size_t offset;
} Parser;

/// Exits on a failed allocation.
static void* checked(void* p) {
    if (!p) exit(1);
    return p;
}

/// Appends one byte to an owned string.
static void string_push(JsonString* s, char c) {
    if (s->len == s->cap) {
        s->cap = s->cap ? s->cap * 2 : STRING_MIN_CAP;
        s->data = checked(realloc(s->data, s->cap));
    }
    s->data[s->len++] = c;
}

/// Appends a value to an array, growing it by doubling.
static void items_push(JsonValue* array, JsonValue item) {
    if (array->as.items.len == array->as.items.cap) {
        array->as.items.cap = array->as.items.cap ? array->as.items.cap * 2 : LIST_MIN_CAP;
        array->as.items.data = checked(realloc(array->as.items.data, array->as.items.cap * sizeof(JsonValue)));
    }
    array->as.items.data[array->as.items.len++] = item;
}

/// Sets an object member, replacing an existing one and freeing what it held;
/// the object takes ownership of `key` and `value`.
static void members_set(JsonValue* object, JsonString key, JsonValue value) {
    for (size_t i = 0; i < object->as.members.len; ++i) {
        JsonMember* member = &object->as.members.data[i];
        if (member->key.len == key.len && memcmp(member->key.data, key.data, key.len) == 0) {
            json_free(&member->value);
            member->value = value;
            free(key.data);
            return;
        }
    }
    if (object->as.members.len == object->as.members.cap) {
        object->as.members.cap = object->as.members.cap ? object->as.members.cap * 2 : LIST_MIN_CAP;
        object->as.members.data =
            checked(realloc(object->as.members.data, object->as.members.cap * sizeof(JsonMember)));
    }
    JsonMember* slot = &object->as.members.data[object->as.members.len++];
    slot->key = key;
    slot->value = value;
}

/// Advances past JSON whitespace.
static void skip_space(Parser* p) {
    while (p->offset < p->size) {
        char c = p->source[p->offset];
        if (c == ' ' || c == '\t' || c == '\n' || c == '\r') {
            ++p->offset;
            continue;
        }
        break;
    }
}

/// Consumes `c` if it is next.
static bool expect(Parser* p, char c) {
    if (p->offset >= p->size || p->source[p->offset] != c) return false;
    ++p->offset;
    return true;
}

/// Tests the next character without consuming it.
static bool next_is(const Parser* p, char c) {
    return p->offset < p->size && p->source[p->offset] == c;
}

/// Consumes a keyword when the input continues with it.
static bool literal(Parser* p, const char* word, size_t n) {
    if (p->size - p->offset < n || memcmp(p->source + p->offset, word, n) != 0) return false;
    p->offset += n;
    return true;
}

/// Appends the UTF-8 encoding of a code unit.
static void push_utf8(JsonString* out, uint32_t code) {
    if (code < 0x80u) {
        string_push(out, (char)code);
    } else if (code < 0x800u) {
        string_push(out, (char)(0xC0u | (code >> 6u)));
        string_push(out, (char)(0x80u | (code & 0x3Fu)));
    } else {
        string_push(out, (char)(0xE0u | (code >> 12u)));
        string_push(out, (char)(0x80u | ((code >> 6u) & 0x3Fu)));
        string_push(out, (char)(0x80u | (code & 0x3Fu)));
    }
}

/// Decodes the four hex digits of a unicode escape and appends them as UTF-8.
/// Surrogate pairs are not combined.
static bool parse_escape_unit(Parser* p, JsonString* out) {
    if (p->offset + 4 > p->size) return false;
    uint32_t code = 0;
    for (size_t i = 0; i < 4; ++i) {
        char digit = p->source[p->offset++];
        code <<= 4u;
        if (digit >= '0' && digit <= '9') {
            code |= (uint32_t)(digit - '0');
        } else if (digit >= 'a' && digit <= 'f') {
            code |= (uint32_t)(digit - 'a') + 10u;
        } else if (digit >= 'A' && digit <= 'F') {
            code |= (uint32_t)(digit - 'A') + 10u;
        } else {
            return false;
        }
    }
    push_utf8(out, code);
    return true;
}

/// Parses a quoted string into `result`, decoding its escapes. On failure
/// `result` still owns what it collected.
static bool parse_string(Parser* p, JsonString* result) {
    if (!expect(p, '"')) return false;
    while (true) {
        if (p->offset >= p->size) return false;
        char c = p->source[p->offset++];
        if (c == '"') return true;
        if (c != '\\') {
            string_push(result, c);
            continue;
        }
        if (p->offset >= p->size) return false;
        char escape = p->source[p->offset++];
        switch (escape) {
        case '"':
        case '\\':
        case '/':
            string_push(result, escape);
            break;
        case 'b':
            string_push(result, '\b');
            break;
        case 'f':
            string_push(result, '\f');
            break;
        case 'n':
            string_push(result, '\n');
            break;
        case 'r':
            string_push(result, '\r');
            break;
        case 't':
            string_push(result, '\t');
            break;
        case 'u':
            if (!parse_escape_unit(p, result)) return false;
            break;
        default:
            return false;
        }
    }
}

/// Parses a number token with `strtod`, which rounds correctly, and requires
/// it to consume the whole token.
static bool parse_number(Parser* p, double* out) {
    size_t start = p->offset;
    if (p->offset < p->size && p->source[p->offset] == '-') ++p->offset;
    while (p->offset < p->size) {
        char c = p->source[p->offset];
        bool numeric = (c >= '0' && c <= '9') || c == '.' || c == 'e' || c == 'E' || c == '+' || c == '-';
        if (!numeric) break;
        ++p->offset;
    }
    if (p->offset == start) return false;
    char* end = NULL;
    *out = strtod(p->source + start, &end);
    return end == p->source + p->offset;
}

static bool parse_value(Parser* p, size_t depth, JsonValue* out);

/// Parses an object.
static bool parse_object(Parser* p, size_t depth, JsonValue* out) {
    if (!expect(p, '{')) return false;
    JsonValue result = {.kind = JSON_OBJECT};
    skip_space(p);
    if (next_is(p, '}')) {
        ++p->offset;
        *out = result;
        return true;
    }
    while (true) {
        skip_space(p);
        JsonString key = {0};
        if (!parse_string(p, &key)) {
            free(key.data);
            break;
        }
        skip_space(p);
        if (!expect(p, ':')) {
            free(key.data);
            break;
        }
        skip_space(p);
        JsonValue member = {0};
        if (!parse_value(p, depth + 1, &member)) {
            free(key.data);
            break;
        }
        members_set(&result, key, member);
        skip_space(p);
        if (next_is(p, ',')) {
            ++p->offset;
            continue;
        }
        if (!expect(p, '}')) break;
        *out = result;
        return true;
    }
    json_free(&result);
    return false;
}

/// Parses an array.
static bool parse_array(Parser* p, size_t depth, JsonValue* out) {
    if (!expect(p, '[')) return false;
    JsonValue result = {.kind = JSON_ARRAY};
    skip_space(p);
    if (next_is(p, ']')) {
        ++p->offset;
        *out = result;
        return true;
    }
    while (true) {
        skip_space(p);
        JsonValue item = {0};
        if (!parse_value(p, depth + 1, &item)) break;
        items_push(&result, item);
        skip_space(p);
        if (next_is(p, ',')) {
            ++p->offset;
            continue;
        }
        if (!expect(p, ']')) break;
        *out = result;
        return true;
    }
    json_free(&result);
    return false;
}

/// Parses any value at the current position.
static bool parse_value(Parser* p, size_t depth, JsonValue* out) {
    if (depth > MAX_DEPTH || p->offset >= p->size) return false;
    switch (p->source[p->offset]) {
    case '{':
        return parse_object(p, depth, out);
    case '[':
        return parse_array(p, depth, out);
    case '"': {
        JsonValue text = {.kind = JSON_STRING};
        if (!parse_string(p, &text.as.text)) {
            free(text.as.text.data);
            return false;
        }
        *out = text;
        return true;
    }
    case 't':
        if (!literal(p, "true", 4)) return false;
        *out = (JsonValue){.kind = JSON_BOOL, .as.boolean = true};
        return true;
    case 'f':
        if (!literal(p, "false", 5)) return false;
        *out = (JsonValue){.kind = JSON_BOOL, .as.boolean = false};
        return true;
    case 'n':
        if (!literal(p, "null", 4)) return false;
        *out = (JsonValue){.kind = JSON_NULL};
        return true;
    default:
        break;
    }
    double number = 0.0;
    if (!parse_number(p, &number)) return false;
    *out = (JsonValue){.kind = JSON_NUMBER, .as.number = number};
    return true;
}

bool json_parse(const char* source, size_t len, JsonValue* out) {
    Parser p = {source, len, 0};
    *out = (JsonValue){.kind = JSON_NULL};
    skip_space(&p);
    JsonValue root = {0};
    if (!parse_value(&p, 0, &root)) return false;
    skip_space(&p);
    if (p.offset != p.size) {
        json_free(&root);
        return false;
    }
    *out = root;
    return true;
}

void json_free(JsonValue* value) {
    switch (value->kind) {
    case JSON_STRING:
        free(value->as.text.data);
        break;
    case JSON_ARRAY:
        for (size_t i = 0; i < value->as.items.len; ++i) json_free(&value->as.items.data[i]);
        free(value->as.items.data);
        break;
    case JSON_OBJECT:
        for (size_t i = 0; i < value->as.members.len; ++i) {
            free(value->as.members.data[i].key.data);
            json_free(&value->as.members.data[i].value);
        }
        free(value->as.members.data);
        break;
    default:
        break;
    }
    *value = (JsonValue){.kind = JSON_NULL};
}

const JsonValue* json_find(const JsonValue* value, const char* key, size_t key_len) {
    if (!value || value->kind != JSON_OBJECT) return NULL;
    for (size_t i = 0; i < value->as.members.len; ++i) {
        const JsonMember* member = &value->as.members.data[i];
        if (member->key.len == key_len && memcmp(member->key.data, key, key_len) == 0) return &member->value;
    }
    return NULL;
}

bool json_bool_or(const JsonValue* value, const char* key, size_t key_len, bool fallback) {
    const JsonValue* found = json_find(value, key, key_len);
    return found && found->kind == JSON_BOOL ? found->as.boolean : fallback;
}

double json_number_or(const JsonValue* value, const char* key, size_t key_len, double fallback) {
    const JsonValue* found = json_find(value, key, key_len);
    return found && found->kind == JSON_NUMBER ? found->as.number : fallback;
}

JsonText json_text_or(const JsonValue* value, const char* key, size_t key_len) {
    return json_as_text(json_find(value, key, key_len));
}

JsonText json_as_text(const JsonValue* value) {
    if (!value || value->kind != JSON_STRING) return (JsonText){"", 0};
    return (JsonText){value->as.text.data, value->as.text.len};
}
