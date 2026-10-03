#ifndef BENCH_JSON_VALUE_H
#define BENCH_JSON_VALUE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// The type of a JSON value.
typedef enum { JSON_NULL, JSON_BOOL, JSON_NUMBER, JSON_STRING, JSON_ARRAY, JSON_OBJECT } JsonKind;

typedef struct JsonValue JsonValue;
typedef struct JsonMember JsonMember;

/// An owned, growable string: `len` bytes in use out of `cap` allocated.
typedef struct {
    char* data;
    size_t len;
    size_t cap;
} JsonString;

/// A borrowed run of bytes.
typedef struct {
    const char* data;
    size_t len;
} JsonText;

/// A JSON value. Strings, arrays and objects own their storage, one malloc
/// per growable buffer; objects keep members in insertion order.
struct JsonValue {
    JsonKind kind;
    union {
        bool boolean;
        double number;
        JsonString text;
        struct {
            JsonValue* data;
            size_t len;
            size_t cap;
        } items;
        struct {
            JsonMember* data;
            size_t len;
            size_t cap;
        } members;
    } as;
};

/// An object member: an owned key and its value.
struct JsonMember {
    JsonString key;
    JsonValue value;
};

/// Parses `len` bytes of JSON text into `out`. `source[len]` must be a NUL
/// byte. Returns false on malformed input, trailing content or nesting deeper
/// than 128, leaving `out` null.
bool json_parse(const char* source, size_t len, JsonValue* out);

/// Releases everything `value` owns and leaves it null.
void json_free(JsonValue* value);

/// Looks up an object member; null when absent or `value` is not an object.
const JsonValue* json_find(const JsonValue* value, const char* key, size_t key_len);

/// Looks up a member by string literal.
#define JSON_FIND(value, key) json_find((value), (key), sizeof(key) - 1)

/// Reads a boolean member, or `fallback` when absent or not a boolean.
bool json_bool_or(const JsonValue* value, const char* key, size_t key_len, bool fallback);

/// Reads a number member, or `fallback` when absent or not a number.
double json_number_or(const JsonValue* value, const char* key, size_t key_len, double fallback);

/// Reads a string member without copying it, or an empty string when absent
/// or not a string.
JsonText json_text_or(const JsonValue* value, const char* key, size_t key_len);

/// The bytes of a string value; empty for any other kind.
JsonText json_as_text(const JsonValue* value);

#define JSON_BOOL_OR(value, key, fallback) json_bool_or((value), (key), sizeof(key) - 1, (fallback))
#define JSON_NUMBER_OR(value, key, fallback) json_number_or((value), (key), sizeof(key) - 1, (fallback))
#define JSON_TEXT_OR(value, key) json_text_or((value), (key), sizeof(key) - 1)

#endif
