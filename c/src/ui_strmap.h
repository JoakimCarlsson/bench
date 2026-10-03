#ifndef BENCH_UI_STRMAP_H
#define BENCH_UI_STRMAP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

/// Borrowed string: a pointer and a length, not NUL terminated.
typedef struct {
    const char* data;
    size_t size;
} StrView;

/// A view of the string literal `literal`.
#define UI_SV(literal) ((StrView){ "" literal, sizeof(literal) - 1 })

/// String owned by its holder: malloc'd bytes and a length. A zeroed value is empty.
typedef struct {
    char* data;
    size_t size;
} OwnedStr;

/// A view of the NUL terminated string `text`.
static inline StrView ui_sv_of(const char* text) { return (StrView){ text, strlen(text) }; }

/// Whether two views hold the same bytes.
static inline bool ui_sv_equal(StrView a, StrView b) {
    return a.size == b.size && (a.size == 0 || memcmp(a.data, b.data, a.size) == 0);
}

/// A view of the bytes of `str`.
static inline StrView ui_str_view(const OwnedStr* str) { return (StrView){ str->data, str->size }; }

/// Replaces the contents of `str` with a copy of `value`.
void ui_str_assign(OwnedStr* str, StrView value);

/// Releases `str` and leaves it empty.
void ui_str_free(OwnedStr* str);

/// One chained entry of a `StrMap`.
typedef struct StrMapNode StrMapNode;

/// Hash map from byte strings to fixed-size values. Keys are copied into the
/// entries and every entry is one allocation, so a map owns all it points to.
/// A zeroed map with `value_size` set is empty and allocates nothing.
typedef struct {
    StrMapNode** buckets;
    size_t bucket_count;
    size_t count;
    size_t value_size;
} StrMap;

/// An empty map whose values are `value_size` bytes.
void ui_strmap_init(StrMap* map, size_t value_size);

/// The value stored under `key`, or null.
void* ui_strmap_find(const StrMap* map, StrView key);

/// The value stored under `key`, created zeroed when absent; `*created` tells which, and may be null.
void* ui_strmap_emplace(StrMap* map, StrView key, bool* created);

/// Stores a copy of `value` under `key`, replacing an earlier one.
void ui_strmap_set(StrMap* map, StrView key, const void* value);

/// Frees every entry, first calling `destroy_value` on each value when it is not null.
void ui_strmap_clear(StrMap* map, void (*destroy_value)(void* value));

#endif
