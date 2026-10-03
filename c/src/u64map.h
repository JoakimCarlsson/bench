#ifndef BENCH_U64MAP_H
#define BENCH_U64MAP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/// Open-addressing hash map from 64-bit keys to 32-bit values, with linear
/// probing and backward-shift deletion. The all-ones key is reserved as the
/// empty marker. A zeroed struct is an empty map.
typedef struct {
    uint64_t* keys;
    uint32_t* values;
    size_t capacity;
    size_t count;
} U64Map;

/// Releases the storage and leaves an empty map.
void u64map_free(U64Map* map);
/// Removes every entry, keeping the storage.
void u64map_clear(U64Map* map);
/// Whether `key` is present.
bool u64map_contains(const U64Map* map, uint64_t key);
/// Adds `key` with `value` unless it is present; true when added.
bool u64map_insert(U64Map* map, uint64_t key, uint32_t value);
/// Removes `key`; true when it was present.
bool u64map_erase(U64Map* map, uint64_t key);
/// Number of entries.
static inline size_t u64map_size(const U64Map* map) { return map->count; }

#endif
