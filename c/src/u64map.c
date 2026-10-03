#include "u64map.h"

#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "hash.h"

#define EMPTY_KEY UINT64_MAX

enum { MIN_CAPACITY = 64 };

/// Home slot of a key.
static size_t home_slot(const U64Map* map, uint64_t key) { return (size_t)(mix64(key) & (map->capacity - 1)); }

/// Slot holding `key`, or the empty slot where it would go.
static size_t find_slot(const U64Map* map, uint64_t key) {
    size_t i = home_slot(map, key);
    while (map->keys[i] != EMPTY_KEY && map->keys[i] != key) i = (i + 1) & (map->capacity - 1);
    return i;
}

/// Rebuilds the table with `capacity` slots.
static void rehash(U64Map* map, size_t capacity) {
    uint64_t* old_keys = map->keys;
    uint32_t* old_values = map->values;
    size_t old_capacity = map->capacity;
    map->keys = xalloc(capacity * sizeof(uint64_t));
    map->values = xalloc(capacity * sizeof(uint32_t));
    memset(map->keys, 0xff, capacity * sizeof(uint64_t));
    map->capacity = capacity;
    for (size_t i = 0; i < old_capacity; i++) {
        if (old_keys[i] == EMPTY_KEY) continue;
        size_t slot = find_slot(map, old_keys[i]);
        map->keys[slot] = old_keys[i];
        map->values[slot] = old_values[i];
    }
    free(old_keys);
    free(old_values);
}

void u64map_free(U64Map* map) {
    free(map->keys);
    free(map->values);
    *map = (U64Map){ 0 };
}

void u64map_clear(U64Map* map) {
    if (map->capacity != 0) memset(map->keys, 0xff, map->capacity * sizeof(uint64_t));
    map->count = 0;
}

bool u64map_contains(const U64Map* map, uint64_t key) {
    if (map->capacity == 0) return false;
    return map->keys[find_slot(map, key)] == key;
}

bool u64map_insert(U64Map* map, uint64_t key, uint32_t value) {
    if ((map->count + 1) * 2 > map->capacity) rehash(map, map->capacity < MIN_CAPACITY ? MIN_CAPACITY : map->capacity * 2);
    size_t slot = find_slot(map, key);
    if (map->keys[slot] == key) return false;
    map->keys[slot] = key;
    map->values[slot] = value;
    map->count++;
    return true;
}

bool u64map_erase(U64Map* map, uint64_t key) {
    if (map->capacity == 0) return false;
    size_t mask = map->capacity - 1;
    size_t hole = find_slot(map, key);
    if (map->keys[hole] != key) return false;
    size_t next = hole;
    for (;;) {
        next = (next + 1) & mask;
        if (map->keys[next] == EMPTY_KEY) break;
        size_t home = home_slot(map, map->keys[next]);
        bool stays = hole <= next ? (hole < home && home <= next) : (hole < home || home <= next);
        if (stays) continue;
        map->keys[hole] = map->keys[next];
        map->values[hole] = map->values[next];
        hole = next;
    }
    map->keys[hole] = EMPTY_KEY;
    map->count--;
    return true;
}
