#include "ui_strmap.h"

#include <stdlib.h>

#include "alloc.h"

enum { STRMAP_FIRST_BUCKETS = 8, STRMAP_ALIGNMENT = 8 };

/// One entry: the header is followed by the value, then the key bytes.
struct StrMapNode {
    StrMapNode* next;
    uint64_t hash;
    size_t key_size;
};

/// FNV-1a over the bytes of `key`.
static uint64_t hash_key(StrView key) {
    uint64_t hash = 0xcbf29ce484222325ull;
    for (size_t index = 0; index < key.size; ++index) {
        hash ^= (uint8_t)key.data[index];
        hash *= 0x100000001b3ull;
    }
    return hash;
}

/// The size of the value area of an entry, rounded up so the key follows aligned.
static size_t value_stride(const StrMap* map) {
    return (map->value_size + STRMAP_ALIGNMENT - 1) / STRMAP_ALIGNMENT * STRMAP_ALIGNMENT;
}

/// The value inside `node`.
static void* node_value(StrMapNode* node) { return (char*)node + sizeof(StrMapNode); }

/// The key bytes inside `node`.
static const char* node_key(const StrMap* map, const StrMapNode* node) {
    return (const char*)node + sizeof(StrMapNode) + value_stride(map);
}

/// The entry for `key` with hash `hash`, or null.
static StrMapNode* find_node(const StrMap* map, StrView key, uint64_t hash) {
    if (map->count == 0) return NULL;
    StrMapNode* node = map->buckets[hash & (map->bucket_count - 1)];
    while (node != NULL) {
        if (node->hash == hash && node->key_size == key.size &&
                (key.size == 0 || memcmp(node_key(map, node), key.data, key.size) == 0)) {
            return node;
        }
        node = node->next;
    }
    return NULL;
}

/// Moves every entry into `bucket_count` buckets, a power of two.
static void rehash(StrMap* map, size_t bucket_count) {
    StrMapNode** buckets = xalloc(bucket_count * sizeof(StrMapNode*));
    for (size_t index = 0; index < map->bucket_count; ++index) {
        StrMapNode* node = map->buckets[index];
        while (node != NULL) {
            StrMapNode* next = node->next;
            size_t slot = node->hash & (bucket_count - 1);
            node->next = buckets[slot];
            buckets[slot] = node;
            node = next;
        }
    }
    free(map->buckets);
    map->buckets = buckets;
    map->bucket_count = bucket_count;
}

void ui_str_assign(OwnedStr* str, StrView value) {
    free(str->data);
    str->data = NULL;
    str->size = value.size;
    if (value.size == 0) return;
    str->data = xalloc(value.size);
    memcpy(str->data, value.data, value.size);
}

void ui_str_free(OwnedStr* str) {
    free(str->data);
    str->data = NULL;
    str->size = 0;
}

void ui_strmap_init(StrMap* map, size_t value_size) {
    map->buckets = NULL;
    map->bucket_count = 0;
    map->count = 0;
    map->value_size = value_size;
}

void* ui_strmap_find(const StrMap* map, StrView key) {
    if (map->count == 0) return NULL;
    StrMapNode* node = find_node(map, key, hash_key(key));
    return node == NULL ? NULL : node_value(node);
}

void* ui_strmap_emplace(StrMap* map, StrView key, bool* created) {
    uint64_t hash = hash_key(key);
    StrMapNode* node = find_node(map, key, hash);
    if (created != NULL) *created = node == NULL;
    if (node != NULL) return node_value(node);
    if (map->count >= map->bucket_count) rehash(map, map->bucket_count == 0 ? STRMAP_FIRST_BUCKETS : map->bucket_count * 2);
    node = xalloc(sizeof(StrMapNode) + value_stride(map) + key.size);
    node->hash = hash;
    node->key_size = key.size;
    memcpy((char*)node + sizeof(StrMapNode) + value_stride(map), key.data, key.size);
    size_t slot = hash & (map->bucket_count - 1);
    node->next = map->buckets[slot];
    map->buckets[slot] = node;
    map->count += 1;
    return node_value(node);
}

void ui_strmap_set(StrMap* map, StrView key, const void* value) {
    memcpy(ui_strmap_emplace(map, key, NULL), value, map->value_size);
}

void ui_strmap_clear(StrMap* map, void (*destroy_value)(void* value)) {
    for (size_t index = 0; index < map->bucket_count; ++index) {
        StrMapNode* node = map->buckets[index];
        while (node != NULL) {
            StrMapNode* next = node->next;
            if (destroy_value != NULL) destroy_value(node_value(node));
            free(node);
            node = next;
        }
    }
    free(map->buckets);
    ui_strmap_init(map, map->value_size);
}
