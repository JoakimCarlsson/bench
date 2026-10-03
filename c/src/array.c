#include "array.h"

#include "alloc.h"

enum { ARRAY_ALIGNMENT = 64, ARRAY_MIN_CAPACITY = 8 };

void* array_grow(void* data, size_t len, size_t* cap, size_t needed, size_t size) {
    size_t grown = *cap * 2;
    if (grown < needed) grown = needed;
    if (grown < ARRAY_MIN_CAPACITY) grown = ARRAY_MIN_CAPACITY;
    void* fresh = xalloc_aligned(grown * size, ARRAY_ALIGNMENT);
    if (len != 0) memcpy(fresh, data, len * size);
    xfree_aligned(data);
    *cap = grown;
    return fresh;
}

void array_release(void* data) { xfree_aligned(data); }
