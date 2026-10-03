#ifndef BENCH_ARRAY_H
#define BENCH_ARRAY_H

#include <stddef.h>
#include <stdint.h>
#include <string.h>

/// Growable array of `T`: `len` elements in use out of `cap` allocated, on
/// storage aligned for SIMD types. A zeroed struct is an empty array.
#define ARRAY_OF(T) struct { T* data; size_t len; size_t cap; }

/// Storage for at least `needed` elements of `size` bytes holding the first
/// `len` elements of `data`, which it frees; updates `*cap`.
void* array_grow(void* data, size_t len, size_t* cap, size_t needed, size_t size);

/// Ensures room for `n` elements.
#define ARRAY_RESERVE(a, n) \
    ((n) > (a).cap ? (void)((a).data = array_grow((a).data, (a).len, &(a).cap, (n), sizeof *(a).data)) : (void)0)

/// Appends one element.
#define ARRAY_PUSH(a, ...) (ARRAY_RESERVE((a), (a).len + 1), (void)((a).data[(a).len++] = (__VA_ARGS__)))

/// Sets the length to `n`, zero-filling any new elements.
#define ARRAY_RESIZE(a, n)                                                                     \
    do {                                                                                       \
        size_t array_n_ = (n);                                                                 \
        ARRAY_RESERVE((a), array_n_);                                                          \
        if (array_n_ > (a).len) memset((a).data + (a).len, 0, (array_n_ - (a).len) * sizeof *(a).data); \
        (a).len = array_n_;                                                                    \
    } while (0)

/// The last element.
#define ARRAY_BACK(a) ((a).data[(a).len - 1])

/// Removes and yields the last element.
#define ARRAY_POP(a) ((a).data[--(a).len])

/// Releases the storage and leaves an empty array.
#define ARRAY_FREE(a) (array_release((a).data), (a).data = NULL, (a).len = 0, (a).cap = 0)

/// Releases storage from `array_grow`.
void array_release(void* data);

typedef ARRAY_OF(uint32_t) U32Array;
typedef ARRAY_OF(int32_t) I32Array;
typedef ARRAY_OF(uint64_t) U64Array;
typedef ARRAY_OF(uint8_t) U8Array;

#endif
