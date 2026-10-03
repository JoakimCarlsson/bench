#include "alloc.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef _WIN32
#include <malloc.h>
#endif

/// Exits the process after reporting that memory ran out.
static void out_of_memory(void) {
    fprintf(stderr, "out of memory\n");
    exit(2);
}

void* xalloc(size_t bytes) {
    void* p = calloc(1, bytes);
    if (!p) out_of_memory();
    return p;
}

void* xalloc_aligned(size_t bytes, size_t alignment) {
    size_t rounded = (bytes + alignment - 1) / alignment * alignment;
    if (rounded == 0) rounded = alignment;
#ifdef _WIN32
    void* p = _aligned_malloc(rounded, alignment);
#else
    void* p = aligned_alloc(alignment, rounded);
#endif
    if (!p) out_of_memory();
    return p;
}

void xfree_aligned(void* p) {
#ifdef _WIN32
    _aligned_free(p);
#else
    free(p);
#endif
}
