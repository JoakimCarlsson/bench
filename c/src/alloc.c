#include "alloc.h"

#include <stdio.h>
#include <stdlib.h>

void* xalloc(size_t bytes) {
    void* p = calloc(1, bytes);
    if (!p) {
        fprintf(stderr, "out of memory\n");
        exit(2);
    }
    return p;
}
