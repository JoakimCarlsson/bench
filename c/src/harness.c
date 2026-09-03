#include "harness.h"

#include <stdio.h>
#include <stdlib.h>

#include "clock.h"

static int cmp_u64(const void* a, const void* b) {
    uint64_t x = *(const uint64_t*)a, y = *(const uint64_t*)b;
    return x < y ? -1 : x > y;
}

int run_case(const Case* c, int reps, int warmup) {
    void* state = calloc(1, c->state_size);
    uint64_t* times = calloc((size_t)reps, sizeof(uint64_t));
    if (!state || !times) { fprintf(stderr, "out of memory\n"); return 2; }

    c->setup(state);
    uint64_t checksum = 0;
    for (int i = 0; i < warmup + reps; i++) {
        uint64_t t0 = now_ns();
        uint64_t sum = c->run(state);
        uint64_t t1 = now_ns();
        if (i == 0) {
            checksum = sum;
        } else if (sum != checksum) {
            fprintf(stderr, "%s: nondeterministic\n", c->name);
            return 3;
        }
        if (i >= warmup) times[i - warmup] = t1 - t0;
    }
    c->teardown(state);

    qsort(times, (size_t)reps, sizeof(uint64_t), cmp_u64);
    printf("%s %llu %llu %016llx\n", c->name,
           (unsigned long long)times[0],
           (unsigned long long)times[reps / 2],
           (unsigned long long)checksum);

    free(times);
    free(state);
    return 0;
}
