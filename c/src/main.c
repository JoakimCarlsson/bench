#include <stdio.h>
#include <stdlib.h>

#include "chunkmap.h"
#include "dda.h"
#include "flood.h"
#include "harness.h"
#include "integrate.h"
#include "slotmap.h"
#include "solve.h"
#include "sort.h"
#include "surface.h"
#include "sweep.h"

static const Case* const cases[] = {
    &dda_case,     &flood_case,   &surface_case, &sweep_case, &solve_case,
    &integrate_case, &slotmap_case, &chunkmap_case, &sort_case,
};

int main(int argc, char** argv) {
    int reps = argc > 1 ? atoi(argv[1]) : 10;
    int warmup = argc > 2 ? atoi(argv[2]) : 2;
    if (reps < 1 || reps > 1024) {
        fprintf(stderr, "reps must be 1..1024\n");
        return 1;
    }
    for (size_t i = 0; i < sizeof cases / sizeof cases[0]; i++) {
        int rc = run_case(cases[i], reps, warmup);
        if (rc) return rc;
    }
    return 0;
}
