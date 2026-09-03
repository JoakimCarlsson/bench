#include <cstdio>
#include <cstdlib>

#include "dda.hpp"
#include "flood.hpp"
#include "harness.hpp"
#include "slotmap.hpp"
#include "solve.hpp"

int main(int argc, char** argv) {
    int reps = argc > 1 ? std::atoi(argv[1]) : 10;
    int warmup = argc > 2 ? std::atoi(argv[2]) : 2;
    if (reps < 1 || reps > 1024) {
        std::fprintf(stderr, "reps must be 1..1024\n");
        return 1;
    }
    if (int rc = bench::run_case<bench::Dda>(bench::Dda::name, reps, warmup)) return rc;
    if (int rc = bench::run_case<bench::Flood>(bench::Flood::name, reps, warmup)) return rc;
    if (int rc = bench::run_case<bench::Solve>(bench::Solve::name, reps, warmup)) return rc;
    if (int rc = bench::run_case<bench::SlotMap>(bench::SlotMap::name, reps, warmup)) return rc;
    return 0;
}
