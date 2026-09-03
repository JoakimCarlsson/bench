#include <cstdio>
#include <cstdlib>

#include "chunkmap.hpp"
#include "dda.hpp"
#include "flood.hpp"
#include "harness.hpp"
#include "integrate.hpp"
#include "slotmap.hpp"
#include "solve.hpp"
#include "sort.hpp"
#include "surface.hpp"
#include "sweep.hpp"

namespace {

template <typename... Cases>
int run_all(int reps, int warmup) {
    int rc = 0;
    ((rc = rc ? rc : bench::run_case<Cases>(Cases::name, reps, warmup)), ...);
    return rc;
}

} // namespace

int main(int argc, char** argv) {
    int reps = argc > 1 ? std::atoi(argv[1]) : 10;
    int warmup = argc > 2 ? std::atoi(argv[2]) : 2;
    if (reps < 1 || reps > 1024) {
        std::fprintf(stderr, "reps must be 1..1024\n");
        return 1;
    }
    using namespace bench;
    return run_all<Dda, Flood, Surface, Sweep, Solve, Integrate, SlotMap, ChunkMap, Sort>(reps, warmup);
}
