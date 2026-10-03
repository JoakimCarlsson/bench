#include <cstdio>
#include <cstdlib>

#include "boxbox.hpp"
#include "bvh.hpp"
#include "chunkmap.hpp"
#include "colour.hpp"
#include "dda.hpp"
#include "decompose.hpp"
#include "flood.hpp"
#include "harness.hpp"
#include "integrate.hpp"
#include "islands.hpp"
#include "mass.hpp"
#include "mips.hpp"
#include "raycast.hpp"
#include "slotmap.hpp"
#include "solve.hpp"
#include "sort.hpp"
#include "surface.hpp"
#include "sweep.hpp"
#include "transform.hpp"
#include "unproject.hpp"

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
    return run_all<Dda, Flood, Surface, Mips, Mass, Sweep, Bvh, Islands, Colour, Solve, Integrate, Transform, Unproject, Decompose, Raycast, BoxBox, SlotMap, ChunkMap, Sort>(reps, warmup);
}
