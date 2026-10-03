#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "anim.hpp"
#include "boxbox.hpp"
#include "broadphase.hpp"
#include "bvh.hpp"
#include "chunkmap.hpp"
#include "colour.hpp"
#include "dda.hpp"
#include "decompose.hpp"
#include "flood.hpp"
#include "gas.hpp"
#include "harness.hpp"
#include "integrate.hpp"
#include "islands.hpp"
#include "json.hpp"
#include "mass.hpp"
#include "mips.hpp"
#include "particles.hpp"
#include "raycast.hpp"
#include "slotmap.hpp"
#include "solve.hpp"
#include "sort.hpp"
#include "surface.hpp"
#include "sweep.hpp"
#include "transform.hpp"
#include "ui.hpp"
#include "unproject.hpp"
#include "wide.hpp"
#include "world.hpp"

namespace {

/// Whether the kernel `name` was asked for: no names given runs everything.
bool selected(const char* name, int argc, char** argv) {
    if (argc <= 3) return true;
    for (int i = 3; i < argc; i++) {
        if (std::strcmp(argv[i], name) == 0) return true;
    }
    return false;
}

template <typename... Cases>
int run_all(int reps, int warmup, int argc, char** argv) {
    int rc = 0;
    ((rc = rc || !selected(Cases::name, argc, argv) ? rc : bench::run_case<Cases>(Cases::name, reps, warmup)), ...);
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
    return run_all<Dda, Flood, Surface, Mips, Mass, Sweep, Bvh, Islands, Colour, Solve, Integrate, Transform, Unproject, Decompose, Raycast, BoxBox, Wide, BroadPhaseCase, Gas, WorldCase<1>, WorldCase<2>, WorldCase<4>, WorldCase<8>, WorldCase<16>, Particles, Json, Anim, Ui, SlotMap, ChunkMap, Sort>(reps, warmup, argc, argv);
}
