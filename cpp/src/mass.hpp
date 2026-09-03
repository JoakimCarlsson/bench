#pragma once

#include <cstdint>
#include <vector>

namespace bench {

/// Mass properties: for 128 bodies of 32^3 voxels with per-material density,
/// accumulate mass, centre of mass and the inertia tensor voxel by voxel, then
/// shift the tensor to the centre of mass.
class Mass {
public:
    static constexpr const char* name = "mass";

    struct Props { float mass, com[3], inertia[6]; };

    Mass();
    uint64_t run();

private:
    static constexpr int bodies_n = 128;
    static constexpr int dim = 32;
    static constexpr int voxels = dim * dim * dim;

    static Props properties(const uint8_t* mat);

    std::vector<uint8_t> mat_;
};

} // namespace bench
