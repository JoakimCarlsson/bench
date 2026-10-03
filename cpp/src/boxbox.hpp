#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "contact.hpp"
#include "vecmath.hpp"

namespace bench {

/// Box-box narrowphase: 8 frames over 8192 pairs of oriented boxes, a
/// quarter of them stacked face to face. Separating-axis tests over 15 axes,
/// incident-face clipping against the reference face, reduction to four
/// points, and warm starting from the previous frame's manifold.
class BoxBox {
public:
    static constexpr const char* name = "boxbox";

    struct Pair { vm::BoxPose a, b; vm::Vec3 velocity; };

    BoxBox();
    uint64_t run();

private:
    static constexpr uint32_t pairs_n = 8192;
    static constexpr int frames = 8;

    std::vector<Pair> pairs_;
    std::vector<phys::Manifold> manifolds_;
};

} // namespace bench
