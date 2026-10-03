#pragma once

#include <cstdint>
#include <vector>

#include "vecmath.hpp"

namespace bench {

/// Camera picking: for 131072 cameras build the view from a look-at, the
/// perspective projection and their product, invert it by cofactors, and
/// unproject four NDC points to world space and back.
class Unproject {
public:
    static constexpr const char* name = "unproject";

    struct Camera { vm::Vec3 eye, target; float tan_half_fov, aspect, z_near, z_far; };

    Unproject();
    uint64_t run();

private:
    static constexpr uint32_t cameras_n = 131072;

    std::vector<Camera> cameras_;
};

} // namespace bench
