#pragma once

#include <cstdint>
#include <vector>

#include "vecmath.hpp"

namespace bench {

/// Closest-hit ray casts: 1024 rays, each tested against 1024 oriented boxes
/// with the slab test in box space, shrinking the search to the closest hit
/// so far.
class Raycast {
public:
    static constexpr const char* name = "raycast";

    struct Ray { vm::Vec3 origin, translation; };
    struct Hit {
        vm::Vec3 point{};
        vm::Vec3 normal{};
        float fraction{};
        bool hit{};
    };

    Raycast();
    uint64_t run();

private:
    static constexpr uint32_t boxes_n = 1024;
    static constexpr uint32_t rays_n = 1024;

    std::vector<vm::BoxPose> boxes_;
    std::vector<Ray> rays_;
};

} // namespace bench
