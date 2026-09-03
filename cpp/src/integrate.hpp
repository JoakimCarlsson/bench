#pragma once

#include <cstdint>
#include <vector>

namespace bench {

/// Rigid-body integration: 32 steps over 65536 bodies, each with a position,
/// a velocity, an angular velocity and a quaternion that is renormalised every
/// step. Streaming float math with no indirection.
class Integrate {
public:
    static constexpr const char* name = "integrate";

    struct Body { float px, py, pz, vx, vy, vz, wx, wy, wz, qx, qy, qz, qw; };

    Integrate();
    uint64_t run();

private:
    static constexpr int bodies_n = 65536;
    static constexpr int steps = 32;

    void step(float dt);

    std::vector<Body> bodies_;
    std::vector<Body> initial_;
};

} // namespace bench
