#pragma once

#include <cstdint>
#include <vector>

namespace bench {

/// Contact solver: 8 frames of substepped sequential impulses over 4096
/// bodies and 16384 contacts, with accumulated-impulse clamping. Float math
/// through indirect body indices, the shape of a rigid-body solve loop.
class Solve {
public:
    static constexpr const char* name = "solve";

    struct Body { float px, py, pz, vx, vy, vz, inv_mass; };
    struct Contact { uint32_t a, b; float nx, ny, nz, depth, impulse; };

    Solve();
    uint64_t run();

private:
    static constexpr int bodies_n = 4096;
    static constexpr int contacts_n = 16384;
    static constexpr int substeps = 8;
    static constexpr int iters = 4;
    static constexpr int frames = 8;

    void integrate_gravity(float h);
    void solve_contacts(float h);
    void integrate_positions(float h);
    uint64_t checksum() const;

    std::vector<Body> bodies_;
    std::vector<Body> initial_;
    std::vector<Contact> contacts_;
};

} // namespace bench
