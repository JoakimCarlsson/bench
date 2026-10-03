#pragma once

#include <cstdint>
#include <vector>

#include "vecmath.hpp"

namespace bench {

/// Transform decomposition: for 262144 rotation-and-scale pairs, a quarter of
/// them mirrored, build the basis, recover its scale and rotation through
/// the four-branch basis-to-quaternion conversion, invert it, and rotate a
/// point both ways.
class Decompose {
public:
    static constexpr const char* name = "decompose";

    struct Item { vm::Quat rotation; vm::Vec3 scale, point; };

    Decompose();
    uint64_t run();

private:
    static constexpr uint32_t items_n = 262144;

    std::vector<Item> items_;
};

} // namespace bench
