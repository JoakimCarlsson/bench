#pragma once

#include <cstdint>
#include <vector>

#include "vecmath.hpp"

namespace bench {

/// Scene-graph propagation: 8 frames over a 4-ary tree of 65536 nodes. Each
/// node advances its rotation, builds a scaled local transform, composes it
/// with its parent's world transform, and multiplies the result into a
/// model-view-projection matrix.
class Transform {
public:
    static constexpr const char* name = "transform";

    struct Node { vm::Vec3 position, scale, spin; uint32_t parent; };

    Transform();
    uint64_t run();

private:
    static constexpr uint32_t nodes_n = 65536;
    static constexpr int frames = 8;

    /// Advance every node one frame; parents come before their children.
    void frame();

    std::vector<Node> nodes_;
    std::vector<vm::Quat> initial_;
    std::vector<vm::Quat> rotations_;
    std::vector<vm::Transform> world_;
    std::vector<vm::Mat4> mvp_;
    vm::Mat4 view_projection_;
};

} // namespace bench
