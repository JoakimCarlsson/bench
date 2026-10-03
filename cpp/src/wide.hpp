#pragma once

#include <array>
#include <cstdint>
#include <span>
#include <vector>

#include "constraint_graph.hpp"
#include "contact.hpp"
#include "contact_solver.hpp"
#include "rigid_body.hpp"

namespace bench {

/// The engine's eight-lane contact solver, single-threaded: 4 steps of a
/// 32x32 field of 8-box stacks, 8192 bodies and about 12k contacts coloured
/// by the engine's constraint graph. Each step prepares constraints, packs
/// each colour into eight-lane bundles, runs 4 substeps of warm start, soft
/// biased solve, position integration and relaxed solve with friction, then
/// restitution, and writes impulses and poses back.
class Wide {
public:
    static constexpr const char* name = "wide";

    Wide();
    uint64_t run();

private:
    static constexpr uint32_t grid = 32;
    static constexpr uint32_t height = 8;
    static constexpr uint32_t bodies_n = grid * grid * height;
    static constexpr int steps = 4;

    phys::SolverContext context_{};
    std::vector<phys::RigidBody> initial_bodies_;
    std::vector<phys::Contact> initial_contacts_;
    std::vector<phys::RigidBody> bodies_;
    std::vector<phys::Contact> contacts_;
    phys::ConstraintGraph graph_;
    std::array<std::span<const uint32_t>, phys::graph_color_count> colors_{};
    std::vector<uint32_t> body_local_;
    std::vector<phys::ActiveBody> active_;
    std::vector<phys::KinematicMotion> static_motions_;
    std::vector<phys::BodyDelta> deltas_;
    phys::ContactSolver solver_;
};

} // namespace bench
