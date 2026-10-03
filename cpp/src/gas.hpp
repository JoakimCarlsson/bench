#pragma once

#include <array>
#include <cstdint>
#include <vector>

#include "vecmath.hpp"

namespace bench {

/// Particle gas: 4 substeps of the engine's gas solver over 8192 particles
/// stirred by 4 moving boxes. Each substep hashes particles into cells,
/// sorts the keys, searches the 27 neighbouring cells with lower_bound for
/// pressure and viscosity, pushes particles out of and along the movers,
/// then integrates positions.
class Gas {
public:
    static constexpr const char* name = "gas";

    Gas();
    uint64_t run();

    struct Particle {
        vm::Vec3 position{};
        vm::Vec3 velocity{};
        float size{};
        uint32_t seed{};
    };

    struct Mover {
        vm::Transform transform{};
        vm::Vec3 half_extents{};
        vm::Vec3 velocity{};
        vm::Vec3 angular_velocity{};
    };

    /// A particle index keyed by its spatial hash cell.
    struct Entry {
        uint64_t key{};
        uint32_t index{};
    };

private:
    static constexpr uint32_t particles_n = 8192;
    static constexpr uint32_t movers_n = 4;
    static constexpr int substeps = 4;

    /// Pairwise push, velocity blend and weight from overlapping particles.
    void accumulate_pressure();
    /// Pressure, viscosity and stirring for every particle.
    void resolve();

    std::vector<Particle> initial_;
    std::vector<Particle> particles_;
    std::array<Mover, movers_n> movers_{};
    std::vector<Entry> entries_;
    std::vector<vm::Vec3> push_;
    std::vector<vm::Vec3> blend_;
    std::vector<float> weight_;
};

} // namespace bench
