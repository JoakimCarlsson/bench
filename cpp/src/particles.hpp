#pragma once

#include <cstdint>
#include <memory>

namespace bench {

/// Particle emitters: 12 emitters with 2048-particle pools run 4 updates of
/// 6 substeps each through a port of the engine's `ParticleWorld` step.
/// Each substep integrates every live particle with seeded parameter
/// curves, drag, gravity, wind, curl-noise turbulence and collision against
/// a ground plane and static boxes, emits continuously and in bursts by the
/// effect's phase, and spawns sub emitter particles. Particle colour, size
/// and angle come from gradients and curves each step.
///
/// Deviations from the engine, all to keep the results bit-identical across
/// languages (only `+ - * /`, `sqrt` and `floor` are allowed on floats):
/// - `sin` and `cos` (spread cone, hue rotation) are Taylor polynomials.
/// - The wind response `1 - exp(-k)` is `k / (1 + k)`.
/// - The turbulence blend `1 - pow(1 - i, s)` is `i * s`.
/// - `fmod(phase, 1)` is `phase - floor(phase)`, which is exact below 2.
/// - Emission shapes are Point and Box; Sphere, SphereSurface and Ring need
///   `sin`, `cos` and `cbrt`.
/// - The collider is a ground plane and six boxes swept with the slab test
///   of the engine's grid collider, not a voxel grid walked cell by cell.
/// - Emitters simulate in world space only; local coordinates, the gas
///   solver, manual spawn requests and the effect field are left out.
/// - The substep count is fixed at 6 instead of `ceil(longest / max_step)`,
///   and the preprocess is 45 fixed steps run with the collider, not
///   `ceil(preprocess / step)` steps without one.
class Particles {
public:
    static constexpr const char* name = "particles";

    Particles();
    ~Particles();
    uint64_t run();

private:
    struct State;
    std::unique_ptr<State> state_;
};

} // namespace bench
