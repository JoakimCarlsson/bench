#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <vector>

#include "constraint_graph.hpp"
#include "contact.hpp"
#include "rigid_body.hpp"
#include "simd.hpp"
#include "task_pool.hpp"
#include "vecmath.hpp"

namespace bench {

/// The engine's solver_types.hpp: vectors, quaternions, symmetric matrices
/// and constraints whose components are scalars or SIMD lanes.
namespace solver {

inline constexpr uint32_t max_points = 4;
inline constexpr uint32_t null_index = 0xFFFFFFFFu;
inline constexpr uint32_t lanes = 8;

template <class F> struct V3 {
    F x{};
    F y{};
    F z{};
};

template <class F> struct Q4 {
    F x{};
    F y{};
    F z{};
    F w{};
};

/// Symmetric 3x3 matrix stored as its six unique components.
template <class F> struct Sym3 {
    F xx{};
    F xy{};
    F xz{};
    F yy{};
    F yz{};
    F zz{};
};

/// Per-body velocities and accumulated pose delta, padded to 64 bytes for
/// four-float lane loads.
struct alignas(16) BodyState {
    vm::Vec3 velocity{};
    float pad_velocity{};
    vm::Vec3 angular_velocity{};
    float pad_angular{};
    vm::Vec3 delta_position{};
    float pad_position{};
    vm::Quat delta_rotation{};
};

static_assert(sizeof(BodyState) == 64);

/// Solver data of one contact point.
template <class F> struct PointConstraint {
    V3<F> anchor_a{};
    V3<F> anchor_b{};
    F base_separation{};
    F normal_mass{};
    F relative_velocity{};
    F normal_impulse{};
    F total_normal_impulse{};
    F peak_normal_impulse{};
    F lever_arm{};
};

/// Number of lanes in a value type.
template <class F> inline constexpr uint32_t width_of = lanes;
template <> inline constexpr uint32_t width_of<float> = 1;

/// Solver data of one contact manifold, or of one lane per contact in a
/// bundle.
template <class F> struct ContactConstraint {
    std::array<uint32_t, width_of<F>> body_a{};
    std::array<uint32_t, width_of<F>> body_b{};
    std::array<uint32_t, width_of<F>> contact{};
    uint32_t point_count{};
    F inverse_mass_a{};
    F inverse_mass_b{};
    Sym3<F> inverse_inertia_a{};
    Sym3<F> inverse_inertia_b{};
    V3<F> normal{};
    V3<F> tangent1{};
    V3<F> tangent2{};
    F bias_rate{};
    F mass_scale{};
    F impulse_scale{};
    F friction{};
    F restitution{};
    F rolling_resistance{};
    std::array<PointConstraint<F>, max_points> points{};
    V3<F> friction_anchor_a{};
    V3<F> friction_anchor_b{};
    F tangent_mass_xx{};
    F tangent_mass_xy{};
    F tangent_mass_yy{};
    F tangent_impulse_x{};
    F tangent_impulse_y{};
    F twist_mass{};
    F twist_impulse{};
    Sym3<F> rolling_mass{};
    V3<F> rolling_impulse{};
};

} // namespace solver

namespace phys {

/// Position and rotation change of a body over one solve.
struct BodyDelta {
    vm::Vec3 position{};
    vm::Quat rotation{};
};

/// Step timing, gravity and softness shared by every constraint in a solve.
struct SolverContext {
    float dt{};
    float inv_dt{};
    float h{};
    float inv_h{};
    uint32_t substeps{};
    vm::Vec3 gravity{};
    Softness contact_softness{};
    Softness static_softness{};
    float push_out_speed{};
    float restitution_threshold{};
    float linear_slop{};
};

/// Velocity and rotation centre of a static body.
struct KinematicMotion {
    vm::Vec3 velocity{};
    vm::Vec3 angular_velocity{};
    vm::Vec3 center{};
};

/// A dynamic body taking part in the solve and its slot.
struct ActiveBody {
    RigidBody* body{};
    uint32_t slot{};
};

/// Everything one solve reads and writes.
struct SolverInputs {
    std::span<Contact> contacts;
    std::span<const std::span<const uint32_t>> colors;
    std::span<const uint32_t> body_local;
    std::span<const ActiveBody> active_bodies;
    std::span<const KinematicMotion> static_motions;
    std::span<BodyDelta> deltas;
    SolverContext context{};
    TaskPool* pool{};
};

/// The engine's soft step contact solver on eight lanes: bundles each colour
/// of the constraint graph into lanes and runs the stages of every substep,
/// in parallel when given a pool. The overflow colour is not supported.
class ContactSolver final {
public:
    /// Solves every contact for one step and writes velocities, poses,
    /// impulses and deltas back.
    void solve(const SolverInputs& inputs);

private:
    struct BodyProps {
        vm::Vec3 center{};
        float inverse_mass{};
        vm::Basis inverse_inertia{vm::Basis{} * 0.0f};
        vm::Quat rotation{};
        vm::Vec3 force{};
        vm::Vec3 torque{};
        float linear_damping{};
        float angular_damping{};
    };

    enum class StageKind : uint8_t { IntegrateVelocities, WarmStart, SolveBiased, IntegratePositions, SolveRelax, Restitution };

    /// A run of blocks of one kind, executed between barriers.
    struct Stage {
        StageKind kind{};
        uint32_t color{};
        uint32_t begin{};
        uint32_t end{};
        uint32_t grain{};
        uint32_t blocks{};
    };

    struct StageProgress {
        std::atomic<uint32_t> next{0};
        std::atomic<uint32_t> done{0};
    };

    struct Entry {
        uint32_t contact{};
        uint32_t body_a{};
        uint32_t body_b{};
        bool b_fixed{};
    };

    using Bundle = solver::ContactConstraint<simd::FloatW8>;
    using Scalar = solver::ContactConstraint<float>;

    /// Fills body states and properties from the active bodies.
    void build_bodies(const SolverInputs& inputs);
    /// Slot of a static body, created read-only on first use.
    uint32_t read_only_static(const SolverInputs& inputs, uint32_t fixed_index);
    /// Lists contacts colour by colour with their body slots.
    void collect_entries(const SolverInputs& inputs);
    /// Builds the scalar constraint of one contact.
    void prepare_entry(const SolverInputs& inputs, const Entry& entry, Scalar& out) const;
    /// Prepares every constraint, in parallel when large enough, and bundles
    /// them.
    void prepare_constraints(const SolverInputs& inputs);
    /// Sorts constraints by colour and packs each colour into bundles.
    void bundle_colors(const SolverInputs& inputs);
    /// Builds the stage list for every substep plus restitution.
    void plan_stages(const SolverContext& context);
    /// Adds one stage per colour for a constraint kind.
    void add_constraint_stages(StageKind kind);
    /// Appends a stage, ignoring empty ranges.
    void add_stage(StageKind kind, uint32_t color, uint32_t begin, uint32_t end, uint32_t grain);
    /// Executes one block of a stage.
    void run_block(const Stage& stage, uint32_t block, const SolverContext& context);
    /// Runs all stages, spread over the pool when there are enough contacts.
    void run_stages(const SolverContext& context, TaskPool* pool);
    /// Writes impulses, velocities, poses and deltas back.
    void store_results(const SolverInputs& inputs);

    std::vector<solver::BodyState> states_;
    std::vector<BodyProps> props_;
    std::vector<uint32_t> static_lookup_;
    uint32_t writable_count_{};
    uint32_t dummy_index_{};
    std::vector<Entry> entries_;
    std::vector<uint8_t> colors_;
    std::vector<Scalar> scalars_;
    std::vector<uint32_t> order_;
    std::vector<Bundle> bundles_;
    std::array<uint32_t, graph_color_count + 1> color_start_{};
    std::array<uint32_t, graph_color_count> color_begin_{};
    std::array<uint32_t, graph_color_count> color_end_{};
    std::vector<Stage> stages_;
    std::unique_ptr<StageProgress[]> progress_;
    std::size_t progress_capacity_{};
};

} // namespace phys

} // namespace bench
