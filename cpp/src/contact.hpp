#pragma once

#include <array>
#include <cstdint>

#include "vecmath.hpp"

/// The engine's contact.hpp and the pieces of broad_phase.hpp it needs: a
/// contact between two box shapes, its manifold, friction state, cached poses
/// and the links that place it in sets, the graph and islands.
namespace bench::phys {

inline constexpr uint32_t max_manifold_points = 4;
inline constexpr uint32_t null_link = 0xFFFFFFFFu;
inline constexpr uint32_t awake_set = 0;
inline constexpr uint32_t disabled_set = 0xFFFFFFFEu;
inline constexpr float min_friction_weight = 1e-10f;

/// One shape: a rigid body slot, or a static body index.
struct ShapeRef {
    uint32_t body{};
    bool is_static{};

    constexpr bool operator==(const ShapeRef&) const = default;
};

/// The shape packed into 32 bits: static flag, body index and shape index 0.
constexpr uint32_t pack_shape_ref(ShapeRef ref) {
    return (ref.is_static ? 0x80000000u : 0u) | ((ref.body & 0x7FFFFu) << 12);
}

/// Order-independent key of a shape pair.
constexpr uint64_t pair_key(ShapeRef a, ShapeRef b) {
    const uint32_t pa = pack_shape_ref(a);
    const uint32_t pb = pack_shape_ref(b);
    const uint32_t lo = pa < pb ? pa : pb;
    const uint32_t hi = pa < pb ? pb : pa;
    return (static_cast<uint64_t>(lo) << 32) | hi;
}

struct ManifoldPoint {
    vm::Vec3 point{};
    float separation{};
    float normal_impulse{};
    float total_normal_impulse{};
    float peak_normal_impulse{};
    float relative_velocity{};
    vm::Vec3 local_a{};
    vm::Vec3 local_b{};
    float cached_separation{};
    uint32_t feature_id{};
    bool persisted{};
};

struct Manifold {
    vm::Vec3 normal{};
    vm::Vec3 separating_axis{};
    vm::Vec3 local_normal{};
    std::array<ManifoldPoint, max_manifold_points> points{};
    uint32_t point_count{};
};

/// Accumulated tangent, twist and rolling friction impulses carried between
/// steps.
struct FrictionImpulses {
    float tangent_x{};
    float tangent_y{};
    float twist{};
    vm::Vec3 rolling{};
};

/// Soft constraint coefficients.
struct Softness {
    float bias_rate{};
    float mass_scale{1.0f};
    float impulse_scale{};
};

/// Soft constraint coefficients of a spring at `hertz` with `damping_ratio`
/// over substep `h`.
Softness make_softness(float hertz, float damping_ratio, float h);

/// Poses cached to recycle a contact while its bodies barely move.
struct ContactCache {
    vm::Quat rotation_a{};
    vm::Quat rotation_b{};
    vm::Transform relative_pose{};
    bool valid{};
};

struct Contact {
    ShapeRef shape_a{};
    ShapeRef shape_b{};
    int32_t proxy_a{-1};
    int32_t proxy_b{-1};
    Manifold manifold{};
    FrictionImpulses friction_impulses{};
    ContactCache cache{};
    float friction{};
    float restitution{};
    float rolling_resistance{};
    bool touching{};
    bool was_touching{};
    bool linked{};
    bool alive{};
    uint32_t set{null_link};
    uint32_t color{null_link};
    uint32_t local{null_link};
    uint32_t island{null_link};
    uint32_t island_local{null_link};
    std::array<uint32_t, 2> edge_local{null_link, null_link};
};

} // namespace bench::phys
