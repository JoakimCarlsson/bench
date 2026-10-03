#pragma once

#include <cstdint>
#include <unordered_map>
#include <vector>

#include "broad_phase.hpp"
#include "vecmath.hpp"

namespace bench {

/// Broad phase: 16 frames of 4096 tumbling boxes over a field of 1024
/// static tiles, through the engine's dynamic AABB tree. Each frame refits
/// fat bounds that no longer hold, reinserts those leaves with rotations,
/// finds the new pairs of every moved proxy, records them in a hash map
/// keyed by shape pair, and drops pairs whose fat bounds parted.
class BroadPhaseCase {
public:
    static constexpr const char* name = "broadphase";

    BroadPhaseCase();
    uint64_t run();

private:
    static constexpr uint32_t bodies_n = 4096;
    static constexpr uint32_t tiles_side = 32;
    static constexpr int frames = 16;

    struct Body {
        vm::BoxPose pose{};
        vm::Quat rotation{};
        vm::Vec3 velocity{};
        vm::Vec3 spin{};
    };

    struct Pair {
        int32_t proxy_a{};
        int32_t proxy_b{};
        uint64_t key{};
        bool alive{};
    };

    /// Creates every proxy from the initial poses.
    void reset();
    /// Advances the bodies and moves the proxies whose fat bounds they left.
    void move_bodies();
    /// Records the new pairs of the moved proxies.
    void update_pairs();
    /// Drops pairs whose fat bounds no longer overlap.
    void drop_parted_pairs();

    std::vector<Body> initial_;
    std::vector<Body> bodies_;
    std::vector<vm::BoxPose> tiles_;
    std::vector<int32_t> body_proxies_;
    phys::BroadPhase broad_phase_;
    std::vector<Pair> pairs_;
    std::vector<uint32_t> free_pairs_;
    std::unordered_map<uint64_t, uint32_t> pair_index_;
};

} // namespace bench
