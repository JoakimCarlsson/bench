#include "broadphase.hpp"

#include <algorithm>

#include "box_collision.hpp"
#include "hash.hpp"

namespace bench {

namespace {

constexpr float speculative = 0.02f;
constexpr float dt = 1.0f / 60.0f;
constexpr float extent_xz = 45.0f;
constexpr float extent_y = 16.0f;

/// Bounds the broad phase tests against: the box grown by the speculative
/// distance.
vm::Aabb tight_aabb(const vm::BoxPose& pose) { return vm::grow(phys::box_aabb(pose), speculative); }

/// `value` wrapped into [0, extent).
float wrap(float value, float extent) {
    if (value < 0.0f) return value + extent;
    if (value >= extent) return value - extent;
    return value;
}

} // namespace

BroadPhaseCase::BroadPhaseCase() : initial_(bodies_n), body_proxies_(bodies_n) {
    Rng rng{0xb40ad};
    for (Body& b : initial_) {
        b.pose.half_extents = vm::random_vec3(rng, 0.25f, 0.75f);
        b.pose.center = {rng.unit() * extent_xz, rng.unit() * extent_y, rng.unit() * extent_xz};
        b.rotation = vm::random_quat(rng);
        b.pose.basis = vm::basis_from_quat(b.rotation);
        b.velocity = vm::random_vec3(rng, -1.0f, 1.0f);
        b.spin = vm::random_vec3(rng, -0.5f, 0.5f);
    }
    for (uint32_t z = 0; z < tiles_side; ++z) {
        for (uint32_t x = 0; x < tiles_side; ++x) {
            vm::BoxPose tile{};
            tile.half_extents = {1.0f, 0.5f, 1.0f};
            tile.center = {static_cast<float>(x) * 2.0f + 1.0f, -0.5f, static_cast<float>(z) * 2.0f + 1.0f};
            tiles_.push_back(tile);
        }
    }
}

void BroadPhaseCase::reset() {
    broad_phase_ = phys::BroadPhase{};
    pairs_.clear();
    free_pairs_.clear();
    pair_index_.clear();
    bodies_ = initial_;
    for (uint32_t i = 0; i < tiles_.size(); ++i) {
        static_cast<void>(broad_phase_.create_proxy({i, true}, tight_aabb(tiles_[i])));
    }
    for (uint32_t i = 0; i < bodies_n; ++i) {
        const vm::Aabb fat = vm::grow(tight_aabb(bodies_[i].pose), phys::shape_margin(bodies_[i].pose.half_extents));
        body_proxies_[i] = broad_phase_.create_proxy({i, false}, fat);
    }
}

void BroadPhaseCase::move_bodies() {
    for (uint32_t i = 0; i < bodies_n; ++i) {
        Body& b = bodies_[i];
        const vm::Vec3 c = b.pose.center + b.velocity * dt;
        b.pose.center = {wrap(c.x, extent_xz), wrap(c.y, extent_y), wrap(c.z, extent_xz)};
        b.rotation = vm::integrate_rotation(b.rotation, b.spin * dt);
        b.pose.basis = vm::basis_from_quat(b.rotation);
        const vm::Aabb tight = tight_aabb(b.pose);
        if (!vm::contains(broad_phase_.fat_aabb(body_proxies_[i]), tight)) {
            broad_phase_.move_proxy(body_proxies_[i], vm::grow(tight, phys::shape_margin(b.pose.half_extents)));
        }
    }
}

void BroadPhaseCase::update_pairs() {
    broad_phase_.update_pairs([&](int32_t a, int32_t b) {
        const uint64_t key = phys::pair_key(broad_phase_.shape(a), broad_phase_.shape(b));
        if (pair_index_.contains(key)) return;
        uint32_t id{};
        if (free_pairs_.empty()) {
            id = static_cast<uint32_t>(pairs_.size());
            pairs_.emplace_back();
        } else {
            id = free_pairs_.back();
            free_pairs_.pop_back();
        }
        pairs_[id] = Pair{a, b, key, true};
        pair_index_.emplace(key, id);
    });
}

void BroadPhaseCase::drop_parted_pairs() {
    for (uint32_t id = 0; id < pairs_.size(); ++id) {
        Pair& pair = pairs_[id];
        if (!pair.alive) continue;
        if (vm::overlaps(broad_phase_.fat_aabb(pair.proxy_a), broad_phase_.fat_aabb(pair.proxy_b))) continue;
        pair_index_.erase(pair.key);
        pair.alive = false;
        free_pairs_.push_back(id);
    }
}

uint64_t BroadPhaseCase::run() {
    reset();
    uint64_t h = 0;
    for (int frame = 0; frame < frames; ++frame) {
        move_bodies();
        update_pairs();
        drop_parted_pairs();
        h = hash_add(h, pair_index_.size());
    }
    for (const Pair& pair : pairs_) {
        h = hash_add(h, pair.alive ? pair.key : 0u);
    }
    for (uint32_t i = 0; i < bodies_n; ++i) {
        const vm::Aabb& fat = broad_phase_.fat_aabb(body_proxies_[i]);
        h = hash_add(h, static_cast<uint64_t>(f32_bits(fat.min.x)) | (static_cast<uint64_t>(f32_bits(fat.max.y)) << 32));
    }
    return h;
}

} // namespace bench
