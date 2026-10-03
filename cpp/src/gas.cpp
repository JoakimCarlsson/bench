#include "gas.hpp"

#include <algorithm>
#include <cmath>

#include "hash.hpp"

namespace bench {

namespace {

constexpr uint32_t golden = 0x9E3779B9u;
constexpr int64_t gas_key_offset = int64_t{1} << 20;
constexpr uint64_t gas_key_mask = (uint64_t{1} << 21u) - 1u;
constexpr float mover_reach = 1.5f;
constexpr float mover_wake_rate = 6.0f;
constexpr float spacing = 0.8f;
constexpr float pressure = 1.5f;
constexpr float viscosity = 0.5f;
constexpr float stir_strength = 1.0f;
constexpr float step = 1.0f / 120.0f;
constexpr float rise = 0.5f;
constexpr float extent = 12.0f;

/// The engine's 32-bit bit mixer.
constexpr uint32_t mix_bits(uint32_t value) {
    value ^= value >> 16u;
    value *= 0x7FEB352Du;
    value ^= value >> 15u;
    value *= 0x846CA68Bu;
    value ^= value >> 16u;
    return value;
}

/// A deterministic value in [0, 1) from `seed` and `salt`.
constexpr float unit_random(uint32_t seed, uint32_t salt) {
    return static_cast<float>(mix_bits(seed ^ mix_bits(salt + golden)) >> 8u) / 16777216.0f;
}

/// Unit vector along `value`, or `fallback` when it is too short.
vm::Vec3 safe_normalize(vm::Vec3 value, vm::Vec3 fallback) {
    const float size = vm::length(value);
    return size > 1e-6f ? value / size : fallback;
}

/// Packs three cell coordinates into one 63-bit key.
uint64_t gas_key(int64_t x, int64_t y, int64_t z) {
    const auto pack = [](int64_t value) { return static_cast<uint64_t>(value + gas_key_offset) & gas_key_mask; };
    return pack(x) | (pack(y) << 21u) | (pack(z) << 42u);
}

/// Integer cell containing `position` for cell size `reach`.
std::array<int64_t, 3> gas_cell(vm::Vec3 position, float reach) {
    return {static_cast<int64_t>(std::floor(position.x / reach)), static_cast<int64_t>(std::floor(position.y / reach)),
            static_cast<int64_t>(std::floor(position.z / reach))};
}

/// Unit direction pushing `self` away from `other`; a seeded random one when
/// they coincide.
vm::Vec3 separation(const Gas::Particle& self, const Gas::Particle& other, vm::Vec3 offset) {
    const float size = vm::length(offset);
    if (size > 1e-6f) return offset / size;
    const uint32_t mixed = mix_bits(self.seed ^ mix_bits(other.seed));
    return safe_normalize(vm::Vec3{unit_random(mixed, 1u) - 0.5f, unit_random(mixed, 2u) - 0.5f, unit_random(mixed, 3u) - 0.5f},
                          vm::Vec3{0.0f, 1.0f, 0.0f});
}

/// Pushes a particle with a moving box, adding surface velocity and moving it
/// out when it penetrates.
void stir(Gas::Particle& body, const Gas::Mover& mover) {
    const vm::Basis rotation{safe_normalize(mover.transform.basis.x, vm::Vec3{1.0f, 0.0f, 0.0f}),
                             safe_normalize(mover.transform.basis.y, vm::Vec3{0.0f, 1.0f, 0.0f}),
                             safe_normalize(mover.transform.basis.z, vm::Vec3{0.0f, 0.0f, 1.0f})};
    const vm::Vec3 local = vm::transposed(rotation) * (body.position - mover.transform.origin);
    const vm::Vec3 extent = mover.half_extents;
    const vm::Vec3 closest{std::clamp(local.x, -extent.x, extent.x), std::clamp(local.y, -extent.y, extent.y),
                           std::clamp(local.z, -extent.z, extent.z)};
    const float radius = body.size * 0.5f;
    const float reach = std::max({extent.x, extent.y, extent.z}) * mover_reach + radius;
    const vm::Vec3 outside = local - closest;
    const float distance = vm::length(outside);
    if (distance > reach) return;
    const vm::Vec3 arm = rotation * closest;
    const vm::Vec3 surface_velocity = mover.velocity + vm::cross(mover.angular_velocity, arm);
    const float weight = 1.0f - (distance / reach);
    const float wake = std::clamp(stir_strength * weight * step * mover_wake_rate, 0.0f, 1.0f);
    body.velocity = body.velocity + (surface_velocity - body.velocity) * wake;
    if (distance >= radius) return;
    vm::Vec3 normal{};
    if (distance > 1e-6f) {
        normal = outside / distance;
    } else {
        const vm::Vec3 depth = extent - vm::abs(local);
        if (depth.x <= depth.y && depth.x <= depth.z) {
            normal = vm::Vec3{local.x < 0.0f ? -1.0f : 1.0f, 0.0f, 0.0f};
        } else if (depth.y <= depth.z) {
            normal = vm::Vec3{0.0f, local.y < 0.0f ? -1.0f : 1.0f, 0.0f};
        } else {
            normal = vm::Vec3{0.0f, 0.0f, local.z < 0.0f ? -1.0f : 1.0f};
        }
    }
    const vm::Vec3 face = distance > 1e-6f ? closest
                                           : vm::Vec3{normal.x != 0.0f ? normal.x * extent.x : local.x,
                                                      normal.y != 0.0f ? normal.y * extent.y : local.y,
                                                      normal.z != 0.0f ? normal.z * extent.z : local.z};
    const vm::Vec3 world_normal = rotation * normal;
    body.position = mover.transform.origin + (rotation * face) + (world_normal * radius);
    const float into = vm::dot(body.velocity - surface_velocity, world_normal);
    if (into < 0.0f) body.velocity = body.velocity - (world_normal * (into * stir_strength));
}

} // namespace

Gas::Gas() : initial_(particles_n), particles_(particles_n), push_(particles_n), blend_(particles_n), weight_(particles_n) {
    Rng rng{0x6a5};
    for (uint32_t i = 0; i < particles_n; ++i) {
        Particle& p = initial_[i];
        p.position = vm::random_vec3(rng, 0.0f, extent);
        p.velocity = vm::random_vec3(rng, -0.5f, 0.5f);
        p.size = 0.3f + rng.unit() * 0.3f;
        p.seed = static_cast<uint32_t>(rng.next());
    }
    for (Mover& m : movers_) {
        m.transform.basis = vm::basis_from_quat(vm::random_quat(rng));
        m.transform.origin = vm::random_vec3(rng, 4.0f, extent - 4.0f);
        m.half_extents = vm::random_vec3(rng, 1.0f, 2.0f);
        m.velocity = vm::random_vec3(rng, -3.0f, 3.0f);
        m.angular_velocity = vm::random_vec3(rng, -1.0f, 1.0f);
    }
    entries_.reserve(particles_n);
}

void Gas::accumulate_pressure() {
    float reach = 0.0f;
    for (const Particle& body : particles_) reach = std::max(reach, body.size * spacing);
    if (reach <= 0.0f) return;
    entries_.clear();
    for (uint32_t index = 0; index < particles_n; ++index) {
        const auto cell = gas_cell(particles_[index].position, reach);
        entries_.push_back(Entry{gas_key(cell[0], cell[1], cell[2]), index});
    }
    std::ranges::sort(entries_, [](const Entry& a, const Entry& b) { return a.key < b.key || (a.key == b.key && a.index < b.index); });
    for (const Entry& entry : entries_) {
        const Particle& self = particles_[entry.index];
        const auto cell = gas_cell(self.position, reach);
        for (int64_t dz = -1; dz <= 1; ++dz) {
            for (int64_t dy = -1; dy <= 1; ++dy) {
                for (int64_t dx = -1; dx <= 1; ++dx) {
                    const uint64_t key = gas_key(cell[0] + dx, cell[1] + dy, cell[2] + dz);
                    auto next = std::ranges::lower_bound(entries_, key, {}, &Entry::key);
                    for (; next != entries_.end() && next->key == key; ++next) {
                        if (next->index == entry.index) continue;
                        const Particle& other = particles_[next->index];
                        const vm::Vec3 offset = self.position - other.position;
                        const float range = 0.5f * (self.size + other.size) * spacing;
                        const float distance = vm::length(offset);
                        if (range <= 0.0f || distance >= range) continue;
                        const float overlap = 1.0f - (distance / range);
                        push_[entry.index] = push_[entry.index] + (separation(self, other, offset) * overlap);
                        blend_[entry.index] = blend_[entry.index] + ((other.velocity - self.velocity) * overlap);
                        weight_[entry.index] += overlap;
                    }
                }
            }
        }
    }
}

void Gas::resolve() {
    std::fill(push_.begin(), push_.end(), vm::Vec3{});
    std::fill(blend_.begin(), blend_.end(), vm::Vec3{});
    std::fill(weight_.begin(), weight_.end(), 0.0f);
    accumulate_pressure();
    for (uint32_t index = 0; index < particles_n; ++index) {
        Particle& body = particles_[index];
        body.velocity = body.velocity + (push_[index] * (pressure * step));
        if (weight_[index] > 0.0f) {
            const float blend = std::clamp(viscosity * step * 10.0f, 0.0f, 1.0f);
            body.velocity = body.velocity + (blend_[index] * (blend / weight_[index]));
        }
        for (const Mover& mover : movers_) stir(body, mover);
    }
}

uint64_t Gas::run() {
    std::copy(initial_.begin(), initial_.end(), particles_.begin());
    for (int s = 0; s < substeps; ++s) {
        resolve();
        for (Particle& p : particles_) {
            p.velocity.y = p.velocity.y + rise * step;
            p.position = p.position + p.velocity * step;
        }
    }
    uint64_t h = 0;
    for (const Particle& p : particles_) {
        h = hash_add(h, static_cast<uint64_t>(f32_bits(p.position.x)) | (static_cast<uint64_t>(f32_bits(p.position.y)) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(p.velocity.z)) | (static_cast<uint64_t>(f32_bits(p.position.z)) << 32));
    }
    return h;
}

} // namespace bench
