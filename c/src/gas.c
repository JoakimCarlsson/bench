#include "gas.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "hash.h"
#include "vecmath.h"

enum { PARTICLES = 8192, MOVERS = 4, SUBSTEPS = 4 };

static const uint32_t golden = 0x9E3779B9u;
static const int64_t gas_key_offset = (int64_t)1 << 20;
static const uint64_t gas_key_mask = ((uint64_t)1 << 21u) - 1u;
static const float mover_reach = 1.5f;
static const float mover_wake_rate = 6.0f;
static const float spacing = 0.8f;
static const float pressure = 1.5f;
static const float viscosity = 0.5f;
static const float stir_strength = 1.0f;
static const float step = 1.0f / 120.0f;
static const float rise = 0.5f;
static const float extent = 12.0f;

typedef struct {
    Vec3 position;
    Vec3 velocity;
    float size;
    uint32_t seed;
} Particle;

typedef struct {
    Transform transform;
    Vec3 half_extents;
    Vec3 velocity;
    Vec3 angular_velocity;
} Mover;

/// A particle index keyed by its spatial hash cell.
typedef struct {
    uint64_t key;
    uint32_t index;
} Entry;

/// Integer cell coordinates.
typedef struct {
    int64_t x, y, z;
} Cell;

typedef struct {
    Particle* initial;
    Particle* particles;
    Mover movers[MOVERS];
    Entry* entries;
    size_t entry_count;
    Vec3* push;
    Vec3* blend;
    float* weight;
} Gas;

/// The engine's 32-bit bit mixer.
static uint32_t mix_bits(uint32_t value) {
    value ^= value >> 16u;
    value *= 0x7FEB352Du;
    value ^= value >> 15u;
    value *= 0x846CA68Bu;
    value ^= value >> 16u;
    return value;
}

/// A deterministic value in [0, 1) from `seed` and `salt`.
static float unit_random(uint32_t seed, uint32_t salt) {
    return (float)(mix_bits(seed ^ mix_bits(salt + golden)) >> 8u) / 16777216.0f;
}

/// Unit vector along `value`, or `fallback` when it is too short.
static Vec3 safe_normalize(Vec3 value, Vec3 fallback) {
    float size = v3_length(value);
    return size > 1e-6f ? v3_div(value, size) : fallback;
}

/// One cell coordinate offset and masked to 21 bits.
static uint64_t pack_coordinate(int64_t value) { return (uint64_t)(value + gas_key_offset) & gas_key_mask; }

/// Packs three cell coordinates into one 63-bit key.
static uint64_t gas_key(int64_t x, int64_t y, int64_t z) {
    return pack_coordinate(x) | (pack_coordinate(y) << 21u) | (pack_coordinate(z) << 42u);
}

/// Integer cell containing `position` for cell size `reach`.
static Cell gas_cell(Vec3 position, float reach) {
    return (Cell){ (int64_t)floorf(position.x / reach), (int64_t)floorf(position.y / reach), (int64_t)floorf(position.z / reach) };
}

/// Orders entries by key, then by particle index.
static int compare_entries(const void* a, const void* b) {
    const Entry* x = a;
    const Entry* y = b;
    if (x->key != y->key) return x->key < y->key ? -1 : 1;
    return x->index < y->index ? -1 : x->index > y->index;
}

/// First entry whose key is not less than `key`.
static size_t lower_bound(const Entry* entries, size_t count, uint64_t key) {
    size_t first = 0;
    while (count > 0) {
        size_t half = count / 2;
        if (entries[first + half].key < key) {
            first += half + 1;
            count -= half + 1;
        } else {
            count = half;
        }
    }
    return first;
}

/// Unit direction pushing `self` away from `other`; a seeded random one when
/// they coincide.
static Vec3 separation(const Particle* self, const Particle* other, Vec3 offset) {
    float size = v3_length(offset);
    if (size > 1e-6f) return v3_div(offset, size);
    uint32_t mixed = mix_bits(self->seed ^ mix_bits(other->seed));
    Vec3 random = { unit_random(mixed, 1u) - 0.5f, unit_random(mixed, 2u) - 0.5f, unit_random(mixed, 3u) - 0.5f };
    return safe_normalize(random, (Vec3){ 0.0f, 1.0f, 0.0f });
}

/// Outward normal of the face nearest to a point inside the box.
static Vec3 nearest_face_normal(Vec3 local, Vec3 half_extents) {
    Vec3 depth = v3_sub(half_extents, v3_abs(local));
    if (depth.x <= depth.y && depth.x <= depth.z) return (Vec3){ local.x < 0.0f ? -1.0f : 1.0f, 0.0f, 0.0f };
    if (depth.y <= depth.z) return (Vec3){ 0.0f, local.y < 0.0f ? -1.0f : 1.0f, 0.0f };
    return (Vec3){ 0.0f, 0.0f, local.z < 0.0f ? -1.0f : 1.0f };
}

/// Pushes a particle with a moving box, adding surface velocity and moving it
/// out when it penetrates.
static void stir(Particle* body, const Mover* mover) {
    Basis rotation = { safe_normalize(mover->transform.basis.x, (Vec3){ 1.0f, 0.0f, 0.0f }),
                       safe_normalize(mover->transform.basis.y, (Vec3){ 0.0f, 1.0f, 0.0f }),
                       safe_normalize(mover->transform.basis.z, (Vec3){ 0.0f, 0.0f, 1.0f }) };
    Basis inverse = basis_transposed(&rotation);
    Vec3 local = basis_apply(&inverse, v3_sub(body->position, mover->transform.origin));
    Vec3 half = mover->half_extents;
    Vec3 closest = { f32_clamp(local.x, -half.x, half.x), f32_clamp(local.y, -half.y, half.y),
                     f32_clamp(local.z, -half.z, half.z) };
    float radius = body->size * 0.5f;
    float reach = f32_max3(half.x, half.y, half.z) * mover_reach + radius;
    Vec3 outside = v3_sub(local, closest);
    float distance = v3_length(outside);
    if (distance > reach) return;
    Vec3 arm = basis_apply(&rotation, closest);
    Vec3 surface_velocity = v3_add(mover->velocity, v3_cross(mover->angular_velocity, arm));
    float weight = 1.0f - (distance / reach);
    float wake = f32_clamp(stir_strength * weight * step * mover_wake_rate, 0.0f, 1.0f);
    body->velocity = v3_add(body->velocity, v3_scale(v3_sub(surface_velocity, body->velocity), wake));
    if (distance >= radius) return;
    Vec3 normal = distance > 1e-6f ? v3_div(outside, distance) : nearest_face_normal(local, half);
    Vec3 face = distance > 1e-6f ? closest
                                 : (Vec3){ normal.x != 0.0f ? normal.x * half.x : local.x,
                                           normal.y != 0.0f ? normal.y * half.y : local.y,
                                           normal.z != 0.0f ? normal.z * half.z : local.z };
    Vec3 world_normal = basis_apply(&rotation, normal);
    body->position = v3_add(v3_add(mover->transform.origin, basis_apply(&rotation, face)), v3_scale(world_normal, radius));
    float into = v3_dot(v3_sub(body->velocity, surface_velocity), world_normal);
    if (into < 0.0f) body->velocity = v3_sub(body->velocity, v3_scale(world_normal, into * stir_strength));
}

/// Builds the particles and movers.
static void setup(void* state) {
    Gas* s = state;
    s->initial = xalloc(PARTICLES * sizeof(Particle));
    s->particles = xalloc(PARTICLES * sizeof(Particle));
    s->entries = xalloc(PARTICLES * sizeof(Entry));
    s->push = xalloc(PARTICLES * sizeof(Vec3));
    s->blend = xalloc(PARTICLES * sizeof(Vec3));
    s->weight = xalloc(PARTICLES * sizeof(float));
    Rng rng = { 0x6a5 };
    for (uint32_t i = 0; i < PARTICLES; ++i) {
        Particle* p = &s->initial[i];
        p->position = v3_random(&rng, 0.0f, extent);
        p->velocity = v3_random(&rng, -0.5f, 0.5f);
        p->size = 0.3f + rng_unit(&rng) * 0.3f;
        p->seed = (uint32_t)rng_next(&rng);
    }
    for (uint32_t i = 0; i < MOVERS; ++i) {
        Mover* m = &s->movers[i];
        m->transform.basis = basis_from_quat(quat_random(&rng));
        m->transform.origin = v3_random(&rng, 4.0f, extent - 4.0f);
        m->half_extents = v3_random(&rng, 1.0f, 2.0f);
        m->velocity = v3_random(&rng, -3.0f, 3.0f);
        m->angular_velocity = v3_random(&rng, -1.0f, 1.0f);
    }
}

/// Adds the push, velocity blend and weight of the particles in the cell
/// with `key` that overlap `entry`'s particle.
static void accumulate_cell(Gas* s, const Entry* entry, uint64_t key) {
    const Particle* self = &s->particles[entry->index];
    for (size_t next = lower_bound(s->entries, s->entry_count, key); next < s->entry_count && s->entries[next].key == key; ++next) {
        if (s->entries[next].index == entry->index) continue;
        const Particle* other = &s->particles[s->entries[next].index];
        Vec3 offset = v3_sub(self->position, other->position);
        float range = 0.5f * (self->size + other->size) * spacing;
        float distance = v3_length(offset);
        if (range <= 0.0f || distance >= range) continue;
        float overlap = 1.0f - (distance / range);
        s->push[entry->index] = v3_add(s->push[entry->index], v3_scale(separation(self, other, offset), overlap));
        s->blend[entry->index] = v3_add(s->blend[entry->index], v3_scale(v3_sub(other->velocity, self->velocity), overlap));
        s->weight[entry->index] += overlap;
    }
}

/// Pairwise push, velocity blend and weight from overlapping particles.
static void accumulate_pressure(Gas* s) {
    float reach = 0.0f;
    for (uint32_t i = 0; i < PARTICLES; ++i) reach = f32_max(reach, s->particles[i].size * spacing);
    if (reach <= 0.0f) return;
    s->entry_count = 0;
    for (uint32_t index = 0; index < PARTICLES; ++index) {
        Cell cell = gas_cell(s->particles[index].position, reach);
        s->entries[s->entry_count++] = (Entry){ gas_key(cell.x, cell.y, cell.z), index };
    }
    qsort(s->entries, s->entry_count, sizeof(Entry), compare_entries);
    for (size_t e = 0; e < s->entry_count; ++e) {
        const Entry* entry = &s->entries[e];
        Cell cell = gas_cell(s->particles[entry->index].position, reach);
        for (int64_t dz = -1; dz <= 1; ++dz) {
            for (int64_t dy = -1; dy <= 1; ++dy) {
                for (int64_t dx = -1; dx <= 1; ++dx) accumulate_cell(s, entry, gas_key(cell.x + dx, cell.y + dy, cell.z + dz));
            }
        }
    }
}

/// Pressure, viscosity and stirring for every particle.
static void resolve(Gas* s) {
    memset(s->push, 0, PARTICLES * sizeof(Vec3));
    memset(s->blend, 0, PARTICLES * sizeof(Vec3));
    memset(s->weight, 0, PARTICLES * sizeof(float));
    accumulate_pressure(s);
    for (uint32_t index = 0; index < PARTICLES; ++index) {
        Particle* body = &s->particles[index];
        body->velocity = v3_add(body->velocity, v3_scale(s->push[index], pressure * step));
        if (s->weight[index] > 0.0f) {
            float blend = f32_clamp(viscosity * step * 10.0f, 0.0f, 1.0f);
            body->velocity = v3_add(body->velocity, v3_scale(s->blend[index], blend / s->weight[index]));
        }
        for (uint32_t m = 0; m < MOVERS; ++m) stir(body, &s->movers[m]);
    }
}

/// Runs every substep and hashes the particles.
static uint64_t run(void* state) {
    Gas* s = state;
    memcpy(s->particles, s->initial, PARTICLES * sizeof(Particle));
    for (int sub = 0; sub < SUBSTEPS; ++sub) {
        resolve(s);
        for (uint32_t i = 0; i < PARTICLES; ++i) {
            Particle* p = &s->particles[i];
            p->velocity.y = p->velocity.y + rise * step;
            p->position = v3_add(p->position, v3_scale(p->velocity, step));
        }
    }
    uint64_t h = 0;
    for (uint32_t i = 0; i < PARTICLES; ++i) {
        const Particle* p = &s->particles[i];
        h = hash_add(h, (uint64_t)f32_bits(p->position.x) | ((uint64_t)f32_bits(p->position.y) << 32));
        h = hash_add(h, (uint64_t)f32_bits(p->velocity.z) | ((uint64_t)f32_bits(p->position.z) << 32));
    }
    return h;
}

/// Frees the particles and scratch buffers.
static void teardown(void* state) {
    Gas* s = state;
    free(s->initial);
    free(s->particles);
    free(s->entries);
    free(s->push);
    free(s->blend);
    free(s->weight);
}

const Case gas_case = { "gas", sizeof(Gas), setup, run, teardown };
