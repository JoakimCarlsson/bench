#include "particles.h"

#include <math.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "alloc.h"
#include "hash.h"
#include "vecmath.h"

enum {
    EMITTERS = 12,
    POOL = 2048,
    SUB_POOL = 256,
    UPDATES = 4,
    SUBSTEPS = 6,
    PREPROCESS_STEPS = 45,
    CURVE_CAPACITY = 8,
    GRADIENT_CAPACITY = 6,
    EFFECTS = 6,
    BOXES = 7
};

static const float preprocess_seconds = 1.5f;
static const float update_seconds = 0.016666668f;
static const float two_pi = 6.2831855f;
static const float degrees_to_radians = 0.017453292f;
static const float minimum_lifetime = 1e-4f;
static const float contact_offset = 1e-3f;
static const float impact_speed = 0.05f;
static const float turbulence_reference_rate = 60.0f;
static const float wind_response = 2.0f;
static const float one_third = 0.33333334f;
static const uint32_t golden = 0x9E3779B9u;
static const uint32_t no_effect = 0xFFFFFFFFu;
static const Vec3 world_up = { 0.0f, 1.0f, 0.0f };

enum {
    SALT_LIFETIME = 1,
    SALT_RATIO,
    SALT_INITIAL_RAMP,
    SALT_PHASE,
    SALT_SHAPE_A,
    SALT_SHAPE_B,
    SALT_SHAPE_C,
    SALT_DIRECTION_A,
    SALT_DIRECTION_B,
    SALT_TURBULENCE_SPEED,
    SALT_PARAMETER_BASE = 32
};

typedef enum {
    INITIAL_VELOCITY,
    ANGULAR_VELOCITY,
    ORBIT_VELOCITY,
    RADIAL_VELOCITY,
    LINEAR_ACCELERATION,
    RADIAL_ACCELERATION,
    TANGENTIAL_ACCELERATION,
    DAMPING,
    ANGLE,
    SCALE,
    HUE_VARIATION,
    TURBULENCE_INFLUENCE,
    PARAM_COUNT
} Param;

typedef enum { COLLISION_DISABLED, COLLISION_RIGID, COLLISION_HIDE_ON_CONTACT } CollisionMode;

typedef enum { SUB_DISABLED, SUB_CONSTANT, SUB_AT_END, SUB_AT_COLLISION } SubMode;

typedef struct {
    float r, g, b, a;
} Rgba;

typedef struct {
    float offset;
    float value;
} CurvePoint;

typedef struct {
    bool smooth;
    uint32_t count;
    CurvePoint points[CURVE_CAPACITY];
} Curve;

typedef struct {
    float offset;
    Rgba value;
} Stop;

typedef struct {
    uint32_t count;
    Stop stops[GRADIENT_CAPACITY];
} Gradient;

typedef struct {
    float minimum;
    float maximum;
    Curve curve;
} Parameter;

typedef struct {
    uint32_t amount;
    float lifetime;
    bool one_shot;
    float explosiveness;
    float randomness;
    float lifetime_randomness;
    bool box_shape;
    Vec3 offset;
    Vec3 box_extents;
    Vec3 direction;
    float spread_degrees;
    float flatness;
    float inherit_velocity;
    Vec3 gravity;
    float wind_influence;
    Parameter parameters[PARAM_COUNT];
    Rgba color;
    Gradient color_ramp;
    Gradient color_initial_ramp;
    Curve alpha_curve;
    bool turbulence_enabled;
    float turbulence_strength;
    float turbulence_scale;
    Vec3 turbulence_speed;
    float turbulence_speed_random;
    CollisionMode collision;
    float friction;
    float bounce;
    bool use_scale;
    SubMode sub_mode;
    float sub_frequency;
    uint32_t sub_amount;
    bool sub_keep_velocity;
    uint32_t sub_effect;
    float draw_size;
} Effect;

typedef struct {
    Vec3 position;
    Vec3 velocity;
    Vec3 origin;
    Vec3 axis;
    Rgba color;
    Rgba tint;
    float angle;
    float spin;
    float size;
    float age;
    float lifetime;
    float sub_timer;
    uint32_t seed;
    bool active;
} Particle;

typedef struct {
    uint32_t effect;
    uint32_t sub_effect;
    uint32_t first;
    uint32_t sub_first;
    uint32_t sub_count;
    uint32_t emit_cursor;
    uint32_t sub_cursor;
    uint32_t bursts;
    uint32_t current_seed;
    uint32_t burst_pending;
    uint32_t alive;
    uint64_t cycle;
    float phase;
    float elapsed;
    bool emitting;
    float amount_ratio;
    float speed_scale;
    Rgba tint;
    Basis basis;
    Vec3 origin;
    Vec3 move;
    Vec3 velocity;
} Emitter;

typedef struct {
    Effect effects[EFFECTS];
    Aabb boxes[BOXES];
    Vec3 wind;
} Scene;

typedef struct {
    Particle* main;
    Particle* sub;
} Pools;

typedef struct {
    const Effect* effect;
    Basis basis;
    Vec3 origin;
    Vec3 emitter_velocity;
    float time;
    float amount_ratio;
    Vec3 wind;
    Rgba tint;
} Context;

typedef struct {
    Vec3 position;
    Vec3 normal;
} Contact;

typedef struct {
    bool died;
    bool collided;
    Vec3 where;
    Vec3 velocity;
} Outcome;

typedef struct {
    Vec3 position;
    Vec3 velocity;
} SubOrigin;

typedef struct {
    float enter;
    float exit;
    int axis;
} Clip;

typedef struct {
    Scene scene;
    uint64_t collisions;
    Emitter emitters[EMITTERS];
    Emitter initial_emitters[EMITTERS];
    Particle* particles;
    Particle* initial_particles;
    Particle* sub_particles;
    Particle* initial_sub_particles;
} Particles;

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

/// Linear interpolation from `a` to `b` by `t`.
static float mix_float(float a, float b, float t) { return a + ((b - a) * t); }

/// Linear interpolation between two vectors.
static Vec3 v3_lerp(Vec3 a, Vec3 b, float t) { return v3_add(a, v3_scale(v3_sub(b, a), t)); }

/// Degrees in radians.
static float radians(float degrees) { return degrees * degrees_to_radians; }

/// Taylor polynomial of the sine, accurate for small angles.
static float sin_poly(float x) { return x - ((x * x * x) / 6.0f) + ((x * x * x * x * x) / 120.0f); }

/// Taylor polynomial of the cosine, accurate for small angles.
static float cos_poly(float x) { return 1.0f - ((x * x) / 2.0f) + ((x * x * x * x) / 24.0f); }

/// Unit vector along `value`, or `fallback` when it is too short.
static Vec3 safe_normalize(Vec3 value, Vec3 fallback) {
    float size = v3_length(value);
    return size > 1e-6f ? v3_div(value, size) : fallback;
}

/// The component of `value` along axis 0, 1 or 2.
static float component(Vec3 value, int axis) { return axis == 0 ? value.x : (axis == 1 ? value.y : value.z); }

/// Smoothstep of a blend factor.
static float smooth_blend(float blend) { return blend * blend * (3.0f - (2.0f * blend)); }

/// An opaque white colour.
static Rgba rgba_white(void) { return (Rgba){ 1.0f, 1.0f, 1.0f, 1.0f }; }

/// A particle in its initial, inactive state.
static Particle particle_blank(void) {
    Particle p;
    memset(&p, 0, sizeof p);
    p.axis = world_up;
    p.color = rgba_white();
    p.tint = rgba_white();
    return p;
}

/// An effect with the engine's defaults and unit initial velocity and scale.
static Effect effect_make(void) {
    Effect e;
    memset(&e, 0, sizeof e);
    e.amount = 16;
    e.lifetime = 1.0f;
    e.box_extents = (Vec3){ 1.0f, 1.0f, 1.0f };
    e.direction = (Vec3){ 0.0f, 1.0f, 0.0f };
    e.spread_degrees = 45.0f;
    e.gravity = (Vec3){ 0.0f, -9.8f, 0.0f };
    e.parameters[INITIAL_VELOCITY].minimum = 1.0f;
    e.parameters[INITIAL_VELOCITY].maximum = 1.0f;
    e.parameters[SCALE].minimum = 1.0f;
    e.parameters[SCALE].maximum = 1.0f;
    e.color = rgba_white();
    e.turbulence_strength = 1.0f;
    e.turbulence_scale = 4.0f;
    e.turbulence_speed = (Vec3){ 0.0f, 0.5f, 0.0f };
    e.turbulence_speed_random = 0.2f;
    e.sub_frequency = 4.0f;
    e.sub_amount = 1;
    e.sub_effect = no_effect;
    e.draw_size = 0.25f;
    return e;
}

/// An emitter with the engine's defaults.
static Emitter emitter_make(void) {
    Emitter e;
    memset(&e, 0, sizeof e);
    e.sub_effect = no_effect;
    e.emitting = true;
    e.amount_ratio = 1.0f;
    e.speed_scale = 1.0f;
    e.basis = basis_identity();
    return e;
}

/// A step context for `effect` with the engine's defaults.
static Context context_make(const Effect* effect) {
    Context c;
    memset(&c, 0, sizeof c);
    c.effect = effect;
    c.basis = basis_identity();
    c.amount_ratio = 1.0f;
    c.tint = rgba_white();
    return c;
}

/// Samples a curve, clamping outside its range; 1 for an empty curve.
static float sample_curve(const Curve* curve, float offset) {
    if (curve->count == 0) return 1.0f;
    if (offset <= curve->points[0].offset) return curve->points[0].value;
    if (offset >= curve->points[curve->count - 1].offset) return curve->points[curve->count - 1].value;
    uint32_t upper = 1;
    while (upper < curve->count && curve->points[upper].offset < offset) ++upper;
    const CurvePoint* from = &curve->points[upper - 1];
    const CurvePoint* to = &curve->points[upper];
    float span = to->offset - from->offset;
    float blend = span > 0.0f ? (offset - from->offset) / span : 1.0f;
    if (curve->smooth) blend = smooth_blend(blend);
    return from->value + ((to->value - from->value) * blend);
}

/// Samples a gradient, clamping outside its range; white for an empty one.
static Rgba sample_gradient(const Gradient* gradient, float offset) {
    if (gradient->count == 0) return rgba_white();
    if (offset <= gradient->stops[0].offset) return gradient->stops[0].value;
    if (offset >= gradient->stops[gradient->count - 1].offset) return gradient->stops[gradient->count - 1].value;
    uint32_t upper = 1;
    while (upper < gradient->count && gradient->stops[upper].offset < offset) ++upper;
    const Stop* from = &gradient->stops[upper - 1];
    const Stop* to = &gradient->stops[upper];
    float span = to->offset - from->offset;
    float blend = span > 0.0f ? (offset - from->offset) / span : 1.0f;
    return (Rgba){ mix_float(from->value.r, to->value.r, blend), mix_float(from->value.g, to->value.g, blend),
                   mix_float(from->value.b, to->value.b, blend), mix_float(from->value.a, to->value.a, blend) };
}

/// The seeded random value between a parameter's minimum and maximum.
static float parameter_random(const Parameter* parameter, uint32_t seed, Param id) {
    return mix_float(parameter->minimum, parameter->maximum, unit_random(seed, SALT_PARAMETER_BASE + (uint32_t)id));
}

/// A parameter at life `offset`: its seeded value scaled by its curve.
static float parameter_value(const Effect* effect, Param id, uint32_t seed, float offset) {
    const Parameter* parameter = &effect->parameters[id];
    return parameter_random(parameter, seed, id) * sample_curve(&parameter->curve, offset);
}

/// A seeded point in the emission shape, in emitter space.
static Vec3 shape_point(const Effect* effect, uint32_t seed) {
    float a = unit_random(seed, SALT_SHAPE_A);
    float b = unit_random(seed, SALT_SHAPE_B);
    float c = unit_random(seed, SALT_SHAPE_C);
    if (!effect->box_shape) return (Vec3){ 0.0f, 0.0f, 0.0f };
    return (Vec3){ ((a * 2.0f) - 1.0f) * effect->box_extents.x, ((b * 2.0f) - 1.0f) * effect->box_extents.y,
                   ((c * 2.0f) - 1.0f) * effect->box_extents.z };
}

/// A seeded unit launch direction within the effect's spread cone.
static Vec3 spread_direction(const Effect* effect, uint32_t seed) {
    float spread = radians(effect->spread_degrees);
    float first = ((unit_random(seed, SALT_DIRECTION_A) * 2.0f) - 1.0f) * spread;
    float second = ((unit_random(seed, SALT_DIRECTION_B) * 2.0f) - 1.0f) * spread * (1.0f - effect->flatness);
    Vec3 across = { sin_poly(first), 0.0f, cos_poly(first) };
    Vec3 up = { 0.0f, sin_poly(second), cos_poly(second) };
    up.z = up.z / f32_max(0.0001f, sqrtf(up.z < 0.0f ? -up.z : up.z));
    Vec3 local = { across.x * up.z, up.y, across.z * up.z };
    Vec3 forward = safe_normalize(effect->direction, world_up);
    Vec3 binormal = v3_cross(world_up, forward);
    binormal = v3_length(binormal) < 0.0001f ? (Vec3){ 0.0f, 0.0f, 1.0f } : v3_normalize(binormal);
    Vec3 normal = v3_cross(binormal, forward);
    Vec3 mixed = v3_add(v3_add(v3_scale(binormal, local.x), v3_scale(normal, local.y)), v3_scale(forward, local.z));
    return safe_normalize(mixed, forward);
}

/// Rotates the hue of `color` by `turns` of a full circle.
static Rgba rotate_hue(Rgba color, float turns) {
    if (turns == 0.0f) return color;
    float rotation = turns * two_pi;
    float cosine = cos_poly(rotation);
    float sine = sin_poly(rotation);
    float root = sqrtf(one_third);
    float shared = (1.0f - cosine) * one_third;
    float a = cosine + shared;
    float b = shared - (root * sine);
    float c = shared + (root * sine);
    return (Rgba){ (color.r * a) + (color.g * b) + (color.b * c), (color.r * c) + (color.g * a) + (color.b * b),
                   (color.r * b) + (color.g * c) + (color.b * a), color.a };
}

/// The hashed lattice value in [-1, 1) at an integer grid point.
static float lattice(int32_t x, int32_t y, int32_t z, uint32_t salt) {
    uint32_t hashed = mix_bits(((uint32_t)x * 73856093u) ^ ((uint32_t)y * 19349663u) ^ ((uint32_t)z * 83492791u) ^ mix_bits(salt));
    return ((float)(hashed >> 8u) / 8388608.0f) - 1.0f;
}

/// The analytic gradient of smooth 3D value noise at `point`.
static Vec3 noise_gradient(Vec3 point, uint32_t salt) {
    float fx = floorf(point.x);
    float fy = floorf(point.y);
    float fz = floorf(point.z);
    int32_t x = (int32_t)fx;
    int32_t y = (int32_t)fy;
    int32_t z = (int32_t)fz;
    Vec3 f = { point.x - fx, point.y - fy, point.z - fz };
    Vec3 u = { f.x * f.x * (3.0f - (2.0f * f.x)), f.y * f.y * (3.0f - (2.0f * f.y)), f.z * f.z * (3.0f - (2.0f * f.z)) };
    Vec3 du = { 6.0f * f.x * (1.0f - f.x), 6.0f * f.y * (1.0f - f.y), 6.0f * f.z * (1.0f - f.z) };
    float a = lattice(x, y, z, salt);
    float b = lattice(x + 1, y, z, salt);
    float c = lattice(x, y + 1, z, salt);
    float d = lattice(x + 1, y + 1, z, salt);
    float e = lattice(x, y, z + 1, salt);
    float g = lattice(x + 1, y, z + 1, salt);
    float h = lattice(x, y + 1, z + 1, salt);
    float k = lattice(x + 1, y + 1, z + 1, salt);
    float k1 = b - a;
    float k2 = c - a;
    float k3 = e - a;
    float k4 = a - b - c + d;
    float k5 = a - c - e + h;
    float k6 = a - b - e + g;
    float k7 = -a + b + c - d + e - g - h + k;
    return (Vec3){ du.x * (k1 + (k4 * u.y) + (k6 * u.z) + (k7 * u.y * u.z)), du.y * (k2 + (k5 * u.z) + (k4 * u.x) + (k7 * u.z * u.x)),
                   du.z * (k3 + (k6 * u.x) + (k5 * u.y) + (k7 * u.x * u.y)) };
}

/// A divergence-free vector from the curl of three noise fields.
static Vec3 curl_noise(Vec3 point) {
    Vec3 first = noise_gradient(point, 1u);
    Vec3 second = noise_gradient(point, 2u);
    Vec3 third = noise_gradient(point, 3u);
    return (Vec3){ third.y - second.z, first.z - third.x, second.x - first.y };
}

/// Axes of `basis` normalised, removing scale.
static Basis rotation_of(const Basis* basis) {
    return (Basis){ safe_normalize(basis->x, (Vec3){ 1.0f, 0.0f, 0.0f }), safe_normalize(basis->y, (Vec3){ 0.0f, 1.0f, 0.0f }),
                    safe_normalize(basis->z, (Vec3){ 0.0f, 0.0f, 1.0f }) };
}

/// Updates a particle's colour, size and angle from its age, ramps, curves and tint.
static void apply_display(Particle* particle, const Context* context) {
    const Effect* effect = context->effect;
    float offset = f32_clamp(particle->age / particle->lifetime, 0.0f, 1.0f);
    uint32_t seed = particle->seed;
    Rgba color = effect->color;
    if (effect->color_initial_ramp.count != 0) {
        Rgba initial = sample_gradient(&effect->color_initial_ramp, unit_random(seed, SALT_INITIAL_RAMP));
        color = (Rgba){ color.r * initial.r, color.g * initial.g, color.b * initial.b, color.a * initial.a };
    }
    if (effect->color_ramp.count != 0) {
        Rgba ramp = sample_gradient(&effect->color_ramp, offset);
        color = (Rgba){ color.r * ramp.r, color.g * ramp.g, color.b * ramp.b, color.a * ramp.a };
    }
    color.a *= sample_curve(&effect->alpha_curve, offset);
    color.a = f32_clamp(color.a, 0.0f, 1.0f);
    color = rotate_hue(color, parameter_value(effect, HUE_VARIATION, seed, offset));
    color = (Rgba){ color.r * particle->tint.r, color.g * particle->tint.g, color.b * particle->tint.b, color.a };
    particle->color = color;
    particle->size = effect->draw_size * f32_max(parameter_value(effect, SCALE, seed, offset), 0.0f);
    particle->angle = radians(parameter_value(effect, ANGLE, seed, offset)) + particle->spin;
}

/// Initialises `particle` from `seed` at the emission transform; false when
/// the amount ratio culls it.
static bool spawn(Particle* particle, uint32_t seed, const Context* context) {
    const Effect* effect = context->effect;
    *particle = particle_blank();
    particle->seed = seed;
    particle->tint = context->tint;
    if (context->amount_ratio < 1.0f && unit_random(seed, SALT_RATIO) >= context->amount_ratio) return false;
    particle->lifetime =
        f32_max(effect->lifetime * (1.0f - (effect->lifetime_randomness * unit_random(seed, SALT_LIFETIME))), minimum_lifetime);
    Vec3 local_position = v3_add(shape_point(effect, seed), effect->offset);
    Vec3 local_velocity = v3_scale(spread_direction(effect, seed), parameter_random(&effect->parameters[INITIAL_VELOCITY], seed, INITIAL_VELOCITY));
    Basis rotation = rotation_of(&context->basis);
    particle->position = v3_add(basis_apply(&context->basis, local_position), context->origin);
    particle->velocity = v3_add(basis_apply(&rotation, local_velocity), v3_scale(context->emitter_velocity, effect->inherit_velocity));
    particle->origin = context->origin;
    particle->axis = rotation.y;
    particle->active = true;
    apply_display(particle, context);
    return true;
}

/// Clips the segment's parameter range against one slab of a box; false
/// when the segment misses it.
static bool clip_axis(Clip* clip, float start, float delta, float low, float high, int axis) {
    float magnitude = delta < 0.0f ? -delta : delta;
    if (magnitude < 1e-9f) return !(start < low || start >= high);
    float near = (low - start) / delta;
    float far = (high - start) / delta;
    if (near > far) {
        float swap = near;
        near = far;
        far = swap;
    }
    if (near > clip->enter) {
        clip->enter = near;
        clip->axis = axis;
    }
    clip->exit = f32_min(clip->exit, far);
    return true;
}

/// The surface normal where a segment enters a box along `axis`, or against
/// the segment when it starts inside.
static Vec3 contact_normal(int axis, Vec3 delta) {
    Vec3 local;
    if (axis < 0) {
        local = v3_neg(delta);
    } else {
        float sign = component(delta, axis) > 0.0f ? -1.0f : 1.0f;
        local = axis == 0 ? (Vec3){ sign, 0.0f, 0.0f } : (axis == 1 ? (Vec3){ 0.0f, sign, 0.0f } : (Vec3){ 0.0f, 0.0f, sign });
    }
    float size = v3_length(local);
    return size > 1e-9f ? v3_div(local, size) : (Vec3){ 0.0f, 1.0f, 0.0f };
}

/// The nearest point where the segment from `from` to `to` enters a box.
static bool sweep(const Scene* scene, Vec3 from, Vec3 to, Contact* contact) {
    Aabb segment = { v3_min(from, to), v3_max(from, to) };
    Vec3 delta = v3_sub(to, from);
    float best = 1e30f;
    bool hit = false;
    for (uint32_t index = 0; index < BOXES; ++index) {
        const Aabb* box = &scene->boxes[index];
        if (!aabb_overlaps(box, &segment)) continue;
        Clip clip = { 0.0f, 1.0f, -1 };
        bool crosses = clip_axis(&clip, from.x, delta.x, box->min.x, box->max.x, 0) &&
                       clip_axis(&clip, from.y, delta.y, box->min.y, box->max.y, 1) &&
                       clip_axis(&clip, from.z, delta.z, box->min.z, box->max.z, 2);
        if (!crosses || clip.enter > clip.exit || clip.enter >= best) continue;
        best = clip.enter;
        *contact = (Contact){ v3_lerp(from, to, clip.enter), contact_normal(clip.axis, delta) };
        hit = true;
    }
    return hit;
}

/// Advances a particle by `step`: forces, wind, damping, turbulence,
/// controlled velocity and collision.
static Outcome integrate(const Scene* scene, Particle* particle, float step, const Context* context) {
    const Effect* effect = context->effect;
    Outcome outcome = { false, false, { 0.0f, 0.0f, 0.0f }, { 0.0f, 0.0f, 0.0f } };
    particle->age += step;
    if (particle->age >= particle->lifetime) {
        outcome.died = true;
        outcome.where = particle->position;
        outcome.velocity = particle->velocity;
        particle->active = false;
        return outcome;
    }
    float offset = particle->age / particle->lifetime;
    uint32_t seed = particle->seed;
    Vec3 zero = { 0.0f, 0.0f, 0.0f };
    Vec3 diff = v3_sub(particle->position, particle->origin);
    Vec3 radial = safe_normalize(diff, zero);
    Vec3 tangent = safe_normalize(v3_cross(particle->axis, diff), zero);
    Vec3 heading = safe_normalize(particle->velocity, zero);
    float linear = parameter_value(effect, LINEAR_ACCELERATION, seed, offset);
    float radial_force = parameter_value(effect, RADIAL_ACCELERATION, seed, offset);
    float tangential = parameter_value(effect, TANGENTIAL_ACCELERATION, seed, offset);
    Vec3 force = v3_add(v3_add(v3_add(effect->gravity, v3_scale(heading, linear)), v3_scale(radial, radial_force)),
                        v3_scale(tangent, tangential));
    particle->velocity = v3_add(particle->velocity, v3_scale(force, step));
    if (effect->wind_influence > 0.0f) {
        float rate = effect->wind_influence * wind_response * step;
        float pull = rate / (1.0f + rate);
        particle->velocity.x += (context->wind.x - particle->velocity.x) * pull;
        particle->velocity.z += (context->wind.z - particle->velocity.z) * pull;
        particle->velocity.y += context->wind.y * effect->wind_influence * wind_response * step;
    }
    float drag = parameter_value(effect, DAMPING, seed, offset);
    if (drag > 0.0f) {
        float speed = v3_length(particle->velocity);
        if (speed > 0.0f) particle->velocity = v3_scale(particle->velocity, f32_max(speed - (drag * step), 0.0f) / speed);
    }
    if (effect->turbulence_enabled) {
        float rate = 1.0f + (effect->turbulence_speed_random * ((unit_random(seed, SALT_TURBULENCE_SPEED) * 2.0f) - 1.0f));
        Vec3 sample_point = v3_add(v3_div(particle->position, f32_max(effect->turbulence_scale, 1e-3f)),
                                   v3_scale(effect->turbulence_speed, context->time * rate));
        Vec3 direction = safe_normalize(curl_noise(sample_point), zero);
        float influence = f32_clamp(parameter_value(effect, TURBULENCE_INFLUENCE, seed, offset), 0.0f, 1.0f);
        float blend = influence * (step * turbulence_reference_rate);
        float speed = v3_length(particle->velocity);
        particle->velocity = v3_add(v3_lerp(particle->velocity, v3_scale(direction, speed), blend),
                                    v3_scale(direction, effect->turbulence_strength * step));
    }
    float radial_speed = parameter_value(effect, RADIAL_VELOCITY, seed, offset);
    float orbit = parameter_value(effect, ORBIT_VELOCITY, seed, offset);
    Vec3 controlled = v3_add(v3_scale(radial, radial_speed), v3_scale(v3_cross(particle->axis, diff), orbit * two_pi));
    Vec3 from = particle->position;
    Vec3 to = v3_add(from, v3_scale(v3_add(particle->velocity, controlled), step));
    particle->position = to;
    Contact contact;
    if (effect->collision != COLLISION_DISABLED && sweep(scene, from, to, &contact)) {
        outcome.where = contact.position;
        Vec3 velocity = particle->velocity;
        if (effect->collision == COLLISION_HIDE_ON_CONTACT) {
            outcome.collided = true;
            outcome.velocity = velocity;
            particle->active = false;
            outcome.died = true;
            return outcome;
        }
        float radius = effect->use_scale ? particle->size * 0.5f : 0.0f;
        Vec3 rest = v3_add(contact.position, v3_scale(contact.normal, radius + contact_offset));
        Vec3 normal = contact.normal;
        float into = v3_dot(velocity, normal);
        outcome.collided = -into > impact_speed;
        if (into < 0.0f) {
            Vec3 slide = v3_sub(velocity, v3_scale(normal, into));
            velocity = v3_sub(v3_scale(slide, 1.0f - effect->friction), v3_scale(normal, into * effect->bounce));
        }
        particle->position = rest;
        particle->velocity = velocity;
        outcome.velocity = velocity;
    }
    float spin = parameter_value(effect, ANGULAR_VELOCITY, seed, offset);
    particle->spin += radians(spin) * step;
    apply_display(particle, context);
    return outcome;
}

/// The next inactive particle from the emit cursor, advancing the cursor;
/// null when the pool is full.
static Particle* free_particle(Emitter* emitter, Particle* pool, uint32_t count) {
    for (uint32_t probe = 0; probe < count; ++probe) {
        uint32_t index = (emitter->emit_cursor + probe) % count;
        if (!pool[index].active) {
            emitter->emit_cursor = (index + 1) % count;
            return &pool[index];
        }
    }
    return NULL;
}

/// Spawns the parent's sub emitter particles at `origin` into free sub slots.
static void emit_sub(Emitter* emitter, Particle* sub_pool, const Effect* parent, const Effect* sub, SubOrigin origin) {
    if (emitter->sub_count == 0) return;
    Context context = context_make(sub);
    context.origin = origin.position;
    context.time = emitter->elapsed;
    uint32_t pool = emitter->sub_count;
    for (uint32_t count = 0; count < parent->sub_amount; ++count) {
        uint32_t found = pool;
        for (uint32_t probe = 0; probe < pool; ++probe) {
            uint32_t index = (emitter->sub_cursor + probe) % pool;
            if (!sub_pool[index].active) {
                found = index;
                break;
            }
        }
        if (found == pool) return;
        emitter->sub_cursor = (found + 1) % pool;
        uint32_t number = 0xB5297A4Du + emitter->bursts;
        emitter->bursts += 1;
        uint32_t seed = mix_bits(emitter->current_seed ^ mix_bits(number));
        Particle* particle = &sub_pool[found];
        if (spawn(particle, seed, &context) && parent->sub_keep_velocity) particle->velocity = v3_add(particle->velocity, origin.velocity);
    }
}

/// Counts a collision and spawns sub emitter particles by the sub emitter mode.
static void react(Emitter* emitter, Particle* sub_pool, uint64_t* collisions, const Effect* parent, const Effect* sub,
                  Particle* particle, Outcome outcome, float step_seconds) {
    if (outcome.collided) ++*collisions;
    if (sub == NULL) return;
    switch (parent->sub_mode) {
    case SUB_AT_END:
        if (outcome.died && !outcome.collided) emit_sub(emitter, sub_pool, parent, sub, (SubOrigin){ outcome.where, outcome.velocity });
        break;
    case SUB_AT_COLLISION:
        if (outcome.collided) emit_sub(emitter, sub_pool, parent, sub, (SubOrigin){ outcome.where, outcome.velocity });
        break;
    case SUB_CONSTANT: {
        if (!particle->active) break;
        float interval = 1.0f / f32_max(parent->sub_frequency, 1e-3f);
        particle->sub_timer += step_seconds;
        while (particle->sub_timer >= interval) {
            particle->sub_timer -= interval;
            emit_sub(emitter, sub_pool, parent, sub, (SubOrigin){ particle->position, particle->velocity });
        }
        break;
    }
    case SUB_DISABLED:
        break;
    }
}

/// A seed for one numbered emission of an emitter.
static uint32_t emission_seed(const Emitter* emitter, uint32_t number) { return mix_bits(emitter->current_seed ^ mix_bits(number)); }

/// Emits the particles whose restart phase falls inside one phase interval.
static void emit_interval(const Scene* scene, Emitter* emitter, Pools pools, uint64_t* collisions, const Context* context,
                          const Effect* sub, float from, float to, float remaining, uint64_t cycle, float step_seconds) {
    const Effect* effect = context->effect;
    uint32_t amount = POOL;
    float lifetime = f32_max(effect->lifetime, minimum_lifetime);
    for (uint32_t index = 0; index < amount; ++index) {
        uint32_t number = (uint32_t)((cycle * amount) + index);
        uint32_t seed = emission_seed(emitter, number);
        float restart_phase = (float)index / (float)amount;
        if (effect->randomness > 0.0f) restart_phase += effect->randomness * unit_random(seed, SALT_PHASE) / (float)amount;
        restart_phase *= 1.0f - effect->explosiveness;
        if (restart_phase < from || restart_phase >= to) continue;
        Particle* particle = &pools.main[index];
        if (!spawn(particle, seed, context)) continue;
        float local_delta = f32_clamp((remaining - restart_phase) * lifetime, 0.0f, step_seconds);
        Outcome outcome = integrate(scene, particle, local_delta, context);
        react(emitter, pools.sub, collisions, effect, sub, particle, outcome, local_delta);
    }
}

/// Advances one emitter by `step_seconds`: integrates particles, emits by
/// phase, then spawns bursts.
static void step(const Scene* scene, Emitter* emitter, Pools pools, uint64_t* collisions, float step_seconds, Vec3 origin) {
    const Effect* effect = &scene->effects[emitter->effect];
    const Effect* sub = emitter->sub_effect == no_effect ? NULL : &scene->effects[emitter->sub_effect];
    Context context = context_make(effect);
    context.basis = emitter->basis;
    context.origin = origin;
    context.emitter_velocity = emitter->velocity;
    context.time = emitter->elapsed;
    context.amount_ratio = f32_clamp(emitter->amount_ratio, 0.0f, 1.0f);
    context.wind = scene->wind;
    context.tint = emitter->tint;
    for (uint32_t index = 0; index < POOL; ++index) {
        Particle* particle = &pools.main[index];
        if (!particle->active) continue;
        Outcome outcome = integrate(scene, particle, step_seconds, &context);
        react(emitter, pools.sub, collisions, effect, sub, particle, outcome, step_seconds);
    }
    if (sub != NULL) {
        Context sub_context = context_make(sub);
        sub_context.time = emitter->elapsed;
        sub_context.wind = scene->wind;
        sub_context.tint = emitter->tint;
        for (uint32_t index = 0; index < emitter->sub_count; ++index) {
            Particle* particle = &pools.sub[index];
            if (!particle->active) continue;
            Outcome outcome = integrate(scene, particle, step_seconds, &sub_context);
            if (outcome.collided) ++*collisions;
        }
    }
    float lifetime = f32_max(effect->lifetime, minimum_lifetime);
    if (emitter->emitting) {
        float previous = emitter->phase;
        float phase = previous + (step_seconds / lifetime);
        bool wrapped = phase >= 1.0f;
        if (wrapped) phase = phase - floorf(phase);
        if (!wrapped) {
            emit_interval(scene, emitter, pools, collisions, &context, sub, previous, phase, phase, emitter->cycle, step_seconds);
        } else {
            emit_interval(scene, emitter, pools, collisions, &context, sub, previous, 1.0f, 1.0f + phase, emitter->cycle, step_seconds);
            if (!effect->one_shot) {
                emit_interval(scene, emitter, pools, collisions, &context, sub, 0.0f, phase, phase, emitter->cycle + 1, step_seconds);
            }
        }
        emitter->phase = phase;
        if (wrapped) {
            ++emitter->cycle;
            if (effect->one_shot) {
                emitter->emitting = false;
                emitter->phase = 0.0f;
            }
        }
    }
    Context burst_context = context;
    burst_context.amount_ratio = 1.0f;
    while (emitter->burst_pending > 0) {
        --emitter->burst_pending;
        Particle* slot = free_particle(emitter, pools.main, POOL);
        if (slot == NULL) {
            emitter->burst_pending = 0;
            break;
        }
        uint32_t number = 0x68E31DA4u + emitter->bursts;
        emitter->bursts += 1;
        (void)spawn(slot, emission_seed(emitter, number), &burst_context);
    }
    emitter->elapsed += step_seconds;
}

/// Counts the live particles of an emitter.
static void finish(Emitter* emitter, Pools pools) {
    uint32_t alive = 0;
    for (uint32_t index = 0; index < POOL; ++index) alive += pools.main[index].active ? 1u : 0u;
    for (uint32_t index = 0; index < emitter->sub_count; ++index) alive += pools.sub[index].active ? 1u : 0u;
    emitter->alive = alive;
}

/// Appends a point to a curve.
static void curve_add(Curve* curve, float offset, float value) {
    curve->points[curve->count++] = (CurvePoint){ offset, value };
}

/// Appends a stop to a gradient.
static void gradient_add(Gradient* gradient, float offset, Rgba value) {
    gradient->stops[gradient->count++] = (Stop){ offset, value };
}

/// A curve from `count` points.
static Curve make_curve(bool smooth, const CurvePoint* points, uint32_t count) {
    Curve curve;
    memset(&curve, 0, sizeof curve);
    curve.smooth = smooth;
    for (uint32_t index = 0; index < count; ++index) curve_add(&curve, points[index].offset, points[index].value);
    return curve;
}

/// A gradient from `count` stops.
static Gradient make_gradient(const Stop* stops, uint32_t count) {
    Gradient gradient;
    memset(&gradient, 0, sizeof gradient);
    for (uint32_t index = 0; index < count; ++index) gradient_add(&gradient, stops[index].offset, stops[index].value);
    return gradient;
}

#define CURVE(smooth, ...) \
    make_curve(smooth, (const CurvePoint[]){ __VA_ARGS__ }, sizeof((const CurvePoint[]){ __VA_ARGS__ }) / sizeof(CurvePoint))
#define GRADIENT(...) make_gradient((const Stop[]){ __VA_ARGS__ }, sizeof((const Stop[]){ __VA_ARGS__ }) / sizeof(Stop))

/// Sets one parameter of an effect.
static void set_parameter(Effect* effect, Param id, float minimum, float maximum, Curve curve) {
    effect->parameters[id] = (Parameter){ minimum, maximum, curve };
}

/// A curve with no points.
static Curve no_curve(void) {
    Curve curve;
    memset(&curve, 0, sizeof curve);
    return curve;
}

/// A shower of bouncing sparks that leave dust where they land.
static Effect make_sparks(void) {
    Effect e = effect_make();
    e.lifetime = 1.6f;
    e.lifetime_randomness = 0.4f;
    e.randomness = 0.5f;
    e.box_shape = true;
    e.box_extents = (Vec3){ 0.2f, 0.05f, 0.2f };
    e.spread_degrees = 35.0f;
    e.flatness = 0.2f;
    e.inherit_velocity = 0.5f;
    e.wind_influence = 0.1f;
    e.draw_size = 0.15f;
    e.color = (Rgba){ 1.0f, 0.6f, 0.3f, 1.0f };
    e.color_ramp = GRADIENT({ 0.0f, { 1.0f, 1.0f, 0.8f, 1.0f } }, { 0.4f, { 1.0f, 0.5f, 0.2f, 1.0f } }, { 1.0f, { 0.3f, 0.1f, 0.1f, 0.0f } });
    e.alpha_curve = CURVE(false, { 0.0f, 0.0f }, { 0.1f, 1.0f }, { 1.0f, 0.0f });
    set_parameter(&e, INITIAL_VELOCITY, 4.0f, 9.0f, no_curve());
    set_parameter(&e, LINEAR_ACCELERATION, 0.0f, 1.0f, CURVE(false, { 0.0f, 1.0f }, { 1.0f, 0.0f }));
    set_parameter(&e, DAMPING, 0.2f, 0.6f, CURVE(true, { 0.0f, 1.0f }, { 1.0f, 0.2f }));
    set_parameter(&e, ANGLE, 0.0f, 360.0f, no_curve());
    set_parameter(&e, ANGULAR_VELOCITY, -90.0f, 90.0f, no_curve());
    set_parameter(&e, SCALE, 0.6f, 1.2f, CURVE(false, { 0.0f, 0.2f }, { 0.2f, 1.0f }, { 1.0f, 0.1f }));
    set_parameter(&e, HUE_VARIATION, -0.05f, 0.05f, no_curve());
    e.collision = COLLISION_RIGID;
    e.friction = 0.3f;
    e.bounce = 0.5f;
    e.use_scale = true;
    e.sub_mode = SUB_AT_COLLISION;
    e.sub_amount = 2;
    e.sub_effect = 4;
    return e;
}

/// Buoyant smoke pushed by wind and curl noise that vanishes on contact.
static Effect make_smoke(void) {
    Effect e = effect_make();
    e.lifetime = 2.5f;
    e.lifetime_randomness = 0.3f;
    e.randomness = 1.0f;
    e.box_shape = true;
    e.box_extents = (Vec3){ 0.5f, 0.1f, 0.5f };
    e.spread_degrees = 25.0f;
    e.gravity = (Vec3){ 0.0f, 0.6f, 0.0f };
    e.wind_influence = 0.8f;
    e.draw_size = 0.8f;
    e.color = (Rgba){ 0.5f, 0.5f, 0.55f, 1.0f };
    e.color_initial_ramp = GRADIENT({ 0.0f, { 1.0f, 0.9f, 0.8f, 1.0f } }, { 1.0f, { 0.8f, 0.9f, 1.0f, 0.7f } });
    e.color_ramp = GRADIENT({ 0.0f, { 1.0f, 1.0f, 1.0f, 1.0f } }, { 1.0f, { 0.4f, 0.4f, 0.4f, 1.0f } });
    e.alpha_curve = CURVE(true, { 0.0f, 0.0f }, { 0.2f, 0.6f }, { 1.0f, 0.0f });
    set_parameter(&e, INITIAL_VELOCITY, 0.5f, 1.5f, no_curve());
    set_parameter(&e, RADIAL_ACCELERATION, -0.2f, 0.2f, no_curve());
    set_parameter(&e, TANGENTIAL_ACCELERATION, 0.5f, 1.0f, no_curve());
    set_parameter(&e, DAMPING, 0.3f, 0.8f, no_curve());
    set_parameter(&e, SCALE, 1.0f, 2.5f, CURVE(true, { 0.0f, 0.3f }, { 1.0f, 1.0f }));
    set_parameter(&e, TURBULENCE_INFLUENCE, 0.3f, 0.7f, CURVE(false, { 0.0f, 0.2f }, { 1.0f, 1.0f }));
    e.turbulence_enabled = true;
    e.turbulence_strength = 1.5f;
    e.turbulence_scale = 3.0f;
    e.collision = COLLISION_HIDE_ON_CONTACT;
    return e;
}

/// Debris that erupts at once, tumbles, and bursts into embers where it ends.
static Effect make_debris(void) {
    Effect e = effect_make();
    e.lifetime = 2.0f;
    e.randomness = 0.3f;
    e.explosiveness = 0.7f;
    e.box_shape = true;
    e.box_extents = (Vec3){ 0.4f, 0.4f, 0.4f };
    e.spread_degrees = 60.0f;
    e.draw_size = 0.3f;
    e.color = (Rgba){ 0.6f, 0.5f, 0.4f, 1.0f };
    e.color_ramp = GRADIENT({ 0.0f, { 1.0f, 1.0f, 1.0f, 1.0f } }, { 1.0f, { 0.5f, 0.5f, 0.5f, 1.0f } });
    set_parameter(&e, INITIAL_VELOCITY, 3.0f, 8.0f, no_curve());
    set_parameter(&e, ORBIT_VELOCITY, 0.0f, 0.1f, no_curve());
    set_parameter(&e, ANGLE, 0.0f, 360.0f, no_curve());
    set_parameter(&e, ANGULAR_VELOCITY, -180.0f, 180.0f, no_curve());
    set_parameter(&e, SCALE, 0.5f, 1.5f, no_curve());
    e.collision = COLLISION_RIGID;
    e.friction = 0.5f;
    e.bounce = 0.4f;
    e.sub_mode = SUB_AT_END;
    e.sub_amount = 3;
    e.sub_keep_velocity = true;
    e.sub_effect = 5;
    return e;
}

/// A fountain that bounces and sheds dust as it flies.
static Effect make_fountain(void) {
    Effect e = effect_make();
    e.lifetime = 1.2f;
    e.lifetime_randomness = 0.2f;
    e.spread_degrees = 12.0f;
    e.draw_size = 0.2f;
    e.color = (Rgba){ 0.5f, 0.7f, 1.0f, 1.0f };
    e.alpha_curve = CURVE(false, { 0.0f, 1.0f }, { 0.8f, 1.0f }, { 1.0f, 0.0f });
    set_parameter(&e, INITIAL_VELOCITY, 8.0f, 12.0f, no_curve());
    set_parameter(&e, RADIAL_VELOCITY, 0.0f, 0.5f, no_curve());
    e.collision = COLLISION_RIGID;
    e.friction = 0.1f;
    e.bounce = 0.6f;
    e.sub_mode = SUB_CONSTANT;
    e.sub_frequency = 6.0f;
    e.sub_amount = 1;
    e.sub_keep_velocity = true;
    e.sub_effect = 4;
    return e;
}

/// Short-lived dust puffs.
static Effect make_dust(void) {
    Effect e = effect_make();
    e.amount = SUB_POOL;
    e.lifetime = 0.4f;
    e.spread_degrees = 80.0f;
    e.gravity = (Vec3){ 0.0f, -1.0f, 0.0f };
    e.draw_size = 0.1f;
    e.color = (Rgba){ 0.7f, 0.65f, 0.6f, 1.0f };
    e.alpha_curve = CURVE(false, { 0.0f, 1.0f }, { 1.0f, 0.0f });
    set_parameter(&e, INITIAL_VELOCITY, 0.5f, 2.0f, no_curve());
    return e;
}

/// Embers that bounce once.
static Effect make_ember(void) {
    Effect e = effect_make();
    e.amount = SUB_POOL;
    e.lifetime = 0.6f;
    e.spread_degrees = 80.0f;
    e.gravity = (Vec3){ 0.0f, -4.0f, 0.0f };
    e.draw_size = 0.08f;
    e.color = (Rgba){ 1.0f, 0.4f, 0.1f, 1.0f };
    set_parameter(&e, INITIAL_VELOCITY, 1.0f, 3.0f, no_curve());
    e.collision = COLLISION_RIGID;
    e.bounce = 0.3f;
    return e;
}

/// The six effects, the ground and boxes, and the wind.
static void make_scene(Scene* scene, Rng* rng) {
    scene->effects[0] = make_sparks();
    scene->effects[1] = make_smoke();
    scene->effects[2] = make_debris();
    scene->effects[3] = make_fountain();
    scene->effects[4] = make_dust();
    scene->effects[5] = make_ember();
    for (uint32_t i = 0; i < 4; ++i) scene->effects[i].amount = POOL;
    scene->wind = (Vec3){ 3.0f, 0.0f, 1.0f };
    scene->boxes[0] = (Aabb){ { -1000.0f, -1000.0f, -1000.0f }, { 1000.0f, 0.0f, 1000.0f } };
    for (uint32_t i = 1; i < BOXES; ++i) {
        Vec3 where = v3_random(rng, 0.0f, 1.0f);
        Vec3 half = v3_random(rng, 0.5f, 2.0f);
        Vec3 centre = { where.x * 40.0f, half.y, where.z * 40.0f };
        scene->boxes[i] = (Aabb){ v3_sub(centre, half), v3_add(centre, half) };
    }
}

/// An emitter at a random place and orientation, moving at a random velocity.
static Emitter make_emitter(uint32_t index, const Scene* scene, Rng* rng) {
    Emitter emitter = emitter_make();
    emitter.effect = index % 4;
    emitter.sub_effect = scene->effects[emitter.effect].sub_effect;
    emitter.first = index * POOL;
    emitter.sub_first = index * SUB_POOL;
    emitter.sub_count = emitter.sub_effect == no_effect ? 0 : scene->effects[emitter.sub_effect].amount;
    Quat orientation = quat_random(rng);
    emitter.basis = basis_from_quat(orientation);
    Vec3 where = v3_random(rng, 0.0f, 1.0f);
    emitter.origin = (Vec3){ where.x * 40.0f, 2.0f + (where.y * 4.0f), where.z * 40.0f };
    emitter.move = v3_random(rng, -3.0f, 3.0f);
    float red = 0.6f + (rng_unit(rng) * 0.4f);
    float green = 0.6f + (rng_unit(rng) * 0.4f);
    float blue = 0.6f + (rng_unit(rng) * 0.4f);
    emitter.tint = (Rgba){ red, green, blue, 1.0f };
    emitter.amount_ratio = 0.8f + (rng_unit(rng) * 0.2f);
    emitter.speed_scale = 0.9f + (rng_unit(rng) * 0.2f);
    emitter.current_seed = mix_bits((index * golden) ^ mix_bits(1u));
    return emitter;
}

/// Two floats packed by bit pattern into one word.
static uint64_t pack_pair(float low, float high) { return (uint64_t)f32_bits(low) | ((uint64_t)f32_bits(high) << 32); }

/// Folds one particle's state into a 64-bit word.
static uint64_t fold_particle(const Particle* p) {
    uint64_t word = pack_pair(p->position.x, p->position.y);
    word ^= pack_pair(p->position.z, p->velocity.x) * 0x9E3779B97F4A7C15ull;
    word ^= pack_pair(p->velocity.y, p->velocity.z) * 0xC2B2AE3D27D4EB4Full;
    word ^= pack_pair(p->size, p->color.r) * 0x165667B19E3779F9ull;
    word ^= pack_pair(p->color.g, p->color.b) * 0x85EBCA77C2B2AE63ull;
    word ^= pack_pair(p->color.a, p->active ? 1.0f : 0.0f) * 0x27D4EB2F165667C5ull;
    return word;
}

/// The particle pools of emitter `index`.
static Pools pools_of(Particles* s, uint32_t index) {
    const Emitter* emitter = &s->emitters[index];
    return (Pools){ s->particles + emitter->first, s->sub_particles + emitter->sub_first };
}

/// Runs one emitter's preprocess steps from a fresh start.
static void preprocess(Particles* s, uint32_t index) {
    Emitter* emitter = &s->emitters[index];
    float step_seconds = preprocess_seconds / (float)PREPROCESS_STEPS;
    for (uint32_t count = 0; count < PREPROCESS_STEPS; ++count) {
        step(&s->scene, emitter, pools_of(s, index), &s->collisions, step_seconds, emitter->origin);
    }
}

/// Advances every emitter by one update in shared substeps.
static void advance(Particles* s) {
    Vec3 start[EMITTERS];
    Vec3 end[EMITTERS];
    float scaled[EMITTERS];
    for (uint32_t index = 0; index < EMITTERS; ++index) {
        Emitter* emitter = &s->emitters[index];
        start[index] = emitter->origin;
        end[index] = v3_add(emitter->origin, v3_scale(emitter->move, update_seconds));
        emitter->origin = end[index];
        emitter->velocity = v3_div(v3_sub(end[index], start[index]), update_seconds);
        scaled[index] = update_seconds * f32_max(emitter->speed_scale, 0.0f);
    }
    for (uint32_t sub = 0; sub < SUBSTEPS; ++sub) {
        float fraction = (float)(sub + 1) / (float)SUBSTEPS;
        for (uint32_t index = 0; index < EMITTERS; ++index) {
            step(&s->scene, &s->emitters[index], pools_of(s, index), &s->collisions, scaled[index] / (float)SUBSTEPS,
                 v3_lerp(start[index], end[index], fraction));
        }
    }
    for (uint32_t index = 0; index < EMITTERS; ++index) finish(&s->emitters[index], pools_of(s, index));
}

/// Builds the scene and emitters, runs their preprocess and keeps that state.
static void setup(void* state) {
    Particles* s = state;
    Rng rng = { 0x9a7 };
    make_scene(&s->scene, &rng);
    s->particles = xalloc((size_t)EMITTERS * POOL * sizeof(Particle));
    s->initial_particles = xalloc((size_t)EMITTERS * POOL * sizeof(Particle));
    s->sub_particles = xalloc((size_t)EMITTERS * SUB_POOL * sizeof(Particle));
    s->initial_sub_particles = xalloc((size_t)EMITTERS * SUB_POOL * sizeof(Particle));
    for (size_t i = 0; i < (size_t)EMITTERS * POOL; ++i) s->particles[i] = particle_blank();
    for (size_t i = 0; i < (size_t)EMITTERS * SUB_POOL; ++i) s->sub_particles[i] = particle_blank();
    for (uint32_t index = 0; index < EMITTERS; ++index) s->emitters[index] = make_emitter(index, &s->scene, &rng);
    for (uint32_t index = 0; index < EMITTERS; ++index) preprocess(s, index);
    memcpy(s->initial_emitters, s->emitters, sizeof s->emitters);
    memcpy(s->initial_particles, s->particles, (size_t)EMITTERS * POOL * sizeof(Particle));
    memcpy(s->initial_sub_particles, s->sub_particles, (size_t)EMITTERS * SUB_POOL * sizeof(Particle));
}

/// Resets to the preprocessed state, runs every update and hashes the result.
static uint64_t run(void* state) {
    Particles* s = state;
    memcpy(s->emitters, s->initial_emitters, sizeof s->emitters);
    memcpy(s->particles, s->initial_particles, (size_t)EMITTERS * POOL * sizeof(Particle));
    memcpy(s->sub_particles, s->initial_sub_particles, (size_t)EMITTERS * SUB_POOL * sizeof(Particle));
    s->collisions = 0;
    for (uint32_t update = 0; update < UPDATES; ++update) {
        if (update % 2 == 0) {
            for (uint32_t index = 0; index < EMITTERS; index += 3) s->emitters[index].burst_pending += 256;
        }
        advance(s);
    }
    uint64_t h = hash_add(0, s->collisions);
    for (uint32_t index = 0; index < EMITTERS; ++index) h = hash_add(h, s->emitters[index].alive);
    for (size_t i = 0; i < (size_t)EMITTERS * POOL; ++i) h = hash_add(h, fold_particle(&s->particles[i]));
    for (size_t i = 0; i < (size_t)EMITTERS * SUB_POOL; ++i) h = hash_add(h, fold_particle(&s->sub_particles[i]));
    return h;
}

/// Frees the particle pools.
static void teardown(void* state) {
    Particles* s = state;
    free(s->particles);
    free(s->initial_particles);
    free(s->sub_particles);
    free(s->initial_sub_particles);
}

const Case particles_case = { "particles", sizeof(Particles), setup, run, teardown };
