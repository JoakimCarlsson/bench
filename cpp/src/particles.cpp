#include "particles.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <initializer_list>
#include <vector>

#include "hash.hpp"
#include "vecmath.hpp"

namespace bench {

namespace {

using vm::Basis;
using vm::Vec3;

constexpr uint32_t emitter_count = 12;
constexpr uint32_t pool_size = 2048;
constexpr uint32_t sub_pool_size = 256;
constexpr uint32_t updates = 4;
constexpr uint32_t substeps = 6;
constexpr uint32_t preprocess_steps = 45;
constexpr float preprocess_seconds = 1.5f;
constexpr float update_seconds = 0.016666668f;
constexpr float two_pi = 6.2831855f;
constexpr float degrees_to_radians = 0.017453292f;
constexpr float minimum_lifetime = 1e-4f;
constexpr float contact_offset = 1e-3f;
constexpr float impact_speed = 0.05f;
constexpr float turbulence_reference_rate = 60.0f;
constexpr float wind_response = 2.0f;
constexpr float one_third = 0.33333334f;
constexpr uint32_t golden = 0x9E3779B9u;
constexpr uint32_t no_effect = 0xFFFFFFFFu;
constexpr uint32_t curve_capacity = 8;
constexpr uint32_t gradient_capacity = 6;
constexpr uint32_t effect_count = 6;
constexpr uint32_t box_count = 7;
constexpr Vec3 world_up{0.0f, 1.0f, 0.0f};

enum Salt : uint32_t {
    salt_lifetime = 1,
    salt_ratio,
    salt_initial_ramp,
    salt_phase,
    salt_shape_a,
    salt_shape_b,
    salt_shape_c,
    salt_direction_a,
    salt_direction_b,
    salt_turbulence_speed,
    salt_parameter_base = 32,
};

enum Param : uint32_t {
    initial_velocity,
    angular_velocity,
    orbit_velocity,
    radial_velocity,
    linear_acceleration,
    radial_acceleration,
    tangential_acceleration,
    damping,
    angle,
    scale,
    hue_variation,
    turbulence_influence,
    param_count,
};

enum class Collision : uint8_t { Disabled, Rigid, HideOnContact };

enum class SubMode : uint8_t { Disabled, Constant, AtEnd, AtCollision };

struct Rgba {
    float r{1.0f};
    float g{1.0f};
    float b{1.0f};
    float a{1.0f};
};

struct CurvePoint {
    float offset{};
    float value{1.0f};
};

struct Curve {
    bool smooth{};
    uint32_t count{};
    std::array<CurvePoint, curve_capacity> points{};
};

struct Stop {
    float offset{};
    Rgba value{};
};

struct Gradient {
    uint32_t count{};
    std::array<Stop, gradient_capacity> stops{};
};

struct Parameter {
    float minimum{};
    float maximum{};
    Curve curve{};
};

struct Effect {
    uint32_t amount{16};
    float lifetime{1.0f};
    bool one_shot{};
    float explosiveness{};
    float randomness{};
    float lifetime_randomness{};
    bool box_shape{};
    Vec3 offset{};
    Vec3 box_extents{1.0f, 1.0f, 1.0f};
    Vec3 direction{0.0f, 1.0f, 0.0f};
    float spread_degrees{45.0f};
    float flatness{};
    float inherit_velocity{};
    Vec3 gravity{0.0f, -9.8f, 0.0f};
    float wind_influence{};
    std::array<Parameter, param_count> parameters{};
    Rgba color{};
    Gradient color_ramp{};
    Gradient color_initial_ramp{};
    Curve alpha_curve{};
    bool turbulence_enabled{};
    float turbulence_strength{1.0f};
    float turbulence_scale{4.0f};
    Vec3 turbulence_speed{0.0f, 0.5f, 0.0f};
    float turbulence_speed_random{0.2f};
    Collision collision{Collision::Disabled};
    float friction{};
    float bounce{};
    bool use_scale{};
    SubMode sub_mode{SubMode::Disabled};
    float sub_frequency{4.0f};
    uint32_t sub_amount{1};
    bool sub_keep_velocity{};
    uint32_t sub_effect{no_effect};
    float draw_size{0.25f};

    Effect() {
        parameters[initial_velocity] = Parameter{1.0f, 1.0f, {}};
        parameters[scale] = Parameter{1.0f, 1.0f, {}};
    }
};

struct Particle {
    Vec3 position{};
    Vec3 velocity{};
    Vec3 origin{};
    Vec3 axis{0.0f, 1.0f, 0.0f};
    Rgba color{};
    Rgba tint{};
    float angle{};
    float spin{};
    float size{};
    float age{};
    float lifetime{};
    float sub_timer{};
    uint32_t seed{};
    bool active{};
};

struct Emitter {
    uint32_t effect{};
    uint32_t sub_effect{no_effect};
    uint32_t first{};
    uint32_t sub_first{};
    uint32_t sub_count{};
    uint32_t emit_cursor{};
    uint32_t sub_cursor{};
    uint32_t bursts{};
    uint32_t current_seed{};
    uint32_t burst_pending{};
    uint32_t alive{};
    uint64_t cycle{};
    float phase{};
    float elapsed{};
    bool emitting{true};
    float amount_ratio{1.0f};
    float speed_scale{1.0f};
    Rgba tint{};
    Basis basis{};
    Vec3 origin{};
    Vec3 move{};
    Vec3 velocity{};
};

struct Scene {
    std::array<Effect, effect_count> effects{};
    std::array<vm::Aabb, box_count> boxes{};
    Vec3 wind{};
};

struct Pools {
    Particle* main{};
    Particle* sub{};
};

struct Context {
    const Effect* effect{};
    Basis basis{};
    Vec3 origin{};
    Vec3 emitter_velocity{};
    float time{};
    float amount_ratio{1.0f};
    Vec3 wind{};
    Rgba tint{};
};

struct Contact {
    Vec3 position{};
    Vec3 normal{};
};

struct Outcome {
    bool died{};
    bool collided{};
    Vec3 where{};
    Vec3 velocity{};
};

struct SubOrigin {
    Vec3 position{};
    Vec3 velocity{};
};

struct Clip {
    float enter{0.0f};
    float exit{1.0f};
    int axis{-1};
};

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

/// Linear interpolation from `a` to `b` by `t`.
constexpr float mix_float(float a, float b, float t) { return a + ((b - a) * t); }

/// Linear interpolation between two vectors.
constexpr Vec3 lerp(Vec3 a, Vec3 b, float t) { return a + (b - a) * t; }

/// Larger of two floats as `std::max` picks it.
constexpr float max_float(float a, float b) { return a < b ? b : a; }

/// Smaller of two floats as `std::min` picks it.
constexpr float min_float(float a, float b) { return b < a ? b : a; }

/// `value` limited to [low, high] as `std::clamp` does it.
constexpr float clamp_float(float value, float low, float high) { return value < low ? low : (high < value ? high : value); }

/// Degrees in radians.
constexpr float radians(float degrees) { return degrees * degrees_to_radians; }

/// Taylor polynomial of the sine, accurate for small angles.
constexpr float sin_poly(float x) { return x - ((x * x * x) / 6.0f) + ((x * x * x * x * x) / 120.0f); }

/// Taylor polynomial of the cosine, accurate for small angles.
constexpr float cos_poly(float x) { return 1.0f - ((x * x) / 2.0f) + ((x * x * x * x) / 24.0f); }

/// Unit vector along `value`, or `fallback` when it is too short.
Vec3 safe_normalize(Vec3 value, Vec3 fallback) {
    const float size = vm::length(value);
    return size > 1e-6f ? value / size : fallback;
}

/// The component of `value` along axis 0, 1 or 2.
constexpr float component(Vec3 value, int axis) { return axis == 0 ? value.x : (axis == 1 ? value.y : value.z); }

/// Smoothstep of a blend factor.
constexpr float smooth_blend(float blend) { return blend * blend * (3.0f - (2.0f * blend)); }

/// Samples a curve, clamping outside its range; 1 for an empty curve.
float sample_curve(const Curve& curve, float offset) {
    if (curve.count == 0) return 1.0f;
    if (offset <= curve.points[0].offset) return curve.points[0].value;
    if (offset >= curve.points[curve.count - 1].offset) return curve.points[curve.count - 1].value;
    uint32_t upper = 1;
    while (upper < curve.count && curve.points[upper].offset < offset) ++upper;
    const CurvePoint& from = curve.points[upper - 1];
    const CurvePoint& to = curve.points[upper];
    const float span = to.offset - from.offset;
    float blend = span > 0.0f ? (offset - from.offset) / span : 1.0f;
    if (curve.smooth) blend = smooth_blend(blend);
    return from.value + ((to.value - from.value) * blend);
}

/// Samples a gradient, clamping outside its range; white for an empty one.
Rgba sample_gradient(const Gradient& gradient, float offset) {
    if (gradient.count == 0) return Rgba{};
    if (offset <= gradient.stops[0].offset) return gradient.stops[0].value;
    if (offset >= gradient.stops[gradient.count - 1].offset) return gradient.stops[gradient.count - 1].value;
    uint32_t upper = 1;
    while (upper < gradient.count && gradient.stops[upper].offset < offset) ++upper;
    const Stop& from = gradient.stops[upper - 1];
    const Stop& to = gradient.stops[upper];
    const float span = to.offset - from.offset;
    const float blend = span > 0.0f ? (offset - from.offset) / span : 1.0f;
    return Rgba{mix_float(from.value.r, to.value.r, blend), mix_float(from.value.g, to.value.g, blend),
                mix_float(from.value.b, to.value.b, blend), mix_float(from.value.a, to.value.a, blend)};
}

/// The seeded random value between a parameter's minimum and maximum.
float parameter_random(const Parameter& parameter, uint32_t seed, Param id) {
    return mix_float(parameter.minimum, parameter.maximum, unit_random(seed, salt_parameter_base + static_cast<uint32_t>(id)));
}

/// A parameter at life `offset`: its seeded value scaled by its curve.
float parameter_value(const Effect& effect, Param id, uint32_t seed, float offset) {
    const Parameter& parameter = effect.parameters[id];
    return parameter_random(parameter, seed, id) * sample_curve(parameter.curve, offset);
}

/// A seeded point in the emission shape, in emitter space.
Vec3 shape_point(const Effect& effect, uint32_t seed) {
    const float a = unit_random(seed, salt_shape_a);
    const float b = unit_random(seed, salt_shape_b);
    const float c = unit_random(seed, salt_shape_c);
    if (!effect.box_shape) return Vec3{};
    return Vec3{((a * 2.0f) - 1.0f) * effect.box_extents.x, ((b * 2.0f) - 1.0f) * effect.box_extents.y,
                ((c * 2.0f) - 1.0f) * effect.box_extents.z};
}

/// A seeded unit launch direction within the effect's spread cone.
Vec3 spread_direction(const Effect& effect, uint32_t seed) {
    const float spread = radians(effect.spread_degrees);
    const float first = ((unit_random(seed, salt_direction_a) * 2.0f) - 1.0f) * spread;
    const float second = ((unit_random(seed, salt_direction_b) * 2.0f) - 1.0f) * spread * (1.0f - effect.flatness);
    const Vec3 across{sin_poly(first), 0.0f, cos_poly(first)};
    Vec3 up{0.0f, sin_poly(second), cos_poly(second)};
    up.z = up.z / max_float(0.0001f, std::sqrt(up.z < 0.0f ? -up.z : up.z));
    const Vec3 local{across.x * up.z, up.y, across.z * up.z};
    const Vec3 forward = safe_normalize(effect.direction, world_up);
    Vec3 binormal = vm::cross(world_up, forward);
    binormal = vm::length(binormal) < 0.0001f ? Vec3{0.0f, 0.0f, 1.0f} : vm::normalize(binormal);
    const Vec3 normal = vm::cross(binormal, forward);
    return safe_normalize((binormal * local.x) + (normal * local.y) + (forward * local.z), forward);
}

/// Rotates the hue of `color` by `turns` of a full circle.
Rgba rotate_hue(Rgba color, float turns) {
    if (turns == 0.0f) return color;
    const float rotation = turns * two_pi;
    const float cosine = cos_poly(rotation);
    const float sine = sin_poly(rotation);
    const float root = std::sqrt(one_third);
    const float shared = (1.0f - cosine) * one_third;
    const float a = cosine + shared;
    const float b = shared - (root * sine);
    const float c = shared + (root * sine);
    return Rgba{(color.r * a) + (color.g * b) + (color.b * c), (color.r * c) + (color.g * a) + (color.b * b),
                (color.r * b) + (color.g * c) + (color.b * a), color.a};
}

/// The hashed lattice value in [-1, 1) at an integer grid point.
float lattice(int32_t x, int32_t y, int32_t z, uint32_t salt) {
    const uint32_t hashed = mix_bits((static_cast<uint32_t>(x) * 73856093u) ^ (static_cast<uint32_t>(y) * 19349663u) ^
                                     (static_cast<uint32_t>(z) * 83492791u) ^ mix_bits(salt));
    return (static_cast<float>(hashed >> 8u) / 8388608.0f) - 1.0f;
}

/// The analytic gradient of smooth 3D value noise at `point`.
Vec3 noise_gradient(Vec3 point, uint32_t salt) {
    const float fx = std::floor(point.x);
    const float fy = std::floor(point.y);
    const float fz = std::floor(point.z);
    const auto x = static_cast<int32_t>(fx);
    const auto y = static_cast<int32_t>(fy);
    const auto z = static_cast<int32_t>(fz);
    const Vec3 f{point.x - fx, point.y - fy, point.z - fz};
    const Vec3 u{f.x * f.x * (3.0f - (2.0f * f.x)), f.y * f.y * (3.0f - (2.0f * f.y)), f.z * f.z * (3.0f - (2.0f * f.z))};
    const Vec3 du{6.0f * f.x * (1.0f - f.x), 6.0f * f.y * (1.0f - f.y), 6.0f * f.z * (1.0f - f.z)};
    const float a = lattice(x, y, z, salt);
    const float b = lattice(x + 1, y, z, salt);
    const float c = lattice(x, y + 1, z, salt);
    const float d = lattice(x + 1, y + 1, z, salt);
    const float e = lattice(x, y, z + 1, salt);
    const float g = lattice(x + 1, y, z + 1, salt);
    const float h = lattice(x, y + 1, z + 1, salt);
    const float k = lattice(x + 1, y + 1, z + 1, salt);
    const float k1 = b - a;
    const float k2 = c - a;
    const float k3 = e - a;
    const float k4 = a - b - c + d;
    const float k5 = a - c - e + h;
    const float k6 = a - b - e + g;
    const float k7 = -a + b + c - d + e - g - h + k;
    return Vec3{du.x * (k1 + (k4 * u.y) + (k6 * u.z) + (k7 * u.y * u.z)), du.y * (k2 + (k5 * u.z) + (k4 * u.x) + (k7 * u.z * u.x)),
                du.z * (k3 + (k6 * u.x) + (k5 * u.y) + (k7 * u.x * u.y))};
}

/// A divergence-free vector from the curl of three noise fields.
Vec3 curl_noise(Vec3 point) {
    const Vec3 first = noise_gradient(point, 1u);
    const Vec3 second = noise_gradient(point, 2u);
    const Vec3 third = noise_gradient(point, 3u);
    return Vec3{third.y - second.z, first.z - third.x, second.x - first.y};
}

/// Axes of `basis` normalised, removing scale.
Basis rotation_of(const Basis& basis) {
    return Basis{safe_normalize(basis.x, Vec3{1.0f, 0.0f, 0.0f}), safe_normalize(basis.y, Vec3{0.0f, 1.0f, 0.0f}),
                 safe_normalize(basis.z, Vec3{0.0f, 0.0f, 1.0f})};
}

/// Updates a particle's colour, size and angle from its age, ramps, curves and tint.
void apply_display(Particle& particle, const Context& context) {
    const Effect& effect = *context.effect;
    const float offset = clamp_float(particle.age / particle.lifetime, 0.0f, 1.0f);
    const uint32_t seed = particle.seed;
    Rgba color = effect.color;
    if (effect.color_initial_ramp.count != 0) {
        const Rgba initial = sample_gradient(effect.color_initial_ramp, unit_random(seed, salt_initial_ramp));
        color = Rgba{color.r * initial.r, color.g * initial.g, color.b * initial.b, color.a * initial.a};
    }
    if (effect.color_ramp.count != 0) {
        const Rgba ramp = sample_gradient(effect.color_ramp, offset);
        color = Rgba{color.r * ramp.r, color.g * ramp.g, color.b * ramp.b, color.a * ramp.a};
    }
    color.a *= sample_curve(effect.alpha_curve, offset);
    color.a = clamp_float(color.a, 0.0f, 1.0f);
    color = rotate_hue(color, parameter_value(effect, hue_variation, seed, offset));
    color = Rgba{color.r * particle.tint.r, color.g * particle.tint.g, color.b * particle.tint.b, color.a};
    particle.color = color;
    particle.size = effect.draw_size * max_float(parameter_value(effect, scale, seed, offset), 0.0f);
    particle.angle = radians(parameter_value(effect, angle, seed, offset)) + particle.spin;
}

/// Initialises `particle` from `seed` at the emission transform; false when
/// the amount ratio culls it.
bool spawn(Particle& particle, uint32_t seed, const Context& context) {
    const Effect& effect = *context.effect;
    particle = Particle{};
    particle.seed = seed;
    particle.tint = context.tint;
    if (context.amount_ratio < 1.0f && unit_random(seed, salt_ratio) >= context.amount_ratio) return false;
    particle.lifetime =
        max_float(effect.lifetime * (1.0f - (effect.lifetime_randomness * unit_random(seed, salt_lifetime))), minimum_lifetime);
    const Vec3 local_position = shape_point(effect, seed) + effect.offset;
    const Vec3 local_velocity = spread_direction(effect, seed) * parameter_random(effect.parameters[initial_velocity], seed, initial_velocity);
    const Basis rotation = rotation_of(context.basis);
    particle.position = context.basis * local_position + context.origin;
    particle.velocity = (rotation * local_velocity) + (context.emitter_velocity * effect.inherit_velocity);
    particle.origin = context.origin;
    particle.axis = rotation.y;
    particle.active = true;
    apply_display(particle, context);
    return true;
}

/// Clips the segment's parameter range against one slab of a box; false
/// when the segment misses it.
bool clip_axis(Clip& clip, float start, float delta, float low, float high, int axis) {
    const float magnitude = delta < 0.0f ? -delta : delta;
    if (magnitude < 1e-9f) return !(start < low || start >= high);
    float near = (low - start) / delta;
    float far = (high - start) / delta;
    if (near > far) std::swap(near, far);
    if (near > clip.enter) {
        clip.enter = near;
        clip.axis = axis;
    }
    clip.exit = min_float(clip.exit, far);
    return true;
}

/// The surface normal where a segment enters a box along `axis`, or against
/// the segment when it starts inside.
Vec3 contact_normal(int axis, Vec3 delta) {
    Vec3 local{};
    if (axis < 0) {
        local = -delta;
    } else {
        const float sign = component(delta, axis) > 0.0f ? -1.0f : 1.0f;
        local = axis == 0 ? Vec3{sign, 0.0f, 0.0f} : (axis == 1 ? Vec3{0.0f, sign, 0.0f} : Vec3{0.0f, 0.0f, sign});
    }
    const float size = vm::length(local);
    return size > 1e-9f ? local / size : Vec3{0.0f, 1.0f, 0.0f};
}

/// The nearest point where the segment from `from` to `to` enters a box.
bool sweep(const Scene& scene, Vec3 from, Vec3 to, Contact& contact) {
    const vm::Aabb segment{vm::min(from, to), vm::max(from, to)};
    const Vec3 delta = to - from;
    float best = 1e30f;
    bool hit = false;
    for (const vm::Aabb& box : scene.boxes) {
        if (!vm::overlaps(box, segment)) continue;
        Clip clip;
        const bool crosses = clip_axis(clip, from.x, delta.x, box.min.x, box.max.x, 0) &&
                             clip_axis(clip, from.y, delta.y, box.min.y, box.max.y, 1) &&
                             clip_axis(clip, from.z, delta.z, box.min.z, box.max.z, 2);
        if (!crosses || clip.enter > clip.exit || clip.enter >= best) continue;
        best = clip.enter;
        contact = Contact{lerp(from, to, clip.enter), contact_normal(clip.axis, delta)};
        hit = true;
    }
    return hit;
}

/// Advances a particle by `step`: forces, wind, damping, turbulence,
/// controlled velocity and collision.
Outcome integrate(const Scene& scene, Particle& particle, float step, const Context& context) {
    const Effect& effect = *context.effect;
    Outcome outcome;
    particle.age += step;
    if (particle.age >= particle.lifetime) {
        outcome.died = true;
        outcome.where = particle.position;
        outcome.velocity = particle.velocity;
        particle.active = false;
        return outcome;
    }
    const float offset = particle.age / particle.lifetime;
    const uint32_t seed = particle.seed;
    const Vec3 diff = particle.position - particle.origin;
    const Vec3 radial = safe_normalize(diff, Vec3{});
    const Vec3 tangent = safe_normalize(vm::cross(particle.axis, diff), Vec3{});
    const Vec3 heading = safe_normalize(particle.velocity, Vec3{});
    const float linear = parameter_value(effect, linear_acceleration, seed, offset);
    const float radial_force = parameter_value(effect, radial_acceleration, seed, offset);
    const float tangential = parameter_value(effect, tangential_acceleration, seed, offset);
    const Vec3 force = effect.gravity + (heading * linear) + (radial * radial_force) + (tangent * tangential);
    particle.velocity = particle.velocity + (force * step);
    if (effect.wind_influence > 0.0f) {
        const float rate = effect.wind_influence * wind_response * step;
        const float pull = rate / (1.0f + rate);
        particle.velocity.x += (context.wind.x - particle.velocity.x) * pull;
        particle.velocity.z += (context.wind.z - particle.velocity.z) * pull;
        particle.velocity.y += context.wind.y * effect.wind_influence * wind_response * step;
    }
    const float drag = parameter_value(effect, damping, seed, offset);
    if (drag > 0.0f) {
        const float speed = vm::length(particle.velocity);
        if (speed > 0.0f) particle.velocity = particle.velocity * (max_float(speed - (drag * step), 0.0f) / speed);
    }
    if (effect.turbulence_enabled) {
        const float rate =
            1.0f + (effect.turbulence_speed_random * ((unit_random(seed, salt_turbulence_speed) * 2.0f) - 1.0f));
        const Vec3 sample_point =
            (particle.position / max_float(effect.turbulence_scale, 1e-3f)) + (effect.turbulence_speed * (context.time * rate));
        const Vec3 direction = safe_normalize(curl_noise(sample_point), Vec3{});
        const float influence = clamp_float(parameter_value(effect, turbulence_influence, seed, offset), 0.0f, 1.0f);
        const float blend = influence * (step * turbulence_reference_rate);
        const float speed = vm::length(particle.velocity);
        particle.velocity = lerp(particle.velocity, direction * speed, blend) + (direction * (effect.turbulence_strength * step));
    }
    const float radial_speed = parameter_value(effect, radial_velocity, seed, offset);
    const float orbit = parameter_value(effect, orbit_velocity, seed, offset);
    const Vec3 controlled = (radial * radial_speed) + (vm::cross(particle.axis, diff) * (orbit * two_pi));
    const Vec3 from = particle.position;
    const Vec3 to = from + ((particle.velocity + controlled) * step);
    particle.position = to;
    Contact contact;
    if (effect.collision != Collision::Disabled && sweep(scene, from, to, contact)) {
        outcome.where = contact.position;
        Vec3 velocity = particle.velocity;
        if (effect.collision == Collision::HideOnContact) {
            outcome.collided = true;
            outcome.velocity = velocity;
            particle.active = false;
            outcome.died = true;
            return outcome;
        }
        const float radius = effect.use_scale ? particle.size * 0.5f : 0.0f;
        const Vec3 rest = contact.position + (contact.normal * (radius + contact_offset));
        const Vec3 normal = contact.normal;
        const float into = vm::dot(velocity, normal);
        outcome.collided = -into > impact_speed;
        if (into < 0.0f) {
            const Vec3 slide = velocity - (normal * into);
            velocity = (slide * (1.0f - effect.friction)) - (normal * (into * effect.bounce));
        }
        particle.position = rest;
        particle.velocity = velocity;
        outcome.velocity = velocity;
    }
    const float spin = parameter_value(effect, angular_velocity, seed, offset);
    particle.spin += radians(spin) * step;
    apply_display(particle, context);
    return outcome;
}

/// The next inactive particle from the emit cursor, advancing the cursor;
/// null when the pool is full.
Particle* free_particle(Emitter& emitter, Particle* pool, uint32_t count) {
    for (uint32_t probe = 0; probe < count; ++probe) {
        const uint32_t index = (emitter.emit_cursor + probe) % count;
        if (!pool[index].active) {
            emitter.emit_cursor = (index + 1) % count;
            return &pool[index];
        }
    }
    return nullptr;
}

/// Spawns the parent's sub emitter particles at `origin` into free sub slots.
void emit_sub(Emitter& emitter, Particle* sub_pool, const Effect& parent, const Effect& sub, const SubOrigin& origin) {
    if (emitter.sub_count == 0) return;
    Context context;
    context.effect = &sub;
    context.origin = origin.position;
    context.time = emitter.elapsed;
    const uint32_t pool = emitter.sub_count;
    for (uint32_t count = 0; count < parent.sub_amount; ++count) {
        uint32_t found = pool;
        for (uint32_t probe = 0; probe < pool; ++probe) {
            const uint32_t index = (emitter.sub_cursor + probe) % pool;
            if (!sub_pool[index].active) {
                found = index;
                break;
            }
        }
        if (found == pool) return;
        emitter.sub_cursor = (found + 1) % pool;
        const uint32_t number = 0xB5297A4Du + emitter.bursts;
        emitter.bursts += 1;
        const uint32_t seed = mix_bits(emitter.current_seed ^ mix_bits(number));
        Particle& particle = sub_pool[found];
        if (spawn(particle, seed, context) && parent.sub_keep_velocity) particle.velocity = particle.velocity + origin.velocity;
    }
}

/// Counts a collision and spawns sub emitter particles by the sub emitter mode.
void react(Emitter& emitter, Particle* sub_pool, uint64_t& collisions, const Effect& parent, const Effect* sub,
           Particle& particle, const Outcome& outcome, float step_seconds) {
    if (outcome.collided) ++collisions;
    if (sub == nullptr) return;
    switch (parent.sub_mode) {
    case SubMode::AtEnd:
        if (outcome.died && !outcome.collided) emit_sub(emitter, sub_pool, parent, *sub, SubOrigin{outcome.where, outcome.velocity});
        break;
    case SubMode::AtCollision:
        if (outcome.collided) emit_sub(emitter, sub_pool, parent, *sub, SubOrigin{outcome.where, outcome.velocity});
        break;
    case SubMode::Constant: {
        if (!particle.active) break;
        const float interval = 1.0f / max_float(parent.sub_frequency, 1e-3f);
        particle.sub_timer += step_seconds;
        while (particle.sub_timer >= interval) {
            particle.sub_timer -= interval;
            emit_sub(emitter, sub_pool, parent, *sub, SubOrigin{particle.position, particle.velocity});
        }
        break;
    }
    case SubMode::Disabled:
        break;
    }
}

/// A seed for one numbered emission of an emitter.
uint32_t emission_seed(const Emitter& emitter, uint32_t number) { return mix_bits(emitter.current_seed ^ mix_bits(number)); }

/// Emits the particles whose restart phase falls inside one phase interval.
void emit_interval(const Scene& scene, Emitter& emitter, Pools pools, uint64_t& collisions, const Context& context,
                   const Effect* sub, float from, float to, float remaining, uint64_t cycle, float step_seconds) {
    const Effect& effect = *context.effect;
    const uint32_t amount = pool_size;
    const float lifetime = max_float(effect.lifetime, minimum_lifetime);
    for (uint32_t index = 0; index < amount; ++index) {
        const uint32_t number = static_cast<uint32_t>((cycle * amount) + index);
        const uint32_t seed = emission_seed(emitter, number);
        float restart_phase = static_cast<float>(index) / static_cast<float>(amount);
        if (effect.randomness > 0.0f) restart_phase += effect.randomness * unit_random(seed, salt_phase) / static_cast<float>(amount);
        restart_phase *= 1.0f - effect.explosiveness;
        if (restart_phase < from || restart_phase >= to) continue;
        Particle& particle = pools.main[index];
        if (!spawn(particle, seed, context)) continue;
        const float local_delta = clamp_float((remaining - restart_phase) * lifetime, 0.0f, step_seconds);
        const Outcome outcome = integrate(scene, particle, local_delta, context);
        react(emitter, pools.sub, collisions, effect, sub, particle, outcome, local_delta);
    }
}

/// Advances one emitter by `step_seconds`: integrates particles, emits by
/// phase, then spawns bursts.
void step(const Scene& scene, Emitter& emitter, Pools pools, uint64_t& collisions, float step_seconds, Vec3 origin) {
    const Effect& effect = scene.effects[emitter.effect];
    const Effect* sub = emitter.sub_effect == no_effect ? nullptr : &scene.effects[emitter.sub_effect];
    Context context;
    context.effect = &effect;
    context.basis = emitter.basis;
    context.origin = origin;
    context.emitter_velocity = emitter.velocity;
    context.time = emitter.elapsed;
    context.amount_ratio = clamp_float(emitter.amount_ratio, 0.0f, 1.0f);
    context.wind = scene.wind;
    context.tint = emitter.tint;
    for (uint32_t index = 0; index < pool_size; ++index) {
        Particle& particle = pools.main[index];
        if (!particle.active) continue;
        const Outcome outcome = integrate(scene, particle, step_seconds, context);
        react(emitter, pools.sub, collisions, effect, sub, particle, outcome, step_seconds);
    }
    if (sub != nullptr) {
        Context sub_context;
        sub_context.effect = sub;
        sub_context.time = emitter.elapsed;
        sub_context.wind = scene.wind;
        sub_context.tint = emitter.tint;
        for (uint32_t index = 0; index < emitter.sub_count; ++index) {
            Particle& particle = pools.sub[index];
            if (!particle.active) continue;
            const Outcome outcome = integrate(scene, particle, step_seconds, sub_context);
            if (outcome.collided) ++collisions;
        }
    }
    const float lifetime = max_float(effect.lifetime, minimum_lifetime);
    if (emitter.emitting) {
        const float previous = emitter.phase;
        float phase = previous + (step_seconds / lifetime);
        const bool wrapped = phase >= 1.0f;
        if (wrapped) phase = phase - std::floor(phase);
        if (!wrapped) {
            emit_interval(scene, emitter, pools, collisions, context, sub, previous, phase, phase, emitter.cycle, step_seconds);
        } else {
            emit_interval(scene, emitter, pools, collisions, context, sub, previous, 1.0f, 1.0f + phase, emitter.cycle, step_seconds);
            if (!effect.one_shot) {
                emit_interval(scene, emitter, pools, collisions, context, sub, 0.0f, phase, phase, emitter.cycle + 1, step_seconds);
            }
        }
        emitter.phase = phase;
        if (wrapped) {
            ++emitter.cycle;
            if (effect.one_shot) {
                emitter.emitting = false;
                emitter.phase = 0.0f;
            }
        }
    }
    Context burst_context = context;
    burst_context.amount_ratio = 1.0f;
    while (emitter.burst_pending > 0) {
        --emitter.burst_pending;
        Particle* slot = free_particle(emitter, pools.main, pool_size);
        if (slot == nullptr) {
            emitter.burst_pending = 0;
            break;
        }
        const uint32_t number = 0x68E31DA4u + emitter.bursts;
        emitter.bursts += 1;
        static_cast<void>(spawn(*slot, emission_seed(emitter, number), burst_context));
    }
    emitter.elapsed += step_seconds;
}

/// Counts the live particles of an emitter.
void finish(Emitter& emitter, const Pools& pools) {
    uint32_t alive = 0;
    for (uint32_t index = 0; index < pool_size; ++index) alive += pools.main[index].active ? 1u : 0u;
    for (uint32_t index = 0; index < emitter.sub_count; ++index) alive += pools.sub[index].active ? 1u : 0u;
    emitter.alive = alive;
}

/// A curve from its points.
Curve make_curve(bool smooth, std::initializer_list<CurvePoint> points) {
    Curve curve;
    curve.smooth = smooth;
    for (const CurvePoint& point : points) curve.points[curve.count++] = point;
    return curve;
}

/// A gradient from its stops.
Gradient make_gradient(std::initializer_list<Stop> stops) {
    Gradient gradient;
    for (const Stop& stop : stops) gradient.stops[gradient.count++] = stop;
    return gradient;
}

/// Sets one parameter of an effect.
void set_parameter(Effect& effect, Param id, float minimum, float maximum, Curve curve = {}) {
    effect.parameters[id] = Parameter{minimum, maximum, curve};
}

/// A shower of bouncing sparks that leave dust where they land.
Effect make_sparks() {
    Effect e;
    e.lifetime = 1.6f;
    e.lifetime_randomness = 0.4f;
    e.randomness = 0.5f;
    e.box_shape = true;
    e.box_extents = Vec3{0.2f, 0.05f, 0.2f};
    e.spread_degrees = 35.0f;
    e.flatness = 0.2f;
    e.inherit_velocity = 0.5f;
    e.wind_influence = 0.1f;
    e.draw_size = 0.15f;
    e.color = Rgba{1.0f, 0.6f, 0.3f, 1.0f};
    e.color_ramp = make_gradient({{0.0f, {1.0f, 1.0f, 0.8f, 1.0f}}, {0.4f, {1.0f, 0.5f, 0.2f, 1.0f}}, {1.0f, {0.3f, 0.1f, 0.1f, 0.0f}}});
    e.alpha_curve = make_curve(false, {{0.0f, 0.0f}, {0.1f, 1.0f}, {1.0f, 0.0f}});
    set_parameter(e, initial_velocity, 4.0f, 9.0f);
    set_parameter(e, linear_acceleration, 0.0f, 1.0f, make_curve(false, {{0.0f, 1.0f}, {1.0f, 0.0f}}));
    set_parameter(e, damping, 0.2f, 0.6f, make_curve(true, {{0.0f, 1.0f}, {1.0f, 0.2f}}));
    set_parameter(e, angle, 0.0f, 360.0f);
    set_parameter(e, angular_velocity, -90.0f, 90.0f);
    set_parameter(e, scale, 0.6f, 1.2f, make_curve(false, {{0.0f, 0.2f}, {0.2f, 1.0f}, {1.0f, 0.1f}}));
    set_parameter(e, hue_variation, -0.05f, 0.05f);
    e.collision = Collision::Rigid;
    e.friction = 0.3f;
    e.bounce = 0.5f;
    e.use_scale = true;
    e.sub_mode = SubMode::AtCollision;
    e.sub_amount = 2;
    e.sub_effect = 4;
    return e;
}

/// Buoyant smoke pushed by wind and curl noise that vanishes on contact.
Effect make_smoke() {
    Effect e;
    e.lifetime = 2.5f;
    e.lifetime_randomness = 0.3f;
    e.randomness = 1.0f;
    e.box_shape = true;
    e.box_extents = Vec3{0.5f, 0.1f, 0.5f};
    e.spread_degrees = 25.0f;
    e.gravity = Vec3{0.0f, 0.6f, 0.0f};
    e.wind_influence = 0.8f;
    e.draw_size = 0.8f;
    e.color = Rgba{0.5f, 0.5f, 0.55f, 1.0f};
    e.color_initial_ramp = make_gradient({{0.0f, {1.0f, 0.9f, 0.8f, 1.0f}}, {1.0f, {0.8f, 0.9f, 1.0f, 0.7f}}});
    e.color_ramp = make_gradient({{0.0f, {1.0f, 1.0f, 1.0f, 1.0f}}, {1.0f, {0.4f, 0.4f, 0.4f, 1.0f}}});
    e.alpha_curve = make_curve(true, {{0.0f, 0.0f}, {0.2f, 0.6f}, {1.0f, 0.0f}});
    set_parameter(e, initial_velocity, 0.5f, 1.5f);
    set_parameter(e, radial_acceleration, -0.2f, 0.2f);
    set_parameter(e, tangential_acceleration, 0.5f, 1.0f);
    set_parameter(e, damping, 0.3f, 0.8f);
    set_parameter(e, scale, 1.0f, 2.5f, make_curve(true, {{0.0f, 0.3f}, {1.0f, 1.0f}}));
    set_parameter(e, turbulence_influence, 0.3f, 0.7f, make_curve(false, {{0.0f, 0.2f}, {1.0f, 1.0f}}));
    e.turbulence_enabled = true;
    e.turbulence_strength = 1.5f;
    e.turbulence_scale = 3.0f;
    e.collision = Collision::HideOnContact;
    return e;
}

/// Debris that erupts at once, tumbles, and bursts into embers where it ends.
Effect make_debris() {
    Effect e;
    e.lifetime = 2.0f;
    e.randomness = 0.3f;
    e.explosiveness = 0.7f;
    e.box_shape = true;
    e.box_extents = Vec3{0.4f, 0.4f, 0.4f};
    e.spread_degrees = 60.0f;
    e.draw_size = 0.3f;
    e.color = Rgba{0.6f, 0.5f, 0.4f, 1.0f};
    e.color_ramp = make_gradient({{0.0f, {1.0f, 1.0f, 1.0f, 1.0f}}, {1.0f, {0.5f, 0.5f, 0.5f, 1.0f}}});
    set_parameter(e, initial_velocity, 3.0f, 8.0f);
    set_parameter(e, orbit_velocity, 0.0f, 0.1f);
    set_parameter(e, angle, 0.0f, 360.0f);
    set_parameter(e, angular_velocity, -180.0f, 180.0f);
    set_parameter(e, scale, 0.5f, 1.5f);
    e.collision = Collision::Rigid;
    e.friction = 0.5f;
    e.bounce = 0.4f;
    e.sub_mode = SubMode::AtEnd;
    e.sub_amount = 3;
    e.sub_keep_velocity = true;
    e.sub_effect = 5;
    return e;
}

/// A fountain that bounces and sheds dust as it flies.
Effect make_fountain() {
    Effect e;
    e.lifetime = 1.2f;
    e.lifetime_randomness = 0.2f;
    e.spread_degrees = 12.0f;
    e.draw_size = 0.2f;
    e.color = Rgba{0.5f, 0.7f, 1.0f, 1.0f};
    e.alpha_curve = make_curve(false, {{0.0f, 1.0f}, {0.8f, 1.0f}, {1.0f, 0.0f}});
    set_parameter(e, initial_velocity, 8.0f, 12.0f);
    set_parameter(e, radial_velocity, 0.0f, 0.5f);
    e.collision = Collision::Rigid;
    e.friction = 0.1f;
    e.bounce = 0.6f;
    e.sub_mode = SubMode::Constant;
    e.sub_frequency = 6.0f;
    e.sub_amount = 1;
    e.sub_keep_velocity = true;
    e.sub_effect = 4;
    return e;
}

/// Short-lived dust puffs.
Effect make_dust() {
    Effect e;
    e.amount = sub_pool_size;
    e.lifetime = 0.4f;
    e.spread_degrees = 80.0f;
    e.gravity = Vec3{0.0f, -1.0f, 0.0f};
    e.draw_size = 0.1f;
    e.color = Rgba{0.7f, 0.65f, 0.6f, 1.0f};
    e.alpha_curve = make_curve(false, {{0.0f, 1.0f}, {1.0f, 0.0f}});
    set_parameter(e, initial_velocity, 0.5f, 2.0f);
    return e;
}

/// Embers that bounce once.
Effect make_ember() {
    Effect e;
    e.amount = sub_pool_size;
    e.lifetime = 0.6f;
    e.spread_degrees = 80.0f;
    e.gravity = Vec3{0.0f, -4.0f, 0.0f};
    e.draw_size = 0.08f;
    e.color = Rgba{1.0f, 0.4f, 0.1f, 1.0f};
    set_parameter(e, initial_velocity, 1.0f, 3.0f);
    e.collision = Collision::Rigid;
    e.bounce = 0.3f;
    return e;
}

/// The six effects: four emitters' effects and their two sub effects.
Scene make_scene(Rng& rng) {
    Scene scene;
    scene.effects = {make_sparks(), make_smoke(), make_debris(), make_fountain(), make_dust(), make_ember()};
    for (uint32_t i = 0; i < 4; ++i) scene.effects[i].amount = pool_size;
    scene.wind = Vec3{3.0f, 0.0f, 1.0f};
    scene.boxes[0] = vm::Aabb{Vec3{-1000.0f, -1000.0f, -1000.0f}, Vec3{1000.0f, 0.0f, 1000.0f}};
    for (uint32_t i = 1; i < box_count; ++i) {
        const Vec3 where = vm::random_vec3(rng, 0.0f, 1.0f);
        const Vec3 half = vm::random_vec3(rng, 0.5f, 2.0f);
        const Vec3 centre{where.x * 40.0f, half.y, where.z * 40.0f};
        scene.boxes[i] = vm::Aabb{centre - half, centre + half};
    }
    return scene;
}

/// An emitter at a random place and orientation, moving at a random velocity.
Emitter make_emitter(uint32_t index, const Scene& scene, Rng& rng) {
    Emitter emitter;
    emitter.effect = index % 4;
    emitter.sub_effect = scene.effects[emitter.effect].sub_effect;
    emitter.first = index * pool_size;
    emitter.sub_first = index * sub_pool_size;
    emitter.sub_count = emitter.sub_effect == no_effect ? 0 : scene.effects[emitter.sub_effect].amount;
    emitter.basis = vm::basis_from_quat(vm::random_quat(rng));
    const Vec3 where = vm::random_vec3(rng, 0.0f, 1.0f);
    emitter.origin = Vec3{where.x * 40.0f, 2.0f + (where.y * 4.0f), where.z * 40.0f};
    emitter.move = vm::random_vec3(rng, -3.0f, 3.0f);
    const float red = 0.6f + (rng.unit() * 0.4f);
    const float green = 0.6f + (rng.unit() * 0.4f);
    const float blue = 0.6f + (rng.unit() * 0.4f);
    emitter.tint = Rgba{red, green, blue, 1.0f};
    emitter.amount_ratio = 0.8f + (rng.unit() * 0.2f);
    emitter.speed_scale = 0.9f + (rng.unit() * 0.2f);
    emitter.current_seed = mix_bits((index * golden) ^ mix_bits(1u));
    return emitter;
}

/// Folds one particle's state into a 64-bit word.
uint64_t fold_particle(const Particle& p) {
    const auto pair = [](float low, float high) { return static_cast<uint64_t>(f32_bits(low)) | (static_cast<uint64_t>(f32_bits(high)) << 32); };
    uint64_t word = pair(p.position.x, p.position.y);
    word ^= pair(p.position.z, p.velocity.x) * 0x9E3779B97F4A7C15ull;
    word ^= pair(p.velocity.y, p.velocity.z) * 0xC2B2AE3D27D4EB4Full;
    word ^= pair(p.size, p.color.r) * 0x165667B19E3779F9ull;
    word ^= pair(p.color.g, p.color.b) * 0x85EBCA77C2B2AE63ull;
    word ^= pair(p.color.a, p.active ? 1.0f : 0.0f) * 0x27D4EB2F165667C5ull;
    return word;
}

} // namespace

struct Particles::State {
    Scene scene{};
    uint64_t collisions{};
    std::vector<Emitter> emitters;
    std::vector<Emitter> initial_emitters;
    std::vector<Particle> particles;
    std::vector<Particle> initial_particles;
    std::vector<Particle> sub_particles;
    std::vector<Particle> initial_sub_particles;

    /// The particle pools of emitter `index`.
    Pools pools(uint32_t index) {
        const Emitter& emitter = emitters[index];
        return Pools{particles.data() + emitter.first, sub_particles.data() + emitter.sub_first};
    }

    /// Runs one emitter's preprocess steps from a fresh start.
    void preprocess(uint32_t index) {
        Emitter& emitter = emitters[index];
        const float step_seconds = preprocess_seconds / static_cast<float>(preprocess_steps);
        for (uint32_t count = 0; count < preprocess_steps; ++count) {
            step(scene, emitter, pools(index), collisions, step_seconds, emitter.origin);
        }
    }

    /// Advances every emitter by one update in shared substeps.
    void advance() {
        std::array<Vec3, emitter_count> start{};
        std::array<Vec3, emitter_count> end{};
        std::array<float, emitter_count> scaled{};
        for (uint32_t index = 0; index < emitter_count; ++index) {
            Emitter& emitter = emitters[index];
            start[index] = emitter.origin;
            end[index] = emitter.origin + (emitter.move * update_seconds);
            emitter.origin = end[index];
            emitter.velocity = (end[index] - start[index]) / update_seconds;
            scaled[index] = update_seconds * max_float(emitter.speed_scale, 0.0f);
        }
        for (uint32_t sub = 0; sub < substeps; ++sub) {
            const float fraction = static_cast<float>(sub + 1) / static_cast<float>(substeps);
            for (uint32_t index = 0; index < emitter_count; ++index) {
                step(scene, emitters[index], pools(index), collisions, scaled[index] / static_cast<float>(substeps),
                     lerp(start[index], end[index], fraction));
            }
        }
        for (uint32_t index = 0; index < emitter_count; ++index) finish(emitters[index], pools(index));
    }
};

Particles::Particles() : state_(std::make_unique<State>()) {
    State& s = *state_;
    Rng rng{0x9a7};
    s.scene = make_scene(rng);
    s.particles.assign(static_cast<size_t>(emitter_count) * pool_size, Particle{});
    s.sub_particles.assign(static_cast<size_t>(emitter_count) * sub_pool_size, Particle{});
    for (uint32_t index = 0; index < emitter_count; ++index) s.emitters.push_back(make_emitter(index, s.scene, rng));
    for (uint32_t index = 0; index < emitter_count; ++index) s.preprocess(index);
    s.initial_emitters = s.emitters;
    s.initial_particles = s.particles;
    s.initial_sub_particles = s.sub_particles;
}

Particles::~Particles() = default;

uint64_t Particles::run() {
    State& s = *state_;
    s.emitters = s.initial_emitters;
    std::copy(s.initial_particles.begin(), s.initial_particles.end(), s.particles.begin());
    std::copy(s.initial_sub_particles.begin(), s.initial_sub_particles.end(), s.sub_particles.begin());
    s.collisions = 0;
    for (uint32_t update = 0; update < updates; ++update) {
        if (update % 2 == 0) {
            for (uint32_t index = 0; index < emitter_count; index += 3) s.emitters[index].burst_pending += 256;
        }
        s.advance();
    }
    uint64_t h = hash_add(0, s.collisions);
    for (const Emitter& emitter : s.emitters) h = hash_add(h, emitter.alive);
    for (const Particle& p : s.particles) h = hash_add(h, fold_particle(p));
    for (const Particle& p : s.sub_particles) h = hash_add(h, fold_particle(p));
    return h;
}

} // namespace bench
