#include "animation_clip.hpp"

#include <cmath>
#include <optional>

namespace bench::anim {

namespace {

constexpr float sine_quarter_turn = 1.57079633f;
constexpr float sine_fourth_term = 0.0416666667f;
constexpr float sine_sixth_term = 0.00138888889f;
constexpr float back_overshoot = 1.70158f;

/// The first key strictly after `time`, or the key count when none is.
size_t upper_key(const Track& track, float time) {
    size_t index = 0;
    while (index < track.keys.size() && track.keys[index].time <= time) ++index;
    return index;
}

/// The pair of keys surrounding a sample time and the eased blend between them.
struct KeySpan {
    size_t before{};
    size_t after{};
    float blend{};
};

/// The ease-in form of a transition curve at `t` in [0, 1]. Sine and expo are
/// polynomial stand-ins for the engine's cosine and power.
float ease_in_curve(Transition transition, float t) {
    switch (transition) {
    case Transition::Sine: {
        const float x = t * sine_quarter_turn;
        const float x2 = x * x;
        return x2 * (0.5f - x2 * (sine_fourth_term - x2 * sine_sixth_term));
    }
    case Transition::Quad:
        return t * t;
    case Transition::Cubic:
        return t * t * t;
    case Transition::Expo: {
        const float t2 = t * t;
        const float t4 = t2 * t2;
        const float t8 = t4 * t4;
        return t <= 0.0f ? 0.0f : t8 * t2;
    }
    case Transition::Circ:
        return 1.0f - std::sqrt(max_float(1.0f - (t * t), 0.0f));
    case Transition::Back:
        return t * t * (((back_overshoot + 1.0f) * t) - back_overshoot);
    case Transition::Linear:
        break;
    }
    return t;
}

/// The ease-out form of a transition curve, the mirror of ease-in.
float ease_out_curve(Transition transition, float t) {
    return 1.0f - ease_in_curve(transition, 1.0f - t);
}

/// `value`, negated when its dot with `reference` is negative.
vm::Quat aligned(vm::Quat reference, vm::Quat value) {
    if (vm::dot(reference, value) < 0.0f) return vm::Quat{-value.x, -value.y, -value.z, -value.w};
    return value;
}

/// One component of a Catmull-Rom segment from `start` to `end` at `t`.
float catmull_rom(float before, float start, float end, float after, float t) {
    const float t2 = t * t;
    const float t3 = t2 * t;
    return 0.5f * (((2.0f * start) + ((end - before) * t) + (((2.0f * before) - (5.0f * start) + (4.0f * end) - after) * t2) +
                    ((-before + (3.0f * start) - (3.0f * end) + after) * t3)));
}

/// The key `step` places from `index`, clamped to the track.
size_t neighbour(const Track& track, size_t index, int step) {
    const auto count = static_cast<std::ptrdiff_t>(track.keys.size());
    const auto wanted = static_cast<std::ptrdiff_t>(index) + step;
    const std::ptrdiff_t last = count > 0 ? count - 1 : 0;
    return static_cast<size_t>(wanted < 0 ? 0 : (last < wanted ? last : wanted));
}

/// The keys surrounding `time` and the eased blend between them; times
/// outside the keys clamp to the first or last key.
std::optional<KeySpan> span_at(const Track& track, float time) {
    if (track.keys.empty()) return std::nullopt;
    const size_t next = upper_key(track, time);
    if (next == 0) return KeySpan{0, 0, 0.0f};
    if (next >= track.keys.size()) {
        const size_t last = track.keys.size() - 1;
        return KeySpan{last, last, 0.0f};
    }
    const float start = track.keys[next - 1].time;
    const float end = track.keys[next].time;
    const float width = end - start;
    const float blend = width > 0.0f ? (time - start) / width : 0.0f;
    return KeySpan{next - 1, next, ease_curve(clamp_float(blend, 0.0f, 1.0f), track.keys[next - 1].easing)};
}

/// Appends the indices of keys with `from < time <= to`, ascending.
void forward(const Track& track, float from, float to, std::vector<size_t>& out) {
    for (size_t index = 0; index < track.keys.size(); ++index) {
        const float at = track.keys[index].time;
        if (at > from && at <= to) out.push_back(index);
    }
}

/// Appends the indices of keys with `to <= time < from`, descending.
void backward(const Track& track, float from, float to, std::vector<size_t>& out) {
    for (size_t index = track.keys.size(); index > 0; --index) {
        const float at = track.keys[index - 1].time;
        if (at < from && at >= to) out.push_back(index - 1);
    }
}

} // namespace

vm::Vec3 track_rest_vector(TrackKind kind) {
    return kind == TrackKind::Scale ? vm::Vec3{1.0f, 1.0f, 1.0f} : vm::Vec3{};
}

float ease_curve(float blend, Easing easing) {
    const float t = clamp_float(blend, 0.0f, 1.0f);
    if (easing.transition == Transition::Linear) return t;
    switch (easing.ease) {
    case Ease::In:
        return ease_in_curve(easing.transition, t);
    case Ease::Out:
        return ease_out_curve(easing.transition, t);
    case Ease::InOut:
        return t < 0.5f ? ease_in_curve(easing.transition, t * 2.0f) * 0.5f
                        : 0.5f + (ease_out_curve(easing.transition, (t * 2.0f) - 1.0f) * 0.5f);
    case Ease::OutIn:
        break;
    }
    return t < 0.5f ? ease_out_curve(easing.transition, t * 2.0f) * 0.5f
                    : 0.5f + (ease_in_curve(easing.transition, (t * 2.0f) - 1.0f) * 0.5f);
}

vm::Vec3 sample_vector(const Track& track, float time, vm::Vec3 fallback) {
    const std::optional<KeySpan> span = span_at(track, time);
    if (!span.has_value()) return fallback;
    const vm::Vec3 before = track.keys[span->before].vector;
    if (track.interpolation == Interpolation::Nearest || span->before == span->after) return before;
    const vm::Vec3 after = track.keys[span->after].vector;
    if (track.interpolation == Interpolation::Cubic) {
        const vm::Vec3 earlier = track.keys[neighbour(track, span->before, -1)].vector;
        const vm::Vec3 later = track.keys[neighbour(track, span->after, 1)].vector;
        return vm::Vec3{catmull_rom(earlier.x, before.x, after.x, later.x, span->blend),
                        catmull_rom(earlier.y, before.y, after.y, later.y, span->blend),
                        catmull_rom(earlier.z, before.z, after.z, later.z, span->blend)};
    }
    return before + (after - before) * span->blend;
}

vm::Quat sample_rotation(const Track& track, float time, vm::Quat fallback) {
    const std::optional<KeySpan> span = span_at(track, time);
    if (!span.has_value()) return fallback;
    const vm::Quat before = track.keys[span->before].rotation;
    if (track.interpolation == Interpolation::Nearest || span->before == span->after) return before;
    const vm::Quat after = aligned(before, track.keys[span->after].rotation);
    if (track.interpolation == Interpolation::Cubic) {
        const vm::Quat earlier = aligned(before, track.keys[neighbour(track, span->before, -1)].rotation);
        const vm::Quat later = aligned(after, track.keys[neighbour(track, span->after, 1)].rotation);
        return vm::normalize(vm::Quat{catmull_rom(earlier.x, before.x, after.x, later.x, span->blend),
                                      catmull_rom(earlier.y, before.y, after.y, later.y, span->blend),
                                      catmull_rom(earlier.z, before.z, after.z, later.z, span->blend),
                                      catmull_rom(earlier.w, before.w, after.w, later.w, span->blend)});
    }
    return nlerp(before, after, span->blend);
}

bool sample_flag(const Track& track, float time, bool fallback) {
    const std::optional<KeySpan> span = span_at(track, time);
    if (!span.has_value()) return fallback;
    return track.keys[span->before].flag;
}

vm::Quat nlerp(vm::Quat a, vm::Quat b, float t) {
    vm::Quat target = b;
    if (vm::dot(a, b) < 0.0f) target = vm::Quat{-b.x, -b.y, -b.z, -b.w};
    return vm::normalize(vm::Quat{a.x + (target.x - a.x) * t, a.y + (target.y - a.y) * t, a.z + (target.z - a.z) * t,
                                  a.w + (target.w - a.w) * t});
}

vm::Quat conjugate(vm::Quat q) {
    return vm::Quat{-q.x, -q.y, -q.z, q.w};
}

void keys_in_range(const Track& track, const Range& range, std::vector<size_t>& out) {
    out.clear();
    if (!range.wrapped) {
        if (range.to >= range.from) {
            forward(track, range.from, range.to, out);
        } else {
            backward(track, range.from, range.to, out);
        }
        return;
    }
    if (range.to < range.from) {
        forward(track, range.from, range.high, out);
        forward(track, range.low - 1.0f, range.to, out);
        return;
    }
    backward(track, range.from, range.low, out);
    backward(track, range.high + 1.0f, range.to, out);
}

const Marker* find_marker(const Clip& clip, std::string_view name) {
    for (const Marker& candidate : clip.markers) {
        if (candidate.name == name) return &candidate;
    }
    return nullptr;
}

} // namespace bench::anim
