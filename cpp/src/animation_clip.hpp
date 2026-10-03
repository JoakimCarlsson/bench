#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include "vecmath.hpp"

/// The engine's animation clips: keyframe tracks of vectors, rotations and
/// flags with per-key easing, and the sampling of them, from
/// animation_clip.hpp and animation_clip.cpp. Targets and events are views of
/// strings that outlive the clips.
namespace bench::anim {

/// Kind of property an animation track drives.
enum class TrackKind : uint8_t { Position, Rotation, Scale, Active, Event };

/// How values are interpolated between keys.
enum class Interpolation : uint8_t { Nearest, Linear, Cubic };

/// How a clip repeats when playback passes its end.
enum class LoopMode : uint8_t { None, Linear, PingPong };

/// Easing family used to shape a blend.
enum class Transition : uint8_t { Linear, Sine, Quad, Cubic, Expo, Circ, Back };

/// Direction in which an easing transition is applied.
enum class Ease : uint8_t { In, Out, InOut, OutIn };

/// Easing family and direction pair.
struct Easing {
    Transition transition{Transition::Linear};
    Ease ease{Ease::InOut};
};

/// One keyframe on an animation track.
struct Key {
    float time{};
    vm::Vec3 vector{};
    vm::Quat rotation{};
    bool flag{true};
    std::string_view event{};
    Easing easing{};
};

/// A timeline of keys targeting one property of one entity by path.
struct Track {
    TrackKind kind{TrackKind::Position};
    Interpolation interpolation{Interpolation::Linear};
    std::string_view target{};
    bool enabled{true};
    std::vector<Key> keys{};
};

/// A named time on a clip.
struct Marker {
    std::string_view name{};
    float time{};
};

/// An authored animation clip with its tracks and markers.
struct Clip {
    float duration_seconds{1.0f};
    LoopMode loop{LoopMode::None};
    std::vector<Track> tracks{};
    std::vector<Marker> markers{};
};

/// A span of playback time, possibly wrapping across a loop.
struct Range {
    float from{};
    float to{};
    float low{};
    float high{};
    bool wrapped{};
};

/// The larger of two values, as a select.
constexpr float max_float(float a, float b) {
    return a < b ? b : a;
}

/// `value` limited to [low, high], as selects.
constexpr float clamp_float(float value, float low, float high) {
    return value < low ? low : (high < value ? high : value);
}

/// The value a vector track holds when it has no keys.
vm::Vec3 track_rest_vector(TrackKind kind);

/// Evaluates an easing curve for progress `blend`, clamped to 0..1.
float ease_curve(float blend, Easing easing);

/// Samples a vector track at `time`, or returns `fallback` when it has no keys.
vm::Vec3 sample_vector(const Track& track, float time, vm::Vec3 fallback);

/// Samples a rotation track at `time`, or returns `fallback` when it has no keys.
vm::Quat sample_rotation(const Track& track, float time, vm::Quat fallback);

/// Samples an active track at `time`, or returns `fallback` when it has no keys.
bool sample_flag(const Track& track, float time, bool fallback);

/// Normalised linear interpolation from `a` to `b` along the shorter arc.
vm::Quat nlerp(vm::Quat a, vm::Quat b, float t);

/// The inverse of a unit quaternion.
vm::Quat conjugate(vm::Quat q);

/// Replaces `out` with the indices of the keys passed over by `range`, in playback order.
void keys_in_range(const Track& track, const Range& range, std::vector<size_t>& out);

/// The marker named `name`, or null when absent.
const Marker* find_marker(const Clip& clip, std::string_view name);

} // namespace bench::anim
