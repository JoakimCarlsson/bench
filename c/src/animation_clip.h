#ifndef BENCH_ANIMATION_CLIP_H
#define BENCH_ANIMATION_CLIP_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "array.h"
#include "vecmath.h"

/// The engine's animation clips: keyframe tracks of vectors, rotations and
/// flags with per-key easing, and the sampling of them, from
/// animation_clip.hpp and animation_clip.cpp. Targets and events are views of
/// strings that outlive the clips.

/// A view of a string; not null-terminated.
typedef struct {
    const char* ptr;
    size_t len;
} Str;

/// A `Str` of a string literal, as an expression.
#define STR(literal) ((Str){ "" literal, sizeof(literal) - 1 })

/// A `Str` of a string literal, as a static initialiser.
#define STR_INIT(literal) { "" literal, sizeof(literal) - 1 }

/// Whether two strings hold the same characters.
static inline bool str_eq(Str a, Str b) { return a.len == b.len && memcmp(a.ptr, b.ptr, a.len) == 0; }

/// Kind of property an animation track drives.
typedef enum { TRACK_POSITION, TRACK_ROTATION, TRACK_SCALE, TRACK_ACTIVE, TRACK_EVENT } AnimTrackKind;

/// How values are interpolated between keys.
typedef enum { INTERP_NEAREST, INTERP_LINEAR, INTERP_CUBIC } AnimInterpolation;

/// How a clip repeats when playback passes its end.
typedef enum { LOOP_NONE, LOOP_LINEAR, LOOP_PING_PONG } AnimLoopMode;

/// Easing family used to shape a blend.
typedef enum {
    TRANSITION_LINEAR,
    TRANSITION_SINE,
    TRANSITION_QUAD,
    TRANSITION_CUBIC,
    TRANSITION_EXPO,
    TRANSITION_CIRC,
    TRANSITION_BACK
} AnimTransition;

/// Direction in which an easing transition is applied.
typedef enum { EASE_IN, EASE_OUT, EASE_IN_OUT, EASE_OUT_IN } AnimEase;

/// Easing family and direction pair.
typedef struct {
    AnimTransition transition;
    AnimEase ease;
} AnimEasing;

/// One keyframe on an animation track.
typedef struct {
    float time;
    Vec3 vector;
    Quat rotation;
    bool flag;
    Str event;
    AnimEasing easing;
} AnimKey;

/// A timeline of keys targeting one property of one entity by path.
typedef struct {
    AnimTrackKind kind;
    AnimInterpolation interpolation;
    Str target;
    bool enabled;
    AnimKey* keys;
    size_t key_count;
} AnimTrack;

/// A named time on a clip.
typedef struct {
    Str name;
    float time;
} AnimMarker;

/// An authored animation clip with its tracks and markers.
typedef struct {
    float duration_seconds;
    AnimLoopMode loop;
    AnimTrack* tracks;
    size_t track_count;
    AnimMarker* markers;
    size_t marker_count;
} AnimClip;

/// A span of playback time, possibly wrapping across a loop.
typedef struct {
    float from;
    float to;
    float low;
    float high;
    bool wrapped;
} AnimRange;

typedef ARRAY_OF(size_t) SizeArray;

/// The larger of two values, as a select.
static inline float max_float(float a, float b) { return a < b ? b : a; }

/// `value` limited to [low, high], as selects.
static inline float clamp_float(float value, float low, float high) { return value < low ? low : (high < value ? high : value); }

/// An easing of `transition` and `ease`.
static inline AnimEasing anim_easing(AnimTransition transition, AnimEase ease) { return (AnimEasing){ transition, ease }; }

/// A key at time zero holding the rest vector, the identity rotation and a set flag.
AnimKey anim_key_default(void);

/// The value a vector track holds when it has no keys.
Vec3 anim_track_rest_vector(AnimTrackKind kind);

/// Evaluates an easing curve for progress `blend`, clamped to 0..1.
float anim_ease_curve(float blend, AnimEasing easing);

/// Samples a vector track at `time`, or returns `fallback` when it has no keys.
Vec3 anim_sample_vector(const AnimTrack* track, float time, Vec3 fallback);

/// Samples a rotation track at `time`, or returns `fallback` when it has no keys.
Quat anim_sample_rotation(const AnimTrack* track, float time, Quat fallback);

/// Samples an active track at `time`, or returns `fallback` when it has no keys.
bool anim_sample_flag(const AnimTrack* track, float time, bool fallback);

/// Normalised linear interpolation from `a` to `b` along the shorter arc.
Quat anim_nlerp(Quat a, Quat b, float t);

/// The inverse of a unit quaternion.
Quat anim_conjugate(Quat q);

/// Replaces `out` with the indices of the keys passed over by `range`, in playback order.
void anim_keys_in_range(const AnimTrack* track, const AnimRange* range, SizeArray* out);

/// The marker named `name`, or null when absent.
const AnimMarker* anim_find_marker(const AnimClip* clip, Str name);

/// Frees the tracks, their keys and the markers.
void anim_clip_free(AnimClip* clip);

#endif
