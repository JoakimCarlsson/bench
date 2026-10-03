#include "animation_clip.h"

#include <math.h>
#include <stdlib.h>

static const float sine_quarter_turn = 1.57079633f;
static const float sine_fourth_term = 0.0416666667f;
static const float sine_sixth_term = 0.00138888889f;
static const float back_overshoot = 1.70158f;

/// The pair of keys surrounding a sample time and the eased blend between them.
typedef struct {
    size_t before;
    size_t after;
    float blend;
} KeySpan;

/// The first key strictly after `time`, or the key count when none is.
static size_t upper_key(const AnimTrack* track, float time) {
    size_t index = 0;
    while (index < track->key_count && track->keys[index].time <= time) ++index;
    return index;
}

/// The ease-in form of a transition curve at `t` in [0, 1]. Sine and expo are
/// polynomial stand-ins for the engine's cosine and power.
static float ease_in_curve(AnimTransition transition, float t) {
    switch (transition) {
    case TRANSITION_SINE: {
        float x = t * sine_quarter_turn;
        float x2 = x * x;
        return x2 * (0.5f - x2 * (sine_fourth_term - x2 * sine_sixth_term));
    }
    case TRANSITION_QUAD:
        return t * t;
    case TRANSITION_CUBIC:
        return t * t * t;
    case TRANSITION_EXPO: {
        float t2 = t * t;
        float t4 = t2 * t2;
        float t8 = t4 * t4;
        return t <= 0.0f ? 0.0f : t8 * t2;
    }
    case TRANSITION_CIRC:
        return 1.0f - sqrtf(max_float(1.0f - (t * t), 0.0f));
    case TRANSITION_BACK:
        return t * t * (((back_overshoot + 1.0f) * t) - back_overshoot);
    case TRANSITION_LINEAR:
        break;
    }
    return t;
}

/// The ease-out form of a transition curve, the mirror of ease-in.
static float ease_out_curve(AnimTransition transition, float t) { return 1.0f - ease_in_curve(transition, 1.0f - t); }

/// `value`, negated when its dot with `reference` is negative.
static Quat aligned(Quat reference, Quat value) {
    if (quat_dot(reference, value) < 0.0f) return (Quat){ -value.x, -value.y, -value.z, -value.w };
    return value;
}

/// One component of a Catmull-Rom segment from `start` to `end` at `t`.
static float catmull_rom(float before, float start, float end, float after, float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    return 0.5f * (((2.0f * start) + ((end - before) * t) + (((2.0f * before) - (5.0f * start) + (4.0f * end) - after) * t2) +
                    ((-before + (3.0f * start) - (3.0f * end) + after) * t3)));
}

/// The key `step` places from `index`, clamped to the track.
static size_t neighbour(const AnimTrack* track, size_t index, int step) {
    ptrdiff_t count = (ptrdiff_t)track->key_count;
    ptrdiff_t wanted = (ptrdiff_t)index + step;
    ptrdiff_t last = count > 0 ? count - 1 : 0;
    return (size_t)(wanted < 0 ? 0 : (last < wanted ? last : wanted));
}

/// The keys surrounding `time` and the eased blend between them, written to
/// `span`; times outside the keys clamp to the first or last key. False when
/// the track has no keys.
static bool span_at(const AnimTrack* track, float time, KeySpan* span) {
    if (track->key_count == 0) return false;
    size_t next = upper_key(track, time);
    if (next == 0) {
        *span = (KeySpan){ 0, 0, 0.0f };
        return true;
    }
    if (next >= track->key_count) {
        size_t last = track->key_count - 1;
        *span = (KeySpan){ last, last, 0.0f };
        return true;
    }
    float start = track->keys[next - 1].time;
    float end = track->keys[next].time;
    float width = end - start;
    float blend = width > 0.0f ? (time - start) / width : 0.0f;
    *span = (KeySpan){ next - 1, next, anim_ease_curve(clamp_float(blend, 0.0f, 1.0f), track->keys[next - 1].easing) };
    return true;
}

/// Appends the indices of keys with `from < time <= to`, ascending.
static void forward(const AnimTrack* track, float from, float to, SizeArray* out) {
    for (size_t index = 0; index < track->key_count; ++index) {
        float at = track->keys[index].time;
        if (at > from && at <= to) ARRAY_PUSH(*out, index);
    }
}

/// Appends the indices of keys with `to <= time < from`, descending.
static void backward(const AnimTrack* track, float from, float to, SizeArray* out) {
    for (size_t index = track->key_count; index > 0; --index) {
        float at = track->keys[index - 1].time;
        if (at < from && at >= to) ARRAY_PUSH(*out, index - 1);
    }
}

AnimKey anim_key_default(void) {
    AnimKey key;
    memset(&key, 0, sizeof key);
    key.rotation = quat_identity();
    key.flag = true;
    key.easing = anim_easing(TRANSITION_LINEAR, EASE_IN_OUT);
    return key;
}

Vec3 anim_track_rest_vector(AnimTrackKind kind) {
    return kind == TRACK_SCALE ? (Vec3){ 1.0f, 1.0f, 1.0f } : (Vec3){ 0.0f, 0.0f, 0.0f };
}

float anim_ease_curve(float blend, AnimEasing easing) {
    float t = clamp_float(blend, 0.0f, 1.0f);
    if (easing.transition == TRANSITION_LINEAR) return t;
    switch (easing.ease) {
    case EASE_IN:
        return ease_in_curve(easing.transition, t);
    case EASE_OUT:
        return ease_out_curve(easing.transition, t);
    case EASE_IN_OUT:
        return t < 0.5f ? ease_in_curve(easing.transition, t * 2.0f) * 0.5f
                        : 0.5f + (ease_out_curve(easing.transition, (t * 2.0f) - 1.0f) * 0.5f);
    case EASE_OUT_IN:
        break;
    }
    return t < 0.5f ? ease_out_curve(easing.transition, t * 2.0f) * 0.5f
                    : 0.5f + (ease_in_curve(easing.transition, (t * 2.0f) - 1.0f) * 0.5f);
}

Vec3 anim_sample_vector(const AnimTrack* track, float time, Vec3 fallback) {
    KeySpan span;
    if (!span_at(track, time, &span)) return fallback;
    Vec3 before = track->keys[span.before].vector;
    if (track->interpolation == INTERP_NEAREST || span.before == span.after) return before;
    Vec3 after = track->keys[span.after].vector;
    if (track->interpolation == INTERP_CUBIC) {
        Vec3 earlier = track->keys[neighbour(track, span.before, -1)].vector;
        Vec3 later = track->keys[neighbour(track, span.after, 1)].vector;
        return (Vec3){ catmull_rom(earlier.x, before.x, after.x, later.x, span.blend),
                       catmull_rom(earlier.y, before.y, after.y, later.y, span.blend),
                       catmull_rom(earlier.z, before.z, after.z, later.z, span.blend) };
    }
    return v3_add(before, v3_scale(v3_sub(after, before), span.blend));
}

Quat anim_sample_rotation(const AnimTrack* track, float time, Quat fallback) {
    KeySpan span;
    if (!span_at(track, time, &span)) return fallback;
    Quat before = track->keys[span.before].rotation;
    if (track->interpolation == INTERP_NEAREST || span.before == span.after) return before;
    Quat after = aligned(before, track->keys[span.after].rotation);
    if (track->interpolation == INTERP_CUBIC) {
        Quat earlier = aligned(before, track->keys[neighbour(track, span.before, -1)].rotation);
        Quat later = aligned(after, track->keys[neighbour(track, span.after, 1)].rotation);
        return quat_normalize((Quat){ catmull_rom(earlier.x, before.x, after.x, later.x, span.blend),
                                      catmull_rom(earlier.y, before.y, after.y, later.y, span.blend),
                                      catmull_rom(earlier.z, before.z, after.z, later.z, span.blend),
                                      catmull_rom(earlier.w, before.w, after.w, later.w, span.blend) });
    }
    return anim_nlerp(before, after, span.blend);
}

bool anim_sample_flag(const AnimTrack* track, float time, bool fallback) {
    KeySpan span;
    if (!span_at(track, time, &span)) return fallback;
    return track->keys[span.before].flag;
}

Quat anim_nlerp(Quat a, Quat b, float t) {
    Quat target = b;
    if (quat_dot(a, b) < 0.0f) target = (Quat){ -b.x, -b.y, -b.z, -b.w };
    return quat_normalize((Quat){ a.x + (target.x - a.x) * t, a.y + (target.y - a.y) * t, a.z + (target.z - a.z) * t,
                                  a.w + (target.w - a.w) * t });
}

Quat anim_conjugate(Quat q) { return (Quat){ -q.x, -q.y, -q.z, q.w }; }

void anim_keys_in_range(const AnimTrack* track, const AnimRange* range, SizeArray* out) {
    out->len = 0;
    if (!range->wrapped) {
        if (range->to >= range->from) {
            forward(track, range->from, range->to, out);
        } else {
            backward(track, range->from, range->to, out);
        }
        return;
    }
    if (range->to < range->from) {
        forward(track, range->from, range->high, out);
        forward(track, range->low - 1.0f, range->to, out);
        return;
    }
    backward(track, range->from, range->low, out);
    backward(track, range->high + 1.0f, range->to, out);
}

const AnimMarker* anim_find_marker(const AnimClip* clip, Str name) {
    for (size_t i = 0; i < clip->marker_count; ++i) {
        if (str_eq(clip->markers[i].name, name)) return &clip->markers[i];
    }
    return NULL;
}

void anim_clip_free(AnimClip* clip) {
    for (size_t i = 0; i < clip->track_count; ++i) free(clip->tracks[i].keys);
    free(clip->tracks);
    free(clip->markers);
}
