//! The engine's animation clips: keyframe tracks of vectors, rotations and
//! flags with per-key easing, and the sampling of them, from
//! animation_clip.hpp and animation_clip.cpp. Targets and events are static
//! strings.
use crate::vecmath::{Quat, Vec3};

const SINE_QUARTER_TURN: f32 = 1.57079633;
const SINE_FOURTH_TERM: f32 = 0.0416666667;
const SINE_SIXTH_TERM: f32 = 0.00138888889;
const BACK_OVERSHOOT: f32 = 1.70158;

/// Kind of property an animation track drives.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum TrackKind {
    Position,
    Rotation,
    Scale,
    Active,
    Event,
}

/// How values are interpolated between keys.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Interpolation {
    Nearest,
    Linear,
    Cubic,
}

/// How a clip repeats when playback passes its end.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum LoopMode {
    None,
    Linear,
    PingPong,
}

/// Easing family used to shape a blend.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Transition {
    Linear,
    Sine,
    Quad,
    Cubic,
    Expo,
    Circ,
    Back,
}

/// Direction in which an easing transition is applied.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Ease {
    In,
    Out,
    InOut,
    OutIn,
}

/// Easing family and direction pair.
#[derive(Clone, Copy)]
pub struct Easing {
    pub transition: Transition,
    pub ease: Ease,
}

impl Default for Easing {
    /// A linear transition eased in and out.
    fn default() -> Easing {
        Easing { transition: Transition::Linear, ease: Ease::InOut }
    }
}

/// One keyframe on an animation track.
#[derive(Clone, Copy)]
pub struct Key {
    pub time: f32,
    pub vector: Vec3,
    pub rotation: Quat,
    pub flag: bool,
    pub event: &'static str,
    pub easing: Easing,
}

impl Default for Key {
    /// A key at time zero holding the rest vector, the identity rotation and a set flag.
    fn default() -> Key {
        Key { time: 0.0, vector: Vec3::default(), rotation: Quat::default(), flag: true, event: "", easing: Easing::default() }
    }
}

/// A timeline of keys targeting one property of one entity by path.
pub struct Track {
    pub kind: TrackKind,
    pub interpolation: Interpolation,
    pub target: &'static str,
    pub enabled: bool,
    pub keys: Vec<Key>,
}

/// A named time on a clip.
pub struct Marker {
    pub name: &'static str,
    pub time: f32,
}

/// An authored animation clip with its tracks and markers.
pub struct Clip {
    pub duration_seconds: f32,
    pub loop_mode: LoopMode,
    pub tracks: Vec<Track>,
    pub markers: Vec<Marker>,
}

/// A span of playback time, possibly wrapping across a loop.
#[derive(Clone, Copy)]
pub struct Range {
    pub from: f32,
    pub to: f32,
    pub low: f32,
    pub high: f32,
    pub wrapped: bool,
}

/// The pair of keys surrounding a sample time and the eased blend between them.
struct KeySpan {
    before: usize,
    after: usize,
    blend: f32,
}

/// The larger of two values, as a select.
pub fn max_float(a: f32, b: f32) -> f32 {
    if a < b { b } else { a }
}

/// `value` limited to [low, high], as selects.
pub fn clamp_float(value: f32, low: f32, high: f32) -> f32 {
    if value < low {
        low
    } else if high < value {
        high
    } else {
        value
    }
}

/// The first key strictly after `time`, or the key count when none is.
fn upper_key(track: &Track, time: f32) -> usize {
    let mut index = 0;
    while index < track.keys.len() && track.keys[index].time <= time {
        index += 1;
    }
    index
}

/// The ease-in form of a transition curve at `t` in [0, 1]. Sine and expo are
/// polynomial stand-ins for the engine's cosine and power.
fn ease_in_curve(transition: Transition, t: f32) -> f32 {
    match transition {
        Transition::Sine => {
            let x = t * SINE_QUARTER_TURN;
            let x2 = x * x;
            x2 * (0.5 - x2 * (SINE_FOURTH_TERM - x2 * SINE_SIXTH_TERM))
        }
        Transition::Quad => t * t,
        Transition::Cubic => t * t * t,
        Transition::Expo => {
            let t2 = t * t;
            let t4 = t2 * t2;
            let t8 = t4 * t4;
            if t <= 0.0 { 0.0 } else { t8 * t2 }
        }
        Transition::Circ => 1.0 - max_float(1.0 - (t * t), 0.0).sqrt(),
        Transition::Back => t * t * (((BACK_OVERSHOOT + 1.0) * t) - BACK_OVERSHOOT),
        Transition::Linear => t,
    }
}

/// The ease-out form of a transition curve, the mirror of ease-in.
fn ease_out_curve(transition: Transition, t: f32) -> f32 {
    1.0 - ease_in_curve(transition, 1.0 - t)
}

/// `value`, negated when its dot with `reference` is negative.
fn aligned(reference: Quat, value: Quat) -> Quat {
    if reference.dot(value) < 0.0 {
        return Quat { x: -value.x, y: -value.y, z: -value.z, w: -value.w };
    }
    value
}

/// One component of a Catmull-Rom segment from `start` to `end` at `t`.
fn catmull_rom(before: f32, start: f32, end: f32, after: f32, t: f32) -> f32 {
    let t2 = t * t;
    let t3 = t2 * t;
    0.5 * (((2.0 * start)
        + ((end - before) * t)
        + (((2.0 * before) - (5.0 * start) + (4.0 * end) - after) * t2)
        + ((-before + (3.0 * start) - (3.0 * end) + after) * t3)))
}

/// The key `step` places from `index`, clamped to the track.
fn neighbour(track: &Track, index: usize, step: isize) -> usize {
    let count = track.keys.len() as isize;
    let wanted = index as isize + step;
    let last = if count > 0 { count - 1 } else { 0 };
    (if wanted < 0 {
        0
    } else if last < wanted {
        last
    } else {
        wanted
    }) as usize
}

/// The keys surrounding `time` and the eased blend between them; times
/// outside the keys clamp to the first or last key.
fn span_at(track: &Track, time: f32) -> Option<KeySpan> {
    if track.keys.is_empty() {
        return None;
    }
    let next = upper_key(track, time);
    if next == 0 {
        return Some(KeySpan { before: 0, after: 0, blend: 0.0 });
    }
    if next >= track.keys.len() {
        let last = track.keys.len() - 1;
        return Some(KeySpan { before: last, after: last, blend: 0.0 });
    }
    let start = track.keys[next - 1].time;
    let end = track.keys[next].time;
    let width = end - start;
    let blend = if width > 0.0 { (time - start) / width } else { 0.0 };
    Some(KeySpan { before: next - 1, after: next, blend: ease_curve(clamp_float(blend, 0.0, 1.0), track.keys[next - 1].easing) })
}

/// Appends the indices of keys with `from < time <= to`, ascending.
fn forward(track: &Track, from: f32, to: f32, out: &mut Vec<usize>) {
    for (index, key) in track.keys.iter().enumerate() {
        if key.time > from && key.time <= to {
            out.push(index);
        }
    }
}

/// Appends the indices of keys with `to <= time < from`, descending.
fn backward(track: &Track, from: f32, to: f32, out: &mut Vec<usize>) {
    for (index, key) in track.keys.iter().enumerate().rev() {
        if key.time < from && key.time >= to {
            out.push(index);
        }
    }
}

/// The value a vector track holds when it has no keys.
pub fn track_rest_vector(kind: TrackKind) -> Vec3 {
    if kind == TrackKind::Scale { Vec3::new(1.0, 1.0, 1.0) } else { Vec3::default() }
}

/// Evaluates an easing curve for progress `blend`, clamped to 0..1.
pub fn ease_curve(blend: f32, easing: Easing) -> f32 {
    let t = clamp_float(blend, 0.0, 1.0);
    if easing.transition == Transition::Linear {
        return t;
    }
    match easing.ease {
        Ease::In => ease_in_curve(easing.transition, t),
        Ease::Out => ease_out_curve(easing.transition, t),
        Ease::InOut => {
            if t < 0.5 {
                ease_in_curve(easing.transition, t * 2.0) * 0.5
            } else {
                0.5 + (ease_out_curve(easing.transition, (t * 2.0) - 1.0) * 0.5)
            }
        }
        Ease::OutIn => {
            if t < 0.5 {
                ease_out_curve(easing.transition, t * 2.0) * 0.5
            } else {
                0.5 + (ease_in_curve(easing.transition, (t * 2.0) - 1.0) * 0.5)
            }
        }
    }
}

/// Samples a vector track at `time`, or returns `fallback` when it has no keys.
pub fn sample_vector(track: &Track, time: f32, fallback: Vec3) -> Vec3 {
    let Some(span) = span_at(track, time) else {
        return fallback;
    };
    let before = track.keys[span.before].vector;
    if track.interpolation == Interpolation::Nearest || span.before == span.after {
        return before;
    }
    let after = track.keys[span.after].vector;
    if track.interpolation == Interpolation::Cubic {
        let earlier = track.keys[neighbour(track, span.before, -1)].vector;
        let later = track.keys[neighbour(track, span.after, 1)].vector;
        return Vec3::new(
            catmull_rom(earlier.x, before.x, after.x, later.x, span.blend),
            catmull_rom(earlier.y, before.y, after.y, later.y, span.blend),
            catmull_rom(earlier.z, before.z, after.z, later.z, span.blend),
        );
    }
    before + (after - before) * span.blend
}

/// Samples a rotation track at `time`, or returns `fallback` when it has no keys.
pub fn sample_rotation(track: &Track, time: f32, fallback: Quat) -> Quat {
    let Some(span) = span_at(track, time) else {
        return fallback;
    };
    let before = track.keys[span.before].rotation;
    if track.interpolation == Interpolation::Nearest || span.before == span.after {
        return before;
    }
    let after = aligned(before, track.keys[span.after].rotation);
    if track.interpolation == Interpolation::Cubic {
        let earlier = aligned(before, track.keys[neighbour(track, span.before, -1)].rotation);
        let later = aligned(after, track.keys[neighbour(track, span.after, 1)].rotation);
        return Quat {
            x: catmull_rom(earlier.x, before.x, after.x, later.x, span.blend),
            y: catmull_rom(earlier.y, before.y, after.y, later.y, span.blend),
            z: catmull_rom(earlier.z, before.z, after.z, later.z, span.blend),
            w: catmull_rom(earlier.w, before.w, after.w, later.w, span.blend),
        }
        .normalize();
    }
    nlerp(before, after, span.blend)
}

/// Samples an active track at `time`, or returns `fallback` when it has no keys.
pub fn sample_flag(track: &Track, time: f32, fallback: bool) -> bool {
    match span_at(track, time) {
        Some(span) => track.keys[span.before].flag,
        None => fallback,
    }
}

/// Normalised linear interpolation from `a` to `b` along the shorter arc.
pub fn nlerp(a: Quat, b: Quat, t: f32) -> Quat {
    let target = if a.dot(b) < 0.0 { Quat { x: -b.x, y: -b.y, z: -b.z, w: -b.w } } else { b };
    Quat {
        x: a.x + (target.x - a.x) * t,
        y: a.y + (target.y - a.y) * t,
        z: a.z + (target.z - a.z) * t,
        w: a.w + (target.w - a.w) * t,
    }
    .normalize()
}

/// The inverse of a unit quaternion.
pub fn conjugate(q: Quat) -> Quat {
    Quat { x: -q.x, y: -q.y, z: -q.z, w: q.w }
}

/// Replaces `out` with the indices of the keys passed over by `range`, in playback order.
pub fn keys_in_range(track: &Track, range: &Range, out: &mut Vec<usize>) {
    out.clear();
    if !range.wrapped {
        if range.to >= range.from {
            forward(track, range.from, range.to, out);
        } else {
            backward(track, range.from, range.to, out);
        }
        return;
    }
    if range.to < range.from {
        forward(track, range.from, range.high, out);
        forward(track, range.low - 1.0, range.to, out);
        return;
    }
    backward(track, range.from, range.low, out);
    backward(track, range.high + 1.0, range.to, out);
}

/// The marker named `name`, if any.
pub fn find_marker<'a>(clip: &'a Clip, name: &str) -> Option<&'a Marker> {
    clip.markers.iter().find(|candidate| candidate.name == name)
}
