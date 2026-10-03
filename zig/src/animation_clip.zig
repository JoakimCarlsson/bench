//! The engine's animation clips: keyframe tracks of vectors, rotations and
//! flags with per-key easing, and the sampling of them, from
//! animation_clip.hpp and animation_clip.cpp. Targets and events are slices of
//! strings that outlive the clips.
const std = @import("std");
const vm = @import("vecmath.zig");

const Vec3 = vm.Vec3;
const Quat = vm.Quat;
const Allocator = std.mem.Allocator;

const sine_quarter_turn: f32 = 1.57079633;
const sine_fourth_term: f32 = 0.0416666667;
const sine_sixth_term: f32 = 0.00138888889;
const back_overshoot: f32 = 1.70158;

/// Kind of property an animation track drives.
pub const TrackKind = enum(u8) { position, rotation, scale, active, event };

/// How values are interpolated between keys.
pub const Interpolation = enum(u8) { nearest, linear, cubic };

/// How a clip repeats when playback passes its end.
pub const LoopMode = enum(u8) { none, linear, ping_pong };

/// Easing family used to shape a blend.
pub const Transition = enum(u8) { linear, sine, quad, cubic, expo, circ, back };

/// Direction in which an easing transition is applied.
pub const Ease = enum(u8) { in, out, in_out, out_in };

/// Easing family and direction pair.
pub const Easing = struct {
    transition: Transition = .linear,
    ease: Ease = .in_out,
};

/// One keyframe on an animation track.
pub const Key = struct {
    time: f32 = 0.0,
    vector: Vec3 = .{},
    rotation: Quat = .{},
    flag: bool = true,
    event: []const u8 = "",
    easing: Easing = .{},
};

/// A timeline of keys targeting one property of one entity by path.
pub const Track = struct {
    kind: TrackKind = .position,
    interpolation: Interpolation = .linear,
    target: []const u8 = "",
    enabled: bool = true,
    keys: []Key = &.{},
};

/// A named time on a clip.
pub const Marker = struct {
    name: []const u8 = "",
    time: f32 = 0.0,
};

/// An authored animation clip with its tracks and markers.
pub const Clip = struct {
    duration_seconds: f32 = 1.0,
    loop: LoopMode = .none,
    tracks: []Track = &.{},
    markers: []Marker = &.{},

    /// Frees the tracks, their keys and the markers.
    pub fn deinit(self: *Clip, gpa: Allocator) void {
        for (self.tracks) |track| gpa.free(track.keys);
        gpa.free(self.tracks);
        gpa.free(self.markers);
    }
};

/// A span of playback time, possibly wrapping across a loop.
pub const Range = struct {
    from: f32 = 0.0,
    to: f32 = 0.0,
    low: f32 = 0.0,
    high: f32 = 0.0,
    wrapped: bool = false,
};

/// The pair of keys surrounding a sample time and the eased blend between them.
const KeySpan = struct {
    before: usize,
    after: usize,
    blend: f32,
};

/// The larger of two values, as a select.
pub fn maxFloat(a: f32, b: f32) f32 {
    return if (a < b) b else a;
}

/// `value` limited to [low, high], as selects.
pub fn clampFloat(value: f32, low: f32, high: f32) f32 {
    return if (value < low) low else if (high < value) high else value;
}

/// The first key strictly after `time`, or the key count when none is.
fn upperKey(track: *const Track, time: f32) usize {
    var index: usize = 0;
    while (index < track.keys.len and track.keys[index].time <= time) index += 1;
    return index;
}

/// The ease-in form of a transition curve at `t` in [0, 1]. Sine and expo are
/// polynomial stand-ins for the engine's cosine and power.
fn easeInCurve(transition: Transition, t: f32) f32 {
    switch (transition) {
        .sine => {
            const x = t * sine_quarter_turn;
            const x2 = x * x;
            return x2 * (0.5 - x2 * (sine_fourth_term - x2 * sine_sixth_term));
        },
        .quad => return t * t,
        .cubic => return t * t * t,
        .expo => {
            const t2 = t * t;
            const t4 = t2 * t2;
            const t8 = t4 * t4;
            return if (t <= 0.0) 0.0 else t8 * t2;
        },
        .circ => return 1.0 - @sqrt(maxFloat(1.0 - (t * t), 0.0)),
        .back => return t * t * (((back_overshoot + 1.0) * t) - back_overshoot),
        .linear => return t,
    }
}

/// The ease-out form of a transition curve, the mirror of ease-in.
fn easeOutCurve(transition: Transition, t: f32) f32 {
    return 1.0 - easeInCurve(transition, 1.0 - t);
}

/// `value`, negated when its dot with `reference` is negative.
fn aligned(reference: Quat, value: Quat) Quat {
    if (reference.dot(value) < 0.0) return .{ .x = -value.x, .y = -value.y, .z = -value.z, .w = -value.w };
    return value;
}

/// One component of a Catmull-Rom segment from `start` to `end` at `t`.
fn catmullRom(before: f32, start: f32, end: f32, after: f32, t: f32) f32 {
    const t2 = t * t;
    const t3 = t2 * t;
    return 0.5 * (((2.0 * start) + ((end - before) * t) + (((2.0 * before) - (5.0 * start) + (4.0 * end) - after) * t2) +
        ((-before + (3.0 * start) - (3.0 * end) + after) * t3)));
}

/// The key `step` places from `index`, clamped to the track.
fn neighbour(track: *const Track, index: usize, step: isize) usize {
    const count: isize = @intCast(track.keys.len);
    const wanted = @as(isize, @intCast(index)) + step;
    const last: isize = if (count > 0) count - 1 else 0;
    return @intCast(if (wanted < 0) 0 else if (last < wanted) last else wanted);
}

/// The keys surrounding `time` and the eased blend between them; times
/// outside the keys clamp to the first or last key.
fn spanAt(track: *const Track, time: f32) ?KeySpan {
    if (track.keys.len == 0) return null;
    const next = upperKey(track, time);
    if (next == 0) return .{ .before = 0, .after = 0, .blend = 0.0 };
    if (next >= track.keys.len) {
        const last = track.keys.len - 1;
        return .{ .before = last, .after = last, .blend = 0.0 };
    }
    const start = track.keys[next - 1].time;
    const end = track.keys[next].time;
    const width = end - start;
    const blend = if (width > 0.0) (time - start) / width else 0.0;
    return .{ .before = next - 1, .after = next, .blend = easeCurve(clampFloat(blend, 0.0, 1.0), track.keys[next - 1].easing) };
}

/// Appends the indices of keys with `from < time <= to`, ascending.
fn forward(gpa: Allocator, track: *const Track, from: f32, to: f32, out: *std.ArrayList(usize)) Allocator.Error!void {
    for (track.keys, 0..) |key, index| {
        if (key.time > from and key.time <= to) try out.append(gpa, index);
    }
}

/// Appends the indices of keys with `to <= time < from`, descending.
fn backward(gpa: Allocator, track: *const Track, from: f32, to: f32, out: *std.ArrayList(usize)) Allocator.Error!void {
    var index = track.keys.len;
    while (index > 0) : (index -= 1) {
        const at = track.keys[index - 1].time;
        if (at < from and at >= to) try out.append(gpa, index - 1);
    }
}

/// The value a vector track holds when it has no keys.
pub fn trackRestVector(kind: TrackKind) Vec3 {
    return if (kind == .scale) .{ .x = 1.0, .y = 1.0, .z = 1.0 } else .{};
}

/// Evaluates an easing curve for progress `blend`, clamped to 0..1.
pub fn easeCurve(blend: f32, easing: Easing) f32 {
    const t = clampFloat(blend, 0.0, 1.0);
    if (easing.transition == .linear) return t;
    switch (easing.ease) {
        .in => return easeInCurve(easing.transition, t),
        .out => return easeOutCurve(easing.transition, t),
        .in_out => return if (t < 0.5)
            easeInCurve(easing.transition, t * 2.0) * 0.5
        else
            0.5 + (easeOutCurve(easing.transition, (t * 2.0) - 1.0) * 0.5),
        .out_in => return if (t < 0.5)
            easeOutCurve(easing.transition, t * 2.0) * 0.5
        else
            0.5 + (easeInCurve(easing.transition, (t * 2.0) - 1.0) * 0.5),
    }
}

/// Samples a vector track at `time`, or returns `fallback` when it has no keys.
pub fn sampleVector(track: *const Track, time: f32, fallback: Vec3) Vec3 {
    const span = spanAt(track, time) orelse return fallback;
    const before = track.keys[span.before].vector;
    if (track.interpolation == .nearest or span.before == span.after) return before;
    const after = track.keys[span.after].vector;
    if (track.interpolation == .cubic) {
        const earlier = track.keys[neighbour(track, span.before, -1)].vector;
        const later = track.keys[neighbour(track, span.after, 1)].vector;
        return .{
            .x = catmullRom(earlier.x, before.x, after.x, later.x, span.blend),
            .y = catmullRom(earlier.y, before.y, after.y, later.y, span.blend),
            .z = catmullRom(earlier.z, before.z, after.z, later.z, span.blend),
        };
    }
    return before.add(after.sub(before).scale(span.blend));
}

/// Samples a rotation track at `time`, or returns `fallback` when it has no keys.
pub fn sampleRotation(track: *const Track, time: f32, fallback: Quat) Quat {
    const span = spanAt(track, time) orelse return fallback;
    const before = track.keys[span.before].rotation;
    if (track.interpolation == .nearest or span.before == span.after) return before;
    const after = aligned(before, track.keys[span.after].rotation);
    if (track.interpolation == .cubic) {
        const earlier = aligned(before, track.keys[neighbour(track, span.before, -1)].rotation);
        const later = aligned(after, track.keys[neighbour(track, span.after, 1)].rotation);
        const spline: Quat = .{
            .x = catmullRom(earlier.x, before.x, after.x, later.x, span.blend),
            .y = catmullRom(earlier.y, before.y, after.y, later.y, span.blend),
            .z = catmullRom(earlier.z, before.z, after.z, later.z, span.blend),
            .w = catmullRom(earlier.w, before.w, after.w, later.w, span.blend),
        };
        return spline.normalize();
    }
    return nlerp(before, after, span.blend);
}

/// Samples an active track at `time`, or returns `fallback` when it has no keys.
pub fn sampleFlag(track: *const Track, time: f32, fallback: bool) bool {
    const span = spanAt(track, time) orelse return fallback;
    return track.keys[span.before].flag;
}

/// Normalised linear interpolation from `a` to `b` along the shorter arc.
pub fn nlerp(a: Quat, b: Quat, t: f32) Quat {
    const target: Quat = if (a.dot(b) < 0.0) .{ .x = -b.x, .y = -b.y, .z = -b.z, .w = -b.w } else b;
    const mixed: Quat = .{
        .x = a.x + (target.x - a.x) * t,
        .y = a.y + (target.y - a.y) * t,
        .z = a.z + (target.z - a.z) * t,
        .w = a.w + (target.w - a.w) * t,
    };
    return mixed.normalize();
}

/// The inverse of a unit quaternion.
pub fn conjugate(q: Quat) Quat {
    return .{ .x = -q.x, .y = -q.y, .z = -q.z, .w = q.w };
}

/// Replaces `out` with the indices of the keys passed over by `range`, in playback order.
pub fn keysInRange(gpa: Allocator, track: *const Track, range: Range, out: *std.ArrayList(usize)) Allocator.Error!void {
    out.clearRetainingCapacity();
    if (!range.wrapped) {
        if (range.to >= range.from) {
            try forward(gpa, track, range.from, range.to, out);
        } else {
            try backward(gpa, track, range.from, range.to, out);
        }
        return;
    }
    if (range.to < range.from) {
        try forward(gpa, track, range.from, range.high, out);
        try forward(gpa, track, range.low - 1.0, range.to, out);
        return;
    }
    try backward(gpa, track, range.from, range.low, out);
    try backward(gpa, track, range.high + 1.0, range.to, out);
}

/// The marker named `name`, or null when absent.
pub fn findMarker(clip: *const Clip, name: []const u8) ?*const Marker {
    for (clip.markers) |*candidate| {
        if (std.mem.eql(u8, candidate.name, name)) return candidate;
    }
    return null;
}
