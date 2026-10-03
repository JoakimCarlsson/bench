//! The engine's animation players and the world that advances them, from
//! animation.hpp and animation.cpp. Clips are held by index instead of the
//! engine's slot map, and names are slices of strings that outlive the world.
const std = @import("std");
const clip_mod = @import("animation_clip.zig");
const vm = @import("vecmath.zig");

const Allocator = std.mem.Allocator;
const Vec3 = vm.Vec3;
const Quat = vm.Quat;
const Clip = clip_mod.Clip;
const Track = clip_mod.Track;
const TrackKind = clip_mod.TrackKind;
const Easing = clip_mod.Easing;
const Range = clip_mod.Range;

/// Index of a clip in the world; `invalid_clip` refers to none.
pub const ClipHandle = u32;
pub const invalid_clip: ClipHandle = std.math.maxInt(u32);

/// Blend duration that asks for the player's configured one.
pub const blend_from_default: f32 = -1.0;

/// A clip registered on a player under a name.
const ClipEntry = struct {
    name: []const u8 = "",
    clip: ClipHandle = invalid_clip,
};

/// The value of one animated property, as emitted by an advance.
pub const Sample = struct {
    target: []const u8 = "",
    kind: TrackKind = .position,
    vector: Vec3 = .{},
    rotation: Quat = .{},
    flag: bool = true,
};

/// Options of a `play` call.
pub const Play = struct {
    blend_seconds: f32 = blend_from_default,
    speed: f32 = 1.0,
    from_end: bool = false,
};

/// The span of a clip that plays.
const Section = struct {
    from: f32 = 0.0,
    to: f32 = 0.0,
};

/// A wanted section in seconds; negative ends mean the clip's own.
pub const SectionRange = struct {
    from: f32 = -1.0,
    to: f32 = -1.0,
};

/// A wanted section between two markers.
pub const MarkerRange = struct {
    from: []const u8 = "",
    to: []const u8 = "",
};

/// Playback events reported by an advance.
pub const Status = enum(u8) { started, finished };

/// A clip starting or finishing.
pub const StatusChange = struct {
    clip: []const u8 = "",
    status: Status = .started,
};

/// Motion of the root target over an advance.
pub const RootMotion = struct {
    position: Vec3 = .{},
    rotation: Quat = .{},
};

/// An event key passed over by an advance.
pub const Trigger = struct {
    clip: []const u8 = "",
    target: []const u8 = "",
    event: []const u8 = "",
    time: f32 = 0.0,
};

/// What is playing: a clip, how far in, and how fast.
const Playback = struct {
    name: []const u8 = "",
    clip: ClipHandle = invalid_clip,
    position: f32 = 0.0,
    length: f32 = 0.0,
    speed: f32 = 1.0,
};

/// A previous playback fading out.
const Blend = struct {
    playback: Playback = .{},
    seconds: f32 = 0.0,
    left: f32 = 0.0,
};

/// A captured pose fading out.
const Capture = struct {
    samples: std.ArrayList(Sample) = .empty,
    seconds: f32 = 0.0,
    left: f32 = 0.0,
    easing: Easing = .{},

    /// Drops the pose and the fade, keeping storage.
    fn clear(self: *Capture) void {
        self.samples.clearRetainingCapacity();
        self.seconds = 0.0;
        self.left = 0.0;
        self.easing = .{};
    }
};

/// A blend duration between two clips.
const BlendEntry = struct {
    from: []const u8,
    to: []const u8,
    seconds: f32,
};

/// The clip that follows another when it finishes.
const NextEntry = struct {
    from: []const u8,
    to: []const u8,
};

/// Plays clips with blends, queueing, sections, pose capture and root motion.
pub const Player = struct {
    gpa: Allocator,
    clips: std.ArrayList(ClipEntry) = .empty,
    blend_times: std.ArrayList(BlendEntry) = .empty,
    next_clips: std.ArrayList(NextEntry) = .empty,
    queue_names: std.ArrayList([]const u8) = .empty,
    blends: std.ArrayList(Blend) = .empty,
    capture: Capture = .{},
    current: Playback = .{},
    section: Section = .{},
    root_motion: RootMotion = .{},
    root_motion_total: RootMotion = .{},
    auto_capture_easing: Easing = .{},
    capture_easing_wanted: Easing = .{},
    assigned: []const u8 = "",
    section_from_marker: []const u8 = "",
    section_to_marker: []const u8 = "",
    root_motion_target: []const u8 = "",
    section_from: f32 = -1.0,
    section_to: f32 = -1.0,
    speed_scale: f32 = 1.0,
    default_blend_seconds: f32 = 0.0,
    capture_duration: f32 = 0.0,
    auto_capture_duration: f32 = 0.0,
    section_wanted: bool = false,
    section_by_marker: bool = false,
    auto_capture: bool = false,
    capture_pending: bool = false,
    enabled: bool = true,
    playing: bool = false,
    seeked: bool = false,
    started: bool = false,
    finished: bool = false,

    /// Frees the lists.
    pub fn deinit(self: *Player) void {
        self.clips.deinit(self.gpa);
        self.blend_times.deinit(self.gpa);
        self.next_clips.deinit(self.gpa);
        self.queue_names.deinit(self.gpa);
        self.blends.deinit(self.gpa);
        self.capture.samples.deinit(self.gpa);
    }

    /// Returns the player to its freshly constructed state, keeping storage.
    pub fn reset(self: *Player) void {
        self.clips.clearRetainingCapacity();
        self.blend_times.clearRetainingCapacity();
        self.next_clips.clearRetainingCapacity();
        self.queue_names.clearRetainingCapacity();
        self.blends.clearRetainingCapacity();
        self.capture.clear();
        self.current = .{};
        self.section = .{};
        self.root_motion = .{};
        self.root_motion_total = .{};
        self.auto_capture_easing = .{};
        self.capture_easing_wanted = .{};
        self.assigned = "";
        self.section_from_marker = "";
        self.section_to_marker = "";
        self.root_motion_target = "";
        self.section_from = -1.0;
        self.section_to = -1.0;
        self.speed_scale = 1.0;
        self.default_blend_seconds = 0.0;
        self.capture_duration = 0.0;
        self.auto_capture_duration = 0.0;
        self.section_wanted = false;
        self.section_by_marker = false;
        self.auto_capture = false;
        self.capture_pending = false;
        self.enabled = true;
        self.playing = false;
        self.seeked = false;
        self.started = false;
        self.finished = false;
    }

    /// The clip registered under `name`, or null.
    fn entry(self: *const Player, name: []const u8) ?ClipEntry {
        for (self.clips.items) |candidate| {
            if (std.mem.eql(u8, candidate.name, name)) return candidate;
        }
        return null;
    }

    /// Registers `clip` under `name`, replacing a clip of that name.
    pub fn addClip(self: *Player, name: []const u8, clip: ClipHandle) Allocator.Error!void {
        for (self.clips.items) |*candidate| {
            if (std.mem.eql(u8, candidate.name, name)) {
                candidate.clip = clip;
                return;
            }
        }
        try self.clips.append(self.gpa, .{ .name = name, .clip = clip });
    }

    /// Whether a clip is registered under `name`.
    pub fn hasClip(self: *const Player, name: []const u8) bool {
        return self.entry(name) != null;
    }

    /// Sets the speed multiplier of every playback.
    pub fn setSpeedScale(self: *Player, scale: f32) void {
        self.speed_scale = scale;
    }

    /// Sets the blend duration used when a pair has none.
    pub fn setDefaultBlendSeconds(self: *Player, seconds: f32) void {
        self.default_blend_seconds = clip_mod.maxFloat(seconds, 0.0);
    }

    /// Sets the blend duration from clip `from` to clip `to`.
    pub fn setBlendSeconds(self: *Player, from: []const u8, to: []const u8, seconds: f32) Allocator.Error!void {
        for (self.blend_times.items) |*candidate| {
            if (std.mem.eql(u8, candidate.from, from) and std.mem.eql(u8, candidate.to, to)) {
                candidate.seconds = clip_mod.maxFloat(seconds, 0.0);
                return;
            }
        }
        try self.blend_times.append(self.gpa, .{ .from = from, .to = to, .seconds = clip_mod.maxFloat(seconds, 0.0) });
    }

    /// The blend duration from clip `from` to clip `to`.
    fn blendSeconds(self: *const Player, from: []const u8, to: []const u8) f32 {
        for (self.blend_times.items) |candidate| {
            if (std.mem.eql(u8, candidate.from, from) and std.mem.eql(u8, candidate.to, to)) return candidate.seconds;
        }
        return self.default_blend_seconds;
    }

    /// Sets the clip that follows `from` when it finishes.
    pub fn setNext(self: *Player, from: []const u8, to: []const u8) Allocator.Error!void {
        for (self.next_clips.items) |*candidate| {
            if (std.mem.eql(u8, candidate.from, from)) {
                candidate.to = to;
                return;
            }
        }
        try self.next_clips.append(self.gpa, .{ .from = from, .to = to });
    }

    /// The clip that follows `from`, or empty.
    fn next(self: *const Player, from: []const u8) []const u8 {
        for (self.next_clips.items) |candidate| {
            if (std.mem.eql(u8, candidate.from, from)) return candidate.to;
        }
        return "";
    }

    /// Starts clip `name`, or the assigned clip when `name` is empty.
    pub fn play(self: *Player, name: []const u8, options: Play) Allocator.Error!void {
        const wanted = if (name.len == 0) self.assigned else name;
        const found = self.entry(wanted) orelse return;
        if (self.auto_capture and !self.capture_pending and self.auto_capture_duration > 0.0) {
            self.capture_duration = self.auto_capture_duration;
            self.capture_easing_wanted = self.auto_capture_easing;
            self.capture_pending = true;
        }
        const blend = if (options.blend_seconds < 0.0) self.blendSeconds(self.current.name, found.name) else options.blend_seconds;
        if (blend > 0.0 and self.current.clip != invalid_clip and !std.mem.eql(u8, self.current.name, found.name)) {
            try self.blends.append(self.gpa, .{ .playback = self.current, .seconds = blend, .left = blend });
        }
        var playback: Playback = .{};
        const same = std.mem.eql(u8, self.current.name, found.name);
        playback.name = found.name;
        playback.clip = found.clip;
        playback.length = if (same) self.current.length else 0.0;
        playback.speed = options.speed;
        playback.position = if (options.from_end) playback.length else 0.0;
        if (same and self.playing) playback.position = self.current.position;
        self.current = playback;
        self.assigned = self.current.name;
        self.playing = true;
        self.started = true;
        self.finished = false;
        self.seeked = false;
    }

    /// Starts clip `name` backwards from its end.
    pub fn playBackwards(self: *Player, name: []const u8, blend_seconds: f32) Allocator.Error!void {
        try self.play(name, .{ .blend_seconds = blend_seconds, .speed = -1.0, .from_end = true });
    }

    /// Starts clip `name` over the section `range`.
    pub fn playSection(self: *Player, name: []const u8, range: SectionRange, options: Play) Allocator.Error!void {
        self.setSection(range);
        try self.play(name, options);
    }

    /// Starts clip `name` over the section between two markers.
    pub fn playSectionWithMarkers(self: *Player, name: []const u8, markers: MarkerRange, options: Play) Allocator.Error!void {
        self.setSectionWithMarkers(markers);
        try self.play(name, options);
    }

    /// Starts clip `name` and blends out from a captured pose over `duration_seconds`.
    pub fn playWithCapture(self: *Player, name: []const u8, duration_seconds: f32, options: Play, easing: Easing) Allocator.Error!void {
        self.capture_duration = if (duration_seconds < 0.0) self.auto_capture_duration else duration_seconds;
        self.capture_easing_wanted = easing;
        self.capture_pending = self.capture_duration > 0.0;
        try self.play(name, options);
    }

    /// Wants the section `range`.
    pub fn setSection(self: *Player, range: SectionRange) void {
        self.section_from = range.from;
        self.section_to = range.to;
        self.section_from_marker = "";
        self.section_to_marker = "";
        self.section_by_marker = false;
        self.section_wanted = true;
    }

    /// Wants the section between two markers.
    pub fn setSectionWithMarkers(self: *Player, markers: MarkerRange) void {
        self.section_from_marker = markers.from;
        self.section_to_marker = markers.to;
        self.section_from = -1.0;
        self.section_to = -1.0;
        self.section_by_marker = true;
        self.section_wanted = true;
    }

    /// Wants the whole clip again.
    pub fn resetSection(self: *Player) void {
        self.section_wanted = false;
        self.section_by_marker = false;
        self.section_from = -1.0;
        self.section_to = -1.0;
        self.section_from_marker = "";
        self.section_to_marker = "";
        self.section = .{};
    }

    /// Sets whether every `play` captures the current pose.
    pub fn setAutoCapture(self: *Player, capture: bool) void {
        self.auto_capture = capture;
    }

    /// Sets the duration of automatic captures.
    pub fn setAutoCaptureDuration(self: *Player, seconds: f32) void {
        self.auto_capture_duration = clip_mod.maxFloat(seconds, 0.0);
    }

    /// Sets the easing of automatic captures.
    pub fn setAutoCaptureEasing(self: *Player, easing: Easing) void {
        self.auto_capture_easing = easing;
    }

    /// Whether a pose capture is waiting for `World.applyCapture`.
    pub fn capturePending(self: *const Player) bool {
        return self.capture_pending;
    }

    /// Sets the target whose tracks drive root motion, and clears the motion.
    pub fn setRootMotionTarget(self: *Player, target: []const u8) void {
        self.root_motion_target = target;
        self.resetRootMotion();
    }

    /// The root motion summed over every advance.
    pub fn rootMotionAccumulator(self: *const Player) RootMotion {
        return self.root_motion_total;
    }

    /// Clears the root motion of the last advance and the total.
    pub fn resetRootMotion(self: *Player) void {
        self.root_motion = .{};
        self.root_motion_total = .{};
    }

    /// Queues clip `name` to play after the current one finishes.
    pub fn queue(self: *Player, name: []const u8) Allocator.Error!void {
        try self.queue_names.append(self.gpa, name);
    }

    /// Moves the playback to `seconds`.
    pub fn seek(self: *Player, seconds: f32) void {
        self.current.position = seconds;
        self.seeked = true;
    }

    /// Playback position in seconds.
    pub fn position(self: *const Player) f32 {
        return self.current.position;
    }

    /// Name of the playing clip.
    pub fn currentClip(self: *const Player) []const u8 {
        return self.current.name;
    }

    /// Whether the last advance finished the clip.
    pub fn isFinished(self: *const Player) bool {
        return self.finished;
    }
};

/// A player with the samples, event triggers and status changes of its latest advance.
pub const PlayerState = struct {
    player: Player,
    samples: std.ArrayList(Sample) = .empty,
    triggers: std.ArrayList(Trigger) = .empty,
    status: std.ArrayList(StatusChange) = .empty,
};

/// Weighted blend of every sample that targets one property.
const Accumulator = struct {
    target: []const u8,
    kind: TrackKind,
    vector: Vec3 = .{},
    rotation: Quat = .{},
    flag: bool = true,
    weight: f32 = 0.0,
    best: f32 = 0.0,
};

/// Time to sample a track at and the weight of that sample in the blend.
const Contribution = struct {
    time: f32,
    weight: f32,
};

/// `position` wrapped into [0, length), or zero for a non-positive length.
/// Uses floor where the engine uses fmod, which is a libm call.
fn wrappedPosition(position: f32, length: f32) f32 {
    if (length <= 0.0) return 0.0;
    var result = position - @floor(position / length) * length;
    if (result < 0.0) result += length;
    return result;
}

/// `value` without its sign.
fn absolute(value: f32) f32 {
    return if (value < 0.0) -value else value;
}

/// Resolves the section a player wants into clamped times for a clip.
fn resolveSection(player: *const Player, asset: *const Clip) Section {
    const length = clip_mod.maxFloat(asset.duration_seconds, 0.0);
    var section: Section = .{ .from = 0.0, .to = length };
    if (!player.section_wanted) return section;
    if (player.section_by_marker) {
        const from = clip_mod.findMarker(asset, player.section_from_marker);
        const to = clip_mod.findMarker(asset, player.section_to_marker);
        section.from = if (from) |marker| marker.time else 0.0;
        section.to = if (to) |marker| marker.time else length;
    } else {
        section.from = if (player.section_from < 0.0) 0.0 else player.section_from;
        section.to = if (player.section_to < 0.0) length else player.section_to;
    }
    section.from = clip_mod.clampFloat(section.from, 0.0, length);
    section.to = clip_mod.clampFloat(section.to, 0.0, length);
    if (section.to < section.from) std.mem.swap(f32, &section.from, &section.to);
    return section;
}

/// Adds the root target's motion over a range to the player's running total.
fn collectRootMotion(player: *Player, asset: *const Clip, range: Range) void {
    player.root_motion = .{};
    if (player.root_motion_target.len == 0) return;
    for (asset.tracks) |*track| {
        if (!track.enabled or !std.mem.eql(u8, track.target, player.root_motion_target)) continue;
        if (track.kind == .position) {
            const first = clip_mod.sampleVector(track, range.from, .{});
            const last = clip_mod.sampleVector(track, range.to, .{});
            const delta = if (range.wrapped)
                clip_mod.sampleVector(track, range.high, .{}).sub(first).add(last.sub(clip_mod.sampleVector(track, range.low, .{})))
            else
                last.sub(first);
            player.root_motion.position = player.root_motion.position.add(delta);
        } else if (track.kind == .rotation) {
            const first = clip_mod.sampleRotation(track, range.from, .{});
            const last = clip_mod.sampleRotation(track, range.to, .{});
            var delta = clip_mod.conjugate(first).mul(last);
            if (range.wrapped) {
                const high = clip_mod.sampleRotation(track, range.high, .{});
                const low = clip_mod.sampleRotation(track, range.low, .{});
                delta = clip_mod.conjugate(first).mul(high).mul(clip_mod.conjugate(low).mul(last));
            }
            player.root_motion.rotation = player.root_motion.rotation.mul(delta).normalize();
        }
    }
    player.root_motion_total.position = player.root_motion_total.position.add(player.root_motion.position);
    player.root_motion_total.rotation = player.root_motion_total.rotation.mul(player.root_motion.rotation).normalize();
}

/// Owns the clips and players and advances every player by a time step.
pub const World = struct {
    gpa: Allocator,
    clips: std.ArrayList(Clip) = .empty,
    players: []PlayerState = &.{},
    accumulators: std.ArrayList(Accumulator) = .empty,
    key_scratch: std.ArrayList(usize) = .empty,

    /// Frees the clips, the players and the scratch.
    pub fn deinit(self: *World) void {
        for (self.clips.items) |*clip| clip.deinit(self.gpa);
        self.clips.deinit(self.gpa);
        for (self.players) |*state| {
            state.player.deinit();
            state.samples.deinit(self.gpa);
            state.triggers.deinit(self.gpa);
            state.status.deinit(self.gpa);
        }
        self.gpa.free(self.players);
        self.accumulators.deinit(self.gpa);
        self.key_scratch.deinit(self.gpa);
    }

    /// Adds a clip and returns its handle.
    pub fn addClip(self: *World, clip: Clip) Allocator.Error!ClipHandle {
        try self.clips.append(self.gpa, clip);
        return @intCast(self.clips.items.len - 1);
    }

    /// Creates `count` players.
    pub fn createPlayers(self: *World, count: usize) Allocator.Error!void {
        self.players = try self.gpa.alloc(PlayerState, count);
        for (self.players) |*state| state.* = .{ .player = .{ .gpa = self.gpa } };
    }

    /// The clip behind `handle`, or null.
    fn findClip(self: *const World, handle: ClipHandle) ?*const Clip {
        return if (handle < self.clips.items.len) &self.clips.items[handle] else null;
    }

    /// Finds or creates the accumulator of a property.
    fn accumulator(self: *World, target: []const u8, kind: TrackKind) Allocator.Error!*Accumulator {
        for (self.accumulators.items) |*candidate| {
            if (candidate.kind == kind and std.mem.eql(u8, candidate.target, target)) return candidate;
        }
        try self.accumulators.append(self.gpa, .{ .target = target, .kind = kind });
        return &self.accumulators.items[self.accumulators.items.len - 1];
    }

    /// Adds a weighted position or scale sample of a track.
    fn accumulateVector(self: *World, track: *const Track, at: Contribution) Allocator.Error!void {
        const found = try self.accumulator(track.target, track.kind);
        found.vector = found.vector.add(clip_mod.sampleVector(track, at.time, clip_mod.trackRestVector(track.kind)).scale(at.weight));
        found.weight += at.weight;
    }

    /// Blends a rotation sample of a track into its accumulator.
    fn accumulateRotation(self: *World, track: *const Track, at: Contribution) Allocator.Error!void {
        const found = try self.accumulator(track.target, track.kind);
        const value = clip_mod.sampleRotation(track, at.time, .{});
        const total = found.weight + at.weight;
        found.rotation = if (found.weight <= 0.0) value else clip_mod.nlerp(found.rotation, value, if (total > 0.0) at.weight / total else 0.0);
        found.weight = total;
    }

    /// Takes a flag sample of a track when it outweighs those accumulated so far.
    fn accumulateFlag(self: *World, track: *const Track, at: Contribution) Allocator.Error!void {
        const found = try self.accumulator(track.target, track.kind);
        if (at.weight > found.best) {
            found.flag = clip_mod.sampleFlag(track, at.time, true);
            found.best = at.weight;
        }
        found.weight += at.weight;
    }

    /// Accumulates every enabled track of a playback at its position.
    fn samplePlayback(self: *World, playback: Playback, skip_target: []const u8, weight: f32) Allocator.Error!void {
        const asset = self.findClip(playback.clip) orelse return;
        if (weight <= 0.0) return;
        for (asset.tracks) |*track| {
            if (!track.enabled or track.kind == .event) continue;
            if (skip_target.len != 0 and std.mem.eql(u8, track.target, skip_target) and track.kind != .active) continue;
            const at: Contribution = .{ .time = playback.position, .weight = weight };
            switch (track.kind) {
                .position, .scale => try self.accumulateVector(track, at),
                .rotation => try self.accumulateRotation(track, at),
                .active => try self.accumulateFlag(track, at),
                .event => {},
            }
        }
    }

    /// Accumulates a captured pose.
    fn sampleCapture(self: *World, capture: *const Capture, weight: f32) Allocator.Error!void {
        if (weight <= 0.0) return;
        for (capture.samples.items) |sample| {
            const found = try self.accumulator(sample.target, sample.kind);
            const total = found.weight + weight;
            switch (sample.kind) {
                .position, .scale => found.vector = found.vector.add(sample.vector.scale(weight)),
                .rotation => found.rotation = if (found.weight <= 0.0)
                    sample.rotation
                else
                    clip_mod.nlerp(found.rotation, sample.rotation, if (total > 0.0) weight / total else 0.0),
                .active => if (weight > found.best) {
                    found.flag = sample.flag;
                    found.best = weight;
                },
                .event => {},
            }
            found.weight = total;
        }
    }

    /// Converts the accumulators into the player's output samples.
    fn emit(self: *World, state: *PlayerState) Allocator.Error!void {
        for (self.accumulators.items) |accumulated| {
            if (accumulated.weight <= 0.0) continue;
            try state.samples.append(self.gpa, .{
                .target = accumulated.target,
                .kind = accumulated.kind,
                .vector = accumulated.vector.div(accumulated.weight),
                .rotation = accumulated.rotation.normalize(),
                .flag = accumulated.flag,
            });
        }
    }

    /// Records the event keys of the playing clip passed over by `range`.
    fn collectTriggers(self: *World, state: *PlayerState, asset: *const Clip, range: Range) Allocator.Error!void {
        for (asset.tracks) |*track| {
            if (!track.enabled or track.kind != .event) continue;
            try clip_mod.keysInRange(self.gpa, track, range, &self.key_scratch);
            for (self.key_scratch.items) |index| {
                try state.triggers.append(self.gpa, .{
                    .clip = state.player.current.name,
                    .target = track.target,
                    .event = track.keys[index].event,
                    .time = track.keys[index].time,
                });
            }
        }
    }

    /// Advances one player, then rebuilds its samples, triggers and status changes.
    fn advancePlayer(self: *World, state: *PlayerState, delta: f32) Allocator.Error!void {
        const player = &state.player;
        state.samples.clearRetainingCapacity();
        state.triggers.clearRetainingCapacity();
        state.status.clearRetainingCapacity();
        player.finished = false;
        if (!player.enabled) return;
        const asset = self.findClip(player.current.clip) orelse return;
        if (player.started) {
            player.started = false;
            try state.status.append(self.gpa, .{ .clip = player.current.name, .status = .started });
        }
        const section = resolveSection(player, asset);
        player.section = section;
        player.current.length = asset.duration_seconds;
        const seeked = player.seeked;
        player.seeked = false;

        const step: f32 = if (player.playing and !seeked) delta * player.speed_scale * player.current.speed else 0.0;
        const from = clip_mod.clampFloat(player.current.position, section.from, section.to);
        var to = from + step;
        var wrapped = false;
        if (step != 0.0) {
            const span = section.to - section.from;
            switch (asset.loop) {
                .none => {
                    if (to >= section.to) {
                        to = section.to;
                        player.finished = true;
                    } else if (to <= section.from) {
                        to = section.from;
                        player.finished = true;
                    }
                },
                .linear => {
                    if (to > section.to or to < section.from) {
                        to = section.from + wrappedPosition(to - section.from, span);
                        wrapped = true;
                    }
                },
                .ping_pong => {
                    if (to > section.to) {
                        to = section.to;
                        player.current.speed = -player.current.speed;
                    } else if (to < section.from) {
                        to = section.from;
                        player.current.speed = -player.current.speed;
                    }
                },
            }
        }
        player.current.position = clip_mod.clampFloat(to, section.from, section.to);

        var blended: f32 = 0.0;
        for (player.blends.items) |*blend| {
            blend.left = clip_mod.maxFloat(blend.left - absolute(delta), 0.0);
            blend.playback.position = clip_mod.clampFloat(
                blend.playback.position + (delta * player.speed_scale * blend.playback.speed),
                0.0,
                blend.playback.length,
            );
            blended += if (blend.seconds > 0.0) blend.left / blend.seconds else 0.0;
        }
        removeFinishedBlends(&player.blends);
        var captured: f32 = 0.0;
        if (player.capture.left > 0.0 and player.capture.seconds > 0.0) {
            player.capture.left = clip_mod.maxFloat(player.capture.left - absolute(delta), 0.0);
            captured = clip_mod.easeCurve(player.capture.left / player.capture.seconds, player.capture.easing);
            if (player.capture.left <= 0.0) player.capture.clear();
        }
        blended = clip_mod.clampFloat(blended + captured, 0.0, 1.0);

        self.accumulators.clearRetainingCapacity();
        try self.samplePlayback(player.current, player.root_motion_target, 1.0 - blended);
        for (player.blends.items) |blend| {
            const weight: f32 = if (blend.seconds > 0.0) blend.left / blend.seconds else 0.0;
            try self.samplePlayback(blend.playback, player.root_motion_target, weight);
        }
        try self.sampleCapture(&player.capture, captured);
        try self.emit(state);

        const range: Range = .{ .from = from, .to = player.current.position, .low = section.from, .high = section.to, .wrapped = wrapped };
        const motion_range: Range = if (step != 0.0) range else .{ .from = from, .to = from, .low = section.from, .high = section.to, .wrapped = false };
        collectRootMotion(player, asset, motion_range);

        if (step != 0.0 and blended < 1.0) try self.collectTriggers(state, asset, range);

        player.capture_pending = false;
        if (!player.finished) return;
        try state.status.append(self.gpa, .{ .clip = player.current.name, .status = .finished });
        const following = player.next(player.current.name);
        if (following.len != 0 and player.hasClip(following)) {
            try player.play(following, .{});
        } else if (player.queue_names.items.len != 0) {
            const wanted = player.queue_names.orderedRemove(0);
            try player.play(wanted, .{});
        } else {
            player.playing = false;
            return;
        }
        if (player.started) {
            player.started = false;
            try state.status.append(self.gpa, .{ .clip = player.current.name, .status = .started });
        }
    }

    /// Advances every player by `delta_seconds`, rebuilding its samples, triggers and status.
    pub fn advance(self: *World, delta_seconds: f32) Allocator.Error!void {
        for (self.players) |*state| try self.advancePlayer(state, delta_seconds);
    }

    /// Stores the player's latest samples as the pose its pending capture blends out from.
    pub fn applyCapture(self: *World, state: *PlayerState) Allocator.Error!void {
        const player = &state.player;
        if (!player.capture_pending) return;
        player.capture.samples.clearRetainingCapacity();
        try player.capture.samples.appendSlice(self.gpa, state.samples.items);
        player.capture.seconds = player.capture_duration;
        player.capture.left = player.capture_duration;
        player.capture.easing = player.capture_easing_wanted;
        player.capture_pending = false;
        player.blends.clearRetainingCapacity();
    }
};

/// Drops every blend whose time has run out, keeping the order of the rest.
fn removeFinishedBlends(blends: *std.ArrayList(Blend)) void {
    var kept: usize = 0;
    for (blends.items) |blend| {
        if (blend.left > 0.0) {
            blends.items[kept] = blend;
            kept += 1;
        }
    }
    blends.shrinkRetainingCapacity(kept);
}
