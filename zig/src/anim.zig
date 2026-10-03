//! Animation playback: 1024 players of 3 clips each, every clip 8 tracks of 48
//! keys, stepped for 48 frames of 1/30 s. The engine's `AnimationWorld`
//! samples every playing clip and every blend from the previous clip, by key
//! search, easing and interpolation, accumulates the samples by target path
//! in a string-keyed accumulator list, emits per-target samples, sums root
//! motion, fires event keys, and chains to the next or queued clip. A scripted
//! driver changes clips, blends from captured poses, queues, seeks and plays
//! sections. Differences from the engine: slerp is a normalised lerp, sine and
//! exponential easing are polynomials, and `fmod` is a floor; each keeps the
//! kernel free of libm calls, which round differently between languages.
const std = @import("std");
const animation = @import("animation.zig");
const clip_mod = @import("animation_clip.zig");
const hash = @import("hash.zig");
const vm = @import("vecmath.zig");

const Allocator = std.mem.Allocator;
const Vec3 = vm.Vec3;
const Clip = clip_mod.Clip;
const Track = clip_mod.Track;
const Key = clip_mod.Key;
const Easing = clip_mod.Easing;
const Anim = @This();

pub const name = "anim";

const player_count: usize = 1024;
const frame_count: u32 = 48;
const set_count: usize = 8;
const clips_per_player: usize = 3;
const key_count: usize = 48;
const frame_seconds: f32 = @as(f32, 1.0) / @as(f32, 30.0);
const digest_prime: u64 = 0x9e3779b97f4a7c15;

const clip_names = [clips_per_player][]const u8{ "idle", "walk", "jump" };
const event_names = [4][]const u8{ "step", "splash", "dust", "whoosh" };
const root_path = "armature/root";
const marker_names = [3][]const u8{ "foot_l", "foot_r", "land" };
const marker_fractions = [3]f32{ 0.2, 0.6, 0.9 };

/// What one track of every clip animates.
const TrackSpec = struct {
    kind: clip_mod.TrackKind,
    interpolation: clip_mod.Interpolation,
    target: []const u8,
    travels: bool,
};

const track_specs = [8]TrackSpec{
    .{ .kind = .position, .interpolation = .linear, .target = root_path, .travels = true },
    .{ .kind = .rotation, .interpolation = .linear, .target = root_path, .travels = false },
    .{ .kind = .position, .interpolation = .cubic, .target = "armature/root/hips", .travels = false },
    .{ .kind = .rotation, .interpolation = .cubic, .target = "armature/root/hips", .travels = false },
    .{ .kind = .rotation, .interpolation = .linear, .target = "armature/root/hips/spine/chest", .travels = false },
    .{ .kind = .scale, .interpolation = .linear, .target = "armature/root/hips/spine/chest/head", .travels = false },
    .{ .kind = .active, .interpolation = .nearest, .target = "armature/root/hips/spine/chest/arm_l/hand_l", .travels = false },
    .{ .kind = .event, .interpolation = .nearest, .target = "armature/fx", .travels = false },
};

world: animation.World,
digest: []u64,

/// Draws the clips and creates the players.
pub fn init(gpa: Allocator) !Anim {
    var rng: hash.Rng = .{ .s = 0xa41 };
    var world: animation.World = .{ .gpa = gpa };
    errdefer world.deinit();
    for (0..set_count) |set| {
        const variant: f32 = @floatFromInt(set);
        _ = try world.addClip(try randomClip(gpa, &rng, .linear, 1.6 + 0.1 * variant));
        _ = try world.addClip(try randomClip(gpa, &rng, .ping_pong, 1.2 + 0.05 * variant));
        _ = try world.addClip(try randomClip(gpa, &rng, .none, 0.9 + 0.02 * variant));
    }
    try world.createPlayers(player_count);
    const digest = try gpa.alloc(u64, player_count);
    return .{ .world = world, .digest = digest };
}

/// Frees the clips, the players and the digests.
pub fn deinit(self: *Anim, gpa: Allocator) void {
    self.world.deinit();
    gpa.free(self.digest);
}

/// Folds `value` into the running digest `state`.
fn fold(state: u64, value: u64) u64 {
    const mixed = (state ^ value) *% digest_prime;
    return mixed ^ (mixed >> 32);
}

/// Two floats' bit patterns in one word.
fn pack(low: f32, high: f32) u64 {
    return @as(u64, hash.f32Bits(low)) | (@as(u64, hash.f32Bits(high)) << 32);
}

/// Folds the samples, triggers and status changes of one advance into `state`.
fn foldPlayer(state_in: u64, player: *const animation.PlayerState) u64 {
    var state = state_in;
    for (player.samples.items) |sample| {
        const tag = @as(u64, @intFromBool(sample.flag)) | (@as(u64, @intFromEnum(sample.kind)) << 1) | (@as(u64, sample.target.len) << 8);
        state = fold(state, pack(sample.vector.x, sample.vector.y));
        state = fold(state, pack(sample.vector.z, sample.rotation.x));
        state = fold(state, pack(sample.rotation.y, sample.rotation.z));
        state = fold(state, pack(sample.rotation.w, 0.0) | (tag << 32));
    }
    for (player.triggers.items) |trigger| {
        const tag = @as(u64, trigger.event[0]) | (@as(u64, trigger.target.len) << 8) | (@as(u64, trigger.clip.len) << 16);
        state = fold(state, pack(trigger.time, 0.0) | (tag << 32));
    }
    for (player.status.items) |change| {
        state = fold(state, (@as(u64, change.clip.len) << 1) | @as(u64, @intFromEnum(change.status)));
    }
    return fold(state, @as(u64, player.samples.items.len) | (@as(u64, player.triggers.items.len) << 16) |
        (@as(u64, @intFromBool(player.player.isFinished())) << 32));
}

/// The key time of key `index`, jittered by `unit` except at both ends.
fn keyTime(index: usize, spacing: f32, duration: f32, unit: f32) f32 {
    if (index == 0) return 0.0;
    if (index == key_count - 1) return duration;
    return (@as(f32, @floatFromInt(index)) + (unit - 0.5) * 0.5) * spacing;
}

/// A random easing curve.
fn randomEasing(rng: *hash.Rng) Easing {
    const transition = rng.next() % 7;
    const ease = rng.next() % 4;
    return .{
        .transition = @enumFromInt(transition),
        .ease = @enumFromInt(ease),
    };
}

/// Key `index` of a track made to `spec`.
fn randomKey(rng: *hash.Rng, spec: TrackSpec, index: usize, spacing: f32, duration: f32) Key {
    var key: Key = .{};
    const unit = rng.unit();
    key.time = keyTime(index, spacing, duration, unit);
    const position: f32 = @floatFromInt(index);
    switch (spec.kind) {
        .position => {
            if (spec.travels) {
                const jitter = Vec3.random(rng, -0.05, 0.05);
                key.vector = jitter.add(.{ .x = position * 0.1, .y = 0.0, .z = position * 0.2 });
            } else {
                key.vector = Vec3.random(rng, -1.0, 1.0);
            }
        },
        .scale => key.vector = Vec3.random(rng, 0.8, 1.2),
        .rotation => key.rotation = vm.Quat.random(rng),
        .active => key.flag = rng.unit() < 0.5,
        .event => key.event = event_names[rng.next() % event_names.len],
    }
    key.easing = randomEasing(rng);
    return key;
}

/// A clip of every track spec with `key_count` keys each over `duration` seconds.
fn randomClip(gpa: Allocator, rng: *hash.Rng, loop: clip_mod.LoopMode, duration: f32) !Clip {
    const spacing = duration / @as(f32, @floatFromInt(key_count - 1));
    const tracks = try gpa.alloc(Track, track_specs.len);
    errdefer gpa.free(tracks);
    var made: usize = 0;
    errdefer for (tracks[0..made]) |track| gpa.free(track.keys);
    for (track_specs, 0..) |spec, slot| {
        const keys = try gpa.alloc(Key, key_count);
        for (keys, 0..) |*key, index| key.* = randomKey(rng, spec, index, spacing, duration);
        tracks[slot] = .{ .kind = spec.kind, .interpolation = spec.interpolation, .target = spec.target, .enabled = true, .keys = keys };
        made += 1;
    }
    const markers = try gpa.alloc(clip_mod.Marker, marker_names.len);
    for (markers, 0..) |*marker, slot| marker.* = .{ .name = marker_names[slot], .time = duration * marker_fractions[slot] };
    return .{ .duration_seconds = duration, .loop = loop, .tracks = tracks, .markers = markers };
}

/// A value in [0, 1) from 16 bits of `bits`.
fn unitFromBits(bits: u64) f32 {
    return @as(f32, @floatFromInt(bits & 0xFFFF)) * (1.0 / 65536.0);
}

/// Returns player `index` to its initial clips, blends and playback.
fn configurePlayer(state: *animation.PlayerState, index: usize) !void {
    state.samples.clearRetainingCapacity();
    state.triggers.clearRetainingCapacity();
    state.status.clearRetainingCapacity();
    const player = &state.player;
    player.reset();
    const set = index % set_count;
    for (clip_names, 0..) |clip_name, c| try player.addClip(clip_name, @intCast(set * clips_per_player + c));
    player.setDefaultBlendSeconds(0.25);
    try player.setBlendSeconds("idle", "walk", 0.3);
    try player.setBlendSeconds("walk", "jump", 0.1);
    try player.setBlendSeconds("jump", "walk", 0.15);
    try player.setBlendSeconds("walk", "idle", 0.4);
    try player.setNext("jump", "walk");
    try player.setNext("walk", "idle");
    player.setRootMotionTarget(root_path);
    player.setSpeedScale(0.75 + @as(f32, @floatFromInt(index % 5)) * 0.125);
    if (index % 4 == 0) {
        player.setAutoCapture(true);
        player.setAutoCaptureDuration(0.2);
        player.setAutoCaptureEasing(.{ .transition = .quad, .ease = .out });
    }
    try player.play(clip_names[index % clips_per_player], .{});
}

/// Applies the scripted clip change, if any, of player `index` at `frame`.
fn scriptPlayer(world: *animation.World, index: usize, frame: u32) !void {
    const roll = hash.mix64((@as(u64, index) << 32) | frame);
    if (roll & 31 != 0) return;
    const state = &world.players[index];
    const player = &state.player;
    const pick = clip_names[(roll >> 16) % clips_per_player];
    const unit = unitFromBits(roll >> 24);
    switch ((roll >> 8) & 7) {
        0 => {
            player.resetSection();
            try player.play(pick, .{});
        },
        1 => try player.playWithCapture(pick, 0.2 + 0.2 * unit, .{}, .{ .transition = .cubic, .ease = .in_out }),
        2 => try player.queue(pick),
        3 => try player.playBackwards(pick, 0.15),
        4 => try player.playSectionWithMarkers(pick, .{ .from = "foot_l", .to = "land" }, .{}),
        5 => player.seek(unit * 1.5),
        6 => player.setSpeedScale(0.5 + unit),
        else => try player.playSection(pick, .{ .from = 0.2 + 0.2 * unit, .to = 0.9 }, .{}),
    }
    if (player.capturePending()) try world.applyCapture(state);
}

/// Resets every player, steps every frame, and hashes the digests, root motion and positions.
pub fn run(self: *Anim) !u64 {
    const players = self.world.players;
    for (players, 0..) |*state, index| {
        try configurePlayer(state, index);
        self.digest[index] = 0;
    }
    for (0..frame_count) |frame| {
        for (0..players.len) |index| try scriptPlayer(&self.world, index, @intCast(frame));
        try self.world.advance(frame_seconds);
        for (players, self.digest) |*state, *digest| digest.* = foldPlayer(digest.*, state);
    }
    var h: u64 = 0;
    for (players, self.digest) |*state, digest| {
        const player = &state.player;
        const root = player.rootMotionAccumulator();
        h = hash.add(h, digest);
        h = hash.add(h, pack(root.position.x, root.position.y));
        h = hash.add(h, pack(root.position.z, root.rotation.x));
        h = hash.add(h, pack(root.rotation.y, root.rotation.z));
        h = hash.add(h, pack(root.rotation.w, player.position()));
        h = hash.add(h, player.currentClip().len);
    }
    return h;
}
