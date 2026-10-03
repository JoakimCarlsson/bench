//! The engine's animation players and the world that advances them, from
//! animation.hpp and animation.cpp. Clips are held by index instead of the
//! engine's slot map, and names are static strings.
use crate::animation_clip::{
    clamp_float, conjugate, ease_curve, find_marker, keys_in_range, max_float, nlerp, sample_flag, sample_rotation,
    sample_vector, track_rest_vector, Clip, Easing, LoopMode, Range, Track, TrackKind,
};
use crate::vecmath::{Quat, Vec3};

/// Index of a clip in the world; `INVALID_CLIP` refers to none.
pub type ClipHandle = u32;
pub const INVALID_CLIP: ClipHandle = u32::MAX;

/// Blend duration that asks for the player's configured one.
pub const BLEND_FROM_DEFAULT: f32 = -1.0;

/// A clip registered on a player under a name.
#[derive(Clone, Copy)]
struct ClipEntry {
    name: &'static str,
    clip: ClipHandle,
}

/// The value of one animated property, as emitted by an advance.
#[derive(Clone, Copy)]
pub struct Sample {
    pub target: &'static str,
    pub kind: TrackKind,
    pub vector: Vec3,
    pub rotation: Quat,
    pub flag: bool,
}

/// Options of a `play` call.
#[derive(Clone, Copy)]
pub struct Play {
    pub blend_seconds: f32,
    pub speed: f32,
    pub from_end: bool,
}

impl Default for Play {
    /// The configured blend, forwards, from the start.
    fn default() -> Play {
        Play { blend_seconds: BLEND_FROM_DEFAULT, speed: 1.0, from_end: false }
    }
}

/// The span of a clip that plays.
#[derive(Clone, Copy, Default)]
struct Section {
    from: f32,
    to: f32,
}

/// A wanted section in seconds; negative ends mean the clip's own.
#[derive(Clone, Copy)]
pub struct SectionRange {
    pub from: f32,
    pub to: f32,
}

/// A wanted section between two markers.
#[derive(Clone, Copy)]
pub struct MarkerRange {
    pub from: &'static str,
    pub to: &'static str,
}

/// Playback events reported by an advance.
#[derive(Clone, Copy, PartialEq, Eq)]
pub enum Status {
    Started,
    Finished,
}

/// A clip starting or finishing.
#[derive(Clone, Copy)]
pub struct StatusChange {
    pub clip: &'static str,
    pub status: Status,
}

/// Motion of the root target over an advance.
#[derive(Clone, Copy, Default)]
pub struct RootMotion {
    pub position: Vec3,
    pub rotation: Quat,
}

/// An event key passed over by an advance.
#[derive(Clone, Copy)]
pub struct Trigger {
    pub clip: &'static str,
    pub target: &'static str,
    pub event: &'static str,
    pub time: f32,
}

/// What is playing: a clip, how far in, and how fast.
#[derive(Clone, Copy)]
struct Playback {
    name: &'static str,
    clip: ClipHandle,
    position: f32,
    length: f32,
    speed: f32,
}

impl Default for Playback {
    /// Nothing playing, at unit speed.
    fn default() -> Playback {
        Playback { name: "", clip: INVALID_CLIP, position: 0.0, length: 0.0, speed: 1.0 }
    }
}

/// A previous playback fading out.
#[derive(Clone, Copy)]
struct Blend {
    playback: Playback,
    seconds: f32,
    left: f32,
}

/// A captured pose fading out.
#[derive(Default)]
struct Capture {
    samples: Vec<Sample>,
    seconds: f32,
    left: f32,
    easing: Easing,
}

impl Capture {
    /// Drops the pose and the fade, keeping storage.
    fn clear(&mut self) {
        self.samples.clear();
        self.seconds = 0.0;
        self.left = 0.0;
        self.easing = Easing::default();
    }
}

/// A blend duration between two clips.
struct BlendEntry {
    from: &'static str,
    to: &'static str,
    seconds: f32,
}

/// The clip that follows another when it finishes.
struct NextEntry {
    from: &'static str,
    to: &'static str,
}

/// Plays clips with blends, queueing, sections, pose capture and root motion.
pub struct Player {
    clips: Vec<ClipEntry>,
    blend_times: Vec<BlendEntry>,
    next_clips: Vec<NextEntry>,
    queue: Vec<&'static str>,
    blends: Vec<Blend>,
    capture: Capture,
    current: Playback,
    section: Section,
    root_motion: RootMotion,
    root_motion_total: RootMotion,
    auto_capture_easing: Easing,
    capture_easing_wanted: Easing,
    assigned: &'static str,
    section_from_marker: &'static str,
    section_to_marker: &'static str,
    root_motion_target: &'static str,
    section_from: f32,
    section_to: f32,
    speed_scale: f32,
    default_blend_seconds: f32,
    capture_duration: f32,
    auto_capture_duration: f32,
    section_wanted: bool,
    section_by_marker: bool,
    auto_capture: bool,
    capture_pending: bool,
    enabled: bool,
    playing: bool,
    seeked: bool,
    started: bool,
    finished: bool,
}

impl Default for Player {
    /// A player with no clips, wanting no section.
    fn default() -> Player {
        Player {
            clips: Vec::new(),
            blend_times: Vec::new(),
            next_clips: Vec::new(),
            queue: Vec::new(),
            blends: Vec::new(),
            capture: Capture::default(),
            current: Playback::default(),
            section: Section::default(),
            root_motion: RootMotion::default(),
            root_motion_total: RootMotion::default(),
            auto_capture_easing: Easing::default(),
            capture_easing_wanted: Easing::default(),
            assigned: "",
            section_from_marker: "",
            section_to_marker: "",
            root_motion_target: "",
            section_from: -1.0,
            section_to: -1.0,
            speed_scale: 1.0,
            default_blend_seconds: 0.0,
            capture_duration: 0.0,
            auto_capture_duration: 0.0,
            section_wanted: false,
            section_by_marker: false,
            auto_capture: false,
            capture_pending: false,
            enabled: true,
            playing: false,
            seeked: false,
            started: false,
            finished: false,
        }
    }
}

impl Player {
    /// Returns the player to its freshly constructed state, keeping storage.
    pub fn reset(&mut self) {
        self.clips.clear();
        self.blend_times.clear();
        self.next_clips.clear();
        self.queue.clear();
        self.blends.clear();
        self.capture.clear();
        self.current = Playback::default();
        self.section = Section::default();
        self.root_motion = RootMotion::default();
        self.root_motion_total = RootMotion::default();
        self.auto_capture_easing = Easing::default();
        self.capture_easing_wanted = Easing::default();
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

    /// The clip registered under `name`, if any.
    fn entry(&self, name: &str) -> Option<ClipEntry> {
        self.clips.iter().find(|candidate| candidate.name == name).copied()
    }

    /// Registers `clip` under `name`, replacing a clip of that name.
    pub fn add_clip(&mut self, name: &'static str, clip: ClipHandle) {
        for candidate in self.clips.iter_mut() {
            if candidate.name == name {
                candidate.clip = clip;
                return;
            }
        }
        self.clips.push(ClipEntry { name, clip });
    }

    /// Whether a clip is registered under `name`.
    pub fn has_clip(&self, name: &str) -> bool {
        self.entry(name).is_some()
    }

    /// Sets the speed multiplier of every playback.
    pub fn set_speed_scale(&mut self, scale: f32) {
        self.speed_scale = scale;
    }

    /// Sets the blend duration used when a pair has none.
    pub fn set_default_blend_seconds(&mut self, seconds: f32) {
        self.default_blend_seconds = max_float(seconds, 0.0);
    }

    /// Sets the blend duration from clip `from` to clip `to`.
    pub fn set_blend_seconds(&mut self, from: &'static str, to: &'static str, seconds: f32) {
        for candidate in self.blend_times.iter_mut() {
            if candidate.from == from && candidate.to == to {
                candidate.seconds = max_float(seconds, 0.0);
                return;
            }
        }
        self.blend_times.push(BlendEntry { from, to, seconds: max_float(seconds, 0.0) });
    }

    /// The blend duration from clip `from` to clip `to`.
    fn blend_seconds(&self, from: &str, to: &str) -> f32 {
        for candidate in self.blend_times.iter() {
            if candidate.from == from && candidate.to == to {
                return candidate.seconds;
            }
        }
        self.default_blend_seconds
    }

    /// Sets the clip that follows `from` when it finishes.
    pub fn set_next(&mut self, from: &'static str, to: &'static str) {
        for candidate in self.next_clips.iter_mut() {
            if candidate.from == from {
                candidate.to = to;
                return;
            }
        }
        self.next_clips.push(NextEntry { from, to });
    }

    /// The clip that follows `from`, or empty.
    fn next(&self, from: &str) -> &'static str {
        for candidate in self.next_clips.iter() {
            if candidate.from == from {
                return candidate.to;
            }
        }
        ""
    }

    /// Starts clip `name`, or the assigned clip when `name` is empty.
    pub fn play(&mut self, name: &str, options: Play) {
        let wanted = if name.is_empty() { self.assigned } else { name };
        let Some(found) = self.entry(wanted) else {
            return;
        };
        if self.auto_capture && !self.capture_pending && self.auto_capture_duration > 0.0 {
            self.capture_duration = self.auto_capture_duration;
            self.capture_easing_wanted = self.auto_capture_easing;
            self.capture_pending = true;
        }
        let blend =
            if options.blend_seconds < 0.0 { self.blend_seconds(self.current.name, found.name) } else { options.blend_seconds };
        if blend > 0.0 && self.current.clip != INVALID_CLIP && self.current.name != found.name {
            self.blends.push(Blend { playback: self.current, seconds: blend, left: blend });
        }
        let mut playback = Playback::default();
        playback.name = found.name;
        playback.clip = found.clip;
        playback.length = if self.current.name == found.name { self.current.length } else { 0.0 };
        playback.speed = options.speed;
        playback.position = if options.from_end { playback.length } else { 0.0 };
        if self.current.name == found.name && self.playing {
            playback.position = self.current.position;
        }
        self.current = playback;
        self.assigned = self.current.name;
        self.playing = true;
        self.started = true;
        self.finished = false;
        self.seeked = false;
    }

    /// Starts clip `name` backwards from its end.
    pub fn play_backwards(&mut self, name: &str, blend_seconds: f32) {
        self.play(name, Play { blend_seconds, speed: -1.0, from_end: true });
    }

    /// Starts clip `name` over the section `range`.
    pub fn play_section(&mut self, name: &str, range: SectionRange, options: Play) {
        self.set_section(range);
        self.play(name, options);
    }

    /// Starts clip `name` over the section between two markers.
    pub fn play_section_with_markers(&mut self, name: &str, markers: MarkerRange, options: Play) {
        self.set_section_with_markers(markers);
        self.play(name, options);
    }

    /// Starts clip `name` and blends out from a captured pose over `duration_seconds`.
    pub fn play_with_capture(&mut self, name: &str, duration_seconds: f32, options: Play, easing: Easing) {
        self.capture_duration = if duration_seconds < 0.0 { self.auto_capture_duration } else { duration_seconds };
        self.capture_easing_wanted = easing;
        self.capture_pending = self.capture_duration > 0.0;
        self.play(name, options);
    }

    /// Wants the section `range`.
    pub fn set_section(&mut self, range: SectionRange) {
        self.section_from = range.from;
        self.section_to = range.to;
        self.section_from_marker = "";
        self.section_to_marker = "";
        self.section_by_marker = false;
        self.section_wanted = true;
    }

    /// Wants the section between two markers.
    pub fn set_section_with_markers(&mut self, markers: MarkerRange) {
        self.section_from_marker = markers.from;
        self.section_to_marker = markers.to;
        self.section_from = -1.0;
        self.section_to = -1.0;
        self.section_by_marker = true;
        self.section_wanted = true;
    }

    /// Wants the whole clip again.
    pub fn reset_section(&mut self) {
        self.section_wanted = false;
        self.section_by_marker = false;
        self.section_from = -1.0;
        self.section_to = -1.0;
        self.section_from_marker = "";
        self.section_to_marker = "";
        self.section = Section::default();
    }

    /// Sets whether every `play` captures the current pose.
    pub fn set_auto_capture(&mut self, capture: bool) {
        self.auto_capture = capture;
    }

    /// Sets the duration of automatic captures.
    pub fn set_auto_capture_duration(&mut self, seconds: f32) {
        self.auto_capture_duration = max_float(seconds, 0.0);
    }

    /// Sets the easing of automatic captures.
    pub fn set_auto_capture_easing(&mut self, easing: Easing) {
        self.auto_capture_easing = easing;
    }

    /// Whether a pose capture is waiting for `World::apply_capture`.
    pub fn capture_pending(&self) -> bool {
        self.capture_pending
    }

    /// Sets the target whose tracks drive root motion, and clears the motion.
    pub fn set_root_motion_target(&mut self, target: &'static str) {
        self.root_motion_target = target;
        self.reset_root_motion();
    }

    /// The root motion summed over every advance.
    pub fn root_motion_accumulator(&self) -> &RootMotion {
        &self.root_motion_total
    }

    /// Clears the root motion of the last advance and the total.
    pub fn reset_root_motion(&mut self) {
        self.root_motion = RootMotion::default();
        self.root_motion_total = RootMotion::default();
    }

    /// Queues clip `name` to play after the current one finishes.
    pub fn queue(&mut self, name: &'static str) {
        self.queue.push(name);
    }

    /// Moves the playback to `seconds`.
    pub fn seek(&mut self, seconds: f32) {
        self.current.position = seconds;
        self.seeked = true;
    }

    /// Playback position in seconds.
    pub fn position(&self) -> f32 {
        self.current.position
    }

    /// Name of the playing clip.
    pub fn current_clip(&self) -> &'static str {
        self.current.name
    }

    /// Whether the last advance finished the clip.
    pub fn finished(&self) -> bool {
        self.finished
    }
}

/// A player with the samples, event triggers and status changes of its latest advance.
#[derive(Default)]
pub struct PlayerState {
    pub player: Player,
    pub samples: Vec<Sample>,
    pub triggers: Vec<Trigger>,
    pub status: Vec<StatusChange>,
}

/// Weighted blend of every sample that targets one property.
struct Accumulator {
    target: &'static str,
    kind: TrackKind,
    vector: Vec3,
    rotation: Quat,
    flag: bool,
    weight: f32,
    best: f32,
}

/// Time to sample a track at and the weight of that sample in the blend.
#[derive(Clone, Copy)]
struct Contribution {
    time: f32,
    weight: f32,
}

/// The scratch used while sampling: the accumulators of the player being advanced.
#[derive(Default)]
struct Sampler {
    accumulators: Vec<Accumulator>,
    key_scratch: Vec<usize>,
}

/// Owns the clips and players and advances every player by a time step.
#[derive(Default)]
pub struct World {
    clips: Vec<Clip>,
    pub players: Vec<PlayerState>,
    sampler: Sampler,
}

/// `position` wrapped into [0, length), or zero for a non-positive length.
/// Uses floor where the engine uses fmod, which is a libm call.
fn wrapped_position(position: f32, length: f32) -> f32 {
    if length <= 0.0 {
        return 0.0;
    }
    let mut result = position - (position / length).floor() * length;
    if result < 0.0 {
        result += length;
    }
    result
}

/// `value` without its sign.
fn absolute(value: f32) -> f32 {
    if value < 0.0 { -value } else { value }
}

/// The clip behind `handle`, if any.
fn find_clip(clips: &[Clip], handle: ClipHandle) -> Option<&Clip> {
    clips.get(handle as usize)
}

/// Resolves the section a player wants into clamped times for a clip.
fn resolve_section(player: &Player, asset: &Clip) -> Section {
    let length = max_float(asset.duration_seconds, 0.0);
    let mut section = Section { from: 0.0, to: length };
    if !player.section_wanted {
        return section;
    }
    if player.section_by_marker {
        let from = find_marker(asset, player.section_from_marker);
        let to = find_marker(asset, player.section_to_marker);
        section.from = from.map_or(0.0, |marker| marker.time);
        section.to = to.map_or(length, |marker| marker.time);
    } else {
        section.from = if player.section_from < 0.0 { 0.0 } else { player.section_from };
        section.to = if player.section_to < 0.0 { length } else { player.section_to };
    }
    section.from = clamp_float(section.from, 0.0, length);
    section.to = clamp_float(section.to, 0.0, length);
    if section.to < section.from {
        std::mem::swap(&mut section.from, &mut section.to);
    }
    section
}

impl Sampler {
    /// Finds or creates the accumulator of a property.
    fn accumulator(&mut self, target: &'static str, kind: TrackKind) -> &mut Accumulator {
        let found = self.accumulators.iter().position(|candidate| candidate.kind == kind && candidate.target == target);
        let index = match found {
            Some(index) => index,
            None => {
                self.accumulators.push(Accumulator {
                    target,
                    kind,
                    vector: Vec3::default(),
                    rotation: Quat::default(),
                    flag: true,
                    weight: 0.0,
                    best: 0.0,
                });
                self.accumulators.len() - 1
            }
        };
        &mut self.accumulators[index]
    }

    /// Adds a weighted position or scale sample of a track.
    fn accumulate_vector(&mut self, track: &Track, at: Contribution) {
        let sampled = sample_vector(track, at.time, track_rest_vector(track.kind));
        let found = self.accumulator(track.target, track.kind);
        found.vector = found.vector + (sampled * at.weight);
        found.weight += at.weight;
    }

    /// Blends a rotation sample of a track into its accumulator.
    fn accumulate_rotation(&mut self, track: &Track, at: Contribution) {
        let value = sample_rotation(track, at.time, Quat::default());
        let found = self.accumulator(track.target, track.kind);
        let total = found.weight + at.weight;
        found.rotation = if found.weight <= 0.0 {
            value
        } else {
            nlerp(found.rotation, value, if total > 0.0 { at.weight / total } else { 0.0 })
        };
        found.weight = total;
    }

    /// Takes a flag sample of a track when it outweighs those accumulated so far.
    fn accumulate_flag(&mut self, track: &Track, at: Contribution) {
        let found = self.accumulator(track.target, track.kind);
        if at.weight > found.best {
            found.flag = sample_flag(track, at.time, true);
            found.best = at.weight;
        }
        found.weight += at.weight;
    }

    /// Accumulates every enabled track of a playback at its position.
    fn sample_playback(&mut self, clips: &[Clip], playback: &Playback, skip_target: &str, weight: f32) {
        let Some(asset) = find_clip(clips, playback.clip) else {
            return;
        };
        if weight <= 0.0 {
            return;
        }
        for track in asset.tracks.iter() {
            if !track.enabled || track.kind == TrackKind::Event {
                continue;
            }
            if !skip_target.is_empty() && track.target == skip_target && track.kind != TrackKind::Active {
                continue;
            }
            let at = Contribution { time: playback.position, weight };
            match track.kind {
                TrackKind::Position | TrackKind::Scale => self.accumulate_vector(track, at),
                TrackKind::Rotation => self.accumulate_rotation(track, at),
                TrackKind::Active => self.accumulate_flag(track, at),
                TrackKind::Event => {}
            }
        }
    }

    /// Accumulates a captured pose.
    fn sample_capture(&mut self, capture: &Capture, weight: f32) {
        if weight <= 0.0 {
            return;
        }
        for sample in capture.samples.iter() {
            let found = self.accumulator(sample.target, sample.kind);
            let total = found.weight + weight;
            match sample.kind {
                TrackKind::Position | TrackKind::Scale => {
                    found.vector = found.vector + (sample.vector * weight);
                }
                TrackKind::Rotation => {
                    found.rotation = if found.weight <= 0.0 {
                        sample.rotation
                    } else {
                        nlerp(found.rotation, sample.rotation, if total > 0.0 { weight / total } else { 0.0 })
                    };
                }
                TrackKind::Active => {
                    if weight > found.best {
                        found.flag = sample.flag;
                        found.best = weight;
                    }
                }
                TrackKind::Event => {}
            }
            found.weight = total;
        }
    }

    /// Converts the accumulators into the player's output samples.
    fn emit(&self, samples: &mut Vec<Sample>) {
        for accumulator in self.accumulators.iter() {
            if accumulator.weight <= 0.0 {
                continue;
            }
            samples.push(Sample {
                target: accumulator.target,
                kind: accumulator.kind,
                vector: accumulator.vector / accumulator.weight,
                rotation: accumulator.rotation.normalize(),
                flag: accumulator.flag,
            });
        }
    }

    /// Records the event keys of the playing clip passed over by `range`.
    fn collect_triggers(&mut self, triggers: &mut Vec<Trigger>, clip_name: &'static str, asset: &Clip, range: &Range) {
        for track in asset.tracks.iter() {
            if !track.enabled || track.kind != TrackKind::Event {
                continue;
            }
            keys_in_range(track, range, &mut self.key_scratch);
            for &index in self.key_scratch.iter() {
                triggers.push(Trigger { clip: clip_name, target: track.target, event: track.keys[index].event, time: track.keys[index].time });
            }
        }
    }

    /// Advances one player, then rebuilds its samples, triggers and status changes.
    fn advance_player(&mut self, clips: &[Clip], state: &mut PlayerState, delta: f32) {
        let PlayerState { player, samples, triggers, status } = state;
        samples.clear();
        triggers.clear();
        status.clear();
        player.finished = false;
        if !player.enabled {
            return;
        }
        let Some(asset) = find_clip(clips, player.current.clip) else {
            return;
        };
        if player.started {
            player.started = false;
            status.push(StatusChange { clip: player.current.name, status: Status::Started });
        }
        let section = resolve_section(player, asset);
        player.section = section;
        player.current.length = asset.duration_seconds;
        let seeked = player.seeked;
        player.seeked = false;

        let step = if player.playing && !seeked { delta * player.speed_scale * player.current.speed } else { 0.0 };
        let from = clamp_float(player.current.position, section.from, section.to);
        let mut to = from + step;
        let mut wrapped = false;
        if step != 0.0 {
            let span = section.to - section.from;
            match asset.loop_mode {
                LoopMode::None => {
                    if to >= section.to {
                        to = section.to;
                        player.finished = true;
                    } else if to <= section.from {
                        to = section.from;
                        player.finished = true;
                    }
                }
                LoopMode::Linear => {
                    if to > section.to || to < section.from {
                        to = section.from + wrapped_position(to - section.from, span);
                        wrapped = true;
                    }
                }
                LoopMode::PingPong => {
                    if to > section.to {
                        to = section.to;
                        player.current.speed = -player.current.speed;
                    } else if to < section.from {
                        to = section.from;
                        player.current.speed = -player.current.speed;
                    }
                }
            }
        }
        player.current.position = clamp_float(to, section.from, section.to);

        let mut blended = 0.0;
        for blend in player.blends.iter_mut() {
            blend.left = max_float(blend.left - absolute(delta), 0.0);
            blend.playback.position =
                clamp_float(blend.playback.position + (delta * player.speed_scale * blend.playback.speed), 0.0, blend.playback.length);
            blended += if blend.seconds > 0.0 { blend.left / blend.seconds } else { 0.0 };
        }
        player.blends.retain(|blend| blend.left > 0.0);
        let mut captured = 0.0;
        if player.capture.left > 0.0 && player.capture.seconds > 0.0 {
            player.capture.left = max_float(player.capture.left - absolute(delta), 0.0);
            captured = ease_curve(player.capture.left / player.capture.seconds, player.capture.easing);
            if player.capture.left <= 0.0 {
                player.capture.clear();
            }
        }
        blended = clamp_float(blended + captured, 0.0, 1.0);

        self.accumulators.clear();
        self.sample_playback(clips, &player.current, player.root_motion_target, 1.0 - blended);
        for blend in player.blends.iter() {
            let weight = if blend.seconds > 0.0 { blend.left / blend.seconds } else { 0.0 };
            self.sample_playback(clips, &blend.playback, player.root_motion_target, weight);
        }
        self.sample_capture(&player.capture, captured);
        self.emit(samples);

        let range = Range { from, to: player.current.position, low: section.from, high: section.to, wrapped };
        let motion_range = if step != 0.0 { range } else { Range { from, to: from, low: section.from, high: section.to, wrapped: false } };
        collect_root_motion(player, asset, &motion_range);

        if step != 0.0 && blended < 1.0 {
            self.collect_triggers(triggers, player.current.name, asset, &range);
        }

        player.capture_pending = false;
        if !player.finished {
            return;
        }
        status.push(StatusChange { clip: player.current.name, status: Status::Finished });
        let following = player.next(player.current.name);
        if !following.is_empty() && player.has_clip(following) {
            player.play(following, Play::default());
        } else if !player.queue.is_empty() {
            let wanted = player.queue.remove(0);
            player.play(wanted, Play::default());
        } else {
            player.playing = false;
            return;
        }
        if player.started {
            player.started = false;
            status.push(StatusChange { clip: player.current.name, status: Status::Started });
        }
    }
}

/// Adds the root target's motion over a range to the player's running total.
fn collect_root_motion(player: &mut Player, asset: &Clip, range: &Range) {
    player.root_motion = RootMotion::default();
    if player.root_motion_target.is_empty() {
        return;
    }
    for track in asset.tracks.iter() {
        if !track.enabled || track.target != player.root_motion_target {
            continue;
        }
        if track.kind == TrackKind::Position {
            let first = sample_vector(track, range.from, Vec3::default());
            let last = sample_vector(track, range.to, Vec3::default());
            let delta = if range.wrapped {
                (sample_vector(track, range.high, Vec3::default()) - first) + (last - sample_vector(track, range.low, Vec3::default()))
            } else {
                last - first
            };
            player.root_motion.position = player.root_motion.position + delta;
        } else if track.kind == TrackKind::Rotation {
            let first = sample_rotation(track, range.from, Quat::default());
            let last = sample_rotation(track, range.to, Quat::default());
            let mut delta = conjugate(first) * last;
            if range.wrapped {
                let high = sample_rotation(track, range.high, Quat::default());
                let low = sample_rotation(track, range.low, Quat::default());
                delta = (conjugate(first) * high) * (conjugate(low) * last);
            }
            player.root_motion.rotation = (player.root_motion.rotation * delta).normalize();
        }
    }
    player.root_motion_total.position = player.root_motion_total.position + player.root_motion.position;
    player.root_motion_total.rotation = (player.root_motion_total.rotation * player.root_motion.rotation).normalize();
}

impl World {
    /// Adds a clip and returns its handle.
    pub fn add_clip(&mut self, clip: Clip) -> ClipHandle {
        self.clips.push(clip);
        (self.clips.len() - 1) as ClipHandle
    }

    /// Adds `count` players.
    pub fn add_players(&mut self, count: usize) {
        self.players.resize_with(self.players.len() + count, PlayerState::default);
    }

    /// Advances every player by `delta_seconds`, rebuilding its samples, triggers and status.
    pub fn advance(&mut self, delta_seconds: f32) {
        let World { clips, players, sampler } = self;
        for state in players.iter_mut() {
            sampler.advance_player(clips, state, delta_seconds);
        }
    }

    /// Stores the player's latest samples as the pose its pending capture blends out from.
    pub fn apply_capture(state: &mut PlayerState) {
        let player = &mut state.player;
        if !player.capture_pending {
            return;
        }
        player.capture.samples.clear();
        player.capture.samples.extend_from_slice(&state.samples);
        player.capture.seconds = player.capture_duration;
        player.capture.left = player.capture_duration;
        player.capture.easing = player.capture_easing_wanted;
        player.capture_pending = false;
        player.blends.clear();
    }
}
