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
use crate::animation::{ClipHandle, MarkerRange, Play, Player, PlayerState, SectionRange, World};
use crate::animation_clip::{Clip, Ease, Easing, Interpolation, Key, LoopMode, Marker, Track, TrackKind, Transition};
use crate::harness::Case;
use crate::hash::{self, Rng};
use crate::vecmath::{Quat, Vec3};

const PLAYERS: usize = 1024;
const FRAMES: u32 = 48;
const SETS: usize = 8;
const CLIPS_PER_PLAYER: usize = 3;
const KEYS: usize = 48;
const FRAME_SECONDS: f32 = 1.0 / 30.0;
const DIGEST_PRIME: u64 = 0x9e37_79b9_7f4a_7c15;

const CLIP_NAMES: [&str; CLIPS_PER_PLAYER] = ["idle", "walk", "jump"];
const EVENT_NAMES: [&str; 4] = ["step", "splash", "dust", "whoosh"];
const ROOT_PATH: &str = "armature/root";
const MARKER_NAMES: [&str; 3] = ["foot_l", "foot_r", "land"];
const MARKER_FRACTIONS: [f32; 3] = [0.2, 0.6, 0.9];

/// What one track of every clip animates.
struct TrackSpec {
    kind: TrackKind,
    interpolation: Interpolation,
    target: &'static str,
    travels: bool,
}

const TRACK_SPECS: [TrackSpec; 8] = [
    TrackSpec { kind: TrackKind::Position, interpolation: Interpolation::Linear, target: ROOT_PATH, travels: true },
    TrackSpec { kind: TrackKind::Rotation, interpolation: Interpolation::Linear, target: ROOT_PATH, travels: false },
    TrackSpec { kind: TrackKind::Position, interpolation: Interpolation::Cubic, target: "armature/root/hips", travels: false },
    TrackSpec { kind: TrackKind::Rotation, interpolation: Interpolation::Cubic, target: "armature/root/hips", travels: false },
    TrackSpec {
        kind: TrackKind::Rotation,
        interpolation: Interpolation::Linear,
        target: "armature/root/hips/spine/chest",
        travels: false,
    },
    TrackSpec {
        kind: TrackKind::Scale,
        interpolation: Interpolation::Linear,
        target: "armature/root/hips/spine/chest/head",
        travels: false,
    },
    TrackSpec {
        kind: TrackKind::Active,
        interpolation: Interpolation::Nearest,
        target: "armature/root/hips/spine/chest/arm_l/hand_l",
        travels: false,
    },
    TrackSpec { kind: TrackKind::Event, interpolation: Interpolation::Nearest, target: "armature/fx", travels: false },
];

pub struct Anim {
    world: World,
    digest: Vec<u64>,
}

/// Folds `value` into the running digest `state`.
fn fold(state: u64, value: u64) -> u64 {
    let mixed = (state ^ value).wrapping_mul(DIGEST_PRIME);
    mixed ^ (mixed >> 32)
}

/// Two floats' bit patterns in one word.
fn pack(low: f32, high: f32) -> u64 {
    u64::from(hash::f32_bits(low)) | (u64::from(hash::f32_bits(high)) << 32)
}

/// Folds the samples, triggers and status changes of one advance into `state`.
fn fold_player(mut state: u64, player: &PlayerState) -> u64 {
    for sample in player.samples.iter() {
        let tag = u64::from(sample.flag) | ((sample.kind as u64) << 1) | ((sample.target.len() as u64) << 8);
        state = fold(state, pack(sample.vector.x, sample.vector.y));
        state = fold(state, pack(sample.vector.z, sample.rotation.x));
        state = fold(state, pack(sample.rotation.y, sample.rotation.z));
        state = fold(state, pack(sample.rotation.w, 0.0) | (tag << 32));
    }
    for trigger in player.triggers.iter() {
        let tag = u64::from(trigger.event.as_bytes()[0]) | ((trigger.target.len() as u64) << 8) | ((trigger.clip.len() as u64) << 16);
        state = fold(state, pack(trigger.time, 0.0) | (tag << 32));
    }
    for change in player.status.iter() {
        state = fold(state, ((change.clip.len() as u64) << 1) | change.status as u64);
    }
    fold(
        state,
        player.samples.len() as u64 | ((player.triggers.len() as u64) << 16) | (u64::from(player.player.finished()) << 32),
    )
}

/// The key time of key `index`, jittered by `unit` except at both ends.
fn key_time(index: usize, spacing: f32, duration: f32, unit: f32) -> f32 {
    if index == 0 {
        return 0.0;
    }
    if index == KEYS - 1 {
        return duration;
    }
    (index as f32 + (unit - 0.5) * 0.5) * spacing
}

/// A random easing curve.
fn random_easing(rng: &mut Rng) -> Easing {
    let transition = rng.next() % 7;
    let ease = rng.next() % 4;
    Easing {
        transition: [
            Transition::Linear,
            Transition::Sine,
            Transition::Quad,
            Transition::Cubic,
            Transition::Expo,
            Transition::Circ,
            Transition::Back,
        ][transition as usize],
        ease: [Ease::In, Ease::Out, Ease::InOut, Ease::OutIn][ease as usize],
    }
}

/// Key `index` of a track made to `spec`.
fn random_key(rng: &mut Rng, spec: &TrackSpec, index: usize, spacing: f32, duration: f32) -> Key {
    let mut key = Key::default();
    let unit = rng.unit();
    key.time = key_time(index, spacing, duration, unit);
    let position = index as f32;
    match spec.kind {
        TrackKind::Position => {
            if spec.travels {
                let jitter = Vec3::random(rng, -0.05, 0.05);
                key.vector = jitter + Vec3::new(position * 0.1, 0.0, position * 0.2);
            } else {
                key.vector = Vec3::random(rng, -1.0, 1.0);
            }
        }
        TrackKind::Scale => key.vector = Vec3::random(rng, 0.8, 1.2),
        TrackKind::Rotation => key.rotation = Quat::random(rng),
        TrackKind::Active => key.flag = rng.unit() < 0.5,
        TrackKind::Event => key.event = EVENT_NAMES[(rng.next() % EVENT_NAMES.len() as u64) as usize],
    }
    key.easing = random_easing(rng);
    key
}

/// A clip of every track spec with `KEYS` keys each over `duration` seconds.
fn random_clip(rng: &mut Rng, loop_mode: LoopMode, duration: f32) -> Clip {
    let spacing = duration / (KEYS - 1) as f32;
    let mut tracks = Vec::with_capacity(TRACK_SPECS.len());
    for spec in TRACK_SPECS.iter() {
        let mut keys = Vec::with_capacity(KEYS);
        for index in 0..KEYS {
            keys.push(random_key(rng, spec, index, spacing, duration));
        }
        tracks.push(Track { kind: spec.kind, interpolation: spec.interpolation, target: spec.target, enabled: true, keys });
    }
    let markers = MARKER_NAMES
        .iter()
        .zip(MARKER_FRACTIONS.iter())
        .map(|(&marker, &fraction)| Marker { name: marker, time: duration * fraction })
        .collect();
    Clip { duration_seconds: duration, loop_mode, tracks, markers }
}

/// A value in [0, 1) from 16 bits of `bits`.
fn unit_from_bits(bits: u64) -> f32 {
    (bits & 0xFFFF) as f32 * (1.0 / 65536.0)
}

/// Returns `player` to its initial clips, blends and playback as player `index`.
fn configure_player(state: &mut PlayerState, index: usize) {
    state.samples.clear();
    state.triggers.clear();
    state.status.clear();
    let player: &mut Player = &mut state.player;
    player.reset();
    let set = index % SETS;
    for (c, name) in CLIP_NAMES.iter().enumerate() {
        player.add_clip(name, (set * CLIPS_PER_PLAYER + c) as ClipHandle);
    }
    player.set_default_blend_seconds(0.25);
    player.set_blend_seconds("idle", "walk", 0.3);
    player.set_blend_seconds("walk", "jump", 0.1);
    player.set_blend_seconds("jump", "walk", 0.15);
    player.set_blend_seconds("walk", "idle", 0.4);
    player.set_next("jump", "walk");
    player.set_next("walk", "idle");
    player.set_root_motion_target(ROOT_PATH);
    player.set_speed_scale(0.75 + (index % 5) as f32 * 0.125);
    if index % 4 == 0 {
        player.set_auto_capture(true);
        player.set_auto_capture_duration(0.2);
        player.set_auto_capture_easing(Easing { transition: Transition::Quad, ease: Ease::Out });
    }
    player.play(CLIP_NAMES[index % CLIPS_PER_PLAYER], Play::default());
}

/// Applies the scripted clip change, if any, of player `index` at `frame`.
fn script_player(state: &mut PlayerState, index: usize, frame: u32) {
    let roll = hash::mix64(((index as u64) << 32) | u64::from(frame));
    if roll & 31 != 0 {
        return;
    }
    let player = &mut state.player;
    let pick = CLIP_NAMES[((roll >> 16) % CLIPS_PER_PLAYER as u64) as usize];
    let unit = unit_from_bits(roll >> 24);
    match (roll >> 8) & 7 {
        0 => {
            player.reset_section();
            player.play(pick, Play::default());
        }
        1 => player.play_with_capture(pick, 0.2 + 0.2 * unit, Play::default(), Easing { transition: Transition::Cubic, ease: Ease::InOut }),
        2 => player.queue(pick),
        3 => player.play_backwards(pick, 0.15),
        4 => player.play_section_with_markers(pick, MarkerRange { from: "foot_l", to: "land" }, Play::default()),
        5 => player.seek(unit * 1.5),
        6 => player.set_speed_scale(0.5 + unit),
        _ => player.play_section(pick, SectionRange { from: 0.2 + 0.2 * unit, to: 0.9 }, Play::default()),
    }
    if player.capture_pending() {
        World::apply_capture(state);
    }
}

impl Case for Anim {
    const NAME: &'static str = "anim";

    /// Draws the clips and creates the players.
    fn init() -> Anim {
        let mut rng = Rng::new(0xa41);
        let mut world = World::default();
        for set in 0..SETS {
            let variant = set as f32;
            world.add_clip(random_clip(&mut rng, LoopMode::Linear, 1.6 + 0.1 * variant));
            world.add_clip(random_clip(&mut rng, LoopMode::PingPong, 1.2 + 0.05 * variant));
            world.add_clip(random_clip(&mut rng, LoopMode::None, 0.9 + 0.02 * variant));
        }
        world.add_players(PLAYERS);
        Anim { world, digest: vec![0; PLAYERS] }
    }

    /// Resets every player, steps every frame, and hashes the digests, root motion and positions.
    fn run(&mut self) -> u64 {
        for (index, state) in self.world.players.iter_mut().enumerate() {
            configure_player(state, index);
            self.digest[index] = 0;
        }
        for frame in 0..FRAMES {
            for (index, state) in self.world.players.iter_mut().enumerate() {
                script_player(state, index, frame);
            }
            self.world.advance(FRAME_SECONDS);
            for (digest, state) in self.digest.iter_mut().zip(self.world.players.iter()) {
                *digest = fold_player(*digest, state);
            }
        }
        let mut h = 0u64;
        for (digest, state) in self.digest.iter().zip(self.world.players.iter()) {
            let player = &state.player;
            let root = player.root_motion_accumulator();
            h = hash::add(h, *digest);
            h = hash::add(h, pack(root.position.x, root.position.y));
            h = hash::add(h, pack(root.position.z, root.rotation.x));
            h = hash::add(h, pack(root.rotation.y, root.rotation.z));
            h = hash::add(h, pack(root.rotation.w, player.position()));
            h = hash::add(h, player.current_clip().len() as u64);
        }
        h
    }
}
