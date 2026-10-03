#include "anim.hpp"

#include <array>
#include <string_view>

#include "hash.hpp"

namespace bench {

namespace {

using anim::Clip;
using anim::Easing;
using anim::Ease;
using anim::Interpolation;
using anim::Key;
using anim::LoopMode;
using anim::Track;
using anim::TrackKind;
using anim::Transition;

constexpr size_t players_n = 1024;
constexpr uint32_t frames_n = 48;
constexpr size_t sets_n = 8;
constexpr size_t clips_per_player = 3;
constexpr size_t keys_n = 48;
constexpr float frame_seconds = 1.0f / 30.0f;
constexpr uint64_t digest_prime = 0x9e3779b97f4a7c15ull;

constexpr std::array<std::string_view, clips_per_player> clip_names{"idle", "walk", "jump"};
constexpr std::array<std::string_view, 4> event_names{"step", "splash", "dust", "whoosh"};
constexpr std::string_view root_path = "armature/root";
constexpr std::string_view marker_names[] = {"foot_l", "foot_r", "land"};
constexpr float marker_fractions[] = {0.2f, 0.6f, 0.9f};

/// What one track of every clip animates.
struct TrackSpec {
    TrackKind kind;
    Interpolation interpolation;
    std::string_view target;
    bool travels;
};

constexpr std::array<TrackSpec, 8> track_specs{{
    {TrackKind::Position, Interpolation::Linear, root_path, true},
    {TrackKind::Rotation, Interpolation::Linear, root_path, false},
    {TrackKind::Position, Interpolation::Cubic, "armature/root/hips", false},
    {TrackKind::Rotation, Interpolation::Cubic, "armature/root/hips", false},
    {TrackKind::Rotation, Interpolation::Linear, "armature/root/hips/spine/chest", false},
    {TrackKind::Scale, Interpolation::Linear, "armature/root/hips/spine/chest/head", false},
    {TrackKind::Active, Interpolation::Nearest, "armature/root/hips/spine/chest/arm_l/hand_l", false},
    {TrackKind::Event, Interpolation::Nearest, "armature/fx", false},
}};

/// Folds `value` into the running digest `state`.
uint64_t fold(uint64_t state, uint64_t value) {
    state = (state ^ value) * digest_prime;
    return state ^ (state >> 32);
}

/// Two floats' bit patterns in one word.
uint64_t pack(float low, float high) {
    return static_cast<uint64_t>(f32_bits(low)) | (static_cast<uint64_t>(f32_bits(high)) << 32);
}

/// Folds the samples, triggers and status changes of one advance into `state`.
uint64_t fold_player(uint64_t state, const anim::PlayerState& player) {
    for (const anim::Sample& sample : player.samples) {
        const uint64_t tag = static_cast<uint64_t>(sample.flag) | (static_cast<uint64_t>(sample.kind) << 1) |
                             (static_cast<uint64_t>(sample.target.size()) << 8);
        state = fold(state, pack(sample.vector.x, sample.vector.y));
        state = fold(state, pack(sample.vector.z, sample.rotation.x));
        state = fold(state, pack(sample.rotation.y, sample.rotation.z));
        state = fold(state, pack(sample.rotation.w, 0.0f) | (tag << 32));
    }
    for (const anim::Trigger& trigger : player.triggers) {
        const uint64_t tag = static_cast<uint64_t>(static_cast<uint8_t>(trigger.event.front())) |
                             (static_cast<uint64_t>(trigger.target.size()) << 8) | (static_cast<uint64_t>(trigger.clip.size()) << 16);
        state = fold(state, pack(trigger.time, 0.0f) | (tag << 32));
    }
    for (const anim::StatusChange& change : player.status) {
        state = fold(state, (static_cast<uint64_t>(change.clip.size()) << 1) | static_cast<uint64_t>(change.status));
    }
    return fold(state, static_cast<uint64_t>(player.samples.size()) | (static_cast<uint64_t>(player.triggers.size()) << 16) |
                           (static_cast<uint64_t>(player.player.finished()) << 32));
}

/// The key time of key `index` of `count`, jittered by `unit` except at both ends.
float key_time(size_t index, float spacing, float duration, float unit) {
    if (index == 0) return 0.0f;
    if (index == keys_n - 1) return duration;
    return (static_cast<float>(index) + (unit - 0.5f) * 0.5f) * spacing;
}

/// A random easing curve.
Easing random_easing(Rng& rng) {
    Easing easing;
    const uint64_t transition = rng.next() % 7;
    const uint64_t ease = rng.next() % 4;
    easing.transition = static_cast<Transition>(transition);
    easing.ease = static_cast<Ease>(ease);
    return easing;
}

/// Key `index` of a track made to `spec`.
Key random_key(Rng& rng, const TrackSpec& spec, size_t index, float spacing, float duration) {
    Key key;
    const float unit = rng.unit();
    key.time = key_time(index, spacing, duration, unit);
    const float position = static_cast<float>(index);
    switch (spec.kind) {
    case TrackKind::Position: {
        const vm::Vec3 jitter = spec.travels ? vm::random_vec3(rng, -0.05f, 0.05f) : vm::random_vec3(rng, -1.0f, 1.0f);
        key.vector = spec.travels ? jitter + vm::Vec3{position * 0.1f, 0.0f, position * 0.2f} : jitter;
        break;
    }
    case TrackKind::Scale:
        key.vector = vm::random_vec3(rng, 0.8f, 1.2f);
        break;
    case TrackKind::Rotation:
        key.rotation = vm::random_quat(rng);
        break;
    case TrackKind::Active:
        key.flag = rng.unit() < 0.5f;
        break;
    case TrackKind::Event:
        key.event = event_names[rng.next() % event_names.size()];
        break;
    }
    key.easing = random_easing(rng);
    return key;
}

/// A clip of every track spec with `keys_n` keys each over `duration` seconds.
Clip random_clip(Rng& rng, LoopMode loop, float duration) {
    Clip clip;
    clip.duration_seconds = duration;
    clip.loop = loop;
    const float spacing = duration / static_cast<float>(keys_n - 1);
    for (const TrackSpec& spec : track_specs) {
        Track track;
        track.kind = spec.kind;
        track.interpolation = spec.interpolation;
        track.target = spec.target;
        for (size_t index = 0; index < keys_n; ++index) track.keys.push_back(random_key(rng, spec, index, spacing, duration));
        clip.tracks.push_back(std::move(track));
    }
    for (size_t m = 0; m < std::size(marker_names); ++m) clip.markers.push_back(anim::Marker{marker_names[m], duration * marker_fractions[m]});
    return clip;
}

/// A value in [0, 1) from 16 bits of `bits`.
float unit_from_bits(uint64_t bits) {
    return static_cast<float>(bits & 0xFFFF) * (1.0f / 65536.0f);
}

} // namespace

Anim::Anim() : digest_(players_n) {
    Rng rng{0xa41};
    for (size_t set = 0; set < sets_n; ++set) {
        const float variant = static_cast<float>(set);
        world_.add_clip(random_clip(rng, LoopMode::Linear, 1.6f + 0.1f * variant));
        world_.add_clip(random_clip(rng, LoopMode::PingPong, 1.2f + 0.05f * variant));
        world_.add_clip(random_clip(rng, LoopMode::None, 0.9f + 0.02f * variant));
    }
    world_.add_players(players_n);
}

void Anim::configure_player(size_t index) {
    anim::PlayerState& state = world_.players()[index];
    state.samples.clear();
    state.triggers.clear();
    state.status.clear();
    anim::Player& player = state.player;
    player.reset();
    const size_t set = index % sets_n;
    for (size_t c = 0; c < clips_per_player; ++c) player.add_clip(clip_names[c], static_cast<anim::ClipHandle>(set * clips_per_player + c));
    player.set_default_blend_seconds(0.25f);
    player.set_blend_seconds("idle", "walk", 0.3f);
    player.set_blend_seconds("walk", "jump", 0.1f);
    player.set_blend_seconds("jump", "walk", 0.15f);
    player.set_blend_seconds("walk", "idle", 0.4f);
    player.set_next("jump", "walk");
    player.set_next("walk", "idle");
    player.set_root_motion_target(root_path);
    player.set_speed_scale(0.75f + static_cast<float>(index % 5) * 0.125f);
    if (index % 4 == 0) {
        player.set_auto_capture(true);
        player.set_auto_capture_duration(0.2f);
        player.set_auto_capture_easing(Easing{Transition::Quad, Ease::Out});
    }
    player.play(clip_names[index % clips_per_player]);
}

void Anim::script_player(size_t index, uint32_t frame) {
    const uint64_t roll = mix64((static_cast<uint64_t>(index) << 32) | frame);
    if ((roll & 31) != 0) return;
    anim::PlayerState& state = world_.players()[index];
    anim::Player& player = state.player;
    const std::string_view pick = clip_names[(roll >> 16) % clips_per_player];
    const float unit = unit_from_bits(roll >> 24);
    switch ((roll >> 8) & 7) {
    case 0:
        player.reset_section();
        player.play(pick);
        break;
    case 1:
        player.play_with_capture(pick, 0.2f + 0.2f * unit, anim::Play{}, Easing{Transition::Cubic, Ease::InOut});
        break;
    case 2:
        player.queue(pick);
        break;
    case 3:
        player.play_backwards(pick, 0.15f);
        break;
    case 4:
        player.play_section_with_markers(pick, anim::MarkerRange{"foot_l", "land"});
        break;
    case 5:
        player.seek(unit * 1.5f);
        break;
    case 6:
        player.set_speed_scale(0.5f + unit);
        break;
    default:
        player.play_section(pick, anim::SectionRange{0.2f + 0.2f * unit, 0.9f});
        break;
    }
    if (player.capture_pending()) world_.apply_capture(state);
}

uint64_t Anim::run() {
    for (size_t p = 0; p < players_n; ++p) {
        configure_player(p);
        digest_[p] = 0;
    }
    for (uint32_t frame = 0; frame < frames_n; ++frame) {
        for (size_t p = 0; p < players_n; ++p) script_player(p, frame);
        world_.advance(frame_seconds);
        for (size_t p = 0; p < players_n; ++p) digest_[p] = fold_player(digest_[p], world_.players()[p]);
    }
    uint64_t h = 0;
    for (size_t p = 0; p < players_n; ++p) {
        const anim::Player& player = world_.players()[p].player;
        const anim::RootMotion& root = player.root_motion_accumulator();
        h = hash_add(h, digest_[p]);
        h = hash_add(h, pack(root.position.x, root.position.y));
        h = hash_add(h, pack(root.position.z, root.rotation.x));
        h = hash_add(h, pack(root.rotation.y, root.rotation.z));
        h = hash_add(h, pack(root.rotation.w, player.position()));
        h = hash_add(h, player.current_clip().size());
    }
    return h;
}

} // namespace bench
