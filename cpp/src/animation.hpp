#pragma once

#include <cstddef>
#include <cstdint>
#include <string_view>
#include <vector>

#include "animation_clip.hpp"

/// The engine's animation players and the world that advances them, from
/// animation.hpp and animation.cpp. Clips are held by index instead of the
/// engine's slot map, and names are views of strings that outlive the world.
namespace bench::anim {

/// Index of a clip in the world; `invalid_clip` refers to none.
using ClipHandle = uint32_t;
inline constexpr ClipHandle invalid_clip = UINT32_MAX;

/// Blend duration that asks for the player's configured one.
inline constexpr float blend_from_default = -1.0f;

/// A clip registered on a player under a name.
struct ClipEntry {
    std::string_view name{};
    ClipHandle clip{invalid_clip};
};

/// The value of one animated property, as emitted by an advance.
struct Sample {
    std::string_view target{};
    TrackKind kind{TrackKind::Position};
    vm::Vec3 vector{};
    vm::Quat rotation{};
    bool flag{true};
};

/// Options of a `play` call.
struct Play {
    float blend_seconds{blend_from_default};
    float speed{1.0f};
    bool from_end{};
};

/// The span of a clip that plays.
struct Section {
    float from{};
    float to{};
    bool active{};
};

/// A wanted section in seconds; negative ends mean the clip's own.
struct SectionRange {
    float from{-1.0f};
    float to{-1.0f};
};

/// A wanted section between two markers.
struct MarkerRange {
    std::string_view from{};
    std::string_view to{};
};

/// Playback events reported by an advance.
enum class Status : uint8_t { Started, Finished };

/// A clip starting or finishing.
struct StatusChange {
    std::string_view clip{};
    Status status{Status::Started};
};

/// Motion of the root target over an advance.
struct RootMotion {
    vm::Vec3 position{};
    vm::Quat rotation{};
};

/// An event key passed over by an advance.
struct Trigger {
    std::string_view clip{};
    std::string_view target{};
    std::string_view event{};
    float time{};
};

class World;

/// Plays clips with blends, queueing, sections, pose capture and root motion.
class Player final {
public:
    /// Returns the player to its freshly constructed state, keeping storage.
    void reset();
    /// Registers `clip` under `name`, replacing a clip of that name.
    void add_clip(std::string_view name, ClipHandle clip);
    /// Whether a clip is registered under `name`.
    bool has_clip(std::string_view name) const;
    /// Sets the speed multiplier of every playback.
    void set_speed_scale(float scale);
    /// Sets the blend duration used when a pair has none.
    void set_default_blend_seconds(float seconds);
    /// Sets the blend duration from clip `from` to clip `to`.
    void set_blend_seconds(std::string_view from, std::string_view to, float seconds);
    /// The blend duration from clip `from` to clip `to`.
    float blend_seconds(std::string_view from, std::string_view to) const;
    /// Sets the clip that follows `from` when it finishes.
    void set_next(std::string_view from, std::string_view to);
    /// The clip that follows `from`, or empty.
    std::string_view next(std::string_view from) const;
    /// Starts clip `name`, or the assigned clip when `name` is empty.
    void play(std::string_view name = {}, Play options = {});
    /// Starts clip `name` backwards from its end.
    void play_backwards(std::string_view name, float blend_seconds);
    /// Starts clip `name` over the section `range`.
    void play_section(std::string_view name, SectionRange range, Play options = {});
    /// Starts clip `name` over the section between two markers.
    void play_section_with_markers(std::string_view name, MarkerRange markers, Play options = {});
    /// Starts clip `name` and blends out from a captured pose over `duration_seconds`.
    void play_with_capture(std::string_view name, float duration_seconds, Play options, Easing easing);
    /// Wants the section `range`.
    void set_section(SectionRange range);
    /// Wants the section between two markers.
    void set_section_with_markers(MarkerRange markers);
    /// Wants the whole clip again.
    void reset_section();
    /// Sets whether every `play` captures the current pose.
    void set_auto_capture(bool capture);
    /// Sets the duration of automatic captures.
    void set_auto_capture_duration(float seconds);
    /// Sets the easing of automatic captures.
    void set_auto_capture_easing(Easing easing);
    /// Whether a pose capture is waiting for `World::apply_capture`.
    bool capture_pending() const;
    /// Sets the target whose tracks drive root motion, and clears the motion.
    void set_root_motion_target(std::string_view target);
    /// The root motion summed over every advance.
    const RootMotion& root_motion_accumulator() const;
    /// Clears the root motion of the last advance and the total.
    void reset_root_motion();
    /// Queues clip `name` to play after the current one finishes.
    void queue(std::string_view name);
    /// Moves the playback to `seconds`.
    void seek(float seconds);
    /// Playback position in seconds.
    float position() const;
    /// Name of the playing clip.
    std::string_view current_clip() const;
    /// Whether the last advance finished the clip.
    bool finished() const;

private:
    friend class World;

    struct Playback {
        std::string_view name{};
        ClipHandle clip{invalid_clip};
        float position{};
        float length{};
        float speed{1.0f};
    };

    struct Blend {
        Playback playback{};
        float seconds{};
        float left{};
    };

    struct Capture {
        std::vector<Sample> samples{};
        float seconds{};
        float left{};
        Easing easing{};
    };

    struct BlendEntry {
        std::string_view from{};
        std::string_view to{};
        float seconds{};
    };

    struct NextEntry {
        std::string_view from{};
        std::string_view to{};
    };

    /// The clip registered under `name`, or null.
    const ClipEntry* entry(std::string_view name) const;

    std::vector<ClipEntry> clips_{};
    std::vector<BlendEntry> blend_times_{};
    std::vector<NextEntry> next_{};
    std::vector<std::string_view> queue_{};
    std::vector<Blend> blends_{};
    Capture capture_{};
    Playback current_{};
    Section section_{};
    RootMotion root_motion_{};
    RootMotion root_motion_total_{};
    Easing auto_capture_easing_{};
    Easing capture_easing_wanted_{};
    std::string_view assigned_{};
    std::string_view section_from_marker_{};
    std::string_view section_to_marker_{};
    std::string_view root_motion_target_{};
    float section_from_{-1.0f};
    float section_to_{-1.0f};
    float speed_scale_{1.0f};
    float default_blend_seconds_{};
    float capture_duration_{};
    float auto_capture_duration_{};
    bool section_wanted_{};
    bool section_by_marker_{};
    bool auto_capture_{};
    bool capture_pending_{};
    bool enabled_{true};
    bool playing_{};
    bool seeked_{};
    bool started_{};
    bool finished_{};
};

/// A player with the samples, event triggers and status changes of its latest advance.
struct PlayerState {
    Player player{};
    std::vector<Sample> samples{};
    std::vector<Trigger> triggers{};
    std::vector<StatusChange> status{};
};

/// Owns the clips and players and advances every player by a time step.
class World final {
public:
    /// Adds a clip and returns its handle.
    ClipHandle add_clip(Clip clip);
    /// Adds `count` players.
    void add_players(size_t count);
    /// The players.
    std::vector<PlayerState>& players();
    /// Advances every player by `delta_seconds`, rebuilding its samples, triggers and status.
    void advance(float delta_seconds);
    /// Stores the player's latest samples as the pose its pending capture blends out from.
    void apply_capture(PlayerState& state);

private:
    /// Weighted blend of every sample that targets one property.
    struct Accumulator {
        std::string_view target{};
        TrackKind kind{TrackKind::Position};
        vm::Vec3 vector{};
        vm::Quat rotation{};
        bool flag{true};
        float weight{};
        float best{};
    };

    /// Time to sample a track at and the weight of that sample in the blend.
    struct Contribution {
        float time{};
        float weight{};
    };

    /// The clip behind `handle`, or null.
    const Clip* find_clip(ClipHandle handle) const;
    /// Advances one player, then rebuilds its samples, triggers and status changes.
    void advance_player(PlayerState& state, float delta);
    /// Resolves the section a player wants into clamped times for a clip.
    Section resolve_section(const Player& player, const Clip& asset) const;
    /// Finds or creates the accumulator of a property.
    Accumulator& accumulator(std::string_view target, TrackKind kind);
    /// Adds a weighted position or scale sample of a track.
    void accumulate_vector(const Track& track, Contribution at);
    /// Blends a rotation sample of a track into its accumulator.
    void accumulate_rotation(const Track& track, Contribution at);
    /// Takes a flag sample of a track when it outweighs those accumulated so far.
    void accumulate_flag(const Track& track, Contribution at);
    /// Accumulates every enabled track of a playback at its position.
    void sample_playback(const Player::Playback& playback, std::string_view skip_target, float weight);
    /// Accumulates a captured pose.
    void sample_capture(const Player::Capture& capture, float weight);
    /// Adds the root target's motion over a range to the player's running total.
    void collect_root_motion(Player& player, const Clip& asset, const Range& range);
    /// Converts the accumulators into the player's output samples.
    void emit(PlayerState& state);
    /// Records the event keys of the playing clip passed over by `range`.
    void collect_triggers(PlayerState& state, const Clip& asset, const Range& range);

    std::vector<Clip> clips_{};
    std::vector<PlayerState> players_{};
    std::vector<Accumulator> accumulators_{};
    std::vector<size_t> key_scratch_{};
};

} // namespace bench::anim
