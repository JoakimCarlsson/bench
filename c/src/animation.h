#ifndef BENCH_ANIMATION_H
#define BENCH_ANIMATION_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "animation_clip.h"
#include "array.h"

/// The engine's animation players and the world that advances them, from
/// animation.hpp and animation.cpp. Clips are held by index instead of the
/// engine's slot map, and names are views of strings that outlive the world.

/// Index of a clip in the world; `ANIM_INVALID_CLIP` refers to none.
typedef uint32_t AnimClipHandle;
#define ANIM_INVALID_CLIP UINT32_MAX

/// Blend duration that asks for the player's configured one.
#define ANIM_BLEND_FROM_DEFAULT (-1.0f)

/// A clip registered on a player under a name.
typedef struct {
    Str name;
    AnimClipHandle clip;
} AnimClipEntry;

/// The value of one animated property, as emitted by an advance.
typedef struct {
    Str target;
    AnimTrackKind kind;
    Vec3 vector;
    Quat rotation;
    bool flag;
} AnimSample;

/// Options of a `play` call.
typedef struct {
    float blend_seconds;
    float speed;
    bool from_end;
} AnimPlay;

/// The configured blend, forwards, from the start.
#define ANIM_PLAY_DEFAULT ((AnimPlay){ ANIM_BLEND_FROM_DEFAULT, 1.0f, false })

/// A section of a clip in seconds; as a wanted section, negative ends mean the clip's own.
typedef struct {
    float from;
    float to;
} AnimSection;

/// A wanted section between two markers.
typedef struct {
    Str from;
    Str to;
} AnimMarkerRange;

/// Playback events reported by an advance.
typedef enum { STATUS_STARTED, STATUS_FINISHED } AnimStatus;

/// A clip starting or finishing.
typedef struct {
    Str clip;
    AnimStatus status;
} AnimStatusChange;

/// Motion of the root target over an advance.
typedef struct {
    Vec3 position;
    Quat rotation;
} AnimRootMotion;

/// An event key passed over by an advance.
typedef struct {
    Str clip;
    Str target;
    Str event;
    float time;
} AnimTrigger;

/// What is playing: a clip, how far in, and how fast.
typedef struct {
    Str name;
    AnimClipHandle clip;
    float position;
    float length;
    float speed;
} AnimPlayback;

/// A previous playback fading out.
typedef struct {
    AnimPlayback playback;
    float seconds;
    float left;
} AnimBlend;

typedef ARRAY_OF(AnimSample) SampleArray;
typedef ARRAY_OF(AnimTrigger) TriggerArray;
typedef ARRAY_OF(AnimStatusChange) StatusArray;
typedef ARRAY_OF(AnimClipEntry) ClipEntryArray;
typedef ARRAY_OF(AnimBlend) BlendArray;
typedef ARRAY_OF(Str) StrArray;

/// A captured pose fading out.
typedef struct {
    SampleArray samples;
    float seconds;
    float left;
    AnimEasing easing;
} AnimCapture;

/// A blend duration between two clips.
typedef struct {
    Str from;
    Str to;
    float seconds;
} AnimBlendEntry;

/// The clip that follows another when it finishes.
typedef struct {
    Str from;
    Str to;
} AnimNextEntry;

typedef ARRAY_OF(AnimBlendEntry) BlendEntryArray;
typedef ARRAY_OF(AnimNextEntry) NextEntryArray;

/// Plays clips with blends, queueing, sections, pose capture and root motion.
typedef struct {
    ClipEntryArray clips;
    BlendEntryArray blend_times;
    NextEntryArray next_clips;
    StrArray queue;
    BlendArray blends;
    AnimCapture capture;
    AnimPlayback current;
    AnimSection section;
    AnimRootMotion root_motion;
    AnimRootMotion root_motion_total;
    AnimEasing auto_capture_easing;
    AnimEasing capture_easing_wanted;
    Str assigned;
    Str section_from_marker;
    Str section_to_marker;
    Str root_motion_target;
    float section_from;
    float section_to;
    float speed_scale;
    float default_blend_seconds;
    float capture_duration;
    float auto_capture_duration;
    bool section_wanted;
    bool section_by_marker;
    bool auto_capture;
    bool capture_pending;
    bool enabled;
    bool playing;
    bool seeked;
    bool started;
    bool finished;
} AnimPlayer;

/// A player with the samples, event triggers and status changes of its latest advance.
typedef struct {
    AnimPlayer player;
    SampleArray samples;
    TriggerArray triggers;
    StatusArray status;
} AnimPlayerState;

/// Weighted blend of every sample that targets one property.
typedef struct {
    Str target;
    AnimTrackKind kind;
    Vec3 vector;
    Quat rotation;
    bool flag;
    float weight;
    float best;
} AnimAccumulator;

typedef ARRAY_OF(AnimAccumulator) AccumulatorArray;
typedef ARRAY_OF(AnimClip) ClipArray;

/// Owns the clips and players and advances every player by a time step.
typedef struct {
    ClipArray clips;
    AnimPlayerState* players;
    size_t player_count;
    AccumulatorArray accumulators;
    SizeArray key_scratch;
} AnimWorld;

/// Returns the player to its freshly constructed state, keeping storage.
void anim_player_reset(AnimPlayer* player);
/// Frees the player's storage.
void anim_player_free(AnimPlayer* player);
/// Registers `clip` under `name`, replacing a clip of that name.
void anim_player_add_clip(AnimPlayer* player, Str name, AnimClipHandle clip);
/// Whether a clip is registered under `name`.
bool anim_player_has_clip(const AnimPlayer* player, Str name);
/// Sets the speed multiplier of every playback.
void anim_player_set_speed_scale(AnimPlayer* player, float scale);
/// Sets the blend duration used when a pair has none.
void anim_player_set_default_blend_seconds(AnimPlayer* player, float seconds);
/// Sets the blend duration from clip `from` to clip `to`.
void anim_player_set_blend_seconds(AnimPlayer* player, Str from, Str to, float seconds);
/// Sets the clip that follows `from` when it finishes.
void anim_player_set_next(AnimPlayer* player, Str from, Str to);
/// Starts clip `name`, or the assigned clip when `name` is empty.
void anim_player_play(AnimPlayer* player, Str name, AnimPlay options);
/// Starts clip `name` backwards from its end.
void anim_player_play_backwards(AnimPlayer* player, Str name, float blend_seconds);
/// Starts clip `name` over the section `range`.
void anim_player_play_section(AnimPlayer* player, Str name, AnimSection range, AnimPlay options);
/// Starts clip `name` over the section between two markers.
void anim_player_play_section_with_markers(AnimPlayer* player, Str name, AnimMarkerRange markers, AnimPlay options);
/// Starts clip `name` and blends out from a captured pose over `duration_seconds`.
void anim_player_play_with_capture(AnimPlayer* player, Str name, float duration_seconds, AnimPlay options, AnimEasing easing);
/// Wants the whole clip again.
void anim_player_reset_section(AnimPlayer* player);
/// Sets whether every `play` captures the current pose.
void anim_player_set_auto_capture(AnimPlayer* player, bool capture);
/// Sets the duration of automatic captures.
void anim_player_set_auto_capture_duration(AnimPlayer* player, float seconds);
/// Sets the easing of automatic captures.
void anim_player_set_auto_capture_easing(AnimPlayer* player, AnimEasing easing);
/// Sets the target whose tracks drive root motion, and clears the motion.
void anim_player_set_root_motion_target(AnimPlayer* player, Str target);
/// Queues clip `name` to play after the current one finishes.
void anim_player_queue(AnimPlayer* player, Str name);
/// Moves the playback to `seconds`.
void anim_player_seek(AnimPlayer* player, float seconds);

/// Adds a clip, taking ownership of its storage, and returns its handle.
AnimClipHandle anim_world_add_clip(AnimWorld* world, AnimClip clip);
/// Creates `count` players.
void anim_world_add_players(AnimWorld* world, size_t count);
/// Advances every player by `delta_seconds`, rebuilding its samples, triggers and status.
void anim_world_advance(AnimWorld* world, float delta_seconds);
/// Stores the player's latest samples as the pose its pending capture blends out from.
void anim_world_apply_capture(AnimPlayerState* state);
/// Frees the clips, the players and the scratch.
void anim_world_free(AnimWorld* world);

#endif
