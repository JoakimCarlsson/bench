#include "animation.h"

#include <math.h>
#include <stdlib.h>

#include "alloc.h"

/// `position` wrapped into [0, length), or zero for a non-positive length.
/// Uses floor where the engine uses fmod, which is a libm call.
static float wrapped_position(float position, float length) {
    if (length <= 0.0f) return 0.0f;
    float result = position - floorf(position / length) * length;
    if (result < 0.0f) result += length;
    return result;
}

/// `value` without its sign.
static float absolute(float value) { return value < 0.0f ? -value : value; }

/// Nothing playing, at unit speed.
static AnimPlayback playback_default(void) {
    return (AnimPlayback){ STR(""), ANIM_INVALID_CLIP, 0.0f, 0.0f, 1.0f };
}

/// The easing a fresh player and capture hold.
static AnimEasing easing_default(void) { return anim_easing(TRANSITION_LINEAR, EASE_IN_OUT); }

/// Drops the pose and the fade, keeping storage.
static void capture_clear(AnimCapture* capture) {
    capture->samples.len = 0;
    capture->seconds = 0.0f;
    capture->left = 0.0f;
    capture->easing = easing_default();
}

void anim_player_reset(AnimPlayer* player) {
    player->clips.len = 0;
    player->blend_times.len = 0;
    player->next_clips.len = 0;
    player->queue.len = 0;
    player->blends.len = 0;
    capture_clear(&player->capture);
    player->current = playback_default();
    player->section = (AnimSection){ 0.0f, 0.0f };
    player->root_motion = (AnimRootMotion){ { 0.0f, 0.0f, 0.0f }, quat_identity() };
    player->root_motion_total = player->root_motion;
    player->auto_capture_easing = easing_default();
    player->capture_easing_wanted = easing_default();
    player->assigned = STR("");
    player->section_from_marker = STR("");
    player->section_to_marker = STR("");
    player->root_motion_target = STR("");
    player->section_from = -1.0f;
    player->section_to = -1.0f;
    player->speed_scale = 1.0f;
    player->default_blend_seconds = 0.0f;
    player->capture_duration = 0.0f;
    player->auto_capture_duration = 0.0f;
    player->section_wanted = false;
    player->section_by_marker = false;
    player->auto_capture = false;
    player->capture_pending = false;
    player->enabled = true;
    player->playing = false;
    player->seeked = false;
    player->started = false;
    player->finished = false;
}

void anim_player_free(AnimPlayer* player) {
    ARRAY_FREE(player->clips);
    ARRAY_FREE(player->blend_times);
    ARRAY_FREE(player->next_clips);
    ARRAY_FREE(player->queue);
    ARRAY_FREE(player->blends);
    ARRAY_FREE(player->capture.samples);
}

/// The clip registered under `name`, or null.
static const AnimClipEntry* player_entry(const AnimPlayer* player, Str name) {
    for (size_t i = 0; i < player->clips.len; ++i) {
        if (str_eq(player->clips.data[i].name, name)) return &player->clips.data[i];
    }
    return NULL;
}

void anim_player_add_clip(AnimPlayer* player, Str name, AnimClipHandle clip) {
    for (size_t i = 0; i < player->clips.len; ++i) {
        if (str_eq(player->clips.data[i].name, name)) {
            player->clips.data[i].clip = clip;
            return;
        }
    }
    ARRAY_PUSH(player->clips, ((AnimClipEntry){ name, clip }));
}

bool anim_player_has_clip(const AnimPlayer* player, Str name) { return player_entry(player, name) != NULL; }

void anim_player_set_speed_scale(AnimPlayer* player, float scale) { player->speed_scale = scale; }

void anim_player_set_default_blend_seconds(AnimPlayer* player, float seconds) {
    player->default_blend_seconds = max_float(seconds, 0.0f);
}

void anim_player_set_blend_seconds(AnimPlayer* player, Str from, Str to, float seconds) {
    for (size_t i = 0; i < player->blend_times.len; ++i) {
        AnimBlendEntry* candidate = &player->blend_times.data[i];
        if (str_eq(candidate->from, from) && str_eq(candidate->to, to)) {
            candidate->seconds = max_float(seconds, 0.0f);
            return;
        }
    }
    ARRAY_PUSH(player->blend_times, ((AnimBlendEntry){ from, to, max_float(seconds, 0.0f) }));
}

/// The blend duration from clip `from` to clip `to`.
static float player_blend_seconds(const AnimPlayer* player, Str from, Str to) {
    for (size_t i = 0; i < player->blend_times.len; ++i) {
        const AnimBlendEntry* candidate = &player->blend_times.data[i];
        if (str_eq(candidate->from, from) && str_eq(candidate->to, to)) return candidate->seconds;
    }
    return player->default_blend_seconds;
}

void anim_player_set_next(AnimPlayer* player, Str from, Str to) {
    for (size_t i = 0; i < player->next_clips.len; ++i) {
        AnimNextEntry* candidate = &player->next_clips.data[i];
        if (str_eq(candidate->from, from)) {
            candidate->to = to;
            return;
        }
    }
    ARRAY_PUSH(player->next_clips, ((AnimNextEntry){ from, to }));
}

/// The clip that follows `from`, or empty.
static Str player_next(const AnimPlayer* player, Str from) {
    for (size_t i = 0; i < player->next_clips.len; ++i) {
        const AnimNextEntry* candidate = &player->next_clips.data[i];
        if (str_eq(candidate->from, from)) return candidate->to;
    }
    return STR("");
}

void anim_player_play(AnimPlayer* player, Str name, AnimPlay options) {
    Str wanted = name.len == 0 ? player->assigned : name;
    const AnimClipEntry* found = player_entry(player, wanted);
    if (found == NULL) return;
    if (player->auto_capture && !player->capture_pending && player->auto_capture_duration > 0.0f) {
        player->capture_duration = player->auto_capture_duration;
        player->capture_easing_wanted = player->auto_capture_easing;
        player->capture_pending = true;
    }
    float blend = options.blend_seconds < 0.0f ? player_blend_seconds(player, player->current.name, found->name) : options.blend_seconds;
    if (blend > 0.0f && player->current.clip != ANIM_INVALID_CLIP && !str_eq(player->current.name, found->name)) {
        ARRAY_PUSH(player->blends, ((AnimBlend){ player->current, blend, blend }));
    }
    bool same = str_eq(player->current.name, found->name);
    AnimPlayback playback = playback_default();
    playback.name = found->name;
    playback.clip = found->clip;
    playback.length = same ? player->current.length : 0.0f;
    playback.speed = options.speed;
    playback.position = options.from_end ? playback.length : 0.0f;
    if (same && player->playing) playback.position = player->current.position;
    player->current = playback;
    player->assigned = player->current.name;
    player->playing = true;
    player->started = true;
    player->finished = false;
    player->seeked = false;
}

void anim_player_play_backwards(AnimPlayer* player, Str name, float blend_seconds) {
    anim_player_play(player, name, (AnimPlay){ blend_seconds, -1.0f, true });
}

/// Wants the section `range`.
static void player_set_section(AnimPlayer* player, AnimSection range) {
    player->section_from = range.from;
    player->section_to = range.to;
    player->section_from_marker = STR("");
    player->section_to_marker = STR("");
    player->section_by_marker = false;
    player->section_wanted = true;
}

/// Wants the section between two markers.
static void player_set_section_with_markers(AnimPlayer* player, AnimMarkerRange markers) {
    player->section_from_marker = markers.from;
    player->section_to_marker = markers.to;
    player->section_from = -1.0f;
    player->section_to = -1.0f;
    player->section_by_marker = true;
    player->section_wanted = true;
}

void anim_player_play_section(AnimPlayer* player, Str name, AnimSection range, AnimPlay options) {
    player_set_section(player, range);
    anim_player_play(player, name, options);
}

void anim_player_play_section_with_markers(AnimPlayer* player, Str name, AnimMarkerRange markers, AnimPlay options) {
    player_set_section_with_markers(player, markers);
    anim_player_play(player, name, options);
}

void anim_player_play_with_capture(AnimPlayer* player, Str name, float duration_seconds, AnimPlay options, AnimEasing easing) {
    player->capture_duration = duration_seconds < 0.0f ? player->auto_capture_duration : duration_seconds;
    player->capture_easing_wanted = easing;
    player->capture_pending = player->capture_duration > 0.0f;
    anim_player_play(player, name, options);
}

void anim_player_reset_section(AnimPlayer* player) {
    player->section_wanted = false;
    player->section_by_marker = false;
    player->section_from = -1.0f;
    player->section_to = -1.0f;
    player->section_from_marker = STR("");
    player->section_to_marker = STR("");
    player->section = (AnimSection){ 0.0f, 0.0f };
}

void anim_player_set_auto_capture(AnimPlayer* player, bool capture) { player->auto_capture = capture; }

void anim_player_set_auto_capture_duration(AnimPlayer* player, float seconds) {
    player->auto_capture_duration = max_float(seconds, 0.0f);
}

void anim_player_set_auto_capture_easing(AnimPlayer* player, AnimEasing easing) { player->auto_capture_easing = easing; }

void anim_player_set_root_motion_target(AnimPlayer* player, Str target) {
    player->root_motion_target = target;
    player->root_motion = (AnimRootMotion){ { 0.0f, 0.0f, 0.0f }, quat_identity() };
    player->root_motion_total = player->root_motion;
}

void anim_player_queue(AnimPlayer* player, Str name) { ARRAY_PUSH(player->queue, name); }

void anim_player_seek(AnimPlayer* player, float seconds) {
    player->current.position = seconds;
    player->seeked = true;
}

AnimClipHandle anim_world_add_clip(AnimWorld* world, AnimClip clip) {
    ARRAY_PUSH(world->clips, clip);
    return (AnimClipHandle)(world->clips.len - 1);
}

void anim_world_add_players(AnimWorld* world, size_t count) {
    world->players = xalloc(count * sizeof(AnimPlayerState));
    world->player_count = count;
    for (size_t i = 0; i < count; ++i) anim_player_reset(&world->players[i].player);
}

/// The clip behind `handle`, or null.
static const AnimClip* find_clip(const AnimWorld* world, AnimClipHandle handle) {
    return handle < world->clips.len ? &world->clips.data[handle] : NULL;
}

/// Finds or creates the accumulator of a property.
static AnimAccumulator* accumulator(AnimWorld* world, Str target, AnimTrackKind kind) {
    for (size_t i = 0; i < world->accumulators.len; ++i) {
        AnimAccumulator* candidate = &world->accumulators.data[i];
        if (candidate->kind == kind && str_eq(candidate->target, target)) return candidate;
    }
    ARRAY_PUSH(world->accumulators, ((AnimAccumulator){ target, kind, { 0.0f, 0.0f, 0.0f }, quat_identity(), true, 0.0f, 0.0f }));
    return &ARRAY_BACK(world->accumulators);
}

/// Adds a weighted position or scale sample of a track.
static void accumulate_vector(AnimWorld* world, const AnimTrack* track, float time, float weight) {
    AnimAccumulator* found = accumulator(world, track->target, track->kind);
    found->vector = v3_add(found->vector, v3_scale(anim_sample_vector(track, time, anim_track_rest_vector(track->kind)), weight));
    found->weight += weight;
}

/// Blends a rotation sample of a track into its accumulator.
static void accumulate_rotation(AnimWorld* world, const AnimTrack* track, float time, float weight) {
    AnimAccumulator* found = accumulator(world, track->target, track->kind);
    Quat value = anim_sample_rotation(track, time, quat_identity());
    float total = found->weight + weight;
    found->rotation = found->weight <= 0.0f ? value : anim_nlerp(found->rotation, value, total > 0.0f ? weight / total : 0.0f);
    found->weight = total;
}

/// Takes a flag sample of a track when it outweighs those accumulated so far.
static void accumulate_flag(AnimWorld* world, const AnimTrack* track, float time, float weight) {
    AnimAccumulator* found = accumulator(world, track->target, track->kind);
    if (weight > found->best) {
        found->flag = anim_sample_flag(track, time, true);
        found->best = weight;
    }
    found->weight += weight;
}

/// Accumulates every enabled track of a playback at its position.
static void sample_playback(AnimWorld* world, const AnimPlayback* playback, Str skip_target, float weight) {
    const AnimClip* asset = find_clip(world, playback->clip);
    if (asset == NULL || weight <= 0.0f) return;
    for (size_t i = 0; i < asset->track_count; ++i) {
        const AnimTrack* track = &asset->tracks[i];
        if (!track->enabled || track->kind == TRACK_EVENT) continue;
        if (skip_target.len != 0 && str_eq(track->target, skip_target) && track->kind != TRACK_ACTIVE) continue;
        switch (track->kind) {
        case TRACK_POSITION:
        case TRACK_SCALE:
            accumulate_vector(world, track, playback->position, weight);
            break;
        case TRACK_ROTATION:
            accumulate_rotation(world, track, playback->position, weight);
            break;
        case TRACK_ACTIVE:
            accumulate_flag(world, track, playback->position, weight);
            break;
        case TRACK_EVENT:
            break;
        }
    }
}

/// Accumulates a captured pose.
static void sample_capture(AnimWorld* world, const AnimCapture* capture, float weight) {
    if (weight <= 0.0f) return;
    for (size_t i = 0; i < capture->samples.len; ++i) {
        const AnimSample* sample = &capture->samples.data[i];
        AnimAccumulator* found = accumulator(world, sample->target, sample->kind);
        float total = found->weight + weight;
        switch (sample->kind) {
        case TRACK_POSITION:
        case TRACK_SCALE:
            found->vector = v3_add(found->vector, v3_scale(sample->vector, weight));
            break;
        case TRACK_ROTATION:
            found->rotation =
                found->weight <= 0.0f ? sample->rotation : anim_nlerp(found->rotation, sample->rotation, total > 0.0f ? weight / total : 0.0f);
            break;
        case TRACK_ACTIVE:
            if (weight > found->best) {
                found->flag = sample->flag;
                found->best = weight;
            }
            break;
        case TRACK_EVENT:
            break;
        }
        found->weight = total;
    }
}

/// Adds the root target's motion over a range to the player's running total.
static void collect_root_motion(AnimPlayer* player, const AnimClip* asset, const AnimRange* range) {
    const Vec3 zero = { 0.0f, 0.0f, 0.0f };
    player->root_motion = (AnimRootMotion){ zero, quat_identity() };
    if (player->root_motion_target.len == 0) return;
    for (size_t i = 0; i < asset->track_count; ++i) {
        const AnimTrack* track = &asset->tracks[i];
        if (!track->enabled || !str_eq(track->target, player->root_motion_target)) continue;
        if (track->kind == TRACK_POSITION) {
            Vec3 first = anim_sample_vector(track, range->from, zero);
            Vec3 last = anim_sample_vector(track, range->to, zero);
            Vec3 delta;
            if (range->wrapped) {
                Vec3 high = anim_sample_vector(track, range->high, zero);
                Vec3 low = anim_sample_vector(track, range->low, zero);
                delta = v3_add(v3_sub(high, first), v3_sub(last, low));
            } else {
                delta = v3_sub(last, first);
            }
            player->root_motion.position = v3_add(player->root_motion.position, delta);
        } else if (track->kind == TRACK_ROTATION) {
            Quat first = anim_sample_rotation(track, range->from, quat_identity());
            Quat last = anim_sample_rotation(track, range->to, quat_identity());
            Quat delta = quat_mul(anim_conjugate(first), last);
            if (range->wrapped) {
                Quat high = anim_sample_rotation(track, range->high, quat_identity());
                Quat low = anim_sample_rotation(track, range->low, quat_identity());
                delta = quat_mul(quat_mul(anim_conjugate(first), high), quat_mul(anim_conjugate(low), last));
            }
            player->root_motion.rotation = quat_normalize(quat_mul(player->root_motion.rotation, delta));
        }
    }
    player->root_motion_total.position = v3_add(player->root_motion_total.position, player->root_motion.position);
    player->root_motion_total.rotation = quat_normalize(quat_mul(player->root_motion_total.rotation, player->root_motion.rotation));
}

/// Converts the accumulators into the player's output samples.
static void emit(const AnimWorld* world, AnimPlayerState* state) {
    for (size_t i = 0; i < world->accumulators.len; ++i) {
        const AnimAccumulator* accumulated = &world->accumulators.data[i];
        if (accumulated->weight <= 0.0f) continue;
        AnimSample sample;
        sample.target = accumulated->target;
        sample.kind = accumulated->kind;
        sample.vector = v3_div(accumulated->vector, accumulated->weight);
        sample.rotation = quat_normalize(accumulated->rotation);
        sample.flag = accumulated->flag;
        ARRAY_PUSH(state->samples, sample);
    }
}

/// Records the event keys of the playing clip passed over by `range`.
static void collect_triggers(AnimWorld* world, AnimPlayerState* state, const AnimClip* asset, const AnimRange* range) {
    for (size_t i = 0; i < asset->track_count; ++i) {
        const AnimTrack* track = &asset->tracks[i];
        if (!track->enabled || track->kind != TRACK_EVENT) continue;
        anim_keys_in_range(track, range, &world->key_scratch);
        for (size_t k = 0; k < world->key_scratch.len; ++k) {
            const AnimKey* key = &track->keys[world->key_scratch.data[k]];
            ARRAY_PUSH(state->triggers, ((AnimTrigger){ state->player.current.name, track->target, key->event, key->time }));
        }
    }
}

/// Resolves the section a player wants into clamped times for a clip.
static AnimSection resolve_section(const AnimPlayer* player, const AnimClip* asset) {
    float length = max_float(asset->duration_seconds, 0.0f);
    AnimSection section = { 0.0f, length };
    if (!player->section_wanted) return section;
    if (player->section_by_marker) {
        const AnimMarker* from = anim_find_marker(asset, player->section_from_marker);
        const AnimMarker* to = anim_find_marker(asset, player->section_to_marker);
        section.from = from != NULL ? from->time : 0.0f;
        section.to = to != NULL ? to->time : length;
    } else {
        section.from = player->section_from < 0.0f ? 0.0f : player->section_from;
        section.to = player->section_to < 0.0f ? length : player->section_to;
    }
    section.from = clamp_float(section.from, 0.0f, length);
    section.to = clamp_float(section.to, 0.0f, length);
    if (section.to < section.from) {
        float swapped = section.from;
        section.from = section.to;
        section.to = swapped;
    }
    return section;
}

/// Moves the playback of `player` by `step` seconds from `from`, looping, bouncing or finishing at the section's ends.
static void step_playback(AnimPlayer* player, const AnimClip* asset, AnimSection section, float step, float from, bool* wrapped) {
    float to = from + step;
    if (step != 0.0f) {
        float span = section.to - section.from;
        switch (asset->loop) {
        case LOOP_NONE:
            if (to >= section.to) {
                to = section.to;
                player->finished = true;
            } else if (to <= section.from) {
                to = section.from;
                player->finished = true;
            }
            break;
        case LOOP_LINEAR:
            if (to > section.to || to < section.from) {
                to = section.from + wrapped_position(to - section.from, span);
                *wrapped = true;
            }
            break;
        case LOOP_PING_PONG:
            if (to > section.to) {
                to = section.to;
                player->current.speed = -player->current.speed;
            } else if (to < section.from) {
                to = section.from;
                player->current.speed = -player->current.speed;
            }
            break;
        }
    }
    player->current.position = clamp_float(to, section.from, section.to);
}

/// Fades every blend by `delta`, drops the spent ones, and returns their summed weight.
static float fade_blends(AnimPlayer* player, float delta) {
    float blended = 0.0f;
    for (size_t i = 0; i < player->blends.len; ++i) {
        AnimBlend* blend = &player->blends.data[i];
        blend->left = max_float(blend->left - absolute(delta), 0.0f);
        blend->playback.position =
            clamp_float(blend->playback.position + (delta * player->speed_scale * blend->playback.speed), 0.0f, blend->playback.length);
        blended += blend->seconds > 0.0f ? blend->left / blend->seconds : 0.0f;
    }
    size_t kept = 0;
    for (size_t i = 0; i < player->blends.len; ++i) {
        if (player->blends.data[i].left > 0.0f) player->blends.data[kept++] = player->blends.data[i];
    }
    player->blends.len = kept;
    return blended;
}

/// Moves a finished player on to its next or queued clip, if it has one.
static void chain_clip(AnimPlayer* player, AnimPlayerState* state) {
    ARRAY_PUSH(state->status, ((AnimStatusChange){ player->current.name, STATUS_FINISHED }));
    Str following = player_next(player, player->current.name);
    if (following.len != 0 && anim_player_has_clip(player, following)) {
        anim_player_play(player, following, ANIM_PLAY_DEFAULT);
    } else if (player->queue.len != 0) {
        Str wanted = player->queue.data[0];
        memmove(player->queue.data, player->queue.data + 1, (player->queue.len - 1) * sizeof(Str));
        player->queue.len -= 1;
        anim_player_play(player, wanted, ANIM_PLAY_DEFAULT);
    } else {
        player->playing = false;
        return;
    }
    if (player->started) {
        player->started = false;
        ARRAY_PUSH(state->status, ((AnimStatusChange){ player->current.name, STATUS_STARTED }));
    }
}

/// Advances one player, then rebuilds its samples, triggers and status changes.
static void advance_player(AnimWorld* world, AnimPlayerState* state, float delta) {
    AnimPlayer* player = &state->player;
    state->samples.len = 0;
    state->triggers.len = 0;
    state->status.len = 0;
    player->finished = false;
    if (!player->enabled) return;
    const AnimClip* asset = find_clip(world, player->current.clip);
    if (asset == NULL) return;
    if (player->started) {
        player->started = false;
        ARRAY_PUSH(state->status, ((AnimStatusChange){ player->current.name, STATUS_STARTED }));
    }
    AnimSection section = resolve_section(player, asset);
    player->section = section;
    player->current.length = asset->duration_seconds;
    bool seeked = player->seeked;
    player->seeked = false;

    float step = player->playing && !seeked ? delta * player->speed_scale * player->current.speed : 0.0f;
    float from = clamp_float(player->current.position, section.from, section.to);
    bool wrapped = false;
    step_playback(player, asset, section, step, from, &wrapped);

    float blended = fade_blends(player, delta);
    float captured = 0.0f;
    if (player->capture.left > 0.0f && player->capture.seconds > 0.0f) {
        player->capture.left = max_float(player->capture.left - absolute(delta), 0.0f);
        captured = anim_ease_curve(player->capture.left / player->capture.seconds, player->capture.easing);
        if (player->capture.left <= 0.0f) capture_clear(&player->capture);
    }
    blended = clamp_float(blended + captured, 0.0f, 1.0f);

    world->accumulators.len = 0;
    sample_playback(world, &player->current, player->root_motion_target, 1.0f - blended);
    for (size_t i = 0; i < player->blends.len; ++i) {
        const AnimBlend* blend = &player->blends.data[i];
        sample_playback(world, &blend->playback, player->root_motion_target, blend->seconds > 0.0f ? blend->left / blend->seconds : 0.0f);
    }
    sample_capture(world, &player->capture, captured);
    emit(world, state);

    AnimRange range = { from, player->current.position, section.from, section.to, wrapped };
    AnimRange still = { from, from, section.from, section.to, false };
    collect_root_motion(player, asset, step != 0.0f ? &range : &still);

    if (step != 0.0f && blended < 1.0f) collect_triggers(world, state, asset, &range);

    player->capture_pending = false;
    if (player->finished) chain_clip(player, state);
}

void anim_world_advance(AnimWorld* world, float delta_seconds) {
    for (size_t i = 0; i < world->player_count; ++i) advance_player(world, &world->players[i], delta_seconds);
}

void anim_world_apply_capture(AnimPlayerState* state) {
    AnimPlayer* player = &state->player;
    if (!player->capture_pending) return;
    player->capture.samples.len = 0;
    ARRAY_RESERVE(player->capture.samples, state->samples.len);
    if (state->samples.len != 0) memcpy(player->capture.samples.data, state->samples.data, state->samples.len * sizeof(AnimSample));
    player->capture.samples.len = state->samples.len;
    player->capture.seconds = player->capture_duration;
    player->capture.left = player->capture_duration;
    player->capture.easing = player->capture_easing_wanted;
    player->capture_pending = false;
    player->blends.len = 0;
}

void anim_world_free(AnimWorld* world) {
    for (size_t i = 0; i < world->clips.len; ++i) anim_clip_free(&world->clips.data[i]);
    ARRAY_FREE(world->clips);
    for (size_t i = 0; i < world->player_count; ++i) {
        anim_player_free(&world->players[i].player);
        ARRAY_FREE(world->players[i].samples);
        ARRAY_FREE(world->players[i].triggers);
        ARRAY_FREE(world->players[i].status);
    }
    free(world->players);
    ARRAY_FREE(world->accumulators);
    ARRAY_FREE(world->key_scratch);
}
