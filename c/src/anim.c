#include "anim.h"

#include <stdlib.h>

#include "alloc.h"
#include "animation.h"
#include "hash.h"
#include "vecmath.h"

enum {
    PLAYERS = 1024,
    FRAMES = 48,
    SETS = 8,
    CLIPS_PER_PLAYER = 3,
    KEYS = 48,
    TRACKS = 8,
    MARKERS = 3,
    EVENT_NAMES = 4
};

static const float frame_seconds = 1.0f / 30.0f;
static const uint64_t digest_prime = 0x9e3779b97f4a7c15ull;

static const Str clip_names[CLIPS_PER_PLAYER] = { STR_INIT("idle"), STR_INIT("walk"), STR_INIT("jump") };
static const Str event_names[EVENT_NAMES] = { STR_INIT("step"), STR_INIT("splash"), STR_INIT("dust"), STR_INIT("whoosh") };
static const Str marker_names[MARKERS] = { STR_INIT("foot_l"), STR_INIT("foot_r"), STR_INIT("land") };
static const float marker_fractions[MARKERS] = { 0.2f, 0.6f, 0.9f };
static const Str root_path = STR_INIT("armature/root");

/// What one track of every clip animates.
typedef struct {
    AnimTrackKind kind;
    AnimInterpolation interpolation;
    Str target;
    bool travels;
} TrackSpec;

static const TrackSpec track_specs[TRACKS] = {
    { TRACK_POSITION, INTERP_LINEAR, STR_INIT("armature/root"), true },
    { TRACK_ROTATION, INTERP_LINEAR, STR_INIT("armature/root"), false },
    { TRACK_POSITION, INTERP_CUBIC, STR_INIT("armature/root/hips"), false },
    { TRACK_ROTATION, INTERP_CUBIC, STR_INIT("armature/root/hips"), false },
    { TRACK_ROTATION, INTERP_LINEAR, STR_INIT("armature/root/hips/spine/chest"), false },
    { TRACK_SCALE, INTERP_LINEAR, STR_INIT("armature/root/hips/spine/chest/head"), false },
    { TRACK_ACTIVE, INTERP_NEAREST, STR_INIT("armature/root/hips/spine/chest/arm_l/hand_l"), false },
    { TRACK_EVENT, INTERP_NEAREST, STR_INIT("armature/fx"), false },
};

typedef struct {
    AnimWorld world;
    uint64_t* digest;
} Anim;

/// Folds `value` into the running digest `state`.
static uint64_t fold(uint64_t state, uint64_t value) {
    state = (state ^ value) * digest_prime;
    return state ^ (state >> 32);
}

/// Two floats' bit patterns in one word.
static uint64_t pack(float low, float high) { return (uint64_t)f32_bits(low) | ((uint64_t)f32_bits(high) << 32); }

/// Folds the samples, triggers and status changes of one advance into `state`.
static uint64_t fold_player(uint64_t state, const AnimPlayerState* player) {
    for (size_t i = 0; i < player->samples.len; ++i) {
        const AnimSample* sample = &player->samples.data[i];
        uint64_t tag = (uint64_t)sample->flag | ((uint64_t)sample->kind << 1) | ((uint64_t)sample->target.len << 8);
        state = fold(state, pack(sample->vector.x, sample->vector.y));
        state = fold(state, pack(sample->vector.z, sample->rotation.x));
        state = fold(state, pack(sample->rotation.y, sample->rotation.z));
        state = fold(state, pack(sample->rotation.w, 0.0f) | (tag << 32));
    }
    for (size_t i = 0; i < player->triggers.len; ++i) {
        const AnimTrigger* trigger = &player->triggers.data[i];
        uint64_t tag = (uint64_t)(uint8_t)trigger->event.ptr[0] | ((uint64_t)trigger->target.len << 8) | ((uint64_t)trigger->clip.len << 16);
        state = fold(state, pack(trigger->time, 0.0f) | (tag << 32));
    }
    for (size_t i = 0; i < player->status.len; ++i) {
        const AnimStatusChange* change = &player->status.data[i];
        state = fold(state, ((uint64_t)change->clip.len << 1) | (uint64_t)change->status);
    }
    return fold(state, (uint64_t)player->samples.len | ((uint64_t)player->triggers.len << 16) | ((uint64_t)player->player.finished << 32));
}

/// The key time of key `index`, jittered by `unit` except at both ends.
static float key_time(size_t index, float spacing, float duration, float unit) {
    if (index == 0) return 0.0f;
    if (index == KEYS - 1) return duration;
    return ((float)index + (unit - 0.5f) * 0.5f) * spacing;
}

/// A random easing curve.
static AnimEasing random_easing(Rng* rng) {
    uint64_t transition = rng_next(rng) % 7;
    uint64_t ease = rng_next(rng) % 4;
    return anim_easing((AnimTransition)transition, (AnimEase)ease);
}

/// Key `index` of a track made to `spec`.
static AnimKey random_key(Rng* rng, const TrackSpec* spec, size_t index, float spacing, float duration) {
    AnimKey key = anim_key_default();
    float unit = rng_unit(rng);
    key.time = key_time(index, spacing, duration, unit);
    float position = (float)index;
    switch (spec->kind) {
    case TRACK_POSITION:
        if (spec->travels) {
            Vec3 jitter = v3_random(rng, -0.05f, 0.05f);
            key.vector = v3_add(jitter, (Vec3){ position * 0.1f, 0.0f, position * 0.2f });
        } else {
            key.vector = v3_random(rng, -1.0f, 1.0f);
        }
        break;
    case TRACK_SCALE:
        key.vector = v3_random(rng, 0.8f, 1.2f);
        break;
    case TRACK_ROTATION:
        key.rotation = quat_random(rng);
        break;
    case TRACK_ACTIVE:
        key.flag = rng_unit(rng) < 0.5f;
        break;
    case TRACK_EVENT: {
        uint64_t which = rng_next(rng) % EVENT_NAMES;
        key.event = event_names[which];
        break;
    }
    }
    key.easing = random_easing(rng);
    return key;
}

/// A clip of every track spec with `KEYS` keys each over `duration` seconds.
static AnimClip random_clip(Rng* rng, AnimLoopMode loop, float duration) {
    AnimClip clip;
    clip.duration_seconds = duration;
    clip.loop = loop;
    float spacing = duration / (float)(KEYS - 1);
    clip.track_count = TRACKS;
    clip.tracks = xalloc(TRACKS * sizeof(AnimTrack));
    for (size_t t = 0; t < TRACKS; ++t) {
        const TrackSpec* spec = &track_specs[t];
        AnimTrack* track = &clip.tracks[t];
        track->kind = spec->kind;
        track->interpolation = spec->interpolation;
        track->target = spec->target;
        track->enabled = true;
        track->key_count = KEYS;
        track->keys = xalloc(KEYS * sizeof(AnimKey));
        for (size_t k = 0; k < KEYS; ++k) track->keys[k] = random_key(rng, spec, k, spacing, duration);
    }
    clip.marker_count = MARKERS;
    clip.markers = xalloc(MARKERS * sizeof(AnimMarker));
    for (size_t m = 0; m < MARKERS; ++m) clip.markers[m] = (AnimMarker){ marker_names[m], duration * marker_fractions[m] };
    return clip;
}

/// A value in [0, 1) from 16 bits of `bits`.
static float unit_from_bits(uint64_t bits) { return (float)(bits & 0xFFFF) * (1.0f / 65536.0f); }

/// Returns player `index` to its initial clips, blends and playback.
static void configure_player(AnimPlayerState* state, size_t index) {
    state->samples.len = 0;
    state->triggers.len = 0;
    state->status.len = 0;
    AnimPlayer* player = &state->player;
    anim_player_reset(player);
    size_t set = index % SETS;
    for (size_t c = 0; c < CLIPS_PER_PLAYER; ++c) anim_player_add_clip(player, clip_names[c], (AnimClipHandle)(set * CLIPS_PER_PLAYER + c));
    anim_player_set_default_blend_seconds(player, 0.25f);
    anim_player_set_blend_seconds(player, STR("idle"), STR("walk"), 0.3f);
    anim_player_set_blend_seconds(player, STR("walk"), STR("jump"), 0.1f);
    anim_player_set_blend_seconds(player, STR("jump"), STR("walk"), 0.15f);
    anim_player_set_blend_seconds(player, STR("walk"), STR("idle"), 0.4f);
    anim_player_set_next(player, STR("jump"), STR("walk"));
    anim_player_set_next(player, STR("walk"), STR("idle"));
    anim_player_set_root_motion_target(player, root_path);
    anim_player_set_speed_scale(player, 0.75f + (float)(index % 5) * 0.125f);
    if (index % 4 == 0) {
        anim_player_set_auto_capture(player, true);
        anim_player_set_auto_capture_duration(player, 0.2f);
        anim_player_set_auto_capture_easing(player, anim_easing(TRANSITION_QUAD, EASE_OUT));
    }
    anim_player_play(player, clip_names[index % CLIPS_PER_PLAYER], ANIM_PLAY_DEFAULT);
}

/// Applies the scripted clip change, if any, of player `index` at `frame`.
static void script_player(AnimWorld* world, size_t index, uint32_t frame) {
    uint64_t roll = mix64(((uint64_t)index << 32) | frame);
    if ((roll & 31) != 0) return;
    AnimPlayerState* state = &world->players[index];
    AnimPlayer* player = &state->player;
    Str pick = clip_names[(roll >> 16) % CLIPS_PER_PLAYER];
    float unit = unit_from_bits(roll >> 24);
    switch ((roll >> 8) & 7) {
    case 0:
        anim_player_reset_section(player);
        anim_player_play(player, pick, ANIM_PLAY_DEFAULT);
        break;
    case 1:
        anim_player_play_with_capture(player, pick, 0.2f + 0.2f * unit, ANIM_PLAY_DEFAULT, anim_easing(TRANSITION_CUBIC, EASE_IN_OUT));
        break;
    case 2:
        anim_player_queue(player, pick);
        break;
    case 3:
        anim_player_play_backwards(player, pick, 0.15f);
        break;
    case 4:
        anim_player_play_section_with_markers(player, pick, (AnimMarkerRange){ STR("foot_l"), STR("land") }, ANIM_PLAY_DEFAULT);
        break;
    case 5:
        anim_player_seek(player, unit * 1.5f);
        break;
    case 6:
        anim_player_set_speed_scale(player, 0.5f + unit);
        break;
    default:
        anim_player_play_section(player, pick, (AnimSection){ 0.2f + 0.2f * unit, 0.9f }, ANIM_PLAY_DEFAULT);
        break;
    }
    if (player->capture_pending) anim_world_apply_capture(state);
}

/// Draws the clips and creates the players.
static void setup(void* state) {
    Anim* s = state;
    Rng rng = { 0xa41 };
    for (size_t set = 0; set < SETS; ++set) {
        float variant = (float)set;
        anim_world_add_clip(&s->world, random_clip(&rng, LOOP_LINEAR, 1.6f + 0.1f * variant));
        anim_world_add_clip(&s->world, random_clip(&rng, LOOP_PING_PONG, 1.2f + 0.05f * variant));
        anim_world_add_clip(&s->world, random_clip(&rng, LOOP_NONE, 0.9f + 0.02f * variant));
    }
    anim_world_add_players(&s->world, PLAYERS);
    s->digest = xalloc(PLAYERS * sizeof(uint64_t));
}

/// Resets every player, steps every frame, and hashes the digests, root motion and positions.
static uint64_t run(void* state) {
    Anim* s = state;
    AnimWorld* world = &s->world;
    for (size_t p = 0; p < PLAYERS; ++p) {
        configure_player(&world->players[p], p);
        s->digest[p] = 0;
    }
    for (uint32_t frame = 0; frame < FRAMES; ++frame) {
        for (size_t p = 0; p < PLAYERS; ++p) script_player(world, p, frame);
        anim_world_advance(world, frame_seconds);
        for (size_t p = 0; p < PLAYERS; ++p) s->digest[p] = fold_player(s->digest[p], &world->players[p]);
    }
    uint64_t h = 0;
    for (size_t p = 0; p < PLAYERS; ++p) {
        const AnimPlayer* player = &world->players[p].player;
        const AnimRootMotion* root = &player->root_motion_total;
        h = hash_add(h, s->digest[p]);
        h = hash_add(h, pack(root->position.x, root->position.y));
        h = hash_add(h, pack(root->position.z, root->rotation.x));
        h = hash_add(h, pack(root->rotation.y, root->rotation.z));
        h = hash_add(h, pack(root->rotation.w, player->current.position));
        h = hash_add(h, player->current.name.len);
    }
    return h;
}

/// Frees the world and the digests.
static void teardown(void* state) {
    Anim* s = state;
    anim_world_free(&s->world);
    free(s->digest);
}

const Case anim_case = { "anim", sizeof(Anim), setup, run, teardown };
