#include "animation.hpp"

#include <cmath>
#include <utility>

namespace bench::anim {

namespace {

/// `position` wrapped into [0, length), or zero for a non-positive length.
/// Uses floor where the engine uses fmod, which is a libm call.
float wrapped_position(float position, float length) {
    if (length <= 0.0f) return 0.0f;
    float result = position - std::floor(position / length) * length;
    if (result < 0.0f) result += length;
    return result;
}

/// `value` without its sign.
float absolute(float value) {
    return value < 0.0f ? -value : value;
}

} // namespace

void Player::reset() {
    clips_.clear();
    blend_times_.clear();
    next_.clear();
    queue_.clear();
    blends_.clear();
    capture_.samples.clear();
    capture_.seconds = 0.0f;
    capture_.left = 0.0f;
    capture_.easing = Easing{};
    current_ = Playback{};
    section_ = Section{};
    root_motion_ = RootMotion{};
    root_motion_total_ = RootMotion{};
    auto_capture_easing_ = Easing{};
    capture_easing_wanted_ = Easing{};
    assigned_ = {};
    section_from_marker_ = {};
    section_to_marker_ = {};
    root_motion_target_ = {};
    section_from_ = -1.0f;
    section_to_ = -1.0f;
    speed_scale_ = 1.0f;
    default_blend_seconds_ = 0.0f;
    capture_duration_ = 0.0f;
    auto_capture_duration_ = 0.0f;
    section_wanted_ = false;
    section_by_marker_ = false;
    auto_capture_ = false;
    capture_pending_ = false;
    enabled_ = true;
    playing_ = false;
    seeked_ = false;
    started_ = false;
    finished_ = false;
}

const ClipEntry* Player::entry(std::string_view name) const {
    for (const ClipEntry& candidate : clips_) {
        if (candidate.name == name) return &candidate;
    }
    return nullptr;
}

void Player::add_clip(std::string_view name, ClipHandle clip) {
    for (ClipEntry& candidate : clips_) {
        if (candidate.name == name) {
            candidate.clip = clip;
            return;
        }
    }
    clips_.push_back(ClipEntry{name, clip});
}

bool Player::has_clip(std::string_view name) const {
    return entry(name) != nullptr;
}

void Player::set_speed_scale(float scale) {
    speed_scale_ = scale;
}

void Player::set_default_blend_seconds(float seconds) {
    default_blend_seconds_ = max_float(seconds, 0.0f);
}

void Player::set_blend_seconds(std::string_view from, std::string_view to, float seconds) {
    for (BlendEntry& candidate : blend_times_) {
        if (candidate.from == from && candidate.to == to) {
            candidate.seconds = max_float(seconds, 0.0f);
            return;
        }
    }
    blend_times_.push_back(BlendEntry{from, to, max_float(seconds, 0.0f)});
}

float Player::blend_seconds(std::string_view from, std::string_view to) const {
    for (const BlendEntry& candidate : blend_times_) {
        if (candidate.from == from && candidate.to == to) return candidate.seconds;
    }
    return default_blend_seconds_;
}

void Player::set_next(std::string_view from, std::string_view to) {
    for (NextEntry& candidate : next_) {
        if (candidate.from == from) {
            candidate.to = to;
            return;
        }
    }
    next_.push_back(NextEntry{from, to});
}

std::string_view Player::next(std::string_view from) const {
    for (const NextEntry& candidate : next_) {
        if (candidate.from == from) return candidate.to;
    }
    return {};
}

void Player::play(std::string_view name, Play options) {
    const std::string_view wanted = name.empty() ? assigned_ : name;
    const ClipEntry* found = entry(wanted);
    if (found == nullptr) return;
    if (auto_capture_ && !capture_pending_ && auto_capture_duration_ > 0.0f) {
        capture_duration_ = auto_capture_duration_;
        capture_easing_wanted_ = auto_capture_easing_;
        capture_pending_ = true;
    }
    const float blend = options.blend_seconds < 0.0f ? blend_seconds(current_.name, found->name) : options.blend_seconds;
    if (blend > 0.0f && current_.clip != invalid_clip && current_.name != found->name) {
        blends_.push_back(Blend{current_, blend, blend});
    }
    Playback playback;
    playback.name = found->name;
    playback.clip = found->clip;
    playback.length = current_.name == found->name ? current_.length : 0.0f;
    playback.speed = options.speed;
    playback.position = options.from_end ? playback.length : 0.0f;
    if (current_.name == found->name && playing_) playback.position = current_.position;
    current_ = playback;
    assigned_ = current_.name;
    playing_ = true;
    started_ = true;
    finished_ = false;
    seeked_ = false;
}

void Player::play_backwards(std::string_view name, float blend_seconds) {
    play(name, Play{blend_seconds, -1.0f, true});
}

void Player::play_section(std::string_view name, SectionRange range, Play options) {
    set_section(range);
    play(name, options);
}

void Player::play_section_with_markers(std::string_view name, MarkerRange markers, Play options) {
    set_section_with_markers(markers);
    play(name, options);
}

void Player::play_with_capture(std::string_view name, float duration_seconds, Play options, Easing easing) {
    capture_duration_ = duration_seconds < 0.0f ? auto_capture_duration_ : duration_seconds;
    capture_easing_wanted_ = easing;
    capture_pending_ = capture_duration_ > 0.0f;
    play(name, options);
}

void Player::set_section(SectionRange range) {
    section_from_ = range.from;
    section_to_ = range.to;
    section_from_marker_ = {};
    section_to_marker_ = {};
    section_by_marker_ = false;
    section_wanted_ = true;
}

void Player::set_section_with_markers(MarkerRange markers) {
    section_from_marker_ = markers.from;
    section_to_marker_ = markers.to;
    section_from_ = -1.0f;
    section_to_ = -1.0f;
    section_by_marker_ = true;
    section_wanted_ = true;
}

void Player::reset_section() {
    section_wanted_ = false;
    section_by_marker_ = false;
    section_from_ = -1.0f;
    section_to_ = -1.0f;
    section_from_marker_ = {};
    section_to_marker_ = {};
    section_ = Section{};
}

void Player::set_auto_capture(bool capture) {
    auto_capture_ = capture;
}

void Player::set_auto_capture_duration(float seconds) {
    auto_capture_duration_ = max_float(seconds, 0.0f);
}

void Player::set_auto_capture_easing(Easing easing) {
    auto_capture_easing_ = easing;
}

bool Player::capture_pending() const {
    return capture_pending_;
}

void Player::set_root_motion_target(std::string_view target) {
    root_motion_target_ = target;
    reset_root_motion();
}

const RootMotion& Player::root_motion_accumulator() const {
    return root_motion_total_;
}

void Player::reset_root_motion() {
    root_motion_ = RootMotion{};
    root_motion_total_ = RootMotion{};
}

void Player::queue(std::string_view name) {
    queue_.push_back(name);
}

void Player::seek(float seconds) {
    current_.position = seconds;
    seeked_ = true;
}

float Player::position() const {
    return current_.position;
}

std::string_view Player::current_clip() const {
    return current_.name;
}

bool Player::finished() const {
    return finished_;
}

ClipHandle World::add_clip(Clip clip) {
    clips_.push_back(std::move(clip));
    return static_cast<ClipHandle>(clips_.size() - 1);
}

void World::add_players(size_t count) {
    players_.resize(players_.size() + count);
}

std::vector<PlayerState>& World::players() {
    return players_;
}

const Clip* World::find_clip(ClipHandle handle) const {
    return handle < clips_.size() ? &clips_[handle] : nullptr;
}

World::Accumulator& World::accumulator(std::string_view target, TrackKind kind) {
    for (Accumulator& candidate : accumulators_) {
        if (candidate.kind == kind && candidate.target == target) return candidate;
    }
    accumulators_.push_back(Accumulator{target, kind, vm::Vec3{}, vm::Quat{}, true, 0.0f, 0.0f});
    return accumulators_.back();
}

void World::accumulate_vector(const Track& track, Contribution at) {
    Accumulator& found = accumulator(track.target, track.kind);
    found.vector = found.vector + (sample_vector(track, at.time, track_rest_vector(track.kind)) * at.weight);
    found.weight += at.weight;
}

void World::accumulate_rotation(const Track& track, Contribution at) {
    Accumulator& found = accumulator(track.target, track.kind);
    const vm::Quat value = sample_rotation(track, at.time, vm::Quat{});
    const float total = found.weight + at.weight;
    found.rotation = found.weight <= 0.0f ? value : nlerp(found.rotation, value, total > 0.0f ? at.weight / total : 0.0f);
    found.weight = total;
}

void World::accumulate_flag(const Track& track, Contribution at) {
    Accumulator& found = accumulator(track.target, track.kind);
    if (at.weight > found.best) {
        found.flag = sample_flag(track, at.time, true);
        found.best = at.weight;
    }
    found.weight += at.weight;
}

void World::sample_playback(const Player::Playback& playback, std::string_view skip_target, float weight) {
    const Clip* asset = find_clip(playback.clip);
    if (asset == nullptr || weight <= 0.0f) return;
    for (const Track& track : asset->tracks) {
        if (!track.enabled || track.kind == TrackKind::Event) continue;
        if (!skip_target.empty() && track.target == skip_target && track.kind != TrackKind::Active) continue;
        switch (track.kind) {
        case TrackKind::Position:
        case TrackKind::Scale:
            accumulate_vector(track, Contribution{playback.position, weight});
            break;
        case TrackKind::Rotation:
            accumulate_rotation(track, Contribution{playback.position, weight});
            break;
        case TrackKind::Active:
            accumulate_flag(track, Contribution{playback.position, weight});
            break;
        case TrackKind::Event:
            break;
        }
    }
}

void World::sample_capture(const Player::Capture& capture, float weight) {
    if (weight <= 0.0f) return;
    for (const Sample& sample : capture.samples) {
        Accumulator& found = accumulator(sample.target, sample.kind);
        const float total = found.weight + weight;
        switch (sample.kind) {
        case TrackKind::Position:
        case TrackKind::Scale:
            found.vector = found.vector + (sample.vector * weight);
            break;
        case TrackKind::Rotation:
            found.rotation =
                found.weight <= 0.0f ? sample.rotation : nlerp(found.rotation, sample.rotation, total > 0.0f ? weight / total : 0.0f);
            break;
        case TrackKind::Active:
            if (weight > found.best) {
                found.flag = sample.flag;
                found.best = weight;
            }
            break;
        case TrackKind::Event:
            break;
        }
        found.weight = total;
    }
}

void World::collect_root_motion(Player& player, const Clip& asset, const Range& range) {
    player.root_motion_ = RootMotion{};
    if (player.root_motion_target_.empty()) return;
    for (const Track& track : asset.tracks) {
        if (!track.enabled || track.target != player.root_motion_target_) continue;
        if (track.kind == TrackKind::Position) {
            const vm::Vec3 first = sample_vector(track, range.from, vm::Vec3{});
            const vm::Vec3 last = sample_vector(track, range.to, vm::Vec3{});
            const vm::Vec3 delta = range.wrapped ? (sample_vector(track, range.high, vm::Vec3{}) - first) +
                                                       (last - sample_vector(track, range.low, vm::Vec3{}))
                                                 : last - first;
            player.root_motion_.position = player.root_motion_.position + delta;
        } else if (track.kind == TrackKind::Rotation) {
            const vm::Quat first = sample_rotation(track, range.from, vm::Quat{});
            const vm::Quat last = sample_rotation(track, range.to, vm::Quat{});
            vm::Quat delta = conjugate(first) * last;
            if (range.wrapped) {
                const vm::Quat high = sample_rotation(track, range.high, vm::Quat{});
                const vm::Quat low = sample_rotation(track, range.low, vm::Quat{});
                delta = (conjugate(first) * high) * (conjugate(low) * last);
            }
            player.root_motion_.rotation = vm::normalize(player.root_motion_.rotation * delta);
        }
    }
    player.root_motion_total_.position = player.root_motion_total_.position + player.root_motion_.position;
    player.root_motion_total_.rotation = vm::normalize(player.root_motion_total_.rotation * player.root_motion_.rotation);
}

void World::emit(PlayerState& state) {
    for (const Accumulator& accumulator : accumulators_) {
        if (accumulator.weight <= 0.0f) continue;
        Sample sample;
        sample.target = accumulator.target;
        sample.kind = accumulator.kind;
        sample.vector = accumulator.vector / accumulator.weight;
        sample.rotation = vm::normalize(accumulator.rotation);
        sample.flag = accumulator.flag;
        state.samples.push_back(sample);
    }
}

void World::collect_triggers(PlayerState& state, const Clip& asset, const Range& range) {
    for (const Track& track : asset.tracks) {
        if (!track.enabled || track.kind != TrackKind::Event) continue;
        keys_in_range(track, range, key_scratch_);
        for (const size_t index : key_scratch_) {
            state.triggers.push_back(Trigger{state.player.current_.name, track.target, track.keys[index].event, track.keys[index].time});
        }
    }
}

Section World::resolve_section(const Player& player, const Clip& asset) const {
    const float length = max_float(asset.duration_seconds, 0.0f);
    Section section{0.0f, length, false};
    if (!player.section_wanted_) return section;
    if (player.section_by_marker_) {
        const Marker* from = find_marker(asset, player.section_from_marker_);
        const Marker* to = find_marker(asset, player.section_to_marker_);
        section.from = from != nullptr ? from->time : 0.0f;
        section.to = to != nullptr ? to->time : length;
    } else {
        section.from = player.section_from_ < 0.0f ? 0.0f : player.section_from_;
        section.to = player.section_to_ < 0.0f ? length : player.section_to_;
    }
    section.from = clamp_float(section.from, 0.0f, length);
    section.to = clamp_float(section.to, 0.0f, length);
    if (section.to < section.from) std::swap(section.from, section.to);
    section.active = true;
    return section;
}

void World::advance_player(PlayerState& state, float delta) {
    Player& player = state.player;
    state.samples.clear();
    state.triggers.clear();
    state.status.clear();
    player.finished_ = false;
    if (!player.enabled_) return;
    const Clip* asset = find_clip(player.current_.clip);
    if (asset == nullptr) return;
    if (player.started_) {
        player.started_ = false;
        state.status.push_back(StatusChange{player.current_.name, Status::Started});
    }
    const Section section = resolve_section(player, *asset);
    player.section_ = section;
    player.current_.length = asset->duration_seconds;
    const bool seeked = player.seeked_;
    player.seeked_ = false;

    const float step = player.playing_ && !seeked ? delta * player.speed_scale_ * player.current_.speed : 0.0f;
    const float from = clamp_float(player.current_.position, section.from, section.to);
    float to = from + step;
    bool wrapped = false;
    if (step != 0.0f) {
        const float span = section.to - section.from;
        switch (asset->loop) {
        case LoopMode::None:
            if (to >= section.to) {
                to = section.to;
                player.finished_ = true;
            } else if (to <= section.from) {
                to = section.from;
                player.finished_ = true;
            }
            break;
        case LoopMode::Linear:
            if (to > section.to || to < section.from) {
                to = section.from + wrapped_position(to - section.from, span);
                wrapped = true;
            }
            break;
        case LoopMode::PingPong:
            if (to > section.to) {
                to = section.to;
                player.current_.speed = -player.current_.speed;
            } else if (to < section.from) {
                to = section.from;
                player.current_.speed = -player.current_.speed;
            }
            break;
        }
    }
    player.current_.position = clamp_float(to, section.from, section.to);

    float blended = 0.0f;
    for (Player::Blend& blend : player.blends_) {
        blend.left = max_float(blend.left - absolute(delta), 0.0f);
        blend.playback.position =
            clamp_float(blend.playback.position + (delta * player.speed_scale_ * blend.playback.speed), 0.0f, blend.playback.length);
        blended += blend.seconds > 0.0f ? blend.left / blend.seconds : 0.0f;
    }
    std::erase_if(player.blends_, [](const Player::Blend& blend) { return blend.left <= 0.0f; });
    float captured = 0.0f;
    if (player.capture_.left > 0.0f && player.capture_.seconds > 0.0f) {
        player.capture_.left = max_float(player.capture_.left - absolute(delta), 0.0f);
        captured = ease_curve(player.capture_.left / player.capture_.seconds, player.capture_.easing);
        if (player.capture_.left <= 0.0f) {
            player.capture_.samples.clear();
            player.capture_.seconds = 0.0f;
            player.capture_.left = 0.0f;
            player.capture_.easing = Easing{};
        }
    }
    blended = clamp_float(blended + captured, 0.0f, 1.0f);

    accumulators_.clear();
    sample_playback(player.current_, player.root_motion_target_, 1.0f - blended);
    for (const Player::Blend& blend : player.blends_) {
        sample_playback(blend.playback, player.root_motion_target_, blend.seconds > 0.0f ? blend.left / blend.seconds : 0.0f);
    }
    sample_capture(player.capture_, captured);
    emit(state);

    const Range range{from, player.current_.position, section.from, section.to, wrapped};
    collect_root_motion(player, *asset, step != 0.0f ? range : Range{from, from, section.from, section.to, false});

    if (step != 0.0f && blended < 1.0f) collect_triggers(state, *asset, range);

    player.capture_pending_ = false;
    if (!player.finished_) return;
    state.status.push_back(StatusChange{player.current_.name, Status::Finished});
    const std::string_view following = player.next(player.current_.name);
    if (!following.empty() && player.has_clip(following)) {
        player.play(following);
    } else if (!player.queue_.empty()) {
        const std::string_view wanted = player.queue_.front();
        player.queue_.erase(player.queue_.begin());
        player.play(wanted);
    } else {
        player.playing_ = false;
        return;
    }
    if (player.started_) {
        player.started_ = false;
        state.status.push_back(StatusChange{player.current_.name, Status::Started});
    }
}

void World::advance(float delta_seconds) {
    for (PlayerState& state : players_) advance_player(state, delta_seconds);
}

void World::apply_capture(PlayerState& state) {
    Player& player = state.player;
    if (!player.capture_pending_) return;
    player.capture_.samples.assign(state.samples.begin(), state.samples.end());
    player.capture_.seconds = player.capture_duration_;
    player.capture_.left = player.capture_duration_;
    player.capture_.easing = player.capture_easing_wanted_;
    player.capture_pending_ = false;
    player.blends_.clear();
}

} // namespace bench::anim
