#pragma once

#include <cstdint>
#include <vector>

#include "animation.hpp"

namespace bench {

/// Animation playback: 1024 players of 3 clips each, every clip 8 tracks of 48
/// keys, stepped for 48 frames of 1/30 s. The engine's `AnimationWorld`
/// samples every playing clip and every blend from the previous clip, by key
/// search, easing and interpolation, accumulates the samples by target path
/// in a string-keyed accumulator list, emits per-target samples, sums root
/// motion, fires event keys, and chains to the next or queued clip. A scripted
/// driver changes clips, blends from captured poses, queues, seeks and plays
/// sections. Differences from the engine: slerp is a normalised lerp, sine and
/// exponential easing are polynomials, and `fmod` is a floor; each keeps the
/// kernel free of libm calls, which round differently between languages.
class Anim {
public:
    static constexpr const char* name = "anim";

    Anim();
    uint64_t run();

private:
    /// Returns player `index` to its initial clips, blends and playback.
    void configure_player(size_t index);
    /// Applies the scripted clip change, if any, of player `index` at `frame`.
    void script_player(size_t index, uint32_t frame);

    anim::World world_{};
    std::vector<uint64_t> digest_{};
};

} // namespace bench
