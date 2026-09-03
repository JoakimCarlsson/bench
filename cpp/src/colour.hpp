#pragma once

#include <cstdint>
#include <vector>

namespace bench {

/// Graph colouring: assign each of 524288 contacts the lowest colour not used
/// by another contact on either of its dynamic bodies, so every colour is a
/// set of contacts a solver can run in parallel. Bitmask per body, 31 colours
/// plus an overflow bucket.
class Colour {
public:
    static constexpr const char* name = "colour";

    struct Pair { uint32_t a, b; };

    Colour();
    uint64_t run();

private:
    static constexpr uint32_t bodies_n = 65536;
    static constexpr int contacts_n = 524288;
    static constexpr uint32_t overflow = 31;

    std::vector<uint8_t> dynamic_;
    std::vector<Pair> contacts_;
    std::vector<uint32_t> used_;
    std::vector<uint8_t> colour_;
};

} // namespace bench
