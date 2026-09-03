#pragma once

#include <cstdint>
#include <vector>

namespace bench {

/// Island partition: union-find over 524288 contacts between 65536 bodies,
/// joining only pairs where both are dynamic, then labelling every dynamic
/// body by the lowest slot in its island.
class Islands {
public:
    static constexpr const char* name = "islands";

    struct Pair { uint32_t a, b; };

    Islands();
    uint64_t run();

private:
    static constexpr uint32_t bodies_n = 65536;
    static constexpr int contacts_n = 524288;

    uint32_t find(uint32_t i);

    std::vector<uint8_t> dynamic_;
    std::vector<Pair> contacts_;
    std::vector<uint32_t> parent_;
};

} // namespace bench
