#pragma once

#include <cstdint>

namespace bench {

/// Monotonic nanoseconds from an unspecified origin.
uint64_t now_ns();

} // namespace bench
