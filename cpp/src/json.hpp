#pragma once

#include <cstdint>
#include <string>

namespace bench {

/// Scene load: the engine's JSON reader over a generated scene document of a
/// few megabytes. The constructor writes the text with integer arithmetic
/// only, so it is the same bytes in every language. Each run parses it into a
/// `Value` tree with the engine's strict recursive descent parser, walks the
/// entities the way the entity document code does (components looked up by
/// key, vectors and transforms read from number arrays), folds everything it
/// reads into a checksum, then drops the tree.
class Json {
public:
    static constexpr const char* name = "json";

    Json();
    uint64_t run();

    /// The generated document text.
    [[nodiscard]] const std::string& text() const noexcept { return text_; }

private:
    std::string text_;
};

} // namespace bench
