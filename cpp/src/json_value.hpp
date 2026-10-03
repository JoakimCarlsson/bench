#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace bench::json {

/// The type of a JSON value.
enum class Kind : uint8_t { Null, Bool, Number, String, Array, Object };

/// A JSON value: null, boolean, number, string, array or object. Objects keep
/// members in insertion order. The engine's `voxel::json::Value`, minus the
/// exceptions this build disables.
class Value final {
public:
    /// Creates a null value.
    Value() = default;
    /// Creates an empty value of the given kind.
    explicit Value(Kind kind) : kind_{kind} {}

    /// Creates a boolean value.
    [[nodiscard]] static Value boolean(bool value);
    /// Creates a number value.
    [[nodiscard]] static Value number(double value);
    /// Creates a string value, moving the text in.
    [[nodiscard]] static Value text(std::string value);
    /// Creates an empty array value.
    [[nodiscard]] static Value array();
    /// Creates an empty object value.
    [[nodiscard]] static Value object();

    /// Returns the kind of this value.
    [[nodiscard]] Kind kind() const noexcept { return kind_; }
    /// Tests the kind of this value.
    [[nodiscard]] bool is(Kind kind) const noexcept { return kind_ == kind; }

    /// Returns the boolean, or false for any other kind.
    [[nodiscard]] bool as_bool() const noexcept { return boolean_; }
    /// Returns the number, or zero for any other kind.
    [[nodiscard]] double as_number() const noexcept { return number_; }
    /// Returns the string, or an empty string for any other kind.
    [[nodiscard]] const std::string& as_string() const noexcept { return text_; }
    /// Returns the array elements; empty for any other kind.
    [[nodiscard]] const std::vector<Value>& items() const noexcept { return items_; }
    /// Returns the object members in insertion order; empty for any other kind.
    [[nodiscard]] const std::vector<std::pair<std::string, Value>>& members() const noexcept {
        return members_;
    }

    /// Appends an element and turns this value into an array.
    void push(Value value);
    /// Sets an object member, replacing an existing one, and turns this value
    /// into an object.
    void set(std::string key, Value value);
    /// Looks up an object member; null when absent.
    [[nodiscard]] const Value* find(std::string_view key) const noexcept;

    /// Reads a boolean member, or `fallback` when absent or not a boolean.
    [[nodiscard]] bool bool_or(std::string_view key, bool fallback) const;
    /// Reads a number member, or `fallback` when absent or not a number.
    [[nodiscard]] double number_or(std::string_view key, double fallback) const;
    /// Reads a string member, or `fallback` when absent or not a string.
    [[nodiscard]] std::string string_or(std::string_view key, std::string fallback) const;
    /// Reads a string member without copying it, or `fallback` when absent or
    /// not a string.
    [[nodiscard]] std::string_view text_or(std::string_view key, std::string_view fallback) const;

private:
    Kind kind_{Kind::Null};
    bool boolean_{};
    double number_{};
    std::string text_;
    std::vector<Value> items_;
    std::vector<std::pair<std::string, Value>> members_;
};

/// Parses JSON text into `out`. Returns false on malformed input, trailing
/// content or nesting deeper than 128.
[[nodiscard]] bool parse(std::string_view source, Value& out);

} // namespace bench::json
