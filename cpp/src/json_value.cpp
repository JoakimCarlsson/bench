#include "json_value.hpp"

#include <charconv>

namespace bench::json {
namespace {

/// A recursive descent JSON parser over a string view, as the engine's.
class Parser final {
public:
    /// Creates a parser over `source`, which must outlive it.
    explicit Parser(std::string_view source) noexcept : source_{source} {}

    /// Parses the whole source as one value into `out`.
    [[nodiscard]] bool run(Value& out) {
        skip_space();
        if (!parse_value(0, out)) return false;
        skip_space();
        return offset_ == source_.size();
    }

private:
    /// Advances past JSON whitespace.
    void skip_space() {
        while (offset_ < source_.size()) {
            const char character = source_[offset_];
            if (character == ' ' || character == '\t' || character == '\n' || character == '\r') {
                ++offset_;
                continue;
            }
            break;
        }
    }

    /// Consumes `character` if it is next.
    [[nodiscard]] bool expect(char character) {
        if (offset_ >= source_.size() || source_[offset_] != character) return false;
        ++offset_;
        return true;
    }

    /// Tests the next character without consuming it.
    [[nodiscard]] bool next_is(char character) const {
        return offset_ < source_.size() && source_[offset_] == character;
    }

    /// Consumes a keyword when the input continues with it.
    [[nodiscard]] bool literal(std::string_view word) {
        if (source_.substr(offset_, word.size()) != word) return false;
        offset_ += word.size();
        return true;
    }

    /// Parses any value at the current position.
    [[nodiscard]] bool parse_value(std::size_t depth, Value& out) {
        if (depth > 128 || offset_ >= source_.size()) return false;
        switch (source_[offset_]) {
        case '{':
            return parse_object(depth, out);
        case '[':
            return parse_array(depth, out);
        case '"': {
            std::string text;
            if (!parse_string(text)) return false;
            out = Value::text(std::move(text));
            return true;
        }
        case 't':
            if (!literal("true")) return false;
            out = Value::boolean(true);
            return true;
        case 'f':
            if (!literal("false")) return false;
            out = Value::boolean(false);
            return true;
        case 'n':
            if (!literal("null")) return false;
            out = Value{};
            return true;
        default:
            break;
        }
        double number = 0.0;
        if (!parse_number(number)) return false;
        out = Value::number(number);
        return true;
    }

    /// Parses an object.
    [[nodiscard]] bool parse_object(std::size_t depth, Value& out) {
        if (!expect('{')) return false;
        Value result = Value::object();
        skip_space();
        if (next_is('}')) {
            ++offset_;
            out = std::move(result);
            return true;
        }
        while (true) {
            skip_space();
            std::string key;
            if (!parse_string(key)) return false;
            skip_space();
            if (!expect(':')) return false;
            skip_space();
            Value member;
            if (!parse_value(depth + 1, member)) return false;
            result.set(std::move(key), std::move(member));
            skip_space();
            if (next_is(',')) {
                ++offset_;
                continue;
            }
            if (!expect('}')) return false;
            out = std::move(result);
            return true;
        }
    }

    /// Parses an array.
    [[nodiscard]] bool parse_array(std::size_t depth, Value& out) {
        if (!expect('[')) return false;
        Value result = Value::array();
        skip_space();
        if (next_is(']')) {
            ++offset_;
            out = std::move(result);
            return true;
        }
        while (true) {
            skip_space();
            Value item;
            if (!parse_value(depth + 1, item)) return false;
            result.push(std::move(item));
            skip_space();
            if (next_is(',')) {
                ++offset_;
                continue;
            }
            if (!expect(']')) return false;
            out = std::move(result);
            return true;
        }
    }

    /// Parses a quoted string into `result` and decodes its escapes.
    [[nodiscard]] bool parse_string(std::string& result) {
        if (!expect('"')) return false;
        while (true) {
            if (offset_ >= source_.size()) return false;
            const char character = source_[offset_++];
            if (character == '"') return true;
            if (character != '\\') {
                result.push_back(character);
                continue;
            }
            if (offset_ >= source_.size()) return false;
            const char escape = source_[offset_++];
            switch (escape) {
            case '"':
            case '\\':
            case '/':
                result.push_back(escape);
                break;
            case 'b':
                result.push_back('\b');
                break;
            case 'f':
                result.push_back('\f');
                break;
            case 'n':
                result.push_back('\n');
                break;
            case 'r':
                result.push_back('\r');
                break;
            case 't':
                result.push_back('\t');
                break;
            case 'u':
                if (!parse_escape_unit(result)) return false;
                break;
            default:
                return false;
            }
        }
    }

    /// Decodes the four hex digits of a unicode escape to UTF-8 and appends
    /// them. Surrogate pairs are not combined.
    [[nodiscard]] bool parse_escape_unit(std::string& out) {
        if (offset_ + 4 > source_.size()) return false;
        uint32_t code = 0;
        for (std::size_t index = 0; index < 4; ++index) {
            const char digit = source_[offset_++];
            code <<= 4U;
            if (digit >= '0' && digit <= '9') {
                code |= static_cast<uint32_t>(digit - '0');
            } else if (digit >= 'a' && digit <= 'f') {
                code |= static_cast<uint32_t>(digit - 'a') + 10U;
            } else if (digit >= 'A' && digit <= 'F') {
                code |= static_cast<uint32_t>(digit - 'A') + 10U;
            } else {
                return false;
            }
        }
        if (code < 0x80U) {
            out.push_back(static_cast<char>(code));
        } else if (code < 0x800U) {
            out.push_back(static_cast<char>(0xC0U | (code >> 6U)));
            out.push_back(static_cast<char>(0x80U | (code & 0x3FU)));
        } else {
            out.push_back(static_cast<char>(0xE0U | (code >> 12U)));
            out.push_back(static_cast<char>(0x80U | ((code >> 6U) & 0x3FU)));
            out.push_back(static_cast<char>(0x80U | (code & 0x3FU)));
        }
        return true;
    }

    /// Parses a number token with `std::from_chars`, which rounds correctly.
    [[nodiscard]] bool parse_number(double& out) {
        const std::size_t start = offset_;
        if (offset_ < source_.size() && source_[offset_] == '-') ++offset_;
        while (offset_ < source_.size()) {
            const char character = source_[offset_];
            const bool numeric = (character >= '0' && character <= '9') || character == '.' ||
                                 character == 'e' || character == 'E' || character == '+' ||
                                 character == '-';
            if (!numeric) break;
            ++offset_;
        }
        if (offset_ == start) return false;
        const char* first = source_.data() + start;
        const char* last = source_.data() + offset_;
        const std::from_chars_result result = std::from_chars(first, last, out);
        return result.ec == std::errc{} && result.ptr == last;
    }

    std::string_view source_;
    std::size_t offset_{};
};

} // namespace

Value Value::boolean(bool value) {
    Value result{Kind::Bool};
    result.boolean_ = value;
    return result;
}

Value Value::number(double value) {
    Value result{Kind::Number};
    result.number_ = value;
    return result;
}

Value Value::text(std::string value) {
    Value result{Kind::String};
    result.text_ = std::move(value);
    return result;
}

Value Value::array() {
    return Value{Kind::Array};
}

Value Value::object() {
    return Value{Kind::Object};
}

void Value::push(Value value) {
    kind_ = Kind::Array;
    items_.push_back(std::move(value));
}

void Value::set(std::string key, Value value) {
    kind_ = Kind::Object;
    for (std::pair<std::string, Value>& member : members_) {
        if (member.first == key) {
            member.second = std::move(value);
            return;
        }
    }
    members_.emplace_back(std::move(key), std::move(value));
}

const Value* Value::find(std::string_view key) const noexcept {
    for (const std::pair<std::string, Value>& member : members_) {
        if (member.first == key) return &member.second;
    }
    return nullptr;
}

bool Value::bool_or(std::string_view key, bool fallback) const {
    const Value* value = find(key);
    return value != nullptr && value->is(Kind::Bool) ? value->as_bool() : fallback;
}

double Value::number_or(std::string_view key, double fallback) const {
    const Value* value = find(key);
    return value != nullptr && value->is(Kind::Number) ? value->as_number() : fallback;
}

std::string Value::string_or(std::string_view key, std::string fallback) const {
    const Value* value = find(key);
    if (value != nullptr && value->is(Kind::String)) return value->as_string();
    return fallback;
}

std::string_view Value::text_or(std::string_view key, std::string_view fallback) const {
    const Value* value = find(key);
    if (value != nullptr && value->is(Kind::String)) return value->as_string();
    return fallback;
}

bool parse(std::string_view source, Value& out) {
    return Parser{source}.run(out);
}

} // namespace bench::json
