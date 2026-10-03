#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <span>
#include <vector>

/// The engine's constraint graph: colours contacts so that no two in a colour
/// share a dynamic body, keeping static contacts out of colour 0 and sending
/// what does not fit to the overflow colour.
namespace bench::phys {

inline constexpr uint32_t graph_color_count = 24;
inline constexpr uint32_t overflow_color = graph_color_count - 1;
inline constexpr uint32_t dynamic_color_count = graph_color_count - 4;

struct GraphBodies {
    uint32_t a{};
    uint32_t b{};
    bool b_is_static{};
};

struct GraphSlot {
    uint32_t color{};
    uint32_t local{};
};

class ConstraintGraph final {
public:
    /// Grows every colour's body bitset to cover `body_count` bodies.
    void reserve_bodies(std::size_t body_count);
    /// Adds a contact to the first colour free for its bodies.
    GraphSlot add(uint32_t contact, const GraphBodies& bodies);
    /// Removes a contact; returns the contact moved into its place, or
    /// null_link.
    uint32_t remove(const GraphSlot& slot, const GraphBodies& bodies);
    /// Contacts of one colour.
    std::span<const uint32_t> contacts(uint32_t color) const { return colors_[color].contacts; }
    /// Number of contacts in the graph.
    std::size_t size() const { return size_; }

private:
    struct Color {
        std::vector<uint64_t> bodies;
        std::vector<uint32_t> contacts;
    };

    /// Whether `body` is not yet in `color`.
    bool is_free(const Color& color, uint32_t body) const;
    /// Marks `body` as in `color`.
    static void take(Color& color, uint32_t body);
    /// Clears `body` from `color`.
    static void release(Color& color, uint32_t body);

    std::array<Color, graph_color_count> colors_;
    std::size_t words_{};
    std::size_t size_{};
};

} // namespace bench::phys
