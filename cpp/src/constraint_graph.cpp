#include "constraint_graph.hpp"

#include "contact.hpp"

namespace bench::phys {

void ConstraintGraph::reserve_bodies(std::size_t body_count) {
    const std::size_t words = (body_count + 63u) / 64u;
    if (words <= words_) return;
    words_ = words;
    for (uint32_t color = 0; color < overflow_color; ++color) colors_[color].bodies.resize(words_, 0u);
}

bool ConstraintGraph::is_free(const Color& color, uint32_t body) const {
    return (color.bodies[body / 64u] & (uint64_t{1} << (body % 64u))) == 0u;
}

void ConstraintGraph::take(Color& color, uint32_t body) { color.bodies[body / 64u] |= uint64_t{1} << (body % 64u); }

void ConstraintGraph::release(Color& color, uint32_t body) { color.bodies[body / 64u] &= ~(uint64_t{1} << (body % 64u)); }

GraphSlot ConstraintGraph::add(uint32_t contact, const GraphBodies& bodies) {
    uint32_t chosen = overflow_color;
    if (bodies.b_is_static) {
        for (uint32_t color = overflow_color - 1; color >= 1; --color) {
            if (is_free(colors_[color], bodies.a)) {
                take(colors_[color], bodies.a);
                chosen = color;
                break;
            }
        }
    } else {
        for (uint32_t color = 0; color < dynamic_color_count; ++color) {
            if (is_free(colors_[color], bodies.a) && is_free(colors_[color], bodies.b)) {
                take(colors_[color], bodies.a);
                take(colors_[color], bodies.b);
                chosen = color;
                break;
            }
        }
    }
    Color& target = colors_[chosen];
    const auto local = static_cast<uint32_t>(target.contacts.size());
    target.contacts.push_back(contact);
    ++size_;
    return {chosen, local};
}

uint32_t ConstraintGraph::remove(const GraphSlot& slot, const GraphBodies& bodies) {
    Color& target = colors_[slot.color];
    if (slot.color != overflow_color) {
        release(target, bodies.a);
        if (!bodies.b_is_static) release(target, bodies.b);
    }
    const uint32_t last = target.contacts.back();
    uint32_t moved = null_link;
    if (slot.local + 1u != target.contacts.size()) {
        target.contacts[slot.local] = last;
        moved = last;
    }
    target.contacts.pop_back();
    --size_;
    return moved;
}

} // namespace bench::phys
