#include "broad_phase.hpp"

namespace bench::phys {

BroadPhase::ProxyId BroadPhase::create_proxy(ShapeRef ref, const vm::Aabb& fat_aabb,
                                             bool kinematic) {
    ProxyId id{};
    if (free_.empty()) {
        proxies_.emplace_back();
        id = static_cast<ProxyId>(proxies_.size() - 1);
    } else {
        id = free_.back();
        free_.pop_back();
    }
    Proxy& proxy = proxies_[static_cast<std::size_t>(id)];
    proxy = Proxy{};
    proxy.ref = ref;
    proxy.fat_aabb = fat_aabb;
    proxy.is_static = ref.is_static;
    proxy.kinematic = kinematic && proxy.is_static;
    proxy.alive = true;
    AabbTree& tree = tree_of(proxy);
    proxy.node = tree.create_proxy(fat_aabb, static_cast<std::uint32_t>(id));
    buffer_move(id);
    return id;
}

void BroadPhase::destroy_proxy(ProxyId id) {
    Proxy& proxy = proxies_[static_cast<std::size_t>(id)];
    AabbTree& tree = tree_of(proxy);
    tree.destroy_proxy(proxy.node);
    proxy = Proxy{};
    free_.push_back(id);
}

void BroadPhase::move_proxy(ProxyId id, const vm::Aabb& fat_aabb) {
    Proxy& proxy = proxies_[static_cast<std::size_t>(id)];
    proxy.fat_aabb = fat_aabb;
    AabbTree& tree = tree_of(proxy);
    tree.move_proxy(proxy.node, fat_aabb);
    buffer_move(id);
}

AabbTree& BroadPhase::tree_of(const Proxy& proxy) noexcept {
    if (!proxy.is_static) {
        return dynamic_tree_;
    }
    return proxy.kinematic ? kinematic_tree_ : static_tree_;
}

void BroadPhase::buffer_move(ProxyId id) {
    Proxy& proxy = proxies_[static_cast<std::size_t>(id)];
    if (proxy.moved) {
        return;
    }
    proxy.moved = true;
    move_buffer_.push_back(id);
}

} // namespace bench::phys
