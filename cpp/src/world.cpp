#include "world.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <numeric>
#include <utility>

#include "box_collision.hpp"
#include "contact_recycle.hpp"
#include "hash.hpp"

namespace bench {

namespace {

using phys::awake_set;
using phys::Contact;
using phys::disabled_set;
using phys::null_link;
using phys::ShapeRef;

constexpr float sleep_velocity_threshold = 0.05f;
constexpr float sleep_angular_velocity_threshold = 0.05f;
constexpr float time_to_sleep = 0.5f;
constexpr uint32_t piles_side = 12;
constexpr uint32_t pile_height = 6;
constexpr float pile_spacing = 3.0f;

/// World pose of a body's single box, shape transform identity.
vm::BoxPose world_pose(const vm::Transform& body, vm::Vec3 half_extents) {
    const vm::Transform shape{};
    vm::BoxPose pose{};
    pose.half_extents = half_extents;
    pose.center = vm::transform_point(body, shape.origin);
    pose.basis = body.basis * shape.basis;
    return pose;
}

} // namespace

World::World(uint32_t threads) {
    if (threads > 1) pool_ = std::make_unique<TaskPool>(threads);
    phys::SolverContext& c = context_;
    c.dt = 1.0f / 60.0f;
    c.inv_dt = 1.0f / c.dt;
    c.substeps = 4;
    c.h = c.dt / static_cast<float>(c.substeps);
    c.inv_h = 1.0f / c.h;
    c.gravity = {0.0f, -9.81f, 0.0f};
    const float contact_hertz = std::min(30.0f, 0.125f * c.inv_h);
    c.contact_softness = phys::make_softness(contact_hertz, 10.0f, c.h);
    c.static_softness = phys::make_softness(2.0f * contact_hertz, 0.5f * 10.0f, c.h);
    c.push_out_speed = 3.0f;
    c.restitution_threshold = 1.0f;
    c.linear_slop = 0.005f;
    tolerances_ = {4.0f * c.linear_slop, c.linear_slop};
}

void World::reset() {
    rigid_.clear();
    bodies_.clear();
    rigid_proxies_.clear();
    broad_phase_ = phys::BroadPhase{};
    contacts_.clear();
    free_contacts_.clear();
    contact_index_.clear();
    awake_contacts_.clear();
    awake_bits_.clear();
    disabled_contacts_.clear();
    graph_ = phys::ConstraintGraph{};
    static_edges_.assign(1, {});
    awake_bodies_.clear();
    islands_.clear();
    free_islands_.clear();
    awake_islands_.clear();
    sleeping_sets_.assign(1, SleepingSet{});
    free_sets_.clear();
    live_islands_ = 0;
    sleeping_body_count_ = 0;
    sleeping_touching_ = 0;
    sleeping_points_ = 0;
    split_island_id_ = null_link;
    change_log_.clear();

    Rng rng{0x3011d};
    rigid_.reserve(piles_side * piles_side * pile_height + piles_side * piles_side / 4);
    for (uint32_t pz = 0; pz < piles_side; ++pz) {
        for (uint32_t px = 0; px < piles_side; ++px) {
            const float x = static_cast<float>(px) * pile_spacing - 16.5f + rng.unit() * 0.04f - 0.02f;
            const float z = static_cast<float>(pz) * pile_spacing - 16.5f + rng.unit() * 0.04f - 0.02f;
            float top = 0.0f;
            for (uint32_t level = 0; level < pile_height; ++level) {
                const vm::Vec3 half = vm::random_vec3(rng, 0.4f, 0.5f);
                const vm::Quat yaw = vm::normalize(vm::Quat{0.0f, rng.unit() * 0.2f - 0.1f, 0.0f, 1.0f});
                create_body({x, top + half.y + 0.005f, z}, half, yaw);
                top = top + 2.0f * half.y + 0.005f;
            }
        }
    }

    for (uint32_t pile = 0; pile < piles_side * piles_side; pile += 4) {
        const float x = static_cast<float>(pile % piles_side) * pile_spacing - 16.5f;
        const float z = static_cast<float>(pile / piles_side) * pile_spacing - 16.5f;
        const vm::Vec3 half = vm::random_vec3(rng, 0.3f, 0.5f);
        const vm::Vec3 position{x + rng.unit() * 0.4f - 0.2f, 9.5f + rng.unit(), z + rng.unit() * 0.4f - 0.2f};
        create_body(position, half, vm::random_quat(rng));
    }

    ground_ = phys::RigidBody{};
    ground_.transform.origin = {0.0f, -1.0f, 0.0f};
    ground_.half_extents = {24.0f, 1.0f, 24.0f};
    static_motions_.assign(1, phys::KinematicMotion{{}, {}, ground_.transform.origin});
    ground_proxy_ = broad_phase_.create_proxy({0, true}, vm::grow(phys::box_aabb(world_pose(ground_.transform, ground_.half_extents)),
                                                                  tolerances_.speculative_distance));
}

void World::create_body(vm::Vec3 position, vm::Vec3 half, vm::Quat rotation) {
    const auto slot = static_cast<uint32_t>(rigid_.size());
    phys::RigidBody& body = rigid_.emplace_back();
    body.rotation = rotation;
    body.transform = {vm::basis_from_quat(rotation), position};
    phys::set_box_mass(body, half);
    BodyRecord& record = bodies_.emplace_back();
    record.alive = true;
    activate_body(slot);
    const vm::Aabb tight = vm::grow(phys::box_aabb(world_pose(body.transform, half)), tolerances_.speculative_distance);
    rigid_proxies_.push_back(broad_phase_.create_proxy({slot, false}, vm::grow(tight, phys::shape_margin(half))));
}

void World::activate_body(uint32_t slot) {
    BodyRecord& record = bodies_[slot];
    record.set = awake_set;
    record.local = static_cast<uint32_t>(awake_bodies_.size());
    awake_bodies_.push_back(slot);
    const uint32_t island_id = create_island(awake_set);
    Island& island = islands_[island_id];
    record.island = island_id;
    record.island_local = 0;
    island.bodies.push_back(slot);
}

void World::notify(uint32_t slot) {
    BodyRecord& record = bodies_[slot];
    if (record.logged) return;
    record.logged = true;
    change_log_.push_back(slot);
}

void World::shove() {
    for (uint32_t pile = 0; pile < piles_side * piles_side; pile += 8) {
        const uint32_t slot = pile * pile_height + pile_height - 1;
        phys::RigidBody& body = rigid_[slot];
        body.linear_velocity = {2.0f, 0.0f, 1.0f};
        body.sleeping = false;
        body.sleep_time = 0.0f;
        notify(slot);
    }
}

void World::process_body_changes() {
    if (change_log_.empty()) return;
    pending_changes_.assign(change_log_.begin(), change_log_.end());
    change_log_.clear();
    std::ranges::sort(pending_changes_);
    for (const uint32_t slot : pending_changes_) {
        BodyRecord& record = bodies_[slot];
        record.logged = false;
        if (!rigid_[slot].sleeping && record.set != awake_set) wake_set(record.set);
    }
}

vm::BoxPose World::pose_of(ShapeRef ref) const {
    const phys::RigidBody& body = body_of(ref);
    return world_pose(body.transform, body.half_extents);
}

const phys::RigidBody& World::body_of(ShapeRef ref) const { return ref.is_static ? ground_ : rigid_[ref.body]; }

std::vector<uint32_t>& World::edges_of(ShapeRef ref) {
    if (ref.is_static) return static_edges_[ref.body];
    return bodies_[ref.body].edges;
}

void World::add_edge(uint32_t id, EdgeSide side) {
    Contact& contact = contacts_[id];
    const auto index = static_cast<uint32_t>(side);
    std::vector<uint32_t>& list = edges_of(index == 0 ? contact.shape_a : contact.shape_b);
    contact.edge_local[index] = static_cast<uint32_t>(list.size());
    list.push_back((id << 1u) | index);
}

void World::remove_edge(uint32_t id, EdgeSide side) {
    Contact& contact = contacts_[id];
    const auto index = static_cast<uint32_t>(side);
    std::vector<uint32_t>& list = edges_of(index == 0 ? contact.shape_a : contact.shape_b);
    const uint32_t local = contact.edge_local[index];
    const uint32_t last = list.back();
    list[local] = last;
    list.pop_back();
    if (local < list.size()) contacts_[last >> 1u].edge_local[last & 1u] = local;
    contact.edge_local[index] = null_link;
}

void World::list_remove(std::vector<uint32_t>& list, uint32_t id) {
    const uint32_t local = contacts_[id].local;
    const uint32_t last = list.back();
    list[local] = last;
    list.pop_back();
    if (local < list.size()) contacts_[last].local = local;
    contacts_[id].local = null_link;
}

void World::mark_awake(uint32_t id, bool awake) {
    const std::size_t word = id / 64u;
    if (awake_bits_.size() <= word) awake_bits_.resize(word + 1, 0u);
    const uint64_t bit = uint64_t{1} << (id % 64u);
    if (awake) {
        awake_bits_[word] |= bit;
    } else {
        awake_bits_[word] &= ~bit;
    }
}

void World::awake_add(uint32_t id) {
    mark_awake(id, true);
    Contact& contact = contacts_[id];
    contact.set = awake_set;
    contact.color = null_link;
    contact.local = static_cast<uint32_t>(awake_contacts_.size());
    awake_contacts_.push_back(id);
}

void World::disabled_add(uint32_t id) {
    mark_awake(id, false);
    Contact& contact = contacts_[id];
    contact.set = disabled_set;
    contact.color = null_link;
    contact.local = static_cast<uint32_t>(disabled_contacts_.size());
    disabled_contacts_.push_back(id);
}

void World::graph_add(uint32_t id) {
    Contact& contact = contacts_[id];
    graph_.reserve_bodies(bodies_.size());
    const bool b_static = contact.shape_b.is_static;
    const phys::GraphSlot slot = graph_.add(id, {contact.shape_a.body, b_static ? 0u : contact.shape_b.body, b_static});
    contact.color = slot.color;
    contact.local = slot.local;
    contact.set = awake_set;
    mark_awake(id, true);
}

void World::graph_remove(uint32_t id) {
    Contact& contact = contacts_[id];
    const bool b_static = contact.shape_b.is_static;
    const uint32_t moved =
        graph_.remove({contact.color, contact.local}, {contact.shape_a.body, b_static ? 0u : contact.shape_b.body, b_static});
    if (moved != null_link) contacts_[moved].local = contact.local;
    contact.color = null_link;
    contact.local = null_link;
}

uint32_t World::create_contact(const ProxyPair& pair) {
    uint32_t id{};
    if (free_contacts_.empty()) {
        id = static_cast<uint32_t>(contacts_.size());
        contacts_.emplace_back();
    } else {
        id = free_contacts_.back();
        free_contacts_.pop_back();
    }
    Contact& contact = contacts_[id];
    contact = Contact{};
    contact.alive = true;
    contact.shape_a = broad_phase_.shape(pair.a);
    contact.shape_b = broad_phase_.shape(pair.b);
    contact.proxy_a = pair.a;
    contact.proxy_b = pair.b;
    contact_index_.emplace(pair.key, id);
    add_edge(id, EdgeSide::A);
    add_edge(id, EdgeSide::B);

    const bool a_awake = bodies_[contact.shape_a.body].set == awake_set;
    const bool b_awake = !contact.shape_b.is_static && bodies_[contact.shape_b.body].set == awake_set;
    if (a_awake || b_awake) {
        awake_add(id);
    } else {
        disabled_add(id);
    }
    return id;
}

void World::destroy_contact(uint32_t id, bool wake) {
    Contact& contact = contacts_[id];
    contact_index_.erase(phys::pair_key(contact.shape_a, contact.shape_b));
    const uint32_t slot_a = contact.shape_a.body;
    const uint32_t slot_b = contact.shape_b.is_static ? null_link : contact.shape_b.body;

    if (wake && contact.linked) {
        if (bodies_[slot_a].set != awake_set && bodies_[slot_a].set != null_link) wake_set(bodies_[slot_a].set);
        if (slot_b != null_link && bodies_[slot_b].set != awake_set && bodies_[slot_b].set != null_link) {
            wake_set(bodies_[slot_b].set);
        }
    }

    remove_edge(id, EdgeSide::A);
    remove_edge(id, EdgeSide::B);

    if (contact.island != null_link) unlink_contact(id);
    if (contact.color != null_link) {
        graph_remove(id);
    } else if (contact.set == awake_set) {
        list_remove(awake_contacts_, id);
    } else if (contact.set == disabled_set) {
        list_remove(disabled_contacts_, id);
    } else if (contact.set != null_link) {
        SleepingSet& set = sleeping_sets_[contact.set];
        list_remove(set.contacts, id);
        if (contact.touching) {
            --sleeping_touching_;
            sleeping_points_ -= contact.manifold.point_count;
        }
    }
    mark_awake(id, false);
    contact.alive = false;
    contact.set = null_link;
    free_contacts_.push_back(id);
}

void World::link_contact(uint32_t id) {
    Contact& contact = contacts_[id];
    const uint32_t slot_a = contact.shape_a.body;
    const bool b_static = contact.shape_b.is_static;
    const uint32_t slot_b = b_static ? null_link : contact.shape_b.body;

    if (slot_b != null_link) {
        const uint32_t set_a = bodies_[slot_a].set;
        const uint32_t set_b = bodies_[slot_b].set;
        if (set_a == awake_set && set_b != awake_set && set_b != null_link) {
            wake_set(set_b);
        } else if (set_b == awake_set && set_a != awake_set && set_a != null_link) {
            wake_set(set_a);
        }
    }

    const uint32_t island_a = bodies_[slot_a].island;
    const uint32_t island_b = slot_b == null_link ? null_link : bodies_[slot_b].island;
    const uint32_t merged = merge_islands(island_a, island_b);

    Island& island = islands_[merged];
    contact.island = merged;
    contact.island_local = static_cast<uint32_t>(island.contacts.size());
    island.contacts.push_back({id, slot_a, slot_b});
    contact.linked = true;
}

void World::unlink_contact(uint32_t id) {
    Contact& contact = contacts_[id];
    Island& island = islands_[contact.island];
    const uint32_t local = contact.island_local;
    const ContactLink last = island.contacts.back();
    island.contacts[local] = last;
    island.contacts.pop_back();
    if (local < island.contacts.size()) contacts_[last.contact].island_local = local;
    ++island.remove_count;
    contact.island = null_link;
    contact.island_local = null_link;
    contact.linked = false;
}

uint32_t World::create_island(uint32_t set) {
    uint32_t id{};
    if (free_islands_.empty()) {
        id = static_cast<uint32_t>(islands_.size());
        islands_.emplace_back();
    } else {
        id = free_islands_.back();
        free_islands_.pop_back();
    }
    Island& island = islands_[id];
    island = Island{};
    island.alive = true;
    island.set = set;
    if (set == awake_set) {
        island.local = static_cast<uint32_t>(awake_islands_.size());
        awake_islands_.push_back(id);
    }
    ++live_islands_;
    return id;
}

void World::destroy_island(uint32_t id) {
    if (split_island_id_ == id) split_island_id_ = null_link;
    Island& island = islands_[id];
    std::vector<uint32_t>& list = island.set == awake_set ? awake_islands_ : sleeping_sets_[island.set].islands;
    const uint32_t last = list.back();
    list[island.local] = last;
    list.pop_back();
    if (island.local < list.size()) islands_[last].local = island.local;
    island = Island{};
    free_islands_.push_back(id);
    --live_islands_;
}

uint32_t World::merge_islands(uint32_t a, uint32_t b) {
    if (a == b) return a;
    if (a == null_link) return b;
    if (b == null_link) return a;
    uint32_t big = a;
    uint32_t small = b;
    if (islands_[a].bodies.size() < islands_[b].bodies.size()) std::swap(big, small);
    Island& big_island = islands_[big];
    Island& small_island = islands_[small];
    for (const uint32_t slot : small_island.bodies) {
        BodyRecord& record = bodies_[slot];
        record.island = big;
        record.island_local = static_cast<uint32_t>(big_island.bodies.size());
        big_island.bodies.push_back(slot);
    }
    for (const ContactLink& link : small_island.contacts) {
        Contact& contact = contacts_[link.contact];
        contact.island = big;
        contact.island_local = static_cast<uint32_t>(big_island.contacts.size());
        big_island.contacts.push_back(link);
    }
    big_island.remove_count += small_island.remove_count;
    destroy_island(small);
    return big;
}

void World::split_island(uint32_t base_id) {
    std::vector<uint32_t> base_bodies = std::move(islands_[base_id].bodies);
    std::vector<ContactLink> base_contacts = std::move(islands_[base_id].contacts);
    islands_[base_id].bodies.clear();
    islands_[base_id].contacts.clear();
    const auto count = static_cast<uint32_t>(base_bodies.size());

    std::vector<uint32_t> parents(count);
    std::vector<uint32_t> ranks(count, 0u);
    std::iota(parents.begin(), parents.end(), 0u);
    const auto find = [&](uint32_t node) {
        while (parents[node] != node) {
            parents[node] = parents[parents[node]];
            node = parents[node];
        }
        return node;
    };
    for (const ContactLink& link : base_contacts) {
        if (link.body_b == null_link) continue;
        uint32_t root_a = find(bodies_[link.body_a].island_local);
        uint32_t root_b = find(bodies_[link.body_b].island_local);
        if (root_a == root_b) continue;
        if (ranks[root_a] < ranks[root_b]) std::swap(root_a, root_b);
        parents[root_b] = root_a;
        if (ranks[root_a] == ranks[root_b]) ++ranks[root_a];
    }

    uint32_t components = 0;
    for (uint32_t i = 0; i < count; ++i) {
        parents[i] = find(i);
        components += parents[i] == i ? 1u : 0u;
    }
    if (components == 1) {
        Island& island = islands_[base_id];
        island.bodies = std::move(base_bodies);
        island.contacts = std::move(base_contacts);
        island.remove_count = 0;
        return;
    }

    std::vector<uint32_t> root_island(count, null_link);
    std::vector<uint32_t> island_ids;
    island_ids.reserve(components);
    for (uint32_t i = 0; i < count; ++i) {
        if (parents[i] == i) {
            root_island[i] = static_cast<uint32_t>(island_ids.size());
            island_ids.push_back(create_island(awake_set));
        }
    }
    for (uint32_t i = 0; i < count; ++i) {
        const uint32_t slot = base_bodies[i];
        const uint32_t target = island_ids[root_island[parents[i]]];
        Island& island = islands_[target];
        BodyRecord& record = bodies_[slot];
        record.island = target;
        record.island_local = static_cast<uint32_t>(island.bodies.size());
        island.bodies.push_back(slot);
    }
    for (const ContactLink& link : base_contacts) {
        const uint32_t target = bodies_[link.body_a].island;
        Island& island = islands_[target];
        Contact& contact = contacts_[link.contact];
        contact.island = target;
        contact.island_local = static_cast<uint32_t>(island.contacts.size());
        island.contacts.push_back(link);
    }
    destroy_island(base_id);
}

void World::split_pending_island() {
    const uint32_t id = split_island_id_;
    split_island_id_ = null_link;
    if (id == null_link || id >= islands_.size() || !islands_[id].alive || islands_[id].set != awake_set ||
        islands_[id].remove_count == 0) {
        return;
    }
    split_island(id);
}

void World::try_sleep_island(uint32_t id) {
    if (islands_[id].remove_count > 0 && islands_[id].bodies.size() > 1) return;
    uint32_t set_id{};
    if (free_sets_.empty()) {
        set_id = static_cast<uint32_t>(sleeping_sets_.size());
        sleeping_sets_.emplace_back();
    } else {
        set_id = free_sets_.back();
        free_sets_.pop_back();
    }
    SleepingSet& set = sleeping_sets_[set_id];
    set = SleepingSet{};
    set.alive = true;
    Island& island = islands_[id];

    for (const uint32_t slot : island.bodies) {
        BodyRecord& record = bodies_[slot];
        const uint32_t last = awake_bodies_.back();
        awake_bodies_[record.local] = last;
        awake_bodies_.pop_back();
        if (record.local < awake_bodies_.size()) bodies_[last].local = record.local;
        phys::RigidBody& body = rigid_[slot];
        body.sleeping = true;
        body.linear_velocity = {};
        body.angular_velocity = {};
        body.sleep_velocity = 0.0f;
        record.set = set_id;
        record.local = static_cast<uint32_t>(set.bodies.size());
        set.bodies.push_back(slot);

        for (const uint32_t key : record.edges) {
            const uint32_t contact_id = key >> 1u;
            const Contact& contact = contacts_[contact_id];
            if (contact.color != null_link) continue;
            const uint32_t side = key & 1u;
            uint32_t other = null_link;
            if (side == 1u) {
                other = bodies_[contact.shape_a.body].set;
            } else if (!contact.shape_b.is_static) {
                other = bodies_[contact.shape_b.body].set;
            }
            if (other == awake_set) continue;
            list_remove(awake_contacts_, contact_id);
            disabled_add(contact_id);
        }
    }

    for (const ContactLink& link : island.contacts) {
        Contact& contact = contacts_[link.contact];
        graph_remove(link.contact);
        mark_awake(link.contact, false);
        contact.set = set_id;
        contact.local = static_cast<uint32_t>(set.contacts.size());
        set.contacts.push_back(link.contact);
        ++sleeping_touching_;
        sleeping_points_ += contact.manifold.point_count;
    }

    {
        Island& moved = islands_[id];
        const uint32_t last = awake_islands_.back();
        awake_islands_[moved.local] = last;
        awake_islands_.pop_back();
        if (moved.local < awake_islands_.size()) islands_[last].local = moved.local;
        moved.set = set_id;
        moved.local = static_cast<uint32_t>(set.islands.size());
        set.islands.push_back(id);
    }
    sleeping_body_count_ += static_cast<uint32_t>(set.bodies.size());
    if (split_island_id_ == id) split_island_id_ = null_link;
}

void World::wake_set(uint32_t set_id) {
    SleepingSet& set = sleeping_sets_[set_id];
    for (const uint32_t slot : set.bodies) {
        BodyRecord& record = bodies_[slot];
        record.set = awake_set;
        record.local = static_cast<uint32_t>(awake_bodies_.size());
        awake_bodies_.push_back(slot);
        phys::RigidBody& body = rigid_[slot];
        body.sleeping = false;
        body.sleep_time = 0.0f;
        for (const uint32_t key : record.edges) {
            const uint32_t contact_id = key >> 1u;
            if (contacts_[contact_id].set == disabled_set) {
                list_remove(disabled_contacts_, contact_id);
                awake_add(contact_id);
            }
        }
    }
    for (const uint32_t contact_id : set.contacts) {
        Contact& contact = contacts_[contact_id];
        --sleeping_touching_;
        sleeping_points_ -= contact.manifold.point_count;
        graph_add(contact_id);
    }
    for (const uint32_t island_id : set.islands) {
        Island& island = islands_[island_id];
        island.set = awake_set;
        island.local = static_cast<uint32_t>(awake_islands_.size());
        awake_islands_.push_back(island_id);
    }
    sleeping_body_count_ -= static_cast<uint32_t>(set.bodies.size());
    set = SleepingSet{};
    free_sets_.push_back(set_id);
}

void World::refresh_proxy_bounds() {
    const float speculative = tolerances_.speculative_distance;
    const std::size_t count = awake_bodies_.size();
    if (count == 0) return;
    constexpr std::size_t grain = 256;
    const std::size_t blocks = (count + grain - 1) / grain;
    if (proxy_moves_.size() < blocks) proxy_moves_.resize(blocks);
    for (std::size_t block = 0; block < blocks; ++block) proxy_moves_[block].clear();
    parallel_for(count, grain, [&](std::size_t begin, std::size_t end) {
        std::vector<ProxyMove>& moves = proxy_moves_[begin / grain];
        for (std::size_t index = begin; index < end; ++index) {
            const uint32_t slot = awake_bodies_[index];
            const phys::RigidBody& body = rigid_[slot];
            const vm::Aabb tight = vm::grow(phys::box_aabb(world_pose(body.transform, body.half_extents)), speculative);
            const int32_t proxy = rigid_proxies_[slot];
            if (!vm::contains(broad_phase_.fat_aabb(proxy), tight)) moves.push_back({proxy, vm::grow(tight, phys::shape_margin(body.half_extents))});
        }
    });
    for (std::size_t block = 0; block < blocks; ++block) {
        for (const ProxyMove& move : proxy_moves_[block]) broad_phase_.move_proxy(move.proxy, move.fat);
    }
}

void World::update_pairs() {
    constexpr std::size_t grain = 64;
    const std::size_t moved = broad_phase_.moved_count();
    const std::size_t blocks = (moved + grain - 1) / grain;
    if (pair_candidates_.size() < blocks) pair_candidates_.resize(blocks);
    for (std::size_t block = 0; block < blocks; ++block) pair_candidates_[block].clear();
    parallel_for(moved, grain, [&](std::size_t begin, std::size_t end) {
        std::vector<ProxyPair>& found = pair_candidates_[begin / grain];
        broad_phase_.query_moved(begin, end, [&](int32_t a, int32_t b) {
            const uint64_t key = phys::pair_key(broad_phase_.shape(a), broad_phase_.shape(b));
            if (!contact_index_.contains(key)) found.push_back({a, b, key});
        });
    });
    for (std::size_t block = 0; block < blocks; ++block) {
        for (const ProxyPair& pair : pair_candidates_[block]) {
            if (!contact_index_.contains(pair.key)) static_cast<void>(create_contact(pair));
        }
    }
    broad_phase_.clear_moves();
}

void World::collide() {
    collide_ids_.clear();
    for (std::size_t word = 0; word < awake_bits_.size(); ++word) {
        uint64_t bits = awake_bits_[word];
        while (bits != 0u) {
            collide_ids_.push_back(static_cast<uint32_t>(word * 64u + static_cast<unsigned>(std::countr_zero(bits))));
            bits &= bits - 1u;
        }
    }
    constexpr std::size_t grain = 128;
    const std::size_t blocks = (collide_ids_.size() + grain - 1) / grain;
    if (changed_blocks_.size() < blocks) changed_blocks_.resize(blocks);
    for (std::size_t block = 0; block < blocks; ++block) changed_blocks_[block].clear();
    parallel_for(collide_ids_.size(), grain, [&](std::size_t begin, std::size_t end) {
        std::vector<uint32_t>& changed = changed_blocks_[begin / grain];
        for (std::size_t i = begin; i < end; ++i) {
            const uint32_t id = collide_ids_[i];
            Contact& contact = contacts_[id];
            if (!vm::overlaps(broad_phase_.fat_aabb(contact.proxy_a), broad_phase_.fat_aabb(contact.proxy_b))) {
                contact.touching = false;
                changed.push_back((id << 1u) | 1u);
                continue;
            }
            const phys::ContactPoses poses{pose_of(contact.shape_a), pose_of(contact.shape_b)};
            if (!phys::try_recycle_contact(contact, poses, tolerances_)) {
                phys::collide_boxes(poses.a, poses.b, tolerances_, contact.manifold);
                phys::cache_contact(contact, poses);
            }
            contact.touching = contact.manifold.point_count > 0;
            const phys::RigidBody& a = body_of(contact.shape_a);
            const phys::RigidBody& b = body_of(contact.shape_b);
            contact.friction = std::sqrt(a.friction * b.friction);
            contact.restitution = std::max(a.restitution, b.restitution);
            contact.rolling_resistance = std::max(a.rolling_resistance, b.rolling_resistance);
            if (contact.touching != contact.linked) changed.push_back(id << 1u);
        }
    });

    changed_ids_.clear();
    for (std::size_t block = 0; block < blocks; ++block) {
        changed_ids_.insert(changed_ids_.end(), changed_blocks_[block].begin(), changed_blocks_[block].end());
    }
    std::ranges::sort(changed_ids_);
    process_contact_changes();
}

void World::process_contact_changes() {
    for (const uint32_t encoded : changed_ids_) {
        const uint32_t id = encoded >> 1u;
        Contact& contact = contacts_[id];
        if (!contact.alive) continue;
        if ((encoded & 1u) != 0u) {
            destroy_contact(id, false);
        } else if (contact.touching && !contact.linked) {
            link_contact(id);
            list_remove(awake_contacts_, id);
            graph_add(id);
        } else if (!contact.touching && contact.linked) {
            contact.was_touching = false;
            unlink_contact(id);
            graph_remove(id);
            awake_add(id);
        }
    }
}

void World::solve() {
    active_bodies_.clear();
    body_local_.resize(bodies_.size());
    for (uint32_t index = 0; index < awake_bodies_.size(); ++index) {
        const uint32_t slot = awake_bodies_[index];
        active_bodies_.push_back({&rigid_[slot], slot});
        body_local_[slot] = index;
    }
    deltas_.assign(awake_bodies_.size(), phys::BodyDelta{});
    for (uint32_t color = 0; color < phys::graph_color_count; ++color) color_lists_[color] = graph_.contacts(color);
    phys::SolverInputs inputs{};
    inputs.contacts = contacts_;
    inputs.colors = color_lists_;
    inputs.body_local = body_local_;
    inputs.active_bodies = active_bodies_;
    inputs.static_motions = static_motions_;
    inputs.deltas = deltas_;
    inputs.context = context_;
    inputs.pool = pool_.get();
    solver_.solve(inputs);
}

void World::finalize_sleep() {
    if (island_awake_.size() < islands_.size()) island_awake_.resize(islands_.size(), 0u);
    for (const uint32_t id : awake_islands_) island_awake_[id] = 0u;
    uint32_t candidate = null_link;
    float candidate_time = 0.0f;
    for (uint32_t index = 0; index < awake_bodies_.size(); ++index) {
        const uint32_t slot = awake_bodies_[index];
        const BodyRecord& record = bodies_[slot];
        phys::RigidBody& body = rigid_[slot];
        const float reach = vm::length(body.max_extent);
        const float velocity = vm::length(body.linear_velocity) + vm::length(body.angular_velocity) * reach;
        const phys::BodyDelta& delta = deltas_[index];
        const vm::Vec3 axis{delta.rotation.x, delta.rotation.y, delta.rotation.z};
        const float angle = 2.0f * vm::length(axis);
        const float moved = vm::length(delta.position) + 2.0f * angle * reach;
        const float sleep_velocity = std::max(velocity, 0.5f * context_.inv_dt * moved);
        const float angular_sleep_velocity = std::max(vm::length(body.angular_velocity), angle * context_.inv_dt);
        body.sleep_velocity = sleep_velocity;
        if (!body.can_sleep || sleep_velocity > sleep_velocity_threshold || angular_sleep_velocity > sleep_angular_velocity_threshold) {
            body.sleep_time = 0.0f;
        } else {
            body.sleep_time += context_.dt;
        }
        const Island& island = islands_[record.island];
        body.island = island.bodies.front();
        if (body.sleep_time <= time_to_sleep) {
            island_awake_[record.island] = 1u;
        } else if (island.remove_count > 0 &&
                   (body.sleep_time > candidate_time || (body.sleep_time == candidate_time && record.island > candidate))) {
            candidate = record.island;
            candidate_time = body.sleep_time;
        }
    }
    split_island_id_ = candidate;
    for (std::size_t i = awake_islands_.size(); i > 0; --i) {
        const uint32_t id = awake_islands_[i - 1];
        if (island_awake_[id] == 0u) try_sleep_island(id);
    }
}

void World::step() {
    process_body_changes();
    refresh_proxy_bounds();
    update_pairs();
    collide();
    split_pending_island();
    solve();
    finalize_sleep();
    for (const phys::ActiveBody& active : active_bodies_) {
        active.body->applied_force = {};
        active.body->applied_torque = {};
    }
}

uint64_t World::checksum() const {
    uint64_t h = 0;
    for (const phys::RigidBody& body : rigid_) {
        const vm::Vec3 p = body.transform.origin;
        h = hash_add(h, static_cast<uint64_t>(f32_bits(p.x)) | (static_cast<uint64_t>(f32_bits(p.y)) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(p.z)) | (static_cast<uint64_t>(f32_bits(body.rotation.w)) << 32));
        h = hash_add(h, static_cast<uint64_t>(f32_bits(body.linear_velocity.y)) | (static_cast<uint64_t>(body.sleeping) << 32));
    }
    h = hash_add(h, live_islands_);
    h = hash_add(h, sleeping_body_count_);
    h = hash_add(h, sleeping_touching_);
    h = hash_add(h, sleeping_points_);
    h = hash_add(h, contact_index_.size());
    h = hash_add(h, graph_.size());
    return h;
}

} // namespace bench
