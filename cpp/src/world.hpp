#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <span>
#include <unordered_map>
#include <vector>

#include "box_collision.hpp"
#include "broad_phase.hpp"
#include "constraint_graph.hpp"
#include "contact.hpp"
#include "contact_solver.hpp"
#include "rigid_body.hpp"
#include "task_pool.hpp"

namespace bench {

/// The engine's PhysicsWorld step for bodies of one box each on a static
/// ground: body changes and waking, proxy refits, pair finding into a hash
/// map, box collision with contact recycling, contact begin and end with
/// island linking and graph colouring, the eight-lane solver, and sleep with
/// island splitting.
class World {
public:
    /// A world stepped on `threads` threads, one meaning no pool.
    explicit World(uint32_t threads);

    /// Removes everything and builds the piles again.
    void reset();
    /// Advances one fixed step.
    void step();
    /// Wakes the top body of every eighth pile with a sideways shove.
    void shove();
    /// Checksum of the bodies and the bookkeeping counts.
    uint64_t checksum() const;
    /// Number of awake bodies.
    std::size_t awake_count() const { return awake_bodies_.size(); }

private:
    enum class EdgeSide : uint8_t { A = 0, B = 1 };

    struct BodyRecord {
        uint32_t set{phys::null_link};
        uint32_t local{phys::null_link};
        uint32_t island{phys::null_link};
        uint32_t island_local{phys::null_link};
        std::vector<uint32_t> edges;
        bool alive{};
        bool logged{};
    };

    struct ContactLink {
        uint32_t contact{};
        uint32_t body_a{};
        uint32_t body_b{};
    };

    struct Island {
        uint32_t set{phys::null_link};
        uint32_t local{phys::null_link};
        uint32_t remove_count{};
        std::vector<uint32_t> bodies;
        std::vector<ContactLink> contacts;
        bool alive{};
    };

    struct SleepingSet {
        std::vector<uint32_t> bodies;
        std::vector<uint32_t> contacts;
        std::vector<uint32_t> islands;
        bool alive{};
    };

    struct ProxyMove {
        int32_t proxy{};
        vm::Aabb fat{};
    };

    struct ProxyPair {
        int32_t a{};
        int32_t b{};
        uint64_t key{};
    };

    /// Runs `fn(begin, end)` over [0, count), on the pool when worthwhile.
    template <class Fn> void parallel_for(std::size_t count, std::size_t grain, Fn&& fn) {
        if (pool_ && count >= 2 * grain) {
            pool_->parallel_for(count, grain, fn);
        } else if (count != 0) {
            fn(std::size_t{0}, count);
        }
    }

    /// Adds a body at `position` with half extents `half` and `rotation`.
    void create_body(vm::Vec3 position, vm::Vec3 half, vm::Quat rotation);
    /// Puts a body in the awake set in an island of its own.
    void activate_body(uint32_t slot);
    /// Queues a body for the next step's change processing.
    void notify(uint32_t slot);
    /// Wakes the sets of bodies whose state changed since the last step.
    void process_body_changes();
    /// Moves the proxies of awake bodies whose bounds left their fat bounds.
    void refresh_proxy_bounds();
    /// Creates contacts for the new pairs of the moved proxies.
    void update_pairs();
    /// Collides every awake contact and processes those that began or ended
    /// touching.
    void collide();
    /// Applies the begin and end changes found by collide.
    void process_contact_changes();
    /// Runs the contact solver over the awake bodies.
    void solve();
    /// Updates sleep timers and puts resting islands to sleep.
    void finalize_sleep();

    /// Pose of a shape in world space.
    vm::BoxPose pose_of(phys::ShapeRef ref) const;
    /// Body of a shape.
    const phys::RigidBody& body_of(phys::ShapeRef ref) const;
    /// Edge list of a shape's body.
    std::vector<uint32_t>& edges_of(phys::ShapeRef ref);
    /// Records a contact in one of its bodies' edge lists.
    void add_edge(uint32_t id, EdgeSide side);
    /// Removes a contact from one of its bodies' edge lists.
    void remove_edge(uint32_t id, EdgeSide side);
    /// Swap-removes a contact from a contact list.
    void list_remove(std::vector<uint32_t>& list, uint32_t id);
    /// Sets or clears a contact's bit in the awake bitset.
    void mark_awake(uint32_t id, bool awake);
    /// Adds a non-touching contact to the awake list.
    void awake_add(uint32_t id);
    /// Adds a contact to the list of contacts between sleeping bodies.
    void disabled_add(uint32_t id);
    /// Colours a touching contact into the constraint graph.
    void graph_add(uint32_t id);
    /// Takes a contact out of the constraint graph.
    void graph_remove(uint32_t id);
    /// Creates a contact for a new pair.
    uint32_t create_contact(const ProxyPair& pair);
    /// Destroys a contact and unlinks it from everything.
    void destroy_contact(uint32_t id, bool wake);
    /// Links a touching contact into its bodies' islands, merging them.
    void link_contact(uint32_t id);
    /// Unlinks a contact that stopped touching from its island.
    void unlink_contact(uint32_t id);
    /// A new island in `set`.
    uint32_t create_island(uint32_t set);
    /// Frees an island.
    void destroy_island(uint32_t id);
    /// Merges the smaller island into the larger; returns the survivor.
    uint32_t merge_islands(uint32_t a, uint32_t b);
    /// Splits an island into its connected components.
    void split_island(uint32_t base_id);
    /// Splits the island chosen by the last sleep pass.
    void split_pending_island();
    /// Moves a resting island and its contacts into a new sleeping set.
    void try_sleep_island(uint32_t id);
    /// Moves a sleeping set back to the awake set.
    void wake_set(uint32_t set_id);

    std::unique_ptr<TaskPool> pool_;
    phys::SolverContext context_{};
    phys::CollisionTolerances tolerances_{};

    std::vector<phys::RigidBody> rigid_;
    std::vector<BodyRecord> bodies_;
    std::vector<int32_t> rigid_proxies_;
    phys::RigidBody ground_{};
    int32_t ground_proxy_{-1};
    std::vector<phys::KinematicMotion> static_motions_;

    phys::BroadPhase broad_phase_;
    std::vector<phys::Contact> contacts_;
    std::vector<uint32_t> free_contacts_;
    std::unordered_map<uint64_t, uint32_t> contact_index_;
    std::vector<uint32_t> awake_contacts_;
    std::vector<uint64_t> awake_bits_;
    std::vector<uint32_t> disabled_contacts_;
    phys::ConstraintGraph graph_;
    std::vector<std::vector<uint32_t>> static_edges_;

    std::vector<uint32_t> awake_bodies_;
    std::vector<Island> islands_;
    std::vector<uint32_t> free_islands_;
    std::vector<uint32_t> awake_islands_;
    std::vector<SleepingSet> sleeping_sets_;
    std::vector<uint32_t> free_sets_;
    uint32_t live_islands_{};
    uint32_t sleeping_body_count_{};
    uint32_t sleeping_touching_{};
    uint32_t sleeping_points_{};
    uint32_t split_island_id_{phys::null_link};
    std::vector<uint32_t> change_log_;
    std::vector<uint32_t> pending_changes_;
    std::vector<uint8_t> island_awake_;

    std::vector<uint32_t> collide_ids_;
    std::vector<std::vector<uint32_t>> changed_blocks_;
    std::vector<uint32_t> changed_ids_;
    std::vector<std::vector<ProxyMove>> proxy_moves_;
    std::vector<std::vector<ProxyPair>> pair_candidates_;
    std::vector<phys::BodyDelta> deltas_;
    std::vector<phys::ActiveBody> active_bodies_;
    std::vector<uint32_t> body_local_;
    std::array<std::span<const uint32_t>, phys::graph_color_count> color_lists_{};
    phys::ContactSolver solver_;
};

/// The world step: 64 steps of 864 boxes in 144 piles of six settling on a
/// ground and falling asleep island by island, 36 tumbling boxes dropped onto
/// every fourth pile that wake it on landing, and every eighth pile woken by a
/// shove at step 45. `threads` workers run the engine's parallel
/// sections.
template <uint32_t threads> class WorldCase {
public:
    static constexpr const char* name =
        threads == 1 ? "world" : threads == 2 ? "world2" : threads == 4 ? "world4" : threads == 8 ? "world8" : "world16";

    WorldCase() : world_(threads) {}

    uint64_t run() {
        world_.reset();
        uint64_t h = 0;
        for (int step = 0; step < steps; ++step) {
            if (step == shove_step) world_.shove();
            world_.step();
            h = h * 31u + world_.awake_count();
        }
        return h ^ world_.checksum();
    }

private:
    static constexpr int steps = 64;
    static constexpr int shove_step = 45;

    World world_;
};

} // namespace bench
