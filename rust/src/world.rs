//! The engine's PhysicsWorld step for bodies of one box each on a static
//! ground: body changes and waking, proxy refits, pair finding into a hash
//! map, box collision with contact recycling, contact begin and end with
//! island linking and graph colouring, the eight-lane solver, and sleep with
//! island splitting.
use crate::box_collision::{CollisionTolerances, box_aabb, collide_boxes};
use crate::broad_phase::{BroadPhase, ProxyId};
use crate::broadphase::shape_margin;
use crate::constraint_graph::{ConstraintGraph, GRAPH_COLOR_COUNT, GraphBodies, GraphSlot};
use crate::contact::{AWAKE_SET, Contact, DISABLED_SET, NULL_LINK, ShapeRef, pair_key};
use crate::contact_recycle::{ContactPoses, cache_contact, try_recycle_contact};
use crate::contact_solver::{BodyDelta, ContactSolver, KinematicMotion, SolverContext, SolverInputs};
use crate::harness::Case;
use crate::hash;
use crate::rigid_body::RigidBody;
use crate::task_pool::{Disjoint, TaskPool};
use crate::vecmath::{self, Aabb, Basis, BoxPose, Quat, Transform, Vec3};
use rustc_hash::FxHashMap;

const SLEEP_VELOCITY_THRESHOLD: f32 = 0.05;
const SLEEP_ANGULAR_VELOCITY_THRESHOLD: f32 = 0.05;
const TIME_TO_SLEEP: f32 = 0.5;
const PILES_SIDE: u32 = 12;
const PILE_HEIGHT: u32 = 6;
const PILE_SPACING: f32 = 3.0;
const STEPS: usize = 64;
const SHOVE_STEP: usize = 45;

/// Which side of a contact an edge list entry stands for.
#[derive(Clone, Copy)]
enum EdgeSide {
    A = 0,
    B = 1,
}

#[derive(Clone)]
struct BodyRecord {
    set: u32,
    local: u32,
    island: u32,
    island_local: u32,
    edges: Vec<u32>,
    logged: bool,
}

impl Default for BodyRecord {
    /// A record outside every set and island.
    fn default() -> BodyRecord {
        BodyRecord { set: NULL_LINK, local: NULL_LINK, island: NULL_LINK, island_local: NULL_LINK, edges: Vec::new(), logged: false }
    }
}

#[derive(Clone, Copy)]
struct ContactLink {
    contact: u32,
    body_a: u32,
    body_b: u32,
}

#[derive(Clone)]
struct Island {
    set: u32,
    local: u32,
    remove_count: u32,
    bodies: Vec<u32>,
    contacts: Vec<ContactLink>,
    alive: bool,
}

impl Default for Island {
    /// A dead island outside every set.
    fn default() -> Island {
        Island { set: NULL_LINK, local: NULL_LINK, remove_count: 0, bodies: Vec::new(), contacts: Vec::new(), alive: false }
    }
}

#[derive(Clone, Default)]
struct SleepingSet {
    bodies: Vec<u32>,
    contacts: Vec<u32>,
    islands: Vec<u32>,
}

#[derive(Clone, Copy)]
struct ProxyMove {
    proxy: ProxyId,
    fat: Aabb,
}

#[derive(Clone, Copy)]
struct ProxyPair {
    a: ProxyId,
    b: ProxyId,
    key: u64,
}

/// World pose of a body's single box, shape transform identity.
fn world_pose(body: &Transform, half_extents: Vec3) -> BoxPose {
    let shape = Transform::default();
    BoxPose { half_extents, center: body.transform_point(shape.origin), basis: body.basis * shape.basis }
}

/// The rigid bodies and the static ground, as shapes see them.
struct BodyView<'a> {
    rigid: &'a [RigidBody],
    ground: &'a RigidBody,
}

impl BodyView<'_> {
    /// Body of a shape.
    fn body_of(&self, shape: ShapeRef) -> &RigidBody {
        if shape.is_static { self.ground } else { &self.rigid[shape.body as usize] }
    }

    /// Pose of a shape in world space.
    fn pose_of(&self, shape: ShapeRef) -> BoxPose {
        let body = self.body_of(shape);
        world_pose(&body.transform, body.half_extents)
    }
}

/// Swap-removes a contact from a contact list, fixing the moved contact's
/// local index.
fn list_remove(contacts: &mut [Contact], list: &mut Vec<u32>, id: u32) {
    let local = contacts[id as usize].local;
    list.swap_remove(local as usize);
    if (local as usize) < list.len() {
        contacts[list[local as usize] as usize].local = local;
    }
    contacts[id as usize].local = NULL_LINK;
}

/// Graph bodies of a contact: a static B counts as body 0.
fn graph_bodies(contact: &Contact) -> GraphBodies {
    let b_static = contact.shape_b.is_static;
    GraphBodies { a: contact.shape_a.body, b: if b_static { 0 } else { contact.shape_b.body }, b_is_static: b_static }
}

/// Root of `node` in a union-find forest, halving the path on the way.
fn find_root(parents: &mut [u32], mut node: u32) -> u32 {
    while parents[node as usize] != node {
        parents[node as usize] = parents[parents[node as usize] as usize];
        node = parents[node as usize];
    }
    node
}

pub struct World {
    pool: Option<TaskPool>,
    context: SolverContext,
    tolerances: CollisionTolerances,

    rigid: Vec<RigidBody>,
    bodies: Vec<BodyRecord>,
    rigid_proxies: Vec<ProxyId>,
    ground: RigidBody,
    static_motions: Vec<KinematicMotion>,

    broad_phase: BroadPhase,
    contacts: Vec<Contact>,
    free_contacts: Vec<u32>,
    contact_index: FxHashMap<u64, u32>,
    awake_contacts: Vec<u32>,
    awake_bits: Vec<u64>,
    disabled_contacts: Vec<u32>,
    graph: ConstraintGraph,
    static_edges: Vec<Vec<u32>>,

    awake_bodies: Vec<u32>,
    islands: Vec<Island>,
    free_islands: Vec<u32>,
    awake_islands: Vec<u32>,
    sleeping_sets: Vec<SleepingSet>,
    free_sets: Vec<u32>,
    live_islands: u32,
    sleeping_body_count: u32,
    sleeping_touching: u32,
    sleeping_points: u32,
    split_island_id: u32,
    change_log: Vec<u32>,
    pending_changes: Vec<u32>,
    island_awake: Vec<u8>,

    collide_ids: Vec<u32>,
    changed_blocks: Vec<Vec<u32>>,
    changed_ids: Vec<u32>,
    proxy_moves: Vec<Vec<ProxyMove>>,
    pair_candidates: Vec<Vec<ProxyPair>>,
    deltas: Vec<BodyDelta>,
    active_bodies: Vec<u32>,
    body_local: Vec<u32>,
    solver: ContactSolver,
}

impl World {
    /// A world stepped on `threads` threads, one meaning no pool.
    pub fn new(threads: u32) -> World {
        let context = SolverContext::engine_default();
        World {
            pool: if threads > 1 { Some(TaskPool::new(threads)) } else { None },
            context,
            tolerances: CollisionTolerances { speculative_distance: 4.0 * context.linear_slop, linear_slop: context.linear_slop },
            rigid: Vec::new(),
            bodies: Vec::new(),
            rigid_proxies: Vec::new(),
            ground: RigidBody::default(),
            static_motions: Vec::new(),
            broad_phase: BroadPhase::default(),
            contacts: Vec::new(),
            free_contacts: Vec::new(),
            contact_index: FxHashMap::default(),
            awake_contacts: Vec::new(),
            awake_bits: Vec::new(),
            disabled_contacts: Vec::new(),
            graph: ConstraintGraph::default(),
            static_edges: Vec::new(),
            awake_bodies: Vec::new(),
            islands: Vec::new(),
            free_islands: Vec::new(),
            awake_islands: Vec::new(),
            sleeping_sets: Vec::new(),
            free_sets: Vec::new(),
            live_islands: 0,
            sleeping_body_count: 0,
            sleeping_touching: 0,
            sleeping_points: 0,
            split_island_id: NULL_LINK,
            change_log: Vec::new(),
            pending_changes: Vec::new(),
            island_awake: Vec::new(),
            collide_ids: Vec::new(),
            changed_blocks: Vec::new(),
            changed_ids: Vec::new(),
            proxy_moves: Vec::new(),
            pair_candidates: Vec::new(),
            deltas: Vec::new(),
            active_bodies: Vec::new(),
            body_local: Vec::new(),
            solver: ContactSolver::default(),
        }
    }

    /// Runs `f(begin, end)` over [0, count), on the pool when worthwhile.
    fn parallel_for(pool: Option<&TaskPool>, count: usize, grain: usize, f: impl Fn(usize, usize) + Sync) {
        match pool {
            Some(pool) if count >= 2 * grain => pool.parallel_for(count, grain, f),
            _ if count != 0 => f(0, count),
            _ => {}
        }
    }

    /// Removes everything and builds the piles again.
    pub fn reset(&mut self) {
        self.rigid.clear();
        self.bodies.clear();
        self.rigid_proxies.clear();
        self.broad_phase = BroadPhase::default();
        self.contacts.clear();
        self.free_contacts.clear();
        self.contact_index.clear();
        self.awake_contacts.clear();
        self.awake_bits.clear();
        self.disabled_contacts.clear();
        self.graph = ConstraintGraph::default();
        self.static_edges.clear();
        self.static_edges.push(Vec::new());
        self.awake_bodies.clear();
        self.islands.clear();
        self.free_islands.clear();
        self.awake_islands.clear();
        self.sleeping_sets.clear();
        self.sleeping_sets.push(SleepingSet::default());
        self.free_sets.clear();
        self.live_islands = 0;
        self.sleeping_body_count = 0;
        self.sleeping_touching = 0;
        self.sleeping_points = 0;
        self.split_island_id = NULL_LINK;
        self.change_log.clear();

        let mut rng = hash::Rng::new(0x3011d);
        self.rigid.reserve((PILES_SIDE * PILES_SIDE * PILE_HEIGHT + PILES_SIDE * PILES_SIDE / 4) as usize);
        for pz in 0..PILES_SIDE {
            for px in 0..PILES_SIDE {
                let x = px as f32 * PILE_SPACING - 16.5 + rng.unit() * 0.04 - 0.02;
                let z = pz as f32 * PILE_SPACING - 16.5 + rng.unit() * 0.04 - 0.02;
                let mut top = 0.0f32;
                for _ in 0..PILE_HEIGHT {
                    let half = Vec3::random(&mut rng, 0.4, 0.5);
                    let yaw = Quat { x: 0.0, y: rng.unit() * 0.2 - 0.1, z: 0.0, w: 1.0 }.normalize();
                    self.create_body(Vec3::new(x, top + half.y + 0.005, z), half, yaw);
                    top = top + 2.0 * half.y + 0.005;
                }
            }
        }

        for pile in (0..PILES_SIDE * PILES_SIDE).step_by(4) {
            let x = (pile % PILES_SIDE) as f32 * PILE_SPACING - 16.5;
            let z = (pile / PILES_SIDE) as f32 * PILE_SPACING - 16.5;
            let half = Vec3::random(&mut rng, 0.3, 0.5);
            let px = x + rng.unit() * 0.4 - 0.2;
            let py = 9.5 + rng.unit();
            let pz = z + rng.unit() * 0.4 - 0.2;
            let rotation = Quat::random(&mut rng);
            self.create_body(Vec3::new(px, py, pz), half, rotation);
        }

        self.ground = RigidBody::default();
        self.ground.transform.origin = Vec3::new(0.0, -1.0, 0.0);
        self.ground.half_extents = Vec3::new(24.0, 1.0, 24.0);
        self.static_motions.clear();
        self.static_motions.push(KinematicMotion { center: self.ground.transform.origin, ..KinematicMotion::default() });
        let ground_aabb = box_aabb(&world_pose(&self.ground.transform, self.ground.half_extents)).grow(self.tolerances.speculative_distance);
        self.broad_phase.create_proxy(ShapeRef { body: 0, is_static: true }, &ground_aabb, false);
    }

    /// Adds a body at `position` with half extents `half` and `rotation`.
    fn create_body(&mut self, position: Vec3, half: Vec3, rotation: Quat) {
        let slot = self.rigid.len() as u32;
        let mut body = RigidBody { rotation, ..RigidBody::default() };
        body.transform = Transform { basis: Basis::from_quat(rotation), origin: position };
        body.set_box_mass(half);
        let tight = box_aabb(&world_pose(&body.transform, half)).grow(self.tolerances.speculative_distance);
        self.rigid.push(body);
        self.bodies.push(BodyRecord::default());
        self.activate_body(slot);
        let proxy = self.broad_phase.create_proxy(ShapeRef { body: slot, is_static: false }, &tight.grow(shape_margin(half)), false);
        self.rigid_proxies.push(proxy);
    }

    /// Puts a body in the awake set in an island of its own.
    fn activate_body(&mut self, slot: u32) {
        let local = self.awake_bodies.len() as u32;
        self.awake_bodies.push(slot);
        let island_id = self.create_island(AWAKE_SET);
        self.islands[island_id as usize].bodies.push(slot);
        let record = &mut self.bodies[slot as usize];
        record.set = AWAKE_SET;
        record.local = local;
        record.island = island_id;
        record.island_local = 0;
    }

    /// Queues a body for the next step's change processing.
    fn notify(&mut self, slot: u32) {
        let record = &mut self.bodies[slot as usize];
        if record.logged {
            return;
        }
        record.logged = true;
        self.change_log.push(slot);
    }

    /// Wakes the top body of every eighth pile with a sideways shove.
    pub fn shove(&mut self) {
        for pile in (0..PILES_SIDE * PILES_SIDE).step_by(8) {
            let slot = pile * PILE_HEIGHT + PILE_HEIGHT - 1;
            let body = &mut self.rigid[slot as usize];
            body.linear_velocity = Vec3::new(2.0, 0.0, 1.0);
            body.sleeping = false;
            body.sleep_time = 0.0;
            self.notify(slot);
        }
    }

    /// Wakes the sets of bodies whose state changed since the last step.
    fn process_body_changes(&mut self) {
        if self.change_log.is_empty() {
            return;
        }
        self.pending_changes.clear();
        self.pending_changes.extend_from_slice(&self.change_log);
        self.change_log.clear();
        self.pending_changes.sort_unstable();
        for i in 0..self.pending_changes.len() {
            let slot = self.pending_changes[i] as usize;
            self.bodies[slot].logged = false;
            let set = self.bodies[slot].set;
            if !self.rigid[slot].sleeping && set != AWAKE_SET {
                self.wake_set(set);
            }
        }
    }

    /// Edge list of a shape's body.
    fn edges_of(&mut self, shape: ShapeRef) -> &mut Vec<u32> {
        if shape.is_static { &mut self.static_edges[shape.body as usize] } else { &mut self.bodies[shape.body as usize].edges }
    }

    /// The shape of one side of a contact.
    fn side_shape(contact: &Contact, side: EdgeSide) -> ShapeRef {
        match side {
            EdgeSide::A => contact.shape_a,
            EdgeSide::B => contact.shape_b,
        }
    }

    /// Records a contact in one of its bodies' edge lists.
    fn add_edge(&mut self, id: u32, side: EdgeSide) {
        let index = side as u32;
        let shape = World::side_shape(&self.contacts[id as usize], side);
        let list = self.edges_of(shape);
        let local = list.len() as u32;
        list.push((id << 1) | index);
        self.contacts[id as usize].edge_local[index as usize] = local;
    }

    /// Removes a contact from one of its bodies' edge lists.
    fn remove_edge(&mut self, id: u32, side: EdgeSide) {
        let index = side as u32;
        let contact = self.contacts[id as usize];
        let local = contact.edge_local[index as usize];
        let list = self.edges_of(World::side_shape(&contact, side));
        list.swap_remove(local as usize);
        let moved = list.get(local as usize).copied();
        if let Some(last) = moved {
            self.contacts[(last >> 1) as usize].edge_local[(last & 1) as usize] = local;
        }
        self.contacts[id as usize].edge_local[index as usize] = NULL_LINK;
    }

    /// Sets or clears a contact's bit in the awake bitset.
    fn mark_awake(&mut self, id: u32, awake: bool) {
        let word = id as usize / 64;
        if self.awake_bits.len() <= word {
            self.awake_bits.resize(word + 1, 0);
        }
        let bit = 1u64 << (id % 64);
        if awake {
            self.awake_bits[word] |= bit;
        } else {
            self.awake_bits[word] &= !bit;
        }
    }

    /// Adds a non-touching contact to the awake list.
    fn awake_add(&mut self, id: u32) {
        self.mark_awake(id, true);
        let contact = &mut self.contacts[id as usize];
        contact.set = AWAKE_SET;
        contact.color = NULL_LINK;
        contact.local = self.awake_contacts.len() as u32;
        self.awake_contacts.push(id);
    }

    /// Adds a contact to the list of contacts between sleeping bodies.
    fn disabled_add(&mut self, id: u32) {
        self.mark_awake(id, false);
        let contact = &mut self.contacts[id as usize];
        contact.set = DISABLED_SET;
        contact.color = NULL_LINK;
        contact.local = self.disabled_contacts.len() as u32;
        self.disabled_contacts.push(id);
    }

    /// Colours a touching contact into the constraint graph.
    fn graph_add(&mut self, id: u32) {
        self.graph.reserve_bodies(self.bodies.len());
        let contact = &mut self.contacts[id as usize];
        let slot = self.graph.add(id, &graph_bodies(contact));
        contact.color = slot.color;
        contact.local = slot.local;
        contact.set = AWAKE_SET;
        self.mark_awake(id, true);
    }

    /// Takes a contact out of the constraint graph.
    fn graph_remove(&mut self, id: u32) {
        let contact = self.contacts[id as usize];
        let moved = self.graph.remove(GraphSlot { color: contact.color, local: contact.local }, &graph_bodies(&contact));
        if moved != NULL_LINK {
            self.contacts[moved as usize].local = contact.local;
        }
        let contact = &mut self.contacts[id as usize];
        contact.color = NULL_LINK;
        contact.local = NULL_LINK;
    }

    /// Creates a contact for a new pair.
    fn create_contact(&mut self, pair: &ProxyPair) -> u32 {
        let id = match self.free_contacts.pop() {
            None => {
                self.contacts.push(Contact::default());
                (self.contacts.len() - 1) as u32
            }
            Some(id) => id,
        };
        let contact = Contact {
            alive: true,
            shape_a: self.broad_phase.shape(pair.a),
            shape_b: self.broad_phase.shape(pair.b),
            proxy_a: pair.a,
            proxy_b: pair.b,
            ..Contact::default()
        };
        self.contacts[id as usize] = contact;
        self.contact_index.entry(pair.key).or_insert(id);
        self.add_edge(id, EdgeSide::A);
        self.add_edge(id, EdgeSide::B);

        let a_awake = self.bodies[contact.shape_a.body as usize].set == AWAKE_SET;
        let b_awake = !contact.shape_b.is_static && self.bodies[contact.shape_b.body as usize].set == AWAKE_SET;
        if a_awake || b_awake {
            self.awake_add(id);
        } else {
            self.disabled_add(id);
        }
        id
    }

    /// Wakes the sleeping set of a body, if it sleeps.
    fn wake_body_set(&mut self, slot: u32) {
        let set = self.bodies[slot as usize].set;
        if set != AWAKE_SET && set != NULL_LINK {
            self.wake_set(set);
        }
    }

    /// Destroys a contact and unlinks it from everything.
    fn destroy_contact(&mut self, id: u32, wake: bool) {
        let contact = self.contacts[id as usize];
        self.contact_index.remove(&pair_key(contact.shape_a, contact.shape_b));
        let slot_a = contact.shape_a.body;
        let slot_b = if contact.shape_b.is_static { NULL_LINK } else { contact.shape_b.body };

        if wake && contact.linked {
            self.wake_body_set(slot_a);
            if slot_b != NULL_LINK {
                self.wake_body_set(slot_b);
            }
        }

        self.remove_edge(id, EdgeSide::A);
        self.remove_edge(id, EdgeSide::B);

        if self.contacts[id as usize].island != NULL_LINK {
            self.unlink_contact(id);
        }
        let contact = self.contacts[id as usize];
        if contact.color != NULL_LINK {
            self.graph_remove(id);
        } else if contact.set == AWAKE_SET {
            list_remove(&mut self.contacts, &mut self.awake_contacts, id);
        } else if contact.set == DISABLED_SET {
            list_remove(&mut self.contacts, &mut self.disabled_contacts, id);
        } else if contact.set != NULL_LINK {
            list_remove(&mut self.contacts, &mut self.sleeping_sets[contact.set as usize].contacts, id);
            if contact.touching {
                self.sleeping_touching -= 1;
                self.sleeping_points -= contact.manifold.point_count;
            }
        }
        self.mark_awake(id, false);
        let contact = &mut self.contacts[id as usize];
        contact.alive = false;
        contact.set = NULL_LINK;
        self.free_contacts.push(id);
    }

    /// Links a touching contact into its bodies' islands, merging them.
    fn link_contact(&mut self, id: u32) {
        let contact = self.contacts[id as usize];
        let slot_a = contact.shape_a.body;
        let slot_b = if contact.shape_b.is_static { NULL_LINK } else { contact.shape_b.body };

        if slot_b != NULL_LINK {
            let set_a = self.bodies[slot_a as usize].set;
            let set_b = self.bodies[slot_b as usize].set;
            if set_a == AWAKE_SET && set_b != AWAKE_SET && set_b != NULL_LINK {
                self.wake_set(set_b);
            } else if set_b == AWAKE_SET && set_a != AWAKE_SET && set_a != NULL_LINK {
                self.wake_set(set_a);
            }
        }

        let island_a = self.bodies[slot_a as usize].island;
        let island_b = if slot_b == NULL_LINK { NULL_LINK } else { self.bodies[slot_b as usize].island };
        let merged = self.merge_islands(island_a, island_b);

        let island = &mut self.islands[merged as usize];
        let local = island.contacts.len() as u32;
        island.contacts.push(ContactLink { contact: id, body_a: slot_a, body_b: slot_b });
        let contact = &mut self.contacts[id as usize];
        contact.island = merged;
        contact.island_local = local;
        contact.linked = true;
    }

    /// Unlinks a contact that stopped touching from its island.
    fn unlink_contact(&mut self, id: u32) {
        let contact = self.contacts[id as usize];
        let island = &mut self.islands[contact.island as usize];
        let local = contact.island_local;
        island.contacts.swap_remove(local as usize);
        if let Some(moved) = island.contacts.get(local as usize) {
            self.contacts[moved.contact as usize].island_local = local;
        }
        island.remove_count += 1;
        let contact = &mut self.contacts[id as usize];
        contact.island = NULL_LINK;
        contact.island_local = NULL_LINK;
        contact.linked = false;
    }

    /// A new island in `set`.
    fn create_island(&mut self, set: u32) -> u32 {
        let id = match self.free_islands.pop() {
            None => {
                self.islands.push(Island::default());
                (self.islands.len() - 1) as u32
            }
            Some(id) => id,
        };
        let mut island = Island { alive: true, set, ..Island::default() };
        if set == AWAKE_SET {
            island.local = self.awake_islands.len() as u32;
            self.awake_islands.push(id);
        }
        self.islands[id as usize] = island;
        self.live_islands += 1;
        id
    }

    /// Frees an island.
    fn destroy_island(&mut self, id: u32) {
        if self.split_island_id == id {
            self.split_island_id = NULL_LINK;
        }
        let island = std::mem::take(&mut self.islands[id as usize]);
        let list = if island.set == AWAKE_SET { &mut self.awake_islands } else { &mut self.sleeping_sets[island.set as usize].islands };
        list.swap_remove(island.local as usize);
        if let Some(&moved) = list.get(island.local as usize) {
            self.islands[moved as usize].local = island.local;
        }
        self.free_islands.push(id);
        self.live_islands -= 1;
    }

    /// Merges the smaller island into the larger; returns the survivor.
    fn merge_islands(&mut self, a: u32, b: u32) -> u32 {
        if a == b {
            return a;
        }
        if a == NULL_LINK {
            return b;
        }
        if b == NULL_LINK {
            return a;
        }
        let (big, small) = if self.islands[a as usize].bodies.len() < self.islands[b as usize].bodies.len() { (b, a) } else { (a, b) };
        let small_bodies = std::mem::take(&mut self.islands[small as usize].bodies);
        let small_contacts = std::mem::take(&mut self.islands[small as usize].contacts);
        for &slot in &small_bodies {
            let big_island = &mut self.islands[big as usize];
            let record = &mut self.bodies[slot as usize];
            record.island = big;
            record.island_local = big_island.bodies.len() as u32;
            big_island.bodies.push(slot);
        }
        for &link in &small_contacts {
            let big_island = &mut self.islands[big as usize];
            let contact = &mut self.contacts[link.contact as usize];
            contact.island = big;
            contact.island_local = big_island.contacts.len() as u32;
            big_island.contacts.push(link);
        }
        let small_removes = self.islands[small as usize].remove_count;
        self.islands[big as usize].remove_count += small_removes;
        self.destroy_island(small);
        big
    }

    /// Splits an island into its connected components.
    fn split_island(&mut self, base_id: u32) {
        let base_bodies = std::mem::take(&mut self.islands[base_id as usize].bodies);
        let base_contacts = std::mem::take(&mut self.islands[base_id as usize].contacts);
        let count = base_bodies.len() as u32;

        let mut parents: Vec<u32> = (0..count).collect();
        let mut ranks = vec![0u32; count as usize];
        for link in &base_contacts {
            if link.body_b == NULL_LINK {
                continue;
            }
            let mut root_a = find_root(&mut parents, self.bodies[link.body_a as usize].island_local);
            let mut root_b = find_root(&mut parents, self.bodies[link.body_b as usize].island_local);
            if root_a == root_b {
                continue;
            }
            if ranks[root_a as usize] < ranks[root_b as usize] {
                std::mem::swap(&mut root_a, &mut root_b);
            }
            parents[root_b as usize] = root_a;
            if ranks[root_a as usize] == ranks[root_b as usize] {
                ranks[root_a as usize] += 1;
            }
        }

        let mut components = 0u32;
        for i in 0..count {
            parents[i as usize] = find_root(&mut parents, i);
            components += u32::from(parents[i as usize] == i);
        }
        if components == 1 {
            let island = &mut self.islands[base_id as usize];
            island.bodies = base_bodies;
            island.contacts = base_contacts;
            island.remove_count = 0;
            return;
        }

        let mut root_island = vec![NULL_LINK; count as usize];
        let mut island_ids = Vec::with_capacity(components as usize);
        for i in 0..count {
            if parents[i as usize] == i {
                root_island[i as usize] = island_ids.len() as u32;
                island_ids.push(self.create_island(AWAKE_SET));
            }
        }
        for (i, &slot) in base_bodies.iter().enumerate() {
            let target = island_ids[root_island[parents[i] as usize] as usize];
            let island = &mut self.islands[target as usize];
            let record = &mut self.bodies[slot as usize];
            record.island = target;
            record.island_local = island.bodies.len() as u32;
            island.bodies.push(slot);
        }
        for &link in &base_contacts {
            let target = self.bodies[link.body_a as usize].island;
            let island = &mut self.islands[target as usize];
            let contact = &mut self.contacts[link.contact as usize];
            contact.island = target;
            contact.island_local = island.contacts.len() as u32;
            island.contacts.push(link);
        }
        self.destroy_island(base_id);
    }

    /// Splits the island chosen by the last sleep pass.
    fn split_pending_island(&mut self) {
        let id = self.split_island_id;
        self.split_island_id = NULL_LINK;
        let Some(island) = self.islands.get(id as usize) else {
            return;
        };
        if !island.alive || island.set != AWAKE_SET || island.remove_count == 0 {
            return;
        }
        self.split_island(id);
    }

    /// Moves a resting island and its contacts into a new sleeping set.
    fn try_sleep_island(&mut self, id: u32) {
        if self.islands[id as usize].remove_count > 0 && self.islands[id as usize].bodies.len() > 1 {
            return;
        }
        let set_id = match self.free_sets.pop() {
            None => {
                self.sleeping_sets.push(SleepingSet::default());
                (self.sleeping_sets.len() - 1) as u32
            }
            Some(id) => id,
        };
        self.sleeping_sets[set_id as usize] = SleepingSet::default();

        for k in 0..self.islands[id as usize].bodies.len() {
            let slot = self.islands[id as usize].bodies[k];
            self.sleep_body(slot, set_id);
        }

        for k in 0..self.islands[id as usize].contacts.len() {
            let contact_id = self.islands[id as usize].contacts[k].contact;
            self.graph_remove(contact_id);
            self.mark_awake(contact_id, false);
            let set = &mut self.sleeping_sets[set_id as usize];
            let contact = &mut self.contacts[contact_id as usize];
            contact.set = set_id;
            contact.local = set.contacts.len() as u32;
            set.contacts.push(contact_id);
            self.sleeping_touching += 1;
            self.sleeping_points += contact.manifold.point_count;
        }

        let local = self.islands[id as usize].local;
        self.awake_islands.swap_remove(local as usize);
        if let Some(&moved) = self.awake_islands.get(local as usize) {
            self.islands[moved as usize].local = local;
        }
        let set = &mut self.sleeping_sets[set_id as usize];
        let island = &mut self.islands[id as usize];
        island.set = set_id;
        island.local = set.islands.len() as u32;
        set.islands.push(id);
        self.sleeping_body_count += set.bodies.len() as u32;
        if self.split_island_id == id {
            self.split_island_id = NULL_LINK;
        }
    }

    /// Moves one body of a falling-asleep island into sleeping set `set_id`
    /// and disables its awake contacts with sleeping or static partners.
    fn sleep_body(&mut self, slot: u32, set_id: u32) {
        let local = self.bodies[slot as usize].local;
        self.awake_bodies.swap_remove(local as usize);
        if let Some(&moved) = self.awake_bodies.get(local as usize) {
            self.bodies[moved as usize].local = local;
        }
        let body = &mut self.rigid[slot as usize];
        body.sleeping = true;
        body.linear_velocity = Vec3::default();
        body.angular_velocity = Vec3::default();
        body.sleep_velocity = 0.0;
        let set = &mut self.sleeping_sets[set_id as usize];
        let record = &mut self.bodies[slot as usize];
        record.set = set_id;
        record.local = set.bodies.len() as u32;
        set.bodies.push(slot);

        for k in 0..self.bodies[slot as usize].edges.len() {
            let key = self.bodies[slot as usize].edges[k];
            let contact_id = key >> 1;
            let contact = &self.contacts[contact_id as usize];
            if contact.color != NULL_LINK {
                continue;
            }
            let other = if key & 1 == 1 {
                self.bodies[contact.shape_a.body as usize].set
            } else if !contact.shape_b.is_static {
                self.bodies[contact.shape_b.body as usize].set
            } else {
                NULL_LINK
            };
            if other == AWAKE_SET {
                continue;
            }
            list_remove(&mut self.contacts, &mut self.awake_contacts, contact_id);
            self.disabled_add(contact_id);
        }
    }

    /// Moves a sleeping set back to the awake set.
    fn wake_set(&mut self, set_id: u32) {
        for k in 0..self.sleeping_sets[set_id as usize].bodies.len() {
            let slot = self.sleeping_sets[set_id as usize].bodies[k];
            let record = &mut self.bodies[slot as usize];
            record.set = AWAKE_SET;
            record.local = self.awake_bodies.len() as u32;
            self.awake_bodies.push(slot);
            let body = &mut self.rigid[slot as usize];
            body.sleeping = false;
            body.sleep_time = 0.0;
            for e in 0..self.bodies[slot as usize].edges.len() {
                let contact_id = self.bodies[slot as usize].edges[e] >> 1;
                if self.contacts[contact_id as usize].set == DISABLED_SET {
                    list_remove(&mut self.contacts, &mut self.disabled_contacts, contact_id);
                    self.awake_add(contact_id);
                }
            }
        }
        for k in 0..self.sleeping_sets[set_id as usize].contacts.len() {
            let contact_id = self.sleeping_sets[set_id as usize].contacts[k];
            self.sleeping_touching -= 1;
            self.sleeping_points -= self.contacts[contact_id as usize].manifold.point_count;
            self.graph_add(contact_id);
        }
        for k in 0..self.sleeping_sets[set_id as usize].islands.len() {
            let island_id = self.sleeping_sets[set_id as usize].islands[k];
            let island = &mut self.islands[island_id as usize];
            island.set = AWAKE_SET;
            island.local = self.awake_islands.len() as u32;
            self.awake_islands.push(island_id);
        }
        let set = std::mem::take(&mut self.sleeping_sets[set_id as usize]);
        self.sleeping_body_count -= set.bodies.len() as u32;
        self.free_sets.push(set_id);
    }

    /// Clears the first `blocks` per-block buffers, growing the list to fit.
    fn reset_blocks<T>(buffers: &mut Vec<Vec<T>>, blocks: usize) {
        if buffers.len() < blocks {
            buffers.resize_with(blocks, Vec::new);
        }
        for buffer in &mut buffers[..blocks] {
            buffer.clear();
        }
    }

    /// Moves the proxies of awake bodies whose bounds left their fat bounds.
    fn refresh_proxy_bounds(&mut self) {
        let speculative = self.tolerances.speculative_distance;
        let count = self.awake_bodies.len();
        if count == 0 {
            return;
        }
        const GRAIN: usize = 256;
        let blocks = count.div_ceil(GRAIN);
        World::reset_blocks(&mut self.proxy_moves, blocks);
        let (awake_bodies, rigid, rigid_proxies, broad_phase) = (&self.awake_bodies, &self.rigid, &self.rigid_proxies, &self.broad_phase);
        let proxy_moves = Disjoint::new(&mut self.proxy_moves);
        World::parallel_for(self.pool.as_ref(), count, GRAIN, |begin, end| {
            // SAFETY: each block owns the move buffer at its own block index.
            let moves = unsafe { proxy_moves.get_mut(begin / GRAIN) };
            for &slot in &awake_bodies[begin..end] {
                let body = &rigid[slot as usize];
                let tight = box_aabb(&world_pose(&body.transform, body.half_extents)).grow(speculative);
                let proxy = rigid_proxies[slot as usize];
                if !broad_phase.fat_aabb(proxy).contains(&tight) {
                    moves.push(ProxyMove { proxy, fat: tight.grow(shape_margin(body.half_extents)) });
                }
            }
        });
        for block in 0..blocks {
            for &ProxyMove { proxy, fat } in &self.proxy_moves[block] {
                self.broad_phase.move_proxy(proxy, &fat);
            }
        }
    }

    /// Creates contacts for the new pairs of the moved proxies.
    fn update_pairs(&mut self) {
        const GRAIN: usize = 64;
        let moved = self.broad_phase.moved_count();
        let blocks = moved.div_ceil(GRAIN);
        World::reset_blocks(&mut self.pair_candidates, blocks);
        let (broad_phase, contact_index) = (&self.broad_phase, &self.contact_index);
        let pair_candidates = Disjoint::new(&mut self.pair_candidates);
        World::parallel_for(self.pool.as_ref(), moved, GRAIN, |begin, end| {
            // SAFETY: each block owns the candidate buffer at its own block index.
            let found = unsafe { pair_candidates.get_mut(begin / GRAIN) };
            broad_phase.query_moved(begin, end, |a, b| {
                let key = pair_key(broad_phase.shape(a), broad_phase.shape(b));
                if !contact_index.contains_key(&key) {
                    found.push(ProxyPair { a, b, key });
                }
            });
        });
        for block in 0..blocks {
            for k in 0..self.pair_candidates[block].len() {
                let pair = self.pair_candidates[block][k];
                if !self.contact_index.contains_key(&pair.key) {
                    self.create_contact(&pair);
                }
            }
        }
        self.broad_phase.clear_moves();
    }

    /// Collides every awake contact and processes those that began or ended
    /// touching.
    fn collide(&mut self) {
        self.collide_ids.clear();
        for (word, &bits) in self.awake_bits.iter().enumerate() {
            let mut bits = bits;
            while bits != 0 {
                self.collide_ids.push((word * 64) as u32 + bits.trailing_zeros());
                bits &= bits - 1;
            }
        }
        const GRAIN: usize = 128;
        let blocks = self.collide_ids.len().div_ceil(GRAIN);
        World::reset_blocks(&mut self.changed_blocks, blocks);
        let view = BodyView { rigid: &self.rigid, ground: &self.ground };
        let (collide_ids, broad_phase, tolerances) = (&self.collide_ids, &self.broad_phase, &self.tolerances);
        let changed_blocks = Disjoint::new(&mut self.changed_blocks);
        let contacts = Disjoint::new(&mut self.contacts);
        World::parallel_for(self.pool.as_ref(), collide_ids.len(), GRAIN, |begin, end| {
            // SAFETY: each block owns the change buffer at its own block index.
            let changed = unsafe { changed_blocks.get_mut(begin / GRAIN) };
            for &id in &collide_ids[begin..end] {
                // SAFETY: the ids come from a bitset, so each is collided by
                // exactly one block.
                let contact = unsafe { contacts.get_mut(id as usize) };
                if !broad_phase.fat_aabb(contact.proxy_a).overlaps(broad_phase.fat_aabb(contact.proxy_b)) {
                    contact.touching = false;
                    changed.push((id << 1) | 1);
                    continue;
                }
                let poses = ContactPoses { a: view.pose_of(contact.shape_a), b: view.pose_of(contact.shape_b) };
                if !try_recycle_contact(contact, &poses, tolerances) {
                    collide_boxes(&poses.a, &poses.b, tolerances, &mut contact.manifold);
                    cache_contact(contact, &poses);
                }
                contact.touching = contact.manifold.point_count > 0;
                let a = view.body_of(contact.shape_a);
                let b = view.body_of(contact.shape_b);
                contact.friction = (a.friction * b.friction).sqrt();
                contact.restitution = vecmath::max(a.restitution, b.restitution);
                contact.rolling_resistance = vecmath::max(a.rolling_resistance, b.rolling_resistance);
                if contact.touching != contact.linked {
                    changed.push(id << 1);
                }
            }
        });

        self.changed_ids.clear();
        for block in &self.changed_blocks[..blocks] {
            self.changed_ids.extend_from_slice(block);
        }
        self.changed_ids.sort_unstable();
        self.process_contact_changes();
    }

    /// Applies the begin and end changes found by collide.
    fn process_contact_changes(&mut self) {
        for k in 0..self.changed_ids.len() {
            let encoded = self.changed_ids[k];
            let id = encoded >> 1;
            let contact = self.contacts[id as usize];
            if !contact.alive {
                continue;
            }
            if encoded & 1 != 0 {
                self.destroy_contact(id, false);
            } else if contact.touching && !contact.linked {
                self.link_contact(id);
                list_remove(&mut self.contacts, &mut self.awake_contacts, id);
                self.graph_add(id);
            } else if !contact.touching && contact.linked {
                self.contacts[id as usize].was_touching = false;
                self.unlink_contact(id);
                self.graph_remove(id);
                self.awake_add(id);
            }
        }
    }

    /// Runs the contact solver over the awake bodies.
    fn solve(&mut self) {
        self.active_bodies.clear();
        self.body_local.resize(self.bodies.len(), 0);
        for (index, &slot) in self.awake_bodies.iter().enumerate() {
            self.active_bodies.push(slot);
            self.body_local[slot as usize] = index as u32;
        }
        self.deltas.clear();
        self.deltas.resize(self.awake_bodies.len(), BodyDelta::default());
        let graph = &self.graph;
        let mut inputs = SolverInputs {
            contacts: &mut self.contacts,
            colors: std::array::from_fn::<_, GRAPH_COLOR_COUNT, _>(|color| graph.contacts(color)),
            body_local: &self.body_local,
            active_bodies: &self.active_bodies,
            bodies: &mut self.rigid,
            static_motions: &self.static_motions,
            deltas: &mut self.deltas,
            context: self.context,
            pool: self.pool.as_ref(),
        };
        self.solver.solve(&mut inputs);
    }

    /// Updates sleep timers and puts resting islands to sleep.
    fn finalize_sleep(&mut self) {
        if self.island_awake.len() < self.islands.len() {
            self.island_awake.resize(self.islands.len(), 0);
        }
        for &id in &self.awake_islands {
            self.island_awake[id as usize] = 0;
        }
        let mut candidate = NULL_LINK;
        let mut candidate_time = 0.0f32;
        for (index, &slot) in self.awake_bodies.iter().enumerate() {
            let record = &self.bodies[slot as usize];
            let body = &mut self.rigid[slot as usize];
            let reach = body.max_extent.length();
            let velocity = body.linear_velocity.length() + body.angular_velocity.length() * reach;
            let delta = &self.deltas[index];
            let axis = Vec3::new(delta.rotation.x, delta.rotation.y, delta.rotation.z);
            let angle = 2.0 * axis.length();
            let moved = delta.position.length() + 2.0 * angle * reach;
            let sleep_velocity = vecmath::max(velocity, 0.5 * self.context.inv_dt * moved);
            let angular_sleep_velocity = vecmath::max(body.angular_velocity.length(), angle * self.context.inv_dt);
            body.sleep_velocity = sleep_velocity;
            if !body.can_sleep || sleep_velocity > SLEEP_VELOCITY_THRESHOLD || angular_sleep_velocity > SLEEP_ANGULAR_VELOCITY_THRESHOLD {
                body.sleep_time = 0.0;
            } else {
                body.sleep_time += self.context.dt;
            }
            let island = &self.islands[record.island as usize];
            body.island = island.bodies[0];
            if body.sleep_time <= TIME_TO_SLEEP {
                self.island_awake[record.island as usize] = 1;
            } else if island.remove_count > 0 && (body.sleep_time > candidate_time || (body.sleep_time == candidate_time && record.island > candidate)) {
                candidate = record.island;
                candidate_time = body.sleep_time;
            }
        }
        self.split_island_id = candidate;
        for i in (0..self.awake_islands.len()).rev() {
            let id = self.awake_islands[i];
            if self.island_awake[id as usize] == 0 {
                self.try_sleep_island(id);
            }
        }
    }

    /// Advances one fixed step.
    pub fn step(&mut self) {
        self.process_body_changes();
        self.refresh_proxy_bounds();
        self.update_pairs();
        self.collide();
        self.split_pending_island();
        self.solve();
        self.finalize_sleep();
        for &slot in &self.active_bodies {
            let body = &mut self.rigid[slot as usize];
            body.applied_force = Vec3::default();
            body.applied_torque = Vec3::default();
        }
    }

    /// Number of awake bodies.
    pub fn awake_count(&self) -> usize {
        self.awake_bodies.len()
    }

    /// Checksum of the bodies and the bookkeeping counts.
    pub fn checksum(&self) -> u64 {
        let bits = hash::f32_bits;
        let mut h = 0u64;
        for body in &self.rigid {
            let p = body.transform.origin;
            h = hash::add(h, bits(p.x) as u64 | ((bits(p.y) as u64) << 32));
            h = hash::add(h, bits(p.z) as u64 | ((bits(body.rotation.w) as u64) << 32));
            h = hash::add(h, bits(body.linear_velocity.y) as u64 | ((body.sleeping as u64) << 32));
        }
        h = hash::add(h, self.live_islands as u64);
        h = hash::add(h, self.sleeping_body_count as u64);
        h = hash::add(h, self.sleeping_touching as u64);
        h = hash::add(h, self.sleeping_points as u64);
        h = hash::add(h, self.contact_index.len() as u64);
        h = hash::add(h, self.graph.size() as u64);
        h
    }
}

/// The world step: 64 steps of 864 boxes in 144 piles of six settling on a
/// ground and falling asleep island by island, 36 tumbling boxes dropped onto
/// every fourth pile that wake it on landing, and every eighth pile woken by a
/// shove at step 45. `THREADS` workers run the engine's parallel sections.
pub struct WorldCase<const THREADS: u32> {
    world: World,
}

impl<const THREADS: u32> Case for WorldCase<THREADS> {
    const NAME: &'static str = if THREADS == 1 { "world" } else { "world4" };

    /// A world with its pool.
    fn init() -> WorldCase<THREADS> {
        WorldCase { world: World::new(THREADS) }
    }

    /// Rebuilds the piles and steps them, folding the awake counts.
    fn run(&mut self) -> u64 {
        self.world.reset();
        let mut h = 0u64;
        for step in 0..STEPS {
            if step == SHOVE_STEP {
                self.world.shove();
            }
            self.world.step();
            h = h.wrapping_mul(31).wrapping_add(self.world.awake_count() as u64);
        }
        h ^ self.world.checksum()
    }
}
