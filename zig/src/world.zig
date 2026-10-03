//! The engine's PhysicsWorld step for bodies of one box each on a static
//! ground: body changes and waking, proxy refits, pair finding into a hash
//! map, box collision with contact recycling, contact begin and end with
//! island linking and graph colouring, the eight-lane solver, and sleep with
//! island splitting.
const std = @import("std");
const hash = @import("hash.zig");
const vm = @import("vecmath.zig");
const contact_types = @import("contact.zig");
const box_collision = @import("box_collision.zig");
const contact_recycle = @import("contact_recycle.zig");
const BroadPhase = @import("broad_phase.zig");
const ConstraintGraph = @import("constraint_graph.zig");
const RigidBody = @import("rigid_body.zig").RigidBody;
const ContactSolver = @import("contact_solver.zig");
const TaskPool = @import("task_pool.zig");

const Vec3 = vm.Vec3;
const Aabb = vm.Aabb;
const BoxPose = vm.BoxPose;
const Contact = contact_types.Contact;
const ShapeRef = contact_types.ShapeRef;
const Allocator = std.mem.Allocator;
const awake_set = contact_types.awake_set;
const disabled_set = contact_types.disabled_set;
const null_link = contact_types.null_link;
const graph_color_count = ConstraintGraph.graph_color_count;

const sleep_velocity_threshold: f32 = 0.05;
const sleep_angular_velocity_threshold: f32 = 0.05;
const time_to_sleep: f32 = 0.5;
const piles_side: u32 = 12;
const pile_height: u32 = 6;
const pile_spacing: f32 = 3.0;

/// World pose of a body's single box, shape transform identity.
fn worldPose(body: vm.Transform, half_extents: Vec3) BoxPose {
    const shape: vm.Transform = .{};
    return .{
        .half_extents = half_extents,
        .center = body.transformPoint(shape.origin),
        .basis = body.basis.mul(shape.basis),
    };
}

/// Replaces a list's contents with `count` copies of `value`.
fn assign(comptime T: type, list: *std.ArrayList(T), gpa: Allocator, count: usize, value: T) Allocator.Error!void {
    list.clearRetainingCapacity();
    try list.appendNTimes(gpa, value, count);
}

/// Resizes a list, filling new elements with `value`.
fn resizeWith(comptime T: type, list: *std.ArrayList(T), gpa: Allocator, count: usize, value: T) Allocator.Error!void {
    if (count <= list.items.len) {
        list.shrinkRetainingCapacity(count);
        return;
    }
    try list.appendNTimes(gpa, value, count - list.items.len);
}

/// Deinitialises every element of a list of owning values, then empties it.
fn clearOwned(comptime T: type, list: *std.ArrayList(T), gpa: Allocator) void {
    for (list.items) |*item| item.deinit(gpa);
    list.clearRetainingCapacity();
}

/// Deinitialises every element of a list of owning values and frees it.
fn deinitOwned(comptime T: type, list: *std.ArrayList(T), gpa: Allocator) void {
    for (list.items) |*item| item.deinit(gpa);
    list.deinit(gpa);
}

/// Swap-removes `list[local]`; returns the element moved into its place, if
/// any.
fn swapRemoveAt(comptime T: type, list: *std.ArrayList(T), local: u32) ?T {
    const last = list.pop().?;
    if (local < list.items.len) {
        list.items[local] = last;
        return last;
    }
    return null;
}

pub const World = struct {
    const EdgeSide = enum(u32) { a = 0, b = 1 };

    const BodyRecord = struct {
        set: u32 = null_link,
        local: u32 = null_link,
        island: u32 = null_link,
        island_local: u32 = null_link,
        edges: std.ArrayList(u32) = .empty,
        alive: bool = false,
        logged: bool = false,

        /// Frees the edge list.
        fn deinit(self: *BodyRecord, gpa: Allocator) void {
            self.edges.deinit(gpa);
        }
    };

    const ContactLink = struct {
        contact: u32 = 0,
        body_a: u32 = 0,
        body_b: u32 = 0,
    };

    const Island = struct {
        set: u32 = null_link,
        local: u32 = null_link,
        remove_count: u32 = 0,
        bodies: std.ArrayList(u32) = .empty,
        contacts: std.ArrayList(ContactLink) = .empty,
        alive: bool = false,

        /// Frees the body and contact lists.
        fn deinit(self: *Island, gpa: Allocator) void {
            self.bodies.deinit(gpa);
            self.contacts.deinit(gpa);
        }

        /// Frees the lists and returns the island to its default state.
        fn reset(self: *Island, gpa: Allocator) void {
            self.deinit(gpa);
            self.* = .{};
        }
    };

    const SleepingSet = struct {
        bodies: std.ArrayList(u32) = .empty,
        contacts: std.ArrayList(u32) = .empty,
        islands: std.ArrayList(u32) = .empty,
        alive: bool = false,

        /// Frees the body, contact and island lists.
        fn deinit(self: *SleepingSet, gpa: Allocator) void {
            self.bodies.deinit(gpa);
            self.contacts.deinit(gpa);
            self.islands.deinit(gpa);
        }

        /// Frees the lists and returns the set to its default state.
        fn reset(self: *SleepingSet, gpa: Allocator) void {
            self.deinit(gpa);
            self.* = .{};
        }
    };

    const ProxyMove = struct {
        proxy: i32,
        fat: Aabb,
    };

    const ProxyPair = struct {
        a: i32,
        b: i32,
        key: u64,
    };

    gpa: Allocator,
    pool: ?*TaskPool = null,
    context: ContactSolver.SolverContext,
    tolerances: box_collision.CollisionTolerances,
    task_failed: std.atomic.Value(bool) = .init(false),

    rigid: std.ArrayList(RigidBody) = .empty,
    bodies: std.ArrayList(BodyRecord) = .empty,
    rigid_proxies: std.ArrayList(i32) = .empty,
    ground: RigidBody = .{},
    ground_proxy: i32 = -1,
    static_motions: std.ArrayList(ContactSolver.KinematicMotion) = .empty,

    broad_phase: BroadPhase = .{},
    contacts: std.ArrayList(Contact) = .empty,
    free_contacts: std.ArrayList(u32) = .empty,
    contact_index: std.AutoHashMapUnmanaged(u64, u32) = .empty,
    awake_contacts: std.ArrayList(u32) = .empty,
    awake_bits: std.ArrayList(u64) = .empty,
    disabled_contacts: std.ArrayList(u32) = .empty,
    graph: ConstraintGraph = .{},
    static_edges: std.ArrayList(std.ArrayList(u32)) = .empty,

    awake_bodies: std.ArrayList(u32) = .empty,
    islands: std.ArrayList(Island) = .empty,
    free_islands: std.ArrayList(u32) = .empty,
    awake_islands: std.ArrayList(u32) = .empty,
    sleeping_sets: std.ArrayList(SleepingSet) = .empty,
    free_sets: std.ArrayList(u32) = .empty,
    live_islands: u32 = 0,
    sleeping_body_count: u32 = 0,
    sleeping_touching: u32 = 0,
    sleeping_points: u32 = 0,
    split_island_id: u32 = null_link,
    change_log: std.ArrayList(u32) = .empty,
    pending_changes: std.ArrayList(u32) = .empty,
    island_awake: std.ArrayList(u8) = .empty,

    collide_ids: std.ArrayList(u32) = .empty,
    changed_blocks: std.ArrayList(std.ArrayList(u32)) = .empty,
    changed_ids: std.ArrayList(u32) = .empty,
    proxy_moves: std.ArrayList(std.ArrayList(ProxyMove)) = .empty,
    pair_candidates: std.ArrayList(std.ArrayList(ProxyPair)) = .empty,
    deltas: std.ArrayList(ContactSolver.BodyDelta) = .empty,
    active_bodies: std.ArrayList(ContactSolver.ActiveBody) = .empty,
    body_local: std.ArrayList(u32) = .empty,
    color_lists: [graph_color_count][]const u32 = @splat(&.{}),
    solver: ContactSolver = .{},

    /// A world stepped on `threads` threads, one meaning no pool.
    pub fn init(gpa: Allocator, io: std.Io, threads: u32) !World {
        const context: ContactSolver.SolverContext = .standard();
        return .{
            .gpa = gpa,
            .pool = if (threads > 1) try TaskPool.create(gpa, io, threads) else null,
            .context = context,
            .tolerances = .{ .speculative_distance = 4.0 * context.linear_slop, .linear_slop = context.linear_slop },
        };
    }

    /// Stops the pool and frees everything.
    pub fn deinit(self: *World) void {
        const gpa = self.gpa;
        if (self.pool) |pool| pool.destroy(gpa);
        self.rigid.deinit(gpa);
        deinitOwned(BodyRecord, &self.bodies, gpa);
        self.rigid_proxies.deinit(gpa);
        self.static_motions.deinit(gpa);
        self.broad_phase.deinit(gpa);
        self.contacts.deinit(gpa);
        self.free_contacts.deinit(gpa);
        self.contact_index.deinit(gpa);
        self.awake_contacts.deinit(gpa);
        self.awake_bits.deinit(gpa);
        self.disabled_contacts.deinit(gpa);
        self.graph.deinit(gpa);
        deinitOwned(std.ArrayList(u32), &self.static_edges, gpa);
        self.awake_bodies.deinit(gpa);
        deinitOwned(Island, &self.islands, gpa);
        self.free_islands.deinit(gpa);
        self.awake_islands.deinit(gpa);
        deinitOwned(SleepingSet, &self.sleeping_sets, gpa);
        self.free_sets.deinit(gpa);
        self.change_log.deinit(gpa);
        self.pending_changes.deinit(gpa);
        self.island_awake.deinit(gpa);
        self.collide_ids.deinit(gpa);
        deinitOwned(std.ArrayList(u32), &self.changed_blocks, gpa);
        self.changed_ids.deinit(gpa);
        deinitOwned(std.ArrayList(ProxyMove), &self.proxy_moves, gpa);
        deinitOwned(std.ArrayList(ProxyPair), &self.pair_candidates, gpa);
        self.deltas.deinit(gpa);
        self.active_bodies.deinit(gpa);
        self.body_local.deinit(gpa);
        self.solver.deinit(gpa);
    }

    /// Removes everything and builds the piles again.
    pub fn reset(self: *World) Allocator.Error!void {
        const gpa = self.gpa;
        self.rigid.clearRetainingCapacity();
        clearOwned(BodyRecord, &self.bodies, gpa);
        self.rigid_proxies.clearRetainingCapacity();
        self.broad_phase.deinit(gpa);
        self.contacts.clearRetainingCapacity();
        self.free_contacts.clearRetainingCapacity();
        self.contact_index.clearRetainingCapacity();
        self.awake_contacts.clearRetainingCapacity();
        self.awake_bits.clearRetainingCapacity();
        self.disabled_contacts.clearRetainingCapacity();
        self.graph.deinit(gpa);
        clearOwned(std.ArrayList(u32), &self.static_edges, gpa);
        try self.static_edges.append(gpa, .empty);
        self.awake_bodies.clearRetainingCapacity();
        clearOwned(Island, &self.islands, gpa);
        self.free_islands.clearRetainingCapacity();
        self.awake_islands.clearRetainingCapacity();
        clearOwned(SleepingSet, &self.sleeping_sets, gpa);
        try self.sleeping_sets.append(gpa, .{});
        self.free_sets.clearRetainingCapacity();
        self.live_islands = 0;
        self.sleeping_body_count = 0;
        self.sleeping_touching = 0;
        self.sleeping_points = 0;
        self.split_island_id = null_link;
        self.change_log.clearRetainingCapacity();

        var rng: hash.Rng = .{ .s = 0x3011d };
        try self.rigid.ensureTotalCapacity(gpa, piles_side * piles_side * pile_height + piles_side * piles_side / 4);
        for (0..piles_side) |pz| {
            for (0..piles_side) |px| {
                const x = @as(f32, @floatFromInt(px)) * pile_spacing - 16.5 + rng.unit() * 0.04 - 0.02;
                const z = @as(f32, @floatFromInt(pz)) * pile_spacing - 16.5 + rng.unit() * 0.04 - 0.02;
                var top: f32 = 0.0;
                for (0..pile_height) |_| {
                    const half = Vec3.random(&rng, 0.4, 0.5);
                    const raw_yaw: vm.Quat = .{ .x = 0.0, .y = rng.unit() * 0.2 - 0.1, .z = 0.0, .w = 1.0 };
                    try self.createBody(Vec3.init(x, top + half.y + 0.005, z), half, raw_yaw.normalize());
                    top = top + 2.0 * half.y + 0.005;
                }
            }
        }

        var pile: u32 = 0;
        while (pile < piles_side * piles_side) : (pile += 4) {
            const x = @as(f32, @floatFromInt(pile % piles_side)) * pile_spacing - 16.5;
            const z = @as(f32, @floatFromInt(pile / piles_side)) * pile_spacing - 16.5;
            const half = Vec3.random(&rng, 0.3, 0.5);
            const px = x + rng.unit() * 0.4 - 0.2;
            const py = 9.5 + rng.unit();
            const pz = z + rng.unit() * 0.4 - 0.2;
            try self.createBody(Vec3.init(px, py, pz), half, vm.Quat.random(&rng));
        }

        self.ground = .{};
        self.ground.transform.origin = Vec3.init(0.0, -1.0, 0.0);
        self.ground.half_extents = Vec3.init(24.0, 1.0, 24.0);
        try assign(ContactSolver.KinematicMotion, &self.static_motions, gpa, 1, .{ .center = self.ground.transform.origin });
        const ground_box = worldPose(self.ground.transform, self.ground.half_extents);
        self.ground_proxy = try self.broad_phase.createProxy(gpa, .{ .body = 0, .is_static = true }, box_collision.boxAabb(&ground_box).grow(self.tolerances.speculative_distance), false);
    }

    /// Advances one fixed step.
    pub fn step(self: *World) !void {
        try self.processBodyChanges();
        try self.refreshProxyBounds();
        try self.updatePairs();
        try self.collide();
        try self.splitPendingIsland();
        try self.solve();
        try self.finalizeSleep();
        for (self.active_bodies.items) |active| {
            active.body.applied_force = .{};
            active.body.applied_torque = .{};
        }
    }

    /// Wakes the top body of every eighth pile with a sideways shove.
    pub fn shove(self: *World) Allocator.Error!void {
        var pile: u32 = 0;
        while (pile < piles_side * piles_side) : (pile += 8) {
            const slot = pile * pile_height + pile_height - 1;
            const body = &self.rigid.items[slot];
            body.linear_velocity = Vec3.init(2.0, 0.0, 1.0);
            body.sleeping = false;
            body.sleep_time = 0.0;
            try self.notify(slot);
        }
    }

    /// Checksum of the bodies and the bookkeeping counts.
    pub fn checksum(self: *const World) u64 {
        var h: u64 = 0;
        for (self.rigid.items) |body| {
            const p = body.transform.origin;
            h = hash.add(h, @as(u64, hash.f32Bits(p.x)) | (@as(u64, hash.f32Bits(p.y)) << 32));
            h = hash.add(h, @as(u64, hash.f32Bits(p.z)) | (@as(u64, hash.f32Bits(body.rotation.w)) << 32));
            h = hash.add(h, @as(u64, hash.f32Bits(body.linear_velocity.y)) | (@as(u64, @intFromBool(body.sleeping)) << 32));
        }
        h = hash.add(h, self.live_islands);
        h = hash.add(h, self.sleeping_body_count);
        h = hash.add(h, self.sleeping_touching);
        h = hash.add(h, self.sleeping_points);
        h = hash.add(h, self.contact_index.count());
        h = hash.add(h, self.graph.size());
        return h;
    }

    /// Number of awake bodies.
    pub fn awakeCount(self: *const World) usize {
        return self.awake_bodies.items.len;
    }

    /// Runs `task.range(begin, end)` over [0, count), on the pool when
    /// worthwhile; an allocation failure inside a task surfaces here.
    fn parallelFor(self: *World, count: usize, grain: usize, task: anytype) Allocator.Error!void {
        self.task_failed.store(false, .monotonic);
        if (self.pool != null and count >= 2 * grain) {
            self.pool.?.parallelFor(count, grain, task);
        } else if (count != 0) {
            task.range(0, count);
        }
        if (self.task_failed.load(.monotonic)) return error.OutOfMemory;
    }

    /// Records that a parallel task ran out of memory.
    fn failTask(self: *World) void {
        self.task_failed.store(true, .monotonic);
    }

    /// Adds a body at `position` with half extents `half` and `rotation`.
    fn createBody(self: *World, position: Vec3, half: Vec3, rotation: vm.Quat) Allocator.Error!void {
        const gpa = self.gpa;
        const slot: u32 = @intCast(self.rigid.items.len);
        const body = try self.rigid.addOne(gpa);
        body.* = .{};
        body.rotation = rotation;
        body.transform = .{ .basis = vm.Basis.fromQuat(rotation), .origin = position };
        body.setBoxMass(half);
        try self.bodies.append(gpa, .{ .alive = true });
        try self.activateBody(slot);
        const box = worldPose(body.transform, half);
        const tight = box_collision.boxAabb(&box).grow(self.tolerances.speculative_distance);
        const proxy = try self.broad_phase.createProxy(gpa, .{ .body = slot, .is_static = false }, tight.grow(BroadPhase.shapeMargin(half)), false);
        try self.rigid_proxies.append(gpa, proxy);
    }

    /// Puts a body in the awake set in an island of its own.
    fn activateBody(self: *World, slot: u32) Allocator.Error!void {
        const record = &self.bodies.items[slot];
        record.set = awake_set;
        record.local = @intCast(self.awake_bodies.items.len);
        try self.awake_bodies.append(self.gpa, slot);
        const island_id = try self.createIsland(awake_set);
        record.island = island_id;
        record.island_local = 0;
        try self.islands.items[island_id].bodies.append(self.gpa, slot);
    }

    /// Queues a body for the next step's change processing.
    fn notify(self: *World, slot: u32) Allocator.Error!void {
        const record = &self.bodies.items[slot];
        if (record.logged) return;
        record.logged = true;
        try self.change_log.append(self.gpa, slot);
    }

    /// Wakes the sets of bodies whose state changed since the last step.
    fn processBodyChanges(self: *World) Allocator.Error!void {
        if (self.change_log.items.len == 0) return;
        self.pending_changes.clearRetainingCapacity();
        try self.pending_changes.appendSlice(self.gpa, self.change_log.items);
        self.change_log.clearRetainingCapacity();
        std.mem.sortUnstable(u32, self.pending_changes.items, {}, std.sort.asc(u32));
        for (self.pending_changes.items) |slot| {
            const record = &self.bodies.items[slot];
            record.logged = false;
            if (!self.rigid.items[slot].sleeping and record.set != awake_set) try self.wakeSet(record.set);
        }
    }

    /// Pose of a shape in world space.
    fn poseOf(self: *const World, ref: ShapeRef) BoxPose {
        const body = self.bodyOf(ref);
        return worldPose(body.transform, body.half_extents);
    }

    /// Body of a shape.
    fn bodyOf(self: *const World, ref: ShapeRef) *const RigidBody {
        return if (ref.is_static) &self.ground else &self.rigid.items[ref.body];
    }

    /// Edge list of a shape's body.
    fn edgesOf(self: *World, ref: ShapeRef) *std.ArrayList(u32) {
        if (ref.is_static) return &self.static_edges.items[ref.body];
        return &self.bodies.items[ref.body].edges;
    }

    /// Shape on one side of a contact.
    fn shapeOnSide(contact: *const Contact, side: EdgeSide) ShapeRef {
        return if (side == .a) contact.shape_a else contact.shape_b;
    }

    /// Records a contact in one of its bodies' edge lists.
    fn addEdge(self: *World, id: u32, side: EdgeSide) Allocator.Error!void {
        const contact = &self.contacts.items[id];
        const index = @intFromEnum(side);
        const list = self.edgesOf(shapeOnSide(contact, side));
        contact.edge_local[index] = @intCast(list.items.len);
        try list.append(self.gpa, (id << 1) | index);
    }

    /// Removes a contact from one of its bodies' edge lists.
    fn removeEdge(self: *World, id: u32, side: EdgeSide) void {
        const contact = &self.contacts.items[id];
        const index = @intFromEnum(side);
        const list = self.edgesOf(shapeOnSide(contact, side));
        const local = contact.edge_local[index];
        if (swapRemoveAt(u32, list, local)) |moved| self.contacts.items[moved >> 1].edge_local[moved & 1] = local;
        contact.edge_local[index] = null_link;
    }

    /// Swap-removes a contact from a contact list.
    fn listRemove(self: *World, list: *std.ArrayList(u32), id: u32) void {
        const local = self.contacts.items[id].local;
        if (swapRemoveAt(u32, list, local)) |moved| self.contacts.items[moved].local = local;
        self.contacts.items[id].local = null_link;
    }

    /// Sets or clears a contact's bit in the awake bitset.
    fn markAwake(self: *World, id: u32, awake: bool) Allocator.Error!void {
        const word = id / 64;
        if (self.awake_bits.items.len <= word) try resizeWith(u64, &self.awake_bits, self.gpa, word + 1, 0);
        const bit = @as(u64, 1) << @intCast(id % 64);
        if (awake) {
            self.awake_bits.items[word] |= bit;
        } else {
            self.awake_bits.items[word] &= ~bit;
        }
    }

    /// Adds a non-touching contact to the awake list.
    fn awakeAdd(self: *World, id: u32) Allocator.Error!void {
        try self.markAwake(id, true);
        const contact = &self.contacts.items[id];
        contact.set = awake_set;
        contact.color = null_link;
        contact.local = @intCast(self.awake_contacts.items.len);
        try self.awake_contacts.append(self.gpa, id);
    }

    /// Adds a contact to the list of contacts between sleeping bodies.
    fn disabledAdd(self: *World, id: u32) Allocator.Error!void {
        try self.markAwake(id, false);
        const contact = &self.contacts.items[id];
        contact.set = disabled_set;
        contact.color = null_link;
        contact.local = @intCast(self.disabled_contacts.items.len);
        try self.disabled_contacts.append(self.gpa, id);
    }

    /// Graph bodies of a contact: body b is unused against the ground.
    fn graphBodies(contact: *const Contact) ConstraintGraph.GraphBodies {
        const b_static = contact.shape_b.is_static;
        return .{ .a = contact.shape_a.body, .b = if (b_static) 0 else contact.shape_b.body, .b_is_static = b_static };
    }

    /// Colours a touching contact into the constraint graph.
    fn graphAdd(self: *World, id: u32) Allocator.Error!void {
        const contact = &self.contacts.items[id];
        try self.graph.reserveBodies(self.gpa, self.bodies.items.len);
        const slot = try self.graph.add(self.gpa, id, graphBodies(contact));
        contact.color = slot.color;
        contact.local = slot.local;
        contact.set = awake_set;
        try self.markAwake(id, true);
    }

    /// Takes a contact out of the constraint graph.
    fn graphRemove(self: *World, id: u32) void {
        const contact = &self.contacts.items[id];
        const moved = self.graph.remove(.{ .color = contact.color, .local = contact.local }, graphBodies(contact));
        if (moved != null_link) self.contacts.items[moved].local = contact.local;
        contact.color = null_link;
        contact.local = null_link;
    }

    /// Creates a contact for a new pair.
    fn createContact(self: *World, pair: ProxyPair) Allocator.Error!u32 {
        var id: u32 = undefined;
        if (self.free_contacts.pop()) |reused| {
            id = reused;
        } else {
            id = @intCast(self.contacts.items.len);
            try self.contacts.append(self.gpa, .{});
        }
        const contact = &self.contacts.items[id];
        contact.* = .{};
        contact.alive = true;
        contact.shape_a = self.broad_phase.shape(pair.a);
        contact.shape_b = self.broad_phase.shape(pair.b);
        contact.proxy_a = pair.a;
        contact.proxy_b = pair.b;
        try self.contact_index.putNoClobber(self.gpa, pair.key, id);
        try self.addEdge(id, .a);
        try self.addEdge(id, .b);

        const a_awake = self.bodies.items[contact.shape_a.body].set == awake_set;
        const b_awake = !contact.shape_b.is_static and self.bodies.items[contact.shape_b.body].set == awake_set;
        if (a_awake or b_awake) {
            try self.awakeAdd(id);
        } else {
            try self.disabledAdd(id);
        }
        return id;
    }

    /// Wakes the set of a body that sleeps.
    fn wakeIfSleeping(self: *World, slot: u32) Allocator.Error!void {
        const set = self.bodies.items[slot].set;
        if (set != awake_set and set != null_link) try self.wakeSet(set);
    }

    /// Destroys a contact and unlinks it from everything.
    fn destroyContact(self: *World, id: u32, wake: bool) Allocator.Error!void {
        const contact = &self.contacts.items[id];
        _ = self.contact_index.remove(contact_types.pairKey(contact.shape_a, contact.shape_b));
        const slot_a = contact.shape_a.body;
        const slot_b = if (contact.shape_b.is_static) null_link else contact.shape_b.body;

        if (wake and contact.linked) {
            try self.wakeIfSleeping(slot_a);
            if (slot_b != null_link) try self.wakeIfSleeping(slot_b);
        }

        self.removeEdge(id, .a);
        self.removeEdge(id, .b);

        if (contact.island != null_link) self.unlinkContact(id);
        if (contact.color != null_link) {
            self.graphRemove(id);
        } else if (contact.set == awake_set) {
            self.listRemove(&self.awake_contacts, id);
        } else if (contact.set == disabled_set) {
            self.listRemove(&self.disabled_contacts, id);
        } else if (contact.set != null_link) {
            self.listRemove(&self.sleeping_sets.items[contact.set].contacts, id);
            if (contact.touching) {
                self.sleeping_touching -= 1;
                self.sleeping_points -= contact.manifold.point_count;
            }
        }
        try self.markAwake(id, false);
        contact.alive = false;
        contact.set = null_link;
        try self.free_contacts.append(self.gpa, id);
    }

    /// Links a touching contact into its bodies' islands, merging them.
    fn linkContact(self: *World, id: u32) Allocator.Error!void {
        const contact = &self.contacts.items[id];
        const slot_a = contact.shape_a.body;
        const slot_b = if (contact.shape_b.is_static) null_link else contact.shape_b.body;

        if (slot_b != null_link) {
            const set_a = self.bodies.items[slot_a].set;
            const set_b = self.bodies.items[slot_b].set;
            if (set_a == awake_set and set_b != awake_set and set_b != null_link) {
                try self.wakeSet(set_b);
            } else if (set_b == awake_set and set_a != awake_set and set_a != null_link) {
                try self.wakeSet(set_a);
            }
        }

        const island_a = self.bodies.items[slot_a].island;
        const island_b = if (slot_b == null_link) null_link else self.bodies.items[slot_b].island;
        const merged = try self.mergeIslands(island_a, island_b);

        const island = &self.islands.items[merged];
        contact.island = merged;
        contact.island_local = @intCast(island.contacts.items.len);
        try island.contacts.append(self.gpa, .{ .contact = id, .body_a = slot_a, .body_b = slot_b });
        contact.linked = true;
    }

    /// Unlinks a contact that stopped touching from its island.
    fn unlinkContact(self: *World, id: u32) void {
        const contact = &self.contacts.items[id];
        const island = &self.islands.items[contact.island];
        const local = contact.island_local;
        if (swapRemoveAt(ContactLink, &island.contacts, local)) |moved| self.contacts.items[moved.contact].island_local = local;
        island.remove_count += 1;
        contact.island = null_link;
        contact.island_local = null_link;
        contact.linked = false;
    }

    /// A new island in `set`.
    fn createIsland(self: *World, set: u32) Allocator.Error!u32 {
        var id: u32 = undefined;
        if (self.free_islands.pop()) |reused| {
            id = reused;
        } else {
            id = @intCast(self.islands.items.len);
            try self.islands.append(self.gpa, .{});
        }
        const island = &self.islands.items[id];
        island.reset(self.gpa);
        island.alive = true;
        island.set = set;
        if (set == awake_set) {
            island.local = @intCast(self.awake_islands.items.len);
            try self.awake_islands.append(self.gpa, id);
        }
        self.live_islands += 1;
        return id;
    }

    /// Frees an island.
    fn destroyIsland(self: *World, id: u32) Allocator.Error!void {
        if (self.split_island_id == id) self.split_island_id = null_link;
        const island = &self.islands.items[id];
        const list = if (island.set == awake_set) &self.awake_islands else &self.sleeping_sets.items[island.set].islands;
        if (swapRemoveAt(u32, list, island.local)) |moved| self.islands.items[moved].local = island.local;
        island.reset(self.gpa);
        try self.free_islands.append(self.gpa, id);
        self.live_islands -= 1;
    }

    /// Merges the smaller island into the larger; returns the survivor.
    fn mergeIslands(self: *World, a: u32, b: u32) Allocator.Error!u32 {
        if (a == b) return a;
        if (a == null_link) return b;
        if (b == null_link) return a;
        var big = a;
        var small = b;
        if (self.islands.items[a].bodies.items.len < self.islands.items[b].bodies.items.len) std.mem.swap(u32, &big, &small);
        const big_island = &self.islands.items[big];
        const small_island = &self.islands.items[small];
        for (small_island.bodies.items) |slot| {
            const record = &self.bodies.items[slot];
            record.island = big;
            record.island_local = @intCast(big_island.bodies.items.len);
            try big_island.bodies.append(self.gpa, slot);
        }
        for (small_island.contacts.items) |link| {
            const contact = &self.contacts.items[link.contact];
            contact.island = big;
            contact.island_local = @intCast(big_island.contacts.items.len);
            try big_island.contacts.append(self.gpa, link);
        }
        big_island.remove_count += small_island.remove_count;
        try self.destroyIsland(small);
        return big;
    }

    /// Union-find root of `start`, halving the path on the way.
    fn findRoot(parents: []u32, start: u32) u32 {
        var node = start;
        while (parents[node] != node) {
            parents[node] = parents[parents[node]];
            node = parents[node];
        }
        return node;
    }

    /// Splits an island into its connected components.
    fn splitIsland(self: *World, base_id: u32) Allocator.Error!void {
        const gpa = self.gpa;
        var base_bodies = self.islands.items[base_id].bodies;
        var base_contacts = self.islands.items[base_id].contacts;
        self.islands.items[base_id].bodies = .empty;
        self.islands.items[base_id].contacts = .empty;
        defer base_bodies.deinit(gpa);
        defer base_contacts.deinit(gpa);
        const count: u32 = @intCast(base_bodies.items.len);

        const parents = try gpa.alloc(u32, count);
        defer gpa.free(parents);
        const ranks = try gpa.alloc(u32, count);
        defer gpa.free(ranks);
        @memset(ranks, 0);
        for (parents, 0..) |*parent, i| parent.* = @intCast(i);
        for (base_contacts.items) |link| {
            if (link.body_b == null_link) continue;
            var root_a = findRoot(parents, self.bodies.items[link.body_a].island_local);
            var root_b = findRoot(parents, self.bodies.items[link.body_b].island_local);
            if (root_a == root_b) continue;
            if (ranks[root_a] < ranks[root_b]) std.mem.swap(u32, &root_a, &root_b);
            parents[root_b] = root_a;
            if (ranks[root_a] == ranks[root_b]) ranks[root_a] += 1;
        }

        var components: u32 = 0;
        for (0..count) |i| {
            parents[i] = findRoot(parents, @intCast(i));
            components += if (parents[i] == i) 1 else 0;
        }
        if (components == 1) {
            const island = &self.islands.items[base_id];
            island.bodies = base_bodies;
            island.contacts = base_contacts;
            island.remove_count = 0;
            base_bodies = .empty;
            base_contacts = .empty;
            return;
        }

        const root_island = try gpa.alloc(u32, count);
        defer gpa.free(root_island);
        @memset(root_island, null_link);
        var island_ids: std.ArrayList(u32) = try .initCapacity(gpa, components);
        defer island_ids.deinit(gpa);
        for (0..count) |i| {
            if (parents[i] == i) {
                root_island[i] = @intCast(island_ids.items.len);
                try island_ids.append(gpa, try self.createIsland(awake_set));
            }
        }
        for (base_bodies.items, parents) |slot, parent| {
            const target = island_ids.items[root_island[parent]];
            const island = &self.islands.items[target];
            const record = &self.bodies.items[slot];
            record.island = target;
            record.island_local = @intCast(island.bodies.items.len);
            try island.bodies.append(gpa, slot);
        }
        for (base_contacts.items) |link| {
            const target = self.bodies.items[link.body_a].island;
            const island = &self.islands.items[target];
            const contact = &self.contacts.items[link.contact];
            contact.island = target;
            contact.island_local = @intCast(island.contacts.items.len);
            try island.contacts.append(gpa, link);
        }
        try self.destroyIsland(base_id);
    }

    /// Splits the island chosen by the last sleep pass.
    fn splitPendingIsland(self: *World) Allocator.Error!void {
        const id = self.split_island_id;
        self.split_island_id = null_link;
        if (id == null_link or id >= self.islands.items.len) return;
        const island = &self.islands.items[id];
        if (!island.alive or island.set != awake_set or island.remove_count == 0) return;
        try self.splitIsland(id);
    }

    /// Moves a resting island and its contacts into a new sleeping set.
    fn trySleepIsland(self: *World, id: u32) Allocator.Error!void {
        const gpa = self.gpa;
        if (self.islands.items[id].remove_count > 0 and self.islands.items[id].bodies.items.len > 1) return;
        var set_id: u32 = undefined;
        if (self.free_sets.pop()) |reused| {
            set_id = reused;
        } else {
            set_id = @intCast(self.sleeping_sets.items.len);
            try self.sleeping_sets.append(gpa, .{});
        }
        const set = &self.sleeping_sets.items[set_id];
        set.reset(gpa);
        set.alive = true;
        const island = &self.islands.items[id];

        for (island.bodies.items) |slot| {
            const record = &self.bodies.items[slot];
            if (swapRemoveAt(u32, &self.awake_bodies, record.local)) |moved| self.bodies.items[moved].local = record.local;
            const body = &self.rigid.items[slot];
            body.sleeping = true;
            body.linear_velocity = .{};
            body.angular_velocity = .{};
            body.sleep_velocity = 0.0;
            record.set = set_id;
            record.local = @intCast(set.bodies.items.len);
            try set.bodies.append(gpa, slot);

            for (record.edges.items) |key| {
                const contact_id = key >> 1;
                const contact = &self.contacts.items[contact_id];
                if (contact.color != null_link) continue;
                const side = key & 1;
                var other = null_link;
                if (side == 1) {
                    other = self.bodies.items[contact.shape_a.body].set;
                } else if (!contact.shape_b.is_static) {
                    other = self.bodies.items[contact.shape_b.body].set;
                }
                if (other == awake_set) continue;
                self.listRemove(&self.awake_contacts, contact_id);
                try self.disabledAdd(contact_id);
            }
        }

        for (island.contacts.items) |link| {
            const contact = &self.contacts.items[link.contact];
            self.graphRemove(link.contact);
            try self.markAwake(link.contact, false);
            contact.set = set_id;
            contact.local = @intCast(set.contacts.items.len);
            try set.contacts.append(gpa, link.contact);
            self.sleeping_touching += 1;
            self.sleeping_points += contact.manifold.point_count;
        }

        if (swapRemoveAt(u32, &self.awake_islands, island.local)) |moved| self.islands.items[moved].local = island.local;
        island.set = set_id;
        island.local = @intCast(set.islands.items.len);
        try set.islands.append(gpa, id);
        self.sleeping_body_count += @intCast(set.bodies.items.len);
        if (self.split_island_id == id) self.split_island_id = null_link;
    }

    /// Moves a sleeping set back to the awake set.
    fn wakeSet(self: *World, set_id: u32) Allocator.Error!void {
        const gpa = self.gpa;
        const set = &self.sleeping_sets.items[set_id];
        for (set.bodies.items) |slot| {
            const record = &self.bodies.items[slot];
            record.set = awake_set;
            record.local = @intCast(self.awake_bodies.items.len);
            try self.awake_bodies.append(gpa, slot);
            const body = &self.rigid.items[slot];
            body.sleeping = false;
            body.sleep_time = 0.0;
            for (record.edges.items) |key| {
                const contact_id = key >> 1;
                if (self.contacts.items[contact_id].set == disabled_set) {
                    self.listRemove(&self.disabled_contacts, contact_id);
                    try self.awakeAdd(contact_id);
                }
            }
        }
        for (set.contacts.items) |contact_id| {
            const contact = &self.contacts.items[contact_id];
            self.sleeping_touching -= 1;
            self.sleeping_points -= contact.manifold.point_count;
            try self.graphAdd(contact_id);
        }
        for (set.islands.items) |island_id| {
            const island = &self.islands.items[island_id];
            island.set = awake_set;
            island.local = @intCast(self.awake_islands.items.len);
            try self.awake_islands.append(gpa, island_id);
        }
        self.sleeping_body_count -= @intCast(set.bodies.items.len);
        set.reset(gpa);
        try self.free_sets.append(gpa, set_id);
    }

    /// Empties the first `blocks` per-block lists, growing the list of lists
    /// to cover them.
    fn prepareBlocks(comptime T: type, lists: *std.ArrayList(std.ArrayList(T)), gpa: Allocator, blocks: usize) Allocator.Error!void {
        if (lists.items.len < blocks) try resizeWith(std.ArrayList(T), lists, gpa, blocks, .empty);
        for (lists.items[0..blocks]) |*list| list.clearRetainingCapacity();
    }

    const refresh_grain: usize = 256;

    /// Finds the awake bodies whose bounds left their fat bounds.
    const RefreshTask = struct {
        world: *World,

        /// Collects the proxy moves of awake bodies [begin, end).
        pub fn range(self: *const RefreshTask, begin: usize, end: usize) void {
            const world = self.world;
            const moves = &world.proxy_moves.items[begin / refresh_grain];
            for (world.awake_bodies.items[begin..end]) |slot| {
                const body = &world.rigid.items[slot];
                const box = worldPose(body.transform, body.half_extents);
                const tight = box_collision.boxAabb(&box).grow(world.tolerances.speculative_distance);
                const proxy = world.rigid_proxies.items[slot];
                if (!world.broad_phase.fatAabb(proxy).contains(tight)) {
                    moves.append(world.gpa, .{ .proxy = proxy, .fat = tight.grow(BroadPhase.shapeMargin(body.half_extents)) }) catch return world.failTask();
                }
            }
        }
    };

    /// Moves the proxies of awake bodies whose bounds left their fat bounds.
    fn refreshProxyBounds(self: *World) Allocator.Error!void {
        const count = self.awake_bodies.items.len;
        if (count == 0) return;
        const blocks = (count + refresh_grain - 1) / refresh_grain;
        try prepareBlocks(ProxyMove, &self.proxy_moves, self.gpa, blocks);
        const task: RefreshTask = .{ .world = self };
        try self.parallelFor(count, refresh_grain, &task);
        for (self.proxy_moves.items[0..blocks]) |moves| {
            for (moves.items) |move| try self.broad_phase.moveProxy(self.gpa, move.proxy, move.fat);
        }
    }

    const pair_grain: usize = 64;

    /// Collects one block's pairs that have no contact yet.
    const PairCollector = struct {
        world: *const World,
        found: *std.ArrayList(ProxyPair),

        /// Keeps the pair of proxies `a` and `b` when its shapes have no contact.
        pub fn pair(self: *const PairCollector, a: i32, b: i32) Allocator.Error!void {
            const world = self.world;
            const key = contact_types.pairKey(world.broad_phase.shape(a), world.broad_phase.shape(b));
            if (!world.contact_index.contains(key)) try self.found.append(world.gpa, .{ .a = a, .b = b, .key = key });
        }
    };

    /// Queries the moved proxies for new pairs.
    const PairTask = struct {
        world: *World,

        /// Queries moved proxies [begin, end).
        pub fn range(self: *const PairTask, begin: usize, end: usize) void {
            const world = self.world;
            const collector: PairCollector = .{ .world = world, .found = &world.pair_candidates.items[begin / pair_grain] };
            world.broad_phase.queryMoved(world.gpa, begin, end, &collector) catch return world.failTask();
        }
    };

    /// Creates contacts for the new pairs of the moved proxies.
    fn updatePairs(self: *World) Allocator.Error!void {
        const moved = self.broad_phase.movedCount();
        const blocks = (moved + pair_grain - 1) / pair_grain;
        try prepareBlocks(ProxyPair, &self.pair_candidates, self.gpa, blocks);
        const task: PairTask = .{ .world = self };
        try self.parallelFor(moved, pair_grain, &task);
        for (self.pair_candidates.items[0..blocks]) |found| {
            for (found.items) |pair| {
                if (!self.contact_index.contains(pair.key)) _ = try self.createContact(pair);
            }
        }
        self.broad_phase.clearMoves();
    }

    const collide_grain: usize = 128;

    /// Collides a range of awake contacts and records those that changed.
    const CollideTask = struct {
        world: *World,

        /// Collides awake contacts [begin, end).
        pub fn range(self: *const CollideTask, begin: usize, end: usize) void {
            const world = self.world;
            const changed = &world.changed_blocks.items[begin / collide_grain];
            for (world.collide_ids.items[begin..end]) |id| {
                const contact = &world.contacts.items[id];
                if (!world.broad_phase.fatAabb(contact.proxy_a).overlaps(world.broad_phase.fatAabb(contact.proxy_b))) {
                    contact.touching = false;
                    changed.append(world.gpa, (id << 1) | 1) catch return world.failTask();
                    continue;
                }
                const poses: contact_recycle.ContactPoses = .{ .a = world.poseOf(contact.shape_a), .b = world.poseOf(contact.shape_b) };
                if (!contact_recycle.tryRecycleContact(contact, &poses, world.tolerances)) {
                    box_collision.collideBoxes(&poses.a, &poses.b, world.tolerances, &contact.manifold);
                    contact_recycle.cacheContact(contact, &poses);
                }
                contact.touching = contact.manifold.point_count > 0;
                const a = world.bodyOf(contact.shape_a);
                const b = world.bodyOf(contact.shape_b);
                contact.friction = @sqrt(a.friction * b.friction);
                contact.restitution = vm.maxf(a.restitution, b.restitution);
                contact.rolling_resistance = vm.maxf(a.rolling_resistance, b.rolling_resistance);
                if (contact.touching != contact.linked) changed.append(world.gpa, id << 1) catch return world.failTask();
            }
        }
    };

    /// Collides every awake contact and processes those that began or ended
    /// touching.
    fn collide(self: *World) Allocator.Error!void {
        const gpa = self.gpa;
        self.collide_ids.clearRetainingCapacity();
        for (self.awake_bits.items, 0..) |word_bits, word| {
            var bits = word_bits;
            while (bits != 0) {
                try self.collide_ids.append(gpa, @intCast(word * 64 + @ctz(bits)));
                bits &= bits - 1;
            }
        }
        const blocks = (self.collide_ids.items.len + collide_grain - 1) / collide_grain;
        try prepareBlocks(u32, &self.changed_blocks, gpa, blocks);
        const task: CollideTask = .{ .world = self };
        try self.parallelFor(self.collide_ids.items.len, collide_grain, &task);

        self.changed_ids.clearRetainingCapacity();
        for (self.changed_blocks.items[0..blocks]) |changed| try self.changed_ids.appendSlice(gpa, changed.items);
        std.mem.sortUnstable(u32, self.changed_ids.items, {}, std.sort.asc(u32));
        try self.processContactChanges();
    }

    /// Applies the begin and end changes found by collide.
    fn processContactChanges(self: *World) Allocator.Error!void {
        for (self.changed_ids.items) |encoded| {
            const id = encoded >> 1;
            const contact = &self.contacts.items[id];
            if (!contact.alive) continue;
            if ((encoded & 1) != 0) {
                try self.destroyContact(id, false);
            } else if (contact.touching and !contact.linked) {
                try self.linkContact(id);
                self.listRemove(&self.awake_contacts, id);
                try self.graphAdd(id);
            } else if (!contact.touching and contact.linked) {
                contact.was_touching = false;
                self.unlinkContact(id);
                self.graphRemove(id);
                try self.awakeAdd(id);
            }
        }
    }

    /// Runs the contact solver over the awake bodies.
    fn solve(self: *World) !void {
        const gpa = self.gpa;
        self.active_bodies.clearRetainingCapacity();
        try resizeWith(u32, &self.body_local, gpa, self.bodies.items.len, 0);
        for (self.awake_bodies.items, 0..) |slot, index| {
            try self.active_bodies.append(gpa, .{ .body = &self.rigid.items[slot], .slot = slot });
            self.body_local.items[slot] = @intCast(index);
        }
        try assign(ContactSolver.BodyDelta, &self.deltas, gpa, self.awake_bodies.items.len, .{});
        for (&self.color_lists, 0..) |*list, color| list.* = self.graph.contacts(@intCast(color));
        const inputs: ContactSolver.SolverInputs = .{
            .contacts = self.contacts.items,
            .colors = &self.color_lists,
            .body_local = self.body_local.items,
            .active_bodies = self.active_bodies.items,
            .static_motions = self.static_motions.items,
            .deltas = self.deltas.items,
            .context = self.context,
            .pool = self.pool,
        };
        try self.solver.solve(gpa, &inputs);
    }

    /// Updates sleep timers and puts resting islands to sleep.
    fn finalizeSleep(self: *World) Allocator.Error!void {
        if (self.island_awake.items.len < self.islands.items.len) try resizeWith(u8, &self.island_awake, self.gpa, self.islands.items.len, 0);
        for (self.awake_islands.items) |id| self.island_awake.items[id] = 0;
        var candidate = null_link;
        var candidate_time: f32 = 0.0;
        for (self.awake_bodies.items, self.deltas.items) |slot, delta| {
            const record = &self.bodies.items[slot];
            const body = &self.rigid.items[slot];
            const reach = body.max_extent.length();
            const velocity = body.linear_velocity.length() + body.angular_velocity.length() * reach;
            const axis = Vec3.init(delta.rotation.x, delta.rotation.y, delta.rotation.z);
            const angle = 2.0 * axis.length();
            const moved = delta.position.length() + 2.0 * angle * reach;
            const sleep_velocity = vm.maxf(velocity, 0.5 * self.context.inv_dt * moved);
            const angular_sleep_velocity = vm.maxf(body.angular_velocity.length(), angle * self.context.inv_dt);
            body.sleep_velocity = sleep_velocity;
            if (!body.can_sleep or sleep_velocity > sleep_velocity_threshold or angular_sleep_velocity > sleep_angular_velocity_threshold) {
                body.sleep_time = 0.0;
            } else {
                body.sleep_time += self.context.dt;
            }
            const island = &self.islands.items[record.island];
            body.island = island.bodies.items[0];
            if (body.sleep_time <= time_to_sleep) {
                self.island_awake.items[record.island] = 1;
            } else if (island.remove_count > 0 and
                (body.sleep_time > candidate_time or (body.sleep_time == candidate_time and record.island > candidate)))
            {
                candidate = record.island;
                candidate_time = body.sleep_time;
            }
        }
        self.split_island_id = candidate;
        var i = self.awake_islands.items.len;
        while (i > 0) : (i -= 1) {
            const id = self.awake_islands.items[i - 1];
            if (self.island_awake.items[id] == 0) try self.trySleepIsland(id);
        }
    }
};

/// The world step: 64 steps of 864 boxes in 144 piles of six settling on a
/// ground and falling asleep island by island, 36 tumbling boxes dropped onto
/// every fourth pile that wake it on landing, and every eighth pile woken by
/// a shove at step 45. `threads` workers run the engine's parallel sections.
pub fn WorldCase(comptime threads: u32) type {
    return struct {
        const Self = @This();

        pub const name = switch (threads) {
            1 => "world",
            2 => "world2",
            4 => "world4",
            8 => "world8",
            else => "world16",
        };

        const steps: usize = 64;
        const shove_step: usize = 45;

        world: World,

        /// A world on `threads` threads.
        pub fn init(gpa: Allocator, io: std.Io) !Self {
            return .{ .world = try World.init(gpa, io, threads) };
        }

        /// Stops the world's pool and frees it.
        pub fn deinit(self: *Self, gpa: Allocator) void {
            _ = gpa;
            self.world.deinit();
        }

        /// Sixty-four steps from a fresh scene; the awake counts folded with
        /// the final checksum.
        pub fn run(self: *Self) !u64 {
            try self.world.reset();
            var h: u64 = 0;
            for (0..steps) |step| {
                if (step == shove_step) try self.world.shove();
                try self.world.step();
                h = h *% 31 +% self.world.awakeCount();
            }
            return h ^ self.world.checksum();
        }
    };
}
