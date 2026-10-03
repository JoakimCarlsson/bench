//! The engine's eight-lane contact solver, single-threaded: 4 steps of a
//! 32x32 field of 8-box stacks, 8192 bodies and about 12k contacts coloured
//! by the engine's constraint graph. Each step prepares constraints, packs
//! each colour into eight-lane bundles, runs 4 substeps of warm start, soft
//! biased solve, position integration and relaxed solve with friction, then
//! restitution, and writes impulses and poses back.
const std = @import("std");
const hash = @import("hash.zig");
const vm = @import("vecmath.zig");
const contact_types = @import("contact.zig");
const ConstraintGraph = @import("constraint_graph.zig");
const RigidBody = @import("rigid_body.zig").RigidBody;
const ContactSolver = @import("contact_solver.zig");

const Vec3 = vm.Vec3;
const Quat = vm.Quat;
const Contact = contact_types.Contact;
const ShapeRef = contact_types.ShapeRef;
const Allocator = std.mem.Allocator;

const Wide = @This();

pub const name = "wide";

const grid: u32 = 32;
const height: u32 = 8;
const body_count: u32 = grid * grid * height;
const steps: usize = 4;

gpa: Allocator,
context: ContactSolver.SolverContext,
initial_bodies: []RigidBody,
initial_contacts: []Contact,
bodies: []RigidBody,
contacts: []Contact,
graph: ConstraintGraph,
colors: [ConstraintGraph.graph_color_count][]const u32,
body_local: []u32,
active: []ContactSolver.ActiveBody,
static_motions: []ContactSolver.KinematicMotion,
deltas: []ContactSolver.BodyDelta,
solver: ContactSolver,

/// Builds the scene's contacts with the scene's material and random
/// separations.
const SceneBuilder = struct {
    contacts: std.ArrayList(Contact),
    rng: *hash.Rng,
    gpa: Allocator,

    /// Appends a contact from body `a` to shape `b`.
    fn addContact(self: *SceneBuilder, a: u32, b: ShapeRef, normal: Vec3) Allocator.Error!*Contact {
        const index: u32 = @intCast(self.contacts.items.len);
        const contact = try self.contacts.addOne(self.gpa);
        contact.* = .{};
        contact.shape_a = .{ .body = a, .is_static = false };
        contact.shape_b = b;
        contact.alive = true;
        contact.manifold.normal = normal;
        contact.friction = 0.6;
        contact.restitution = if (index % 8 == 0) 0.3 else 0.0;
        contact.rolling_resistance = if (index % 5 == 0) 0.05 else 0.0;
        return contact;
    }

    /// Appends a manifold point with a random separation.
    fn addPoint(self: *SceneBuilder, contact: *Contact, point: Vec3) void {
        const p = &contact.manifold.points[contact.manifold.point_count];
        contact.manifold.point_count += 1;
        p.point = point;
        p.separation = self.rng.unit() * 0.025 - 0.02;
    }
};

/// Builds the stacks, their contacts and the colouring.
pub fn init(gpa: Allocator) !Wide {
    var rng: hash.Rng = .{ .s = 0x501e };
    const initial_bodies = try gpa.alloc(RigidBody, body_count);
    for (0..grid) |cz| {
        for (0..grid) |cx| {
            for (0..height) |level| {
                const i = (cz * grid + cx) * height + level;
                const b = &initial_bodies[i];
                b.* = .{};
                const half = Vec3.random(&rng, 0.4, 0.5);
                var tilt: Quat = .{};
                tilt.x = rng.unit() * 0.1 - 0.05;
                tilt.y = rng.unit() * 0.2 - 0.1;
                tilt.z = rng.unit() * 0.1 - 0.05;
                tilt.w = 1.0;
                b.rotation = tilt.normalize();
                b.transform.basis = vm.Basis.fromQuat(b.rotation);
                b.transform.origin = Vec3.init(
                    @as(f32, @floatFromInt(cx)) * 1.1,
                    @as(f32, @floatFromInt(level)) * 1.0 + 0.5,
                    @as(f32, @floatFromInt(cz)) * 1.1,
                );
                b.setBoxMass(half);
                b.linear_velocity = Vec3.random(&rng, -0.1, 0.1);
                if (i % 16 == 0) b.linear_velocity.y = -2.0;
                b.angular_velocity = Vec3.random(&rng, -0.1, 0.1);
                b.angular_damp = 0.05;
            }
        }
    }

    var builder: SceneBuilder = .{ .contacts = .empty, .rng = &rng, .gpa = gpa };
    const corners = [4][2]f32{ .{ 1.0, 1.0 }, .{ -1.0, 1.0 }, .{ -1.0, -1.0 }, .{ 1.0, -1.0 } };
    const half = Vec3.init(0.45, 0.45, 0.45);
    for (0..grid) |cz| {
        for (0..grid) |cx| {
            for (0..height) |level| {
                const i: u32 = @intCast((cz * grid + cx) * height + level);
                const center = initial_bodies[i].transform.origin;
                const down = if (level == 0)
                    try builder.addContact(i, .{ .body = 0, .is_static = true }, Vec3.init(0.0, -1.0, 0.0))
                else
                    try builder.addContact(i, .{ .body = i - 1 }, Vec3.init(0.0, -1.0, 0.0));
                for (corners) |corner| builder.addPoint(down, center.add(Vec3.init(corner[0] * half.x, -half.y, corner[1] * half.z)));
                if (cx + 1 < grid and rng.next() % 4 == 0) {
                    const side = try builder.addContact(i, .{ .body = i + height }, Vec3.init(1.0, 0.0, 0.0));
                    builder.addPoint(side, center.add(Vec3.init(half.x, 0.5 * half.y, 0.0)));
                    builder.addPoint(side, center.add(Vec3.init(half.x, -0.5 * half.y, 0.0)));
                }
                if (cz + 1 < grid and rng.next() % 4 == 0) {
                    const side = try builder.addContact(i, .{ .body = i + grid * height }, Vec3.init(0.0, 0.0, 1.0));
                    builder.addPoint(side, center.add(Vec3.init(0.0, 0.5 * half.y, half.z)));
                    builder.addPoint(side, center.add(Vec3.init(0.0, -0.5 * half.y, half.z)));
                }
            }
        }
    }
    const initial_contacts = try builder.contacts.toOwnedSlice(gpa);

    var graph: ConstraintGraph = .{};
    try graph.reserveBodies(gpa, body_count);
    for (initial_contacts, 0..) |*contact, index| {
        const b_static = contact.shape_b.is_static;
        const slot = try graph.add(gpa, @intCast(index), .{
            .a = contact.shape_a.body,
            .b = if (b_static) 0 else contact.shape_b.body,
            .b_is_static = b_static,
        });
        contact.color = slot.color;
        contact.local = slot.local;
    }

    var self: Wide = .{
        .gpa = gpa,
        .context = .standard(),
        .initial_bodies = initial_bodies,
        .initial_contacts = initial_contacts,
        .bodies = try gpa.dupe(RigidBody, initial_bodies),
        .contacts = try gpa.dupe(Contact, initial_contacts),
        .graph = graph,
        .colors = undefined,
        .body_local = try gpa.alloc(u32, body_count),
        .active = try gpa.alloc(ContactSolver.ActiveBody, body_count),
        .static_motions = try gpa.alloc(ContactSolver.KinematicMotion, 1),
        .deltas = try gpa.alloc(ContactSolver.BodyDelta, body_count),
        .solver = .{},
    };
    for (&self.colors, 0..) |*list, color| list.* = self.graph.contacts(@intCast(color));
    for (self.body_local, self.active, self.bodies, 0..) |*local, *active, *body, i| {
        local.* = @intCast(i);
        active.* = .{ .body = body, .slot = @intCast(i) };
    }
    @memset(self.static_motions, .{});
    @memset(self.deltas, .{});
    return self;
}

/// Frees the scene and the solver.
pub fn deinit(self: *Wide, gpa: Allocator) void {
    gpa.free(self.initial_bodies);
    gpa.free(self.initial_contacts);
    gpa.free(self.bodies);
    gpa.free(self.contacts);
    self.graph.deinit(gpa);
    gpa.free(self.body_local);
    gpa.free(self.active);
    gpa.free(self.static_motions);
    gpa.free(self.deltas);
    self.solver.deinit(gpa);
}

/// Four solver steps from the initial scene, then the checksum.
pub fn run(self: *Wide) !u64 {
    @memcpy(self.bodies, self.initial_bodies);
    @memcpy(self.contacts, self.initial_contacts);
    const inputs: ContactSolver.SolverInputs = .{
        .contacts = self.contacts,
        .colors = &self.colors,
        .body_local = self.body_local,
        .active_bodies = self.active,
        .static_motions = self.static_motions,
        .deltas = self.deltas,
        .context = self.context,
    };
    for (0..steps) |_| try self.solver.solve(self.gpa, &inputs);
    var h: u64 = 0;
    for (self.bodies, self.deltas) |b, delta| {
        h = hash.add(h, @as(u64, hash.f32Bits(b.linear_velocity.x)) | (@as(u64, hash.f32Bits(b.linear_velocity.y)) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(b.transform.origin.y)) | (@as(u64, hash.f32Bits(b.rotation.w)) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(b.angular_velocity.z)) | (@as(u64, hash.f32Bits(delta.rotation.x)) << 32));
    }
    for (self.contacts) |c| {
        const m = &c.manifold;
        for (m.points[0..m.point_count]) |p| {
            h = hash.add(h, @as(u64, hash.f32Bits(p.normal_impulse)) | (@as(u64, hash.f32Bits(p.total_normal_impulse)) << 32));
            h = hash.add(h, @as(u64, hash.f32Bits(p.peak_normal_impulse)) | (@as(u64, hash.f32Bits(p.relative_velocity)) << 32));
        }
        const f = c.friction_impulses;
        h = hash.add(h, @as(u64, hash.f32Bits(f.tangent_x)) | (@as(u64, hash.f32Bits(f.tangent_y)) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(f.twist)) | (@as(u64, hash.f32Bits(f.rolling.y)) << 32));
    }
    return h;
}
