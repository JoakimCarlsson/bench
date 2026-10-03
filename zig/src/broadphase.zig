//! Broad phase: 16 frames of 4096 tumbling boxes over a field of 1024
//! static tiles, through the engine's dynamic AABB tree. Each frame refits
//! fat bounds that no longer hold, reinserts those leaves with rotations,
//! finds the new pairs of every moved proxy, records them in a hash map
//! keyed by shape pair, and drops pairs whose fat bounds parted.
const std = @import("std");
const hash = @import("hash.zig");
const vm = @import("vecmath.zig");
const contact_types = @import("contact.zig");
const box_collision = @import("box_collision.zig");
const BroadPhase = @import("broad_phase.zig");

const Vec3 = vm.Vec3;
const Aabb = vm.Aabb;
const BoxPose = vm.BoxPose;
const Allocator = std.mem.Allocator;

const BroadPhaseCase = @This();

pub const name = "broadphase";

const body_count: u32 = 4096;
const tiles_side: u32 = 32;
const frames: usize = 16;
const speculative: f32 = 0.02;
const dt: f32 = @as(f32, 1.0) / @as(f32, 60.0);
const extent_xz: f32 = 45.0;
const extent_y: f32 = 16.0;

const Body = struct {
    pose: BoxPose = .{},
    rotation: vm.Quat = .{},
    velocity: Vec3 = .{},
    spin: Vec3 = .{},
};

const Pair = struct {
    proxy_a: i32 = 0,
    proxy_b: i32 = 0,
    key: u64 = 0,
    alive: bool = false,
};

gpa: Allocator,
initial: []Body,
bodies: []Body,
tiles: []BoxPose,
body_proxies: []i32,
broad_phase: BroadPhase = .{},
pairs: std.ArrayList(Pair) = .empty,
free_pairs: std.ArrayList(u32) = .empty,
pair_index: std.AutoHashMapUnmanaged(u64, u32) = .empty,

/// Bounds the broad phase tests against: the box grown by the speculative
/// distance.
fn tightAabb(pose: *const BoxPose) Aabb {
    return box_collision.boxAabb(pose).grow(speculative);
}

/// `value` wrapped into [0, extent).
fn wrap(value: f32, extent: f32) f32 {
    if (value < 0.0) return value + extent;
    if (value >= extent) return value - extent;
    return value;
}

/// Draws the bodies and lays out the tiles.
pub fn init(gpa: Allocator) !BroadPhaseCase {
    var rng: hash.Rng = .{ .s = 0xb40ad };
    const initial = try gpa.alloc(Body, body_count);
    for (initial) |*b| {
        b.* = .{};
        b.pose.half_extents = Vec3.random(&rng, 0.25, 0.75);
        const x = rng.unit() * extent_xz;
        const y = rng.unit() * extent_y;
        const z = rng.unit() * extent_xz;
        b.pose.center = Vec3.init(x, y, z);
        b.rotation = vm.Quat.random(&rng);
        b.pose.basis = vm.Basis.fromQuat(b.rotation);
        b.velocity = Vec3.random(&rng, -1.0, 1.0);
        b.spin = Vec3.random(&rng, -0.5, 0.5);
    }
    const tiles = try gpa.alloc(BoxPose, tiles_side * tiles_side);
    for (0..tiles_side) |z| {
        for (0..tiles_side) |x| {
            tiles[z * tiles_side + x] = .{
                .half_extents = Vec3.init(1.0, 0.5, 1.0),
                .center = Vec3.init(@as(f32, @floatFromInt(x)) * 2.0 + 1.0, -0.5, @as(f32, @floatFromInt(z)) * 2.0 + 1.0),
            };
        }
    }
    return .{
        .gpa = gpa,
        .initial = initial,
        .bodies = try gpa.alloc(Body, body_count),
        .tiles = tiles,
        .body_proxies = try gpa.alloc(i32, body_count),
    };
}

/// Frees the scene, the broad phase and the pairs.
pub fn deinit(self: *BroadPhaseCase, gpa: Allocator) void {
    gpa.free(self.initial);
    gpa.free(self.bodies);
    gpa.free(self.tiles);
    gpa.free(self.body_proxies);
    self.broad_phase.deinit(gpa);
    self.pairs.deinit(gpa);
    self.free_pairs.deinit(gpa);
    self.pair_index.deinit(gpa);
}

/// Creates every proxy from the initial poses.
fn reset(self: *BroadPhaseCase) Allocator.Error!void {
    const gpa = self.gpa;
    self.broad_phase.deinit(gpa);
    self.pairs.clearRetainingCapacity();
    self.free_pairs.clearRetainingCapacity();
    self.pair_index.clearRetainingCapacity();
    @memcpy(self.bodies, self.initial);
    for (self.tiles, 0..) |*tile, i| {
        _ = try self.broad_phase.createProxy(gpa, .{ .body = @intCast(i), .is_static = true }, tightAabb(tile), false);
    }
    for (self.bodies, self.body_proxies, 0..) |*b, *proxy, i| {
        const fat = tightAabb(&b.pose).grow(BroadPhase.shapeMargin(b.pose.half_extents));
        proxy.* = try self.broad_phase.createProxy(gpa, .{ .body = @intCast(i), .is_static = false }, fat, false);
    }
}

/// Advances the bodies and moves the proxies whose fat bounds they left.
fn moveBodies(self: *BroadPhaseCase) Allocator.Error!void {
    for (self.bodies, self.body_proxies) |*b, proxy| {
        const c = b.pose.center.add(b.velocity.scale(dt));
        b.pose.center = Vec3.init(wrap(c.x, extent_xz), wrap(c.y, extent_y), wrap(c.z, extent_xz));
        b.rotation = b.rotation.integrate(b.spin.scale(dt));
        b.pose.basis = vm.Basis.fromQuat(b.rotation);
        const tight = tightAabb(&b.pose);
        if (!self.broad_phase.fatAabb(proxy).contains(tight)) {
            try self.broad_phase.moveProxy(self.gpa, proxy, tight.grow(BroadPhase.shapeMargin(b.pose.half_extents)));
        }
    }
}

/// Records a new pair in the pair list and the index.
const PairRecorder = struct {
    case: *BroadPhaseCase,

    /// Adds the pair of proxies `a` and `b` unless its shapes are already paired.
    pub fn pair(self: *const PairRecorder, a: i32, b: i32) Allocator.Error!void {
        const case = self.case;
        const gpa = case.gpa;
        const key = contact_types.pairKey(case.broad_phase.shape(a), case.broad_phase.shape(b));
        if (case.pair_index.contains(key)) return;
        var id: u32 = undefined;
        if (case.free_pairs.pop()) |reused| {
            id = reused;
        } else {
            id = @intCast(case.pairs.items.len);
            try case.pairs.append(gpa, .{});
        }
        case.pairs.items[id] = .{ .proxy_a = a, .proxy_b = b, .key = key, .alive = true };
        try case.pair_index.putNoClobber(gpa, key, id);
    }
};

/// Records the new pairs of the moved proxies.
fn updatePairs(self: *BroadPhaseCase) Allocator.Error!void {
    const recorder: PairRecorder = .{ .case = self };
    try self.broad_phase.updatePairs(self.gpa, &recorder);
}

/// Drops pairs whose fat bounds no longer overlap.
fn dropPartedPairs(self: *BroadPhaseCase) Allocator.Error!void {
    for (self.pairs.items, 0..) |*pair, id| {
        if (!pair.alive) continue;
        if (self.broad_phase.fatAabb(pair.proxy_a).overlaps(self.broad_phase.fatAabb(pair.proxy_b))) continue;
        _ = self.pair_index.remove(pair.key);
        pair.alive = false;
        try self.free_pairs.append(self.gpa, @intCast(id));
    }
}

/// Sixteen frames from the initial scene, then the checksum.
pub fn run(self: *BroadPhaseCase) !u64 {
    try self.reset();
    var h: u64 = 0;
    for (0..frames) |_| {
        try self.moveBodies();
        try self.updatePairs();
        try self.dropPartedPairs();
        h = hash.add(h, self.pair_index.count());
    }
    for (self.pairs.items) |pair| h = hash.add(h, if (pair.alive) pair.key else 0);
    for (self.body_proxies) |proxy| {
        const fat = self.broad_phase.fatAabb(proxy);
        h = hash.add(h, @as(u64, hash.f32Bits(fat.min.x)) | (@as(u64, hash.f32Bits(fat.max.y)) << 32));
    }
    return h;
}
