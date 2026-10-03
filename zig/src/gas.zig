//! Particle gas: 4 substeps of the engine's gas solver over 8192 particles
//! stirred by 4 moving boxes. Each substep hashes particles into cells,
//! sorts the keys, searches the 27 neighbouring cells with a lower bound for
//! pressure and viscosity, pushes particles out of and along the movers,
//! then integrates positions.
const std = @import("std");
const hash = @import("hash.zig");
const vm = @import("vecmath.zig");

const Vec3 = vm.Vec3;
const Allocator = std.mem.Allocator;

const Gas = @This();

pub const name = "gas";

const particle_count: u32 = 8192;
const mover_count: u32 = 4;
const substeps: usize = 4;

const golden: u32 = 0x9E3779B9;
const gas_key_offset: i64 = 1 << 20;
const gas_key_mask: u64 = (1 << 21) - 1;
const mover_reach: f32 = 1.5;
const mover_wake_rate: f32 = 6.0;
const spacing: f32 = 0.8;
const pressure: f32 = 1.5;
const viscosity: f32 = 0.5;
const stir_strength: f32 = 1.0;
const step: f32 = @as(f32, 1.0) / @as(f32, 120.0);
const rise: f32 = 0.5;
const extent: f32 = 12.0;

pub const Particle = struct {
    position: Vec3 = .{},
    velocity: Vec3 = .{},
    size: f32 = 0.0,
    seed: u32 = 0,
};

pub const Mover = struct {
    transform: vm.Transform = .{},
    half_extents: Vec3 = .{},
    velocity: Vec3 = .{},
    angular_velocity: Vec3 = .{},
};

/// A particle index keyed by its spatial hash cell.
pub const Entry = struct {
    key: u64 = 0,
    index: u32 = 0,

    /// Orders entries by key, then index.
    fn lessThan(_: void, a: Entry, b: Entry) bool {
        return a.key < b.key or (a.key == b.key and a.index < b.index);
    }
};

const Cell = [3]i64;

gpa: Allocator,
initial: []Particle,
particles: []Particle,
movers: [mover_count]Mover,
entries: std.ArrayList(Entry),
push: []Vec3,
blend: []Vec3,
weight: []f32,

/// The engine's 32-bit bit mixer.
fn mixBits(input: u32) u32 {
    var value = input;
    value ^= value >> 16;
    value *%= 0x7FEB352D;
    value ^= value >> 15;
    value *%= 0x846CA68B;
    value ^= value >> 16;
    return value;
}

/// A deterministic value in [0, 1) from `seed` and `salt`.
fn unitRandom(seed: u32, salt: u32) f32 {
    return @as(f32, @floatFromInt(mixBits(seed ^ mixBits(salt +% golden)) >> 8)) / 16777216.0;
}

/// Unit vector along `value`, or `fallback` when it is too short.
fn safeNormalize(value: Vec3, fallback: Vec3) Vec3 {
    const size = value.length();
    return if (size > 1e-6) value.div(size) else fallback;
}

/// One cell coordinate offset into 21 bits.
fn packCoordinate(value: i64) u64 {
    return @as(u64, @bitCast(value + gas_key_offset)) & gas_key_mask;
}

/// Packs three cell coordinates into one 63-bit key.
fn gasKey(x: i64, y: i64, z: i64) u64 {
    return packCoordinate(x) | (packCoordinate(y) << 21) | (packCoordinate(z) << 42);
}

/// Integer cell containing `position` for cell size `reach`.
fn gasCell(position: Vec3, reach: f32) Cell {
    return .{
        @intFromFloat(@floor(position.x / reach)),
        @intFromFloat(@floor(position.y / reach)),
        @intFromFloat(@floor(position.z / reach)),
    };
}

/// Unit direction pushing `self` away from `other`; a seeded random one when
/// they coincide.
fn separation(self: *const Particle, other: *const Particle, offset: Vec3) Vec3 {
    const size = offset.length();
    if (size > 1e-6) return offset.div(size);
    const mixed = mixBits(self.seed ^ mixBits(other.seed));
    const x = unitRandom(mixed, 1) - 0.5;
    const y = unitRandom(mixed, 2) - 0.5;
    const z = unitRandom(mixed, 3) - 0.5;
    return safeNormalize(Vec3.init(x, y, z), Vec3.init(0.0, 1.0, 0.0));
}

/// -1 for a negative value, otherwise 1.
fn signOf(value: f32) f32 {
    return if (value < 0.0) -1.0 else 1.0;
}

/// The face coordinate along a normal axis, or the local coordinate off it.
fn faceCoordinate(normal: f32, half_extent: f32, local: f32) f32 {
    return if (normal != 0.0) normal * half_extent else local;
}

/// Pushes a particle with a moving box, adding surface velocity and moving it
/// out when it penetrates.
fn stir(body: *Particle, mover: *const Mover) void {
    const rotation: vm.Basis = .{
        .x = safeNormalize(mover.transform.basis.x, Vec3.init(1.0, 0.0, 0.0)),
        .y = safeNormalize(mover.transform.basis.y, Vec3.init(0.0, 1.0, 0.0)),
        .z = safeNormalize(mover.transform.basis.z, Vec3.init(0.0, 0.0, 1.0)),
    };
    const local = rotation.transposed().apply(body.position.sub(mover.transform.origin));
    const half = mover.half_extents;
    const closest = Vec3.init(vm.clampf(local.x, -half.x, half.x), vm.clampf(local.y, -half.y, half.y), vm.clampf(local.z, -half.z, half.z));
    const radius = body.size * 0.5;
    const reach = vm.max3(half.x, half.y, half.z) * mover_reach + radius;
    const outside = local.sub(closest);
    const distance = outside.length();
    if (distance > reach) return;
    const arm = rotation.apply(closest);
    const surface_velocity = mover.velocity.add(mover.angular_velocity.cross(arm));
    const weight = 1.0 - (distance / reach);
    const wake = vm.clampf(stir_strength * weight * step * mover_wake_rate, 0.0, 1.0);
    body.velocity = body.velocity.add(surface_velocity.sub(body.velocity).scale(wake));
    if (distance >= radius) return;
    var normal: Vec3 = .{};
    if (distance > 1e-6) {
        normal = outside.div(distance);
    } else {
        const depth = half.sub(local.abs());
        if (depth.x <= depth.y and depth.x <= depth.z) {
            normal = Vec3.init(signOf(local.x), 0.0, 0.0);
        } else if (depth.y <= depth.z) {
            normal = Vec3.init(0.0, signOf(local.y), 0.0);
        } else {
            normal = Vec3.init(0.0, 0.0, signOf(local.z));
        }
    }
    const face = if (distance > 1e-6)
        closest
    else
        Vec3.init(faceCoordinate(normal.x, half.x, local.x), faceCoordinate(normal.y, half.y, local.y), faceCoordinate(normal.z, half.z, local.z));
    const world_normal = rotation.apply(normal);
    body.position = mover.transform.origin.add(rotation.apply(face)).add(world_normal.scale(radius));
    const into = body.velocity.sub(surface_velocity).dot(world_normal);
    if (into < 0.0) body.velocity = body.velocity.sub(world_normal.scale(into * stir_strength));
}

/// Draws the particles and the movers.
pub fn init(gpa: Allocator) !Gas {
    var rng: hash.Rng = .{ .s = 0x6a5 };
    const initial = try gpa.alloc(Particle, particle_count);
    for (initial) |*p| {
        p.position = Vec3.random(&rng, 0.0, extent);
        p.velocity = Vec3.random(&rng, -0.5, 0.5);
        p.size = 0.3 + rng.unit() * 0.3;
        p.seed = @truncate(rng.next());
    }
    var movers: [mover_count]Mover = undefined;
    for (&movers) |*m| {
        m.transform.basis = vm.Basis.fromQuat(vm.Quat.random(&rng));
        m.transform.origin = Vec3.random(&rng, 4.0, extent - 4.0);
        m.half_extents = Vec3.random(&rng, 1.0, 2.0);
        m.velocity = Vec3.random(&rng, -3.0, 3.0);
        m.angular_velocity = Vec3.random(&rng, -1.0, 1.0);
    }
    return .{
        .gpa = gpa,
        .initial = initial,
        .particles = try gpa.alloc(Particle, particle_count),
        .movers = movers,
        .entries = try .initCapacity(gpa, particle_count),
        .push = try gpa.alloc(Vec3, particle_count),
        .blend = try gpa.alloc(Vec3, particle_count),
        .weight = try gpa.alloc(f32, particle_count),
    };
}

/// Frees the particles and the scratch buffers.
pub fn deinit(self: *Gas, gpa: Allocator) void {
    gpa.free(self.initial);
    gpa.free(self.particles);
    self.entries.deinit(gpa);
    gpa.free(self.push);
    gpa.free(self.blend);
    gpa.free(self.weight);
}

/// Index of the first entry whose key is not below `key`.
fn lowerBound(entries: []const Entry, key: u64) usize {
    var low: usize = 0;
    var high: usize = entries.len;
    while (low < high) {
        const middle = low + (high - low) / 2;
        if (entries[middle].key < key) {
            low = middle + 1;
        } else {
            high = middle;
        }
    }
    return low;
}

/// Pairwise push, velocity blend and weight from overlapping particles.
fn accumulatePressure(self: *Gas) Allocator.Error!void {
    var reach: f32 = 0.0;
    for (self.particles) |body| reach = vm.maxf(reach, body.size * spacing);
    if (reach <= 0.0) return;
    self.entries.clearRetainingCapacity();
    for (self.particles, 0..) |particle, index| {
        const cell = gasCell(particle.position, reach);
        try self.entries.append(self.gpa, .{ .key = gasKey(cell[0], cell[1], cell[2]), .index = @intCast(index) });
    }
    const entries = self.entries.items;
    std.mem.sortUnstable(Entry, entries, {}, Entry.lessThan);
    for (entries) |entry| {
        const me = &self.particles[entry.index];
        const cell = gasCell(me.position, reach);
        var dz: i64 = -1;
        while (dz <= 1) : (dz += 1) {
            var dy: i64 = -1;
            while (dy <= 1) : (dy += 1) {
                var dx: i64 = -1;
                while (dx <= 1) : (dx += 1) {
                    const key = gasKey(cell[0] + dx, cell[1] + dy, cell[2] + dz);
                    for (entries[lowerBound(entries, key)..]) |next| {
                        if (next.key != key) break;
                        if (next.index == entry.index) continue;
                        const other = &self.particles[next.index];
                        const offset = me.position.sub(other.position);
                        const range = 0.5 * (me.size + other.size) * spacing;
                        const distance = offset.length();
                        if (range <= 0.0 or distance >= range) continue;
                        const overlap = 1.0 - (distance / range);
                        self.push[entry.index] = self.push[entry.index].add(separation(me, other, offset).scale(overlap));
                        self.blend[entry.index] = self.blend[entry.index].add(other.velocity.sub(me.velocity).scale(overlap));
                        self.weight[entry.index] += overlap;
                    }
                }
            }
        }
    }
}

/// Pressure, viscosity and stirring for every particle.
fn resolve(self: *Gas) Allocator.Error!void {
    @memset(self.push, .{});
    @memset(self.blend, .{});
    @memset(self.weight, 0.0);
    try self.accumulatePressure();
    for (self.particles, self.push, self.blend, self.weight) |*body, push, blend, weight| {
        body.velocity = body.velocity.add(push.scale(pressure * step));
        if (weight > 0.0) {
            const blend_rate = vm.clampf(viscosity * step * 10.0, 0.0, 1.0);
            body.velocity = body.velocity.add(blend.scale(blend_rate / weight));
        }
        for (&self.movers) |*mover| stir(body, mover);
    }
}

/// Four substeps from the initial particles, then the checksum.
pub fn run(self: *Gas) !u64 {
    @memcpy(self.particles, self.initial);
    for (0..substeps) |_| {
        try self.resolve();
        for (self.particles) |*p| {
            p.velocity.y = p.velocity.y + rise * step;
            p.position = p.position.add(p.velocity.scale(step));
        }
    }
    var h: u64 = 0;
    for (self.particles) |p| {
        h = hash.add(h, @as(u64, hash.f32Bits(p.position.x)) | (@as(u64, hash.f32Bits(p.position.y)) << 32));
        h = hash.add(h, @as(u64, hash.f32Bits(p.velocity.z)) | (@as(u64, hash.f32Bits(p.position.z)) << 32));
    }
    return h;
}
