//! Generational slot map: 4M random insert, remove and lookup operations
//! over 65536 slots, with a stale-handle lookup on every hit. Branchy
//! integer code over a free list, the shape of an entity or body registry.
const std = @import("std");
const hash = @import("hash.zig");

const SlotMap = @This();

pub const name = "slotmap";

const cap: u32 = 65536;
const ops: usize = 4_000_000;

pub const Slot = struct { gen: u32, next: u32, value: u64 };

slots: []Slot,
handles: []u64,
head: u32 = 0,
count: u32 = 0,

pub fn init(gpa: std.mem.Allocator) !SlotMap {
    const slots = try gpa.alloc(Slot, cap);
    errdefer gpa.free(slots);
    const handles = try gpa.alloc(u64, cap);
    return .{ .slots = slots, .handles = handles };
}

pub fn deinit(self: *SlotMap, gpa: std.mem.Allocator) void {
    gpa.free(self.slots);
    gpa.free(self.handles);
}

fn reset(self: *SlotMap) void {
    for (self.slots, 0..) |*s, i| s.* = .{ .gen = 1, .next = @intCast(i + 1), .value = 0 };
    self.head = 0;
    self.count = 0;
}

fn insert(self: *SlotMap, value: u64) void {
    if (self.count >= cap) return;
    const idx = self.head;
    self.head = self.slots[idx].next;
    self.slots[idx].value = value;
    self.handles[self.count] = (@as(u64, self.slots[idx].gen) << 32) | idx;
    self.count += 1;
}

fn removeAt(self: *SlotMap, k: u32) void {
    const hd = self.handles[k];
    self.count -= 1;
    self.handles[k] = self.handles[self.count];
    const idx: u32 = @truncate(hd);
    self.slots[idx].gen +%= 1;
    self.slots[idx].next = self.head;
    self.head = idx;
}

fn lookup(self: *const SlotMap, hd: u64) ?u64 {
    const idx: u32 = @truncate(hd);
    if (self.slots[idx].gen != @as(u32, @truncate(hd >> 32))) return null;
    return self.slots[idx].value;
}

pub fn run(self: *SlotMap) u64 {
    self.reset();
    var sum: u64 = 0;
    var misses: u64 = 0;
    var rng: hash.Rng = .{ .s = 0x510 };
    for (0..ops) |_| {
        const r = rng.next();
        switch (r & 3) {
            0, 1 => self.insert(r),
            2 => if (self.count > 0) self.removeAt(@intCast((r >> 2) % self.count)),
            else => if (self.count > 0) {
                const hd = self.handles[(r >> 2) % self.count];
                if (self.lookup(hd)) |v| sum +%= v;
                if (self.lookup(hd ^ (@as(u64, 1) << 32))) |v| sum +%= v else misses += 1;
            },
        }
    }
    return hash.add(hash.add(sum, misses), self.count);
}
