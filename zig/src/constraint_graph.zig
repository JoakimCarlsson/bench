//! The engine's constraint graph: colours contacts so that no two in a colour
//! share a dynamic body, keeping static contacts out of colour 0 and sending
//! what does not fit to the overflow colour.
const std = @import("std");
const null_link = @import("contact.zig").null_link;

const Allocator = std.mem.Allocator;

const ConstraintGraph = @This();

pub const graph_color_count: u32 = 24;
pub const overflow_color: u32 = graph_color_count - 1;
pub const dynamic_color_count: u32 = graph_color_count - 4;

pub const GraphBodies = struct {
    a: u32 = 0,
    b: u32 = 0,
    b_is_static: bool = false,
};

pub const GraphSlot = struct {
    color: u32 = 0,
    local: u32 = 0,
};

const Color = struct {
    bodies: std.ArrayList(u64) = .empty,
    contacts: std.ArrayList(u32) = .empty,

    /// Whether `body` is not yet in the colour.
    fn isFree(self: *const Color, body: u32) bool {
        return (self.bodies.items[body / 64] & (@as(u64, 1) << @intCast(body % 64))) == 0;
    }

    /// Marks `body` as in the colour.
    fn take(self: *Color, body: u32) void {
        self.bodies.items[body / 64] |= @as(u64, 1) << @intCast(body % 64);
    }

    /// Clears `body` from the colour.
    fn release(self: *Color, body: u32) void {
        self.bodies.items[body / 64] &= ~(@as(u64, 1) << @intCast(body % 64));
    }
};

colors: [graph_color_count]Color = @splat(.{}),
words: usize = 0,
len: usize = 0,

/// Frees every colour's bitset and contact list.
pub fn deinit(self: *ConstraintGraph, gpa: Allocator) void {
    for (&self.colors) |*color| {
        color.bodies.deinit(gpa);
        color.contacts.deinit(gpa);
    }
    self.* = .{};
}

/// Grows every colour's body bitset to cover `body_count` bodies.
pub fn reserveBodies(self: *ConstraintGraph, gpa: Allocator, body_count: usize) Allocator.Error!void {
    const words = (body_count + 63) / 64;
    if (words <= self.words) return;
    self.words = words;
    for (self.colors[0..overflow_color]) |*color| try color.bodies.appendNTimes(gpa, 0, words - color.bodies.items.len);
}

/// Adds a contact to the first colour free for its bodies.
pub fn add(self: *ConstraintGraph, gpa: Allocator, contact: u32, bodies: GraphBodies) Allocator.Error!GraphSlot {
    var chosen = overflow_color;
    if (bodies.b_is_static) {
        var color = overflow_color - 1;
        while (color >= 1) : (color -= 1) {
            if (self.colors[color].isFree(bodies.a)) {
                self.colors[color].take(bodies.a);
                chosen = color;
                break;
            }
        }
    } else {
        for (self.colors[0..dynamic_color_count], 0..) |*color, index| {
            if (color.isFree(bodies.a) and color.isFree(bodies.b)) {
                color.take(bodies.a);
                color.take(bodies.b);
                chosen = @intCast(index);
                break;
            }
        }
    }
    const target = &self.colors[chosen];
    const local: u32 = @intCast(target.contacts.items.len);
    try target.contacts.append(gpa, contact);
    self.len += 1;
    return .{ .color = chosen, .local = local };
}

/// Removes a contact; returns the contact moved into its place, or null_link.
pub fn remove(self: *ConstraintGraph, slot: GraphSlot, bodies: GraphBodies) u32 {
    const target = &self.colors[slot.color];
    if (slot.color != overflow_color) {
        target.release(bodies.a);
        if (!bodies.b_is_static) target.release(bodies.b);
    }
    const last = target.contacts.items[target.contacts.items.len - 1];
    var moved = null_link;
    if (slot.local + 1 != target.contacts.items.len) {
        target.contacts.items[slot.local] = last;
        moved = last;
    }
    _ = target.contacts.pop();
    self.len -= 1;
    return moved;
}

/// Contacts of one colour.
pub fn contacts(self: *const ConstraintGraph, color: u32) []const u32 {
    return self.colors[color].contacts.items;
}

/// Number of contacts in the graph.
pub fn size(self: *const ConstraintGraph) usize {
    return self.len;
}
