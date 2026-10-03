//! Entry point: `bench_zig [reps] [warmup] [kernel...]`, one output line per kernel.
const std = @import("std");
const harness = @import("harness.zig");

const cases = .{
    @import("dda.zig"),
    @import("flood.zig"),
    @import("surface.zig"),
    @import("mips.zig"),
    @import("mass.zig"),
    @import("sweep.zig"),
    @import("bvh.zig"),
    @import("islands.zig"),
    @import("colour.zig"),
    @import("solve.zig"),
    @import("integrate.zig"),
    @import("transform.zig"),
    @import("unproject.zig"),
    @import("decompose.zig"),
    @import("raycast.zig"),
    @import("boxbox.zig"),
    @import("wide.zig"),
    @import("broadphase.zig"),
    @import("gas.zig"),
    @import("world.zig").WorldCase(1),
    @import("world.zig").WorldCase(2),
    @import("world.zig").WorldCase(4),
    @import("world.zig").WorldCase(8),
    @import("world.zig").WorldCase(16),
    @import("particles.zig"),
    @import("json.zig"),
    @import("anim.zig"),
    @import("ui.zig"),
    @import("slotmap.zig"),
    @import("chunkmap.zig"),
    @import("sort.zig"),
};

/// Parses `[reps] [warmup] [kernel...]` and runs the selected kernels in order.
pub fn main(init: std.process.Init.Minimal) !u8 {
    const gpa = std.heap.smp_allocator;

    var threaded: std.Io.Threaded = .init(gpa, .{
        .environ = init.environ,
        .argv0 = .init(init.args),
    });
    defer threaded.deinit();
    const io = threaded.io();

    var it = try init.args.iterateAllocator(gpa);
    defer it.deinit();
    _ = it.next();
    const reps = try parseOr(it.next(), 10);
    const warmup = try parseOr(it.next(), 2);
    if (reps < 1 or reps > 1024) {
        std.debug.print("reps must be 1..1024\n", .{});
        return 1;
    }

    var names: std.ArrayList([]const u8) = .empty;
    defer names.deinit(gpa);
    while (it.next()) |name| try names.append(gpa, name);

    inline for (cases) |Case| {
        if (selected(Case.name, names.items)) try harness.runCase(Case, gpa, io, reps, warmup);
    }
    return 0;
}

/// Whether the kernel `name` was asked for: no names given runs everything.
fn selected(name: []const u8, names: []const []const u8) bool {
    if (names.len == 0) return true;
    for (names) |wanted| {
        if (std.mem.eql(u8, wanted, name)) return true;
    }
    return false;
}

/// `arg` as a decimal count, or `default` when absent.
fn parseOr(arg: ?[]const u8, default: usize) !usize {
    const text = arg orelse return default;
    return std.fmt.parseInt(usize, text, 10);
}
