//! Entry point: `bench_zig [reps] [warmup]`, one output line per kernel.
const std = @import("std");
const harness = @import("harness.zig");

const cases = .{
    @import("dda.zig"),
    @import("flood.zig"),
    @import("surface.zig"),
    @import("sweep.zig"),
    @import("solve.zig"),
    @import("integrate.zig"),
    @import("slotmap.zig"),
    @import("chunkmap.zig"),
    @import("sort.zig"),
};

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

    inline for (cases) |Case| try harness.runCase(Case, gpa, io, reps, warmup);
    return 0;
}

fn parseOr(arg: ?[]const u8, default: usize) !usize {
    const text = arg orelse return default;
    return std.fmt.parseInt(usize, text, 10);
}
