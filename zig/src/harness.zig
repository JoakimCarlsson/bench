//! Runs one kernel and prints its line of the output protocol.
const std = @import("std");

/// Initialise a `Case`, run it `warmup + reps` times, deinitialise it, and
/// print one line: `name min_ns median_ns checksum_hex`.
///
/// A `Case` declares `name`, `init(gpa) !Case` or `init(gpa, io) !Case`,
/// `run(*Case) u64` or `run(*Case) !u64`, and `deinit(*Case, gpa) void`.
/// `run` must be deterministic; a checksum that changes between repetitions
/// is `error.Nondeterministic`.
pub fn runCase(
    comptime Case: type,
    gpa: std.mem.Allocator,
    io: std.Io,
    reps: usize,
    warmup: usize,
) !void {
    var case = try initCase(Case, gpa, io);
    defer case.deinit(gpa);

    const times = try gpa.alloc(u64, reps);
    defer gpa.free(times);

    var checksum: u64 = 0;
    for (0..warmup + reps) |i| {
        const t0 = now(io);
        const sum = try runOnce(&case);
        const t1 = now(io);
        if (i == 0) {
            checksum = sum;
        } else if (sum != checksum) {
            std.debug.print("{s}: nondeterministic\n", .{Case.name});
            return error.Nondeterministic;
        }
        if (i >= warmup) times[i - warmup] = t1 - t0;
    }

    std.mem.sort(u64, times, {}, std.sort.asc(u64));
    var buf: [128]u8 = undefined;
    const line = try std.fmt.bufPrint(&buf, "{s} {d} {d} {x:0>16}\n", .{
        Case.name,
        times[0],
        times[reps / 2],
        checksum,
    });
    try std.Io.File.stdout().writeStreamingAll(io, line);
}

/// `Case.init(gpa, io)` for a case that runs threads, else `Case.init(gpa)`.
fn initCase(comptime Case: type, gpa: std.mem.Allocator, io: std.Io) !Case {
    if (@typeInfo(@TypeOf(Case.init)).@"fn".params.len == 2) return Case.init(gpa, io);
    return Case.init(gpa);
}

/// One call of `run`, which may return `u64` or an error union of it.
fn runOnce(case: anytype) !u64 {
    return case.run();
}

/// Monotonic nanoseconds from an unspecified origin.
fn now(io: std.Io) u64 {
    return @intCast(std.Io.Timestamp.now(io, .awake).nanoseconds);
}
