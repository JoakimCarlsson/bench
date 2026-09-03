//! Builds `bench_zig`. ReleaseFast by default, the native CPU by default,
//! mirroring the `-O3 -march=native` the C and C++ builds use.
const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.option(
        std.builtin.OptimizeMode,
        "optimize",
        "Prioritize performance, safety, or binary size (default: ReleaseFast)",
    ) orelse .ReleaseFast;

    const exe = b.addExecutable(.{
        .name = "bench_zig",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });
    b.installArtifact(exe);

    const run_cmd = b.addRunArtifact(exe);
    run_cmd.step.dependOn(b.getInstallStep());
    if (b.args) |args| run_cmd.addArgs(args);
    b.step("run", "Run the Zig benchmark").dependOn(&run_cmd.step);
}
