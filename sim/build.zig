const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{ .preferred_optimize_mode = .ReleaseFast });

    const mod = b.createModule(.{
        .root_source_file = b.path("src/sim.zig"),
        .target = target,
        .optimize = optimize,
        .pic = true,
    });
    generic(b, mod);

    // static for the Odin demo, shared for Python ctypes
    const static = b.addLibrary(.{ .linkage = .static, .name = "sim", .root_module = mod });
    static.bundle_compiler_rt = true;
    b.installArtifact(static);
    b.installArtifact(b.addLibrary(.{ .linkage = .dynamic, .name = "sim", .root_module = mod }));

    const test_step = b.step("test", "run sim tests");
    test_step.dependOn(&b.addRunArtifact(b.addTest(.{ .root_module = mod })).step);

    // tests/<name>.zig imports src/<name>.zig as <name>, the made-up cars and surfaces as "data"
    const data = b.createModule(.{ .root_source_file = b.path("data/data.zig") });
    for ([_][]const u8{ "d3", "fh", "dr1", "material" }) |name| {
        const root = b.createModule(.{ .root_source_file = b.path(b.fmt("src/{s}.zig", .{name})), .target = target, .optimize = optimize });
        generic(b, root);
        const tests = b.addTest(.{ .root_module = b.createModule(.{
            .root_source_file = b.path(b.fmt("tests/{s}.zig", .{name})),
            .target = target,
            .optimize = optimize,
            .imports = &.{ .{ .name = name, .module = root }, .{ .name = "data", .module = data } },
        }) });
        test_step.dependOn(&b.addRunArtifact(tests).step);
    }
}

// sim_create's car (sim.zig)
fn generic(b: *std.Build, m: *std.Build.Module) void {
    m.addAnonymousImport("generic_car", .{ .root_source_file = b.path("data/cars/hatch.zon") });
}
