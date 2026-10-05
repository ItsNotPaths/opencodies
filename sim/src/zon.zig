// The dataset's files: a Car or a list of Material as ZON. Floats keep their bits.
const std = @import("std");

pub fn save(path: []const u8, value: anytype) !void {
    @setEvalBranchQuota(100_000);
    var out: std.Io.Writer.Allocating = .init(std.heap.page_allocator);
    defer out.deinit();
    try std.zon.stringify.serialize(value, .{}, &out.writer);
    try out.writer.writeByte('\n');
    try std.Io.Dir.cwd().writeFile(io(), .{ .sub_path = path, .data = out.written() });
}

// free the result with std.zon.parse.free
pub fn load(T: type, gpa: std.mem.Allocator, path: []const u8) !T {
    const source = try std.Io.Dir.cwd().readFileAllocOptions(io(), path, gpa, .unlimited, .of(u8), 0);
    defer gpa.free(source);
    return parse(T, gpa, source);
}

pub fn parse(T: type, gpa: std.mem.Allocator, source: [:0]const u8) !T {
    @setEvalBranchQuota(100_000);
    return std.zon.parse.fromSliceAlloc(T, gpa, source, null, .{});
}

fn io() std.Io {
    return std.Io.Threaded.global_single_threaded.io();
}
