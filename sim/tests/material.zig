const std = @import("std");
const material = @import("material");
const Material = material.Material;
const named = material.named;
const bare = material.bare;

fn plain(origin: []const u8, code: [4]u8, grip: ?f32) Material {
    return .{ .name = named(origin, code), .depth = 0, .grip = grip, .bumps = material.smooth(.by_time), .slowdown = .{} };
}

test "a name holds its origin and code" {
    const m = plain("abc", "GLD+".*, null);
    try std.testing.expectEqualSlices(u8, "abc-GLD+", &m.name);
    try std.testing.expectEqual("GLD+".*, m.code());
}

test "a level material borrows what it lacks from the car's list" {
    const list: [3]Material = .{ plain("abc", "DEFA".*, null), plain("abc", "GLD*".*, 1.3), plain("abc", "TSD*".*, 1.6) };
    var gld = bare("xyz", "GLD!".*);
    gld.depth = 0.02;
    try std.testing.expectEqual(.{ 1, material.How.family }, material.match(gld, &list).?);
    gld.name = named("xyz", "GVO+".*);
    gld.tyre_class = @bitCast(@as([4]u8, "GVLD".*));
    try std.testing.expectEqual(.{ 1, material.How.table }, material.match(gld, &list).?);
    try std.testing.expectEqual(.{ 0, material.How.default }, material.match(bare("xyz", "XXXX".*), &list).?);

    const out, const taken = material.borrow(gld, list[1]);
    try std.testing.expectEqual(@as(f32, 0.02), out.depth);
    try std.testing.expectEqual(@as(?f32, 1.3), out.grip);
    try std.testing.expect(out.bumps == .by_time and taken.contains(.bumps) and taken.contains(.grip) and !taken.contains(.tyre_class));
    try std.testing.expect(bare("xyz", "GLD+".*).isBare() and !gld.isBare());
}
