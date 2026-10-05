const std = @import("std");

// The track codes of a DR1 surface_materials.xml: the codes in file order, the materials come from elsewhere.
// \0BXML: an element is u32 size, attribute count, name, the attribute pairs and one byte (0 or 1);
// children follow, then the end mark 04 05 00 00 00 00 (04 06 .. closes the file)
pub fn codes(gpa: std.mem.Allocator, bxml: []const u8) ![][4]u8 {
    if (!std.mem.startsWith(u8, bxml, "\x00BXML\x00")) return error.NotBxml;
    var r: Reader = .{ .data = bxml, .at = 6 };
    var out: std.ArrayList([4]u8) = .empty;
    errdefer out.deinit(gpa);
    while (r.at < bxml.len) {
        if (r.endMark()) continue;
        const e = try r.element();
        if (!std.mem.eql(u8, e.name, "MATERIAL")) continue;
        const code = e.get("name") orelse return error.NoCode;
        if (code.len != 4) return error.BadCode;
        try out.append(gpa, code[0..4].*);
    }
    return out.toOwnedSlice(gpa);
}

const Element = struct {
    name: []const u8,
    attributes: []const u8, // count pairs of NUL-terminated strings
    count: u8,

    fn get(e: Element, key: []const u8) ?[]const u8 {
        var it = std.mem.splitScalar(u8, e.attributes, 0);
        for (0..e.count) |_| {
            const k = it.next() orelse return null;
            const v = it.next() orelse return null;
            if (std.mem.eql(u8, k, key)) return v;
        }
        return null;
    }
};

const Reader = struct {
    data: []const u8,
    at: usize,

    fn endMark(r: *Reader) bool {
        const rest = r.data[r.at..];
        if (rest.len < 2 or rest[0] != 4 or (rest[1] != 5 and rest[1] != 6)) return false;
        r.at += @min(rest.len, 6);
        return true;
    }

    fn element(r: *Reader) !Element {
        if (r.at + 5 > r.data.len) return error.Truncated;
        r.at += 4; // size
        const count = r.data[r.at];
        r.at += 1;
        const name = try r.string();
        const start = r.at;
        for (0..2 * @as(usize, count)) |_| _ = try r.string();
        const attributes = r.data[start..r.at];
        if (r.at >= r.data.len) return error.Truncated;
        r.at += 1;
        return .{ .name = name, .attributes = attributes, .count = count };
    }

    fn string(r: *Reader) ![]const u8 {
        const end = std.mem.indexOfScalarPos(u8, r.data, r.at, 0) orelse return error.Truncated;
        defer r.at = end + 1;
        return r.data[r.at..end];
    }
};

test "a two-material file" {
    const bxml = "\x00BXML\x00" ++
        "\x0e\x00\x00\x00\x00MATERIALS\x00\x00" ++
        "\x00\x00\x00\x00\x01MATERIAL\x00name\x00DRT*\x00\x00" ++
        "\x00\x00\x00\x00\x03MECHANICS\x00grip\x001.800\x00depth\x000.01\x00bumpsrealproportion\x002\x00\x00" ++
        "\x04\x05\x00\x00\x00\x00\x04\x05\x00\x00\x00\x00" ++
        "\x00\x00\x00\x00\x01MATERIAL\x00name\x00TSD*\x00\x00\x04\x05\x00\x00\x00\x00\x04\x05\x00\x00\x00\x00";
    const cs = try codes(std.testing.allocator, bxml);
    defer std.testing.allocator.free(cs);
    try std.testing.expectEqualSlices([4]u8, &.{ "DRT*".*, "TSD*".* }, cs);
}
