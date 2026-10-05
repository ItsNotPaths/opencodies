// track.jpk of D3 and DR1: the .vcqtc chunks to one triangle list; the quadtree nodes and leaf lists are not read
const std = @import("std");
const vec = @import("vec.zig");
const material = @import("../material.zig");
const Material = material.Material;
const mesh = @import("mesh.zig");

const V3 = vec.V3;
const Triangle = mesh.Triangle;
const cross = vec.cross;

// the 4-byte triangle record: first vertex, flags, material slot, then two vertex steps
pub const Packing = enum {
    d3, // 2 flag bits, 10-bit first vertex, 4-bit slot
    dr1, // 10-bit first vertex, 3 flag bits (edges [guess]), 3-bit slot

    fn decode(p: Packing, b: [4]u8) struct { first: usize, slot: u4 } {
        return switch (p) {
            .d3 => .{ .first = @as(usize, b[0] & 0x3f) << 4 | b[1] >> 4, .slot = @intCast(b[1] & 0xf) },
            .dr1 => .{ .first = @as(usize, b[0]) << 2 | b[1] >> 6, .slot = @intCast(b[1] & 0x7) },
        };
    }
};

// materials: the track's codes index them
pub fn load(gpa: std.mem.Allocator, jpk: []const u8, materials: []const Material, packing: Packing) !mesh.Mesh {
    if (materials.len == 0) return error.NoMaterials;
    var tris: std.ArrayList(Triangle) = .empty;
    defer tris.deinit(gpa);
    var it = try Entries.init(jpk);
    var index: u16 = 0;
    while (try it.next()) |e| {
        if (!std.mem.endsWith(u8, e.name, ".vcqtc")) continue;
        try chunk(gpa, e.data, materials, packing, index, &tris);
        index += 1;
    }
    return mesh.Mesh.init(gpa, tris.items, materials);
}

// header: box, -triangle count, -vertex count, code count, vertex and triangle offsets, 4-char codes at 52
fn chunk(gpa: std.mem.Allocator, data: []const u8, materials: []const Material, packing: Packing, index: u16, out: *std.ArrayList(Triangle)) !void {
    if (data.len < 52) return error.Truncated;
    const lo: V3 = .{ f32At(data, 0), f32At(data, 4), f32At(data, 8) };
    const hi: V3 = .{ f32At(data, 12), f32At(data, 16), f32At(data, 20) };
    const tri_count: usize = @intCast(-@as(i32, @bitCast(le32(data, 24))));
    const vert_count: usize = @intCast(-@as(i32, @bitCast(le32(data, 28))));
    const code_count: usize = @min(le32(data, 32), 16);
    const verts_at: usize = le32(data, 36);
    const tris_at: usize = le32(data, 44);
    if (verts_at + 8 * vert_count > data.len or tris_at + 4 * tri_count > data.len or 52 + 4 * code_count > data.len) return error.Truncated;

    const vertices = try gpa.alloc(V3, vert_count);
    defer gpa.free(vertices);
    const normal_vertices = try gpa.alloc(V3, vert_count);
    defer gpa.free(normal_vertices);
    const scale = (hi - lo) * V3{ 0x1p-24, 0x1p-16, 0x1p-24 };
    for (vertices, normal_vertices, 0..) |*v, *n, k| {
        const b = data[verts_at + 8 * k ..][0..8];
        const q: V3 = .{
            @floatFromInt(std.mem.readInt(u24, b[0..3], .big)),
            @floatFromInt(std.mem.readInt(u16, b[3..5], .big)),
            @floatFromInt(std.mem.readInt(u24, b[5..8], .big)),
        };
        v.* = q * (hi - lo) * V3{ 0x1p-24, 0x1p-16, 0x1p-24 } + lo; // 0x870a90, for the hit test
        n.* = q * scale + lo; // 0x86cde0, for the face normal
    }

    var slots: [16]u16 = undefined;
    for (&slots, 0..) |*m, k| {
        const code = if (k < code_count) data[52 + 4 * k ..][0..4].* else "BARR".*;
        m.* = @intCast(material.find(materials, code) orelse 0); // unknown: the first record, BARR
    }

    for (0..tri_count) |k| {
        const b = data[tris_at + 4 * k ..][0..4];
        const t = packing.decode(b.*);
        const at = [3]usize{ t.first, t.first + b[2], t.first + b[3] };
        for (at) |i| if (i >= vert_count) return error.BadTriangle;
        const n = normal_vertices;
        try out.append(gpa, .{
            .corners = .{ vertices[at[0]], vertices[at[1]], vertices[at[2]] },
            .normal = vec.normalize(cross(n[at[1]] - n[at[0]], n[at[2]] - n[at[0]])),
            .material = slots[t.slot],
            .chunk = index,
        });
    }
}

fn le32(data: []const u8, at: usize) u32 {
    return std.mem.readInt(u32, data[at..][0..4], .little);
}

fn f32At(data: []const u8, at: usize) f32 {
    return @bitCast(le32(data, at));
}

// JPAK: entry count at 8, then 32-byte records of name offset, size, data offset from 32
const Entries = struct {
    data: []const u8,
    count: u32,
    k: u32 = 0,

    const Entry = struct { name: []const u8, data: []const u8 };

    fn init(data: []const u8) !Entries {
        if (data.len < 32) return error.Truncated;
        return .{ .data = data, .count = le32(data, 8) };
    }

    fn next(e: *Entries) !?Entry {
        if (e.k == e.count) return null;
        const at = 32 + 32 * @as(usize, e.k);
        e.k += 1;
        if (at + 12 > e.data.len) return error.Truncated;
        const name_at, const size, const start = .{ le32(e.data, at), le32(e.data, at + 4), le32(e.data, at + 8) };
        const end = std.mem.indexOfScalarPos(u8, e.data, name_at, 0) orelse return error.Truncated;
        if (start + size > e.data.len) return error.Truncated;
        return .{ .name = e.data[name_at..end], .data = e.data[start..][0..size] };
    }
};
