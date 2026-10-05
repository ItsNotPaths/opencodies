const std = @import("std");
const vec = @import("vec.zig");
const Material = @import("../material.zig").Material;

const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;
const cross = vec.cross;

pub const Hit = extern struct { t: f32, normal: V3, material: *const Material }; // t along from -> to

pub const Triangle = struct {
    corners: [3]V3,
    normal: V3,
    material: u16, // into Mesh.materials
    chunk: u16 = 0, // the track.jpk chunk; the chassis contact keeps one contact per chunk

    pub fn overlaps(f: Triangle, lo: V3, hi: V3) bool {
        const v = f.corners;
        return !@reduce(.Or, @min(v[0], v[1], v[2]) > hi) and !@reduce(.Or, @max(v[0], v[1], v[2]) < lo);
    }

    pub fn fromCorners(corners: [3]V3, material: u16) Triangle {
        const v = corners;
        return .{ .corners = v, .normal = vec.normalize(cross(v[1] - v[0], v[2] - v[0])), .material = material };
    }
};

const cell_size = 8; // m, xz
const pad = 1e-3; // m, the hit test reaches a little past the edges

const Cell = [2]i32; // x, z
const Range = struct { start: u32, end: u32 };

// a triangle soup with its surfaces, binned on an xz grid
pub const Mesh = struct {
    arena: std.heap.ArenaAllocator,
    triangles: []Triangle,
    materials: []Material, // by material index, as the car's step reads them (sim.zig Sim.fitLevel)
    cells: std.AutoHashMapUnmanaged(Cell, Range) = .empty,
    refs: []u32 = &.{}, // triangle indices, grouped by cell

    pub fn init(gpa: std.mem.Allocator, triangles: []const Triangle, materials: []const Material) !Mesh {
        var m: Mesh = .{ .arena = .init(gpa), .triangles = &.{}, .materials = &.{} };
        errdefer m.arena.deinit();
        const a = m.arena.allocator();
        m.triangles = try a.dupe(Triangle, triangles);
        m.materials = try a.dupe(Material, materials);
        try m.bin(gpa);
        return m;
    }

    pub fn deinit(m: *Mesh) void {
        m.arena.deinit();
    }

    fn bin(m: *Mesh, gpa: std.mem.Allocator) !void {
        const Ref = struct { cell: Cell, tri: u32 };
        var refs: std.ArrayList(Ref) = .empty;
        defer refs.deinit(gpa);
        for (m.triangles, 0..) |t, i| {
            var cells = Cells.init(cellSpan(t));
            while (cells.next()) |c| try refs.append(gpa, .{ .cell = c, .tri = @intCast(i) });
        }
        std.mem.sort(Ref, refs.items, {}, struct {
            fn less(_: void, p: Ref, q: Ref) bool {
                if (p.cell[1] != q.cell[1]) return p.cell[1] < q.cell[1];
                if (p.cell[0] != q.cell[0]) return p.cell[0] < q.cell[0];
                return p.tri < q.tri;
            }
        }.less);
        const a = m.arena.allocator();
        m.refs = try a.alloc(u32, refs.items.len);
        var start: u32 = 0;
        for (refs.items, m.refs, 0..) |r, *out, k| {
            out.* = r.tri;
            const last = k + 1 == refs.items.len or !std.meta.eql(refs.items[k + 1].cell, r.cell);
            if (!last) continue;
            try m.cells.put(a, r.cell, .{ .start = start, .end = @intCast(k + 1) });
            start = @intCast(k + 1);
        }
    }

    fn inCell(m: Mesh, c: Cell) []const u32 {
        const r = m.cells.get(c) orelse return &.{};
        return m.refs[r.start..r.end];
    }

    // 0x7e9340 without the quadtree: the nearest hit over the whole ray; the lower triangle index wins a tie
    pub fn raycast(m: Mesh, from: V3, to: V3) ?Hit {
        const ray = to - from;
        var best: ?struct { t: f32, tri: u32 } = null;
        var cells = Cells.init(.{ cellOf(@min(from, to)), cellOf(@max(from, to)) });
        while (cells.next()) |c| for (m.inCell(c)) |i| {
            const t = hitFraction(from, ray, m.triangles[i].corners) orelse continue;
            if (best == null or t < best.?.t or (t == best.?.t and i < best.?.tri)) best = .{ .t = t, .tri = i };
        };
        const b = best orelse return null;
        const tri = m.triangles[b.tri];
        return .{ .t = b.t, .normal = tri.normal, .material = &m.materials[tri.material] };
    }

    // the triangles whose box meets lo..hi, each once, by cell and then index
    pub fn faces(m: Mesh, lo: V3, hi: V3, out: []Triangle) []Triangle {
        const first = cellOf(lo);
        var cells = Cells.init(.{ first, cellOf(hi) });
        var count: usize = 0;
        while (cells.next()) |c| for (m.inCell(c)) |i| {
            const t = m.triangles[i];
            const tlo = cellSpan(t)[0];
            if (@max(tlo[0], first[0]) != c[0] or @max(tlo[1], first[1]) != c[1]) continue; // listed in an earlier cell
            if (!t.overlaps(lo, hi)) continue;
            if (count == out.len) return out;
            out[count] = t;
            count += 1;
        };
        return out[0..count];
    }

    // a flat 100 m TSD* square around the origin, for tests
    pub fn testQuad(gpa: std.mem.Allocator) !Mesh {
        const materials = [_]Material{@import("ground.zig").plain};
        const quad = [2]Triangle{
            .fromCorners(.{ .{ -50, 0, -50 }, .{ -50, 0, 50 }, .{ 50, 0, -50 } }, 0),
            .fromCorners(.{ .{ 50, 0, -50 }, .{ -50, 0, 50 }, .{ 50, 0, 50 } }, 0),
        };
        return init(gpa, &quad, &materials);
    }
};

// the cells of a box, z rows of x
const Cells = struct {
    lo: Cell,
    hi: Cell,
    at: Cell,

    fn init(box: [2]Cell) Cells {
        return .{ .lo = box[0], .hi = box[1], .at = box[0] };
    }

    fn next(c: *Cells) ?Cell {
        if (c.at[1] > c.hi[1]) return null;
        const out = c.at;
        c.at[0] += 1;
        if (c.at[0] > c.hi[0]) c.at = .{ c.lo[0], c.at[1] + 1 };
        return out;
    }
};

fn cellOf(p: V3) Cell {
    return .{ @intFromFloat(@floor(p[0] / cell_size)), @intFromFloat(@floor(p[2] / cell_size)) };
}

fn cellSpan(t: Triangle) [2]Cell {
    const v = t.corners;
    return .{ cellOf(@min(v[0], v[1], v[2]) - splat(pad)), cellOf(@max(v[0], v[1], v[2]) + splat(pad)) };
}

const tolerance: f32 = 1e-6; // 0xe85e08
const just_past_one: f32 = 0x1.000008p0; // 0xf00a60

// 0x8055d0: two-sided Moller-Trumbore, the fraction of ray at the hit
fn hitFraction(origin: V3, ray: V3, v: [3]V3) ?f32 {
    const v0, const v1, const v2 = v;
    const e1 = v1 - v0;
    const e2 = v2 - v0;
    const p = cross(ray, e2);
    const det = dot(p, e1);
    if (det > -tolerance and det < tolerance) return null;
    const inv = 1 / det;
    const s = origin - v0;
    const u = dot(p, s) * inv;
    if (u < -tolerance or u > just_past_one) return null;
    const q = cross(s, e1);
    const w = dot(ray, q) * inv;
    if (w < -tolerance or w + u > just_past_one) return null;
    const t = dot(e2, q) * inv;
    if (t < -tolerance or t > just_past_one) return null;
    return t;
}

test "a ray straight down onto the quad, each face once" {
    var m = try Mesh.testQuad(std.testing.allocator);
    defer m.deinit();
    const h = m.raycast(.{ 0.2, 1, 0.2 }, .{ 0.2, -3, 0.2 }).?;
    try std.testing.expectApproxEqAbs(@as(f32, 0.25), h.t, 1e-7);
    try std.testing.expectApproxEqAbs(@as(f32, 1), h.normal[1], 1e-6);
    try std.testing.expect(m.raycast(.{ 60, 1, 0 }, .{ 60, -3, 0 }) == null);
    var buf: [8]Triangle = undefined;
    try std.testing.expectEqual(2, m.faces(.{ -60, -1, -60 }, .{ 60, 1, 60 }, &buf).len);
}
