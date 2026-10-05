const std = @import("std");
const vec = @import("vec.zig");

const V3 = vec.V3;
const D3 = @Vector(3, f64); // the large track triangles need the precision

fn dot(a: D3, b: D3) f64 {
    return @reduce(.Add, a * b);
}

fn splat(x: f64) D3 {
    return @splat(x);
}

pub const Closest = struct { a: V3, b: V3, distance: f32 }; // a in the first set's hull, b in the second's

const Vertex = struct { w: D3, a: D3, b: D3 }; // w = a - b, a point of the Minkowski difference
const Simplex = struct { v: [4]Vertex = undefined, len: usize = 0 };
const Nearest = struct { point: D3, weights: [4]f64, keep: Simplex };

const max_iterations = 32;

// GJK: the closest points of the convex hulls of two point sets; distance 0 when they overlap
pub fn closest(set_a: []const V3, set_b: []const V3) Closest {
    var s: Simplex = .{};
    s.v[0] = support(set_a, set_b, @floatCast(set_b[0] - set_a[0]));
    s.len = 1;
    var near = nearest(s);
    for (0..max_iterations) |_| {
        const v = near.point;
        const vv = dot(v, v);
        if (vv < 1e-12 or near.keep.len == 4) break; // the origin inside a tetrahedron: overlap
        const w = support(set_a, set_b, -v);
        if (vv - dot(v, w.w) <= 1e-6 * vv or known(near.keep, w.w)) break;
        s = near.keep;
        s.v[s.len] = w;
        s.len += 1;
        near = nearest(s);
    }
    var a = splat(0);
    var b = splat(0);
    for (near.keep.v[0..near.keep.len], near.weights[0..near.keep.len]) |x, l| {
        a += x.a * splat(l);
        b += x.b * splat(l);
    }
    const distance = if (near.keep.len == 4) 0 else @sqrt(dot(near.point, near.point));
    return .{ .a = @floatCast(a), .b = @floatCast(b), .distance = @floatCast(distance) };
}

fn support(set_a: []const V3, set_b: []const V3, dir: D3) Vertex {
    const a = farthest(set_a, dir);
    const b = farthest(set_b, -dir);
    return .{ .w = a - b, .a = a, .b = b };
}

fn farthest(set: []const V3, dir: D3) D3 {
    var best: D3 = @floatCast(set[0]);
    for (set[1..]) |q| {
        const p: D3 = @floatCast(q);
        if (dot(p, dir) > dot(best, dir)) best = p;
    }
    return best;
}

fn known(s: Simplex, w: D3) bool {
    for (s.v[0..s.len]) |x| if (@reduce(.And, x.w == w)) return true;
    return false;
}

// the point of the simplex nearest the origin: the best sub-simplex whose affine projection lies inside it
fn nearest(s: Simplex) Nearest {
    var best: ?Nearest = null;
    for (1..@as(usize, 1) << @intCast(s.len)) |mask| {
        var sub: Simplex = .{};
        for (s.v[0..s.len], 0..) |x, i| if (mask >> @intCast(i) & 1 == 1) {
            sub.v[sub.len] = x;
            sub.len += 1;
        };
        const weights = project(sub) orelse continue;
        var point = splat(0);
        for (sub.v[0..sub.len], weights[0..sub.len]) |x, l| point += x.w * splat(l);
        if (best == null or dot(point, point) < dot(best.?.point, best.?.point))
            best = .{ .point = point, .weights = weights, .keep = sub };
    }
    return best.?;
}

// barycentric weights of the origin's projection on the affine hull, null outside the simplex or degenerate
fn project(s: Simplex) ?[4]f64 {
    var weights: [4]f64 = .{ 1, 0, 0, 0 };
    if (s.len == 1) return weights;
    const p0 = s.v[0].w;
    var e: [3]D3 = undefined;
    for (e[0 .. s.len - 1], s.v[1..s.len]) |*ei, x| ei.* = x.w - p0;
    const k = s.len - 1;
    var g: [3][3]f64 = undefined;
    var r: [3]f64 = undefined;
    for (0..k) |i| {
        for (0..k) |j| g[i][j] = dot(e[i], e[j]);
        r[i] = -dot(e[i], p0);
    }
    const mu = solve(g, r, k) orelse return null;
    weights[0] = 1;
    for (mu[0..k], 1..) |m, i| {
        weights[i] = m;
        weights[0] -= m;
    }
    for (weights[0..s.len]) |l| if (l <= 0) return null;
    return weights;
}

// g x = r for the leading k x k block (Cramer's rule)
fn solve(g: [3][3]f64, r: [3]f64, k: usize) ?[3]f64 {
    var m = g;
    for (k..3) |i| {
        m[i] = .{ 0, 0, 0 };
        m[i][i] = 1;
        for (0..k) |j| m[j][i] = 0;
    }
    const det = det3(m);
    if (@abs(det) <= 1e-12 * m[0][0] * m[1][1] * m[2][2]) return null; // flat, relative to the edge lengths
    var x: [3]f64 = undefined;
    for (0..3) |c| {
        var mc = m;
        for (0..3) |i| mc[i][c] = if (i < k) r[i] else 0;
        x[c] = det3(mc) / det;
    }
    return x;
}

fn det3(m: [3][3]f64) f64 {
    return m[0][0] * (m[1][1] * m[2][2] - m[1][2] * m[2][1]) - m[0][1] * (m[1][0] * m[2][2] - m[1][2] * m[2][0]) +
        m[0][2] * (m[1][0] * m[2][1] - m[1][1] * m[2][0]);
}

const testing = std.testing;

test "a cube corner above a triangle: the corner and the point under it" {
    const cube = [_]V3{ .{ 0, 1, 0 }, .{ 1, 1, 0 }, .{ 0, 2, 0 }, .{ 1, 2, 0 }, .{ 0, 1, 1 }, .{ 1, 1, 1 }, .{ 0, 2, 1 }, .{ 1, 2, 1 } };
    const tri = [_]V3{ .{ -5, 0, -5 }, .{ 5, 0, -5 }, .{ 0, 0, 5 } };
    const c = closest(&tri, &cube);
    try testing.expectApproxEqAbs(1, c.distance, 1e-5);
    try testing.expectApproxEqAbs(0, c.a[1], 1e-5);
    try testing.expectApproxEqAbs(1, c.b[1], 1e-5);
}

test "edge against edge, and overlap" {
    const a = [_]V3{ .{ -1, 0, 0 }, .{ 1, 0, 0 } };
    const b = [_]V3{ .{ 0, 2, -1 }, .{ 0, 2, 1 } };
    try testing.expectApproxEqAbs(2, closest(&a, &b).distance, 1e-5);
    const c = [_]V3{ .{ 0, -1, -1 }, .{ 0, 1, -1 }, .{ 0, 0, 1 } };
    try testing.expectApproxEqAbs(0, closest(&a, &c).distance, 1e-5);
}
