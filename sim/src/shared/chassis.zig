const std = @import("std");
const vec = @import("vec.zig");
const body = @import("body.zig");
const mesh = @import("mesh.zig");
const gjk = @import("gjk.zig");

const Mesh = mesh.Mesh;
const Triangle = mesh.Triangle;
const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;
const cross = vec.cross;
const toBody = vec.toBody;
const toWorld = vec.toWorld;

pub const Piece = []const V3; // the corners of a convex shape, body frame

pub const Hull = struct {
    pieces: []const Piece,
    margin: f32, // m the shapes reach past their corners
    friction: f32, // the body material; the track's joins it in a geometric mean
    softness: f32, // N s/m: the normal impulse per m of depth that a contact may give [stub]
    impact_softness: ?[]const f32 = null, // the softness divides by this, by impact speed in steps of 10 mph; null: 1
};

pub const Contact = struct { point: V3, normal: V3, depth: f32, piece: u16 = 0 }; // normal: the way out for the car

// the rigid-body world around the car (island +0xd0..+0xf4, the material pair of 0x80c2f0)
pub const track_margin = 0.02; // shape+0xd8 of a track triangle, 0xfb49e8
const track_friction = 0.55; // the track material [measured]
const cfm = 0.01;
const erp = 0.2;
const slop = 0.003; // m
const max_push = 1.0; // m/s, the depth term of a normal row
const iterations = 5;
const project_slop = 0.02; // m, depth that the position pass leaves
const project_iterations = 3;

const reach = 0.5; // m, deepest a hull corner may sit behind a face and still count when the cores overlap
const face_normal_cos = 0.7;
const merge = 0.05; // m, 0x7e2970 drops a corner this close to an earlier one
const max_contacts = 256;
const max_faces = 4096;
const max_corners = 1024; // all pieces of a hull

// D3's world stages 2-5 after the vehicle step (ghidra-physics.md §16): contacts, the velocity solve, the position pass
pub fn step(b: body.Body, hull: Hull, s: *body.State, track: *const Mesh, dt: f32) void {
    var found: [max_contacts]Contact = undefined;
    const contacts = collide(hull, s.*, track, &found);
    if (contacts.len == 0) return;
    solve(b, hull, s, contacts, dt);
    const pushed = project(b, s.*, contacts);
    s.pos = pushed.pos;
    s.axes = pushed.axes;
}

// 0x86a0f0 / 0x852380: where the cores come closer than both margins; the deepest per piece and track chunk
pub fn collide(hull: Hull, s: body.State, track: *const Mesh, out: []Contact) []Contact {
    var chunks: [max_contacts]u16 = undefined;
    const range = hull.margin + track_margin;
    var world: [max_corners]V3 = undefined;
    var n: usize = 0;
    for (hull.pieces) |p| for (p) |c| {
        world[n] = toWorld(s.axes, c) + s.pos;
        n += 1;
    };
    const lo, const hi = bounds(world[0..n], range);
    var buf: [max_faces]Triangle = undefined;
    const faces = track.faces(lo, hi, &buf);
    var count: usize = 0;
    var at: usize = 0;
    for (hull.pieces, 0..) |p, index| {
        const corners = world[at..][0..p.len];
        at += p.len;
        const plo, const phi = bounds(corners, range);
        const first = count;
        for (faces) |f| {
            if (!f.overlaps(plo, phi)) continue;
            var c = contact(corners, f, range) orelse continue;
            c.piece = @intCast(index);
            const same = for (out[first..count], chunks[first..count]) |*kept, k| {
                if (k == f.chunk) break kept;
            } else null;
            if (same) |kept| {
                if (c.depth > kept.depth) kept.* = c;
            } else if (count < @min(out.len, chunks.len)) {
                out[count] = c;
                chunks[count] = f.chunk;
                count += 1;
            }
        }
    }
    return out[0..count];
}

fn bounds(points: []const V3, margin: f32) [2]V3 {
    var lo = points[0];
    var hi = points[0];
    for (points[1..]) |p| {
        lo = @min(lo, p);
        hi = @max(hi, p);
    }
    return .{ lo - splat(margin), hi + splat(margin) };
}

// 0x852380: the closest points of the two cores (GJK), the point on the triangle's margin. Cores that
// overlap take the deepest corner under the face instead
fn contact(corners: []const V3, f: Triangle, range: f32) ?Contact {
    const near = gjk.closest(&f.corners, corners);
    if (near.distance >= range) return null;
    if (near.distance > 1e-4) {
        const normal = (near.b - near.a) / splat(near.distance);
        if (dot(normal, f.normal) > face_normal_cos) {
            const gap = dot(f.normal, near.b - near.a);
            return .{ .point = near.a + f.normal * splat(track_margin), .normal = f.normal, .depth = range - gap };
        }
        return .{ .point = near.a + normal * splat(track_margin), .normal = normal, .depth = range - near.distance };
    }
    var best: ?Contact = null;
    for (corners) |c| {
        const d = dot(f.normal, c - f.corners[0]);
        if (d < -reach or !inside(f, c - f.normal * splat(d))) continue;
        if (best == null or range - d > best.?.depth)
            best = .{ .point = c - f.normal * splat(d - track_margin), .normal = f.normal, .depth = range - d };
    }
    return best;
}

fn inside(f: Triangle, q: V3) bool {
    const v = f.corners;
    for (0..3) |k| if (dot(cross(v[(k + 1) % 3] - v[k], q - v[k]), f.normal) < -1e-6) return false;
    return true;
}

// one constraint row on the body: dot(lin, v) + dot(ang, w), its summed impulse within lo..hi
const Row = struct {
    lin: V3,
    ang: V3,
    rhs: f32 = 0, // target speed
    cfm: f32 = 0,
    hi: f32 = std.math.inf(f32),
    sum: f32 = 0,

    fn at(arm: V3, dir: V3) Row {
        return .{ .lin = dir, .ang = cross(arm, dir) };
    }

    // a projected Gauss-Seidel step (0x83a3e0)
    fn relax(r: *Row, b: body.Body, s: *body.State, lo: f32, hi: f32) void {
        const turn = inverseInertia(b, s.*, r.ang);
        const k = dot(r.lin, r.lin) / b.mass + dot(r.ang, turn);
        const speed = dot(r.lin, s.vel) + dot(r.ang, s.angvel);
        const total = std.math.clamp(r.sum + (r.rhs - speed - r.cfm * r.sum) / (k + r.cfm), lo, hi);
        const d = total - r.sum;
        s.vel += r.lin * splat(d / b.mass);
        s.angvel += turn * splat(d);
        r.sum = total;
    }
};

fn inverseInertia(b: body.Body, s: body.State, v: V3) V3 {
    return toWorld(s.axes, toBody(s.axes, v) / b.inertia);
}

// 0x854160: a soft normal row per contact, capped by the softness, then one friction patch for all of them
// (twist and two slides at the mean point and normal), bound by the friction times the normal sum
fn solve(b: body.Body, hull: Hull, s: *body.State, contacts: []const Contact, dt: f32) void {
    var rows: [max_contacts]Row = undefined;
    var point = splat(0);
    var normal = splat(0);
    for (contacts, rows[0..contacts.len]) |c, *r| {
        r.* = .at(c.point - s.pos, c.normal);
        r.rhs = @min(erp / dt * @max(c.depth - slop, 0), max_push);
        r.cfm = cfm / (dt * b.mass);
        r.hi = hull.softness * c.depth;
        point += c.point;
        normal += c.normal;
    }
    const count = splat(@floatFromInt(contacts.len));
    point /= count;
    normal = vec.normalize(normal / count);
    const axis: V3 = if (@abs(normal[1]) <= 0.707) .{ 0, 1, 0 } else .{ 1, 0, 0 };
    const slide = vec.normalize(cross(axis, normal));
    const arm = point - s.pos;
    var patch = [3]Row{ .{ .lin = splat(0), .ang = normal }, .at(arm, slide), .at(arm, cross(slide, normal)) };
    const mu = @sqrt(hull.friction * track_friction);
    for (0..iterations) |_| {
        var pressed: f32 = 0;
        for (rows[0..contacts.len]) |*r| {
            r.relax(b, s, 0, r.hi);
            pressed += r.sum;
        }
        for (&patch) |*r| r.relax(b, s, -mu * pressed, mu * pressed);
    }
}

// 0x854d70: depth past the position slop goes in one step; the pose moves, the velocity does not
fn project(b: body.Body, s: body.State, contacts: []const Contact) body.State {
    var rows: [max_contacts]Row = undefined;
    var deep = false;
    for (contacts, rows[0..contacts.len]) |c, *r| {
        r.* = .at(c.point - s.pos, c.normal);
        r.rhs = @max(c.depth - project_slop, 0);
        deep = deep or c.depth >= project_slop;
    }
    if (!deep) return s;
    var moved = s;
    moved.vel = splat(0);
    moved.angvel = splat(0);
    for (0..project_iterations) |_| for (rows[0..contacts.len]) |*r| r.relax(b, &moved, 0, r.hi);
    return body.advance(moved, 1);
}

const testing = std.testing;

test {
    _ = gjk;
}
