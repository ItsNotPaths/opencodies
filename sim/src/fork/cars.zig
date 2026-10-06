// Car against car, hand-rolled. Not game correct: no telemetry with two cars exists, so no tool checks it
// against a game.
const std = @import("std");
const body = @import("../shared/body.zig");
const chassis = @import("../shared/chassis.zig");
const gjk = @import("../shared/gjk.zig");
const vec = @import("../shared/vec.zig");

const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;
const cross = vec.cross;

// (hole car-pair-friction :tags fork :sev tune) a guess; low, so that bodies slide apart and do not lock
pub const friction = 0.2;

pub const Car = struct { body: body.Body, hull: chassis.Hull, state: *body.State };

pub const Pair = struct { a: u16, b: u16, contact: chassis.Contact }; // a < b; contact.normal: the way out for a; depth < 0: a gap

pub const max_cars = 8; // a lobby
pub const per_pair = 8; // contacts a car pair keeps
pub const max_pairs = max_cars * (max_cars - 1) / 2 * per_pair; // every pair fits

const max_corners = 1024; // all pieces of a hull
const merge = 0.05; // m, a contact this close to a kept one of its pair merges into it, the deeper stays
const max_ahead = 5.0; // m, the farthest a pair looks for contacts past the margins: 2 x 225 km/h in one step

// as chassis
const cfm = 0.01;
const erp = 0.2;
const slop = 0.003; // m
const max_push = 1.0; // m/s, the depth term of a normal row
const iterations = 5;
const project_slop = 0.02; // m, depth that the position pass leaves
const project_iterations = 3;

// pairs a < b where the cores come closer than both margins plus the distance the pair can close in the next
// step (speculative contacts): one contact per piece pair, a's pieces first, merged by point; a full pair swaps
// its shallowest for a deeper one
pub fn collide(cars: []const Car, out: []Pair, dt: f32) []Pair {
    var count: usize = 0;
    var world_a: [max_corners]V3 = undefined;
    var world_b: [max_corners]V3 = undefined;
    for (cars, 0..) |a, i| {
        const all_a = place(a, &world_a);
        for (cars[i + 1 ..], i + 1..) |b, j| {
            const all_b = place(b, &world_b);
            const range = a.hull.margin + b.hull.margin;
            const reach = range + @min(closing(a, all_a, b, all_b) * dt, max_ahead);
            if (!overlaps(bounds(all_a, reach), bounds(all_b, 0))) continue;
            const axes = a.state.axes ++ b.state.axes;
            const first = count;
            var at_a: usize = 0;
            for (a.hull.pieces, 0..) |pa, index| {
                const corners_a = all_a[at_a..][0..pa.len];
                at_a += pa.len;
                var at_b: usize = 0;
                for (b.hull.pieces) |pb| {
                    const corners_b = all_b[at_b..][0..pb.len];
                    at_b += pb.len;
                    if (!overlaps(bounds(corners_a, reach), bounds(corners_b, 0))) continue;
                    var c = contact(corners_a, corners_b, all_a, all_b, axes, b.hull.margin, range, reach) orelse continue;
                    c.piece = @intCast(index);
                    const pair: Pair = .{ .a = @intCast(i), .b = @intCast(j), .contact = c };
                    const kept = out[first..count];
                    if (close(kept, c.point) orelse if (kept.len == per_pair) shallowest(kept) else null) |old| {
                        if (c.depth > old.contact.depth) old.* = pair;
                    } else if (count < out.len) {
                        out[count] = pair;
                        count += 1;
                    }
                }
            }
        }
    }
    return out[0..count];
}

fn close(kept: []Pair, point: V3) ?*Pair {
    for (kept) |*k| {
        const d = k.contact.point - point;
        if (dot(d, d) < merge * merge) return k;
    }
    return null;
}

fn shallowest(kept: []Pair) *Pair {
    var low = &kept[0];
    for (kept[1..]) |*k| if (k.contact.depth < low.contact.depth) {
        low = k;
    };
    return low;
}

// m/s, a bound on how fast any two points of the cars approach
fn closing(a: Car, all_a: []const V3, b: Car, all_b: []const V3) f32 {
    const v = a.state.vel - b.state.vel;
    return @sqrt(dot(v, v)) + spinSpeed(a, all_a) + spinSpeed(b, all_b);
}

fn spinSpeed(car: Car, corners: []const V3) f32 {
    var far: f32 = 0;
    for (corners) |c| far = @max(far, dot(c - car.state.pos, c - car.state.pos));
    return @sqrt(dot(car.state.angvel, car.state.angvel) * far);
}

fn place(car: Car, buf: *[max_corners]V3) []V3 {
    var n: usize = 0;
    for (car.hull.pieces) |p| for (p) |c| {
        buf[n] = vec.toWorld(car.state.axes, c) + car.state.pos;
        n += 1;
    };
    return buf[0..n];
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

fn overlaps(x: [2]V3, y: [2]V3) bool {
    return @reduce(.And, x[0] <= y[1]) and @reduce(.And, y[0] <= x[1]);
}

// as chassis.contact with b as the track: the closest points of the cores (GJK), out to reach. Cores that
// overlap push out along the body axis on which the whole hulls overlap least: one way for all pieces, so a
// seam between two pieces is no way out. The point: the middle of where the pieces overlap across the normal,
// on b's margin
fn contact(a: []const V3, b: []const V3, hull_a: []const V3, hull_b: []const V3, axes: [6]V3, margin_b: f32, range: f32, reach: f32) ?chassis.Contact {
    const near = gjk.closest(b, a);
    if (near.distance >= reach) return null;
    const normal, const depth = if (near.distance > 1e-4)
        .{ (near.b - near.a) / splat(near.distance), range - near.distance }
    else blk: {
        const n = way(hull_a, hull_b, axes);
        break :blk .{ n, range - gap(a, b, n) };
    };
    const across_1 = vec.normalize(cross(if (@abs(normal[1]) <= 0.707) V3{ 0, 1, 0 } else V3{ 1, 0, 0 }, normal));
    const across_2 = cross(across_1, normal);
    const point = normal * splat(span(b, normal)[1] + margin_b) + across_1 * splat(middle(a, b, across_1)) +
        across_2 * splat(middle(a, b, across_2));
    return .{ .point = point, .normal = normal, .depth = depth };
}

// the middle of the overlap of the two spans along dir
fn middle(a: []const V3, b: []const V3, dir: V3) f32 {
    const sa = span(a, dir);
    const sb = span(b, dir);
    return (@max(sa[0], sb[0]) + @min(sa[1], sb[1])) * 0.5;
}

// the axis, toward a, with the largest gap between the point sets
fn way(a: []const V3, b: []const V3, axes: [6]V3) V3 {
    var best: V3 = undefined;
    var most = -std.math.inf(f32);
    const apart = center(a) - center(b);
    for (axes) |axis| {
        const n = if (dot(axis, apart) < 0) -axis else axis;
        const g = gap(a, b, n);
        if (g > most) {
            most = g;
            best = n;
        }
    }
    return best;
}

fn gap(a: []const V3, b: []const V3, n: V3) f32 {
    return span(a, n)[0] - span(b, n)[1]; // m, < 0 inside
}

fn center(points: []const V3) V3 {
    var sum = splat(0);
    for (points) |p| sum += p;
    return sum / splat(@floatFromInt(points.len));
}

fn span(points: []const V3, dir: V3) [2]f32 {
    var lo = dot(points[0], dir);
    var hi = lo;
    for (points[1..]) |p| {
        lo = @min(lo, dot(p, dir));
        hi = @max(hi, dot(p, dir));
    }
    return .{ lo, hi };
}

// as chassis.solve with each row on both cars: a soft normal row per contact, then one friction patch per car
// pair, bound by the friction times the pair's normal sum. Then the position pass
pub fn solve(cars: []const Car, pairs: []const Pair, dt: f32) void {
    if (pairs.len == 0) return;
    var rows: [max_pairs]Row = undefined;
    var patches: [max_pairs]Patch = undefined;
    var n: usize = 0;
    for (pairs, rows[0..pairs.len], 0..) |p, *r, i| {
        const a = cars[p.a];
        const b = cars[p.b];
        const c = p.contact;
        r.* = .at(p, a.state.*, b.state.*, c.point, c.normal);
        if (c.depth < 0) {
            r.rhs = c.depth / dt; // a gap: stop only the approach that closes it in the next step
        } else {
            r.rhs = @min(erp / dt * @max(c.depth - slop, 0), max_push);
            r.cfm = cfm / (dt * a.body.mass * b.body.mass / (a.body.mass + b.body.mass)); // the reduced mass
        }
        if (i == 0 or p.a != pairs[i - 1].a or p.b != pairs[i - 1].b) {
            patches[n] = .{ .from = i, .to = i };
            n += 1;
        }
        patches[n - 1].to = i + 1;
    }
    for (patches[0..n]) |*patch| patch.place(pairs, cars);
    for (0..iterations) |_| for (patches[0..n]) |*patch| {
        var pressed: f32 = 0;
        for (rows[patch.from..patch.to]) |*r| {
            r.relax(cars, 0, r.hi);
            pressed += r.sum;
        }
        for (&patch.rows) |*r| r.relax(cars, -friction * pressed, friction * pressed);
    };
    project(cars, pairs);
}

// one row on two cars: dot(lin, v_a - v_b) + dot(ang_a, w_a) + dot(ang_b, w_b); impulses equal and opposite
const Row = struct {
    a: u16,
    b: u16,
    lin: V3,
    ang_a: V3,
    ang_b: V3,
    rhs: f32 = 0, // target speed
    cfm: f32 = 0,
    hi: f32 = std.math.inf(f32),
    sum: f32 = 0,

    fn at(p: Pair, a: body.State, b: body.State, point: V3, dir: V3) Row {
        return .{ .a = p.a, .b = p.b, .lin = dir, .ang_a = cross(point - a.pos, dir), .ang_b = cross(dir, point - b.pos) };
    }

    // a projected Gauss-Seidel step, as chassis
    fn relax(r: *Row, cars: []const Car, lo: f32, hi: f32) void {
        const a = cars[r.a];
        const b = cars[r.b];
        const turn_a = inverseInertia(a.body, a.state.*, r.ang_a);
        const turn_b = inverseInertia(b.body, b.state.*, r.ang_b);
        const k = dot(r.lin, r.lin) * (1 / a.body.mass + 1 / b.body.mass) + dot(r.ang_a, turn_a) + dot(r.ang_b, turn_b);
        const speed = dot(r.lin, a.state.vel - b.state.vel) + dot(r.ang_a, a.state.angvel) + dot(r.ang_b, b.state.angvel);
        const total = std.math.clamp(r.sum + (r.rhs - speed - r.cfm * r.sum) / (k + r.cfm), lo, hi);
        const d = total - r.sum;
        a.state.vel += r.lin * splat(d / a.body.mass);
        a.state.angvel += turn_a * splat(d);
        b.state.vel -= r.lin * splat(d / b.body.mass);
        b.state.angvel += turn_b * splat(d);
        r.sum = total;
    }
};

// a car pair's contacts pairs[from..to] and its friction rows: twist and two slides at the mean point and normal
const Patch = struct {
    from: usize,
    to: usize,
    rows: [3]Row = undefined,

    fn place(patch: *Patch, pairs: []const Pair, cars: []const Car) void {
        const own = pairs[patch.from..patch.to];
        var point = splat(0);
        var normal = splat(0);
        for (own) |p| {
            point += p.contact.point;
            normal += p.contact.normal;
        }
        const count = splat(@floatFromInt(own.len));
        point /= count;
        normal = vec.normalize(normal / count);
        const axis: V3 = if (@abs(normal[1]) <= 0.707) .{ 0, 1, 0 } else .{ 1, 0, 0 };
        const slide = vec.normalize(cross(axis, normal));
        const p = own[0];
        const a = cars[p.a].state.*;
        const b = cars[p.b].state.*;
        patch.rows = .{
            .{ .a = p.a, .b = p.b, .lin = splat(0), .ang_a = normal, .ang_b = -normal },
            .at(p, a, b, point, slide),
            .at(p, a, b, point, cross(slide, normal)),
        };
    }
};

fn inverseInertia(b: body.Body, s: body.State, v: V3) V3 {
    return vec.toWorld(s.axes, vec.toBody(s.axes, v) / b.inertia);
}

// as chassis.project on both cars of a row: depth past the position slop goes in one step, shared by the
// inverse masses; the poses move, the velocities do not
fn project(cars: []const Car, pairs: []const Pair) void {
    var rows: [max_pairs]Row = undefined;
    var deep = false;
    for (pairs, rows[0..pairs.len]) |p, *r| {
        r.* = .at(p, cars[p.a].state.*, cars[p.b].state.*, p.contact.point, p.contact.normal);
        r.rhs = if (p.contact.depth < 0) p.contact.depth else @max(p.contact.depth - project_slop, 0); // a gap may close
        deep = deep or p.contact.depth >= project_slop;
    }
    if (!deep) return;
    var moved: [max_cars]body.State = undefined;
    var ghosts: [max_cars]Car = undefined;
    for (cars, moved[0..cars.len], ghosts[0..cars.len]) |c, *m, *g| {
        m.* = c.state.*;
        m.vel = splat(0);
        m.angvel = splat(0);
        g.* = .{ .body = c.body, .hull = c.hull, .state = m };
    }
    for (0..project_iterations) |_| for (rows[0..pairs.len]) |*r| r.relax(ghosts[0..cars.len], 0, r.hi);
    for (cars, moved[0..cars.len]) |c, m| {
        const pushed = body.advance(m, 1);
        c.state.pos = pushed.pos;
        c.state.axes = pushed.axes;
    }
}
