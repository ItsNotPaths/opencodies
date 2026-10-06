// Car against car on two made-up hatches (sim/data/cars/hatch.zon): where contacts must come out, which way, and
// that the solve keeps the cars apart.
// The hatch: body x ±0.7, y -0.07..0.25, z -1.85..1.95; cabin to y 0.85; margin 0.1 m
const std = @import("std");
const fork = @import("fork");

const cars = fork.cars;
const body = fork.body;
const V3 = fork.vec.V3;
const expect = std.testing.expect;
const approx = std.testing.expectApproxEqAbs;

const sim = fork.sim;
const hatch = sim.default_car;
const range = 2 * hatch.hull.?.margin;
const dt = sim.d3_dt;

const ahead = [3]V3{ .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, 1 } };
const back = [3]V3{ .{ -1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, -1 } };
const across = [3]V3{ .{ 0, 0, 1 }, .{ 0, 1, 0 }, .{ -1, 0, 0 } }; // forward along -x

fn at(pos: V3, axes: [3]V3) body.State {
    return .{ .pos = pos, .axes = axes, .vel = @splat(0), .angvel = @splat(0) };
}

fn car(s: *body.State) cars.Car {
    return .{ .body = hatch.rigid(), .hull = hatch.hull.?, .state = s };
}

fn collide(a: body.State, b: body.State, out: []cars.Pair) []cars.Pair {
    var sa = a;
    var sb = b;
    return cars.collide(&.{ car(&sa), car(&sb) }, out, dt);
}

// every contact: pair 0-1, normal n, depth about d
fn expectAll(pairs: []const cars.Pair, n: V3, d: f32) !void {
    try expect(pairs.len > 0);
    for (pairs) |p| {
        try expect(p.a == 0 and p.b == 1);
        inline for (0..3) |k| try approx(n[k], p.contact.normal[k], 1e-4);
        try approx(d, p.contact.depth, 1e-4);
    }
}

test "two cars apart by more than the margins: no contacts" {
    var out: [64]cars.Pair = undefined;
    try expect(collide(at(.{ 0, 0, 0 }, ahead), at(.{ 0, 0, 3.8 + range + 0.1 }, ahead), &out).len == 0);
    try expect(collide(at(.{ 0, 0, 0 }, ahead), at(.{ 1.4 + range + 0.01, 0, 0 }, ahead), &out).len == 0);
}

test "nose to nose: the normal pushes a back, the depth is the overlap" {
    var out: [64]cars.Pair = undefined;
    const noses = 1.95 + 1.95;
    try expectAll(collide(at(.{ 0, 0, 0 }, ahead), at(.{ 0, 0, noses + range - 0.04 }, back), &out), .{ 0, 0, -1 }, 0.04);
    // the cores 3 cm into each other
    try expectAll(collide(at(.{ 0, 0, 0 }, ahead), at(.{ 0, 0, noses - 0.03 }, back), &out), .{ 0, 0, -1 }, range + 0.03);
}

test "side by side and T-bone: the normal points from b to a" {
    var out: [64]cars.Pair = undefined;
    try expectAll(collide(at(.{ 0, 0, 0 }, ahead), at(.{ 1.4 + range - 0.02, 0, 0 }, ahead), &out), .{ -1, 0, 0 }, 0.02);
    // a's nose into b's left side
    try expectAll(collide(at(.{ 0, 0, 0 }, ahead), at(.{ 0, 0, 1.95 + 0.7 + range - 0.03 }, across), &out), .{ 0, 0, -1 }, 0.03);
}

test "a car on the roof of another: the top car's way out is up" {
    var out: [64]cars.Pair = undefined;
    const top = at(.{ 0, 0.85 + 0.07 + range - 0.03, 0 }, ahead);
    const bottom = at(.{ 0, 0, 0 }, ahead);
    try expectAll(collide(top, bottom, &out), .{ 0, 1, 0 }, 0.03);
    try expectAll(collide(bottom, top, &out), .{ 0, -1, 0 }, 0.03);
}

test "contacts stop at the length of out" {
    var out: [2]cars.Pair = undefined;
    try expect(collide(at(.{ 0, 0, 0 }, ahead), at(.{ 1.4 + range - 0.02, 0, 0 }, ahead), &out).len == 2);
}

test "the same input gives the same bits" {
    const tilt = [3]V3{ .{ 0.96, 0.28, 0 }, .{ -0.28, 0.96, 0 }, .{ 0, 0, 1 } };
    var first: [64]cars.Pair = undefined;
    var second: [64]cars.Pair = undefined;
    const a = at(.{ 0.3, 0.1, 0 }, tilt);
    const b = at(.{ 0.2, 0.4, 2.6 }, across);
    const one = collide(a, b, &first);
    const two = collide(a, b, &second);
    try expect(one.len > 0 and one.len == two.len);
    for (one, two) |x, y| try std.testing.expectEqual(bits(x), bits(y));
}

// two free cars, no gravity: each step moves them, then finds and solves their contacts
const Run = struct { a: body.State, b: body.State, first: ?usize = null, apart: ?usize = null };

fn run(a: body.State, b: body.State, steps: usize) Run {
    var r: Run = .{ .a = a, .b = b };
    for (0..steps) |i| {
        r.a = body.advance(r.a, dt);
        r.b = body.advance(r.b, dt);
        const list = [_]cars.Car{ car(&r.a), car(&r.b) };
        var out: [cars.max_pairs]cars.Pair = undefined;
        const pairs = cars.collide(&list, &out, dt);
        if (pairs.len > 0 and r.first == null) r.first = i;
        if (pairs.len == 0 and r.first != null and r.apart == null) r.apart = i;
        cars.solve(&list, pairs, dt);
    }
    return r;
}

fn moving(pos: V3, axes: [3]V3, speed: f32) body.State {
    var s = at(pos, axes);
    s.vel = axes[2] * @as(V3, @splat(speed));
    return s;
}

// two full cars on flat ground, a with the input: the deepest contact after each sim_collide_cars
fn together(a: *sim.Sim, b: *sim.Sim, input: sim.Input, steps: usize) f32 {
    var worst: f32 = 0;
    for (0..steps) |_| {
        sim.sim_step(a, &input, dt);
        sim.sim_step(b, &.{}, dt);
        fork.exports.sim_collide_cars(&[_]*sim.Sim{ a, b }, 2, dt);
        var out: [cars.max_pairs]cars.Pair = undefined;
        for (cars.collide(&.{ car(&a.game.d3.body), car(&b.game.d3.body) }, &out, dt)) |p| worst = @max(worst, p.contact.depth);
    }
    return worst;
}

test "head on at equal speed: the momentum stays, the cars separate" {
    const r = run(moving(.{ 0, 0, 0 }, ahead, 10), moving(.{ 0, 0, 6 }, back, 10), 50);
    const momentum = (r.a.vel + r.b.vel) * @as(V3, @splat(hatch.mass));
    inline for (0..3) |k| try approx(0, momentum[k], 0.01 * hatch.mass * 10);
    try expect(r.first != null and r.apart != null);
}

test "a car pushes a car at rest from the side: no more than 5 cm in" {
    var pusher: sim.Sim = .{};
    pusher.game.d3.body.pos = .{ -3, 0.39, 0 };
    pusher.game.d3.body.axes = .{ .{ 0, 0, -1 }, .{ 0, 1, 0 }, .{ 1, 0, 0 } }; // forward along +x
    var rest: sim.Sim = .{};
    const worst = together(&pusher, &rest, .{ .throttle = 1, .gear = 1 }, 75);
    try expect(worst < 0.05);
    try expect(rest.game.d3.body.pos[0] > 0.5); // pushed along
}

test "a car on the roof of another does not sink through" {
    var bottom: sim.Sim = .{};
    var top: sim.Sim = .{};
    top.game.d3.body.pos[1] += 0.85 + 0.07 + range + 0.05;
    const worst = together(&bottom, &top, .{}, 50);
    try expect(worst < 0.05);
    try expect(top.game.d3.body.pos[1] - bottom.game.d3.body.pos[1] > 0.85 + 0.07); // the cores stay apart
}

test "the same run gives the same bits" {
    const tilt = [3]V3{ .{ 0.96, 0.28, 0 }, .{ -0.28, 0.96, 0 }, .{ 0, 0, 1 } };
    const one = run(moving(.{ 0.3, 0.1, 0 }, tilt, 8), moving(.{ 0.2, 0.4, 4 }, across, 3), 50);
    const two = run(moving(.{ 0.3, 0.1, 0 }, tilt, 8), moving(.{ 0.2, 0.4, 4 }, across, 3), 50);
    try expect(one.first != null);
    try std.testing.expectEqual(stateBits(one.a) ++ stateBits(one.b), stateBits(two.a) ++ stateBits(two.b));
}

// a hatch on flat ground, settled, then rolling forward at km/h
fn launch(x: f32, z: f32, axes: [3]V3, kmh: f32) sim.Sim {
    var s: sim.Sim = .{};
    s.game.d3.body.pos = .{ x, 0.39, z };
    s.game.d3.body.axes = axes;
    for (0..25) |_| sim.sim_step(&s, &.{}, dt);
    s.game.d3.body.vel = axes[2] * @as(V3, @splat(kmh / 3.6));
    for (&s.game.d3.wheels, hatch.wheels) |*w, p| w.spin = kmh / 3.6 / p.radius;
    return s;
}

const Crash = struct {
    overlap: f32 = 0, // m, the deepest the cores went into each other
    low: f32 = std.math.inf(f32), // m, the lowest body centre
    passed: bool = false, // b went behind a along the line a saw it on at the start
    touching: bool = false, // at the end
};

// both coasting in neutral for 3 s
fn crash(a: *sim.Sim, b: *sim.Sim) Crash {
    var r: Crash = .{};
    const line = b.game.d3.body.pos - a.game.d3.body.pos;
    for (0..75) |_| {
        sim.sim_step(a, &.{}, dt);
        sim.sim_step(b, &.{}, dt);
        const sa = &a.game.d3.body;
        const sb = &b.game.d3.body;
        r.overlap = @max(r.overlap, overlap(sa, sb)); // the vehicle step's move
        fork.exports.sim_collide_cars(&[_]*sim.Sim{ a, b }, 2, dt);
        r.overlap = @max(r.overlap, overlap(sa, sb));
        var out: [cars.max_pairs]cars.Pair = undefined;
        r.touching = cars.collide(&.{ car(sa), car(sb) }, &out, 0).len > 0;
        r.low = @min(r.low, sa.pos[1], sb.pos[1]);
        r.passed = r.passed or fork.vec.dot(sb.pos - sa.pos, line) < 0;
    }
    return r;
}

// m, the deepest the cores of the two cars are in each other
fn overlap(a: *body.State, b: *body.State) f32 {
    var out: [cars.max_pairs]cars.Pair = undefined;
    var deepest_core: f32 = 0;
    for (cars.collide(&.{ car(a), car(b) }, &out, 0)) |p| deepest_core = @max(deepest_core, p.contact.depth - range);
    return deepest_core;
}

const speeds = [_]f32{ 36, 72, 100, 150, 200 };

test "head on at speed: no deep overlap, no fall, no pass" {
    for (speeds) |kmh| {
        var a = launch(0, 0, ahead, kmh);
        var b = launch(0, 40, back, kmh);
        try expectSafe(crash(&a, &b), true);
    }
}

test "glancing at speed: no deep overlap, no fall, they spin away" {
    for (speeds) |kmh| {
        var a = launch(0, 0, ahead, kmh);
        var b = launch(0.7, 40, back, kmh); // half a car width
        const r = crash(&a, &b);
        try expectSafe(r, false); // they may slide past each other
        try expect(!r.touching);
        try expect(@max(@abs(yaw(a, ahead)), @abs(yaw(b, back))) > 5);
    }
}

test "T-bone at 100 km/h into a parked car" {
    var a = launch(0, 0, ahead, 100);
    var b = launch(0, 30, across, 0);
    try expectSafe(crash(&a, &b), true);
}

test "a crash at speed gives the same bits twice" {
    var runs: [2][36]u32 = undefined;
    for (&runs) |*x| {
        var a = launch(0, 0, ahead, 150);
        var b = launch(0.7, 40, back, 150);
        _ = crash(&a, &b);
        x.* = stateBits(a.game.d3.body) ++ stateBits(b.game.d3.body);
    }
    try std.testing.expectEqual(runs[0], runs[1]);
}

fn expectSafe(r: Crash, no_pass: bool) !void {
    try expect(r.overlap <= 0.1);
    try expect(r.low > 0); // above the ground
    if (no_pass) try expect(!r.passed);
}

// degrees the car turned from its start heading
fn yaw(s: sim.Sim, start: [3]V3) f32 {
    const f = s.game.d3.body.axes[2];
    const g = start[2];
    return std.math.radiansToDegrees(std.math.atan2(f[0] * g[2] - f[2] * g[0], f[0] * g[0] + f[2] * g[2]));
}

fn stateBits(s: body.State) [18]u32 {
    var out: [18]u32 = undefined;
    inline for (.{ s.pos, s.axes[0], s.axes[1], s.axes[2], s.vel, s.angvel }, 0..) |v, i| {
        const u: @Vector(3, u32) = @bitCast(v);
        out[3 * i ..][0..3].* = u;
    }
    return out;
}

fn bits(p: cars.Pair) [10]u32 {
    const c = p.contact;
    const u: @Vector(3, u32) = @bitCast(c.point);
    const v: @Vector(3, u32) = @bitCast(c.normal);
    return .{ p.a, p.b, c.piece, u[0], u[1], u[2], v[0], v[1], v[2], @bitCast(c.depth) };
}
