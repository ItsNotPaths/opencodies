// Rollback on the made-up hatch (sim/data/cars/hatch.zon): a restored car runs the same bits as the original, and
// the hash tells states apart by their bits only.
const std = @import("std");
const rollback = @import("rollback");
const data = @import("data");

const sim = rollback.sim;
const mesh = rollback.mesh;
const dt = sim.d3_dt;
const expect = std.testing.expect;
const gpa = std.testing.allocator;

const wall_z = 30; // m
const before = 50; // steps before the snapshot
const steps = 100; // after it

const Snap = [sim.snapshot_size]u8;
const Trace = [steps][2]Snap; // each step, up to two cars

// flat tarmac, a wall across +z
fn onLevel(s: *sim.Sim) !void {
    const T = mesh.Triangle;
    const tris = [4]T{
        .fromCorners(.{ .{ -50, 0, -50 }, .{ -50, 0, 50 }, .{ 50, 0, -50 } }, 0),
        .fromCorners(.{ .{ 50, 0, -50 }, .{ -50, 0, 50 }, .{ 50, 0, 50 } }, 0),
        .fromCorners(.{ .{ -50, 0, wall_z }, .{ -50, 5, wall_z }, .{ 50, 0, wall_z } }, 0),
        .fromCorners(.{ .{ 50, 0, wall_z }, .{ -50, 5, wall_z }, .{ 50, 5, wall_z } }, 0),
    };
    try s.world.setTrack(try mesh.Mesh.init(sim.allocator, &tris, &.{try tarmac()}));
    try expect(s.fitLevel());
}

fn tarmac() !rollback.material.Material {
    const all = try rollback.zon.parse([]rollback.material.Material, gpa, data.materials);
    defer gpa.free(all);
    return all[rollback.material.find(all, "TSD*".*).?];
}

// car 0 drives with varied inputs, car 1 stays parked
fn input(i: usize, car: usize) sim.Input {
    if (car > 0) return .{};
    return .{
        .steer = if (i % 30 < 15) 0.1 else -0.1,
        .throttle = if (i % 25 == 24) 0 else 1,
        .brake = if (i % 40 == 39) 0.5 else 0,
        .gear = if (i < 40) 1 else 2,
        .handbrake = i % 70 == 69,
    };
}

const Seen = struct { reach: f32 = -std.math.inf(f32), hits: usize = 0 }; // reach: car 0's farthest z

// steps from..from+n: each car, then car against car
fn drive(cars: []const *sim.Sim, from: usize, n: usize, trace: ?*Trace) Seen {
    var seen: Seen = .{};
    for (from..from + n, 0..) |i, k| {
        for (cars, 0..) |c, j| sim.sim_step(c, &input(i, j), dt);
        rollback.fork.sim_collide_cars(cars.ptr, @intCast(cars.len), dt);
        var hit: [1]rollback.Hit = undefined;
        seen.hits += rollback.fork.sim_hits(&hit, 1);
        seen.reach = @max(seen.reach, cars[0].game.d3.body.pos[2]);
        if (trace) |t| for (cars, 0..) |c, j| sim.sim_snapshot(c, &t[k][j]);
    }
    return seen;
}

// a run from the start, a snapshot after `before` steps, `steps` more; then the same steps again from the snapshot,
// on the cars and on fresh ones of the same car and level: the same bits every step
fn replays(cars: []const *sim.Sim, fresh: []const *sim.Sim) !Seen {
    _ = drive(cars, 0, before, null);
    var snaps: [2]Snap = undefined;
    for (cars, 0..) |c, j| sim.sim_snapshot(c, &snaps[j]);
    const first = try gpa.create(Trace);
    defer gpa.destroy(first);
    const again = try gpa.create(Trace);
    defer gpa.destroy(again);
    first.* = std.mem.zeroes(Trace);
    const seen = drive(cars, before, steps, first);
    for (&[_][]const *sim.Sim{ cars, fresh }) |run| {
        again.* = std.mem.zeroes(Trace);
        for (run, 0..) |c, j| try expect(sim.sim_restore(c, &snaps[j]));
        _ = drive(run, before, steps, again);
        for (first, again) |x, y| try std.testing.expectEqualSlices(u8, std.mem.asBytes(&x), std.mem.asBytes(&y));
    }
    return seen;
}

fn hatchOnLevel() !sim.Sim {
    var s: sim.Sim = .{};
    try onLevel(&s);
    return s;
}

test "a restored car runs the same bits, into a wall" {
    var a = try hatchOnLevel();
    defer a.world.deinit();
    var fresh = try hatchOnLevel();
    defer fresh.world.deinit();
    const seen = try replays(&.{&a}, &.{&fresh});
    const nose = wall_z - 1.95; // m, the hatch's centre with its nose at the wall
    try expect(seen.reach > nose - 0.3 and seen.reach < nose); // the hull met the wall and held
}

test "two cars restored together run the same bits through a hit" {
    var a = try hatchOnLevel();
    defer a.world.deinit();
    var b = try hatchOnLevel();
    defer b.world.deinit();
    b.game.d3.body.pos[2] = 22; // a runs into it after the snapshot and pushes it into the wall
    var fresh: [2]sim.Sim = .{ try hatchOnLevel(), try hatchOnLevel() };
    defer for (&fresh) |*f| f.world.deinit();
    fresh[1].game.d3.body.pos[2] = 22;
    const seen = try replays(&.{ &a, &b }, &.{ &fresh[0], &fresh[1] });
    try expect(seen.hits > 0);
}

test "the same state by another route has the same hash" {
    var stepped = try hatchOnLevel();
    defer stepped.world.deinit();
    var restored = try hatchOnLevel();
    defer restored.world.deinit();
    const start = sim.sim_hash(&stepped);
    _ = drive(&.{&stepped}, 0, before, null);
    var snap: Snap = undefined;
    sim.sim_snapshot(&stepped, &snap);
    try expect(sim.sim_restore(&restored, &snap));
    try expect(sim.sim_hash(&stepped) != start);
    try std.testing.expectEqual(sim.sim_hash(&stepped), sim.sim_hash(&restored));
    _ = drive(&.{&stepped}, before, 10, null);
    _ = drive(&.{&restored}, before, 10, null);
    try std.testing.expectEqual(sim.sim_hash(&stepped), sim.sim_hash(&restored));
}

test "1 ulp in any position or velocity component changes the hash" {
    var s = try hatchOnLevel();
    defer s.world.deinit();
    _ = drive(&.{&s}, 0, before, null);
    const base = sim.sim_hash(&s);
    const b = &s.game.d3.body;
    for ([_]*rollback.vec.V3{ &b.pos, &b.vel, &b.angvel }) |v| inline for (0..3) |k| {
        const was = v[k];
        v[k] = @bitCast(@as(u32, @bitCast(was)) + 1);
        try expect(sim.sim_hash(&s) != base);
        v[k] = was;
    };
    try std.testing.expectEqual(base, sim.sim_hash(&s));
}

test "leftover memory in a vector's padding lane does not change the hash" {
    var s = try hatchOnLevel();
    defer s.world.deinit();
    _ = drive(&.{&s}, 0, before, null);
    const base = sim.sim_hash(&s);
    for (&s.game.d3.body.axes) |*v| @memset(std.mem.asBytes(v)[12..], 0xab);
    try std.testing.expectEqual(base, sim.sim_hash(&s));
}
