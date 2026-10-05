// The tyre-graph step on the made-up coupe (sim/data/cars/coupe.zon), and DR1-model parts on made-up numbers.
const std = @import("std");
const dr1 = @import("dr1");
const data = @import("data");

const testing = std.testing;
const expect = testing.expect;
const approx = testing.expectApproxEqAbs;
const gpa = testing.allocator;
const V3 = @Vector(3, f32);

test "a tyre curve peaks at its peak slip and falls toward the sliding level" {
    const g: dr1.tyre.Graph = .{ .peak_x1 = 0.2, .peak_x4 = 0.25, .peak_y1 = 1, .peak_y4 = 0, .end_y1 = 0.8, .shape_1 = 2, .shape_4 = 2, .preshape_1 = 0.25, .preshape_4 = 0.25 };
    const l = dr1.tyre.loadGraph(g, 1, 1);
    try approx(@as(f32, 1), dr1.tyre.sample(l, 0.2), 1e-4);
    try expect(dr1.tyre.sample(l, 0.1) < 1 and dr1.tyre.sample(l, 0.5) < 1);
    try expect(dr1.tyre.sample(l, 50) < 0.9);
}

test "the sea level goes to the air density as whole mm" {
    const raw = dr1.spec.seaLevelRaw(-123.4567);
    try testing.expectEqual(@as(i64, -123457), @as(i64, @bitCast(raw *% 0x887e527813a1713d))); // the factor aero.airDensity reads it with
}

test "a contact on an undamaged lattice point gives 15 body masses per strength per metre" {
    const p: dr1.softness.Point = .{ .pos = .{ 0, 0, 0, 1 }, .velocity = @splat(0), .min = .{ -0.1, -0.1, -0.1, 1 }, .max = .{ 0.1, 0.1, 0.1, 2 } };
    const a = dr1.softness.absorbed(1200, &.{p}, 0.45, .{ 0, 0, 0, 1 }, .{ 0, 1, 0, 0 }, 0.05);
    try testing.expectEqual(@as(f32, 15 * 1200 / 2), a.all);
    try testing.expectEqual(a.all, a.directional.?);
}

test "a harder impact scales the contact softness up; past the table the last entry holds" {
    const table = [_]f32{ 0.1, 0.2, 0.3, 0.5 }; // in 10 mph steps
    const s = dr1.softness.speedScale;
    try expect(s(&table, 2) < s(&table, 8));
    try approx(@as(f32, 0.25), s(&table, 15.0 / 2.232), 1e-5); // half way between the 10 and 20 mph entries
    try testing.expectEqual(s(&table, 40), s(&table, 200));
    try testing.expectEqual(@as(f32, 0.5), s(&table, 200));
}

test "the world moves the body by its end velocity" {
    var p: dr1.body.WorldPose = .{ .rows = .{ .{ 1, 0, 0, 0 }, .{ 0, 1, 0, 0 }, .{ 0, 0, 1, 0 }, .{ 10, 20, 30, 1 } }, .vel = .{ 6, -3, 12, 0 }, .angvel = @splat(0) };
    dr1.body.integrateInWorld(&p, 0.5);
    try testing.expectEqual([4]f32{ 13, 18.5, 36, 1 }, p.rows[3]);
    p.angvel = .{ 0, 2, 0, 0 };
    dr1.body.integrateInWorld(&p, 0.25);
    const forward: V3 = .{ p.rows[2][0], p.rows[2][1], p.rows[2][2] };
    try approx(@sin(@as(f32, 0.5)), forward[0], 1e-5); // half a radian of yaw
    try approx(1, @reduce(.Add, forward * forward), 1e-5);
}

fn coupe() !dr1.Car {
    return dr1.zon.parse(dr1.Car, gpa, data.coupe);
}

test "the hull pushes a car sunk into the ground out, and stops its fall" {
    const c = try coupe();
    defer std.zon.parse.free(gpa, c);
    var track = try dr1.mesh.Mesh.testQuad(gpa);
    defer track.deinit();
    var p: dr1.body.WorldPose = .{ .rows = .{ .{ 1, 0, 0, 0 }, .{ 0, 1, 0, 0 }, .{ 0, 0, 1, 0 }, .{ 0, 0.1, 0, 1 } }, .vel = .{ 0, -3, 0, 0 }, .angvel = @splat(0) };
    dr1.world.collide(c.rigid(), c.hull.?, &p, &track, 1.0 / 60.0);
    try expect(p.vel[1] > 0);
    try expect(p.rows[3][1] > 0.1);
}

// the coupe at rest on 100 m of made-up tarmac, through the sim as sim_load_dr1_car puts it
fn onTarmac(c: dr1.Car) !dr1.sim.Sim {
    const all = try dr1.zon.parse([]dr1.material.Material, gpa, data.materials);
    defer gpa.free(all);
    const quad = [2]dr1.mesh.Triangle{
        .fromCorners(.{ .{ -50, 0, -50 }, .{ -50, 0, 50 }, .{ 50, 0, -50 } }, 0),
        .fromCorners(.{ .{ 50, 0, -50 }, .{ -50, 0, 50 }, .{ 50, 0, 50 } }, 0),
    };
    var s: dr1.sim.Sim = .{ .car = c };
    try s.world.setTrack(try dr1.mesh.Mesh.init(dr1.sim.allocator, &quad, all[dr1.material.find(all, "TSD*".*).?..][0..1]));
    const pose = [4][4]f32{ .{ 1, 0, 0, 0 }, .{ 0, 1, 0, 0 }, .{ 0, 0, 1, 0 }, .{ 0, 0.45, -40, 1 } };
    s.game = .{ .dr1 = .{ .state = dr1.spec.init(c, pose, dr1.spec.seaLevelRaw(0), dr1.spec.undamaged) } };
    try expect(s.fitLevel());
    return s;
}

fn run(s: *dr1.sim.Sim, in: dr1.sim.Input, seconds: f32) void {
    for (0..@intFromFloat(seconds * 60)) |_| dr1.sim.sim_step(s, &in, dr1.sim.dr1_dt);
}

fn height(s: *const dr1.sim.Sim) f32 {
    return s.game.dr1.state.body.rows[3][1];
}

test "the coupe settles on its two-segment springs" {
    const c = try coupe();
    defer std.zon.parse.free(gpa, c);
    var s = try onTarmac(c);
    defer s.world.deinit();
    defer dr1.sim.allocator.free(s.game.dr1.track_materials);
    run(&s, .{}, 3);
    const h = height(&s);
    run(&s, .{}, 1);
    try approx(h, height(&s), 1e-3);
    try expect(h > 0.25 and h < 0.45); // on its wheels: hubs 0.05 m under the body, radius 0.31 m
}

test "the coupe drives forward in first and stops on the brake" {
    const c = try coupe();
    defer std.zon.parse.free(gpa, c);
    var s = try onTarmac(c);
    defer s.world.deinit();
    defer dr1.sim.allocator.free(s.game.dr1.track_materials);
    run(&s, .{}, 2);
    run(&s, .{ .throttle = 1, .gear = 1 }, 2);
    const v = s.game.dr1.state.body.vel;
    try expect(v[2] > 5 and @abs(v[0]) < 0.05 * v[2]);
    run(&s, .{ .brake = 1, .gear = 1 }, 4);
    try approx(0, s.game.dr1.state.body.vel[2], 0.3);
}
