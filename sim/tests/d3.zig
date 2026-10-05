// The friction-circle tyre step on the made-up hatch (sim/data/cars/hatch.zon): what the car and its parts must do.
const std = @import("std");
const d3 = @import("d3");
const data = @import("data");

const sim = d3.sim;
const Car = d3.Car;
const V3 = d3.vec.V3;
const expect = std.testing.expect;
const approx = std.testing.expectApproxEqAbs;
const gpa = std.testing.allocator;

const hatch = sim.default_car;

// on flat made-up tarmac
fn settled() sim.Sim {
    var s: sim.Sim = .{};
    s.world.plane_material = tarmac();
    for (0..100) |_| sim.sim_step(&s, &.{}, sim.d3_dt);
    return s;
}

fn tarmac() d3.material.Material {
    const all = d3.zon.parse([]d3.material.Material, gpa, data.materials) catch unreachable;
    defer gpa.free(all);
    return all[d3.material.find(all, "TSD*".*).?];
}

test "the car settles on its springs" {
    var s = settled();
    const height = s.game.d3.body.pos[1];
    for (0..25) |_| sim.sim_step(&s, &.{}, sim.d3_dt);
    const b = s.game.d3.body;
    try approx(height, b.pos[1], 1e-3);
    try approx(0, b.vel[1], 5e-3); // the tyre springs keep a small beat
    try approx(0, b.pos[0], 1e-3);
    try expect(b.pos[1] > 0.3 and b.pos[1] < 0.45); // on its wheels: the hubs 0.07 m under the body, radius 0.32 m
}

test "uneven substeps settle the car at the same height as even ones" {
    var even: sim.Sim = .{};
    even.world.plane_material = tarmac();
    even.car.substep_jitter = null;
    for (0..100) |_| sim.sim_step(&even, &.{}, sim.d3_dt);
    try approx(settled().game.d3.body.pos[1], even.game.d3.body.pos[1], 1e-4);
}

test "full throttle in 1st drives forward" {
    var s = settled();
    for (0..50) |_| sim.sim_step(&s, &.{ .throttle = 1, .gear = 1 }, sim.d3_dt);
    const v = s.game.d3.body.vel;
    try expect(v[2] > 5 and @abs(v[0]) < 0.03 * v[2]);
}

test "the car rests on a mesh and reports its surface" {
    var s: sim.Sim = .{};
    s.world.track = try d3.mesh.Mesh.testQuad(gpa);
    defer s.world.track.?.deinit();
    const flat = settled().game.d3.body.pos[1];
    for (0..100) |_| sim.sim_step(&s, &.{}, sim.d3_dt);
    try approx(flat, s.game.d3.body.pos[1], 5e-3);
}

test "the clutch drags the mean wheel spin toward the crank" {
    var st = std.mem.zeroes(d3.force.Step);
    st.ratio = d3.drivetrain.ratio(hatch, 2);
    st.clutch_cap = d3.drivetrain.clutchCap(hatch, 2, 0);
    st.engine_rate = 500;
    for (0..200) |_| d3.drivetrain.clutch(hatch, &st, 0.001);
    var mean: f32 = 0;
    for (st.wheels) |w| mean += w.spin / 4;
    try std.testing.expectApproxEqRel(st.engine_rate * st.ratio, mean, 1e-3);
}

test "a spring at travel 0 carries its static load" {
    const p = hatch.wheels[0];
    var w = std.mem.zeroes(d3.force.WheelStep);
    w.rebound_ratio = 1;
    const s = p.spring.linear;
    try std.testing.expectApproxEqRel(-s.rate * s.preload, d3.suspension.strutForce(p, w, 0), 1e-6);
}

test "a small steer turns both fronts the same way, the inner wheel more" {
    var wheels: [4]d3.force.Wheel = .{d3.force.Wheel{}} ** 4;
    for (0..50) |_| d3.steering.update(hatch, &wheels, -0.2, 20, 0.01);
    try expect(wheels[2].steer < 0 and wheels[3].steer < wheels[2].steer);
    try std.testing.expectEqual(@as(f32, 0), wheels[0].steer);
}

test "the grip weight follows its slope in each band of static loads" {
    const g = hatch.tyre_grip_by_load.?;
    try approx(1 + 0.5 * g[0], d3.tyre.gripWeight(hatch, 1.5), 1e-6);
    try approx(1 + g[0] + g[1], d3.tyre.gripWeight(hatch, 3), 1e-6);
    try approx(1 + g[0] + 2 * g[1] + 2 * g[2], d3.tyre.gripWeight(hatch, 6), 1e-6);
    var round = hatch;
    round.tyre_grip_by_load = null;
    try std.testing.expectEqual(@as(f32, 1), d3.tyre.gripWeight(round, 3));
}

test "a leaning tyre's lowest edge sits below a round tyre's; a round tyre has none" {
    const p = hatch.wheels[2];
    try expect(d3.tyre.contactOffset(p, 0) < 0);
    var round = p;
    round.visual_camber = null;
    try std.testing.expectEqual(@as(f32, 0), d3.tyre.contactOffset(round, 0));
}

test "full throttle gives the curve's power over the speed; a closed throttle brakes, more off idle" {
    const c = hatch.engine.curve.power;
    const rate = 12 * c.step;
    try std.testing.expectApproxEqRel(c.table[12] / c.divisor / rate, d3.engine.torque(hatch, rate, 1, false), 1e-6);
    try expect(d3.engine.torque(hatch, 300, 0, false) < d3.engine.torque(hatch, 200, 0, false));
    try expect(d3.engine.torque(hatch, 200, 0, false) < 0);
    try std.testing.expectEqual(@as(f32, 0), d3.engine.torque(hatch, 720, 1, false) - d3.engine.torque(hatch, 720, 0, false));
}

test "the hull pushes a sunk car out and stops its fall" {
    var t = try d3.mesh.Mesh.testQuad(gpa);
    defer t.deinit();
    var s: d3.body.State = .{ .pos = .{ 0, -0.05, 0 }, .axes = .{ .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, 1 } }, .vel = .{ 2, -3, 0 }, .angvel = @splat(0) };
    d3.chassis.step(hatch.rigid(), hatch.hull.?, &s, &t, 0.04);
    try expect(s.pos[1] > -0.05); // the position pass lifts it
    try expect(s.vel[1] > 0.5 and s.vel[1] <= 1); // the depth term pushes at up to 1 m/s
    try expect(s.vel[0] > 0 and s.vel[0] < 2); // friction slows the slide
}

const fit: d3.longitudinal.Fit = .{ .idle_rate = 110, .eta = 0.9, .drag = 0.8, .rolling = 300, .friction_torque = 30, .friction_per_rate = 0.1, .brake_torque = 4900 };

test "the road model: a standing car stays, neutral loses only drag and rolling, lower gears brake more" {
    const car = d3.longitudinal.of(hatch, fit);
    try std.testing.expectEqual(@as(f32, 0), d3.longitudinal.step(car, .{}, 0, 0.01));
    try std.testing.expectEqual(@as(f32, 0), d3.longitudinal.step(car, .{ .brake = 1, .gear = 1 }, 0.01, 0.01));
    const v: f32 = 30;
    const m = car.mass + car.wheel_inertia / (car.wheel_radius * car.wheel_radius);
    try std.testing.expectApproxEqRel(-(car.drag * v * v + car.rolling) / m, d3.longitudinal.accel(car, .{}, v), 1e-5);
    const fifth = d3.longitudinal.accel(car, .{ .gear = 5 }, 30);
    try expect(d3.longitudinal.accel(car, .{ .gear = 3 }, 30) < fifth);
    try expect(fifth < d3.longitudinal.accel(car, .{}, 30));
}

test "the road model reaches a top speed in 5th" {
    const car = d3.longitudinal.of(hatch, fit);
    const input: sim.Input = .{ .throttle = 1, .gear = 5 };
    var v: f32 = 20;
    for (0..120_000) |_| v = d3.longitudinal.step(car, input, v, 1.0 / 500.0);
    try approx(0, d3.longitudinal.accel(car, input, v), 1e-3);
    try expect(v > 40 and v < 80);
}

test "the made-up car and surfaces load from their files" {
    const c = try d3.zon.parse(Car, gpa, data.hatch);
    defer std.zon.parse.free(gpa, c);
    try std.testing.expectEqual(hatch.mass, c.mass);
    const ms = try d3.zon.parse([]d3.material.Material, gpa, data.materials);
    defer gpa.free(ms);
    try expect(d3.material.find(ms, "TSD*".*) != null);
}
