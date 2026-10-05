// The curve-table tyre step: what the tyre, the suspension and the car must do, on made-up cars
// (sim/data/cars/roadster.zon, wagon.zon).
const std = @import("std");
const fh = @import("fh");
const data = @import("data");
const tyre = fh.tyre;
const car = fh.car;
const V3 = fh.vec.V3;

const expect = std.testing.expect;
const approx = std.testing.expectApproxEqAbs;

// friction rises to 1 at sample 10 and falls to 0.9 at the end
fn shape() [tyre.samples]f32 {
    var s: [tyre.samples]f32 = undefined;
    for (&s, 0..) |*v, i| {
        const x: f32 = @floatFromInt(i);
        v.* = if (i <= 10) x / 10 else 1 - 0.1 * (x - 10) / 89;
    }
    return s;
}

fn curves(max_slip: f32) tyre.Curves {
    return .{ .max_slip = max_slip, .loads = .{ 100, 10000 }, .load_cap = 20000, .scale = .{ 1.2, 0.9 }, .shape = .{ shape(), shape() } };
}

const lat = tyre.table(curves(0.86));
const long = tyre.table(curves(1.1));
const front: tyre.Wheel = .{ .rear = false, .handbrake = false };

fn force(load: f32, v_long: f32, v_lat: f32, v_slip: f32) tyre.Out {
    return tyre.friction(lat, long, front, .{}, .{}, load, v_long, v_lat, v_slip, 0, 0, 20);
}

test "a rolling tyre without slip has no force" {
    const o = force(4000, 20, 0, 0);
    try approx(0, o.long, 1e-3);
    try approx(0, o.lat, 1e-3);
}

test "the tyre force works against the slip" {
    try expect(force(4000, 20, 0, 2).long < 0);
    try expect(force(4000, 20, 0, -2).long > 0);
    try expect(force(4000, 20, 1, 0).lat < 0);
    try expect(force(4000, 20, -1, 0).lat > 0);
}

test "pure slip at the peak gives the peak friction times the load" {
    const peak_sr = long.peak_slip[0] + (long.peak_slip[1] - long.peak_slip[0]) * (4000 - 100) / 9900.0;
    const o = force(4000, 20, 0, -peak_sr * 20);
    try approx(1, o.combined, 1e-3);
    try approx(o.long_max, o.long, 1);
}

test "combined slip shares one friction circle" {
    const both = force(4000, 20, 2, -2);
    const pure = force(4000, 20, 0, -2);
    try expect(@sqrt(both.long * both.long + both.lat * both.lat) <= both.long_max * 1.0001);
    try expect(@abs(both.long) < @abs(pure.long));
}

test "friction per load falls with load" {
    const light = force(1000, 20, 0, -2);
    const heavy = force(8000, 20, 0, -2);
    try expect(heavy.long / 8000 < light.long / 1000);
}

test "the load stops counting where load times friction peaks" {
    // scale 1.2 at 100 N to 0.9 at 10 kN: load x friction peaks near 19.9 kN, past it the force is flat
    try approx(force(25000, 20, 0, -2).long, force(30000, 20, 0, -2).long, 1e-3);
}

fn testCar() car.Params {
    const sp = fh.suspension.Wheel{
        .hub = .{ 0, 0, 0 },
        .radius = 0.33,
        .spring = 40000,
        .free_travel = 0.08,
        .bump = 3000,
        .rebound = 4000,
        .bump_cap = 20000,
        .rebound_cap = 20000,
        .bump_stop = 200000,
        .bump_stop_damping = 2000,
        .max_stretch = -0.12,
        .max_compress = 0.1,
        .tyre_max_compress = 0.01,
        .unsprung = 40,
    };
    var wheels: [4]car.Wheel = undefined;
    const at = [4][2]f32{ .{ -0.8, -1.3 }, .{ 0.8, -1.3 }, .{ -0.8, 1.3 }, .{ 0.8, 1.3 } };
    for (&wheels, at, 0..) |*w, xz, i| {
        var s = sp;
        s.hub = .{ xz[0], 0, xz[1] };
        w.* = .{ .suspension = s, .lateral = lat, .longitudinal = long, .inertia = 1.2, .brake = 1, .handbrake = if (i < 2) 1 else 0, .steered = i >= 2 };
    }
    const diff: fh.drivetrain.Diff = .{ .ratio = 3.5, .accel_lock = 0, .decel_lock = 0, .full_lock_torque = 1, .full_lock_slip = 1 };
    return .{
        .mass = 1300,
        .inertia = .{ 2000, 2300, 600 },
        .wheels = wheels,
        .bars = .{ .{ .stiffness = 10000, .damping = 0 }, .{ .stiffness = 15000, .damping = 0 } },
        .engine = .{ .inertia = 0.2, .idle = 85, .torque = &.{ 150, 250, 300, 300, 280, 240 }, .step = 150, .redline = 700, .max_speed = 750, .max_power_speed = 650, .braking = 50, .braking_low = 1 },
        .rev_limit = 720,
        .gearbox = box: {
            var gb: fh.gearbox.Params = .{ .gears = .{ 0, 3.4, 2.2, 0, 0, 0, 0, 0, 0, 0, -3.2 }, .top = 2, .shift_time = 0.2, .clutch_in = 10, .clutch_out = 10, .mode = .manual, .stall = 60, .idle = 85, .start = 300, .redline = 700 };
            fh.gearbox.shiftPoints(&gb);
            break :box gb;
        },
        .clutch_torque = 600,
        .line = .{ .gearbox_inertia = 0.05, .shaft_inertia = 0.02, .efficiency = 1 },
        .layout = .rear,
        .diffs = .{ diff, diff, diff },
        .final_drive = 3.5,
        .aero = .{ .forward_drag = 0.4, .vertical_drag = 0, .side_drag = .{ 0, 0 }, .downforce = .{ 0, 0 }, .zero_downforce_cos = 0, .wings = .{ null, null } },
        .steering = .{ .max = 0.6, .lock = @splat(0.6), .rates = null },
    };
}

fn run(p: car.Params, s: *car.State, in: car.Input, seconds: f32) void {
    const frames: usize = @intFromFloat(seconds * 60);
    for (0..frames) |_| car.step(p, s, in, .{ .planes = .{} }, 1.0 / 60.0);
}

fn dropped() struct { p: car.Params, s: car.State } {
    const p = testCar();
    const pose: fh.body.State = .{ .pos = .{ 0, 0.4, 0 }, .axes = .{ .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, 1 } }, .vel = @splat(0), .angvel = @splat(0) };
    return .{ .p = p, .s = car.rest(p, pose) };
}

test "the car settles on its springs and the wheels carry its weight" {
    var c = dropped();
    run(c.p, &c.s, .{}, 3);
    var load: f32 = 0;
    for (c.s.wheels) |w| load += w.load;
    try approx(c.p.mass * fh.suspension.g, load, c.p.mass * 0.05);
    try approx(0, c.s.body.vel[1], 0.01);
    for (c.s.wheels) |w| try expect(w.travel.contact);
}

test "full throttle in first gear drives the car forward" {
    var c = dropped();
    run(c.p, &c.s, .{ .gear = 1 }, 1);
    run(c.p, &c.s, .{ .throttle = 1 }, 2);
    try expect(c.s.body.vel[2] > 5);
    try approx(0, c.s.body.vel[0], 0.2);
}

test "the brakes stop a moving car" {
    var c = dropped();
    run(c.p, &c.s, .{}, 1);
    c.s.body.vel = .{ 0, 0, 20 };
    for (&c.s.wheels) |*w| w.spin = 20.0 / 0.33;
    run(c.p, &c.s, .{ .brake = 1 }, 4);
    try approx(0, c.s.body.vel[2], 0.3);
}

test "steering left turns the car left" {
    var c = dropped();
    run(c.p, &c.s, .{}, 1);
    c.s.body.vel = .{ 0, 0, 15 };
    for (&c.s.wheels) |*w| w.spin = 15.0 / 0.33;
    run(c.p, &c.s, .{ .steer = -0.3 }, 1);
    try expect(c.s.body.vel[0] < -0.5);
}

test "front and all-wheel drive also drive the car forward" {
    for ([_]car.Layout{ .front, .all }) |layout| {
        var c = dropped();
        c.p.layout = layout;
        run(c.p, &c.s, .{ .gear = 1 }, 1);
        run(c.p, &c.s, .{ .throttle = 1 }, 2);
        try expect(c.s.body.vel[2] > 5);
    }
}

test "in neutral the engine does not drive the wheels" {
    var c = dropped();
    run(c.p, &c.s, .{}, 1);
    run(c.p, &c.s, .{ .throttle = 1 }, 1);
    try approx(0, c.s.body.vel[2], 0.05);
    try expect(c.s.engine > c.p.engine.idle);
}

const Loaded = struct { p: car.Params, s: car.State, c: fh.Car };

// a made-up car from its file, at rest
fn fromFile(text: [:0]const u8) !Loaded {
    const c = try fh.zon.parse(fh.Car, std.testing.allocator, text);
    errdefer std.zon.parse.free(std.testing.allocator, c);
    const p = try fh.spec.params(c);
    const pose: fh.body.State = .{ .pos = .{ 0, 0.35, 0 }, .axes = .{ .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, 1 } }, .vel = @splat(0), .angvel = @splat(0) };
    return .{ .p = p, .s = car.rest(p, pose), .c = c };
}

fn roadster() !Loaded {
    return fromFile(data.roadster);
}

test "a car from its file sits at its ride height with its weight on the springs" {
    for ([_][:0]const u8{ data.roadster, data.wagon }) |text| try sitsAtRideHeight(text);
}

fn sitsAtRideHeight(text: [:0]const u8) !void {
    var m = try fromFile(text);
    defer std.zon.parse.free(std.testing.allocator, m.c);
    run(m.p, &m.s, .{}, 3);
    var load: f32 = 0;
    for (m.s.wheels) |w| load += w.load;
    try approx(m.p.mass * fh.suspension.g, load, m.p.mass * 0.03);
    // the springs were set so that the hubs sit at ride height under the static load
    for (m.s.wheels, m.p.wheels) |w, pw| try approx(pw.suspension.hub[1], w.travel.y, 0.02);
    const on_front = m.s.wheels[2].load + m.s.wheels[3].load;
    try approx(m.c.front_weight.?, on_front / load, 0.03);
}

test "a car from its file accelerates, shifts up on its own and brakes to a stop" {
    for ([_][:0]const u8{ data.roadster, data.wagon }) |text| try accelerates(text);
}

fn accelerates(text: [:0]const u8) !void {
    var m = try fromFile(text);
    defer std.zon.parse.free(std.testing.allocator, m.c);
    m.p.gearbox.mode = .automatic;
    fh.gearbox.shiftPoints(&m.p.gearbox);
    run(m.p, &m.s, .{ .gear = 1 }, 1);
    run(m.p, &m.s, .{ .throttle = 1 }, 10);
    const top = m.s.body.vel[2];
    try expect(top > 25); // about 100 km/h in 10 s
    try expect(m.s.box.gear >= 3);
    run(m.p, &m.s, .{ .brake = 1 }, 6);
    try approx(0, m.s.body.vel[2], 0.5);
}

// a flat 100 m square of one material
fn square(m: fh.material.Material) !fh.mesh.Mesh {
    const T = fh.mesh.Triangle;
    const quad = [2]T{
        .fromCorners(.{ .{ -50, 0, -50 }, .{ -50, 0, 50 }, .{ 50, 0, -50 } }, 0),
        .fromCorners(.{ .{ 50, 0, -50 }, .{ -50, 0, 50 }, .{ 50, 0, 50 } }, 0),
    };
    return fh.mesh.Mesh.init(std.testing.allocator, &quad, &.{m});
}

// m/s lost in 1 s of full braking from 20 m/s
fn braking(track: *const fh.mesh.Mesh) !f32 {
    var m = try roadster();
    defer std.zon.parse.free(std.testing.allocator, m.c);
    m.s.body.pos[2] = -40;
    for (0..180) |_| car.step(m.p, &m.s, .{ .gear = 1 }, .{ .track = track }, 1.0 / 60.0);
    m.s.body.vel = .{ 0, 0, 20 };
    for (&m.s.wheels, m.p.wheels) |*w, pw| w.spin = 20 / pw.suspension.radius;
    for (0..60) |_| car.step(m.p, &m.s, .{ .brake = 1 }, .{ .track = track }, 1.0 / 60.0);
    return 20 - m.s.body.vel[2];
}

test "the surface's curve grip sets the tyre friction; another model's grip does not" {
    const name = fh.material.named("fh", "SNO+".*);
    const slippery: fh.material.Material = .{ .name = name, .depth = 0, .bumps = fh.material.smooth(.by_distance), .slowdown = .{}, .curve_grip = .{ .friction = 0.3, .off_road = 1, .peak_slip_angle = 0.7, .rear = 1, .handbrake = 0.5, .min_arcade = 0 } };
    const tarmac: fh.material.Material = .{ .name = fh.material.named("d3", "TSD*".*), .depth = 0, .bumps = fh.material.smooth(.by_time), .slowdown = .{}, .grip = 2 };
    var snow = try square(slippery);
    defer snow.deinit();
    var road = try square(tarmac);
    defer road.deinit();
    const on_snow = try braking(&snow);
    const on_road = try braking(&road);
    try expect(on_snow < 0.5 * on_road);
    try expect(on_road > 9.81 and on_road < 25); // the tyre's own grip without a curve grip: over 1 g with its extra grip, not twice that
}

fn drive(m: anytype, in: car.Input, seconds: f32) void {
    run(m.p, &m.s, in, seconds);
}

test "a manual box changes gear on request in 0.75 of the automatic times" {
    var m = try roadster();
    defer std.zon.parse.free(std.testing.allocator, m.c);
    m.p.gearbox.mode = .manual;
    drive(&m, .{ .gear = 1 }, 1);
    drive(&m, .{ .gear = 1, .throttle = 1 }, 3);
    try std.testing.expectEqual(1, m.s.box.gear);
    const g = m.p.gearbox;
    const clutch_in = 0.75 / g.clutch_in; // s to open the clutch
    var t: f32 = 0;
    while (m.s.box.gear != 2 and t < 2) : (t += 1.0 / 60.0) drive(&m, .{ .gear = 2, .throttle = 1 }, 1.0 / 60.0);
    try std.testing.expectEqual(2, m.s.box.gear);
    try approx(clutch_in + 0.75 * g.shift_time, t, 2.5 / 60.0);
}

test "with the clutch pedal: a launch, a shift, then the clutch dumped at idle in third stalls the engine for 2 s" {
    var m = try roadster();
    defer std.zon.parse.free(std.testing.allocator, m.c);
    m.p.gearbox.mode = .clutch_pedal;
    drive(&m, .{ .gear = 1, .clutch = 1 }, 1);
    // launch: revs up, the pedal comes up over 1 s
    var k: f32 = 0;
    while (k < 1) : (k += 1.0 / 60.0) drive(&m, .{ .gear = 1, .throttle = 0.7, .clutch = 1 - k }, 1.0 / 60.0);
    drive(&m, .{ .gear = 1, .throttle = 1 }, 2);
    try expect(m.s.body.vel[2] > 5);
    try expect(m.s.box.phase != .stalled);
    // a shift with the pedal
    drive(&m, .{ .gear = 2, .clutch = 1 }, 0.5);
    drive(&m, .{ .gear = 2, .throttle = 1 }, 1);
    try std.testing.expectEqual(2, m.s.box.gear);
    // stop, third, pedal down, then let it up at once with no throttle
    drive(&m, .{ .gear = 2, .clutch = 1, .brake = 1 }, 4);
    drive(&m, .{ .gear = 3, .clutch = 1, .brake = 1 }, 1);
    try std.testing.expectEqual(3, m.s.box.gear);
    drive(&m, .{ .gear = 3 }, 0.5);
    try std.testing.expectEqual(.stalled, m.s.box.phase);
    try approx(0, m.s.body.vel[2], 0.5);
    // the throttle does nothing while stalled; after 2 s, pedal down, in first, it pulls away again
    drive(&m, .{ .gear = 3, .throttle = 1 }, 1);
    try approx(0, m.s.body.vel[2], 0.5);
    drive(&m, .{ .gear = 1, .clutch = 1 }, 2);
    k = 0;
    while (k < 1) : (k += 1.0 / 60.0) drive(&m, .{ .gear = 1, .throttle = 0.7, .clutch = 1 - k }, 1.0 / 60.0);
    drive(&m, .{ .gear = 1, .throttle = 1 }, 1);
    try expect(m.s.body.vel[2] > 3);
}
