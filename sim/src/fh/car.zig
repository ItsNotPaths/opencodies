// The vehicle step: n equal substeps of semi-implicit Euler over the body, the wheel travels and spins, the
// steering, the engine and the clutch. FH2 CCarDynamics::Update 0x82e35338, IntegrateEuler 0x82e352a0,
// CalcForceTorque 0x82e341d0, CalcFrictionForceTorque 0x82e31bf8, StateStep 0x82e26360.
const std = @import("std");
const vec = @import("../shared/vec.zig");
const body = @import("../shared/body.zig");
const ground = @import("../shared/ground.zig");
const Material = @import("../material.zig").Material;
const CurveGrip = @import("../material.zig").CurveGrip;
const Axles = @import("../car.zig").Axles;
const FreeGrip = @import("../car.zig").FreeGrip;
const tyre = @import("tyre.zig");
const suspension = @import("suspension.zig");
const drivetrain = @import("drivetrain.zig");
const aero = @import("aero.zig");
const steering = @import("steering.zig");
const gearbox = @import("gearbox.zig");

const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;
const cross = vec.cross;
const g = suspension.g;

pub const rate = 359; // Hz, PhysicsSettings.ini CarPhysicsUpdateHz

// game-wide constants from PhysicsSettings.ini
const gravity_scale = 1.0; // CarGravityScale
const airborne_gravity_scale = 1.15;
const off_ground_gravity_scale = 1.05; // per wheel off the ground
const brake_input = [2][4]f32{ .{ 0, 0.001, 0.9, 1 }, .{ 0, 0.12, 0.7, 0.81 } }; // BrakeInputScale
const rev_cut_time = 0.04; // s without throttle past the rev limit

pub const Wheel = struct {
    suspension: suspension.Wheel,
    lateral: tyre.Table,
    longitudinal: tyre.Table,
    lateral_grip: f32 = 1,
    longitudinal_grip: f32 = 1,
    free_grip: ?FreeGrip = null,
    inertia: f32,
    brake: f32, // brake pressure at full pedal: 1 stops the car at 1 g on four wheels
    handbrake: f32, // added pressure at full handbrake
    steered: bool,
};

pub const Layout = enum { rear, front, all };

pub const Params = struct {
    mass: f32,
    inertia: V3, // body frame diagonal
    wheels: [4]Wheel, // RL RR FL FR
    bars: [2]suspension.Bar, // rear, front
    rear_slide: ?tyre.RearSlide = null,
    engine: drivetrain.Engine,
    rev_limit: f32,
    gearbox: gearbox.Params,
    clutch_torque: f32,
    line: drivetrain.Line,
    layout: Layout,
    diffs: [3]drivetrain.Diff, // rear, front, centre (all-wheel drive only)
    final_drive: f32,
    aero: Axles,
    steering: steering.Params,
};

pub const WheelState = struct {
    travel: suspension.Travel,
    spin: f32 = 0, // rad/s
    angle: f32 = 0,
    material: ?*const Material = null,
    load: f32 = 0,
    pressure: f32 = 0, // brake
    out: ?tyre.Out = null,
};

pub const State = struct {
    body: body.State,
    momentum: V3,
    steer: steering.State = .{},
    engine: f32, // rad/s
    box: gearbox.State = .{},
    rev_cut: f32 = 0, // s of throttle cut left
    wheels: [4]WheelState,
};

pub const Input = struct {
    throttle: f32 = 0,
    brake: f32 = 0,
    handbrake: f32 = 0,
    steer: f32 = 0, // + turns toward body +x
    gear: ?usize = null, // a request: 0 neutral, 1.., 10 reverse
    clutch: f32 = 0, // the pedal, 0 up .. 1 down; only the clutch pedal box reads it
};

pub fn rest(p: Params, pose: body.State) State {
    var s: State = .{ .body = pose, .momentum = splat(0), .engine = p.engine.idle, .wheels = undefined };
    for (&s.wheels, p.wheels) |*w, pw| w.* = .{ .travel = .{ .y = pw.suspension.hub[1], .vel = 0 } };
    s.body.vel = splat(0);
    s.body.angvel = splat(0);
    return s;
}

// one frame of dt seconds
pub fn step(p: Params, s: *State, in: Input, gr: ground.Ground, dt: f32) void {
    const frame = std.math.clamp(dt, -0.1, 0.1);
    const n: u32 = @max(1, @as(u32, @intFromFloat(@ceil(@abs(frame) * rate))));
    const h = frame / @as(f32, @floatFromInt(n));
    keepOnGround(p, s, gr);
    var slip_ok = true;
    for (s.wheels) |w| if (w.out) |o| {
        slip_ok = slip_ok and @abs(o.norm_slip_ratio) <= 1.1;
    };
    var input = in;
    const b = s.body;
    input.throttle = gearbox.update(p.gearbox, &s.box, .{
        .request = in.gear,
        .throttle = in.throttle,
        .pedal = in.clutch,
        .engine = s.engine,
        .shaft = shaftSpeed(p, s.*),
        .forward_speed = dot(b.vel, b.axes[2]),
        .slip_ok = slip_ok,
    }, @abs(frame));
    if (s.box.phase == .stalled) s.engine = @max(s.engine, p.gearbox.stall);
    for (0..n) |_| substep(p, s, input, gr, h);
}

const Deriv = struct {
    force: V3,
    torque: V3,
    engine_acc: f32,
    engine_torque: f32,
    steer_vel: f32,
    wheel_acc: [4]f32, // along up
    spin_acc: [4]f32,
};

fn substep(p: Params, s: *State, in: Input, gr: ground.Ground, h: f32) void {
    var input = in;
    if (s.engine > p.rev_limit) s.rev_cut = rev_cut_time;
    if (s.rev_cut > 0) {
        input.throttle = 0;
        s.rev_cut -= h;
    }
    const d = derivative(p, s, input, h);
    integrate(p, s, d, h);
    for (&s.wheels, p.wheels) |*w, pw| suspension.clampTravel(s.body, pw.suspension, &w.travel);
    const shaft = shaftSpeed(p, s.*);
    if (s.box.locked) s.engine = p.gearbox.gears[s.box.gear] * shaft;
    keepOnGround(p, s, gr);
    gearbox.lock(p.gearbox, &s.box, clutchCapacity(p, s.*, d.engine_torque), d.engine_torque, s.engine, shaft);
}

// after something else moved the body (the hull against the world): its momentum from its new spin
pub fn syncMomentum(p: Params, s: *State) void {
    s.momentum = vec.toWorld(s.body.axes, vec.toBody(s.body.axes, s.body.angvel) * p.inertia);
}

fn integrate(p: Params, s: *State, d: Deriv, h: f32) void {
    const b = &s.body;
    b.vel += d.force * splat(h / p.mass);
    s.momentum += d.torque * splat(h);
    b.angvel = vec.toWorld(b.axes, vec.toBody(b.axes, s.momentum) / p.inertia);
    const w2 = dot(b.angvel, b.angvel);
    if (w2 > 36) {
        b.angvel *= splat(6 / @sqrt(w2));
        s.momentum = vec.toWorld(b.axes, vec.toBody(b.axes, b.angvel) * p.inertia);
    }
    b.* = body.advance(b.*, h);
    s.steer.angle = std.math.clamp(s.steer.angle + d.steer_vel * h, -p.steering.max, p.steering.max);
    s.engine += d.engine_acc * h;
    for (&s.wheels, p.wheels, 0..) |*w, pw, i| {
        const t = &w.travel;
        t.vel += d.wheel_acc[i] * h;
        const at = suspension.worldPoint(b.*, suspension.mount(pw.suspension, t.*));
        t.y += (t.vel - dot(body.pointVelocity(b.*, at), b.axes[1])) * h;
        // a hard-braked wheel below pi rad/s stops
        if (@abs(w.spin) <= std.math.pi and w.pressure >= 0.6) w.spin = 0 else w.spin += d.spin_acc[i] * h;
        w.angle = wrapPi(w.angle + w.spin * h);
        t.tyre = @max(t.tyre - 5 * h, 0);
    }
}

fn keepOnGround(p: Params, s: *State, gr: ground.Ground) void {
    for (&s.wheels, p.wheels, 0..) |*w, pw, i| {
        const found = probe(s.body, pw.suspension, gr, i);
        w.material = if (found) |f| f.material else null;
        suspension.keepOnGround(s.body, pw.suspension, &w.travel, if (found) |f| f.ground else null);
    }
}

const Found = struct { ground: suspension.Ground, material: *const Material };

// a ray down the body up axis through the hub, from above full compression to a little below full droop
fn probe(b: body.State, w: suspension.Wheel, gr: ground.Ground, i: usize) ?Found {
    const top = w.max_compress + w.radius;
    const bottom = w.max_stretch - w.radius - 0.05;
    const from = suspension.worldPoint(b, .{ w.hub[0], top, w.hub[2] });
    const to = suspension.worldPoint(b, .{ w.hub[0], bottom, w.hub[2] });
    const hit = gr.cast(i, from, to) orelse return null;
    if (dot(hit.normal, b.axes[1]) < 0) return null;
    return .{ .ground = .{ .height = top + hit.t * (bottom - top), .normal = hit.normal }, .material = hit.material };
}

fn derivative(p: Params, s: *State, in: Input, h: f32) Deriv {
    const b = s.body;
    var d: Deriv = .{ .force = splat(0), .torque = splat(0), .engine_acc = 0, .engine_torque = 0, .steer_vel = 0, .wheel_acc = @splat(0), .spin_acc = @splat(0) };

    var on_ground: f32 = 0;
    for (s.wheels) |w| on_ground += @floatFromInt(@intFromBool(w.travel.contact));
    const airborne: f32 = if (on_ground == 0) airborne_gravity_scale - 1 else 0;
    const scale = gravity_scale + (off_ground_gravity_scale - 1) * (4 - on_ground) + airborne;
    d.force[1] -= scale * g * p.mass;

    const air = aero.forces(p.aero, b, .{ axleZ(p, 0), axleZ(p, 1) });
    d.force += air.force;
    d.torque += air.torque;

    for (&s.wheels) |*w| {
        w.travel.sway_spring = 0;
        w.travel.sway_damping = 0;
    }
    for (0..2) |axle| {
        var pair: [2]suspension.Travel = .{ s.wheels[axle * 2].travel, s.wheels[axle * 2 + 1].travel };
        suspension.swayBar(b, p.bars[axle], .{ p.wheels[axle * 2].suspension, p.wheels[axle * 2 + 1].suspension }, &pair);
        s.wheels[axle * 2].travel = pair[0];
        s.wheels[axle * 2 + 1].travel = pair[1];
    }

    var long: [4]f32 = @splat(0);
    for (&s.wheels, p.wheels, 0..) |*w, pw, i| {
        const nf = suspension.normalForce(b, pw.suspension, w.travel);
        w.load = nf.load;
        d.force += nf.force;
        d.torque += nf.torque;
        d.wheel_acc[i] = nf.acc;
        const f = friction(p, s, pw, w, i, in);
        d.force += f.force;
        d.torque += f.torque;
        long[i] = if (w.out) |o| o.long else 0;
    }
    lowSpeedHold(p, s.*, in, h, &d);

    d.steer_vel = steerRate(p, s, in, h);
    drive(p, s, in, long, &d);
    return d;
}

fn axleZ(p: Params, axle: usize) f32 {
    return (p.wheels[axle * 2].suspension.hub[2] + p.wheels[axle * 2 + 1].suspension.hub[2]) * 0.5;
}

fn steerRate(p: Params, s: *State, in: Input, h: f32) f32 {
    const b = s.body;
    var slip: f32 = 0;
    var load: f32 = 0;
    for (s.wheels[2..4]) |w| if (w.out) |o| {
        slip += o.norm_slip_angle * w.load;
        load += w.load;
    };
    const nsa = [2]f32{ if (s.wheels[2].out) |o| o.norm_slip_angle else 0, if (s.wheels[3].out) |o| o.norm_slip_angle else 0 };
    // the game's rule, mirrored: its x points to the car's right, the sim's to its left
    const opposed = if (s.steer.angle < 0) nsa[0] > 0 else nsa[1] < 0;
    const speed = @sqrt(dot(b.vel, b.vel));
    return steering.rate(p.steering, &s.steer, in.steer, dot(b.vel, b.axes[2]), speed, slip / @max(load, 1e-7), opposed, h);
}

const Push = struct { force: V3, torque: V3 };

// the tyre force on the ground plane at the contact
fn friction(p: Params, s: *State, pw: Wheel, w: *WheelState, i: usize, in: Input) Push {
    const b = s.body;
    const t = w.travel;
    if (!t.contact) {
        w.out = null;
        return .{ .force = splat(0), .torque = splat(0) };
    }
    const sp = pw.suspension;
    const up = b.axes[1];
    const centre = suspension.worldPoint(b, suspension.mount(sp, t));
    const at = centre - up * splat(sp.radius);
    const n = t.normal;
    const yaw = if (pw.steered) steering.ackermann(s.steer.angle, axleZ(p, 1) - axleZ(p, 0), sp.hub[0]) else 0;
    const heading = vec.toWorld(b.axes, .{ @sin(yaw), 0, @cos(yaw) });
    const fwd = vec.normalize(heading - n * splat(dot(heading, n)));
    const right = cross(n, fwd);
    const v = body.pointVelocity(b, at);
    const v_long = dot(v, fwd);
    const v_lat = dot(v, right);
    const v_slip = v_long - w.spin * sp.radius;

    const cg = curveGrip(w.material);
    const surface: tyre.Surface = .{ .friction = cg.friction, .off_road = cg.off_road, .peak_slip_angle = cg.peak_slip_angle, .rear_grip = cg.rear, .min_arcade = cg.min_arcade, .handbrake_grip = cg.handbrake };
    const grip = cg.friction;
    // GetTorqueFreeLat/LongFricScale*: driving when the tread runs ahead of the ground
    var free_lat: f32 = 1;
    var free_long: f32 = 1;
    if (pw.free_grip) |fg| {
        free_lat = fg.lateral;
        const speed = @sqrt(dot(b.vel, b.vel));
        const k = std.math.clamp((speed - fg.driving_speeds[0]) / (fg.driving_speeds[1] - fg.driving_speeds[0]), 0, 1);
        free_long = if (w.spin * v_slip < 0) (fg.driving[1] - fg.driving[0]) * k + fg.driving[0] else fg.braking;
    }
    const scales: tyre.Grip = .{ .long = grip * pw.longitudinal_grip * free_long, .lat = grip * pw.lateral_grip * free_lat };
    const kind: tyre.Wheel = .{ .rear = i < 2, .handbrake = pw.handbrake > 0, .rear_slide = p.rear_slide };
    const out = tyre.friction(pw.lateral, pw.longitudinal, kind, surface, scales, w.load, v_long, v_lat, v_slip, s.steer.angle, in.handbrake, @sqrt(dot(b.vel, b.vel)));
    w.out = out;
    const f = fwd * splat(out.long) + right * splat(out.lat);
    const turning = fwd * splat(out.long / free_long) + right * splat(out.lat / free_lat); // [likely]
    return .{ .force = f, .torque = cross(at - b.pos, turning) };
}

// [likely, force simplified] below 2 m/s on four wheels with the throttle off, the tyres hold the car where
// their grip allows: sideways always, along the car on the brake or handbrake. The force acts at the centre of mass.
fn lowSpeedHold(p: Params, s: State, in: Input, h: f32, d: *Deriv) void {
    const b = s.body;
    if (dot(b.vel, b.vel) > 4 or dot(b.angvel, b.angvel) > 0.09 or b.axes[1][1] < 0.7 or in.throttle > 0) return;
    var load: f32 = 0;
    var grip: f32 = 0;
    for (s.wheels) |w| {
        if (!w.travel.contact or @abs(w.spin) > 2) return;
        load += w.load;
        grip += w.load * curveGrip(w.material).friction;
    }
    if (load <= 0) return;
    const mu = grip / load;
    for ([2]V3{ b.axes[0], b.axes[2] }, 0..) |axis, k| {
        if (k == 1 and !(in.brake > 0.787 or in.handbrake > 0.1)) continue;
        const need = -(dot(b.vel, axis) * p.mass / h + dot(d.force, axis));
        if (@abs(need / p.mass) / g < mu) d.force += axis * splat(need);
    }
}

// no surface effect: the tyre's own curves (peak_slip_angle does nothing at off_road 0)
const plain: CurveGrip = .{ .friction = 1, .off_road = 0, .peak_slip_angle = 0, .rear = 1, .handbrake = 1, .min_arcade = 0 };

fn curveGrip(m: ?*const Material) CurveGrip {
    const mat = m orelse return plain;
    return mat.curve_grip orelse plain;
}

fn clutchCapacity(p: Params, s: State, engine_torque: f32) f32 {
    if (s.box.gear == gearbox.neutral) return 0;
    return gearbox.engagement(p.gearbox, s.box) * @max(@abs(engine_torque) + 4, p.clutch_torque);
}

fn drive(p: Params, s: *State, in: Input, long: [4]f32, d: *Deriv) void {
    var torque: [4]f32 = undefined;
    var spin: [4]f32 = undefined;
    var inertia: [4]f32 = undefined;
    var radius: [4]f32 = undefined;
    for (p.wheels, &s.wheels, 0..) |pw, *w, i| {
        w.pressure = lerp4(brake_input, in.brake) * pw.brake + pw.handbrake * in.handbrake;
        radius[i] = pw.suspension.radius;
        torque[i] = drivetrain.brake(p.mass, radius[i], w.pressure, w.spin) - radius[i] * long[i];
        spin[i] = w.spin;
        inertia[i] = pw.inertia;
    }
    const ratio = p.gearbox.gears[s.box.gear];
    const t_engine = drivetrain.engineTorque(p.engine, s.engine, in.throttle);
    const slip = s.engine - ratio * shaftSpeed(p, s.*);
    const dr: drivetrain.Drive = .{
        .ratio = ratio,
        .locked = s.box.locked,
        .engine = t_engine,
        .clutch = if (ratio == 0) 0 else drivetrain.clutch(t_engine, p.clutch_torque, gearbox.engagement(p.gearbox, s.box), slip),
        .engine_inertia = p.engine.inertia,
    };
    const order = wheelOrder(p.layout);
    const r = switch (p.layout) {
        .all => drivetrain.allWheel(p.diffs[1], p.diffs[0], p.diffs[2], p.final_drive, p.line, dr, pick(torque, order), pick(spin, order), pick(inertia, order), pick(radius, order)),
        .rear => drivetrain.twoWheel(p.diffs[0], p.line, dr, pick(torque, order), pick(spin, order), pick(inertia, order)),
        .front => drivetrain.twoWheel(p.diffs[1], p.line, dr, pick(torque, order), pick(spin, order), pick(inertia, order)),
    };
    for (order, r.spin_acc) |i, a| d.spin_acc[i] = a;
    d.engine_acc = r.engine_acc;
    d.engine_torque = t_engine;
}

// the solves want the driven or front pair first
fn wheelOrder(l: Layout) [4]usize {
    return if (l == .rear) .{ 0, 1, 2, 3 } else .{ 2, 3, 0, 1 };
}

fn pick(v: [4]f32, order: [4]usize) [4]f32 {
    return .{ v[order[0]], v[order[1]], v[order[2]], v[order[3]] };
}

// the gearbox output speed from the driven wheels
pub fn shaftSpeed(p: Params, s: State) f32 {
    const w = s.wheels;
    return switch (p.layout) {
        .rear => (w[0].spin + w[1].spin) * 0.5 * p.final_drive,
        .front => (w[2].spin + w[3].spin) * 0.5 * p.final_drive,
        .all => ((w[0].spin + w[1].spin) * p.diffs[0].ratio * 0.5 + (w[2].spin + w[3].spin) * p.diffs[1].ratio * 0.5) * 0.5,
    };
}

fn lerp4(c: [2][4]f32, x: f32) f32 {
    if (x <= c[0][0]) return c[1][0];
    for (1..4) |k| if (x < c[0][k]) {
        const f = (x - c[0][k - 1]) / (c[0][k] - c[0][k - 1]);
        return (c[1][k] - c[1][k - 1]) * f + c[1][k - 1];
    };
    return c[1][3];
}

fn wrapPi(a: f32) f32 {
    return a - 2 * std.math.pi * @round(a / (2 * std.math.pi));
}
