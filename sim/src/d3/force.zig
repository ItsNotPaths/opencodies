const std = @import("std");
const body = @import("../shared/body.zig");
const bumps = @import("../shared/bumps.zig");
const vec = @import("../shared/vec.zig");
const aero = @import("aero.zig");
const coupling = @import("../shared/coupling.zig");
const drivetrain = @import("drivetrain.zig");
const engine = @import("engine.zig");
const integrate = @import("integrate.zig");
const ground = @import("../shared/ground.zig");
const slowdown = @import("slowdown.zig");
const steering = @import("steering.zig");
const material = @import("../material.zig");
const suspension = @import("suspension.zig");
const tyre = @import("tyre.zig");
const Input = @import("../sim.zig").Input;

const car_ = @import("car.zig");
const Car = @import("../car.zig").Car;
const CarWheel = @import("../car.zig").Wheel;
const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;

const even = [1]f32{1}; // substeps of one length

pub const Wheel = extern struct {
    travel: f32 = 0, // m, hub along the suspension axis
    travel_rate: f32 = 0,
    spin: f32 = 0, // rad/s
    mean_spin: f32 = 0, // over the last step
    carcass: [2]f32 = .{ 0, 0 }, // m, lateral and longitudinal deflection
    anchor_rate: [2]f32 = .{ 0, 0 }, // m/s the anchor slipped at
    anchor: V3 = splat(0), // world point the carcass hangs on, see rest()
    ground: f32 = ground.none, // contact height predicted for the end of the last step
    ground_reached: f32 = ground.none, // where the substeps took it
    normal: V3 = .{ 0, 1, 0 }, // of the last contact query
    softness: f32 = 1, // 1/hardness of the last surface hit
    air: f32 = 0, // timer, +2/s in the air, -1/s on the ground, 0..1
    steer: f32 = 0, // sin of the wheel angle
    abs: bool = false,
    contact: bool = false, // ray hit and the tyre touched in the last step
    ray_offset: V3 = splat(0), // car+0x2410, body frame: wheel damage moves the contact ray (0x7ed6a0)
    lift: f32 = 0, // W+0x78, the drawn wheel above travel, tyre.visualLift
    contact_offset: f32 = 0, // W+0x16c, tyre.contactOffset
};

pub const Engine = extern struct {
    rate: f32 = 0, // rad/s
    mean: f32 = 0, // over the last step
    reacted: f32 = 0, // mean seen by the reaction torque
};

pub const State = extern struct {
    body: body.State,
    wheels: [4]Wheel = .{Wheel{}} ** 4,
    engine: Engine = .{},
    jitter: u32 = 0,
    time: f32 = 0, // s, drives the road bumps
    air_yaw: f32 = 1, // tyre yaw inertia multiple, grows in the air
    shift: drivetrain.Shift = .{},
};

pub const Sum = extern struct {
    full: f32 = 0,
    weighted: f32 = 0, // each substep impulse times the share of the step left after it

    pub fn add(s: *Sum, j: f32, frac: f32) void {
        s.full += j;
        s.weighted += j * frac;
    }
};

pub const Dir = enum { axis, lat, lon };

pub const Response = coupling.Response;

pub const WheelStep = extern struct { // one wheel block of T = car+0x690
    point: V3, // contact point
    axis: V3, // suspension axis
    lat: V3,
    lon: V3, // rolling direction
    response: [3]Response, // per unit impulse along axis, lat, lon at this wheel
    bottoming: [4]f32,
    travel: f32,
    travel_rate: f32,
    axis_speed: f32, // body speed along the axis at the wheel
    axis_accel: f32, // of the force code, spread over the step
    ground: f32,
    ground_rate: f32,
    lat_speed: f32,
    lon_slip: f32, // rolling speed minus r * spin
    spin: f32,
    carcass: [2]f32,
    anchor_rate: [2]f32,
    normal: V3, // ground
    softness: f32, // scales the slip angle at the peak
    drag: material.Drag,
    slowdown: material.Slowdown,
    spin_sum: f32,
    rebound_ratio: f32,
    mu: f32,
    brake: f32, // spin change per substep
    abs: bool,
    touched: bool,
    suspension: Sum,
    tyre_normal: Sum,
    tyre_lat: Sum,
    tyre_lon: Sum,
    visual_bump: f32, // the part of the bumps that moves only the drawn wheel

    pub fn rows(w: WheelStep, dir: Dir) Response {
        return w.response[@intFromEnum(dir)];
    }

    pub fn turn(w: *WheelStep, dw: f32, radius: f32) void {
        w.spin += dw;
        w.lon_slip -= radius * dw;
    }
};

pub const Step = extern struct {
    wheels: [4]WheelStep,
    accel: V3, // (v - v0)/dt of the force code
    ang_accel: V3,
    tyre_inertia: V3,
    engine_torque: f32,
    ratio: f32,
    clutch_cap: f32,
    clutch: f32,
    diff_throttle: f32,
    engine_rate: f32,
    engine_sum: f32,
    handbrake: bool,
};

// anchors under the contact patches, as after a reset
pub fn rest(car: Car, s: *State) void {
    for (&s.wheels, car.wheels) |*w, p| w.anchor = tyre.patchPoint(p, w.travel, s.body);
}

// 0x8007b0
pub fn step(car: Car, s: *State, input: Input, g: ground.Ground, dt: f32) void {
    const speed = vec.length(s.body.vel); // car+0x2404
    s.time += dt;
    s.air_yaw = tyre.airYaw(car, s.air_yaw, airborne(s.wheels), speed, dt);
    steering.update(car, &s.wheels, input.steer, speed, dt);
    var moved = s.body;
    aero.apply(car, &moved, s.wheels, dt);
    engine.react(car, &moved, &s.engine, s.wheels);
    anchor(car, s, dt);
    const n = @max(1, @as(usize, @intFromFloat(dt * 999.99994)));
    const share = 1 / @as(f32, @floatFromInt(n));
    const h = share * dt;
    var st = setup(car, s, moved, input, g, speed, h, dt);
    var frac: f32 = 1;
    const lengths = car.substep_jitter orelse &even;
    for (0..n) |_| {
        s.jitter = (s.jitter + 1) % @as(u32, @intCast(lengths.len));
        substep(car, &st, lengths[s.jitter] * h, frac);
        frac -= share;
    }
    finish(car, s, st, share, speed, dt);
}

// 0x928b30, the wheel draw update: once per frame, after the steps of the frame
pub fn drawWheels(car: Car, s: *State) void {
    for (&s.wheels, car.wheels) |*w, p| w.contact_offset = tyre.contactOffset(p, @min(w.lift + w.travel, 0.1)); // car+0x2560
}

pub fn anchor(car: Car, s: *State, dt: f32) void {
    for (&s.wheels, car.wheels) |*w, p| tyre.anchor(p, w, s.body, dt);
}

fn airborne(wheels: [4]Wheel) bool {
    for (wheels) |w| if (w.contact) return false;
    return true;
}

// 0x7fd240 up to the substeps; s.body is the state before the force code, moved after it
pub fn setup(car: Car, s: *State, moved: body.State, input: Input, g: ground.Ground, speed: f32, h: f32, dt: f32) Step {
    const now = s.body;
    const predicted = body.advance(now, dt);
    var st: Step = .{
        .wheels = undefined,
        .accel = (moved.vel - now.vel) * splat(1 / dt),
        .ang_accel = (moved.angvel - now.angvel) * splat(1 / dt),
        .tyre_inertia = tyre.inertia(car, speed, s.air_yaw),
        .engine_torque = engine.torque(car, s.engine.mean, input.throttle, input.gear == 1 and dot(now.axes[2], now.vel) < 0),
        .ratio = drivetrain.shift(car, &s.shift, input.gear, dt),
        .clutch_cap = drivetrain.clutchCap(car, input.gear, input.clutch),
        .clutch = input.clutch,
        .diff_throttle = drivetrain.diffThrottle(car, input.throttle),
        .engine_rate = s.engine.rate,
        .engine_sum = 0,
        .handbrake = input.handbrake,
    };
    for (&st.wheels, &s.wheels, car.wheels, 0..) |*w, *state, p, i| {
        const geo = suspension.geometry(now, p, state.steer);
        const point = suspension.contactPoint(geo, state.travel);
        const axis_speed = dot(body.pointVelocity(moved, point), geo.axis);
        const camber = tyre.camberGrip(p, now.axes, state.normal, state.contact);
        const handbrake = input.handbrake and p.handbrake;
        state.air = std.math.clamp(if (state.contact) state.air - dt else state.air + 2 * dt, 0, 1);
        const next = suspension.geometry(predicted, p, state.steer);
        const probe: ground.Probe = .{ .mount = next.mount, .axes = predicted.axes, .axis = next.axis, .air = state.air, .last = state.ground_reached, .ray_offset = state.ray_offset, .contact_offset = state.contact_offset };
        const ahead = ground.query(car_.groundWheel(car, i), car.bumps.?, g, i, probe, s.time, speed);
        if (ahead.material) |m| {
            if (m.hardness) |hardness| state.softness = 1 / hardness;
        }
        w.* = std.mem.zeroInit(WheelStep, .{
            .point = point,
            .axis = geo.axis,
            .lat = geo.lat,
            .lon = geo.lon,
            .travel = state.travel,
            .travel_rate = state.travel_rate,
            .axis_accel = (axis_speed - dot(body.pointVelocity(now, point), geo.axis)) * (1 / dt),
            .ground = state.ground,
            .spin = state.spin,
            .carcass = state.carcass,
            .anchor_rate = state.anchor_rate,
            .normal = ahead.normal,
            .softness = state.softness,
            .drag = (if (ahead.material) |m| m.side_drag else null) orelse material.Drag{},
            .slowdown = if (ahead.material) |m| m.slowdown else material.Slowdown{},
            .rebound_ratio = suspension.reboundRatio(car, p, state.contact, axis_speed, state.travel_rate),
            .mu = if (ahead.material) |m| mu(car, p, m.*, camber, ahead.normal, handbrake, speed) else 0,
            .brake = ((if (handbrake) car_.handbrakeTorque(car) else 0) + p.max_brake_torque * input.brake) * h / p.inertia,
            .abs = state.abs,
            .visual_bump = if (ahead.material) |m| (1 - m.bumps.by_time.contact_share) * bumps.height(car.bumps.?, m.bumps.by_time, i, s.time, speed) else 0,
        });
        state.ground = ahead.height;
        state.normal = ahead.normal;
    }
    couple(car, now, &st.wheels);
    for (&st.wheels, s.wheels) |*w, state| {
        const tracked = w.ground != ground.none and state.ground != ground.none;
        w.ground_rate = if (tracked) ((state.ground - w.ground) + w.axis_speed * dt) * (1 / dt) else 0;
    }
    return st;
}

// the coupling rows into the wheel blocks
pub fn couple(car: Car, s: body.State, ws: *[4]WheelStep) void {
    var frames: [4]coupling.Frame = undefined;
    for (&frames, ws, car.wheels) |*f, w, p| f.* = .{ .point = w.point, .axis = w.axis, .lat = w.lat, .lon = w.lon, .normal = w.normal, .rolling = p.radius * w.spin };
    for (ws, coupling.rows(car.rigid(), s, frames)) |*w, r| {
        w.axis_speed = r.axis_speed;
        w.lat_speed = r.lat_speed;
        w.lon_slip = r.lon_slip;
        w.response = r.response;
        w.bottoming = r.bottoming;
    }
}

fn mu(car: Car, p: CarWheel, m: material.Material, camber: f32, normal: V3, handbrake: bool, speed: f32) f32 {
    const held = if (handbrake) 1 - (car.handbrake_grip_loss orelse 0) else 1;
    return p.grip.? * (m.grip orelse 1) * held * ground.slopeGrip(m, normal, speed) * camber;
}

// 0x7e64a0
pub fn substep(car: Car, st: *Step, h: f32, frac: f32) void {
    drivetrain.clutch(car, st, h);
    drivetrain.diffs(car, st, h);
    drivetrain.brakes(car, st);
    const j = suspension.substep(car, &st.wheels, h, frac);
    tyre.substep(car, &st.wheels, j, st.handbrake, h, frac);
    st.engine_sum += st.engine_rate;
}

// after the substeps, then 0x7f56d0 and 0x7ebe30
pub fn finish(car: Car, s: *State, st: Step, share: f32, speed: f32, dt: f32) void {
    var arms: [4]V3 = undefined; // 0x7d5480, from the pose and travel before the step
    var surfaces: [4]material.Slowdown = undefined;
    for (&arms, &surfaces, s.wheels, st.wheels, car.wheels) |*a, *sd, state, w, p| {
        a.* = tyre.patchArm(p, state.travel, s.body.axes);
        sd.* = w.slowdown;
    }
    s.engine.rate = st.engine_rate;
    s.engine.mean = share * st.engine_sum;
    var contacts: [4]integrate.Contact = undefined;
    for (st.wheels, &s.wheels, &contacts, car.wheels) |w, *state, *c, p| {
        state.travel = w.travel;
        state.travel_rate = w.travel_rate;
        state.spin = w.spin;
        state.mean_spin = share * w.spin_sum;
        state.carcass = w.carcass;
        state.ground_reached = w.ground;
        state.abs = w.abs;
        state.contact = w.touched and state.ground != ground.none; // 0x7f56d0
        state.lift = tyre.visualLift(p, w.travel, w.ground, w.visual_bump);
        c.* = contact(w);
    }
    s.body = integrate.step(car.rigid(), s.body, .{
        .accel = st.accel,
        .ang_accel = st.ang_accel,
        .tyre_inertia = st.tyre_inertia,
        .contacts = &contacts,
    }, dt);
    slowdown.apply(car, s, surfaces, arms, speed, dt);
}

fn contact(w: WheelStep) integrate.Contact {
    return .{
        .point = w.point,
        .suspension = directed(w.axis, w.suspension),
        .lateral = directed(w.lat, w.tyre_lat),
        .longitudinal = directed(w.lon, w.tyre_lon),
        .across = .{ .full = across(w.tyre_normal.full, w.axis, w.normal), .weighted = across(w.tyre_normal.weighted, w.axis, w.normal) },
    };
}

fn directed(dir: V3, sum: Sum) integrate.Impulse {
    return .{ .full = splat(sum.full) * dir, .weighted = splat(sum.weighted) * dir };
}

// 0x7e7bd0: the part of a ground normal impulse j across the suspension axis
fn across(j: f32, axis: V3, n: V3) V3 {
    const along = dot(n, axis);
    return (n - axis * splat(along)) * splat(j / along);
}
