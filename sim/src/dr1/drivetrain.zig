// the drivetrain (DriveTrainIntegrator, *(T+0x840)): a fixed tree of components that pull the state of what
// is below them and push torque down. engine -> clutch -> gearbox -> centre diff -> front diff -> brakes FL FR
// -> wheels, and centre diff -> rear clutch -> rear diff -> brakes RL RR -> wheels
const std = @import("std");
const body = @import("body.zig");
const math = @import("math.zig");

// drivetrain::State: what a branch looks like from above
pub const State = extern struct { torque: f32 = 0, angvel: f32 = 0, inertia: f32 = 1, ignore: bool = false }; // ignore: takes no torque

pub const Clutch = extern struct {
    friction: f32, // m_friction: torque at full load
    load: f32, // engagement 0..1
    damage_scale: f32,
    power_dissipated: f32,
    applied: f32, // m_appliedClutchTorque
};

pub const Gear = extern struct { ratio: f32, angvel_at_input: f32 };

pub const Brake = extern struct { max_torque: f32, damage_torque: f32, applied: f32 };

pub const Wheel = extern struct {
    road_torque: f32,
    driving_torque: f32,
    state: State,
};

// one half of a compound diff: a LimitedSlipClutch, or a ViscousBrake in a viscous diff
pub const Part = extern struct {
    max_torque: f32, // LimitedSlipClutch; ViscousBrake: viscosity
    target_velocity: f32,
    applied: f32,
    factor: f32, // ViscousBrake only
    plus: f32, // ViscousBrake +0x24: in.torque + applied
};

pub const Kind = enum(u32) { open, lsd, viscous, crown, one_way_front, one_way_rear, disconnected };

pub const Diff = extern struct {
    kind: Kind,
    bias: f32, // the (inner) OpenDifferential m_torqueBias: the share of torque to output 0
    driving_bias: f32, // LSD torque bias ratios
    braking_bias: f32,
    preload: f32,
    override: f32, // LSD/crown clutchOverride: 0 while the handbrake opens it
    viscosity: f32, // viscous
    clutch_torque: f32, // crown
    clutch_bias: f32,
    parts: [2]Part,
};

pub const Drivetrain = extern struct {
    engine: State, // m_state
    engine_inertia: f32,
    rev_limit: f32,
    has_wobbled: bool,
    random: Mt19937,
    clutch: Clutch,
    gearbox: Gear,
    rear_clutch: Gear,
    centre: Diff,
    front: Diff,
    rear: Diff,
    brakes: [4]Brake, // FL FR RL RR
    wheels: [4]Wheel, // FL FR RL RR
};

pub const Mt19937 = extern struct { mt: [624]u32, index: u32 };

// where a component's output goes
const Out = union(enum) { gearbox, centre, rear_clutch, axle: *Diff, part: struct { diff: *Diff, i: u1 }, brake: u2, wheel: u2 };

const Tree = struct {
    d: *Drivetrain,
    h: f32,

    fn outputs(t: Tree, diff: *const Diff) [2]Out {
        if (diff == &t.d.centre) return .{ .{ .axle = &t.d.front }, .rear_clutch };
        return if (diff == &t.d.front) .{ .{ .brake = 0 }, .{ .brake = 1 } } else .{ .{ .brake = 2 }, .{ .brake = 3 } };
    }

    fn pull(t: Tree, out: Out) State {
        return switch (out) {
            .wheel => |i| .{ .torque = t.d.wheels[i].road_torque, .angvel = t.d.wheels[i].state.angvel, .inertia = t.d.wheels[i].state.inertia, .ignore = false },
            .brake => |i| brakePull(t, i),
            .gearbox => gearPull(t, t.d.gearbox, .centre),
            .rear_clutch => gearPull(t, t.d.rear_clutch, .{ .axle = &t.d.rear }),
            .centre => diffPull(t, &t.d.centre),
            .axle => |x| diffPull(t, x),
            .part => |p| t.pull(t.outputs(p.diff)[p.i]),
        };
    }

    fn push(t: Tree, out: Out, in: State) State {
        return switch (out) {
            .wheel => |i| blk: {
                const w = &t.d.wheels[i];
                w.driving_torque = in.torque;
                break :blk .{ .torque = in.torque + w.road_torque, .angvel = in.angvel, .inertia = in.inertia, .ignore = in.ignore };
            },
            .brake => |i| brakePush(t, i, in),
            .gearbox => gearPush(t, &t.d.gearbox, .centre, in),
            .rear_clutch => gearPush(t, &t.d.rear_clutch, .{ .axle = &t.d.rear }, in),
            .centre => diffPush(t, &t.d.centre, in),
            .axle => |x| diffPush(t, x, in),
            .part => |p| partPush(t, p.diff, p.i, in),
        };
    }
};

// drivetrain::Brake: stops the wheel within max + damage torque; the pull counts the road torque twice
fn brakePull(t: Tree, i: u2) State {
    const b = t.d.brakes[i];
    const s = t.pull(.{ .wheel = i });
    const stop = ((-s.inertia * s.angvel) / t.h - s.torque) - s.torque;
    const applied = clampSym(stop, b.max_torque + b.damage_torque);
    return .{ .torque = s.torque + applied, .angvel = s.angvel, .inertia = s.inertia, .ignore = s.ignore };
}

fn brakePush(t: Tree, i: u2, in: State) State {
    const b = &t.d.brakes[i];
    const s = t.pull(.{ .wheel = i });
    const applied = clampSym((-s.inertia * s.angvel) / t.h - s.torque, b.max_torque + b.damage_torque);
    b.applied = applied;
    return t.push(.{ .wheel = i }, .{ .torque = applied + in.torque, .angvel = in.angvel, .inertia = in.inertia, .ignore = in.ignore });
}

// t limited to +-cap the way the clutches clamp: NaN cap or t gives the cap
fn clampSym(t: f32, cap: f32) f32 {
    if (t > 0) return math.minss(t, cap);
    return math.maxss(t, cap * (if (t < 0) @as(f32, -1) else 0));
}

fn gearPull(t: Tree, g: Gear, out: Out) State {
    const s = t.pull(out);
    if (g.ratio == 0) return .{ .ignore = true };
    return .{ .torque = s.torque / g.ratio, .angvel = s.angvel * g.ratio, .inertia = s.inertia / (g.ratio * g.ratio), .ignore = s.ignore };
}

fn gearPush(t: Tree, g: *Gear, out: Out, in: State) State {
    g.angvel_at_input = in.angvel;
    const r = g.ratio;
    if (r == 0) {
        _ = t.push(out, .{});
        return .{ .ignore = true };
    }
    const q = t.push(out, .{ .torque = in.torque * r, .angvel = in.angvel / r, .inertia = r * r * in.inertia, .ignore = in.ignore });
    const back: State = .{ .torque = q.torque / g.ratio, .angvel = q.angvel * g.ratio, .inertia = q.inertia / (g.ratio * g.ratio), .ignore = q.ignore };
    g.angvel_at_input = t.pull(out).angvel * g.ratio;
    return back;
}

// OpenDifferential::combine
fn combine(a: State, b: State) State {
    if (a.ignore) return if (b.ignore) .{ .ignore = true } else b;
    if (b.ignore) return a;
    return .{ .torque = b.torque + a.torque, .angvel = (a.angvel + b.angvel) * 0.5, .inertia = b.inertia + a.inertia, .ignore = false };
}

fn diffPull(t: Tree, x: *Diff) State {
    const out = t.outputs(x);
    return switch (x.kind) {
        .disconnected => .{},
        .one_way_front => t.pull(out[0]),
        .one_way_rear => t.pull(out[1]),
        else => combine(t.pull(out[0]), t.pull(out[1])),
    };
}

// the inner OpenDifferential: splits by the torque bias, into the parts of a compound diff or the outputs
fn openPush(t: Tree, x: *Diff, kids: [2]Out, in: State) State {
    const k = x.bias;
    const a = t.push(kids[0], .{ .torque = k * in.torque, .angvel = in.angvel, .inertia = in.inertia * k, .ignore = in.ignore });
    const j = 1 - k;
    const b = t.push(kids[1], .{ .torque = j * in.torque, .angvel = in.angvel, .inertia = in.inertia * j, .ignore = in.ignore });
    return combine(a, b);
}

fn parts(x: *Diff) [2]Out {
    return .{ .{ .part = .{ .diff = x, .i = 0 } }, .{ .part = .{ .diff = x, .i = 1 } } };
}

// the speed the joined branch reaches this substep, 0 if it would pass zero
fn reach(s: State, in_torque: f32, h: f32) f32 {
    const v = (s.torque + in_torque) * h / s.inertia + s.angvel;
    return if (!(s.angvel * v < 0)) v else 0;
}

fn diffPush(t: Tree, x: *Diff, in: State) State {
    const out = t.outputs(x);
    switch (x.kind) {
        .open => return openPush(t, x, out, in),
        .disconnected => {
            var a = t.pull(out[0]);
            var b = t.pull(out[1]);
            a.torque = 0;
            b.torque = 0;
            _ = t.push(out[0], a);
            _ = t.push(out[1], b);
            return .{};
        },
        .one_way_front, .one_way_rear => {
            var a = t.pull(out[0]);
            var b = t.pull(out[1]);
            const front = x.kind == .one_way_front;
            a.torque = if (front) in.torque else 0;
            b.torque = if (front) 0 else in.torque;
            if (front) {
                _ = t.push(out[1], b);
                return t.push(out[0], a);
            }
            _ = t.push(out[0], a);
            return t.push(out[1], b);
        },
        .lsd => {
            const s1 = t.pull(out[0]);
            const s2 = t.pull(out[1]);
            if (s1.ignore or s2.ignore) {
                x.parts[0].max_torque = 0;
                x.parts[1].max_torque = 0;
            } else {
                const d = in.torque - (s1.torque + s2.torque);
                const k = biasToLocking(if (d > 0) x.driving_bias else x.braking_bias);
                const cap = x.override * (@abs(x.preload) + @abs(d * k));
                const s = combine(s1, s2);
                x.parts[0].max_torque = cap * 0.5;
                x.parts[1].max_torque = cap * 0.5;
                const v = reach(s, in.torque, t.h);
                x.parts[0].target_velocity = v;
                x.parts[1].target_velocity = v;
            }
            return openPush(t, x, parts(x), in);
        },
        .viscous => {
            const s1 = t.pull(out[0]);
            const s2 = t.pull(out[1]);
            const v = reach(combine(s1, s2), in.torque, t.h);
            for (&x.parts) |*p| {
                p.target_velocity = v;
                p.factor = if (s1.ignore or s2.ignore) 0 else 1;
            }
            return openPush(t, x, parts(x), in);
        },
        .crown => {
            const s1 = t.pull(out[0]);
            const s2 = t.pull(out[1]);
            const b: f32, const o: f32 = if (x.clutch_bias > 1) .{ 1, 0 } else if (x.clutch_bias > 0) .{ x.clutch_bias, 1 - x.clutch_bias } else .{ 0, 1 };
            x.clutch_bias = b;
            x.clutch_torque = if (!(x.clutch_torque < 0)) x.clutch_torque else 0;
            const c = x.clutch_torque * x.override;
            x.parts[0].max_torque = b * c;
            x.parts[1].max_torque = c * o;
            if (s1.ignore or s2.ignore) {
                x.parts[0].max_torque = 0;
                x.parts[1].max_torque = 0;
            } else {
                const v = reach(combine(s1, s2), in.torque, t.h);
                x.parts[0].target_velocity = v;
                x.parts[1].target_velocity = v;
            }
            return openPush(t, x, parts(x), in);
        },
    }
}

// LimitedSlipDifferential::biasToLocking: a torque bias ratio as a locking factor
fn biasToLocking(bias: f32) f32 {
    const a = @abs(bias);
    return if (1 > a) (1 - a) / (a + 1) else (a - 1) / (a + 1);
}

// LimitedSlipClutch or ViscousBrake between the inner diff and an output
fn partPush(t: Tree, x: *Diff, i: u1, in: State) State {
    const p = &x.parts[i];
    const out = t.outputs(x)[i];
    const s = t.pull(out);
    const v = reach(s, in.torque, t.h);
    var applied: f32 = undefined;
    if (x.kind == .viscous) {
        const dv = v - p.target_velocity;
        const stop = (-s.inertia * dv) / t.h;
        const visc = -dv * p.max_torque * p.factor;
        applied = if (visc > 0 and visc > stop) stop else if (stop > visc and 0 > visc) stop else visc;
        p.applied = applied;
        p.plus = in.torque + applied;
        return t.push(out, .{ .torque = in.torque + applied, .angvel = in.angvel, .inertia = in.inertia, .ignore = in.ignore });
    }
    applied = clampSym(s.inertia * (p.target_velocity - v) / t.h, p.max_torque);
    p.applied = applied;
    return t.push(out, .{ .torque = applied + in.torque, .angvel = in.angvel, .inertia = in.inertia, .ignore = in.ignore });
}

// drivetrain::Clutch::pushState: the exact torque that equalises both sides this substep, within the load
fn clutchPush(t: Tree, in: State) State {
    const c = &t.d.clutch;
    const s = t.pull(.gearbox);
    const a = in.torque * s.inertia;
    const both = in.inertia * s.inertia;
    const b = s.torque * in.inertia;
    var torque = ((in.angvel - s.angvel) * both / t.h + (a - b)) / (s.inertia + in.inertia);
    torque = clampSym(torque, c.load * c.friction * c.damage_scale);
    const r = t.push(.gearbox, .{ .torque = torque, .angvel = in.angvel, .inertia = in.inertia, .ignore = in.ignore });
    if (r.ignore) {
        c.power_dissipated = 0;
        c.applied = 0;
        return in;
    }
    c.power_dissipated = @abs((in.angvel - s.angvel) * c.load);
    c.applied = torque;
    return .{ .torque = in.torque - torque, .angvel = in.angvel, .inertia = in.inertia, .ignore = in.ignore };
}

// NeRandom::next, MT19937 with the old Knuth seeding
fn next(r: *Mt19937) u32 {
    if (r.index > 623) {
        if (r.index == 625) {
            r.mt[0] = 4357;
            for (1..624) |i| r.mt[i] = r.mt[i - 1] *% 69069;
        }
        for (0..624) |i| {
            const y = (r.mt[i] & 0x80000000) | (r.mt[(i + 1) % 624] & 0x7fffffff);
            r.mt[i] = r.mt[(i + 397) % 624] ^ (y >> 1) ^ (if (y & 1 != 0) @as(u32, 0x9908b0df) else 0);
        }
        r.index = 0;
    }
    var y = r.mt[r.index];
    r.index += 1;
    y ^= y >> 11;
    y ^= (y << 7) & 0x9d2c5680;
    y ^= (y << 15) & 0xefc60000;
    y ^= y >> 18;
    return y;
}

// what integrate reads from and writes to the car
pub const CarSide = extern struct {
    traction_torque: [4]f32, // wheel data +0x170, car wheel order RL RR FL FR
    rolling_torque: [4]f32, // +0x174
    rotation_rate: [4]f32, // +0x198
    forward_velocity: [4]f32, // +0x188
    abs_active: [4]bool, // +0x191
    pos: [4][4]f32, // +0x0
    forward_axis: [4][4]f32, // +0xc0
    radius: [4]f32, // Wheel+0x140
    max_brake_torque: [4]f32, // Wheel+0x178
    applied_brake_torque: [4]f32, // Wheel+0x3ac, out
    braking_torque: [4]f32, // T+0x820
    rotational_resistance: [4]f32, // damage, car+0x11f18 + 0x20 i
    gear_ratio: f32, // T+0x818
    handbrake: f32, // car+0x6670
    clutch_pedal: f32, // car+0x667c
    clutch_on_handbrake: bool, // T+0x848 m_clutchOnHandbrakeOverride
    handbrake_uses_rear_clutch: bool, // setup+0x470
    auto_clutch: bool, // DriveTrain+0x95
    clutch_effect: f32, // damage, car+0x11f04
    centre_index: u32, // the setup's centre diff type
    engine_rate: f32, // car+0x6ff0, out
    engine_rate_sum: f32, // car+0x6ff4, out
    transmission_torque: f32, // car+0x70dc, out
    body: body.Body, // the physics body, for the hub speeds
    since_rev_cut: f32, // engine+0x184: subUpdate clears it after a wobble
};

const car_wheel = [4]usize{ 2, 3, 0, 1 }; // FL FR RL RR -> the car's wheel index

// the centre's input speed as its pullState gives it (Car::getTransmissionSpeed without the wheel radius)
pub fn centreAngvel(d: *Drivetrain) f32 {
    const t: Tree = .{ .d = d, .h = 0.001 };
    return t.pull(.centre).angvel;
}

// 0xee20e0 DriveTrainIntegrator::integrate for one substep. copy: write the result back to the car (false:
// a dry run for the stability assist; the drivetrain state still moves)
pub fn integrate(d: *Drivetrain, car: *CarSide, engine_torque: f32, h: f32, copy: bool) void {
    const t: Tree = .{ .d = d, .h = h };
    for (&d.wheels, car_wheel) |*w, i| {
        w.driving_torque = 0;
        w.road_torque = car.traction_torque[i] + car.rolling_torque[i];
        w.state.torque = 0;
        w.state.angvel = car.rotation_rate[i];
    }
    for (&d.brakes, car_wheel) |*b, i| {
        b.damage_torque = @abs(car.rotational_resistance[i] * car.max_brake_torque[i]);
        const applied = if (car.abs_active[i]) 0 else car.braking_torque[i];
        b.max_torque = math.maxss(b.damage_torque, @abs(applied));
        car.applied_brake_torque[i] = applied;
    }
    d.gearbox.ratio = if (car.gear_ratio == 0) 0 else 1 / car.gear_ratio;
    handbrake(d, car.*);

    const w0 = math.minss(car.engine_rate, d.rev_limit);
    d.engine.angvel = w0;
    const start = d.rev_limit * 0;
    const f = (w0 - start) / (d.rev_limit - start);
    const strength = @abs(engine_torque) * (if (f > 1) 1 else math.maxss(f, 0));
    const wobble = @as(f32, @floatFromInt(next(&d.random))) * 0x1p-32 * 0 * strength; // getFloat(0, 0)
    if (wobble > 0) d.has_wobbled = true;
    d.engine.torque = engine_torque - wobble;
    const ret = clutchPush(t, d.engine);
    d.engine.ignore = ret.ignore;
    d.engine.torque = ret.torque;
    d.engine.inertia = ret.inertia;
    d.engine.angvel = step(ret.torque, ret.angvel, ret.inertia, h);
    for (&d.wheels) |*w| {
        w.state.torque = w.driving_torque + w.road_torque;
        w.state.angvel = step(w.state.torque, w.state.angvel, w.state.inertia, h);
    }
    if (!copy) return;

    const w = math.minss(d.engine.angvel, d.rev_limit);
    car.engine_rate = if (0 > w) 0 else w;
    car.engine_rate_sum = (if (0 > w) 0 else math.maxss(0, w)) + car.engine_rate_sum;
    car.transmission_torque = d.gearbox.ratio * d.clutch.applied;
    for (d.wheels, car_wheel) |wh, i| car.rotation_rate[i] = wh.state.angvel;
    const old = car.forward_velocity;
    for (car_wheel) |i| {
        const forward = body.dot4(body.pointVelocity(car.body, car.pos[i]), car.forward_axis[i]);
        const n = forward - car.radius[i] * car.rotation_rate[i];
        car.forward_velocity[i] = if (0 <= old[i] * n) n else 0;
    }
}

// the speed after a substep under torque, 0 instead of passing through zero
fn step(torque: f32, w: f32, inertia: f32, h: f32) f32 {
    const v = torque * h / inertia + w;
    return if (w * v < 0) 0 else v;
}

// the handbrake opens the centre diff or the rear clutch, depending on the setup and the centre diff type
fn handbrake(d: *Drivetrain, car: CarSide) void {
    const pedal = 1 - car.clutch_pedal;
    var load = pedal;
    var opens_centre = false; // esi
    var opens_rear = false; // r13
    var plain = false; // paths P and OVR: the clutch takes the pedal, damage only above 1
    if (!(car.handbrake > 0)) {
        plain = true;
    } else if (car.clutch_on_handbrake) {
        load = if (car.auto_clutch) 0 else pedal;
        plain = true;
    } else switch (car.centre_index) {
        2, 3, 6 => {
            opens_centre = !car.handbrake_uses_rear_clutch;
            opens_rear = car.handbrake_uses_rear_clutch;
        },
        1 => if (car.handbrake_uses_rear_clutch) {
            opens_rear = true;
        } else {
            load = if (car.auto_clutch) 0 else pedal;
            plain = true;
        },
        5 => {
            load = if (car.auto_clutch) 0 else pedal;
            plain = true;
        },
        else => {},
    }
    const e = car.clutch_effect;
    d.clutch.load = load;
    if (e > 1 and (plain or opens_rear and car.centre_index == 1)) {
        d.clutch.damage_scale = 1;
        onHandbrake(&d.centre, false);
        d.rear_clutch.ratio = if (plain) 1 else 0;
        return;
    }
    d.clutch.damage_scale = if (e > 1) 1 else math.maxss(e, 0);
    onHandbrake(&d.centre, opens_centre);
    d.rear_clutch.ratio = if (opens_rear) 0 else 1;
}

fn onHandbrake(x: *Diff, on: bool) void {
    switch (x.kind) {
        .lsd, .crown => x.override = if (on) 0 else 1,
        .viscous => for (&x.parts) |*p| {
            p.max_torque = if (on) 0 else @abs(x.viscosity);
        },
        else => {},
    }
}
