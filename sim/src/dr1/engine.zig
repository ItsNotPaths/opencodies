// the engine (E = car+0x6e00): torque curve, friction, starter, damage cutouts, turbo, fuel
const std = @import("std");
const body = @import("body.zig");
const math = @import("math.zig");

pub const Turbo = extern struct {
    inlet_pressure: f32, // T+0x0
    inlet_waste: f32,
    outlet_pressure: f32,
    outlet_waste: f32,
    rotation_rate: f32, // shaft
    pressure_to_engine: f32, // +0x14: scales the engine power
    pressure_ratio: f32,
    damage_scale: f32, // +0x1c
    enabled: bool, // +0x20 Properties
    capacity: f32, // +0x24 engine capacity
    inlet_volume: f32, // +0x34
    inlet_torque: f32,
    inlet_pressure_cap: f32,
    outlet_volume: f32,
    outlet_torque: f32,
    outlet_pressure_cap: f32,
    inertia: f32, // +0x4c
    inlet_rate: f32, // +0x5c
    sqr_factor: f32,
    outlet_input_rate: f32, // +0x64
    rpm_for_boost: f32, // +0x28 m_engineRPMForBoost
    spool_rpm_for_boost: f32, // +0x2c
    spool_rpm_max: f32, // +0x30
    max_boost: f32, // +0x50 m_maxBoostBar
    peak: f32, // +0x54 m_peakEngine
};

pub const Starter = extern struct {
    max_torque: f32, // E+0x19c
    on: bool, // +0x1a0
    strength: f32, // +0x1a4 current strength multiple, rolled while cranking slowly
    weight: f32, // +0x1a8: how much of the torque the starter sets, 0..1
    time_scale: f32, // +0x1ac
};

pub const Fuel = extern struct { tank: bool, capacity: f32, full_tank_weight: f32, current: f32 }; // *(E+0x210)

pub const Engine = extern struct {
    // Engine::Properties, kept in E+4[(n key) & 63]; n in brackets
    inertia: f32, // [0] fEngineMOI
    idle: f32, // [1] rad/s
    starter_strength: f32, // [2]
    friction: f32, // [3]
    friction_drive: f32, // [4] the part the clutch takes away
    torque: [32]f32, // [5..36] N m, every rev_step rad/s
    rev_limit: f32, // [37] rad/s
    restrictor: f32, // [38] restricted inlet air flow
    rev_step: f32, // setup+0x4c8 m_powerRevsGraphScale (car+0x6b88)

    throttle: f32, // E+0x178
    damaged_cutoff: f32, // +0x17c: time left of a damage cutout
    since_rev_cut: f32, // +0x184
    cutout_frequency: f32, // +0x188
    power_scale: f32,
    exhaust_damage: f32, // +0x190
    drivetrain_friction_scale: f32,
    starter: Starter,
    drift_friction: [2]f32, // +0x1b0 m_driftModifiedFrictionMultiple, 2
    wobble: [2]f32, // +0x1bc visual: value, rate
    delayed_rate: f32, // +0x1c4
    exhaust_wobble: [2]f32, // +0x1c8
    overrev: f32, // +0x1d0
    max_power: f32, // +0x1e8: in first gear rolling backwards
    rate: f32, // +0x1f4 m_rotationRateSmoothed: set before each update, cleared by it
    previous_rate: f32, // +0x1f8
    difficulty: f32, // +0x200 m_difficultyPowerMultiple
    random: u32, // *(E+0x208): the car's generator
    fuel: Fuel,
    upside_down: f32, // +0x218 seconds
    horse_power: f32, // +0x224
    load: f32, // +0x198 m_load: 0..1
    max_torque: f32, // +0x1e0
    applied_torque: f32, // +0x21c: the torque into the crank's inertia
    current_torque: f32, // +0x220
    turbo: Turbo, // +0x110
};

// what the engine reads of the car
pub const Motion = extern struct {
    up: [4]f32, // car+0x65e0
    forward: [4]f32, // car+0x65f0
    velocity: [4]f32, // car+0xf40, the body velocity at the start of the step
    gear: i32, // car+0x70b8 DriveTrain m_gear; 1 = first
};

// 0xf1ad00 Engine::update: returns the torque on the crank
pub fn update(e: *Engine, car: Motion, dt: f32, clutch: f32, external_pressure: f32) f32 {
    const v: body.V4 = car.velocity;
    if (0 > car.up[1] and body.dot4(v, car.up) > -0.5) e.upside_down = dt + e.upside_down else e.upside_down = 0;
    e.since_rev_cut = e.since_rev_cut + dt;
    updateDamage(e, dt);
    updateStarter(e, dt);

    const rate = e.rate;
    const s = e.starter.weight;
    var thr: f32 = if (!(0 < e.damaged_cutoff)) e.throttle else 0;
    if (0.8 * e.idle > rate) {
        thr = if (0 > s) 0 else below1(s);
    } else {
        thr = below1(if (thr > s) thr else s);
    }
    updateTurbo(&e.turbo, dt, rate, thr, external_pressure, e.restrictor);

    const stall = e.inertia * -rate / dt;
    var tq = power(e.*, car, thr, clutch) / math.maxss(20, @abs(rate));
    tq = tq + (e.starter.max_torque - tq) * e.starter.weight;
    const torque = math.maxss(tq, stall);
    const out_power = torque * rate;

    if (e.fuel.tank) {
        const a = rate * 0.5 / e.rev_limit + 0.5;
        const b: f32 = if (0.3 > e.since_rev_cut) 1.5 else 0.8 * e.throttle + 0.2;
        const fuel = e.fuel.current - b * a * dt;
        e.fuel.current = if (0 < fuel) fuel else 0;
    }
    const w = 3 * dt;
    e.delayed_rate = (if (w > 1) 1 else math.maxss(w, 0)) * (rate - e.delayed_rate) + e.delayed_rate;
    if (0.99 * e.idle > rate) {
        e.wobble = .{ 0, 0 };
        e.exhaust_wobble = .{ 0, 0 };
    } else {
        wobble(&e.random, &e.wobble, dt, 15);
        if (0.99 * e.idle > e.rate) e.exhaust_wobble = .{ 0, 0 } else wobble(&e.random, &e.exhaust_wobble, dt, 25);
    }
    e.horse_power = out_power / 745.7;
    e.rate = 0;
    e.previous_rate = rate;
    return torque;
}

// 0xf1b500 Engine::computePower: the curve power and the friction at the smoothed rate, before the divide by it
pub fn power(e: Engine, car: Motion, throttle: f32, clutch: f32) f32 {
    const rate = e.rate;
    var p = if (car.gear == 1 and 0 > body.dot4(car.velocity, car.forward)) e.max_power else powerAt(e, rate);
    p = p * e.power_scale;
    p = p * e.difficulty;
    if (e.fuel.tank) p = p / ((e.fuel.full_tank_weight * e.fuel.current) / e.fuel.capacity + 1);
    p = p * e.turbo.pressure_to_engine;
    const ic = e.inertia / 0.2;
    const c1 = 1 - clutch;
    var drive = (e.friction_drive + e.friction) * e.drivetrain_friction_scale * 3000 * e.drift_friction[0];
    drive = e.drift_friction[1] * drive;
    var g = @abs(rate) / 30;
    if (rate > 0) {
        const k = (@abs(rate - e.idle) - 1) / 30;
        g = -math.minss(@abs(rate) / 30, if (!(k < 0)) k else 0);
    }
    return frictionAt(e, rate, clutch) * (1 - throttle) + p + g * (drive * ic) * c1;
}

// 0xf178a0 Engine::computeFrictionAtRate: the engine braking at a rate, the clutch taking the drive part away
pub fn frictionAt(e: Engine, rate: f32, clutch: f32) f32 {
    if (!(rate > 0)) return 0;
    const engaged = ((1 - clutch) * e.friction_drive + e.friction) * 3000 * e.drift_friction[0] * e.drift_friction[1];
    const m1 = math.maxss(0, (@abs(rate - e.idle) - 1) / 30);
    const r30 = rate / 30;
    const f = if (rate > e.idle) -math.minss(r30, m1) else if (0.8 * e.idle > rate) (e.idle - e.rev_limit) / 30 else math.minss(r30, m1);
    return engaged * (e.inertia / 0.2) * f;
}

fn below1(x: f32) f32 {
    return if (x < 1) x else 1;
}

// 0xf17160 Engine::getPowerAtRotationRate: the torque curve (stretched by exhaust damage) times the rate
pub fn powerAt(e: Engine, rate: f32) f32 {
    const pos = (0.25 * e.exhaust_damage + 1) * rate / e.rev_step;
    if (pos > 31 or 0 > pos) return 0;
    const c = math.truncate(pos);
    const i: usize = @intCast(std.math.clamp(c, 0, 30));
    const a = e.torque[i];
    return rate * ((pos - @as(f32, @floatFromInt(c))) * (e.torque[i + 1] - a) + a);
}

// 0xf1aa80 Engine::updateDamage: random cutouts at the damage frequency, the overrev indicator
fn updateDamage(e: *Engine, dt: f32) void {
    if (e.damaged_cutoff > 0) e.damaged_cutoff = e.damaged_cutoff - dt;
    if (e.cutout_frequency > 0) {
        const chance = e.cutout_frequency * (dt * 2.2) * 1024;
        if (chance > @as(f32, @floatFromInt(random(&e.random) & 0x3ff))) {
            const t = @as(f32, @floatFromInt(random(&e.random) & 0xff)) * 0.06 / 255 + 0.02;
            if (t > e.damaged_cutoff) e.damaged_cutoff = t;
        }
    }
    const x = math.minss(e.rate, e.rev_limit * 1.1) / (1.05 * e.rev_limit);
    e.overrev = if (1 <= x) x else 0;
}

// 0xf1abd0 Engine::updateStarterMotor: the weight ramps toward 1 while cranking below 1.25 idle
fn updateStarter(e: *Engine, dt: f32) void {
    const st = &e.starter;
    st.max_torque = e.inertia * e.idle / st.time_scale * st.strength * e.starter_strength;
    var target: f32 = 0;
    if (st.on and e.idle * 1.25 > e.rate) {
        target = 1;
        if (e.idle * 1.25 * 0.25 > e.rate) st.strength = @as(f32, @floatFromInt(random(&e.random) & 0xff)) * 1.2 / 255 + 0.4;
    }
    const w = st.weight;
    if (target > w) {
        st.weight = math.minss(target, w + dt * 3);
    } else if (w > target) {
        st.weight = math.maxss(target, w - dt * 1.5);
    }
}

// PhysicsFastRandomNumbersGenerator::getInteger
pub fn random(state: *u32) u32 {
    const s = state.* *% 0x41c64e6d +% 0x40584c0f;
    state.* = s;
    return s +% @as(u32, @truncate((@as(u64, s) * 0x80008001) >> 47));
}

const wobble_limit: f32 = 0x1.eb852p-9; // 0x27361f0: 0x3b75c290, a hair over f32(0.00375)

// 0xedcea0 Engine::updateVisualWobble: a damped random walk within +-0.00375. Visual only, but it draws
// from the car's generator
fn wobble(state: *u32, v: *[2]f32, dt: f32, rate_scale: f32) void {
    var r = (1 - 4 * dt) * v[1];
    const d1: i32 = @intCast(random(state) & 0x1ff);
    r = r - v[0] * dt * 60 * 0.15;
    if (!(d1 > math.truncate(rate_scale * dt * 512))) r = r + (@as(f32, @floatFromInt(random(state) & 0xff)) - 127.5) * 0.15 / 127.5;
    v[1] = r;
    const x = r * dt + v[0];
    if (x > wobble_limit) {
        v.* = .{ wobble_limit, -(@abs(r) * 0.8) };
    } else if (-wobble_limit > x) {
        v.* = .{ -wobble_limit, @abs(r) * 0.8 };
    } else {
        v[0] = x;
    }
}

// 0xedd3e0 Engine::Turbo::estimateAirPressureAtEngineRate: the boost the turbo settles at for an engine rate
pub fn estimatePressure(t: Turbo, rate: f32, external: f32, restricted: f32) f32 {
    const pi = std.math.pi;
    const boost_rate = t.rpm_for_boost * pi / 30;
    const spool_max = t.spool_rpm_max * pi / 30;
    const spool = pi * t.spool_rpm_for_boost / 30;
    const s = math.minss(spool_max, t.peak * t.spool_rpm_for_boost / t.rpm_for_boost);
    const at_boost = math.minss(t.spool_rpm_max * boost_rate / t.spool_rpm_for_boost, t.peak);
    const keep = t.outlet_volume / (at_boost * 0.001 * t.capacity + t.outlet_volume);
    const flow = s * 0.001 * boost_rate * t.capacity / (t.outlet_volume * spool);
    const ratio = (1 - keep) * (t.max_boost + 1) / (flow * keep);
    const k = (1 - ratio) / (ratio * spool * spool - s * s);
    const w = math.maxss(t.spool_rpm_for_boost * rate / t.rpm_for_boost, 0);
    const exhaust = w * (boost_rate * t.capacity / ((1 + spool * k * spool) * spool));
    const sqr = (k * w * w + external) / external;
    const drawn = (if (exhaust > restricted) restricted else math.maxss(exhaust, -restricted)) * 0.001;
    const p = external * drawn * sqr / (t.capacity * (rate * 0.001));
    return math.minss(t.inlet_pressure_cap, if (p > t.outlet_pressure_cap) t.outlet_pressure_cap else math.maxss(p, 0));
}

// 0xf19c00 Engine::Turbo::updateTurbo: inlet and outlet pressure and the shaft, in 1 ms steps
fn updateTurbo(t: *Turbo, dt: f32, rate: f32, throttle: f32, external: f32, restricted: f32) void {
    if (!t.enabled) {
        const flow = rate * t.capacity * external;
        const r = restricted / (if (1e-6 < flow) flow else 1e-6);
        t.pressure_to_engine = (if (r > 1) 1 else math.maxss(r, 0)) * (external * throttle);
        return;
    }
    const filtered = throttle * 0.995 + 0.005;
    const steps = dt / 0.001;
    if (!(steps > 0)) {
        t.pressure_to_engine = t.outlet_pressure * throttle;
        return;
    }
    const flow = t.capacity * (rate * 0.001);
    const inflow = t.pressure_to_engine * flow / t.inlet_volume;
    const outlet_keep = t.outlet_volume / (flow * filtered + t.outlet_volume);
    const floor = 0.005 * external;
    const cap = (0.8 * external - t.inlet_pressure_cap) * t.damage_scale + t.inlet_pressure_cap;
    const lo = cap - 0.1;
    const span = (cap + 0) - lo;
    var p = t.inlet_pressure;
    var q = t.outlet_pressure;
    var w = t.rotation_rate;
    var ratio: f32 = undefined;
    var waste: f32 = undefined;
    var outlet_waste: f32 = undefined;
    var i: i32 = 0;
    while (true) {
        p = (p + inflow) * (t.inlet_volume / (w * 0.001 * t.inlet_rate + t.inlet_volume));
        p = math.maxss(floor, p);
        ratio = (t.sqr_factor * w * w + external) / external;
        const exhaust = math.minss(t.outlet_input_rate * w, restricted);
        q = (q + exhaust * 0.001 * external * ratio / t.outlet_volume) * outlet_keep;
        q = math.maxss(floor, q);
        var open = (q - lo) / span;
        open = if (open > 1) 1 else if (0 < open) open else 0;
        var zero_torque = (q - external) * t.outlet_torque / t.inlet_torque + external;
        zero_torque = math.minss(zero_torque * 0.98, p);
        zero_torque = if (0 < zero_torque) zero_torque else 0;
        const next = open * (zero_torque - p) + p;
        waste = p - next;
        const torque = ((next - external) * t.inlet_torque + 0) - (q - external) * t.outlet_torque;
        if (q > t.outlet_pressure_cap) {
            outlet_waste = q - t.outlet_pressure_cap;
            q = t.outlet_pressure_cap;
        } else outlet_waste = 0;
        w = w + torque * 0.001 / t.inertia;
        w = if (0 < w) w else 0;
        p = next;
        i += 1;
        if (!(steps > @as(f32, @floatFromInt(i)))) break;
    }
    t.pressure_ratio = ratio;
    t.outlet_pressure = q;
    t.inlet_waste = waste;
    t.inlet_pressure = p;
    t.outlet_waste = outlet_waste;
    t.rotation_rate = w;
    t.pressure_to_engine = q * throttle;
}
