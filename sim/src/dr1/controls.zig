// the driver's side of Car::updateAll: pedals, launch control, auto handbrake, steering, the auto clutch, rev
// matching and the gearbox
const std = @import("std");
const body = @import("body.zig");
const math = @import("math.zig");
const aero = @import("aero.zig");
const engine = @import("engine.zig");
const drivetrain = @import("drivetrain.zig");
const car = @import("car.zig");
const State = car.State;

// CAR_CONTROLS: m_controls[14] at car+0x6648, a union of bool, float and int
pub const Control = enum(u4) {
    ignition, // bool [likely]
    throttle,
    brake,
    gear_up, // bool
    gear_down, // bool
    gear, // int: a gear asked for, 9 neutral
    steering,
    adjusted_steering,
    manual_clutch,
    auto_clutch,
    handbrake,
    horn, // bool
    boost,
    final_clutch,
};

pub fn get(s: *const State, c: Control) f32 {
    return @bitCast(s.controls[@intFromEnum(c)]);
}

pub fn set(s: *State, c: Control, x: f32) void {
    s.controls[@intFromEnum(c)] = @bitCast(x);
}

// a bool reads and writes the low byte only, also of a float control
pub fn flag(s: *const State, c: Control) bool {
    return s.controls[@intFromEnum(c)] & 0xff != 0;
}

pub fn setFlag(s: *State, c: Control, b: bool) void {
    const i = @intFromEnum(c);
    s.controls[i] = (s.controls[i] & ~@as(u32, 0xff)) | @intFromBool(b);
}

fn requested(s: *const State) i32 {
    return @bitCast(s.controls[@intFromEnum(Control.gear)]);
}

fn unit(x: f32) f32 {
    return if (x > 1) 1 else math.maxss(x, 0);
}

fn limitedRate(s: *const State) f32 {
    return math.minss(s.engine.rate, s.engine.rev_limit * 1.1);
}

// 0xf193a0 Car::updateControls
pub fn update(s: *State, dt: f32) void {
    const t = &s.transmission;
    const old_gear = t.gear;
    const swap = old_gear == 10 and t.auto_reverse;
    var thr = if (swap) get(s, .brake) else get(s, .throttle);
    const brk = if (swap) get(s, .throttle) else get(s, .brake);
    s.brake = brk;
    s.engine.throttle = thr;
    s.engine.drift_friction[0] = 1;
    s.horn = @truncate(s.controls[@intFromEnum(Control.horn)]);
    if (s.launch.enabled and s.launch.active) {
        if (!s.launch.available and flag(s, .handbrake) or brk > 0 or 0.1 > thr) {
            s.launch.active = false;
        } else if (!flag(s, .handbrake)) {
            thr = 1;
            s.engine.throttle = 1;
        }
    }
    s.auto_handbrake = false;
    if (!s.auto_handbrake_enabled) {
        set(s, .handbrake, 0);
    } else if (0.01 > get(s, .handbrake) and 0.5 > s.speed) {
        const idle = thr == 0 and brk == 0 or !t.auto_clutch and get(s, .final_clutch) == 1 or old_gear == 0;
        if (idle and s.sys.wheels_on_ground == s.wheel_count) {
            set(s, .handbrake, 1);
            s.auto_handbrake = true;
        }
    }
    steeringSystem(s, dt);
    autoClutch(s, dt);
    revMatching(s);

    var start = flag(s, .ignition);
    if (!start and s.engine.idle > s.engine.rate and s.fuel_level > 0 and s.engine.throttle > 0.5) {
        start = true;
        if (!(get(s, .final_clutch) > 0.95)) start = gearRatio(s.*) == 0;
    }
    s.engine.starter.on = start;
    gearbox(s, dt);
    if (s.damage.gearbox_state != 8) {
        setFlag(s, .ignition, false);
        s.engine.starter.on = false;
        if (t.gear != 1) t.swing = 0;
        t.gear = 1;
        s.engine.throttle = 0;
    }
    launchControl(s, dt);
}

// the ratio the wheels see now: blended through a seamless shift
fn gearRatio(s: State) f32 {
    const t = s.transmission;
    const time = if (t.auto_gearbox) t.shift_time[0] else t.shift_time[1];
    const to = t.ratios[@intCast(t.gear)];
    if (!t.seamless or !(time > 0)) return to;
    const from = t.ratios[@intCast(t.previous)];
    return (to - from) * unit(t.since_change / time) + from;
}

// armed after half a second on the handbrake in first, active while the revs stay in the window around the
// optimum
fn launchControl(s: *State, dt: f32) void {
    const l = &s.launch;
    const optimum = s.optimum_launch_rpm;
    if (!l.enabled or !(optimum > 0)) {
        l.available = false;
        l.active = false;
        return;
    }
    const first = s.transmission.gear == 1;
    l.standing = if (first and 0.1 > s.speed and flag(s, .handbrake)) dt + l.standing else 0;
    const lo = math.maxss(optimum - 500, s.engine.idle);
    const hi = math.minss(500 + optimum, s.engine.rev_limit * 60 / 6.2831855);
    if (!(l.standing > 0.5)) {
        l.available = false;
        if (!l.active) return;
    } else {
        l.available = true;
        const rpm = 60 * limitedRate(s) / 6.2831855;
        if (!(rpm >= lo) or !(hi >= rpm)) {
            if (l.sweet_spot > 0) l.sweet_spot = l.sweet_spot - dt;
        } else {
            l.sweet_spot = dt + l.sweet_spot;
        }
        if (l.sweet_spot >= 0.2) {
            l.active = true;
            l.sweet_spot = 0.2;
        } else {
            if (!l.active) return;
            if (0.1 > l.sweet_spot) {
                l.active = false;
                l.sweet_spot = 0;
                return;
            }
        }
    }
    if (!first) l.active = false;
}

// the recorded steering a benchmark replays (VehicleControllerRecorder, the control hook; outside the car)
pub const Recorder = extern struct { playing: bool, adjusted_steering: f32 };

// .data 0x32ec5a0 = 3, a tweak, picks the pad's rate limit; the other modes (and computeDesiredSteeringAngle
// 0xef8240, which only they read) are not ported
const crossing_rate: f32 = 5; // .data 0x32ec59c

// 0xef83f0 Car::updateSteeringSystem: the input to a wheel angle, with the damage wobble, and on a pad the lock by
// speed, the response curve and a rate limit; then Ackermann, toe and the rendered wheel headings
pub fn steeringSystem(s: *State, dt: f32) void {
    var input = get(s, .steering);
    const wobble = s.damage.steer_wobble;
    if (wobble > 0) {
        const sp = s.speed / 5;
        const by_speed = if (sp > 1) 1 else math.maxss(sp, 0);
        const k = (1 - wobble) * 0.5 + wobble;
        var p = math.sin(s.internal_clock * 5.3) * math.sin(s.internal_clock * 8);
        if (0.5 > wobble) p = -@abs(p);
        input = input + by_speed * (p * k) * 0.14;
    }
    const x = if (input > 1) 1 else math.maxss(input, -1);
    const n = s.wheel_count;
    var kept: [4][2]f32 = undefined; // the headings a disconnected wheel keeps
    for (s.wheels[0..n], kept[0..n]) |w, *d| d.* = w.direction;
    const angle = if (s.pad_assists) rateLimit(s, padTarget(s, x), dt) else x * s.steering_lock;

    set(s, .adjusted_steering, angle);
    if (s.recorder.playing) set(s, .adjusted_steering, s.recorder.adjusted_steering);
    const adjusted = get(s, .adjusted_steering);
    applySteering(s, adjusted);
    applyToeIn(s);
    s.steering = adjusted;
    s.wheel_turn = adjusted / s.steering_lock;
    if (n == 0) return;

    for (s.wheels[0..n]) |*w| w.last_direction = w.direction[1];
    var rate = dt * 5;
    if (s.pad_assists) rate = rate * s.visual_turn_rate;
    for (s.wheels[0..n], kept[0..n], s.damage.disconnect_steering[0..n]) |*w, d, off| {
        const target = if (off) d[1] else w.direction[1];
        const r = w.render_direction[1];
        const next = if (target > rate + r) rate + r else math.maxss(target, r - rate);
        w.previous_render_direction = w.render_direction;
        if (r != next) w.render_direction = .{ if (next == 0) 1 else @sqrt(1 - next * next), next };
        const a = w.visual_damage_angle + w.visual_damage_angle;
        if (@abs(a) > 1e-7) w.direction = rotate(w.direction, math.sin(a), math.cos(a));
    }
    for (s.wheels[0..n], kept[0..n], s.damage.disconnect_steering[0..n]) |*w, d, off| {
        if (off) w.direction = d;
    }
}

// a heading {z, x} turned by sin, cos
fn rotate(dir: [2]f32, sn: f32, c: f32) [2]f32 {
    return .{ dir[0] * c - dir[1] * sn, sn * dir[0] + c * dir[1] };
}

// the pad's lock: by speed (graph or v0 / v), never under the minimum, opened by the drift angle with counter steer;
// the input through the response curve
fn padTarget(s: *State, x: f32) f32 {
    const lock = s.steering_lock;
    const drift: f32 = if (s.setup.counter_steer) s.drift_angle else 0;
    var lock_now = lock;
    if (s.setup.lock_by_graph) {
        const q = s.speed / s.setup.pad_lock_scale;
        const c = math.truncate(q);
        const i: usize, const frac: f32 = if (c < 0) .{ 0, 0 } else if (c <= 23) .{ @intCast(c), q - @as(f32, @floatFromInt(c)) } else .{ 23, 1 };
        const a = s.setup.pad_lock[i];
        lock_now = (s.setup.pad_lock[i + 1] - a) * frac + a;
    } else if (s.speed > 0 and s.speed > s.lowest_speed_max_lock) {
        lock_now = s.lowest_speed_max_lock / s.speed * lock;
    }
    lock_now = math.maxss(lock_now, s.pad_min_lock);
    const neg = math.minss(math.maxss(-drift, lock_now), lock);
    const pos = math.minss(math.maxss(drift, lock_now), lock);
    s.lock = .{ -neg, pos };
    const sign: f32 = if (x > 0) 1 else if (x < 0) -1 else 0;
    const lin = s.steering_linearity;
    const e = if (lin >= 0) lin + 1 else 1 / (1 - lin);
    const p = math.pow(x * sign, e) * sign;
    const t = (if (0 < p) pos else neg) * p;
    return if (t > lock) lock else math.maxss(t, -lock);
}

// the pad's rate limit (mode 3): faster back through the centre than away from it, and with the gap to the target
fn rateLimit(s: *const State, target: f32, dt: f32) f32 {
    const min, const max = s.lock;
    const range = max - min;
    const rate = dt * (range * 0.5 / s.steering_lock * s.steering_sensitivity);
    const cur = s.steering;
    const away = 0.5 * @abs(target - cur);
    const back = 0.5 * (@abs(target) + @abs(cur) * crossing_rate);
    var gap: f32 = 0;
    if (0 > cur and cur > target) {
        gap = away;
    } else if (cur >= 0 and cur > target) {
        gap = back;
    } else if (cur > 0 and target > cur) {
        gap = away;
    } else if (cur <= 0 and target > cur) {
        gap = back;
    }
    const f = unitPositive((gap / range - 0.05) / 0.45);
    const step = (f * (s.setup.steering.return_centre - 1) + 1) * s.setup.steering.high_rate * rate;
    var next: f32 = undefined;
    if (target > cur) {
        next = cur + step;
        if (next > target) next = target;
    } else {
        next = cur - step;
        if (target > next) next = target;
    }
    return if (next > max) max else math.maxss(next, min);
}

fn unitPositive(x: f32) f32 {
    return if (x > 1) 1 else if (0 < x) x else 0;
}

// 0xee84b0 Car::applySteering: Ackermann on the two steered wheels, the rear ones straight
fn applySteering(s: *State, angle: f32) void {
    const k = s.ackerman;
    const t = math.tan(@abs(angle));
    const tl = t * k.wheelbase;
    const h = 0.5 * k.track * t;
    const a = k.wheelbase - h;
    const b = k.wheelbase + h;
    const tl2 = tl * tl;
    const inv1 = 1 / @sqrt(a * a + tl2);
    const inv2 = 1 / @sqrt(tl2 + b * b);
    const outer: [2]f32 = .{ a * inv1, tl * inv1 }; // {z, x}
    const inner: [2]f32 = .{ inv2 * b, tl * inv2 };
    if (0 > angle) {
        s.wheels[k.wheels[0]].direction = .{ outer[0], -outer[1] };
        s.wheels[k.wheels[1]].direction = .{ inner[0], -inner[1] };
    } else {
        s.wheels[k.wheels[0]].direction = inner;
        s.wheels[k.wheels[1]].direction = outer;
    }
    s.wheels[0].direction = .{ 1, 0 };
    s.wheels[1].direction = .{ 1, 0 };
}

// 0xee86b0 Car::applyToeIn: static and travel toe, mirrored on the left, and the damage misalignment
fn applyToeIn(s: *State) void {
    for (s.wheels[0..s.wheel_count], 0..) |*w, i| {
        const p = if (i < 2) s.setup.toe[1] else s.setup.toe[0];
        var t = p.at(w.height) + w.toe_in;
        if (0 > w.rest_position[0]) t = 0 - t;
        t = t + s.damage.misalign[i];
        if (t != 0) w.direction = rotateToe(w.direction, math.sin(t), math.cos(t));
    }
}

fn rotateToe(dir: [2]f32, sn: f32, c: f32) [2]f32 {
    return .{ dir[1] * sn + dir[0] * c, c * dir[1] - sn * dir[0] };
}

// 0xee8930 Car::updateAutoClutch: engaged by the revs between a slip and a lock rate (by gear and throttle), with
// an anti stall; full while shifting
pub fn autoClutch(s: *State, dt: f32) void {
    const t = s.transmission;
    if (!t.auto_clutch) {
        set(s, .auto_clutch, 0);
        set(s, .final_clutch, if (t.dual_clutch) math.maxss(0, get(s, .manual_clutch)) else get(s, .manual_clutch));
        return;
    }
    const sequential = !t.auto_gearbox;
    const rates = s.setup.clutch[@intFromBool(sequential)];
    if (get(s, .handbrake) > 0 and 0.1 > s.speed) {
        set(s, .auto_clutch, 1);
    } else {
        const e = s.engine;
        const idle = e.idle;
        const crawl = t.ratios[1] * idle * s.wheels[0].tyre_radius;
        const c = unitPositive((@abs(drivetrain.centreAngvel(&s.drivetrain) * s.wheels[0].radius) - crawl) / (0.25 * crawl - crawl));
        const slip, const lock, const stall = window(s.*, c, idle, e.rev_limit);
        const ac = get(s, .auto_clutch);
        const lift = rates.lift * dt;
        const predicted = ((1 - ac) / lift + 1) * (e.rate - e.previous_rate) + e.previous_rate;
        s.anti_stall = if (predicted < stall) 1 else 0;
        var target = (lock - e.rate) / (lock - slip);
        if (t.gear == 10) target = math.maxss(target, math.minss(((lock + 183.25957) - e.rate) / ((lock + 183.25957) - (slip + 157.07964)), ac));
        target = unitPositive(math.maxss(target, s.anti_stall));
        set(s, .auto_clutch, if (target > ac) math.minss(target, ac + dt * rates.lift) else math.maxss(target, ac - dt * rates.lift));
    }
    if (!t.seamless) {
        const k = 0.5 * s.damage.gearbox + 1;
        if (k * rates.gear_time > t.since_change) set(s, .auto_clutch, 1);
        const off = rates.off_throttle_up;
        if (off > s.since_gear_up) {
            s.engine.throttle = 0;
        } else if (off + 0.15 > s.since_gear_up) {
            const q = (s.since_gear_up - off) / 0.15;
            const back: f32 = if (q > 1) 0 else if (q > 0) q - 1 else -1;
            s.engine.throttle = math.minss(s.engine.throttle, 1 + back * math.maxss(0, get(s, .auto_clutch) - get(s, .manual_clutch)));
        }
    }
    set(s, .final_clutch, if (t.dual_clutch) math.maxss(get(s, .auto_clutch), get(s, .manual_clutch)) else get(s, .auto_clutch));
}

// the clutch window by gear: rates where it slips and locks, and the predicted rate under which it opens to save the
// engine; c: how far the car rolls toward the first gear's idle speed
fn window(s: State, c: f32, idle: f32, rev: f32) [3]f32 {
    const pts = s.setup.clutch_points;
    const thr = s.engine.throttle;
    switch (s.transmission.gear) {
        1 => {
            const k = unitPositive((thr - 0.3) / 0.7);
            const full = pts[0] * idle;
            const none = pts[1] * idle;
            const slip = (((((rev - idle) * 0.5 + idle) - full) * c + full) - 1.25 * idle) * k + 1.25 * idle;
            const lock = ((c * (rev - none) + none) - 1.5 * idle) * k + 1.5 * idle;
            return .{ slip, lock, full };
        },
        10 => {
            const k = unitPositive((thr - 0.3) / 0.7);
            const slip = (pts[2] * idle - 1.25 * idle) * k + 1.25 * idle;
            const lock = k * (idle * pts[3] - 1.5 * idle) + 1.5 * idle;
            return .{ slip, lock, 1.25 * idle };
        },
        else => return .{ 1.05 * idle, 1.25 * idle, 1.05 * idle },
    }
}

const rev_match_blip: f32 = 250; // .data 0x32ec598, rpm, a tweak
const rev_match_span: f32 = 500; // .data 0x32ec594

// 0xee8f50 Car::updateRevMatching: after the upshift's throttle cut the throttle is opened toward the gearbox input
// speed, as far as the clutches allow
pub fn revMatching(s: *State) void {
    const t = s.transmission;
    const off = if (t.auto_gearbox) s.setup.clutch[0].off_throttle_up else s.setup.clutch[1].off_throttle_up;
    if (off > s.since_gear_up or !t.auto_clutch) return;
    const lo, const top = if (0 > rev_match_blip) .{ @as(f32, 0), @as(f32, 1) } else .{ (rev_match_blip + rev_match_blip) * std.math.pi / 60, rev_match_blip + 1 };
    const span = std.math.pi * (math.maxss(top, rev_match_span) + math.maxss(top, rev_match_span)) / 60 - lo;
    const r = unit(((s.drivetrain.gearbox.angvel_at_input - s.side.engine_rate) - lo) / span);
    const a = unit(get(s, .auto_clutch));
    const b = unit(1 - get(s, .manual_clutch));
    if (1e-6 > s.anti_stall) s.engine.throttle = math.maxss(math.minss(math.minss(r, a), b), s.engine.throttle);
}

// 0xf18e10 Car::updateGearbox: the auto box's choice, the shift requests (up, down, a gear) held off while a shift
// or its throttle cut runs; then updateGearChange
pub fn gearbox(s: *State, dt: f32) void {
    const t = &s.transmission;
    const old = t.gear;
    const auto = t.auto_gearbox;
    if (s.damage.gearbox_state != 8) {
        setFlag(s, .ignition, false);
        set(s, .manual_clutch, 0);
        set(s, .auto_clutch, 0);
        set(s, .brake, 1);
        set(s, .throttle, 0);
        set(s, .final_clutch, 0);
        if (old != 1) t.swing = 0;
        t.gear = 1;
    }
    if (t.manual_upshift) {
        setFlag(s, .gear_down, false);
        setRequested(s, 0);
        autoGearbox(s, dt, false);
    } else if (t.manual_override) {
        setRequested(s, 0);
        autoGearbox(s, dt, false);
    } else if (auto) {
        setFlag(s, .gear_down, false);
        setFlag(s, .gear_up, false);
        setRequested(s, 0);
        autoGearbox(s, dt, false);
    } else {
        autoGearbox(s, dt, true);
    }
    const rates = s.setup.clutch[@intFromBool(!auto)];
    const busy = (0.5 * s.damage.gearbox + 1) * rates.gear_time > t.since_change or rates.off_throttle_up > s.since_gear_up or 0.2 >= t.since_change;
    const worn = s.damage.gear_drop > 0.8;
    const top = t.top_gear;
    var g = if (worn and old == top - 1) top else old;
    if (busy or rates.on_throttle_down > s.since_gear_down) {
        setFlag(s, .gear_down, false);
        setFlag(s, .gear_up, false);
    } else {
        if (flag(s, .gear_down)) {
            setFlag(s, .gear_down, false);
            if (old == 0) {
                g = 10;
            } else if (old != 10) {
                g = old - 1;
                if (auto and 0.8 * s.engine.idle > limitedRate(s) and g > 1) g = 1;
                if (g != 0) s.since_gear_down = 0;
            }
            if (worn and g == top - 1) g = top - 2;
        }
        if (flag(s, .gear_up)) {
            setFlag(s, .gear_up, false);
            if (old == 10) {
                g = 0;
            } else if (t.ratios[@intCast(old + 1)] > 0) {
                g = old + 1;
                if (old != 0) s.since_gear_up = 0;
            }
            if (worn and g == top - 1) g = top;
        }
    }
    var target = g;
    if (s.gear_request != 11) {
        target = s.gear_request;
        s.gear_request = 11;
    }
    const req = requested(s);
    if (req > 0) {
        const ratio = t.ratios[@intCast(req)];
        if (ratio == 0 or req > top and req != 10) {
            if (req == 9) return toNeutral(s, old);
            setRequested(s, 0);
            if (target != old) shift(s, target) else if (target == 0) t.jammed = false;
            return;
        }
        if (req == old) {
            setRequested(s, 0);
            if (req == 9) shift(s, 0);
            return;
        }
        const v = @abs(ratio * limitedRate(s) * s.wheels[0].radius);
        if (v > @abs(drivetrain.centreAngvel(&s.drivetrain) * s.wheels[0].radius)) s.since_gear_up = 0 else s.since_gear_down = 0;
        if (req == 9) return toNeutral(s, old);
        setRequested(s, 0);
        return shift(s, req);
    }
    setRequested(s, 0);
    if (req < 0) {
        if (old != 10) shift(s, 10);
        return;
    }
    if (target != old) shift(s, target) else if (target == 0) t.jammed = false;
}

fn setRequested(s: *State, g: i32) void {
    s.controls[@intFromEnum(Control.gear)] = @bitCast(g);
}

fn toNeutral(s: *State, old: i32) void {
    setRequested(s, 0);
    if (old != 0) shift(s, 0) else s.transmission.jammed = false;
}

fn shift(s: *State, g: i32) void {
    const t = &s.transmission;
    t.damaged_drop = t.auto_gearbox and !t.auto_reverse;
    t.since_change = 0;
    t.swing = 0;
    gearChange(s, g);
    if (g == 0) t.jammed = false;
}

// 0xef7fa0 Car::updateGearChange: into the gear unless a manual clutch grinds it (the gears jam until the clutch
// goes down); a worn gearbox may crunch an upshift
pub fn gearChange(s: *State, g: i32) void {
    const t = &s.transmission;
    const old = t.gear;
    const was_jammed = t.jammed;
    const ratio = t.ratios[@intCast(g)];
    const inv: f32 = if (ratio == 0) 0 else 1 / ratio;
    const ts = drivetrain.centreAngvel(&s.drivetrain);
    var apply = true;
    var crunch = false;
    if (g == 0) {
        t.jammed = false;
    } else if (!t.auto_clutch) {
        var x = (@abs(s.engine.rate - ts * inv) - 10.471975) / 2.6179943;
        const clutch = get(s, .final_clutch);
        const grind = if (x > 0.98) 0.98 > clutch else blk: {
            x = if (0 < x) x else 0;
            break :blk x > clutch;
        };
        if (grind and !t.seamless) t.jammed = true;
        if (grind and !t.seamless or t.jammed) {
            apply = false;
            crunch = !was_jammed;
        }
    }
    if (apply) {
        t.since_change = 0;
        t.gear = g;
        t.swing = 0;
        t.previous = old;
        const time = if (t.auto_gearbox) t.shift_time[0] else t.shift_time[1];
        t.seamless = time > 0 and old != 10 and old != 0 and g != 10 and g != 0;
    }
    const drop = s.damage.gear_drop;
    if (drop > 0.5 and g != 1 and old != 10 and @intFromBool(g > old) > @intFromBool(crunch)) {
        const r: f32 = @floatFromInt(engine.random(&s.engine.random) & 0xff);
        crunch = ((drop - 0.5) + (drop - 0.5)) * 256 > r;
    }
    if (crunch) {
        s.gear_crunch = 1;
    } else if (!t.seamless) {
        s.gear_clunk = 1;
    }
}

// 0xf179d0 Car::updateAutoGearbox: the torque at the wheels in each gear (engine, turbo, friction) weighted by when
// a shift into it makes sense; the best gear after a swing timer. advise: only update the advised gear
pub fn autoGearbox(s: *State, dt: f32, advise: bool) void {
    const t = &s.transmission;
    const e = &s.engine;
    const gear = t.gear;
    var driving = gear != 0 and gear != 10;
    if (driving) {
        const thr = e.throttle;
        const lifted = s.autogear.throttle > thr;
        s.autogear.timer = if (lifted) 0 else s.autogear.timer + dt;
        s.autogear.throttle = thr;
        const target = if (t.ratios[@intCast(gear)] > 0) bestGear(s, lifted) else gear;
        const up_rate = rateFrom(thr);
        const down_rate = rateFrom(s.brake);
        var swing = t.swing;
        var settle = false;
        var up = false;
        var down = false;
        var hold = false; // past the "gear changed" fallthrough
        if (target > gear and get(s, .final_clutch) == 0) {
            swing = swing + dt * up_rate;
            if (0 > swing) {
                swing = 0;
                settle = true;
            } else hold = true;
        } else if (target < gear) {
            const y = swing - down_rate * dt;
            if (y > 0) {
                swing = 0;
                settle = true;
            } else swing = y;
        } else if (swing > 0) {
            swing = swing - dt;
            if (0 > swing) {
                swing = 0;
                settle = true;
            }
        } else {
            const y = dt + swing;
            if (y > 0) {
                swing = 0;
                settle = true;
            } else swing = y;
        }
        if (!settle) {
            if (!hold) driving = false;
            if (swing > 2) {
                swing = 2;
                up = driving;
            } else if (swing > -2) {
                if (swing > 1 and driving) {
                    up = true;
                } else down = target < gear and -1 > swing;
            } else {
                swing = -2;
                down = target < gear;
            }
        }
        s.best_gear = if (up or down) target else gear;
        if (!advise) {
            if (up) setFlag(s, .gear_up, true);
            if (down) setFlag(s, .gear_down, true);
        }
        t.swing = swing;
    }
    if (advise) return;
    if (t.auto_reverse) autoReverse(s);
    t.jammed = false;
}

fn rateFrom(pedal: f32) f32 {
    const x = pedal * -0.3 + 0.5;
    return if (x > 1e-6) 1 / x else 0;
}

// the gear the auto box would pick: each gear's torque at the wheels for the throttle and for full power, times
// the lowest of the factors that make a shift there sensible
fn bestGear(s: *State, lifted: bool) i32 {
    const t = s.transmission;
    const e = s.engine;
    const gear = t.gear;
    const rev = e.rev_limit;
    const idle = e.idle;
    const thr = e.throttle;
    const now = limitedRate(s) * t.ratios[@intCast(gear)];
    const rates = s.setup.clutch[@intFromBool(!t.auto_gearbox)];
    const clutch = get(s, .final_clutch);
    const pedal = thr - s.brake;
    const slope = body.dot4(.{ 0, -9.81, 0, 0 }, s.body.rows[2]);
    const projected = slope * rates.off_throttle_up / s.wheels[0].radius + now;
    const k = (pedal + 1) * 0.5;

    var load_sum: f32 = 0;
    var slip_sum: f32 = 0;
    var counted: i32 = 0;
    var driven: i32 = 0;
    for (s.wheels, s.damage.detached) |w, off| {
        if (off) continue;
        if (w.drive_proportion > 0) {
            const g = w.tyre.set.longitudinal;
            const l = w.load_multiple;
            const peak = if (w.tyre.has_record) (if (l > 4) 4 else if (1 < l) l else 1) * (g.peak_x4 - g.peak_x1) + g.peak_x1 else 0.1;
            slip_sum = slip_sum - w.tyre.slip.ratio / peak;
            driven += 1;
        }
        load_sum = load_sum + w.load_multiple;
        counted += 1;
    }
    const avg_slip = slip_sum / @as(f32, @floatFromInt(driven));
    const rho = aero.airDensity(s.altitude, s.sea_level_raw);
    const by_load = unit(load_sum / @as(f32, @floatFromInt(counted)) / 0.1);

    var throttle_torque = [_]f32{0} ** 11;
    var full_torque = [_]f32{0} ** 11;
    var gear_rate = [_]f32{0} ** 11;
    var weight = [_]f32{0} ** 11;
    for (1..11) |gi| {
        const g: i32 = @intCast(gi);
        const ratio = t.ratios[gi];
        if (!(ratio > 0)) continue;
        const r0 = (if (g == gear) now else projected) / ratio;
        const rate = if (0.001 < r0) r0 else 0.001;
        gear_rate[gi] = rate;
        if (g != gear and (!(rate > idle) or !((math.maxss(s.brake, thr) * 0.34999996 + 0.6) * rev > rate))) continue;

        const pressure = if (e.turbo.enabled) engine.estimatePressure(e.turbo, rate, rho, e.restrictor) else blk: {
            const flow = e.turbo.capacity * rate * rho;
            break :blk unit(e.restrictor / (if (1e-6 < flow) flow else 1e-6)) * rho;
        };
        const drive = engine.powerAt(e, rate) * e.power_scale * e.difficulty * pressure;
        const friction = engine.frictionAt(e, rate, clutch);
        const fr = (e.friction + e.friction_drive) * e.drivetrain_friction_scale * 3000 * e.drift_friction[0] * e.drift_friction[1];
        var gf = @abs(rate) / 30;
        if (rate > 0) gf = -math.minss(gf, math.maxss(0, (@abs(rate - idle) - 1) / 30));
        const drag = e.inertia / 0.2 * fr * gf;
        const back = s.side.transmission_torque * ratio;
        const full = (drive + drag) / rate - back;
        const part = (((1 - thr) * friction + thr * drive) + (1 - clutch) * drag) / rate - back;
        const sign: f32 = if (pedal > 0) 1 else if (pedal < 0) -1 else 0;

        var f_slip: f32 = 1;
        var f_down: f32 = 1;
        var f_cross: f32 = 1;
        var f_rate: f32 = 1;
        var f_idle: f32 = 1;
        var f_lift: f32 = 1;
        var f_clutch: f32 = 1;
        var f_load: f32 = 1;
        if (g <= gear) {
            const cross = math.minss(0.96 * rev, t.crossover[gi]);
            if (cross > idle) {
                const lo = math.maxss(0.94 * rev, cross);
                const mid = (cross - lo) * k + lo;
                const top = 0.5 * (math.maxss(rev, mid + 1) + mid);
                f_cross = unit((top - rate) / (top - mid));
            }
        }
        if (!(g == 1 and gear > 0)) {
            if (g != 1) {
                const c2 = (t.ratios[gi - 1] * t.crossover[gi - 1] / ratio + idle) * 0.5;
                const lo2 = math.minss(idle, c2 - 1);
                f_down = unit((rate - lo2) / (c2 - lo2));
            }
            const by_idle = unit((rate - 1.05 * idle) / (1.25 * idle - 1.05 * idle));
            if (g <= gear) {
                f_idle = by_idle;
            } else {
                const cur = gear_rate[@intCast(gear)];
                f_rate = unit(((cur - idle) / (rev - idle) - 0.3) / 0.2);
                if (g != 1) f_idle = by_idle;
                f_lift = if (lifted) 0 else math.maxss(unit((s.autogear.timer - 1) + (s.autogear.timer - 1)), unit((cur / rev - 0.92) / 0.050000012)) * unit(thr + thr);
                f_slip = unit(-(avg_slip - 2));
                f_clutch = if (clutch == 0) 1 else 0;
                f_load = by_load;
            }
        }
        var f = math.minss(f_slip, f_down);
        for ([_]f32{ f_cross, f_rate, f_idle, f_lift, f_clutch, f_load }) |x| f = math.minss(f, x);
        weight[gi] = f;
        throttle_torque[gi] = sign * part * f;
        full_torque[gi] = full * f;
    }
    throttle_torque[1] = throttle_torque[1] * 0.9;
    full_torque[1] = 0.9 * full_torque[1];
    if (0.8 * idle > limitedRate(s)) {
        throttle_torque[1] = 1;
        full_torque[1] = 1;
        weight[1] = 1;
    }
    var b1: usize = @intCast(gear);
    var b2: usize = @intCast(gear);
    var last: i32 = 0;
    for (0..11) |i| {
        if (throttle_torque[i] > 0 and throttle_torque[i] > throttle_torque[b1]) b1 = i;
        if (full_torque[i] > 0 and full_torque[i] > full_torque[b2]) b2 = i;
        if (weight[i] != 0) last = @intCast(i);
    }
    if (last == 0) return gear;
    const x1: i32 = if (b1 > last) last else if (b1 > 0) @intCast(b1) else 1;
    const x2: i32 = if (b2 > last) last else if (b2 > 0) @intCast(b2) else 1;
    return if (x2 > x1) x1 else x2;
}

// with the auto reverse: out of first into reverse on the brake at a standstill, and back out on the throttle
fn autoReverse(s: *State) void {
    const t = &s.transmission;
    const st = s.setup.reverse;
    if (t.gear == 0) {
        t.swing = 0;
        t.gear = 1;
    }
    const thr = get(s, .throttle);
    const brk = get(s, .brake);
    var to_reverse = false;
    if (thr != 0) s.permit_reverse = false;
    if (st.first_to_reverse_speed > s.speed) {
        if (thr == 0 and s.permit_reverse) {
            to_reverse = true;
        } else if (st.threshold > math.maxss(thr, brk)) {
            s.permit_reverse = true;
            to_reverse = true;
        } else to_reverse = s.permit_reverse;
    }
    if (to_reverse and brk > st.threshold + thr) {
        if (t.gear != 10) t.swing = 0;
        t.gear = 10;
        t.damaged_drop = false;
        if (st.reverse_to_first_speed > s.speed and thr > brk + 0.05) {
            t.swing = 0;
            t.gear = 1;
            t.damaged_drop = false;
        }
        return;
    }
    if (st.reverse_to_first_speed > s.speed and (t.gear == 10 or t.gear == 0) and thr > 0.05 + brk) {
        t.swing = 0;
        t.gear = 1;
        t.damaged_drop = false;
    }
}
