// Gear changes, the clutch the box or the driver works, stalls, and when the clutch holds.
// FH2 UpdateShiftState 0x82e2ba08, UpdateShiftStateManualWithClutch 0x82e2c080 (FH1 0x82d31280, the same code),
// UpdateAutomaticShiftControls 0x82e2c658, CalcClutchVel 0x82e2b620, ClutchCalcCurMaxTorque 0x82e1e498,
// PostIntegrationStep 0x82e25c58. Not ported: the downshift throttle blip, gearbox damage from shifts without
// the clutch (longer shift times), CVT, the auto-hold brake.
const std = @import("std");

pub const neutral = 0;
pub const reverse = 10;

pub const Mode = enum {
    automatic, // the box picks the gear and works the clutch
    manual, // the driver picks the gear, the box works the clutch
    clutch_pedal, // the driver picks the gear and works the clutch
};

// the game's shift states; normal is its 2, stalled its 10
pub const Phase = enum { normal, launch, clutch_in, change, match, clutch_out, starting, stalled };

pub const State = struct {
    gear: usize = neutral,
    target: usize = neutral,
    phase: Phase = .normal,
    timer: f32 = 0,
    clutch: f32 = 0, // 0 in .. 1 out
    used_pedal: bool = false, // the driver pressed the pedal past 0.8 for this shift
    locked: bool = false,
    last_slip: f32 = 0,
};

pub const Params = struct {
    gears: [11]f32, // engine/shaft
    top: usize,
    shift_time: f32, // s, an automatic box; the manual boxes take 0.75 of it
    clutch_in: f32, // per s, an automatic box; the manual boxes are 1/0.75 faster
    clutch_out: f32,
    mode: Mode = .manual,
    up: [11]f32 = @splat(0), // rad/s: shift up past this
    down: [11]f32 = @splat(0),
    stall: f32, // rad/s: below it the engine stops (clutch pedal box only)
    idle: f32,
    start: f32, // rad/s the clutch holds the engine near on a launch
    redline: f32,
};

const manual_time_scale = 0.75; // ShiftTimeScaleManualTrans
const pedal_rate: f32 = 15; // ClutchMaxVel, per s
const box_rate: f32 = 3; // ClutchVelShiftWithoutClutch: the box's own speed when the driver leaves the pedal
const pedal_dead = 0.2; // ClutchPedalDistBeforeTorqueStarts
const pedal_exponent = 3; // ClutchTorqueCurveExponent
const stall_time = 2; // s, ClutchStallTimeout
const reverse_speed = 15 * 0.44704; // m/s, MaxForwardVelShiftReverseMPH (FH1 ini)

pub fn shiftPoints(p: *Params) void {
    for (1..p.top + 1) |g| {
        p.up[g] = if (p.mode == .automatic and g < p.top) p.redline * 0.96 else p.redline;
        p.down[g] = if (g < 2) 0 else if (p.gears[g - 1] <= p.gears[g] or p.gears[g] <= 0.001) p.redline * 0.5 else p.gears[g] / p.gears[g - 1] * p.up[g - 1] * 0.85;
    }
}

// the share of the clutch's torque it passes at the position; the pedal box has a dead zone and a cubic curve
pub fn engagement(p: Params, s: State) f32 {
    if (p.mode != .clutch_pedal) return 1 - s.clutch;
    return std.math.pow(f32, std.math.clamp(1 - s.clutch / (1 - pedal_dead), 0, 1), pedal_exponent);
}

pub const Frame = struct {
    request: ?usize, // a gear: 0 neutral, 1.., 10 reverse
    throttle: f32,
    pedal: f32, // 0 up .. 1 down
    engine: f32, // rad/s
    shaft: f32, // the gearbox output, rad/s
    forward_speed: f32, // m/s
    slip_ok: bool, // no driven wheel spins past 1.1 normalized slip
};

// once a frame: the gear, the clutch position, the stall; gives the throttle the box allows
pub fn update(p: Params, s: *State, f: Frame, h: f32) f32 {
    const time = if (p.mode == .automatic) p.shift_time else p.shift_time * manual_time_scale;
    const speeds = [2]f32{ p.clutch_in, p.clutch_out };
    const in_rate, const out_rate = if (p.mode == .automatic) speeds else .{ speeds[0] / manual_time_scale, speeds[1] / manual_time_scale };

    if (p.mode == .automatic and s.phase == .normal and s.locked and s.gear >= 1 and s.gear <= p.top) {
        if (f.engine < p.down[s.gear] and s.gear > 1) s.target = s.gear - 1;
        if (f.engine > p.up[s.gear] and f.slip_ok and s.gear < p.top) s.target = s.gear + 1;
    }
    if (f.request) |r| if (r != s.target and (r <= p.top or r == reverse) and !(r == reverse and f.forward_speed > reverse_speed)) {
        s.target = r;
        s.used_pedal = false;
        s.phase = .clutch_in; // from any phase, as the game's shift requests
    };
    if (p.mode == .clutch_pedal) return pedalBox(p, s, f, time, h);

    if (s.target != s.gear and (s.phase == .normal or s.phase == .launch)) s.phase = .clutch_in;
    // from rest the clutch slips until the wheels catch up with the launch speed
    if (s.phase == .normal and s.gear != neutral and @abs(f.shaft * p.gears[s.gear]) < p.idle) {
        s.phase = .launch;
        s.locked = false;
    }
    switch (s.phase) {
        .normal, .starting, .stalled => {},
        .launch => {
            const low = p.idle + 0.2 * (p.start - p.idle);
            const high = p.start + 0.2 * (p.redline - p.start);
            const hold = (high - low) * std.math.clamp(f.throttle, 0, 1) + low;
            if (@abs(f.shaft * p.gears[s.gear]) > hold) {
                s.phase = .clutch_out;
            } else s.clutch = std.math.clamp(s.clutch + (hold - f.engine) / p.redline * 53 * h, 0, 1);
        },
        .clutch_in => {
            s.clutch += in_rate * h;
            if (s.clutch >= 1) {
                s.clutch = 1;
                s.timer = 0;
                s.phase = .change;
            }
        },
        .change => {
            s.timer += h;
            if (s.timer >= time) {
                s.gear = s.target;
                s.timer = 0;
                s.locked = false;
                s.phase = if (s.gear == neutral) .normal else .match;
            }
        },
        .match => {
            s.timer += h;
            if (s.timer >= 0.05) s.phase = .clutch_out;
        },
        .clutch_out => {
            s.clutch -= out_rate * h;
            if (s.clutch <= 0) {
                s.clutch = 0;
                s.phase = .normal;
            }
        },
    }
    return if (s.phase == .clutch_in or s.phase == .change) 0 else f.throttle;
}

// the pedal moves the clutch at up to 15/s; a shift opens it at least at 3/s and, if the driver did not
// use the pedal, closes it at most at 3/s. No throttle blip, no match phase.
fn pedalBox(p: Params, s: *State, f: Frame, time: f32, h: f32) f32 {
    if (s.phase == .clutch_in and f.pedal > 0.8) s.used_pedal = true;
    // the engine is at or below its stall speed: the clutch opens, the throttle is cut for 2 s
    if (f.engine <= p.stall) {
        s.phase = .stalled;
        s.clutch = 1;
        s.locked = false;
        s.timer = 0;
    }
    const d = f.pedal - s.clutch;
    var rate = if (@abs(d) < pedal_rate * h) d / h else std.math.sign(d) * pedal_rate;
    switch (s.phase) {
        .clutch_in => rate = @max(rate, box_rate),
        .clutch_out => if (!s.used_pedal) {
            rate = @max(rate, -box_rate);
        },
        .change, .starting, .stalled => rate = 0,
        else => {},
    }
    s.clutch = std.math.clamp(s.clutch + rate * h, 0, 1);
    switch (s.phase) {
        .clutch_in => if (s.clutch >= 1) {
            if (s.target == neutral) s.gear = neutral else {
                s.timer = 0;
                s.phase = .change;
            }
        },
        .change => {
            s.timer += h;
            if (s.timer >= time) {
                s.gear = s.target;
                s.locked = false;
                s.phase = .clutch_out;
            }
        },
        .clutch_out => if (s.clutch <= 0) {
            s.phase = .normal;
        },
        .stalled => {
            s.timer += h;
            if (s.timer >= stall_time) s.phase = .starting;
            return 0;
        },
        // after a stall: the clutch stays out until the driver lets the pedal up from the floor with the engine
        // revving or the wheels turning
        .starting => {
            s.gear = s.target;
            const at_stall = @abs(f.shaft * p.gears[s.gear]) <= p.stall and f.engine <= p.idle + 0.2 * (p.start - p.idle);
            if (!at_stall and f.pedal < 1) {
                s.used_pedal = true;
                s.phase = .clutch_out;
            }
        },
        .normal, .launch, .match => {},
    }
    return f.throttle;
}

// after each substep: the clutch locks when it can hold the engine and the slip is small or crosses zero
pub fn lock(p: Params, s: *State, capacity: f32, engine_torque: f32, engine: f32, shaft: f32) void {
    if (s.gear == neutral) {
        s.locked = false;
        return;
    }
    const slip = engine - shaft * p.gears[s.gear];
    if (s.locked) {
        if (@abs(engine_torque) > 1.02 * capacity) s.locked = false;
    } else if (capacity >= @abs(engine_torque) and s.phase == .normal and (@abs(slip) < 31.4159 or slip * s.last_slip < 0)) {
        s.locked = true;
    }
    s.last_slip = slip;
}
