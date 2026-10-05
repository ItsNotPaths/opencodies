// Pad steering: a speed-sensitive lock, rate limits, a slow creep past the lock at full input, and Ackermann.
// FH2 CalcSteerDerivativeDataWithControllerFilters 0x82e323a0, CalcSteerDerivativeDataAccelAngVel
// 0x82e1d728, CalcWheelSteerAnglesAckerman 0x82e1d838. Surface lock caps and the AI blend are not ported.
const std = @import("std");
const SpeedSteering = @import("../car.zig").SpeedSteering;

pub const curve_speed = 67.056; // m/s of the last lock sample
pub const samples = 32;

pub const Params = struct {
    max: f32, // rad
    lock: [samples]f32, // rad by speed over 0..curve_speed
    rates: ?SpeedSteering, // null: the wheels follow the input at once
};

pub const State = struct {
    angle: f32 = 0,
    rate: f32 = 0, // rad/s
    peak_offset: f32 = 0, // rad past the lock while it finds the tyres' peak
};

// Catmull-Rom through the lock samples
pub fn lockAt(p: Params, speed: f32) f32 {
    const x = std.math.clamp(speed, 0, curve_speed) * ((samples - 1.0) / curve_speed);
    const i: usize = @min(@as(usize, @intFromFloat(x)), samples - 2);
    const t = x - @as(f32, @floatFromInt(i));
    const p1 = p.lock[i];
    const p2 = p.lock[i + 1];
    const p0 = if (i == 0) 2 * p1 - p2 else p.lock[i - 1];
    const p3 = if (i + 2 < samples) p.lock[i + 2] else 2 * p2 - p1;
    return 0.5 * ((2 * p1) + (-p0 + p2) * t + (2 * p0 - 5 * p1 + 4 * p2 - p3) * t * t + (-p0 + 3 * p1 - 3 * p2 + p3) * t * t * t);
}

// the angle rate toward goal: it ramps up at acc, stops at the goal
fn toward(s: *State, goal: f32, max_rate: f32, acc: f32, h: f32) f32 {
    const d = goal - s.angle;
    if (std.math.sign(s.rate) != std.math.sign(d)) s.rate = 0;
    s.rate = std.math.clamp(s.rate + std.math.sign(d) * acc * h, -max_rate, max_rate);
    if (@abs(d) < @abs(s.rate) * h) s.rate = d / h;
    return s.rate;
}

// front_slip: the front tyres' normalized slip angle, load-weighted; opposed: a front tyre slips against the turn
pub fn rate(p: Params, s: *State, input: f32, forward_speed: f32, speed: f32, front_slip: f32, opposed: bool, h: f32) f32 {
    const r = p.rates orelse {
        s.rate = (std.math.clamp(input, -1, 1) * p.max - s.angle) / h;
        return s.rate;
    };
    if (forward_speed < 2 or opposed or input * s.angle < 0 or (s.angle == 0 and input == 0)) {
        const goal = p.max * input;
        const away = @abs(goal) > @abs(s.angle);
        const max_rate = if (away) r.turn_rate else r.return_rate;
        s.peak_offset = 0;
        return toward(s, goal, max_rate, max_rate / r.rate_ramp, h);
    }
    const lim = std.math.clamp(lockAt(p, speed), @min(r.min_lock, p.max), @max(r.min_lock, p.max));
    const target = input * lim;
    const sc = std.math.clamp(speed, r.slow_speed, r.fast_speed);
    const ratio = lockAt(p, sc) / lockAt(p, r.slow_speed);
    const k = if (r.fast_speed > r.slow_speed) (sc - r.slow_speed) / (r.fast_speed - r.slow_speed) else 0;
    var max_rate = ratio * r.turn_rate * ((r.fast_rate_scale - 1) * k + 1);
    const full = @abs(input) >= 0.976;
    if (full and !(@abs(s.angle) < lim - max_rate * h or @abs(front_slip) >= 1.05)) {
        max_rate = @min(max_rate, r.find_peak_rate);
        s.peak_offset += std.math.sign(target) * max_rate * h;
    } else if (!full) s.peak_offset = 0;
    var goal = std.math.clamp(target + s.peak_offset, -p.max, p.max);
    if (full and @abs(goal) < @abs(s.angle) and goal * s.angle > 0) goal = s.angle;
    if (@abs(front_slip) >= 1.2 and s.angle * front_slip > 0) {
        const m = @max(@abs(s.angle), r.min_lock);
        goal = std.math.clamp(goal, -m, m);
    }
    return toward(s, goal, max_rate, max_rate / r.rate_ramp, h);
}

// the angle of a steered wheel at body x on a car of this wheelbase
pub fn ackermann(angle: f32, wheelbase: f32, x: f32) f32 {
    if (@abs(angle) <= 1e-4) return angle;
    return std.math.atan(wheelbase / (wheelbase / @tan(angle) - x));
}
