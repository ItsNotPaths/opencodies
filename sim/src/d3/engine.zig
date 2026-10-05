const std = @import("std");
const body = @import("../shared/body.zig");
const vec = @import("../shared/vec.zig");
const car_ = @import("car.zig");
const force = @import("force.zig");
const ground = @import("../shared/ground.zig");

const Car = @import("../car.zig").Car;
const splat = vec.splat;

// 0x7e1fd0, N m at the crank; rate is the mean of the last step
pub fn torque(car: Car, rate: f32, throttle: f32, rolling_back: bool) f32 {
    const e = car.engine;
    const c = e.curve.power;
    const thr = if (rate > e.rev_limit or rate < 0.9 * e.idle) 0 else throttle;
    const p = if (rolling_back) std.mem.max(f32, c.table) else power(car, rate); // E[0x3e]: peak power
    const drive = p / c.divisor;
    const t = ((1 - thr) * (friction(car, rate) * (e.inertia * 5)) + thr * drive) / @max(@abs(rate), 20);
    const starting = rate < e.idle and throttle > 0.5; // 0x7ef5a0 auto start
    return if (starting) @max(t, e.inertia * starter_torque * starter_kick) else t;
}

const starter_torque: f32 = 333.33; // per kg m^2 of crank inertia
const starter_kick: f32 = 1; // E+0xa4, rerolled from the car LCG below 0.31 idle: not ported

fn friction(car: Car, rate: f32) f32 {
    const per_rate: f32 = 1.0 / 30.0;
    const f = car.engine.curve.power.friction;
    if (rate <= 0) return @abs(rate) * per_rate * f;
    const off_idle = @max((@abs(rate - car.engine.idle) - 1) * per_rate, 0);
    return -f * @min(off_idle, @abs(rate) * per_rate);
}

fn power(car: Car, rate: f32) f32 {
    const c = car.engine.curve.power;
    const x = rate / c.step;
    const last: f32 = @floatFromInt(c.table.len - 1);
    if (!(x >= 0 and x <= last)) return 0;
    const i: usize = @intFromFloat(@min(x, last - 1));
    if (c.table[i] == 0) return 0; // no lerp up from a zero entry
    return (c.table[i + 1] - c.table[i]) * (x - @as(f32, @floatFromInt(i))) + c.table[i];
}

// 0x7d8c40: the crank's speed change turns the body about its forward axis (inline engine)
pub fn react(car: Car, s: *body.State, e: *force.Engine, wheels: [4]force.Wheel) void {
    for (wheels) |w| if (w.ground == ground.none) return;
    const change = (e.mean - e.reacted) * car.engine.inertia / car.inertia[2];
    e.reacted = e.mean;
    s.angvel += s.axes[2] * splat(change);
}
