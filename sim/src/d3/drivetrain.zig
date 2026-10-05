const std = @import("std");
const car_ = @import("car.zig");
const force = @import("force.zig");

const Car = @import("../car.zig").Car;
const Diff = @import("../car.zig").RateDiff;
const Step = force.Step;

pub const reverse: i32 = 10;

pub fn ratio(car: Car, gear: i32) f32 {
    const g = car.gearbox;
    if (gear == reverse or gear >= 1 and gear <= g.top_gear) return g.ratios[@intCast(gear)];
    return g.ratios[0];
}

pub const Shift = extern struct { ratio: f32 = 0, timer: f32 = 0 }; // T+0x88c, T+0x78c

// 0x7fd240: the ratio follows the gear once the shift delay has run out
pub fn shift(car: Car, s: *Shift, gear: i32, dt: f32) f32 {
    const target = ratio(car, gear);
    if (s.ratio == target) {
        s.timer = 0;
        return s.ratio;
    }
    if (!(s.timer > 0)) s.timer = car.gearbox.shift_delay orelse 0;
    s.timer -= dt;
    if (!(s.timer > 0)) s.ratio = target;
    return s.ratio;
}

// N m at the crank, 0 in neutral; pedal 1 is fully disengaged
pub fn clutchCap(car: Car, gear: i32, pedal: f32) f32 {
    if (gear == 0) return 0;
    return car.gearbox.clutch_torque * (1 - pedal) / @abs(ratio(car, gear));
}

pub fn diffThrottle(car: Car, throttle: f32) f32 {
    return (1 - throttle) * car.diffs.rate.off_throttle + throttle;
}

// 0x7e64a0 A: the clutch pulls the mean wheel spin toward the crank with 1.5x the matching impulse
pub fn clutch(car: Car, st: *Step, h: f32) void {
    st.engine_rate += st.engine_torque * h / car.engine.inertia;
    if (st.ratio == 0) return;
    var spin: f32 = 0;
    const order = car.clutch_order orelse [4]usize{ 0, 1, 2, 3 };
    for (order) |i| spin += st.wheels[i].spin;
    var split: f32 = 0;
    var k: f32 = 0;
    var q: [4]f32 = undefined;
    for (car.wheels, &q) |p, *qi| {
        split += p.drive_proportion;
        qi.* = p.drive_proportion * h / p.inertia;
        k += qi.*;
    }
    const share = 1 / @as(f32, order.len);
    const dv = share * spin - st.engine_rate * st.ratio;
    const ke = st.ratio * split * h / car.engine.inertia;
    const j = @min(@abs(dv) * 1.5 / (@abs(share * k) + @abs(ke)), st.clutch_cap);
    const signed = if (dv >= 0) -j else j;
    for (&st.wheels, car.wheels, q) |*w, p, qi| w.turn(qi * signed, p.radius);
    st.engine_rate -= signed * ke;
}

// 0x7e64a0 B: rear, front and centre limited-slip diffs; each wheel divides by its inertia the way D3 compiled it
pub fn diffs(car: Car, st: *Step, h: f32) void {
    const lock = if (st.ratio != 0) @abs(st.engine_torque) * 0.5 * 0.01 * (1 - st.clutch) / @abs(st.ratio) else 0;
    const w = &st.wheels;
    const p = car.wheels;
    const d = car.diffs.rate;
    const rear = lsd(d.rear, w[1].spin - w[0].spin, h, st.diff_throttle, lock);
    w[0].turn(rear * (1 / p[0].inertia), p[0].radius);
    w[1].turn(-1 / p[1].inertia * rear, p[1].radius);
    const front = lsd(d.front, w[3].spin - w[2].spin, h, st.diff_throttle, lock);
    w[2].turn(front / p[2].inertia, p[2].radius);
    w[3].turn(-1 / p[3].inertia * front, p[3].radius);
    const centre = lsd(d.centre, w[3].spin + w[2].spin - (w[1].spin + w[0].spin), h, st.diff_throttle, lock);
    w[0].turn(centre * (1 / p[0].inertia), p[0].radius);
    w[1].turn(centre / p[1].inertia, p[1].radius);
    w[2].turn(-1 / p[2].inertia * centre, p[2].radius);
    w[3].turn(-1 / p[3].inertia * centre, p[3].radius);
}

fn lsd(d: Diff, rate: f32, h: f32, throttle: f32, lock: f32) f32 {
    const s: f32 = if (rate > 0) 1 else -1;
    const k = std.math.clamp((s * rate - d.no_effect) / (d.full_effect - d.no_effect), 0, 1);
    return d.max_torque * s * h * throttle * lock * k;
}

// 0x7e64a0 C: brakes pull spin toward 0 and stop there; skipped while ABS releases
pub fn brakes(car: Car, st: *Step) void {
    for (&st.wheels, car.wheels) |*w, p| {
        if (w.abs) continue;
        const toward: f32 = if (w.spin > 0) -1 else 1;
        w.turn(toward * w.brake, p.radius);
        if (w.spin * toward > 0) w.turn(-w.spin, p.radius); // crossed 0
    }
}
