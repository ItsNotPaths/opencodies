const std = @import("std");
const body = @import("../shared/body.zig");
const vec = @import("../shared/vec.zig");
const Car = @import("../car.zig").Car;
const force = @import("force.zig");
const Slowdown = @import("../material.zig").Slowdown;

const V3 = vec.V3;
const splat = vec.splat;

// 0x7ebe30, after the body update: soft surfaces brake each touching wheel, mostly by the mean of all four
pub fn apply(car: Car, s: *force.State, surfaces: [4]Slowdown, arms: [4]V3, speed: f32, dt: f32) void {
    var rates: [4]f32 = undefined;
    var sum: f32 = 0;
    for (&rates, surfaces, s.wheels) |*r, sd, w| {
        r.* = if (w.contact) sd.amount * std.math.clamp((speed - sd.from_speed) / (sd.full_speed - sd.from_speed), 0, 1) else 0;
        sum += r.*;
    }
    const shared = sum / 4 * 0.95;
    for (rates, arms, &s.wheels, car.wheels) |rate, arm, *w, p| {
        const r = rate * 0x1.9999ap-5 + shared; // 0xefe76c, not quite 0.05
        if (!(r > 0)) continue;
        const u = vec.cross(s.body.angvel, arm) + s.body.vel;
        const j = vec.normalize(u) * splat(-(car.mass * r * dt));
        if (w.mean_spin > 0) w.mean_spin = @max(w.mean_spin - r * dt / p.radius, 0);
        body.applyImpulse(&s.body, car.rigid(), j, arm);
    }
}
