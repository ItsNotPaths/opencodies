const std = @import("std");
const body = @import("../shared/body.zig");
const vec = @import("../shared/vec.zig");
const d3math = @import("../shared/d3math.zig");
const car_ = @import("car.zig");
const force = @import("force.zig");
const Impulses = @import("suspension.zig").Impulses;
const Drag = @import("../material.zig").Drag;

const Car = @import("../car.zig").Car;
const Wheel = @import("../car.zig").Wheel;
const TyreYaw = @import("../car.zig").TyreYaw;
const WheelStep = force.WheelStep;
const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;
const clamp = std.math.clamp;

const deflection_max: f32 = 0.02; // m, 0xe798dc

// 0x7d2330: yaw inertia for the tyre impulses grows with speed; the high-speed terms never fire
pub fn inertia(car: Car, speed: f32, air_yaw: f32) V3 {
    const y = car.tyre_yaw orelse TyreYaw{ .mid_scale = 1, .speeds = .{ 0, 1 } }; // no growth
    const lo, const mid = y.speeds;
    const f = clamp((speed - lo) / (mid - lo), 0, 1);
    var i = car.inertia;
    i[1] = ((((y.mid_scale - 1) * f + 1) - 1) * f + 1) * i[1] * air_yaw; // x, z scales and the second curve: not in the dataset (1)
    return i;
}

// 0x7d9520 jump assist
pub fn airYaw(car: Car, multiple: f32, airborne: bool, speed: f32, dt: f32) f32 {
    const rise, const fall = car.jump_rates.?;
    if (airborne and speed >= 8) return @min(rise * dt + multiple, car.jump_cap.?);
    return @max(multiple - fall * dt, 1);
}

// 0x7ffea0 camber term: body roll leans the outer wheels into the ground
pub fn camberGrip(p: Wheel, axes: [3]V3, normal: V3, grounded: bool) f32 {
    if (!grounded) return 1;
    const circle = p.tyre.circle;
    const lean = if (p.rest_position[0] >= 0) -circle.camber else circle.camber;
    const sin_, const cos_ = d3math.sinCos(lean);
    const up = axes[0] * splat(-sin_) + axes[1] * splat(cos_); // body up rolled about forward
    const c = d3math.cos(circle.camber);
    const t = clamp((clamp(dot(up, normal), 0, 1) - c) / (1 - c), 0, 1);
    return 1 + (circle.camber_grip - 1) * t;
}

// 0x7f56d0 W+0x78: the drawn wheel rises out of the ground where the tyre sinks past its depth
pub fn visualLift(p: Wheel, travel: f32, ground: f32, bump: f32) f32 {
    const h = (bump + ground) - p.tyre_depth.?;
    return if (h > travel) clamp(h, p.min_height, p.max_height) - travel else 0;
}

// 0x928b30: how far the lowest tyre edge under the visual camber sits below a round tyre (W+0x16c,
// < 0 when lower), from the drawn travel; the next contact queries lift the ground by it
pub fn contactOffset(p: Wheel, travel: f32) f32 {
    const v = p.visual_camber orelse return 0; // a round tyre
    const lean = (p.tyre.circle.camber + v.angle) - v.gain[if (travel < 0) 0 else 1] * travel;
    const s, const c = d3math.sinCos(if (p.rest_position[0] > 0) -lean else lean);
    const axle = V3{ c, s, 0 }; // body frame; the game also steers it, which moves the last bit at most
    const down = V3{ 0, -1, 0 };
    const d = dot(axle, down);
    const under = vec.normalize(down - splat(d) * axle); // hub to the round contact
    const edge = splat(v.rim[if (d > 0) 1 else 0]) * axle + splat(p.radius) * under;
    return clamp(p.radius - dot(edge, down), -1, 1);
}

// 0x7d9980: the carcass hangs on a ground anchor that rolls with the wheel; past 2 cm the anchor slips
pub fn anchor(p: Wheel, w: *force.Wheel, s: body.State, dt: f32) void {
    const lon, const lat = directions(s.axes, w.steer);
    w.anchor = splat(w.mean_spin) * lon * splat(p.radius) * splat(dt) + w.anchor;
    const patch = patchPoint(p, w.travel, s);
    const d = w.anchor - patch;
    const free = [2]f32{ dot(lat, d), dot(lon, d) };
    const held = [2]f32{ clamp(free[0], -deflection_max, deflection_max), clamp(free[1], -deflection_max, deflection_max) };
    const slip = [2]f32{ held[0] - free[0], held[1] - free[1] };
    w.carcass = held;
    w.anchor_rate = if (dt > 0) .{ (1 / dt) * slip[0], (1 / dt) * slip[1] } else .{ 0, 0 };
    if (slip[0] * slip[0] + slip[1] * slip[1] > 0) w.anchor = splat(held[1]) * lon + splat(held[0]) * lat + patch;
}

// 0x7d5480: bottom of the wheel straight below the hub, travel along body y
pub fn patchPoint(p: Wheel, travel: f32, s: body.State) V3 {
    return patchArm(p, travel, s.axes) + s.pos;
}

pub fn patchArm(p: Wheel, travel: f32, axes: [3]V3) V3 {
    const y = (travel + p.rest_position[1]) - p.radius;
    return vec.toWorld(axes, .{ p.rest_position[0], y, p.rest_position[2] });
}

fn directions(axes: [3]V3, steer: f32) [2]V3 {
    const c = @sqrt(1 - steer * steer);
    return .{ axes[0] * splat(steer) + axes[2] * splat(c), axes[0] * splat(c) + axes[2] * splat(-steer) };
}

// 0x7e64a0 I-J: slip, friction circle, tyre impulses back through the coupling rows
pub fn substep(car: Car, ws: *[4]WheelStep, j: Impulses, handbrake: bool, h: f32, frac: f32) void {
    var lat_lon: [4][2]f32 = undefined;
    for (ws, car.wheels, j.tyre, &lat_lon) |*w, p, load, *f| f.* = impulse(car, p, w, load, handbrake and p.handbrake, h);
    for (lat_lon, 0..) |f, i| apply(car, ws, i, j.suspension[i], f[0], f[1], frac);
}

fn impulse(car: Car, p: Wheel, w: *WheelStep, normal: f32, handbrake: bool, h: f32) [2]f32 {
    w.carcass[0] = clamp(w.carcass[0] - w.lat_speed * h, -deflection_max, deflection_max);
    w.carcass[1] = clamp(w.carcass[1] - w.lon_slip * h, -deflection_max, deflection_max);
    const vref = @max(@abs(p.radius * w.spin), 1);
    const rated = car_.staticLoad(car) * h;
    const load = gripWeight(car, normal / rated) * rated;
    var sl = w.lon_slip / @abs(p.peak_slip_ratio.? * vref);
    var sa = w.lat_speed / (p.tan_peak_slip.? * w.softness * vref);
    if (vref < 10) { // carcass slip
        const k = clamp((10 - vref) * 0.5, 0, 1);
        sa += ((w.anchor_rate[0] * 0.5 + clamp(w.carcass[0] * -50 + w.lat_speed, -1, 1)) - sa) * k;
        sl += (-(clamp(w.carcass[1] * 50 - w.lon_slip, -1, 1) - w.anchor_rate[1] * 0.5) - sl) * k;
    }
    const perp = if (sa * sa <= 1) @sqrt(1 - sa * sa) else 0;
    w.abs = sl > @max(perp, car.abs_slip orelse std.math.inf(f32)) and w.spin > 0 and !handbrake;
    w.spin_sum += w.spin;
    const mag2 = sl * sl + sa * sa;
    if (mag2 > 1) { // friction circle; skidding_grip_drop 0
        const g = 1 / @sqrt(mag2);
        sa *= g;
        sl *= g;
    }
    return .{ -(sa * load * w.mu) - drag(w.drag, w.lat_speed) * load, -(sl * load * w.mu) };
}

// lss: a sideways drag that saturates at lss_speed; NaN counts as full
fn drag(d: Drag, lat_speed: f32) f32 {
    const r = lat_speed / d.speed;
    return d.coefficient * (if (r <= 1) @max(r, -1) else 1);
}

// §4.6, load factor of the normal impulse over the static one
pub fn gripWeight(car: Car, z: f32) f32 {
    const g = car.tyre_grip_by_load orelse [3]f32{ 0, 0, 0 }; // 1 at any load
    if (z <= 0) return 0;
    if (z < 2) return (z - 1) * g[0] + 1;
    if (z < 4) return (z - 2) * g[1] + (g[0] + 1);
    return ((z - 4) * g[2] + g[1] * 2) + (g[0] + 1);
}

fn apply(car: Car, ws: *[4]WheelStep, i: usize, suspension: f32, lat: f32, lon: f32, frac: f32) void {
    const src = &ws[i];
    src.tyre_lat.add(lat, frac);
    src.tyre_lon.add(lon, frac);
    const from_axis, const from_lat, const from_lon = src.response;
    for (ws, 0..) |*w, j| {
        const along = from_lon.axis[j] * lon + from_lat.axis[j] * lat;
        w.axis_speed += along;
        w.travel_rate -= along;
        w.lat_speed += from_axis.lat[j] * suspension + from_lat.lat[j] * lat + from_lon.lat[j] * lon;
        w.lon_slip += from_axis.lon[j] * suspension + from_lat.lon[j] * lat + from_lon.lon[j] * lon;
    }
    const p = car.wheels[i];
    src.turn(-1 / p.inertia * (p.radius * lon), p.radius);
}
