const std = @import("std");
const body = @import("../shared/body.zig");
const vec = @import("../shared/vec.zig");
const car_ = @import("car.zig");
const force = @import("force.zig");

const Car = @import("../car.zig").Car;
const Wheel = @import("../car.zig").Wheel;
const WheelStep = force.WheelStep;
const V3 = vec.V3;
const splat = vec.splat;

const hub_mass: f32 = 30; // 0xe7683c

pub const Geometry = struct { mount: V3, axis: V3, lat: V3, lon: V3 };

// 0x7e61f0; the mount is the bottom of the wheel at travel 0
pub fn geometry(s: body.State, p: Wheel, steer: f32) Geometry {
    const c = @sqrt(1 - steer * steer);
    const a = s.axes;
    const local = p.rest_position - V3{ 0, p.radius, 0 };
    return .{
        .mount = (s.pos + a[1] * splat(local[1])) + (a[2] * splat(local[2]) + a[0] * splat(local[0])),
        .axis = vec.toWorld(a, p.spring_axis),
        .lat = vec.toWorld(a, .{ c, 0, -steer }),
        .lon = vec.toWorld(a, .{ steer, 0, c }),
    };
}

pub fn contactPoint(g: Geometry, travel: f32) V3 {
    return g.mount + g.axis * splat(travel);
}

// 0x7fc7d0: fast/slow rebound past the zone, softened by edamp while the body lifts off a drooping wheel
pub fn reboundRatio(car: Car, p: Wheel, grounded: bool, axis_speed: f32, travel_rate: f32) f32 {
    const f = if (grounded and axis_speed > 0.25 and travel_rate <= -p.rebound_zone.?) 1 - (car.rebound_relief orelse 0) else 1;
    return ((1 - f) * p.slow_rebound + p.fast_rebound.? * f) / p.slow_rebound;
}

pub const Impulses = struct { suspension: [4]f32, tyre: [4]f32 };

// 0x7e64a0 D-H: external acceleration, spring, damper, anti-roll, travel limits, tyre contact
pub fn substep(car: Car, ws: *[4]WheelStep, h: f32, frac: f32) Impulses {
    const axle_mean = [2]f32{ (ws[0].travel + ws[1].travel) * 0.5, (ws[2].travel + ws[3].travel) * 0.5 };
    var out: Impulses = undefined;
    for (ws, car.wheels, 0..) |*w, p, i| {
        w.axis_speed += w.axis_accel * h;
        w.travel_rate = w.travel_rate - w.axis_accel * h - 9.81 * w.axis[1] * h;
        out.suspension[i] = strutForce(p, w.*, axle_mean[i / 2]) * h;
    }
    for (ws) |*w| w.travel += h * w.travel_rate;
    for (out.suspension, 0..) |j, i| {
        ws[i].suspension.add(j, frac);
        push(ws, i, j);
        ws[i].travel_rate -= j / hub_mass;
    }
    for (0..ws.len) |i| out.tyre[i] = tyreContact(car, ws, i, h, frac);
    return out;
}

pub fn strutForce(p: Wheel, w: WheelStep, axle_mean: f32) f32 {
    const v = w.travel_rate;
    const post_zone = p.fast_bump.? / p.slow_bump;
    const damper = if (v <= 0)
        p.slow_rebound * (if (p.rebound_zone.? < -v) w.rebound_ratio - (p.rebound_zone.? / v) * (1 - w.rebound_ratio) else 1)
    else
        p.slow_bump * (if (p.bump_zone.? < v) (p.bump_zone.? / v) * (1 - post_zone) + post_zone else 1);
    const s = p.spring.linear;
    const spring = @max(0, s.rate * (w.travel - s.preload));
    return (w.travel - axle_mean) * p.anti_roll + (damper * v + spring);
}

// impulse j along the axis of wheel i, seen by the body at every wheel
fn push(ws: *[4]WheelStep, i: usize, j: f32) void {
    for (ws, ws[i].rows(.axis).axis) |*w, g| {
        w.axis_speed += j * g;
        w.travel_rate -= j * g;
    }
}

fn tyreContact(car: Car, ws: *[4]WheelStep, i: usize, h: f32, frac: f32) f32 {
    const p = car.wheels[i];
    const w = &ws[i];
    w.ground = (w.ground - w.axis_speed * h) + w.ground_rate * h;
    limitTravel(ws, i, p.bump_stop_height, 1, frac);
    limitTravel(ws, i, p.min_height, -1, frac);
    const pen = w.ground - w.travel;
    if (pen < 0) return 0;
    w.touched = true;
    const inv_mass = 1 / hub_mass;
    const cap = car.mass * 9.81 * h * 7.5; // 0xefe53c
    const closing = (w.ground_rate - w.axis_speed) - w.travel_rate;
    var j = std.math.clamp((p.tyre_damping.? * closing + p.tyre_spring.? * pen) * h, 0, cap);
    w.travel_rate += inv_mass * j;
    if (p.tyre_depth.? < pen) {
        w.travel = w.ground - p.tyre_depth.?;
        var rigid = if (closing > 0) closing / (inv_mass + w.bottoming[i]) else 0;
        if (cap < j + rigid) rigid = cap - j;
        if (rigid > 0) {
            w.travel_rate += rigid * inv_mass;
            for (ws, w.bottoming) |*o, g| o.ground_rate -= g * rigid;
            j += rigid;
            w.tyre_normal.add(rigid, frac); // counted twice in the full sum, like D3
        }
    }
    w.tyre_normal.full += j;
    return j;
}

// bump stop: clamp the travel and stop the hub with a rigid impulse
fn limitTravel(ws: *[4]WheelStep, i: usize, limit: f32, side: f32, frac: f32) void {
    const w = &ws[i];
    if ((w.travel - limit) * side <= 0) return;
    w.travel = limit;
    if (w.travel_rate * side <= 0) return;
    const j = w.travel_rate / (1 / hub_mass + w.rows(.axis).axis[i]);
    w.suspension.add(j, frac);
    push(ws, i, j);
    w.travel_rate = 0;
}
