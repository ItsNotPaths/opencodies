const std = @import("std");
const car_ = @import("car.zig");
const d3math = @import("../shared/d3math.zig");
const Wheel = @import("force.zig").Wheel;

const Car = @import("../car.zig").Car;
const clamp = std.math.clamp;

// 0x7e55c0 in pad mode: Ackermann sines for the front wheels, rate limited; rears stay straight
pub fn update(car: Car, wheels: *[4]Wheel, steer: f32, speed: f32, dt: f32) void {
    const target = ackermann(car, centreAngle(car, clamp(steer, -1, 1), speed));
    const k = clamp((speed - car.steer_rate_speeds.?[0]) / (car.steer_rate_speeds.?[1] - car.steer_rate_speeds.?[0]), 0, 1);
    const step = ((car.steer_rate.?[1] - car.steer_rate.?[0]) * k + car.steer_rate.?[0]) * dt;
    for (wheels[2..], target) |*w, t| w.steer = clamp(t, w.steer - step, w.steer + step);
}

fn centreAngle(car: Car, steer: f32, speed: f32) f32 {
    const shaped = d3math.pow(@abs(steer), 1 + car.pad_linearity.?);
    return lock(car, speed) * std.math.copysign(shaped, steer);
}

fn lock(car: Car, speed: f32) f32 {
    return car_.sample(&car.pad_lock.?, speed / car.pad_lock_step.?);
}

// sines of the left and right front wheel
fn ackermann(car: Car, angle: f32) [2]f32 {
    if (@abs(angle) <= 1e-7) return .{ 0, 0 };
    const radius = -(d3math.cos(angle) / d3math.sin(angle) * car.wheelbase); // turn centre to the car centre line
    const half = car.track * 0.5;
    return .{ wheelSin(car, radius + half), wheelSin(car, radius - half) };
}

fn wheelSin(car: Car, radius: f32) f32 {
    const x = radius * (1 / car.wheelbase);
    return -1 / std.math.copysign(@sqrt(x * x + 1), x);
}
