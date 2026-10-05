// D3 step helpers on a Car
const std = @import("std");
const ground = @import("../shared/ground.zig");
const Car = @import("../car.zig").Car;

pub fn staticLoad(car: Car) f32 {
    return (1 - (car.front_weight orelse 0.5)) * (car.mass * 9.81 * 0.5); // rear wheel, used for all four
}

pub fn handbrakeTorque(car: Car) f32 {
    const front, const rear = .{ car.wheels[2].max_brake_torque, car.wheels[0].max_brake_torque };
    return rear / (front + rear) * car.handbrake_strength * (front + rear);
}

pub fn pressureZ(car: Car, p: f32) f32 {
    return p * car.wheels[2].rest_position[2] + (1 - p) * car.wheels[0].rest_position[2];
}

pub fn groundWheel(car: Car, i: usize) ground.Wheel {
    const p = car.wheels[i];
    return .{ .x = p.rest_position[0], .ray_top = p.max_height, .travel_min = p.min_height };
}

// piecewise linear over x = 0, 1, 2, ..; clamped at both ends
pub fn sample(table: []const f32, x: f32) f32 {
    const last: f32 = @floatFromInt(table.len - 1);
    const t = std.math.clamp(x, 0, last);
    const i: usize = @intFromFloat(@min(t, last - 1));
    return (table[i + 1] - table[i]) * (t - @as(f32, @floatFromInt(i))) + table[i]; // no fused multiply-add, like D3
}
