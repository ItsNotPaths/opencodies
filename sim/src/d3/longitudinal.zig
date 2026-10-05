// A one-dimensional model of the car along its road: engine, gears, brakes, drag and rolling, fitted to telemetry
const std = @import("std");
const Input = @import("../sim.zig").Input;
const Dataset = @import("../car.zig").Car;
const sample = @import("car.zig").sample;

pub const Car = struct {
    mass: f32,
    wheel_radius: f32,
    wheel_inertia: f32, // kg m^2, all four
    engine_inertia: f32, // kg m^2
    idle_rate: f32, // rad/s
    gear_ratios: [5]f32, // wheel/engine, final drive included
    power_step: f32, // rad/s
    power: [14]f32, // W
    eta: f32,
    drag: f32, // N/(m/s)^2
    rolling: f32, // N
    friction_torque: f32, // N m, engine braking
    friction_per_rate: f32, // N m s
    brake_torque: f32, // N m, all wheels
};

// the fitted part of the model (docs/fit.md)
pub const Fit = extern struct { idle_rate: f32, eta: f32, drag: f32, rolling: f32, friction_torque: f32, friction_per_rate: f32, brake_torque: f32 };

// the first 14 samples of a power curve; the curve holds its last value
pub fn of(c: Dataset, fit: Fit) Car {
    const curve = c.engine.curve.power;
    return .{
        .mass = c.mass,
        .wheel_radius = c.wheels[0].radius,
        .wheel_inertia = 2 * (c.wheels[2].inertia + c.wheels[0].inertia),
        .engine_inertia = c.engine.inertia,
        .idle_rate = fit.idle_rate,
        .gear_ratios = c.gearbox.ratios[1..6].*,
        .power_step = curve.step,
        .power = curve.table[0..14].*,
        .eta = fit.eta,
        .drag = fit.drag,
        .rolling = fit.rolling,
        .friction_torque = fit.friction_torque,
        .friction_per_rate = fit.friction_per_rate,
        .brake_torque = fit.brake_torque,
    };
}

pub fn step(car: Car, input: Input, speed: f32, dt: f32) f32 {
    return @max(0, speed + accel(car, input, speed) * dt); // forward only
}

pub fn accel(car: Car, input: Input, speed: f32) f32 {
    const gear_radius = gearRadius(car, input.gear);
    const engine = if (gear_radius) |gr| engineForce(car, input.throttle, speed, gr) else 0;
    const brake = input.brake * car.brake_torque / car.wheel_radius;
    const resistance = car.drag * speed * speed + car.rolling;
    return (engine - brake - resistance) / effectiveMass(car, gear_radius);
}

// m of travel per engine rad, null in neutral
fn gearRadius(car: Car, gear: i32) ?f32 {
    if (gear < 1 or gear > car.gear_ratios.len) return null;
    return car.gear_ratios[@intCast(gear - 1)] * car.wheel_radius;
}

fn engineForce(car: Car, throttle: f32, speed: f32, gear_radius: f32) f32 {
    const rate = @max(speed / gear_radius, car.idle_rate);
    const drive = throttle * car.eta * power(car, rate) / rate;
    const friction = (1 - throttle) * (car.friction_torque + car.friction_per_rate * rate);
    return (drive - friction) / gear_radius;
}

fn effectiveMass(car: Car, gear_radius: ?f32) f32 {
    const wheels = car.wheel_inertia / (car.wheel_radius * car.wheel_radius);
    const engine = if (gear_radius) |gr| car.engine_inertia / (gr * gr) else 0;
    return car.mass + wheels + engine;
}

fn power(car: Car, rate: f32) f32 {
    return sample(&car.power, rate / car.power_step);
}
