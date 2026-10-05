// Car to the step's Params: the curve tables, brake pressures, gear ratios and the speed-sensitive lock.
// FH2 SetWheelsFromInit 0x82e2e398, InitSpeedSensitiveSteeringCurve, InitGearBasedShiftPoints.
const std = @import("std");
const vec = @import("../shared/vec.zig");
const car_ = @import("../car.zig");
const tyre = @import("tyre.zig");
const suspension = @import("suspension.zig");
const drivetrain = @import("drivetrain.zig");
const steering = @import("steering.zig");
const gearbox = @import("gearbox.zig");
const fh = @import("car.zig");

const Car = car_.Car;
const g = suspension.g;
const rpm = std.math.pi / 30.0;

pub const Error = error{ NotThisModel, MissingField };

pub fn params(c: Car) Error!fh.Params {
    const curve = switch (c.engine.curve) {
        .sampled => |s| s,
        else => return Error.NotThisModel,
    };
    const diffs = switch (c.diffs) {
        .limited_slip => |d| d,
        else => return Error.NotThisModel,
    };
    const axles = switch (c.aero) {
        .axles => |a| a,
        else => return Error.NotThisModel,
    };
    var wheels: [4]fh.Wheel = undefined;
    for (&wheels, c.wheels, 0..) |*w, cw, i| {
        const t = switch (cw.tyre) {
            .curve_table => |t| t,
            else => return Error.NotThisModel,
        };
        const caps = cw.damper_caps orelse .{ std.math.inf(f32), std.math.inf(f32) };
        const stop = cw.bump_stop orelse car_.BumpStop{ .rate = 0, .damping = 0 };
        const per_g = c.mass * (g / 4.0) * cw.radius; // N m at brake pressure 1
        const brake = cw.max_brake_torque / per_g;
        w.* = .{
            .suspension = .{
                .hub = cw.rest_position,
                .radius = cw.radius,
                .spring = cw.spring.linear.rate,
                .free_travel = cw.spring.linear.preload,
                .bump = cw.slow_bump,
                .rebound = cw.slow_rebound,
                .bump_cap = caps[0],
                .rebound_cap = caps[1],
                .bump_stop = stop.rate,
                .bump_stop_damping = stop.damping,
                .max_stretch = cw.min_height,
                .max_compress = cw.max_height,
                .tyre_max_compress = cw.tyre_depth orelse return Error.MissingField,
                .unsprung = cw.unsprung_mass orelse return Error.MissingField,
            },
            .lateral = tyre.table(t.lateral),
            .longitudinal = tyre.table(t.longitudinal),
            .lateral_grip = t.lateral_grip,
            .longitudinal_grip = t.longitudinal_grip,
            .free_grip = t.free_grip,
            .inertia = cw.inertia,
            .brake = brake,
            .handbrake = if (cw.handbrake) brake * c.handbrake_strength else 0,
            .steered = i >= 2,
        };
    }
    const n = curve.table.len;
    const max_speed = @as(f32, @floatFromInt(n - 1)) * curve.step;
    var peak_power: f32 = 0;
    var max_power_speed: f32 = 0;
    for (curve.table, 0..) |t, i| {
        const w = @as(f32, @floatFromInt(i)) * curve.step;
        if (t * w > peak_power) {
            peak_power = t * w;
            max_power_speed = w;
        }
    }
    const redline = curve.redline;
    const clutch = c.gearbox.clutch_speeds orelse return Error.MissingField;
    const launch = c.launch orelse return Error.MissingField;
    var box: gearbox.Params = .{
        .gears = @splat(0),
        .top = @intCast(c.gearbox.top_gear),
        .shift_time = (c.gearbox.shift_time orelse return Error.MissingField)[0],
        .clutch_in = clutch[0],
        .clutch_out = clutch[1],
        .mode = if (c.gearbox.auto_gearbox orelse false) .automatic else .manual,
        .stall = curve.stall,
        .idle = c.engine.idle,
        .start = launch.optimum_rpm * rpm,
        .redline = redline,
    };
    for (&box.gears, c.gearbox.ratios) |*gr, r| gr.* = if (r == 0) 0 else 1 / (r * diffs.final_drive);
    gearbox.shiftPoints(&box);
    const line = c.driveline orelse car_.Driveline{ .gearbox_inertia = 0, .shaft_inertia = 0, .torque_scale = 1 };
    const front = c.wheels[2].drive_proportion + c.wheels[3].drive_proportion;
    const rear = c.wheels[0].drive_proportion + c.wheels[1].drive_proportion;
    var p: fh.Params = .{
        .mass = c.mass,
        .inertia = c.inertia,
        .wheels = wheels,
        .bars = .{ bar(c.wheels[0]), bar(c.wheels[2]) },
        .rear_slide = c.rear_slide_grip,
        .engine = .{
            .inertia = c.engine.inertia,
            .idle = c.engine.idle,
            .torque = curve.table,
            .step = curve.step,
            .redline = redline,
            .max_speed = max_speed,
            .max_power_speed = max_power_speed,
            .braking = curve.braking,
            .braking_low = curve.low_speed_braking,
        },
        .rev_limit = c.engine.rev_limit,
        .gearbox = box,
        .clutch_torque = c.gearbox.clutch_torque,
        .line = .{ .gearbox_inertia = line.gearbox_inertia, .shaft_inertia = line.shaft_inertia, .efficiency = line.torque_scale },
        .layout = if (front > 0 and rear > 0) .all else if (front > 0) .front else .rear,
        .diffs = .{ diffs.rear, diffs.front, diffs.centre },
        .final_drive = diffs.final_drive,
        .aero = axles,
        .steering = .{ .max = c.steer_lock orelse return Error.MissingField, .lock = undefined, .rates = c.speed_steering },
    };
    p.steering.lock = lockCurve(p, axles);
    return p;
}

fn bar(w: car_.Wheel) suspension.Bar {
    return .{ .stiffness = w.anti_roll, .damping = w.anti_roll_damping orelse 0 };
}

// the lock that holds max_gees of the peak lateral acceleration: atan(wheelbase a / v^2), as on a bicycle
fn lockCurve(p: fh.Params, a: car_.Axles) [steering.samples]f32 {
    var out: [steering.samples]f32 = undefined;
    const r = p.steering.rates orelse return @splat(p.steering.max);
    const zr = (p.wheels[0].suspension.hub[2] + p.wheels[1].suspension.hub[2]) * 0.5;
    const zf = (p.wheels[2].suspension.hub[2] + p.wheels[3].suspension.hub[2]) * 0.5;
    const front_share = -zr / (zf - zr);
    const wheelbase = zf - zr;
    for (&out, 0..) |*o, i| {
        const v = @max(@as(f32, @floatFromInt(i)) * (steering.curve_speed / (steering.samples - 1.0)), 0.01);
        const axle_load = [2]f32{
            p.mass * g * (1 - front_share) + @max(a.downforce[0], 0) * v * v,
            p.mass * g * front_share + @max(a.downforce[1], 0) * v * v,
        };
        var force: f32 = 0;
        for (0..2) |axle| {
            const w = p.wheels[axle * 2];
            const free = if (w.free_grip) |f| f.lateral else 1;
            force += peakAt(w.lateral, axle_load[axle] / 2) * axle_load[axle] * w.lateral_grip * free;
        }
        const lat = force / p.mass;
        o.* = std.math.clamp(std.math.atan(r.max_gees * wheelbase * lat / (v * v)), @min(r.min_lock, p.steering.max), p.steering.max);
    }
    return out;
}

// GetPeakFrictionAtLoad: the peak friction between the two curves, the load clamped to their loads
fn peakAt(t: tyre.Table, load: f32) f32 {
    const f = std.math.clamp((load - t.light) / (t.heavy - t.light), 0, 1);
    return (t.peak[1] - t.peak[0]) * f + t.peak[0];
}
