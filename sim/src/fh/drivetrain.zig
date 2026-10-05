// Engine torque, clutch, brakes and the driven-wheel solve. FH2 CCarEngine 0x82e1d9a8..0x82e1dd40,
// ClutchCalcTorque 0x82e2b488, WheelCalcBrakeTorque 0x82e32a78, CalcDrivetrainDerivativeData2WD 0x82e34550.
const std = @import("std");

pub const Engine = struct {
    inertia: f32,
    idle: f32, // rad/s
    torque: []const f32, // N m at full throttle, sample i at i * step rad/s
    step: f32,
    redline: f32, // rad/s
    max_speed: f32,
    max_power_speed: f32,
    braking: f32, // N m: the zero-throttle torque is this times a shape of the speed
    braking_low: f32, // times the fixed low-speed shape
};

pub fn fullThrottle(e: Engine, w: f32) f32 {
    const n = e.torque.len;
    const x = std.math.clamp(w, 0, @as(f32, @floatFromInt(n - 1)) * e.step) / e.step;
    const i = @min(@as(usize, @intFromFloat(x)), n - 2);
    const f = x - @as(f32, @floatFromInt(i));
    return (e.torque[i + 1] - e.torque[i]) * f + e.torque[i];
}

const low_shape = [9]f32{ 0.1, 0.09, 0.08, 0.07, 0.06, 0.05, 0.0385, 0.0192, 0 };
const braking_start = 83.77576; // rad/s, 800 rpm

// engine braking, < 0: a fixed shape over the first 8 samples, then 0 at 800 rpm to -1 at peak power,
// -1 to the redline, -1.63 more by max_speed
pub fn zeroThrottle(e: Engine, w: f32) f32 {
    const wc = std.math.clamp(w, 0, e.max_speed);
    const x = std.math.clamp(wc / e.step, 0, @as(f32, @floatFromInt(e.torque.len - 1)));
    if (x < 8) {
        const i: usize = @intFromFloat(x);
        const f = x - @floor(x);
        return ((low_shape[i + 1] - low_shape[i]) * f + low_shape[i]) * e.braking_low * e.braking;
    }
    const shape: f32 = if (wc < e.max_power_speed and wc < e.redline)
        -(wc - braking_start) / (e.max_power_speed - braking_start)
    else if (wc < e.redline)
        -1
    else if (e.redline < e.max_speed)
        (wc - e.redline) / (e.max_speed - e.redline) * -1.63 - 1
    else
        -2.63;
    return shape * e.braking;
}

pub fn engineTorque(e: Engine, w: f32, throttle: f32) f32 {
    const t0 = zeroThrottle(e, w);
    return std.math.clamp(throttle, 0, 1) * (fullThrottle(e, w) - t0) + t0;
}

// the clutch passes at most max(|engine torque| + 4, capacity) times the pedal; its sign follows the
// slip, linear within +-10 pi rad/s
pub fn clutch(engine_torque: f32, capacity: f32, pedal: f32, slip: f32) f32 {
    const reach = 31.415909;
    const sign = std.math.clamp(slip / reach, -1, 1);
    return sign * pedal * @max(@abs(engine_torque) + 4, capacity);
}

// N m against the spin, full past +-pi rad/s; pressure 1 stops the car at 1 g on four wheels
pub fn brake(mass: f32, radius: f32, pressure: f32, spin: f32) f32 {
    return mass * (9.80665 / 4.0) * radius * pressure * std.math.clamp(-spin / std.math.pi, -1, 1);
}

pub const Diff = struct {
    ratio: f32, // shaft/wheel
    accel_lock: f32, // N m of locking torque at full_lock_torque of drive
    decel_lock: f32, // N m at zero or negative drive
    full_lock_torque: f32, // N m of drive
    full_lock_slip: f32, // rad/s of spin difference for the full locking torque
};

// locking torque on the first wheel of the pair, from the spin difference first - second
pub fn lsd(d: Diff, drive: f32, spin_diff: f32) f32 {
    const lo = @min(0, d.full_lock_torque);
    const hi = @max(0, d.full_lock_torque);
    const k = if (drive <= lo) d.decel_lock else if (drive < hi) (drive - lo) / (hi - lo) * (d.accel_lock - d.decel_lock) + d.decel_lock else d.accel_lock;
    return std.math.clamp(spin_diff / @abs(d.full_lock_slip), -1, 1) * k;
}

// the parts between the clutch and the wheels
pub const Line = struct {
    gearbox_inertia: f32, // kg m^2 at the gearbox output
    shaft_inertia: f32,
    efficiency: f32, // times the torque through the gearbox
};

pub const Drive = struct {
    ratio: f32, // engine/shaft of the gear
    locked: bool, // the clutch holds: the engine turns with the shaft
    engine: f32, // N m
    clutch: f32, // N m through the clutch when it slips
    engine_inertia: f32,
};

pub const Result = struct { spin_acc: [4]f32, engine_acc: f32 };

// wheels 0, 1 driven through diff, 2, 3 free; torque: brake and tyre torque on each wheel
pub fn twoWheel(diff: Diff, line: Line, d: Drive, torque: [4]f32, spin: [4]f32, inertia: [4]f32) Result {
    const fd = diff.ratio;
    const engine_side = if (d.locked) d.ratio * d.ratio * d.engine_inertia else 0;
    const i_drive = (engine_side + line.gearbox_inertia + line.shaft_inertia) * fd * fd;
    const t_drive = d.ratio * (if (d.locked) d.engine else d.clutch) * line.efficiency * fd;
    const lock = lsd(diff, t_drive, spin[0] - spin[1]);
    const t: [2]f32 = .{ torque[0] + t_drive * 0.5 - lock, torque[1] + t_drive * 0.5 + lock };
    const det = (inertia[1] + inertia[0]) * i_drive + inertia[0] * inertia[1] * 4;
    const a0 = (t[0] * inertia[1] * 4 + (t[0] * i_drive - t[1] * i_drive)) / det;
    const a1 = (t[1] * inertia[0] * 4 + (t[1] * i_drive - t[0] * i_drive)) / det;
    return .{
        .spin_acc = .{ a0, a1, torque[2] / inertia[2], torque[3] / inertia[3] },
        .engine_acc = if (d.locked) (a0 + a1) * fd * 0.5 * d.ratio else (d.engine - d.clutch) / d.engine_inertia,
    };
}

// wheels 0, 1 front, 2, 3 rear; final_drive turns shaft torque into the diffs' locking input
pub fn allWheel(front: Diff, rear: Diff, centre: Diff, final_drive: f32, line: Line, d: Drive, torque: [4]f32, spin: [4]f32, inertia: [4]f32, radius: [4]f32) Result {
    const fr = front.ratio;
    const rr = rear.ratio;
    const gearbox = line.gearbox_inertia + if (d.locked) d.ratio * d.ratio * d.engine_inertia else 0;
    const t_shaft = line.efficiency * d.ratio * (if (d.locked) d.engine else d.clutch);
    const t_lock = final_drive * t_shaft;
    const lock_f = lsd(front, t_lock, spin[0] - spin[1]);
    const lock_r = lsd(rear, t_lock, spin[2] - spin[3]);
    const lock_c = lsd(centre, t_lock, (spin[0] + spin[1]) * 0.5 / radius[2] * radius[0] - (spin[2] + spin[3]) * 0.5);
    const to_front = t_shaft * fr * 0.25 - lock_c;
    const to_rear = t_shaft * rr * 0.25 + lock_c;
    var t: [4]f32 = .{ to_front + torque[0] - lock_f, to_front + torque[1] + lock_f, to_rear + torque[2] - lock_r, to_rear + torque[3] + lock_r };
    const shared = line.shaft_inertia * 0.25 + gearbox * 0.0625;
    const ff = shared * fr * fr;
    const rr2 = shared * rr * rr;
    const fx = gearbox * fr * rr * 0.0625;
    var m: [4][4]f32 = .{
        .{ inertia[0] + ff, ff, fx, fx },
        .{ ff, inertia[1] + ff, fx, fx },
        .{ fx, fx, inertia[2] + rr2, rr2 },
        .{ fx, fx, rr2, inertia[3] + rr2 },
    };
    solve4(&m, &t);
    return .{
        .spin_acc = t,
        .engine_acc = if (d.locked) ((t[2] + t[3]) * rr * 0.5 + (t[0] + t[1]) * fr * 0.5) * 0.5 * d.ratio else (d.engine - d.clutch) / d.engine_inertia,
    };
}

// Gaussian elimination with partial pivoting; b becomes the solution
fn solve4(m: *[4][4]f32, b: *[4]f32) void {
    for (0..4) |c| {
        var best = c;
        for (c + 1..4) |r| if (@abs(m[r][c]) > @abs(m[best][c])) {
            best = r;
        };
        std.mem.swap([4]f32, &m[c], &m[best]);
        std.mem.swap(f32, &b[c], &b[best]);
        for (c + 1..4) |r| {
            const f = m[r][c] / m[c][c];
            for (c..4) |k| m[r][k] -= f * m[c][k];
            b[r] -= f * b[c];
        }
    }
    var i: usize = 4;
    while (i > 0) {
        i -= 1;
        var x = b[i];
        for (i + 1..4) |k| x -= m[i][k] * b[k];
        b[i] = x / m[i][i];
    }
}
