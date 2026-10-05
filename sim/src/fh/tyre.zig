// Tyre friction: curve tables against normalized slip at two loads, combined by the length of the slip vector.
// FH2 CTireFrictionCurves::CalcPhysicsFromInit 0x82e0d1a8, CCarDynamics::CalcCombinedLongitudinalLateralFriction 0x82e2a168.
const std = @import("std");

pub const samples = 100;

// one direction (lateral or longitudinal) as the car data holds it
pub const Curves = extern struct {
    max_slip: f32, // slip of the last sample: rad (lateral) or slip ratio (longitudinal)
    loads: [2]f32, // N, the load of each curve
    load_cap: f32, // N, the load stops counting here (or at the peak of load x friction, if lower)
    scale: [2]f32, // friction coefficient of each curve
    shape: [2][samples]f32, // friction over scale, sample i at slip i * max_slip / (samples - 1)
};

// the curves ready for lookup
pub const Table = extern struct {
    light: f32,
    heavy: f32,
    load_clamp: f32,
    load_step: f32,
    slip_step: f32,
    slip_end: f32,
    peak_slip: [2]f32, // slip of the highest sample
    peak: [2]f32,
    friction: [2][samples]f32,
    arcade: [2][samples]f32, // never falls by more than 1e-5 per sample
};

pub fn table(c: Curves) Table {
    var t: Table = undefined;
    t.light = c.loads[0];
    t.heavy = c.loads[1];
    t.load_step = t.heavy - t.light;
    t.slip_end = c.max_slip;
    t.slip_step = c.max_slip / (samples - 1);
    for (0..2) |k| {
        var best: usize = 0;
        var peak: f32 = 0;
        for (0..samples) |i| {
            const f = c.shape[k][i] * c.scale[k];
            t.friction[k][i] = f;
            if (f > peak) {
                peak = f;
                best = i;
            }
        }
        t.arcade[k][0] = t.friction[k][0];
        for (1..samples) |i| t.arcade[k][i] = @max(t.friction[k][i], t.arcade[k][i - 1] - 1e-5);
        t.peak[k] = peak;
        t.peak_slip[k] = @as(f32, @floatFromInt(best)) * t.slip_step;
    }
    t.load_clamp = c.load_cap;
    // with friction linear in load, load x friction peaks at -a / 2b
    const s0 = c.scale[0];
    const s1 = c.scale[1];
    if (t.light < t.heavy and s1 < s0) {
        const d = t.heavy - t.light;
        const at_peak = (-s0 - (s0 * t.light - s1 * t.light) / d) * (d / ((s1 - s0) * 2));
        t.load_clamp = @min(at_peak, c.load_cap);
    }
    return t;
}

// the clamped load and the share of the heavy curve
const Blend = struct { f: f32, load: f32 };

fn blend(t: Table, load: f32) Blend {
    const l = std.math.clamp(load, @min(t.light, t.load_clamp), @max(t.light, t.load_clamp));
    return .{ .f = (l - t.light) / t.load_step, .load = l };
}

fn lerp(a: f32, b: f32, f: f32) f32 {
    return (b - a) * f + a;
}

// friction at slip x for curve k, sim and arcade samples blended by arcade
fn sample(t: Table, k: usize, x: f32, arcade: f32) f32 {
    const s = std.math.clamp(x, 0, t.slip_end);
    const j = @min(@as(usize, @intFromFloat(s / t.slip_step)), samples - 2);
    const f = (s - @as(f32, @floatFromInt(j)) * t.slip_step) / t.slip_step;
    const sim = lerp(t.friction[k][j], t.friction[k][j + 1], f);
    const arc = lerp(t.arcade[k][j], t.arcade[k][j + 1], f);
    return lerp(sim, arc, arcade);
}

fn coefficient(t: Table, b: Blend, x: f32, arcade: f32) f32 {
    return lerp(sample(t, 0, x, arcade), sample(t, 1, x, arcade), b.f);
}

fn atLoad(v: [2]f32, b: Blend) f32 {
    return lerp(v[0], v[1], b.f);
}

// the surface under the tyre
pub const Surface = extern struct {
    friction: f32 = 1, // not read here: the caller puts it in Grip
    off_road: f32 = 0, // 0..1, blends the peak slip angle toward peak_slip_angle
    peak_slip_angle: f32 = 0.20944, // rad
    rear_grip: f32 = 1,
    min_arcade: f32 = 0,
    handbrake_grip: f32 = 1,
};

pub const Grip = extern struct {
    long: f32 = 1, // all friction scales of the wheel times the surface friction
    lat: f32 = 1,
    arcade: f32 = 0, // 0 sim curves .. 1 arcade curves
    lateral_arcade: f32 = 0,
};

// rear lateral grip falls with the slip angle unless the front steers against it
pub const RearSlide = extern struct {
    min_scale: f32,
    slip: [2]f32, // normalized slip angle: min_scale below the first, 1 past the second
    steer: [2]f32, // rad: the scale returns to 1 between these
};

pub const Out = extern struct {
    long: f32, // N along the forward tangent, against the slip
    lat: f32,
    slip_ratio: f32,
    slip_angle: f32, // rad; this and the two below have the sign of the force, against the slip
    norm_slip_ratio: f32,
    norm_slip_angle: f32,
    combined: f32, // length of the normalized slip
    long_coefficient: f32,
    lat_coefficient: f32,
    long_max: f32, // N at the peak
    lat_max: f32,
};

pub const Wheel = struct {
    rear: bool,
    handbrake: bool,
    rear_slide: ?RearSlide = null,
};

// v_long, v_lat: contact velocity on the ground; v_slip: v_long minus the tread speed
pub fn friction(lat: Table, long: Table, w: Wheel, s: Surface, g: Grip, load: f32, v_long: f32, v_lat: f32, v_slip: f32, steer: f32, handbrake: f32, speed: f32) Out {
    const sign = struct {
        fn of(x: f32) f32 {
            return if (x < 0) 1 else -1;
        }
    }.of;
    const al = @abs(v_long);
    const at = @abs(v_lat);
    const as = @abs(v_slip);
    const bl = blend(long, load);
    const bt = blend(lat, load);
    const peak_sr = atLoad(long.peak_slip, bl);
    const peak_sa = atLoad(lat.peak_slip, bt);
    const surface_peak = (s.peak_slip_angle / peak_sa - 1) * s.off_road + 1;
    const peak_sa_here = surface_peak * peak_sa;
    const slip_ratio = as / @max(al, 6);
    const nsr = slip_ratio / (surface_peak * peak_sr);

    // below 0.7 m/s the slip angle turns linear in the side speed, blended in up to 1 m/s
    const v2 = al * al + at * at;
    const linear = at / 0.7;
    const nsa = if (v2 > 1)
        std.math.atan2(at, al) / peak_sa_here
    else if (v2 < 0.49)
        linear
    else
        (std.math.atan2(at, al) / peak_sa_here - linear) * (@sqrt(v2) - 0.7) / 0.3 + linear;

    const c = @max(@sqrt(nsa * nsa + nsr * nsr), 1e-4);
    const mu_long = coefficient(long, bl, c * peak_sr, @max(g.arcade, s.min_arcade));
    const mu_lat = coefficient(lat, bt, c * peak_sa, @max(g.lateral_arcade, s.min_arcade));
    var f_long = mu_long * (nsr / c) * bl.load * sign(v_slip) * g.long;
    var f_lat = mu_lat * (nsa / c) * bt.load * sign(v_lat) * g.lat;

    if (w.rear) {
        if (w.rear_slide) |r| f_lat *= rearSlide(r, nsa, @abs(steer));
        f_long *= s.rear_grip;
        f_lat *= s.rear_grip;
    }
    if (w.handbrake) {
        const ramp = std.math.clamp((speed - 10) * 0.1, 0, 1);
        const k = 1 - ramp * (1 - s.handbrake_grip) * handbrake;
        f_long *= k;
        f_lat *= k;
    }
    return .{
        .long = f_long,
        .lat = f_lat,
        .slip_ratio = slip_ratio,
        .slip_angle = nsa * peak_sa_here * sign(v_lat),
        .norm_slip_ratio = nsr * sign(v_slip),
        .norm_slip_angle = nsa * sign(v_lat),
        .combined = c,
        .long_coefficient = mu_long,
        .lat_coefficient = mu_lat,
        .long_max = atLoad(long.peak, bl) * bl.load * g.long,
        .lat_max = atLoad(lat.peak, bt) * bt.load * g.lat,
    };
}

fn rearSlide(r: RearSlide, nsa: f32, steer: f32) f32 {
    const slip_lo = @min(r.slip[0], r.slip[1]);
    const slip_hi = @max(r.slip[0], r.slip[1]);
    const steer_lo = @min(r.steer[0], r.steer[1]);
    const steer_hi = @max(r.steer[0], r.steer[1]);
    const a = @abs(nsa);
    const by_slip = if (a <= slip_lo) r.min_scale else if (a < slip_hi) lerp(r.min_scale, 1, (a - slip_lo) / (slip_hi - slip_lo)) else 1;
    const by_steer: f32 = if (steer <= steer_lo) 0 else if (steer < steer_hi) (steer - steer_lo) / (steer_hi - steer_lo) else 1;
    return lerp(by_slip, 1, by_steer);
}
