const std = @import("std");
const math = @import("math.zig");

// SGraphData: one force curve of a tyre compound, x = slip, y = force over the static wheel load;
// _1 at load ratio 1, _4 at load ratio 4
pub const Graph = extern struct {
    peak_x1: f32,
    peak_x4: f32,
    peak_y1: f32,
    peak_y4: f32, // 0: the peak scales with the caller's load exponent
    end_y1: f32, // sliding level
    shape_1: f32,
    shape_4: f32,
    preshape_1: f32, // > 0: a linear-then-quadratic rise below the peak
    preshape_4: f32,
};

// RuntimeLoadGraph: the curve at one load, F(x) = (x b + a x^p) / (c x^p + 1) past the peak
pub const LoadGraph = extern struct {
    a: f32,
    b: f32,
    c: f32,
    p: f32,
    peak_x: f32,
    peak_y: f32,
    preshape: f32,
    gradient: f32,
};

// GraphSet: the curves of the tyre on the active surface (Tyre+0x18 is the current set)
pub const GraphSet = extern struct {
    longitudinal: Graph,
    lateral: Graph,
    aligning: Graph,
    camber: f32, // camber thrust coefficient
    grip_vs_weight: f32, // load exponent
};

// SSurfaceData: a compound's graphs on one surface (RuntimeTyre+0x28, 0x7c each)
pub const SubTyre = extern struct {
    surface: u32, // m_tyreCode
    lateral: Graph,
    longitudinal: Graph,
    aligning: Graph,
    grip_vs_weight: f32,
    camber: f32,
    rolling_resistance: f32,

    fn set(s: SubTyre) GraphSet {
        return .{ .longitudinal = s.longitudinal, .lateral = s.lateral, .aligning = s.aligning, .camber = s.camber, .grip_vs_weight = s.grip_vs_weight };
    }
};

const default_graph: Graph = .{ .peak_x1 = 0.2, .peak_x4 = 0.2, .peak_y1 = 0, .peak_y4 = 0, .end_y1 = 0, .shape_1 = 2, .shape_4 = 2, .preshape_1 = 0, .preshape_4 = 0 };
const default_set: GraphSet = .{ .longitudinal = default_graph, .lateral = default_graph, .aligning = default_graph, .camber = 0, .grip_vs_weight = 0.7 };

// 0xf58be0 Tyre::setActiveSurface: the compound's graphs for the surface become the target, blended toward at
// |target - current| per unit rate; from no surface they apply at once. Surface 0 resets all three sets, an
// unknown one changes nothing. target, rate: Tyre+0x8c, +0x100
pub fn setActiveSurface(t: *Tyre, target: *GraphSet, rate: *GraphSet, compound: []const SubTyre, surface: u32) void {
    if (t.surface == surface) return;
    if (surface == 0) {
        t.set = default_set;
        target.* = default_set;
        rate.* = default_set;
        t.surface = 0;
        t.has_record = false;
        t.rolling_resistance = 0;
        return;
    }
    const s = for (compound) |s| {
        if (s.surface == surface) break s;
    } else return;
    target.* = s.set();
    if (t.surface == 0) {
        t.set = target.*;
        rate.* = default_set;
    } else {
        const r: *[29]f32 = @ptrCast(rate);
        for (r, @as(*const [29]f32, @ptrCast(target)), @as(*const [29]f32, @ptrCast(&t.set))) |*x, a, b| x.* = @abs(a - b);
    }
    t.surface = surface;
    t.has_record = true;
    t.rolling_resistance = s.rolling_resistance;
}

// the Tyre outputs of calculateForces_v2 (Tyre+0x18c..)
pub const Forces = extern struct {
    longitudinal: f32 = 0,
    lateral: f32 = 0, // camber thrust included
    camber: f32 = 0,
    mixed_lateral_scale: f32 = 1, // lateral force over the pure lateral curve
    slip_ratio_used: f32 = 0, // slip ratio over the pure longitudinal curve: how far into it the tyre works
    slip_angle_used: f32 = 0,
};

const ln4: f32 = 0x1.62e430p0; // 0x27361f8, f32(ln 4)
const half_pi: f32 = 0x1.921fb6p0;
const pi: f32 = 0x1.921fb6p1;

// 0xf56f90 Tyre::createLoadGraph: the curve at load ratio z, peak and sliding level scaled by z^exponent
pub fn loadGraph(g: Graph, exponent: f32, z: f32) LoadGraph {
    if (!(@as(f64, z) > 0.0001) or !(g.peak_y1 > 0)) {
        return .{ .a = 0, .b = 0, .c = 0, .p = 1, .peak_x = g.peak_x1, .peak_y = 0, .preshape = 0, .gradient = 0 };
    }
    const f = blend(z);
    const shape = (g.shape_4 - g.shape_1) * f + g.shape_1;
    const pre = (g.preshape_4 - g.preshape_1) * f + g.preshape_1;
    const peak_x = f * (g.peak_x4 - g.peak_x1) + g.peak_x1;
    const zc = if (z > 4) 4 else if (0 < z) z else 0;
    const sliding = g.end_y1 * math.pow(zc, exponent);
    const peak_exponent = if (g.peak_y4 != 0) math.log(g.peak_y4 / g.peak_y1) / ln4 else exponent;
    const peak_y = g.peak_y1 * math.pow(zc, peak_exponent);
    const xp = math.pow(peak_x, shape);

    // a, b, c: F(peak_x) = peak_y, F'(peak_x) = 0, F(inf) = sliding
    const b = (((1 - shape * xp / peak_x) / ((shape - 1) * xp)) * xp + 1) * peak_y / (peak_x - xp);
    const rest = peak_y - peak_x * b;
    const det = 1 / (-peak_y * xp - xp * -sliding);
    const a = ((-peak_y * xp) * (sliding - b) - -sliding * rest) * det;
    const c = det * (rest - (sliding - b) * xp);
    return .{
        .a = a,
        .b = b,
        .c = c,
        .p = shape,
        .peak_x = peak_x,
        .peak_y = peak_y,
        .preshape = pre,
        .gradient = if (pre > 0) (peak_y + peak_y) / peak_x / (1 + pre) else b,
    };
}

// (z - 1)/3 clamped to 0..1: 0 at load ratio 1, 1 at 4
fn blend(z: f32) f32 {
    const f = (z - 1) / 3;
    if (f > 1) return 1;
    return if (f > 0) f else 0; // maxss: NaN gives 0
}

// 0xf57330 Tyre::sampleGraph: force at slip x
pub fn sample(g: LoadGraph, x: f32) f32 {
    if (g.preshape > 0 and g.peak_x > x) {
        const xn = x / g.peak_x;
        const half = (1 - g.preshape) * 0.5;
        const k = g.peak_y / (g.preshape + half);
        const q = (xn - g.preshape) / (half + half);
        const linear = xn * k;
        return if (q > 0) linear - q * q * (k - g.peak_y) else linear;
    }
    const xp = math.pow(x, g.p);
    return (x * g.b + g.a * xp) / (xp * g.c + 1);
}

// one tyre (W+0x1c0) as the force code sees it
pub const Tyre = extern struct {
    set: GraphSet, // +0x18 m_currentSet
    surface: u32, // +0x8 m_activeSurfaceID, 0: no force
    has_record: bool, // +0x10 m_activeSurface not null
    rolling_resistance: f32, // the active SSurfaceData's
    legacy: bool, // RuntimeTyre::m_useLegacyHack
    slip: Slip, // +0x174
    slip_speed: f32, // +0x184 m_forwardVelocityForSlipRatio: the wheel speed of the last slip ratio
    slip_window: f32, // +0x188 the slip ratio clamp
    forces: Forces, // +0x18c
    skid: [2]f32, // +0x1a0 longitudinal, lateral
    patch_forward: f32, // +0x1b0 m_contactPatchForwardSpeed
    patch_side: f32, // +0x1b4
    wheel_forward: f32, // +0x1b8 m_contactWheelForwardSpeed
    effective_camber: f32, // +0x1bc

    // 0xf5f7d0 Tyre::calculateSlipRatio: clamped to a window that opens with speed; at a crawl it
    // averages with the last value, and it is 0 when the wheel speed changes sign
    pub fn setSlipRatio(t: *Tyre, wheel_forward: f32, patch_forward: f32, patch_side: f32) void {
        const fw = @abs(wheel_forward);
        const k = ((@abs(patch_side) + math.maxss(fw, @abs(patch_forward))) - 0.05) / 3.95;
        const window: f32, const floor: f32 = if (k > 1) .{ 1, -1 } else if (k > 0) .{ k, -k } else .{ 0, -0.0 };
        const last = t.slip_speed;
        t.slip_speed = wheel_forward;
        t.slip_window = window;
        const r = (wheel_forward - patch_forward) / (if (0.05 < fw) fw else 0.05);
        const v = if (r > window) window else math.maxss(r, floor);
        t.slip.ratio = if (0 >= last * wheel_forward) 0 else if (fw > 0.05) v else (v + t.slip.ratio) * 0.5;
        t.slip.braking = 0 < t.slip.ratio * wheel_forward;
    }

    // 0xf5f900 Tyre::calculateSlipAngle: folded into -pi/2..pi/2, 0 below 0.1 m/s
    pub fn setSlipAngle(t: *Tyre, wheel_forward: f32, wheel_side: f32) void {
        if (!(wheel_side * wheel_side + wheel_forward * wheel_forward > 0.01)) {
            t.slip.angle = 0;
            return;
        }
        const a = math.atan2(wheel_side, wheel_forward);
        t.slip.angle = if (!(-half_pi <= a)) -pi - a else if (half_pi < a) pi - a else a;
    }

    // 0xf5f6f0 Tyre::getPeakSlipRatio: blends by the load itself, clamped to 1..4
    pub fn peakSlipRatio(t: Tyre, load: f32) f32 {
        if (!t.has_record) return 0.1;
        const g = t.set.longitudinal;
        return (g.peak_x4 - g.peak_x1) * clampLoad1(load) + g.peak_x1;
    }

    // 0xf5f750 Tyre::getPeakSlipAngle: the game blends peak_x1 with itself, so this is peak_x1
    pub fn peakSlipAngle(t: Tyre, load: f32) f32 {
        if (!t.has_record) return 0.1;
        const x1 = t.set.lateral.peak_x1;
        return (x1 - x1) * clampLoad1(load) + x1;
    }

    // 0xf67000 Tyre::calcSkidSlipRatio
    pub fn skidSlipRatio(t: Tyre, min_speed: f32) f32 {
        const r = (t.wheel_forward - t.patch_forward) / math.maxss(@abs(t.wheel_forward), min_speed);
        return if (r > 1) 1 else math.maxss(r, -1);
    }

    // 0xf67050 Tyre::calcSkidSlipAngle
    pub fn skidSlipAngle(t: Tyre, min_speed: f32) f32 {
        return math.atan2(@abs(t.patch_side), math.maxss(@abs(t.wheel_forward), @abs(min_speed))) * math.sign(t.slip.angle);
    }

    // 0xf66c70 Tyre::calculateSkidding: how far past the peak slips the tyre works, for sound and marks
    pub fn setSkidding(t: *Tyre, wheel_forward: f32, patch_forward_rel: f32, load: f32) void {
        if (!(load > 0)) {
            t.skid = .{ 0, 0 };
            return;
        }
        const w = @abs(t.wheel_forward);
        const speed = if (4 < w) w else 4;
        const angle = @abs(math.sign(t.slip.angle) * math.atan2(@abs(t.patch_side), speed));
        var peak: f32 = 0.1;
        if (!t.has_record) {
            const v = angle - 0.1;
            t.skid[1] = if (v > 0) v else 0;
        } else {
            const x1 = t.set.lateral.peak_x1;
            const v = angle - (x1 + clampLoad1(load) * (x1 - x1));
            t.skid[1] = if (0 < v) v else 0;
            const g = t.set.longitudinal;
            peak = clampLoad1(load) * (g.peak_x4 - g.peak_x1) + g.peak_x1;
        }
        const limit = (peak + 1) * wheel_forward;
        const dir: f32 = if (0 < patch_forward_rel) (if (limit <= patch_forward_rel) 1 else -1) else if (patch_forward_rel < 0) (if (patch_forward_rel <= limit) 1 else -1) else -1;
        const r = (t.wheel_forward - t.patch_forward) / speed;
        const over = (if (r <= 1 and -1 < r) @abs(r) else 1) - peak;
        t.skid[0] = math.maxss(over, 0) * dir;
    }
};

// 0xf5f9a0 Tyre::calculateSlipDirection: the unit (forward, side) slip of the patch, (1, 0) when still
pub fn slipDirection(wheel_forward: f32, patch_forward_rel: f32, patch_side_rel: f32) [2]f32 {
    const x = wheel_forward - patch_forward_rel;
    const s = patch_side_rel * patch_side_rel + x * x;
    if (!(0.001 < s)) return .{ 1, 0 };
    const inv = 1 / @sqrt(s);
    return .{ x * inv, patch_side_rel * inv };
}

// clamped to 1..4, NaN gives 1
fn clampLoad1(load: f32) f32 {
    return if (4 < load) 4 else math.maxss(load, 1);
}

// Tyre+0x174..: the slip the force code reads
pub const Slip = extern struct {
    ratio: f32,
    angle: f32, // rad
    direction: [2]f32, // normalised (ratio, tan angle), for the legacy hack
    braking: bool, // the longitudinal curve is read at ratio/(ratio+1)
};

// 0xf608c0 Tyre::calculateForces_v2: friction ellipse. One curve, blended between the lateral and the
// longitudinal one by the slip direction, gives the force size; the ellipse of the two pure curves its direction.
// z: load ratio; camber: the caller's camber input [likely]
pub fn ellipse(set: GraphSet, surface: u32, slip: Slip, z: f32, camber: f32) Forces {
    const r = @abs(slip.ratio);
    const a = @abs(slip.angle);
    const zc = clampLoad(z);
    const an = a / half_pi;
    const e = set.grip_vs_weight;
    const along = squaredCos(math.atan2(an, r)); // 1 for pure longitudinal slip
    const mixed = mix(set.lateral, 1 - along, set.longitudinal, along);
    if (!(@as(f64, zc) > 0.0001) or !(mixed.peak_y1 > 0)) return .{};
    const combined = loadGraph(mixed, e, zc);
    if (surface == 0 or !(z > 0.01) or !(combined.peak_y > 0.01)) return .{};
    const lon = loadGraph(set.longitudinal, e, zc);
    const lat = loadGraph(set.lateral, e, zc);

    const size = sample(combined, clampUnit(@sqrt(an * an + r * r)));
    const s = @sqrt((an / lat.peak_x) * (an / lat.peak_x) + (r / lon.peak_x) * (r / lon.peak_x));
    var shape: f32 = 1;
    if (0x1.921fb6p2 > s) {
        const q = lat.gradient / (lon.gradient * half_pi);
        shape = (q + 1) * 0.5 - 0.5 * (1 - q) * math.cos(s * 0.5);
    }
    const side = shape * math.tan(a);
    const d = @sqrt(side * side + r * r);
    const fx = if (d > 0) size * r / d else 0;
    const fy = if (d > 0) side * size / d else 0;
    return finish(set, slip, fx, fy, sample(lon, r), sample(lat, an), zc, camber);
}

// 0xf5fa40 Tyre::calculateForces_v1: similarity method, each pure curve scaled down by the other slip.
// Below 0.01 rad or 0.01 slip ratio the curve gradients stand in for the forces.
pub fn similarity(set: GraphSet, surface: u32, slip: Slip, legacy: bool, z: f32, camber: f32) Forces {
    const r = @abs(slip.ratio);
    const a = @abs(slip.angle);
    const zc = clampLoad(z);
    const lon = loadGraph(set.longitudinal, set.grip_vs_weight, zc);
    const lat = loadGraph(set.lateral, set.grip_vs_weight, zc);
    if (surface == 0 or !(z > 0.01) or !(lon.peak_y > 0.01) or !(lat.peak_y > 0.01)) return .{};
    const pure_lon = sample(lon, if (slip.braking) r / (r + 1) else r);
    const pure_lat = sample(lat, a / half_pi);

    const fl2 = pure_lon * pure_lon;
    const ft2 = pure_lat * pure_lat;
    const gt = lat.gradient / half_pi;
    const gt2 = gt * gt;
    const gl2 = lon.gradient * lon.gradient;
    const r2 = r * r;
    const t = math.tan(a);
    const t2 = t * t;
    const small_angle = 0.01 > a;
    const small_ratio = 0.01 > r;
    const x2 = if (small_angle) (if (small_ratio) gl2 else fl2) else if (small_ratio) gl2 * t2 else fl2 * t2;
    const y2 = if (small_angle) (if (small_ratio) gt2 else gt2 * r2) else if (small_ratio) ft2 else ft2 * r2;
    const kx = pure_lon / @sqrt(x2 / y2 + 1);
    const ky = pure_lat / @sqrt(y2 / x2 + 1);

    const slack = (1 - r) * (1 - r);
    const w = if (lon.preshape > 0) 1 else slack;
    const c = math.cos(a);
    const m = if (small_ratio) @sqrt(w * c * c * gl2 / gt2 + 1) else @sqrt(w * c * c * fl2 / (r2 * gt2) + 1);
    const n = if (small_angle) @sqrt(slack * gt2 / gl2 + 1) else @sqrt(slack * ft2 / (gl2 * t2) + 1);
    var fx = kx * m;
    var fy = n * ky;
    if (legacy) { // RuntimeTyre::m_useLegacyHack: the total along the slip direction
        const total = @sqrt(fy * fy + fx * fx);
        fx = @abs(slip.direction[0]) * total;
        fy = @abs(slip.direction[1]) * total;
    }
    return finish(set, slip, fx, fy, pure_lon, pure_lat, zc, camber);
}

// signs, how far into the pure curves the tyre works, camber thrust
fn finish(set: GraphSet, slip: Slip, fx: f32, fy: f32, pure_lon: f32, pure_lat: f32, zc: f32, camber: f32) Forces {
    var out: Forces = .{ .longitudinal = math.sign(slip.ratio) * fx, .lateral = fy * math.sign(slip.angle) };
    out.slip_ratio_used = if (pure_lon > 1e-6) @abs(out.longitudinal) * slip.ratio / pure_lon else 0;
    if (pure_lat > 1e-6) {
        out.slip_angle_used = slip.angle * @abs(out.lateral) / pure_lat;
        out.mixed_lateral_scale = @abs(out.lateral) / pure_lat;
    }
    out.camber = math.pow(zc, set.grip_vs_weight) * (camber * set.camber);
    out.lateral = out.camber + out.lateral;
    if (0 > slip.angle * out.lateral) out.lateral = 0; // camber thrust never turns the force against the slip
    return out;
}

fn clampLoad(z: f32) f32 {
    return if (z > 4) 4 else if (0 < z) z else 0;
}

// cos^2 clamped to 0..1, NaN gives 0
fn squaredCos(x: f32) f32 {
    const c = math.cos(x);
    const c2 = c * c;
    if (c2 > 1) return 1;
    return if (c2 > 0) c2 else 0;
}

fn clampUnit(x: f32) f32 {
    if (x > 1) return 1;
    return if (x > 0) x else 0; // maxss: NaN gives 0
}

fn mix(a: Graph, wa: f32, b: Graph, wb: f32) Graph {
    var out: Graph = undefined;
    inline for (std.meta.fields(Graph)) |f| @field(out, f.name) = @field(a, f.name) * wa + @field(b, f.name) * wb;
    return out;
}
