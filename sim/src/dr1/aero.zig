// Car::updateAerodynamics: drag on three body axes, rolling resistance and downforce, as forces on the body
const body = @import("body.zig");
const math = @import("math.zig");

const V4 = body.V4;

// setup+0x14.. and the car fields the aero reads
pub const Aero = extern struct {
    side_drag: f32, // setup+0x14
    vertical_drag: f32, // +0x18
    forward_drag: f32, // +0x1c fAirResistanceZ
    reverse_drag: f32, // +0x20 fAirResistanceNegativeZ
    centre_x: f32, // +0x24 where the side drag acts, 0 rear axle .. 1 front axle
    centre_y: f32, // +0x28 the vertical drag
    downforce_central: [2]f32, // +0x34, +0x38: forward, reverse (the Pikes Peak model)
    downforce: [3][2]f32, // body, front, rear; forward +0x3c +0x44 +0x4c, reverse +0x40 +0x48 +0x50
    wing_drag: [4]f32, // body forward +0x54, body reverse +0x58, front +0x60, rear +0x68: the forward drag blend by wing damage
    pressure_centre: f32, // +0x48c fCentreOfDownforcePressure
    pressure_centre_air: f32, // +0x490
    legacy_rolling: bool, // +0x148 m_useLegacyRollingResistance
    legacy_downforce: bool, // +0x149 m_useLegacyDownforce
    pikes_peak: bool, // +0x14a: the new downforce scales with speed squared

    drag_scale: f32, // car+0x6644 fAirResistanceZMultiplier
    wing_effect: [2]f32, // car+0x11efc front, +0x11f00 rear: damage
    wheels: u32, // car+0x6de0 m_wheelsNum
    wheels_on_ground: u32, // T+0x838
    legacy_downforce_central: f32, // car+0x8230
    legacy_centre: [2]f32, // car+0x8244: on the ground, in the air
    speed: f32, // car+0x8264
    axle_z: [2]f32, // front (wheel 2) and rear (wheel 0) restPosition.z
    altitude: f32, // car+0x1074, the mid frame height
    sea_level_raw: u64, // Engine::s_seaLevel at 0x338a8b0
    rolling: [4]f32, // per wheel: the tyre record's rolling resistance (0 without one)
    load: [4]f32, // Wheel+0x3a0
    contact: [4]bool, // Wheel+0xf0 not null
};

fn splat(x: f32) V4 {
    return @splat(x);
}

// a body point at local (0, 0, z): sums ordered per call site, as the game's matrix products
fn at(b: body.RigidBody, z: f32) [4]V4 {
    return .{ splat(0) * @as(V4, b.rows[0]), splat(0) * @as(V4, b.rows[1]), splat(z) * @as(V4, b.rows[2]), splat(1) * @as(V4, b.rows[3]) };
}

// the air density relative to sea level, as the aero and the engine read it; sea_level_raw: Engine::s_seaLevel,
// obfuscated
pub fn airDensity(altitude: f32, sea_level_raw: u64) f32 {
    const sea: f32 = @as(f32, @floatFromInt(@as(i64, @bitCast(sea_level_raw *% 0x887e527813a1713d)))) * 0.001;
    const alt = altitude - sea;
    return math.exp((if (0 > alt) @as(f32, -0.0) else -alt) / 7000);
}

// 0xf53490
pub fn update(a: Aero, b: *body.RigidBody) void {
    const rho = airDensity(a.altitude, a.sea_level_raw);
    const v: V4 = b.vel;
    const r0: V4 = b.rows[0];
    const r1: V4 = b.rows[1];
    const r2: V4 = b.rows[2];
    const pos: V4 = b.rows[3];
    const side = body.dot4(r0, v);
    const up = body.dot4(r1, v);
    const forward = body.dot4(r2, v);
    const fs = a.side_drag * rho * side * @abs(side);
    const fv = a.vertical_drag * rho * up * @abs(up);
    const reverse = 0 > forward;
    var cz = if (reverse) a.reverse_drag else a.forward_drag;
    if (!a.legacy_downforce) {
        const body_drag = if (reverse) a.wing_drag[1] else a.wing_drag[0];
        const front = a.wing_drag[2];
        const rear = a.wing_drag[3];
        cz = cz * (((a.wing_effect[0] * front + body_drag) + a.wing_effect[1] * rear) / ((body_drag + front) + rear));
    }
    const ff = @abs(forward) * (cz * rho * forward) * a.drag_scale;
    var rr = rolling(a, b.mass);
    if (!(forward >= 0)) rr = -rr;
    const fz = rr + ff;

    const zf, const zr = a.axle_z;
    const wb = zf - zr;
    const ps = at(b.*, a.centre_x * wb + zr);
    const pv = at(b.*, a.centre_y * wb + zr);
    body.applyForce(b, splat(fs) * (splat(0) - r0), ((ps[3] + ps[1]) + (ps[0] + ps[2])) - pos);
    body.applyForce(b, (splat(0) - r1) * splat(fv), ((pv[2] + pv[0]) + (pv[3] + pv[1])) - pos);
    b.force = (splat(0) - r2) * splat(fz) + @as(V4, b.force);

    if (!(a.speed > 1)) return;
    const q = forward / a.speed;
    if (!a.legacy_downforce) {
        const d = if (0 > q) [3]f32{ a.downforce[0][1], a.downforce[1][1], a.downforce[2][1] } else [3]f32{ a.downforce[0][0], a.downforce[1][0], a.downforce[2][0] };
        const wf = at(b.*, zf);
        const wc = at(b.*, wb * a.pressure_centre + zr);
        const wr = at(b.*, zr);
        var dir = r1;
        if (a.pikes_peak) {
            const central = if (0 > q) a.downforce_central[1] else a.downforce_central[0];
            dir = splat(forward * (-central * forward)) * r1;
        }
        body.applyForce(b, splat(a.wing_effect[0]) * (splat(d[1]) * dir), ((wf[2] + wf[0]) + (wf[1] + wf[3])) - pos);
        body.applyForce(b, splat(d[0]) * dir, ((wc[0] + wc[2]) + (wc[3] + wc[1])) - pos);
        body.applyForce(b, dir * splat(d[2]) * splat(a.wing_effect[1]), ((wr[1] + wr[3]) + (wr[2] + wr[0])) - pos);
        return;
    }
    if (!(q > 0)) return;
    const u = q * a.speed;
    const centre = if (a.wheels > a.wheels_on_ground) (if (0 < up) a.pressure_centre_air else a.legacy_centre[1]) else a.legacy_centre[0];
    const w = at(b.*, wb * centre + zr);
    const k = u * (-rho * a.legacy_downforce_central * u);
    body.applyForce(b, r1 * splat(k), ((w[2] + w[0]) + (w[3] + w[1])) - pos);
}

// the mean rolling resistance of the wheels by load (0 wheels: 0/0), or the legacy 0.07 per touching wheel
fn rolling(a: Aero, mass: f32) f32 {
    const n = a.wheels;
    if (a.legacy_rolling) {
        var sum: f32 = 0;
        for (0..@min(n, 4)) |k| {
            if (a.contact[k]) sum = sum + 0.07 / @as(f32, @floatFromInt(n));
        }
        return sum * (3 * mass);
    }
    var mean: f32 = undefined;
    if (n == 0) {
        const zero: f32 = 0;
        mean = zero / zero;
    } else {
        var sum: f32 = undefined;
        for (0..@min(n, 4)) |k| {
            const l = a.load[k];
            const weight: f32 = if (l > 1) 1 else if (0 < l) l else 0;
            const term = weight * a.rolling[k];
            sum = if (k == 0) term + 0 else sum + term;
        }
        mean = sum / @as(f32, @floatFromInt(n));
    }
    return 9.81 * mass * mean;
}
