// Wheel (W = car+0x7100 + 0x440 i) updates of Car::updateAll outside the suspension substeps: the low-speed
// patch point, tyre wear, dirt and temperature, and the grip those leave
const body = @import("body.zig");
const math = @import("math.zig");
const suspension = @import("suspension.zig");

const V4 = body.V4;
const dot4 = body.dot4;
const Wheel = suspension.Wheel;

fn splat(x: f32) V4 {
    return @splat(x);
}

// CarSolverEnvironment (car+0x210 -> CarJob's stack): the wheel axes and hub offsets for this frame
pub const Solver = extern struct {
    forward: [4][4]f32,
    right: [4][4]f32,
    rel_pos: [4][4]f32, // the hub from the body position, world axes
};

// 0xee39b0 Car::updatePreCalculateValues: the first wheels.len entries
pub fn precalculate(s: *Solver, wheels: []const Wheel, rows: [4][4]f32) void {
    const r0: V4 = rows[0];
    const r2: V4 = rows[2];
    for (wheels, 0..) |w, i| {
        const p = w.rest_position;
        s.rel_pos[i] = body.transform(rows, .{ p[0], (p[1] + w.height) - w.radius, p[2], 0 });
        const z, const x = w.direction;
        s.forward[i] = splat(x) * r0 + splat(z) * r2;
        s.right[i] = r2 * splat(-x) + r0 * splat(z);
    }
}

// 0xf3c030 Wheel::updatePatchPoints: the patch point rolls with the wheel and is held within 5 mm of the hub's
// ground point; the offset becomes the carcass deflection, the pull back its clamp rate. i: m_index
pub fn patchPoints(w: *Wheel, s: Solver, i: usize, position: V4, dt: f32) void {
    const forward: V4 = s.forward[i];
    const right: V4 = s.right[i];
    w.patch_point = ((splat(w.rotation_rate) * forward) * splat(w.radius)) * splat(dt) + @as(V4, w.patch_point);
    const centre = @as(V4, s.rel_pos[i]) + position;
    const d = @as(V4, w.patch_point) - centre;
    var along = dot4(forward, d);
    var across = dot4(right, d);
    const q = along * along + across * across;
    var pull: [2]f32 = .{ 0, 0 };
    if (q > 2.5e-5) {
        const k = 0.005 / @sqrt(q);
        pull = .{ (k - 1) * across, (k - 1) * along };
        along = along * k;
        across = across * k;
        w.patch_point = (forward * splat(w.patch_offset[1]) + right * splat(w.patch_offset[0])) + centre;
    }
    w.patch_offset = .{ across, along };
    w.patch_offset_clamp_rate = if (dt > 0) .{ pull[0] / dt, pull[1] / dt } else .{ 0, 0 };
}

// setup+0x4b0.. the tread temperature model
pub const Heat = extern struct {
    patch_shape: [2]f32, // front, rear: how the heat favours the middle
    spread_rate: f32, // between the zones
    road_rate: f32, // toward the ambient
    work_rate: f32, // per unit force
    roll_rate: f32, // per unit rolling
    ambient: f32, // setup+0x230
};

// 0xf3b8a0 Wheel::updateTemperature: heat from the tyre force and rolling, split over three zones by the camber
// against the ground, then each zone toward the ambient and its neighbour. rows: the car body
pub fn temperature(w: *Wheel, h: Heat, rows: [4][4]f32, dt: f32) void {
    const roll = @abs(w.rotation_rate) * w.upward_force[1] * h.roll_rate;
    const long, const lat = .{ w.generated_force[0], w.generated_force[1] };
    const force = @sqrt(lat * lat + long * long);
    const cap = w.upward_force[1] * 8;
    const work = if (force > cap) cap else if (0 < force) force else 0;
    const heat = work * h.work_rate + roll;
    const shape = if (0 < w.rest_position[2]) h.patch_shape[0] else h.patch_shape[1];
    const side: f32 = if (0 > w.rest_position[0]) -1 else 1;
    const s, const c = .{ math.sin(w.camber), math.cos(w.camber) };
    const axle = (splat(side) * @as(V4, rows[1])) * splat(s) + splat(c) * @as(V4, rows[0]);
    const tilt = side * dot4(axle, w.surface_normal);
    const inner = (0.5 * shape + 1) - tilt;
    const outer = (0.5 * shape + 1) + tilt;
    const t = w.temperature;
    const a = t[0] + inner * heat * dt;
    const a2 = (h.ambient - a) * h.road_rate * dt + a;
    const t0 = (t[1] - a2) * h.spread_rate * dt + a2;
    const b = t[1] + (1 - shape) * heat * dt;
    const b2 = (h.ambient - b) * h.road_rate * dt + b;
    const c2 = (h.ambient - (t[2] + heat * outer * dt)) * h.road_rate * dt + (t[2] + heat * outer * dt);
    const b3 = (t0 - b2) * h.spread_rate * dt + b2;
    const t1 = (t[2] - b3) * h.spread_rate * dt + b3;
    w.temperature = .{ t0, t1, (t1 - c2) * h.spread_rate * dt + c2 };
}

// WearAndTearTemplate (car+0x11e98)
pub const Wear = extern struct {
    horizontal_force: f32, // rate per unit long + lat force
    load: f32, // per unit vertical force
    skid: f32, // per unit skidding
    wheel_spin: f32, // m_rateBlatentWheelSpin
    wheel_spin_speed: f32, // the tread past the car speed by this spins
    grip_loss: f32,
    min_wear: f32, // m_minimumWearToFeelEffect
    full_wear: f32,
};

// the contact surface's part in the wear (SurfaceSpec +0xb4, +0xb8)
pub const Contact = struct { wear_multiple: f32, dirt_buildup: bool };

// 0xf3bc20 Wheel::updateSuspensionNew: off the ground only the air time; on it the tyre picks up or sheds dirt,
// wears (wear_rate 0: not at all) and heats. True when the wheel lands after more than half a second in the air
pub fn suspensionNew(w: *Wheel, contact: ?Contact, wear: Wear, speed: f32, h: Heat, rows: [4][4]f32, dt: f32, wear_rate: f32) bool {
    const surface = contact orelse {
        w.time_in_air = dt + w.time_in_air;
        return false;
    };
    const landed = w.time_in_air > 0.5;
    w.time_in_air = 0;
    const spin = @abs(w.rotation_rate);
    if (!surface.dirt_buildup) {
        const x = spin / 24;
        const rate: f32 = if (x > 1) 0.25 else if (x > 0) 0.25 * x else 0;
        const dirt = w.dirt - rate * dt;
        w.dirt = if (0 < dirt) dirt else 0;
    } else {
        const x = spin / 12;
        const rate: f32 = if (x > 1) 2 else if (x > 0) x + x else 0;
        const dirt = rate * dt + w.dirt;
        w.dirt = if (dirt < 1) dirt else 1;
    }
    if (wear_rate != 0) {
        var sum: f32 = 0;
        if (speed >= 1) {
            const k = wear.horizontal_force * @abs(@abs(w.generated_force[0]) + @abs(w.generated_force[1]));
            sum = if (k > 1) 1 else if (0 < k) k else 0;
            const v = @abs(w.generated_force[2]) * wear.load;
            sum = sum + (if (v > 1) 1 else if (v > 0) v else 0);
        }
        const s = wear.skid * w.skidding[2];
        const skid: f32 = if (s > 1) 1 else if (0 < s) s else 0;
        sum = (sum + skid) / 3;
        if (w.rotation_rate * w.radius > speed + wear.wheel_spin_speed) sum = sum + wear.wheel_spin / wear_rate;
        const worn = wear_rate * sum * surface.wear_multiple * dt + w.wear;
        w.wear = if (worn > 1) 1 else math.maxss(worn, 0);
    }
    temperature(w, h, rows, dt);
    return landed;
}

// SimulationSettings (car+0xe70) flags
pub const settings_temperature: u32 = 4;
pub const settings_dirt: u32 = 8;
pub const settings_wear: u32 = 2;
const best_temperature: f32 = 383.15; // .data 0x32ec474, a tweak

// in Car::updateSuspensionAndIntegration: the external grip multiple (Tw+0x98) left by wear, and by temperature,
// dirt and a burst tyre when the settings say so
pub fn externalGrip(w: Wheel, wear: Wear, settings: u32, burst: bool) f32 {
    const x = (w.wear - wear.min_wear) / (wear.full_wear - wear.min_wear);
    const felt = if (x > 1) 1 else math.maxss(x, 0);
    var g = 1 - w.wear * wear.grip_loss * felt;
    if (settings & settings_temperature != 0) {
        const t = @abs(((w.temperature[0] + w.temperature[1]) + w.temperature[2]) * 0.3333 - best_temperature) / 80;
        g = g * (if (t > 1) 0.65 else if (t > 0) 1 - t * 0.35 else 1);
    }
    if (settings & settings_dirt != 0) g = g * (1 - 0.5 * w.dirt);
    if (burst) g = g * (1 - w.burst_grip);
    return g;
}

// 0xefca00 Car::updateBrakeTemperatures: a braked disc on a turning wheel heats toward 923.15 K by its share of the
// car's weight, from 2 m/s on in full from 4 m/s; else it cools toward 293.15 K
pub fn brakeTemperatures(wheels: []Wheel, inverse_mass: f32, speed: f32, brake: f32, dt: f32) void {
    const k = 0.6795786 * inverse_mass;
    const s = (speed - 2) * 0.5;
    const by_speed: f32 = if (!(s > 0)) 0 else if (s > 1) 1 else s;
    for (wheels) |*w| {
        const u = k * w.upward_force[0];
        const b = (if (u > 1) 1 else if (0 < u) u else 0) * brake * by_speed;
        const t = w.brake_temperature;
        w.brake_temperature = if (b > 0 and @abs(w.rotation_rate) > 1e-6)
            923.15 - (1 - b * dt * 0.7) * ((923.15 - t) / 630) * 630
        else
            (1 - 0.333 * dt) * ((t - 293.15) / 630) * 630 + 293.15;
    }
}
