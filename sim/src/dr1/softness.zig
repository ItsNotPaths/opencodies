// How hard a body contact may push (dirt-rally.md §15): the impulse per metre of depth from the damage lattice
// (DamageFems) near the contact, divided by the impact speed scale
const std = @import("std");

const big = std.math.floatMax(f32); // gPhysicsInfinity: no limit

// DamageFems::m_points (DamageFems+0x2580, 64 bytes each): a lattice point and the box it may move in.
// max[3] is its strength; 0: the point does not count
pub const Point = extern struct { pos: [4]f32, velocity: [4]f32, min: [4]f32, max: [4]f32 };

// 0xf132b0 DamageFemsGetDamageScaleAtVelocity: the table (Hull.impact_softness) at the speed in mph, linear between
// its 10 mph steps; the game's table has 16 entries and holds its last value from 150 mph
pub fn speedScale(table: []const f32, speed: f32) f32 {
    const last = table.len - 1;
    const end: f32 = @floatFromInt(10 * last);
    const mph = speed * 2.232;
    if (0 > mph) return table[0];
    if (mph >= end) return table[last];
    const step = @floor(mph * 0.1);
    const top: f32 = @floatFromInt(last - 1);
    const i: usize = if (step >= top) last - 1 else if (step >= 0) @intFromFloat(step) else 0; // NaN: cvttss2si's INT_MIN, then 0
    const f = (mph - @as(f32, @floatFromInt(10 * i))) * 0.1;
    const w: [2]f32 = if (f > 1) .{ 0, 1 } else if (f > 0) .{ 1 - f, f } else .{ 1, 0 };
    return table[i + 1] * w[1] + table[i] * w[0];
}

// all: the impulse per metre from the strength of the points within one cube size, nearer ones weigh more.
// directional: the same, but a point whose box the push (reach along dir) leaves weighs less; null: not set
pub const Absorbed = struct { all: f32, directional: ?f32 };

// 0xf12f00 ImpulseAbsorbedPerMeterDirectional: at and dir in the lattice frame, mass the body's (PhysicsRigidBody+0x478)
pub fn absorbed(mass: f32, points: []const Point, cube: f32, at: [4]f32, dir: [4]f32, reach: f32) Absorbed {
    const range = cube * cube;
    var nearest = big;
    var nearest_point: ?usize = null;
    var near: f32 = 0; // the sums of the weights and of the weighted strengths
    var near_strength: f32 = 0;
    var kept: f32 = 0;
    var kept_strength: f32 = 0;
    for (points, 0..) |p, i| {
        const strength = p.max[3];
        if (strength == 0) continue;
        const d = [3]f32{ p.pos[0] - at[0], p.pos[1] - at[1], p.pos[2] - at[2] };
        const d2 = d[1] * d[1] + (d[0] * d[0] + d[2] * d[2]);
        if (nearest > d2) {
            nearest = d2;
            nearest_point = i;
        }
        if (!(range > d2)) continue;
        var out: f32 = 0; // how much of the push leaves the point's box
        for (0..3) |k| {
            const q = p.pos[k] + reach * dir[k];
            const leaves = if (dir[k] > 0) q - p.max[k] >= 0 else 0 > q - p.min[k];
            if (leaves) out += @abs(dir[k]);
        }
        const w = range - d2;
        near += w;
        near_strength += w * strength;
        const w_kept = w + d2 * out * 10;
        kept += w_kept;
        const keep = 1 - out;
        const g: f32 = if (keep > 1) 1 else if (keep > 0) keep else 0;
        kept_strength += w_kept * (strength * g);
    }
    const strength = if (near > 0) near_strength / near else if (nearest_point) |k| points[k].max[3] else return .{ .all = big, .directional = null };
    if (!(strength > 0)) return .{ .all = big, .directional = null };
    const all = 15 * mass / strength;
    if (kept > 0) {
        const kept_mean = kept_strength / kept;
        if (strength > kept_mean) return .{ .all = all, .directional = if (kept_mean > 0) mass * 15 / kept_mean else big };
    }
    return .{ .all = all, .directional = all };
}
