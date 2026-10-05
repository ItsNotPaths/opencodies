const std = @import("std");
const vec = @import("vec.zig");
const bumps = @import("bumps.zig");
const Mesh = @import("mesh.zig").Mesh;

const material = @import("../material.zig");
const Material = material.Material;
const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;
const clamp = std.math.clamp;

pub const none: f32 = -1000; // 0xefe768, no hit

pub const Plane = extern struct { normal: V3 = .{ 0, 1, 0 }, offset: f32 = 0 }; // dot(normal, x) = offset; normal 0: a hole
pub const flat: [4]Plane = .{Plane{}} ** 4;

pub const Ground = union(enum) {
    planes: Planes,
    track: *const Mesh,

    pub fn cast(g: Ground, wheel: usize, from: V3, to: V3) ?Hit {
        return switch (g) {
            .planes => |p| castPlane(p.each[wheel], p.material, from, to),
            .track => |t| t.raycast(from, to),
        };
    }
};

// one plane per wheel, all of one material: for tests and the tools
pub const Planes = struct { each: [4]Plane = flat, material: *const Material = &plain };

// no effects at all: no bumps, no slowdown, every optional field null
pub const plain: Material = .{ .name = material.named("gen", "PLAN".*), .depth = 0, .bumps = material.smooth(.by_time), .slowdown = .{} };

pub const Hit = @import("mesh.zig").Hit;

pub const Probe = extern struct { // wheel i in the predicted pose
    mount: V3,
    axes: [3]V3,
    axis: V3, // suspension axis
    air: f32, // timer
    last: f32, // the contact height the substeps reached
    ray_offset: V3 = splat(0),
    contact_offset: f32 = 0, // W+0x16c
};

pub const Contact = extern struct { height: f32, normal: V3, material: ?*const Material };

pub const Wheel = struct { x: f32, ray_top: f32, travel_min: f32 }; // body frame x, the travel window the ray spans

// 0x7f4740 for wheel i: a ray along body up through the wheel travel, then the contact height chain
pub fn query(p: Wheel, gain: bumps.Gain, g: Ground, i: usize, probe: Probe, time: f32, speed: f32) Contact {
    const up = probe.axes[1];
    const top = p.ray_top + 0.25;
    const bottom = p.travel_min - 0.25;
    const side: f32 = if (p.x <= 0) 1 else -1;
    const lean = splat(side * clamp(probe.air * 1.4 - 0.2, 0, 1)); // a wheel long in the air feels outward
    const o = probe.ray_offset;
    const base = (probe.axes[1] * splat(o[1]) + probe.axes[2] * splat(o[2]) + probe.axes[0] * splat(o[0])) + probe.mount;
    const from = splat(top) * up + base + probe.axes[0] * splat(0.3) * lean;
    const to = splat(bottom) * up + base - probe.axes[0] * splat(0.05) * lean;
    const miss: Contact = .{ .height = none, .normal = bend(up, probe.axis), .material = null };
    const hit = g.cast(i, from, to) orelse return miss;
    const facing = dot(hit.normal, probe.axis);
    if (!(facing > 0.25881904)) return miss; // steeper than 75 deg
    const m = hit.material;
    const reached = hit.t * (bottom - top) + top - m.depth - probe.contact_offset; // out-of-round term: 0
    const b = m.bumps.by_time;
    const bumped = b.contact_share * bumps.height(gain, b, i, time, speed) + reached;
    return .{ .height = slope(bumped, facing, clamp(probe.last, p.travel_min, p.ray_top)), .normal = hit.normal, .material = m };
}

fn castPlane(p: Plane, m: *const Material, from: V3, to: V3) ?Hit {
    const toward = dot(p.normal, to - from);
    if (!(toward < 0)) return null;
    const t = (p.offset - dot(p.normal, from)) / toward;
    return if (t >= 0 and t <= 1) .{ .t = t, .normal = p.normal, .material = m } else null;
}

// past 20 deg off the axis a rising contact climbs slower
fn slope(height: f32, facing: f32, last: f32) f32 {
    const k = if (facing < 0.9397) (facing * 1.0641694) * (facing * 1.0641694) else 1;
    return if (k < 1 and height - last > 0) (height - last) * k + last else height;
}

// Tw+0x40 without a hit: the body up, bent toward the axis while it is more than 80 deg off
fn bend(normal: V3, axis: V3) V3 {
    var n = normal;
    while (dot(n, axis) < 0.173) n = vec.normalize(splat(0.05) * axis + n);
    return n;
}

// 0x7fd240: below 35 deg the tyre loses grip, down to 0.2; on metal only below 5 m/s
pub fn slopeGrip(m: Material, normal: V3, speed: f32) f32 {
    const d = normal[1] - 0.8192;
    if (d >= 0 or (m.metal() and speed >= 5)) return 1;
    return @max(5 * d + 1, 0.2);
}

test "flat ground under a level mount" {
    const wheel: Wheel = .{ .x = 0.8, .ray_top = 0.3, .travel_min = -0.15 };
    const gain: bumps.Gain = .{ .surface_factor = 0, .amplification = 1 };
    const axes: [3]V3 = .{ .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, 1 } };
    const probe: Probe = .{ .mount = .{ 0, 0.05, 0 }, .axes = axes, .axis = axes[1], .air = 0, .last = 0 };
    const c = query(wheel, gain, .{ .planes = .{} }, 0, probe, 0, 20);
    try std.testing.expectApproxEqAbs(@as(f32, -0.05), c.height, 1e-6);
    const holes: [4]Plane = .{Plane{ .normal = splat(0) }} ** 4;
    try std.testing.expectEqual(none, query(wheel, gain, .{ .planes = .{ .each = holes } }, 0, probe, 0, 20).height);
}
