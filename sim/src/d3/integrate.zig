const std = @import("std");
const body = @import("../shared/body.zig");
const vec = @import("../shared/vec.zig");
const Body = body.Body;
const State = body.State;
const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;

pub const Impulse = extern struct {
    full: V3,
    weighted: V3, // each substep impulse times the fraction of the step left after it
};

pub const Contact = extern struct { // one wheel's impulses, applied in this order
    point: V3, // world
    suspension: Impulse, // along the axis, body inertia
    lateral: Impulse, // tyre inertia
    longitudinal: Impulse, // tyre inertia
    across: Impulse, // the tyre's normal impulse across the axis, body inertia
};

pub const Forces = struct {
    accel: V3, // (v - v0)/dt of the force code, 0x7fc7d0
    ang_accel: V3,
    tyre_inertia: V3, // speed-scaled body inertia, 0x7d2330
    contacts: []const Contact,
};

const ImpulseSum = std.meta.FieldEnum(Impulse);

// 0x7f56d0: the pose from the weighted sums, the velocity from the full sums
pub fn step(b: Body, s: State, f: Forces, dt: f32) State {
    const moved = body.advance(withImpulses(b, s, f, .weighted), dt);
    var out = withImpulses(b, s, f, .full);
    out.pos = moved.pos + splat(dt) * f.accel * splat(dt * 0.5);
    out.axes = moved.axes;
    out.vel += splat(dt) * f.accel;
    out.angvel += splat(dt) * f.ang_accel; // no rotation term for alpha
    return out;
}

fn withImpulses(b: Body, s: State, f: Forces, comptime sum: ImpulseSum) State {
    const tyre: Body = .{ .mass = b.mass, .inertia = f.tyre_inertia };
    var out = s;
    for (f.contacts) |c| {
        const arm = c.point - s.pos;
        body.applyImpulse(&out, b, @field(c.suspension, @tagName(sum)), arm);
        body.applyImpulse(&out, tyre, @field(c.lateral, @tagName(sum)), arm);
        body.applyImpulse(&out, tyre, @field(c.longitudinal, @tagName(sum)), arm);
        body.applyImpulse(&out, b, @field(c.across, @tagName(sum)), arm);
    }
    return out;
}

fn expectVec(expected: V3, actual: V3, tolerance: f32) !void {
    inline for (0..3) |k| try std.testing.expectApproxEqAbs(expected[k], actual[k], tolerance);
}

const identity: [3]V3 = .{ .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, 1 } };
const at_rest: State = .{ .pos = splat(0), .axes = identity, .vel = splat(0), .angvel = splat(0) };
const zero: Impulse = .{ .full = splat(0), .weighted = splat(0) };
const box: Body = .{ .mass = 1000, .inertia = .{ 1700, 2000, 400 } };
const no_forces: Forces = .{ .accel = splat(0), .ang_accel = splat(0), .tyre_inertia = box.inertia, .contacts = &.{} };

test "free fall is exact for constant acceleration" {
    var f = no_forces;
    f.accel = .{ 0, -9.81, 0 };
    var s = at_rest;
    s.vel = .{ 3, 0, 0 };
    for (0..25) |_| s = step(box, s, f, 0.04);
    try expectVec(.{ 3, -4.905, 0 }, s.pos, 1e-4);
    try expectVec(.{ 3, -9.81, 0 }, s.vel, 1e-4);
}

test "pure spin keeps |w| and turns the axes" {
    const ball: Body = .{ .mass = 1, .inertia = splat(2) };
    var s = at_rest;
    s.angvel = .{ 0, std.math.pi / 2.0, 0 };
    for (0..100) |_| s = step(ball, s, no_forces, 0.01);
    try std.testing.expectApproxEqAbs(std.math.pi / 2.0, @sqrt(dot(s.angvel, s.angvel)), 1e-6);
    try expectVec(.{ 1, 0, 0 }, s.axes[2], 1e-5); // forward turns to right about +y
    try expectVec(.{ 0, 0, -1 }, s.axes[0], 1e-5);
}

test "impulse at the centre of mass only pushes" {
    const j: V3 = .{ 100, 0, 50 };
    const contact: Contact = .{
        .point = splat(0),
        .suspension = .{ .full = j, .weighted = j * splat(0.5) },
        .lateral = zero,
        .longitudinal = zero,
        .across = zero,
    };
    var f = no_forces;
    f.contacts = &.{contact};
    const s = step(box, at_rest, f, 0.1);
    try expectVec(j / splat(box.mass), s.vel, 1e-6);
    try expectVec(splat(0.1) * j * splat(0.5) / splat(box.mass), s.pos, 1e-6);
    try expectVec(splat(0), s.angvel, 0);
}

test "off-centre impulse spins by the matching inertia" {
    const j: V3 = .{ 0, 100, 0 };
    const at_nose: V3 = .{ 0, 0, 2 };
    var f = no_forces;
    f.tyre_inertia = .{ 3400, 2000, 400 };
    f.contacts = &.{
        .{ .point = at_nose, .suspension = .{ .full = j, .weighted = j }, .lateral = zero, .longitudinal = zero, .across = zero },
        .{ .point = at_nose, .suspension = zero, .lateral = .{ .full = j, .weighted = j }, .longitudinal = zero, .across = zero },
    };
    const s = step(box, at_rest, f, 0.01);
    try expectVec(.{ -200.0 / 1700.0 - 200.0 / 3400.0, 0, 0 }, s.angvel, 1e-6);
}
