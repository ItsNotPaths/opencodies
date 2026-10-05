// the physics engine's rigid body as the car code reads it (Physics::PhysicsRigidBody), and the
// DirectXMath vector sums the car code uses
const std = @import("std");

pub const V4 = @Vector(4, f32);

pub const Body = extern struct {
    angvel: [4]f32, // +0x1e0, world
    vel: [4]f32, // +0x1f0
    com: [4]f32, // *(+0x3b0): centre of mass, world
};

// 0xe72d40 PhysicsRigidBody::computeVelocityOfPointOnBody: vel + angvel x (p - com), lane by lane as
// DirectXMath's permuted cross product, so lane w is vel.w + (angvel.w r.w - angvel.w r.w)
pub fn pointVelocity(b: Body, p: V4) V4 {
    const r = p - @as(V4, b.com);
    const w: V4 = b.angvel;
    const a: V4 = .{ w[1], w[2], w[0], w[3] };
    const c: V4 = .{ r[2], r[0], r[1], r[3] };
    const d: V4 = .{ w[2], w[0], w[1], w[3] };
    const e: V4 = .{ r[1], r[2], r[0], r[3] };
    return @as(V4, b.vel) + (a * c - d * e);
}

// XMVector4Dot: (y + w) + (x + z)
pub fn dot4(a: V4, b: V4) f32 {
    const p = a * b;
    return (p[1] + p[3]) + (p[0] + p[2]);
}

// XMVector4Normalize with the sum of squares given (its order differs between call sites):
// 0 for a zero vector, QNaN for an infinite one
pub fn normalize4(v: V4, len_sq: f32) V4 {
    const len = @sqrt(len_sq);
    if (len_sq == std.math.inf(f32)) return @splat(std.math.nan(f32));
    if (len == 0) return @splat(0);
    return v / @as(V4, @splat(len));
}

// XMVector3Cross as the car code permutes it: (b.zxy a.yzx - b.yzx a.zxy), lane w = b.w a.w - b.w a.w
pub fn cross(a: V4, b: V4) V4 {
    const ay: V4 = .{ a[1], a[2], a[0], a[3] };
    const bz: V4 = .{ b[2], b[0], b[1], b[3] };
    const az: V4 = .{ a[2], a[0], a[1], a[3] };
    const by: V4 = .{ b[1], b[2], b[0], b[3] };
    return bz * ay - by * az;
}

// in Car::updateAllCars (0xf75f70) after the car jobs: the pose rows made orthonormal again, forward first
pub fn orthonormal(rows: [4][4]f32) [4][4]f32 {
    const forward = unit(rows[2]);
    const up = unit(cross(forward, rows[0]));
    return .{ unit(cross(up, forward)), up, forward, rows[3] };
}

fn unit(v: V4) V4 {
    return normalize4(v, dot4(v, v));
}

// the car's own body (RigidBody, car+0x6590), integrated by the car code
pub const RigidBody = extern struct {
    mass: f32,
    inverse_mass: f32,
    inertia: [3]f32, // body frame diagonal
    vel: [4]f32, // +0x20
    angvel: [4]f32, // +0x30, world
    rows: [4][4]f32, // +0x40 orientation rows (right, up, forward) and position
    force: [4]f32, // +0x80, summed until the integration
    torque: [4]f32, // +0x90
};

// XMVector4Transform by the orientation rows: lane k = (y r1 + w r3) + (z r2 + x r0)
pub fn transform(rows: [4][4]f32, v: V4) V4 {
    const r0: V4 = rows[0];
    const r1: V4 = rows[1];
    const r2: V4 = rows[2];
    const r3: V4 = rows[3];
    return (@as(V4, @splat(v[1])) * r1 + @as(V4, @splat(v[3])) * r3) + (@as(V4, @splat(v[2])) * r2 + @as(V4, @splat(v[0])) * r0);
}

// 0xf4f0d0 calculateChangeInAngularVelocity: of an impulse j at r from the position
pub fn angularChange(b: RigidBody, j: V4, r: V4) V4 {
    const c = cross(r, j);
    const local: V4 = .{ dot4(b.rows[0], c) / b.inertia[0], dot4(b.rows[1], c) / b.inertia[1], dot4(b.rows[2], c) / b.inertia[2], 0 };
    return transform(b.rows, local);
}

// 0xf51810 RigidBody::applyForceAtRelativePosition: summed until the integration; torque r x f, lane w
// r.w f.w - r.w f.w
pub fn applyForce(b: *RigidBody, f: V4, r: V4) void {
    b.force = @as(V4, b.force) + f;
    const p1: V4 = .{ r[1] * f[2], r[2] * f[0], r[0] * f[1], r[3] * f[3] };
    const p2: V4 = .{ r[2] * f[1], r[0] * f[2], r[1] * f[0], r[3] * f[3] };
    b.torque = (p1 - p2) + @as(V4, b.torque);
}

fn splat(x: f32) V4 {
    return @splat(x);
}

fn bits(u: u32) f32 {
    return @bitCast(u);
}

// cvtps2dq then back: round half to even
fn roundEven(q: f32) f32 {
    const t = @trunc(q);
    const d = q - t;
    if (@abs(d) > 0.5 or (@abs(d) == 0.5 and @mod(t, 2) != 0)) return t + std.math.sign(d);
    return t;
}

// 0x55b610 XMVectorSinCos: reduced to +-pi by a multiple of 2 pi, then Taylor series to x^23 and x^22
pub fn sinCos(angle: f32) [2]f32 {
    var q = angle * bits(0x3e22f983); // 1/(2 pi)
    if (0x4b000000 > @as(i32, @bitCast(@as(u32, @bitCast(q)) & 0x7fffffff))) q = roundEven(q);
    const x = angle - q * bits(0x40c90fdb); // 2 pi
    var p: [24]f32 = undefined;
    p[1] = x;
    p[2] = x * x;
    p[3] = x * p[2];
    p[4] = p[2] * p[2];
    p[5] = p[3] * p[2];
    p[6] = p[3] * p[3];
    p[7] = p[4] * p[3];
    p[8] = p[4] * p[4];
    p[9] = p[5] * p[4];
    p[10] = p[5] * p[5];
    p[11] = p[6] * p[5];
    p[12] = p[6] * p[6];
    p[13] = p[6] * p[7];
    p[14] = p[7] * p[7];
    p[15] = p[7] * p[8];
    p[16] = p[8] * p[8];
    p[17] = p[8] * p[9];
    p[18] = p[9] * p[9];
    p[19] = p[9] * p[10];
    p[20] = p[10] * p[10];
    p[21] = p[10] * p[11];
    p[22] = p[11] * p[11];
    p[23] = p[11] * p[12];
    const cos_k = [_]u32{ 0x3d2aaaab, 0xbab60b61, 0x37d00d01, 0xb493f27e, 0x310f76c8, 0xad49cba5, 0x29573f9f, 0xa53413c3, 0x20f2a15d, 0x9c8671cb };
    const sin_k = [_]u32{ 0x3c088889, 0xb9500d01, 0x3638ef1d, 0xb2d7322b, 0x2f309231, 0xab573f9f, 0x274a963c, 0xa317a4da, 0x1eb8dc78, 0x9a3b0da1 };
    var c = p[2] * -0.5 + 1;
    for (cos_k, 0..) |k, i| c = c + bits(k) * p[4 + 2 * i];
    var s = p[3] * bits(0xbe2aaaab) + x;
    for (sin_k, 0..) |k, i| s = s + bits(k) * p[5 + 2 * i];
    return .{ s, c };
}

// the rows times the rotation by angle about the unit axis n (XMMatrixRotationNormal; also neon::rotateAboutAxis
// 0xeb9770)
fn turn(out: *[4][4]f32, rows: [3]V4, n: V4, angle: f32) void {
    const s, const c = sinCos(angle);
    const sn = splat(s) * n;
    const nsn = splat(0) - sn;
    const omc = splat(1) - splat(c);
    var rot: [3]V4 = .{
        (splat(0) + n * splat(n[0])) * omc + V4{ c, sn[2], nsn[1], 0 },
        V4{ nsn[2], c, sn[0], 0 } + (splat(n[1]) * n + splat(0)) * omc,
        V4{ sn[1], nsn[0], c, 0 } + (splat(n[2]) * n + splat(0)) * omc,
    };
    for (&rot) |*row| row[3] = 0;
    const r3_rot: V4 = .{ 0, 0, 0, 1 };
    for (out[0..3], rows) |*o, r| {
        o.* = (splat(r[1]) * rot[1] + splat(r[3]) * r3_rot) + (splat(r[0]) * rot[0] + splat(r[2]) * rot[2]);
    }
}

// the car's body in the physics world (Physics::PhysicsRigidBody): its centre of mass frame (+0x160, row 3 the
// position), velocity (+0x1f0) and angular velocity (+0x1e0, world)
pub const WorldPose = extern struct {
    rows: [4][4]f32,
    vel: [4]f32,
    angvel: [4]f32,
};

// 0xe638c0 PhysicsRigidBody::integratePositionAndOrientation: the world moves the body by its end velocities, not the
// car code's midpoint; the car takes this pose back in pre_update. Not ported: the world's orthonormalisation of one
// body per frame (round robin, the 555 among 957 bodies)
pub fn integrateInWorld(p: *WorldPose, dt: f32) void {
    p.rows[3] = splat(dt) * @as(V4, p.vel) + @as(V4, p.rows[3]);
    const w: V4 = p.angvel;
    const len = @sqrt(dot4(w, w));
    if (len > bits(0x233877aa)) turn(&p.rows, .{ p.rows[0], p.rows[1], p.rows[2] }, w * splat(1 / len), dt * len);
}

// 0xf54030 RigidBody::integrate: the summed force and torque over dt, semi-implicit; the rotation as the
// orientation times the rotation about the new angular velocity
pub fn integrate(b: *RigidBody, dt: f32) void {
    const r0: V4 = b.rows[0];
    const r1: V4 = b.rows[1];
    const r2: V4 = b.rows[2];
    const r3: V4 = b.rows[3];
    const tdt = @as(V4, b.torque) * splat(dt);
    const dv = splat(b.inverse_mass * dt) * @as(V4, b.force);
    const a: V4 = .{ dot4(tdt, r0) / b.inertia[0], dot4(tdt, r1) / b.inertia[1], dot4(tdt, r2) / b.inertia[2], 0 };
    const dw = (r2 * splat(a[2]) + splat(a[0]) * r0) + (splat(0) * r3 + r1 * splat(a[1]));
    const pos = splat(dt) * (splat(0.5) * dv + @as(V4, b.vel)) + r3;
    const w = @as(V4, b.angvel) + dw;
    const w2 = dot4(w, w);
    if (w2 > bits(0x0704ec3c)) {
        const len = @sqrt(w2);
        turn(&b.rows, .{ r0, r1, r2 }, splat(1 / len) * w, len * dt);
    }
    b.rows[3] = pos;
    b.angvel = w;
    b.vel = dv + @as(V4, b.vel);
    b.force = .{ 0, 0, 0, 0 };
    b.torque = .{ 0, 0, 0, 0 };
}
