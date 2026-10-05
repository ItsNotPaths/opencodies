const std = @import("std");
const body = @import("body.zig");
const vec = @import("vec.zig");

const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;

pub const Response = extern struct { axis: [4]f32, lat: [4]f32, lon: [4]f32 }; // speed change at each wheel

pub const Frame = struct { point: V3, axis: V3, lat: V3, lon: V3, normal: V3, rolling: f32 }; // rolling: radius * spin

pub const Rows = struct {
    axis_speed: f32, // body speed along the axis at the wheel
    lat_speed: f32,
    lon_slip: f32, // rolling speed minus r * spin
    response: [3]Response, // per unit impulse along axis, lat, lon at this wheel
    bottoming: [4]f32,
};

// 0x7f4150: wheel speeds at s and their change per unit impulse; an f32 finite difference with I_s, like D3
pub fn rows(b: body.Body, s: body.State, wheels: [4]Frame) [4]Rows {
    var out: [4]Rows = undefined;
    var base: [4]V3 = undefined;
    for (wheels, &out, &base) |w, *o, *v| {
        v.* = body.pointVelocity(s, w.point);
        o.axis_speed = dot(v.*, w.axis);
        o.lat_speed = dot(v.*, w.lat);
        o.lon_slip = dot(v.*, w.lon) - w.rolling;
    }
    const eps = b.mass * 1e-4; // 0xf30680
    for (wheels, &out) |src, *o| {
        for (&o.response, [3]V3{ src.axis, src.lat, src.lon }, 0..) |*r, dir, k| {
            var probe = s;
            body.applyImpulse(&probe, b, splat(eps) * dir, src.point - s.pos);
            for (wheels, out, base, 0..) |w, at, v0, j| {
                const v = body.pointVelocity(probe, w.point);
                r.axis[j] = (dot(v, w.axis) - at.axis_speed) * (1 / eps);
                r.lat[j] = (dot(v, w.lat) - at.lat_speed) * (1 / eps);
                r.lon[j] = ((dot(v, w.lon) - w.rolling) - at.lon_slip) * (1 / eps);
                if (k == 0) o.bottoming[j] = bottoming((v - v0) * splat(1 / eps), w.axis, src.normal);
            }
        }
    }
    return out;
}

// speed change across the axis of wheel j, measured along the ground normal at the source wheel;
// the division is rcpps (exact under the emulator) and one Newton step, the plain quotient if that is NaN
fn bottoming(dv: V3, axis: V3, normal: V3) f32 {
    const across = dv - axis * splat(dot(dv, axis));
    const den = dot(axis, normal);
    const r = 1 / den;
    const q = dot(across, normal) * r;
    const refined = (1 - r * den) * q + q;
    return if (std.math.isNan(refined)) q else refined;
}
