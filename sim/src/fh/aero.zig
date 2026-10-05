// Body drag along each axis, side drag and downforce at the axles, wings. FH2 CalcAeroForceTorque 0x82e1caa8.
// Drafting, damage, wind and the downforce-to-tyre hack are not ported.
const std = @import("std");
const vec = @import("../shared/vec.zig");
const body = @import("../shared/body.zig");
const Axles = @import("../car.zig").Axles;

const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;
const cross = vec.cross;

// PhysicsSettings.ini HighSpeedDrag: the forward drag grows by half from 119.3 to 122.9 m/s
const high_speed = [2]f32{ 119.3, 122.9 };
const high_speed_scale = [2]f32{ 1, 1.5 };

pub const Push = struct { force: V3, torque: V3 };

// axle_z: rear, front axle position along the body z, from the centre of mass
pub fn forces(a: Axles, b: body.State, axle_z: [2]f32) Push {
    const right, const up, const fwd = b.axes;
    const v = b.vel;
    const vf = dot(v, fwd);
    const vr = dot(v, right);
    const vu = dot(v, up);
    const speed = @sqrt(dot(v, v));
    const cos_a = if (speed > 0) vf / speed else 1;
    const hs = std.math.clamp((@abs(vf) - high_speed[0]) / (high_speed[1] - high_speed[0]), 0, 1);
    const hsd = (high_speed_scale[1] - high_speed_scale[0]) * hs + high_speed_scale[0];

    var out: Push = .{ .force = fwd * splat(-hsd * a.forward_drag * vf * @abs(vf)) + up * splat(-a.vertical_drag * vu * @abs(vu)), .torque = splat(0) };
    for (0..2) |axle| {
        const at = vec.toWorld(b.axes, .{ 0, 0, axle_z[axle] });
        var f = right * splat(-a.side_drag[axle] * vr * @abs(vr));
        f += up * splat(downforce(a.downforce[axle], a.zero_downforce_cos, cos_a, vf, v - up * splat(vu)));
        if (a.wings[axle]) |w| {
            f += fwd * splat(-w.drag * vf * @abs(vf)) + right * splat(-w.side_drag * vr * @abs(vr));
            const df = up * splat(downforce(w.downforce, w.zero_downforce_cos, cos_a, vf, v - up * splat(vu)));
            out.force += df;
            out.torque += cross(at, df * splat(w.torque_share));
        }
        out.force += f;
        out.torque += cross(at, f);
    }
    return out;
}

// along up: < 0 presses down; fades to 0 as the velocity turns from forward to the zero angle.
// A negative coefficient is lift, on the speed in the ground plane.
fn downforce(k: f32, zero_cos: f32, cos_a: f32, vf: f32, flat: V3) f32 {
    if (k < 0) return -k * dot(flat, flat);
    const fade = std.math.clamp((cos_a - zero_cos) / (1 - zero_cos), 0, 1);
    return -k * fade * vf * vf;
}
