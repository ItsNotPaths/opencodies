const vec = @import("vec.zig");
const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;
const cross = vec.cross;
const toBody = vec.toBody;
const toWorld = vec.toWorld;

pub const Body = struct {
    mass: f32,
    inertia: V3, // body frame diagonal, kg m^2
};

pub const State = extern struct {
    pos: V3,
    axes: [3]V3, // body axes in world (D3 matrix rows): right, up, forward
    vel: V3,
    angvel: V3, // world frame
};

// 0x7e9e00 and caller
pub fn applyImpulse(s: *State, b: Body, j: V3, arm: V3) void {
    s.vel += splat(1 / b.mass) * j;
    s.angvel += toWorld(s.axes, toBody(s.axes, cross(arm, j)) / b.inertia);
}

// 0x7d76b0
pub fn advance(s: State, h: f32) State {
    var out = s;
    out.pos += splat(h) * s.vel;
    const w2 = dot(s.angvel, s.angvel);
    if (w2 > 1e-34) {
        const w = @sqrt(w2);
        const turn = vec.rotation(s.angvel * splat(1 / w), w * h);
        for (&out.axes) |*a| a.* = toWorld(turn, a.*);
    }
    return out;
}

pub fn pointVelocity(s: State, p: V3) V3 {
    return s.vel + cross(s.angvel, p - s.pos);
}
