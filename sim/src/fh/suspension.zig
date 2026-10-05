// One vertical degree of freedom per wheel: spring, dampers with force caps, bump stop, anti-roll bars,
// and the wheel kept on the ground. FH2 CalcWheelNormalForceTorque 0x82e29a90, AddSwayBarForceToWheels
// 0x82e29e90, KeepSingleTireOnCurrentSurface 0x82e28550, ClampSuspensionTravel 0x82e1e368.
const std = @import("std");
const vec = @import("../shared/vec.zig");
const body = @import("../shared/body.zig");

const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;
const cross = vec.cross;

pub const g = 9.80665;

pub const Wheel = struct {
    hub: V3, // body frame at ride height; the travel replaces y
    radius: f32,
    spring: f32, // N/m
    free_travel: f32, // m, travel where the spring force is 0
    bump: f32, // N s/m
    rebound: f32,
    bump_cap: f32, // N
    rebound_cap: f32,
    bump_stop: f32, // N/m past max_compress
    bump_stop_damping: f32,
    max_stretch: f32, // m, travel limits
    max_compress: f32,
    tyre_max_compress: f32, // m the tyre gives before the travel moves
    unsprung: f32, // kg
};

// the state of one wheel that the suspension reads and writes
pub const Travel = struct {
    y: f32, // body frame height of the hub: larger is more compressed
    vel: f32, // world velocity of the wheel along the body up axis
    tyre: f32 = 0, // tyre compression, m
    dist_under: f32 = 0, // m past max_compress
    contact: bool = false,
    pseudo: bool = false, // within 2 cm above the ground: counts as contact, gives no wheel force
    normal: V3 = .{ 0, 1, 0 },
    sway_spring: f32 = 0,
    sway_damping: f32 = 0,
};

pub fn mount(w: Wheel, t: Travel) V3 {
    return .{ w.hub[0], t.y, w.hub[2] };
}

pub fn worldPoint(b: body.State, p: V3) V3 {
    return vec.toWorld(b.axes, p) + b.pos;
}

// the hub's travel speed: the wheel's world speed along up minus the body's at the mount
pub fn relativeVel(b: body.State, w: Wheel, t: Travel) f32 {
    const p = worldPoint(b, mount(w, t));
    return t.vel - dot(body.pointVelocity(b, p), b.axes[1]);
}

fn damper(w: Wheel, v: f32) f32 {
    return (if (v < 0) w.rebound else w.bump) * v;
}

fn cap(w: Wheel, f: f32) f32 {
    return std.math.clamp(f, -w.rebound_cap, w.bump_cap);
}

pub const Normal = struct {
    load: f32, // N on the tyre
    force: V3, // on the body at the contact
    torque: V3,
    acc: f32, // wheel acceleration along up
};

pub fn normalForce(b: body.State, w: Wheel, t: Travel) Normal {
    const up = b.axes[1];
    const spring = (t.y - w.free_travel) * w.spring;
    const suspension = cap(w, damper(w, relativeVel(b, w, t)));
    var out: Normal = .{ .load = 0, .force = splat(0), .torque = splat(0), .acc = 0 };
    var along: f32 = 0;
    if (t.contact) {
        const centre = worldPoint(b, mount(w, t));
        const ride = worldPoint(b, w.hub);
        const closing = -dot(body.pointVelocity(b, ride), t.normal);
        var damping = damper(w, closing) + t.sway_damping;
        var sum = spring + t.sway_spring;
        if (t.dist_under > 1e-4) {
            sum += w.bump_stop * t.dist_under;
            damping += @max(closing, 0) * w.bump_stop_damping;
        }
        const f = @max(cap(w, damping) + sum, 0);
        out.load = f;
        out.force = t.normal * splat(f);
        out.torque = cross(centre - up * splat(w.radius) - b.pos, out.force);
        if (!t.pseudo) along = dot(out.force, up);
    }
    out.acc = (along - (suspension + t.sway_spring + spring)) / w.unsprung - up[1] * g;
    return out;
}

pub const Bar = struct { stiffness: f32, damping: f32 }; // N/m, N s/m between the two wheels of an axle

// pair: left, right
pub fn swayBar(b: body.State, bar: Bar, w: [2]Wheel, t: *[2]Travel) void {
    var v: [2]f32 = .{ 0, 0 };
    for (0..2) |i| if (t[i].contact) {
        v[i] = dot(body.pointVelocity(b, worldPoint(b, w[i].hub)), b.axes[1]);
    };
    const spring = bar.stiffness * (t[0].y - t[1].y);
    const damping = -(bar.damping * (v[0] - v[1]));
    t[0].sway_spring += spring;
    t[1].sway_spring -= spring;
    t[0].sway_damping += damping;
    t[1].sway_damping -= damping;
}

// travel limits: the hub stops, and its speed into the stop goes
pub fn clampTravel(b: body.State, w: Wheel, t: *Travel) void {
    if (t.y < w.max_stretch + 1e-4) {
        t.y = @max(t.y, w.max_stretch);
        t.vel -= @min(relativeVel(b, w, t.*), 0);
    } else if (t.y > w.max_compress - 1e-4) {
        t.y = @min(t.y, w.max_compress);
        t.vel -= @max(relativeVel(b, w, t.*), 0);
    }
}

// the ground below the hub along the body down axis
pub const Ground = struct { height: f32, normal: V3 }; // height: body frame y

// the tyre may not go into the ground: first the tyre compresses, then the travel moves, then the bump stop
pub fn keepOnGround(b: body.State, w: Wheel, t: *Travel, ground: ?Ground) void {
    t.dist_under = 0;
    t.pseudo = false;
    const gr = ground orelse {
        t.contact = false;
        return;
    };
    const under = gr.height + w.radius - t.tyre - t.y; // > 0: the tyre is in the ground
    if (under < -0.02) {
        t.contact = false;
        return;
    }
    t.contact = true;
    t.pseudo = under < -0.005;
    t.normal = gr.normal;
    if (under > 0) t.tyre += under;
    if (t.tyre > w.tyre_max_compress) {
        t.y += t.tyre - w.tyre_max_compress;
        t.tyre = w.tyre_max_compress;
        if (t.y > w.max_compress) {
            t.dist_under = t.y - w.max_compress;
            t.y = w.max_compress;
        }
        // the wheel may run into the ground at 0.5 m/s at most
        const into = dot(wheelVelocity(b, w, t.*), gr.normal);
        if (into < -0.5) t.vel -= (into + 0.5) / @max(dot(gr.normal, b.axes[1]), 0.3);
    }
}

pub fn wheelVelocity(b: body.State, w: Wheel, t: Travel) V3 {
    return body.pointVelocity(b, worldPoint(b, mount(w, t))) + b.axes[1] * splat(relativeVel(b, w, t));
}
