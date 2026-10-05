const body = @import("../shared/body.zig");
const vec = @import("../shared/vec.zig");
const car_ = @import("car.zig");
const Car = @import("../car.zig").Car;
const Wheel = @import("force.zig").Wheel;

const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;

// 0x7ec260: gravity, drag, rolling resistance, downforce; each line sees the one before
pub fn apply(car: Car, s: *body.State, wheels: [4]Wheel, dt: f32) void {
    const r = s.axes;
    var v = s.vel;
    v[1] += -9.81 * dt;
    v = drag(v, r[0], car.drag.?[0], car.mass, dt);
    v = drag(v, r[1], car.drag.?[1], car.mass, dt);
    const u = dot(v, r[2]);
    const air = @abs(u) * car.drag.?[2] * u;
    const roll = rolling(car, wheels) * (car.mass * 3);
    v += r[2] * splat(-((if (u > 0) air + roll else air - roll) * dt / car.mass));
    s.vel = v;
    downforce(car, s, wheels, dt);
}

fn drag(v: V3, axis: V3, c: f32, mass: f32, dt: f32) V3 {
    const u = dot(v, axis);
    return v + axis * splat(-(@abs(u) * c * dt * u / mass));
}

fn rolling(car: Car, wheels: [4]Wheel) f32 {
    var sum: f32 = 0;
    for (wheels, car.wheels) |w, p| {
        if (w.contact) sum += p.tyre.circle.rolling / wheels.len;
    }
    return sum;
}

// at the pressure point on the body z axis; directionality 0
fn downforce(car: Car, s: *body.State, wheels: [4]Wheel, dt: f32) void {
    const v = s.vel;
    const speed = vec.length(v);
    const forward = dot(if (speed > 1e-4) v * splat(1 / speed) else v, s.axes[2]) * speed;
    if (forward <= 0) return;
    const j = s.axes[1] * splat(-(car.aero.point.downforce * forward * forward) * dt);
    body.applyImpulse(s, car.rigid(), j, s.axes[2] * splat(car_.pressureZ(car, pressurePoint(car, wheels, v))));
}

fn pressurePoint(car: Car, wheels: [4]Wheel, v: V3) f32 {
    const a = car.aero.point;
    const falling, const rising = a.pressure_point_air;
    for (wheels) |w| if (!w.contact) return if (v[1] > 0) rising else falling;
    return a.pressure_point;
}
