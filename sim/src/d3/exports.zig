// the D3 model function by function, on the loaded car (d3_load_car), for tools/forcediff.py, groundiff.py,
// mathdiff.py and friends
const std = @import("std");
const sim = @import("../sim.zig");
const body = @import("../shared/body.zig");
const chassis = @import("../shared/chassis.zig");
const d3math = @import("../shared/d3math.zig");
const ground = @import("../shared/ground.zig");
const material = @import("../material.zig");
const vec = @import("../shared/vec.zig");
const aero = @import("aero.zig");
const engine = @import("engine.zig");
const force = @import("force.zig");
const integrate = @import("integrate.zig");
const longitudinal = @import("longitudinal.zig");
const slowdown = @import("slowdown.zig");
const steering = @import("steering.zig");

const car_ = @import("car.zig");
const zon = @import("../zon.zig");
const Car = @import("../car.zig").Car;
const Material = material.Material;
const Input = sim.Input;
const State = sim.State;
const World = sim.World;
const allocator = sim.allocator;

pub const Body = extern struct { mass: f32, inertia: [3]f32 };
pub const Impulse = extern struct { full: [3]f32, weighted: [3]f32 };
pub const Contact = extern struct { point: [3]f32, suspension: Impulse, lateral: Impulse, longitudinal: Impulse, across: Impulse };
pub const Forces = extern struct {
    accel: [3]f32,
    ang_accel: [3]f32,
    tyre_inertia: [3]f32,
    contacts: [*]const Contact,
    contact_count: u32,
};

const max_contacts = 4; // wheels

fn impulseFromAbi(i: Impulse) integrate.Impulse {
    return .{ .full = i.full, .weighted = i.weighted };
}

export fn body_step(b: *const Body, st: *State, f: *const Forces, dt: f32) void {
    var contacts: [max_contacts]integrate.Contact = undefined;
    const n = @min(f.contact_count, max_contacts);
    for (f.contacts[0..n], contacts[0..n]) |c, *out| {
        out.* = .{
            .point = c.point,
            .suspension = impulseFromAbi(c.suspension),
            .lateral = impulseFromAbi(c.lateral),
            .longitudinal = impulseFromAbi(c.longitudinal),
            .across = impulseFromAbi(c.across),
        };
    }
    const forces: integrate.Forces = .{ .accel = f.accel, .ang_accel = f.ang_accel, .tyre_inertia = f.tyre_inertia, .contacts = contacts[0..n] };
    st.* = sim.toAbi(integrate.step(.{ .mass = b.mass, .inertia = b.inertia }, sim.fromAbi(st.*), forces, dt));
}

var car: Car = sim.default_car; // the car the functions below use
var loaded = false; // car came from a file

// a car file (sim/data/cars, import/d3.py); false keeps the car
export fn d3_load_car(path: [*:0]const u8) bool {
    const c = zon.load(Car, allocator, std.mem.span(path)) catch return false;
    if (loaded) std.zon.parse.free(allocator, car);
    car = c;
    loaded = true;
    return true;
}

export fn sim_longitudinal_step(fit: *const longitudinal.Fit, speed: f32, input: *const Input, dt: f32) f32 {
    return longitudinal.step(longitudinal.of(car, fit.*), input.*, speed, dt);
}

export fn world_create() ?*World {
    const w = allocator.create(World) catch return null;
    w.* = .{};
    return w;
}

export fn world_destroy(w: *World) void {
    w.deinit();
    allocator.destroy(w);
}

export fn world_set_planes(w: *World, planes: *const [4]ground.Plane) void {
    w.planes = planes.*;
}

// the planes' material: the one material of a file
export fn world_set_plane_material(w: *World, path: [*:0]const u8) bool {
    const m = zon.load(Material, allocator, std.mem.span(path)) catch return false;
    w.plane_material = m;
    return true;
}

export fn world_load_track(w: *World, jpk_path: [*:0]const u8, materials_path: [*:0]const u8) bool {
    w.load(jpk_path, materials_path, .d3) catch return false;
    return true;
}

export fn ground_cast(w: *World, wheel: u32, from: *const vec.V3, to: *const vec.V3, out: *ground.Hit) bool {
    out.* = w.surface().cast(wheel, from.*, to.*) orelse return false;
    return true;
}

// a Hit's or Contact's material code; Material has Zig's own layout
export fn ground_material_code(m: *const material.Material, out: *[4]u8) void {
    out.* = m.code();
}

export fn ground_query(w: *World, wheel: u32, probe: *const ground.Probe, time: f32, speed: f32, out: *ground.Contact) void {
    out.* = ground.query(car_.groundWheel(car, wheel), car.bumps.?, w.surface(), wheel, probe.*, time, speed);
}

export fn force_step(s: *force.State, input: *const Input, w: *World, dt: f32) void {
    force.step(car, s, input.*, w.surface(), dt);
}

export fn force_draw_wheels(s: *force.State) void {
    force.drawWheels(car, s);
}

export fn chassis_step(s: *force.State, w: *World, dt: f32) void {
    w.collide(car, &s.body, dt);
}

pub const Contact3 = extern struct { point: [3]f32, normal: [3]f32, depth: f32 };

export fn chassis_contacts(s: *const force.State, w: *World, out: [*]Contact3, max: u32) u32 {
    const t = if (w.track) |*t| t else return 0;
    var found: [256]chassis.Contact = undefined;
    const contacts = chassis.collide(car.hull.?, s.body, t, &found);
    const n = @min(contacts.len, max);
    for (contacts[0..n], out[0..n]) |c, *o| o.* = .{ .point = c.point, .normal = c.normal, .depth = c.depth };
    return @intCast(n);
}

export fn force_anchor(s: *force.State, dt: f32) void {
    force.anchor(car, s, dt);
}

export fn force_steer(wheels: *[4]force.Wheel, steer: f32, speed: f32, dt: f32) void {
    steering.update(car, wheels, steer, speed, dt);
}

export fn force_aero(s: *body.State, wheels: *const [4]force.Wheel, dt: f32) void {
    aero.apply(car, s, wheels.*, dt);
}

export fn force_react(s: *body.State, e: *force.Engine, wheels: *const [4]force.Wheel) void {
    engine.react(car, s, e, wheels.*);
}

export fn force_engine_torque(rate: f32, throttle: f32, rolling_back: bool) f32 {
    return engine.torque(car, rate, throttle, rolling_back);
}

export fn force_setup(s: *force.State, moved: *const body.State, input: *const Input, w: *World, speed: f32, h: f32, dt: f32, out: *force.Step) void {
    out.* = force.setup(car, s, moved.*, input.*, w.surface(), speed, h, dt);
}

export fn force_slowdown(s: *force.State, surfaces: *const [4]material.Slowdown, arms: *const [4]vec.V3, speed: f32, dt: f32) void {
    slowdown.apply(car, s, surfaces.*, arms.*, speed, dt);
}

export fn force_rows(s: *const body.State, st: *force.Step) void {
    force.couple(car, s.*, &st.wheels);
}

export fn force_substep(st: *force.Step, h: f32, frac: f32) void {
    force.substep(car, st, h, frac);
}

export fn d3_sin(x: f32) f32 {
    return d3math.sin(x);
}

export fn d3_cos(x: f32) f32 {
    return d3math.cos(x);
}

export fn d3_pow(x: f32, y: f32) f32 {
    return d3math.pow(x, y);
}

export fn d3_rotation(axis: *const [3]f32, angle: f32, out: *[3][3]f32) void {
    const m = vec.rotation(axis.*, angle);
    for (out, m) |*o, row| o.* = row;
}
