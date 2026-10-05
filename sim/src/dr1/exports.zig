// the DR1 model function by function, for tools/dr1/fndiff.py
const std = @import("std");
const aero = @import("aero.zig");
const car_ = @import("car.zig");
const controls = @import("controls.zig");
const body = @import("body.zig");
const drivetrain = @import("drivetrain.zig");
const engine = @import("engine.zig");
const ground = @import("ground.zig");
const mesh = @import("../shared/mesh.zig");
const sim_root = @import("../sim.zig");
const softness = @import("softness.zig");
const world = @import("world.zig");
const chassis = @import("../shared/chassis.zig");
const spec = @import("spec.zig");
const suspension = @import("suspension.zig");
const tyre = @import("tyre.zig");
const wheel = @import("wheel.zig");
const wiper = @import("wiper.zig");
const Material = @import("../material.zig").Material;

export fn dr1_tyre_load_graph(g: *const tyre.Graph, exponent: f32, z: f32, out: *tyre.LoadGraph) void {
    out.* = tyre.loadGraph(g.*, exponent, z);
}

export fn dr1_tyre_sample(g: *const tyre.LoadGraph, x: f32) f32 {
    return tyre.sample(g.*, x);
}

export fn dr1_tyre_ellipse(set: *const tyre.GraphSet, surface: u32, slip: *const tyre.Slip, z: f32, camber: f32, out: *tyre.Forces) void {
    out.* = tyre.ellipse(set.*, surface, slip.*, z, camber);
}

export fn dr1_tyre_similarity(set: *const tyre.GraphSet, surface: u32, slip: *const tyre.Slip, legacy: bool, z: f32, camber: f32, out: *tyre.Forces) void {
    out.* = tyre.similarity(set.*, surface, slip.*, legacy, z, camber);
}

export fn dr1_tyre_set_slip_ratio(t: *tyre.Tyre, wheel_forward: f32, patch_forward: f32, patch_side: f32) void {
    t.setSlipRatio(wheel_forward, patch_forward, patch_side);
}

export fn dr1_tyre_set_slip_angle(t: *tyre.Tyre, wheel_forward: f32, wheel_side: f32) void {
    t.setSlipAngle(wheel_forward, wheel_side);
}

export fn dr1_tyre_peak_slip_ratio(t: *const tyre.Tyre, load: f32) f32 {
    return t.peakSlipRatio(load);
}

export fn dr1_tyre_peak_slip_angle(t: *const tyre.Tyre, load: f32) f32 {
    return t.peakSlipAngle(load);
}

export fn dr1_tyre_skid_slip_ratio(t: *const tyre.Tyre, min_speed: f32) f32 {
    return t.skidSlipRatio(min_speed);
}

export fn dr1_tyre_skid_slip_angle(t: *const tyre.Tyre, min_speed: f32) f32 {
    return t.skidSlipAngle(min_speed);
}

export fn dr1_tyre_set_skidding(t: *tyre.Tyre, wheel_forward: f32, patch_forward_rel: f32, load: f32) void {
    t.setSkidding(wheel_forward, patch_forward_rel, load);
}

export fn dr1_tyre_slip_direction(wheel_forward: f32, patch_forward_rel: f32, patch_side_rel: f32, out: *[2]f32) void {
    out.* = tyre.slipDirection(wheel_forward, patch_forward_rel, patch_side_rel);
}

export fn dr1_point_velocity(b: *const body.Body, p: *const [4]f32, out: *[4]f32) void {
    out.* = body.pointVelocity(b.*, p.*);
}

export fn dr1_update_abs(abs: *suspension.Abs, a: *const suspension.Assists, rotating_forwards: bool, speed_proportional: bool, forward_force: f32, abs_cap: f32, t: *const tyre.Tyre, load_multiple: f32, handbrake_wheel: bool, braking_torque: f32) void {
    suspension.updateAbs(abs, a.*, rotating_forwards, speed_proportional, forward_force, abs_cap, t.*, load_multiple, handbrake_wheel, braking_torque);
}

export fn dr1_gravel(sim: *const suspension.Surface, w: *const suspension.Sinking, t: *const tyre.Tyre, ground_impulse: f32, out: *suspension.SurfaceForce) void {
    out.* = suspension.gravel(sim.*, w.*, t.*, ground_impulse);
}

export fn dr1_tyre_impulses(sys: *suspension.System, car: *const suspension.Car, data: *[4]suspension.WheelData, wheels: *[4]suspension.Wheel, dt: f32, ground_impulse: *[4]f32, grip: *const [4]f32, out: *suspension.Impulses) void {
    out.* = suspension.tyreImpulses(sys, car.*, data, wheels, dt, ground_impulse, grip.*);
}

export fn dr1_ground_impulses(sys: *const suspension.System, mass: f32, data: *[4]suspension.WheelData, wheels: *const [4]suspension.Wheel, dt: f32, out: *[4]f32) void {
    out.* = suspension.groundImpulses(sys.*, mass, data, wheels.*, dt);
}

export fn dr1_load_impulses(sys: *const suspension.System, springs: *const suspension.Springs, data: *const [4]suspension.WheelData, wheels: *const [4]suspension.Wheel, dt: f32, out: *[4]f32) void {
    suspension.loadImpulses(sys.*, springs.*, data.*, wheels.*, dt, out);
}

export fn dr1_engine_update(e: *engine.Engine, car: *const engine.Motion, dt: f32, clutch: f32, external_pressure: f32) f32 {
    return engine.update(e, car.*, dt, clutch, external_pressure);
}

export fn dr1_wheel_position(sys: *const suspension.System, b: *const body.RigidBody, d: *suspension.WheelData, w: *const suspension.Wheel) void {
    suspension.wheelPosition(sys.*, b.*, d, w.*);
}

export fn dr1_force_response_tables(sys: *suspension.System, b: *const body.RigidBody, data: *[4]suspension.WheelData, wheels: *const [4]suspension.Wheel) void {
    suspension.forceResponseTables(sys, b.*, data, wheels.*);
}

export fn dr1_drivetrain_integrate(d: *drivetrain.Drivetrain, car: *drivetrain.CarSide, engine_torque: f32, h: f32, copy: bool) void {
    drivetrain.integrate(d, car, engine_torque, h, copy);
}

export fn dr1_rigid_integrate(b: *body.RigidBody, dt: f32) void {
    body.integrate(b, dt);
}

export fn dr1_sin_cos(x: f32, out: *[2]f32) void {
    out.* = body.sinCos(x);
}

export fn dr1_aero(a: *const aero.Aero, b: *body.RigidBody) void {
    aero.update(a.*, b);
}

export fn dr1_copy_state_back(sys: *const suspension.System, b: *body.RigidBody, data: *[4]suspension.WheelData, wheels: *[4]suspension.Wheel, roll_centre: *const [2][3]f32, dt: f32) void {
    suspension.copyStateBack(sys.*, b, data, wheels, roll_centre.*, dt);
}

export fn dr1_sub_setup(sys: *suspension.System, b: *body.RigidBody, start: *const suspension.Frame, data: *[4]suspension.WheelData, wheels: *[4]suspension.Wheel, dt: f32, g: *const ground.Ground) void {
    suspension.subSetup(sys, b, start.*, data, wheels, dt, g.*);
}

export fn dr1_setup_for_frame(sys: *suspension.System, b: *body.RigidBody, frames: *const [2]suspension.Frame, tyre_burst: *const [4]bool, data: *[4]suspension.WheelData, wheels: *[4]suspension.Wheel, dt: f32, g: *const ground.Ground) void {
    suspension.setupForFrame(sys, b, frames[0], frames[1], tyre_burst.*, data, wheels, dt, g.*);
}

export fn dr1_sub_update(sys: *suspension.System, car: *const suspension.Car, springs: *const suspension.Springs, traction: *const suspension.Traction, data: *[4]suspension.WheelData, wheels: *[4]suspension.Wheel, d: *drivetrain.Drivetrain, side: *drivetrain.CarSide, dt: f32, grip: *const [4]f32, engine_torque: f32) void {
    suspension.subUpdate(sys, car.*, springs.*, traction.*, data, wheels, d, side, dt, grip.*, engine_torque);
}

export fn dr1_update(sys: *suspension.System, v: *const suspension.Vehicle, data: *[4]suspension.WheelData, wheels: *[4]suspension.Wheel, dt: f32, g: *const ground.Ground) void {
    suspension.update(sys, v.*, data, wheels, dt, g.*);
}

export fn dr1_height_checks(g: *const ground.Ground, sys: *suspension.System, b: *const body.RigidBody, data: *[4]suspension.WheelData, wheels: *[4]suspension.Wheel, end_frame: bool, dt: f32) void {
    ground.doHeightChecks(g.*, sys, b.*, data, wheels, end_frame, dt);
}

export fn dr1_precalculate(wheels: *const [4]suspension.Wheel, n: u32, rows: *const [4][4]f32, out: *wheel.Solver) void {
    wheel.precalculate(out, wheels[0..n], rows.*);
}

export fn dr1_patch_points(w: *suspension.Wheel, s: *const wheel.Solver, i: u32, position: *const [4]f32, dt: f32) void {
    wheel.patchPoints(w, s.*, i, position.*, dt);
}

export fn dr1_temperature(w: *suspension.Wheel, h: *const wheel.Heat, rows: *const [4][4]f32, dt: f32) void {
    wheel.temperature(w, h.*, rows.*, dt);
}

// contact: wear multiple, dirt buildup (as 0 or 1); null off the ground
export fn dr1_suspension_new(w: *suspension.Wheel, contact: ?*const [2]f32, wear: *const wheel.Wear, speed: f32, h: *const wheel.Heat, rows: *const [4][4]f32, dt: f32, wear_rate: f32) bool {
    const c: ?wheel.Contact = if (contact) |x| .{ .wear_multiple = x[0], .dirt_buildup = x[1] != 0 } else null;
    return wheel.suspensionNew(w, c, wear.*, speed, h.*, rows.*, dt, wear_rate);
}

export fn dr1_brake_temperatures(wheels: *[4]suspension.Wheel, n: u32, inverse_mass: f32, speed: f32, brake: f32, dt: f32) void {
    wheel.brakeTemperatures(wheels[0..n], inverse_mass, speed, brake, dt);
}

export fn dr1_wiper(w: *wiper.Wiper, random: *u32, dt: f32, speed: f32) void {
    wiper.update(w, random, dt, speed);
}

export fn dr1_update_all(s: *car_.State, dt: f32, g: *const ground.Ground) void {
    car_.updateAll(s, dt, g.*);
}

export fn dr1_frame(s: *car_.State, dt: f32, g: *const ground.Ground) void {
    car_.frame(s, dt, g.*);
}

export fn dr1_orthonormal(rows: *[4][4]f32) void {
    rows.* = body.orthonormal(rows.*);
}

// the parts of updateAll the game calls as functions: 0 updateControls, 1 updateSteeringSystem, 2 updateAutoClutch,
// 3 updateRevMatching, 4 updateGearbox, 5 updateAutoGearbox (arg: advise), 6 updateGearChange (arg: the gear),
// 7 updateSuspensionAndIntegration, 8 updateSurfaceEffects
export fn dr1_car_part(which: u32, s: *car_.State, dt: f32, g: *const ground.Ground, arg: i32) void {
    switch (which) {
        0 => controls.update(s, dt),
        1 => controls.steeringSystem(s, dt),
        2 => controls.autoClutch(s, dt),
        3 => controls.revMatching(s),
        4 => controls.gearbox(s, dt),
        5 => controls.autoGearbox(s, dt, arg != 0),
        6 => controls.gearChange(s, arg),
        7 => car_.suspensionAndIntegration(s, dt, g.*),
        8 => car_.surfaceEffects(s, dt, g.*),
        else => {},
    }
    car_.syncSide(s);
}

export fn dr1_slow_down(s: *car_.State, dt: f32, blend: f32, params: *const [4]ground.Slowdown, has: *const [4]bool) void {
    var p: car_.Slowdowns = undefined;
    for (&p, params, has) |*x, q, h| x.* = if (h) q else null;
    car_.slowDown(s, dt, blend, p);
    car_.syncSide(s);
}

// the track a Sim loaded (sim_load_dr1_track), and its material codes
export fn dr1_track(s: *const sim_root.Sim) ?*const mesh.Mesh {
    return if (s.world.track) |*t| t else null;
}

export fn dr1_track_codes(t: *const mesh.Mesh, out: [*][4]u8, cap: u32) u32 {
    const n = @min(t.materials.len, cap);
    for (t.materials[0..n], out[0..n]) |m, *c| c.* = m.code();
    return @intCast(t.materials.len);
}

export fn dr1_gather(t: *const mesh.Mesh, lo: *const [4]f32, hi: *const [4]f32, out: *[100]ground.Triangle) u32 {
    var c: ground.Cache = .{};
    ground.gather(t, lo.*, hi.*, &c);
    @memcpy(out[0..c.len], c.items());
    return @intCast(c.len);
}

export fn dr1_ray_cast(tris: [*]const ground.Triangle, n: u32, start: *const [4]f32, end: *const [4]f32, out: *ground.Hit) bool {
    out.* = ground.rayCast(tris[0..n], start.*, end.*) orelse return false;
    return true;
}

export fn dr1_box_overlaps(center: *const [4]f32, half: *const [4]f32, tri: *const [3][4]f32) bool {
    return ground.boxOverlaps(center.*, half.*, .{ tri[0], tri[1], tri[2] });
}

// size: Wheel radius, max_radius, width, critical_cos
export fn dr1_ellipsoid(tris: [*]const ground.Triangle, n: u32, materials: [*]const ground.Material, size: *const [4]f32, center: *const [4]f32, side: *const [4]f32, axis: *const [4]f32, out: *ground.Push) void {
    var w = std.mem.zeroes(suspension.Wheel);
    w.radius, w.max_radius, w.width, w.critical_cos = size.*;
    out.* = ground.ellipsoid(tris[0..n], materials, w, center.*, side.*, axis.*);
}

export fn dr1_bump_offset(m: *const ground.Material, at: *const [4]f32, scale: f32) f32 {
    return ground.bumpOffset(m.*, at.*, scale);
}

export fn dr1_velocity_at(b: *const body.RigidBody, r: *const [4]f32, out: *[4]f32) void {
    out.* = ground.velocityAt(b.*, r.*);
}

export fn dr1_set_active_surface(t: *tyre.Tyre, target: *tyre.GraphSet, rate: *tyre.GraphSet, compound: [*]const tyre.SubTyre, n: u32, surface: u32) void {
    tyre.setActiveSurface(t, target, rate, compound[0..n], surface);
}

// spec.zig for tools/dr1/specdiff.py: init(specOf(s)) at s's pose, sea level and damage
export fn dr1_spec_round_trip(s: *const car_.State, g: *const ground.Ground, out: *car_.State) void {
    out.* = spec.init(spec.specOf(s.*, g.*), s.body.rows, s.sea_level_raw, spec.damageOf(s.*));
}

// specOf(s) and the materials as the car and material files
export fn dr1_spec_save(s: *const car_.State, g: *const ground.Ground, materials: [*]const ground.Material, n: u32, car_path: [*:0]const u8, materials_path: [*:0]const u8) bool {
    spec.save(std.mem.span(car_path), spec.specOf(s.*, g.*)) catch return false;
    const neutral = sim_root.allocator.alloc(Material, n) catch return false;
    defer sim_root.allocator.free(neutral);
    for (neutral, materials[0..n]) |*m, x| m.* = spec.materialOf(x);
    spec.save(std.mem.span(materials_path), neutral) catch return false;
    return true;
}

// all of a materials file as the step's materials (spec.groundMaterial); returns the count, writes up to cap
export fn dr1_spec_load_materials(path: [*:0]const u8, out: [*]ground.Material, cap: u32) u32 {
    const gpa = sim_root.allocator;
    const all = spec.load([]Material, gpa, std.mem.span(path)) catch return 0;
    defer std.zon.parse.free(gpa, all);
    const n = @min(all.len, cap);
    for (all[0..n], out[0..n]) |m, *g| g.* = spec.groundMaterial(m);
    return @intCast(all.len);
}

// the car and the track's materials from the files, kept until the next load
var loaded: ?struct { car: @import("../car.zig").Car, materials: []ground.Material } = null;

export fn dr1_spec_load(car_path: [*:0]const u8, materials_path: [*:0]const u8, track: *const mesh.Mesh) bool {
    const gpa = sim_root.allocator;
    if (loaded) |l| {
        std.zon.parse.free(gpa, l.car);
        gpa.free(l.materials);
        loaded = null;
    }
    const c = spec.load(@import("../car.zig").Car, gpa, std.mem.span(car_path)) catch return false;
    const all = spec.load([]Material, gpa, std.mem.span(materials_path)) catch return false;
    defer std.zon.parse.free(gpa, all);
    loaded = .{ .car = c, .materials = spec.trackMaterials(gpa, track, all) catch return false };
    return true;
}

export fn dr1_spec_init(from: *const car_.State, out: *car_.State) void {
    out.* = spec.init(loaded.?.car, from.body.rows, from.sea_level_raw, spec.damageOf(from.*));
}

export fn dr1_spec_apply(s: *car_.State) void {
    spec.apply(loaded.?.car, s);
}

export fn dr1_spec_ground(s: *const car_.State, track: *const mesh.Mesh, out: *ground.Ground) void {
    out.* = spec.groundOf(loaded.?.car, s.*, track, loaded.?.materials.ptr);
}

// struct sizes, so the tools' ctypes mirrors can check they match
export fn dr1_size(which: u32) usize {
    return switch (which) {
        0 => @sizeOf(suspension.WheelData),
        1 => @sizeOf(suspension.Wheel),
        2 => @sizeOf(suspension.System),
        3 => @sizeOf(tyre.Tyre),
        4 => @sizeOf(body.RigidBody),
        5 => @sizeOf(drivetrain.Drivetrain),
        6 => @sizeOf(engine.Engine),
        7 => @sizeOf(aero.Aero),
        8 => @sizeOf(suspension.Vehicle),
        9 => @sizeOf(ground.Ground),
        10 => @sizeOf(ground.Material),
        11 => @sizeOf(ground.Triangle),
        12 => @sizeOf(ground.Push),
        13 => @sizeOf(car_.State),
        else => 0,
    };
}

// table: the n entries of Hull.impact_softness
export fn dr1_speed_scale(table: [*]const f32, n: u32, speed: f32) f32 {
    return softness.speedScale(table[0..n], speed);
}

// out[0] the impulse per metre; out[1] the directional one, left alone when false is returned
export fn dr1_absorbed(mass: f32, points: [*]const softness.Point, n: u32, cube: f32, at: *const [4]f32, dir: *const [4]f32, reach: f32, out: *[2]f32) bool {
    const a = softness.absorbed(mass, points[0..n], cube, at.*, dir.*, reach);
    out[0] = a.all;
    out[1] = a.directional orelse return false;
    return true;
}

export fn dr1_integrate_in_world(p: *body.WorldPose, dt: f32) void {
    body.integrateInWorld(p, dt);
}

// the physics world after car.frame for the loaded car (dr1_spec_load); start: the pose before car.frame
export fn dr1_world_step(s: *car_.State, start: *const [4][4]f32, track: *const mesh.Mesh, dt: f32) void {
    world.step(loaded.?.car, s, start.*, track, dt);
}

// the loaded car's hull against the track from an integrated world pose; false: the car has no hull
export fn dr1_body_contacts(p: *body.WorldPose, track: *const mesh.Mesh, dt: f32) bool {
    const c = loaded.?.car;
    world.collide(c.rigid(), c.hull orelse return false, p, track, dt);
    return true;
}

// the car file with a hull: n pieces, counts[i] corners each (body frame, xyz), one after the other; the impact
// softness table (impact_n entries, none if 0)
export fn dr1_car_set_hull(car_path: [*:0]const u8, corners: [*]const [3]f32, counts: [*]const u32, n: u32, margin: f32, friction: f32, softness_: f32, impact: [*]const f32, impact_n: u32) bool {
    const gpa = sim_root.allocator;
    var c = spec.load(@import("../car.zig").Car, gpa, std.mem.span(car_path)) catch return false;
    defer std.zon.parse.free(gpa, c);
    var arena: std.heap.ArenaAllocator = .init(gpa);
    defer arena.deinit();
    const a = arena.allocator();
    const pieces = a.alloc(chassis.Piece, n) catch return false;
    var at: usize = 0;
    for (pieces, counts[0..n]) |*p, k| {
        const piece = a.alloc(@import("../shared/vec.zig").V3, k) catch return false;
        for (piece, corners[at..][0..k]) |*v, x| v.* = x;
        p.* = piece;
        at += k;
    }
    const old = c.hull;
    c.hull = .{ .pieces = pieces, .margin = margin, .friction = friction, .softness = softness_, .impact_softness = if (impact_n > 0) impact[0..impact_n] else null };
    defer c.hull = old;
    spec.save(std.mem.span(car_path), c) catch return false;
    return true;
}

pub const BodyContact = extern struct { point: [3]f32, normal: [3]f32, depth: f32, piece: u32 };

// the contacts world.collide would solve, for the tools
export fn dr1_body_contact_list(p: *const body.WorldPose, track: *const mesh.Mesh, out: [*]BodyContact, max: u32) u32 {
    const hull = loaded.?.car.hull orelse return 0;
    const r = p.rows;
    const s: @import("../shared/body.zig").State = .{ .pos = .{ r[3][0], r[3][1], r[3][2] }, .axes = .{ .{ r[0][0], r[0][1], r[0][2] }, .{ r[1][0], r[1][1], r[1][2] }, .{ r[2][0], r[2][1], r[2][2] } }, .vel = @splat(0), .angvel = @splat(0) };
    var found: [256]chassis.Contact = undefined;
    const contacts = chassis.collide(hull, s, track, &found);
    const n = @min(contacts.len, max);
    for (contacts[0..n], out[0..n]) |c, *o| o.* = .{ .point = c.point, .normal = c.normal, .depth = c.depth, .piece = c.piece };
    return @intCast(n);
}
