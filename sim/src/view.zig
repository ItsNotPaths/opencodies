const std = @import("std");
const sim = @import("sim.zig");
const vec = @import("shared/vec.zig");
const body = @import("shared/body.zig");
const force = @import("d3/force.zig");
const suspension = @import("d3/suspension.zig");
const mesh = @import("shared/mesh.zig");
const zon = @import("zon.zig");
const material = @import("material.zig");
const Material = material.Material;
const dr1_car = @import("dr1/car.zig");
const dr1_ground = @import("dr1/ground.zig");
const fh_car = @import("fh/car.zig");
const Mesh = mesh.Mesh;

const Sim = sim.Sim;
const V3 = vec.V3;
const splat = vec.splat;
const dot = vec.dot;
const allocator = sim.allocator;

pub const WheelView = extern struct {
    centre: [3]f32, // body frame, hub at the current travel
    radius: f32,
    steer: f32, // rad about body up
    travel: f32, // m
    spin: f32, // rad/s, > 0 rolls forward
    load: f32, // N, spring force
    slip_angle: f32, // rad
    slip_ratio: f32,
    grounded: bool,
    surface: [4]u8, // material under the hub, "----" off the mesh
};

pub const CarView = extern struct { engine_rpm: f32, wheels: [4]WheelView };

// a plain triangle list as a level, materials by code from a list of Material (sim/data/materials); no sea level (0)
export fn sim_load_mesh(s: *Sim, corners: [*]const [3][3]f32, codes: [*]const [4]u8, count: u32, materials_path: [*:0]const u8) bool {
    s.world.setTrack(loadMesh(corners[0..count], codes[0..count], materials_path) catch return false) catch return false;
    return s.fitLevel();
}

fn loadMesh(corners: []const [3][3]f32, codes: []const [4]u8, materials_path: [*:0]const u8) !Mesh {
    const materials = try zon.load([]Material, allocator, std.mem.span(materials_path));
    defer allocator.free(materials);
    if (materials.len == 0) return error.NoMaterials;
    const tris = try allocator.alloc(mesh.Triangle, corners.len);
    defer allocator.free(tris);
    for (tris, corners, codes) |*t, c, code| t.* = .fromCorners(.{ c[0], c[1], c[2] }, @intCast(material.find(materials, code) orelse 0));
    return Mesh.init(allocator, tris, materials);
}

export fn view_track_triangle_count(s: *const Sim) u32 {
    const t = s.world.track orelse return 0;
    return @intCast(t.triangles.len);
}

// returns the count written
export fn view_track_triangles(s: *const Sim, corners: [*][3][3]f32, codes: [*][4]u8, max: u32) u32 {
    const t = s.world.track orelse return 0;
    const n = @min(max, t.triangles.len);
    for (t.triangles[0..n], corners[0..n], codes[0..n]) |tri, *out, *code| {
        for (out, tri.corners) |*o, v| o.* = v;
        code.* = t.materials[tri.material].code();
    }
    return @intCast(n);
}

export fn view_car(s: *Sim, out: *CarView) void {
    switch (s.game) {
        .d3 => |*st| viewD3(s, st.*, out),
        .dr1 => |*d| out.* = viewDr1(d.state, if (s.world.track) |*t| t else null),
        .fh => |*f| out.* = viewFh(f.params, f.state),
    }
}

// an FH car's top gear; -1 for the others
export fn view_top_gear(s: *const Sim) i32 {
    return switch (s.game) {
        .fh => |*f| @intCast(f.params.gearbox.top),
        else => -1,
    };
}

// the DR1 or FH gearbox's gear: 1..top, 10 reverse, 0 neutral. -1 for D3: its gear is the input's
export fn view_gear(s: *const Sim) i32 {
    return switch (s.game) {
        .dr1 => |*d| d.state.transmission.gear,
        .fh => |*f| @intCast(f.state.box.gear),
        .d3 => -1,
    };
}

fn rpm(rate: f32) f32 {
    return rate * 60 / (2 * std.math.pi);
}

fn viewD3(s: *Sim, st: force.State, out: *CarView) void {
    const b = st.body;
    out.engine_rpm = rpm(st.engine.rate);
    for (&out.wheels, st.wheels, s.car.wheels) |*v, w, p| {
        const geo = suspension.geometry(b, p, w.steer);
        const contact = suspension.contactPoint(geo, w.travel);
        const speed = body.pointVelocity(b, contact);
        const lon, const lat = .{ dot(speed, geo.lon), dot(speed, geo.lat) };
        const rolling = @abs(lon) > 0.5;
        v.* = .{
            .centre = p.rest_position + p.spring_axis * splat(w.travel),
            .radius = p.radius,
            .steer = std.math.asin(w.steer),
            .travel = w.travel,
            .spin = w.spin,
            .load = @max(0, p.spring.linear.rate * (w.travel - p.spring.linear.preload)),
            .slip_angle = if (rolling) std.math.atan2(lat, @abs(lon)) else 0,
            .slip_ratio = (w.spin * p.radius - lon) / @max(@abs(lon), 1),
            .grounded = w.contact,
            .surface = surfaceUnder(s, contact, b.axes[1]),
        };
    }
}

fn viewDr1(st: dr1_car.State, track: ?*const Mesh) CarView {
    var out: CarView = .{ .engine_rpm = rpm(st.engine.rate), .wheels = undefined };
    for (&out.wheels, st.wheels, st.data) |*v, w, d| {
        const V4 = @Vector(4, f32);
        const hub = @as(V4, w.rest_position) + @as(V4, d.spring_axis) * @as(V4, @splat(w.height)); // the drawn hub (wheel_transform)
        v.* = .{
            .centre = .{ hub[0], hub[1], hub[2] },
            .radius = w.radius,
            .steer = std.math.atan2(w.direction[1], w.direction[0]), // {z, x}
            .travel = w.height,
            .spin = w.rotation_rate,
            .load = w.upward_force[0],
            .slip_angle = w.tyre.slip.angle,
            .slip_ratio = w.tyre.slip.ratio,
            .grounded = w.contact,
            .surface = if (w.contact_surface == dr1_ground.none) "----".* else track.?.materials[w.contact_surface].code(),
        };
    }
    return out;
}

fn viewFh(p: fh_car.Params, st: fh_car.State) CarView {
    var out: CarView = .{ .engine_rpm = rpm(st.engine), .wheels = undefined };
    for (&out.wheels, st.wheels, p.wheels) |*v, w, pw| {
        const o = w.out;
        v.* = .{
            .centre = .{ pw.suspension.hub[0], w.travel.y, pw.suspension.hub[2] },
            .radius = pw.suspension.radius,
            .steer = if (pw.steered) st.steer.angle else 0,
            .travel = w.travel.y - pw.suspension.hub[1],
            .spin = w.spin,
            .load = w.load,
            .slip_angle = if (o) |x| x.slip_angle else 0,
            .slip_ratio = if (o) |x| x.slip_ratio else 0,
            .grounded = w.travel.contact,
            .surface = if (w.material) |m| m.code() else "----".*,
        };
    }
    return out;
}

// a ray along body up through the contact
fn surfaceUnder(s: *Sim, at: V3, up: V3) [4]u8 {
    const t = s.world.track orelse return "TSD*".*;
    const hit = t.raycast(at + up * splat(0.5), at - up * splat(0.5)) orelse return "----".*;
    return hit.material.code();
}
