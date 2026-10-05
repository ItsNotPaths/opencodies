// The car's body in the physics world after the car step (dirt-rally.md §15): the world moves it by its own
// integration, the hull meets the track (shared/chassis.zig), and the next pre_update takes the pose back
const body = @import("body.zig");
const car = @import("car.zig");
const chassis = @import("../shared/chassis.zig");
const shared = @import("../shared/body.zig");
const vec = @import("../shared/vec.zig");
const Car = @import("../car.zig").Car;
const Mesh = @import("../shared/mesh.zig").Mesh;

const V3 = vec.V3;

// start: the pose pre_update gave the car (the world body's) before car.frame; s: the car after car.frame
pub fn step(c: Car, s: *car.State, start: [4][4]f32, track: *const Mesh, dt: f32) void {
    var p: body.WorldPose = .{ .rows = start, .vel = s.world_body.vel, .angvel = s.world_body.angvel };
    body.integrateInWorld(&p, dt);
    if (c.hull) |hull| collide(c.rigid(), hull, &p, track, dt);
    s.body.rows = p.rows;
    s.world_body = .{ .vel = p.vel, .angvel = p.angvel, .com = p.rows[3] };
    s.side.body = s.world_body;
}

// the island solve on the integrated pose: contacts, velocities, the position pass
pub fn collide(rigid: shared.Body, hull: chassis.Hull, p: *body.WorldPose, track: *const Mesh, dt: f32) void {
    var b: shared.State = .{ .pos = xyz(p.rows[3]), .axes = .{ xyz(p.rows[0]), xyz(p.rows[1]), xyz(p.rows[2]) }, .vel = xyz(p.vel), .angvel = xyz(p.angvel) };
    chassis.step(rigid, hull, &b, track, dt);
    p.* = .{ .rows = .{ lanes(b.axes[0], 0), lanes(b.axes[1], 0), lanes(b.axes[2], 0), lanes(b.pos, 1) }, .vel = lanes(b.vel, 0), .angvel = lanes(b.angvel, 0) };
}

fn xyz(v: [4]f32) V3 {
    return .{ v[0], v[1], v[2] };
}

fn lanes(v: V3, w: f32) [4]f32 {
    return .{ v[0], v[1], v[2], w };
}
