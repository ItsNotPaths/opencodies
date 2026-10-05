// C exports of single FH functions, for tools/fh checks against the recompiled game.
const tyre = @import("tyre.zig");

export fn fh_tyre_table(c: *const tyre.Curves, out: *tyre.Table) void {
    out.* = tyre.table(c.*);
}

export fn fh_tyre_friction(lat: *const tyre.Table, long: *const tyre.Table, rear: bool, handbrake: bool, rear_slide: ?*const tyre.RearSlide, s: *const tyre.Surface, g: *const tyre.Grip, load: f32, v_long: f32, v_lat: f32, v_slip: f32, steer: f32, handbrake_input: f32, speed: f32, out: *tyre.Out) void {
    const w: tyre.Wheel = .{ .rear = rear, .handbrake = handbrake, .rear_slide = if (rear_slide) |r| r.* else null };
    out.* = tyre.friction(lat.*, long.*, w, s.*, g.*, load, v_long, v_lat, v_slip, steer, handbrake_input, speed);
}
