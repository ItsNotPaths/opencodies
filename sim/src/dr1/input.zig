// the ABI Input as the DR1 driver controls (CAR_CONTROLS), written before each updateAll
const std = @import("std");
const sim = @import("../sim.zig");
const car = @import("car.zig");
const controls = @import("controls.zig");

const neutral: i32 = 9; // the DR1 request for neutral; 0 asks for nothing

// the outputs (adjusted steering, auto and final clutch) and the gear up/down flags stay the step's
pub fn apply(s: *car.State, in: sim.Input) void {
    controls.setFlag(s, .ignition, false); // a stalled engine starts on the throttle (controls.update)
    controls.set(s, .throttle, in.throttle);
    controls.set(s, .brake, in.brake);
    controls.set(s, .steering, in.steer); // both games: + turns left
    controls.set(s, .manual_clutch, in.clutch);
    controls.set(s, .handbrake, if (in.handbrake) 1 else 0);
    s.controls[@intFromEnum(controls.Control.gear)] = @bitCast(request(s.transmission, in.gear));
}

// the ABI gear (1..top, 10 reverse, else neutral) as a DR1 request; none while the box is in it
fn request(t: car.Transmission, gear: i32) i32 {
    const want: i32 = if (gear == 10 or gear >= 1 and gear <= t.top_gear) gear else 0;
    if (want == t.gear) return 0;
    return if (want == 0) neutral else want;
}

test "the ABI gear as a DR1 request" {
    var t = std.mem.zeroes(car.Transmission);
    t.top_gear = 5;
    t.gear = 2;
    try std.testing.expectEqual(@as(i32, 0), request(t, 2));
    try std.testing.expectEqual(@as(i32, 3), request(t, 3));
    try std.testing.expectEqual(@as(i32, 10), request(t, 10));
    try std.testing.expectEqual(neutral, request(t, 0));
    try std.testing.expectEqual(neutral, request(t, 7));
    t.gear = 0;
    try std.testing.expectEqual(@as(i32, 0), request(t, 0));
}
