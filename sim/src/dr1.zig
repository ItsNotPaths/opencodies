// The DR1-model parts that tests/dr1.zig checks
pub const sim = @import("sim.zig");
pub const spec = @import("dr1/spec.zig");
pub const tyre = @import("dr1/tyre.zig");
pub const softness = @import("dr1/softness.zig");
pub const chassis = @import("shared/chassis.zig");
pub const body = @import("dr1/body.zig");
pub const world = @import("dr1/world.zig");
pub const mesh = @import("shared/mesh.zig");
pub const material = @import("material.zig");
pub const zon = @import("zon.zig");
pub const Car = @import("car.zig").Car;
