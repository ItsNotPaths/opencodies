// the fork parts in the C ABI: not game correct (fork/)
const cars = @import("cars.zig");
const Sim = @import("../sim.zig").Sim;

const max_cars = cars.max_cars;

// after each car's sim_step: D3 cars with a hull push each other apart; other cars take no part
pub export fn sim_collide_cars(sims: [*]const *Sim, count: u32, dt: f32) void {
    var list: [max_cars]cars.Car = undefined;
    var n: usize = 0;
    for (sims[0..@min(count, max_cars)]) |s| {
        const d = switch (s.game) {
            .d3 => |*d| d,
            .dr1, .fh => continue,
        };
        const hull = s.car.hull orelse continue;
        list[n] = .{ .body = s.car.rigid(), .hull = hull, .state = &d.body };
        n += 1;
    }
    var found: [cars.max_pairs]cars.Pair = undefined;
    const pairs = cars.collide(list[0..n], &found);
    cars.solve(list[0..n], pairs, dt);
}
