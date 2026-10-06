// the fork parts in the C ABI: not game correct (fork/)
const cars = @import("cars.zig");
const Sim = @import("../sim.zig").Sim;

const max_cars = cars.max_cars;

var hits: [cars.max_hits]cars.Hit = undefined; // of the last sim_collide_cars
var hit_count: usize = 0;

// after each car's sim_step: D3 cars with a hull push each other apart; other cars take no part
pub export fn sim_collide_cars(sims: [*]const *Sim, count: u32, dt: f32) void {
    var list: [max_cars]cars.Car = undefined;
    var index: [max_cars]u32 = undefined; // into sims
    var n: usize = 0;
    for (sims[0..@min(count, max_cars)], 0..) |s, i| {
        const d = switch (s.game) {
            .d3 => |*d| d,
            .dr1, .fh => continue,
        };
        const hull = s.car.hull orelse continue;
        list[n] = .{ .body = s.car.rigid(), .hull = hull, .state = &d.body };
        index[n] = @intCast(i);
        n += 1;
    }
    var found: [cars.max_pairs]cars.Pair = undefined;
    const pairs = cars.collide(list[0..n], &found, dt);
    const out = cars.solve(list[0..n], pairs, dt, &hits);
    for (out) |*h| {
        h.a = index[h.a];
        h.b = index[h.b];
    }
    hit_count = out.len;
}

// the hits of the last sim_collide_cars, at most max; a < b index its sims
pub export fn sim_hits(out: [*]cars.Hit, max: u32) u32 {
    const n = @min(hit_count, max);
    @memcpy(out[0..n], hits[0..n]);
    return @intCast(n);
}
