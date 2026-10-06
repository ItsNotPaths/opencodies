const std = @import("std");
const body = @import("shared/body.zig");
const chassis = @import("shared/chassis.zig");
const ground = @import("shared/ground.zig");
const vec = @import("shared/vec.zig");
const Mesh = @import("shared/mesh.zig").Mesh;
const jpk = @import("shared/jpk.zig");
const surface_xml = @import("shared/surface.zig");
const zon = @import("zon.zig");
const material = @import("material.zig");
const Material = material.Material;
const force = @import("d3/force.zig");
const Car = @import("car.zig").Car;
const dr1_car = @import("dr1/car.zig");
const dr1_ground = @import("dr1/ground.zig");
const dr1_input = @import("dr1/input.zig");
const dr1_world = @import("dr1/world.zig");
const spec = @import("dr1/spec.zig");
const fh_car = @import("fh/car.zig");
const fh_spec = @import("fh/spec.zig");
const fh_gearbox = @import("fh/gearbox.zig");

comptime {
    _ = @import("view.zig"); // render and HUD exports, plain mesh tracks
    _ = @import("d3/exports.zig"); // the D3 model function by function, for the diff tools
    _ = @import("dr1/exports.zig"); // the same for DR1
    _ = @import("fh/exports.zig"); // single FH functions, for the recompiled-game checks
    _ = @import("fork/exports.zig"); // parts with no game behind them (car against car)
}

// sim_create's car: made up, no game's data (sim/data/cars/hatch.zon)
pub const default_car: Car = @import("generic_car");
const splat = vec.splat;

pub const Input = extern struct { steer: f32 = 0, throttle: f32 = 0, brake: f32 = 0, gear: i32 = 0, clutch: f32 = 0, handbrake: bool = false }; // gear 1..5, 10 reverse, else neutral

pub const State = extern struct {
    pos: [3]f32,
    axes: [3][3]f32, // body axes in world: right, up, forward
    vel: [3]f32,
    angvel: [3]f32, // world frame
};

const start: body.State = .{ // wheels just touching flat ground
    .pos = .{ 0, 0.39, 0 },
    .axes = .{ .{ 1, 0, 0 }, .{ 0, 1, 0 }, .{ 0, 0, 1 } },
    .vel = splat(0),
    .angvel = splat(0),
};

// where the wheels find the ground: one plane per wheel until a level is loaded
pub const World = struct {
    planes: [4]ground.Plane = ground.flat,
    plane_material: Material = ground.plain,
    track: ?Mesh = null,
    level: []Material = &.{}, // the level's own materials, by material index
    sea_level: f64 = 0, // m; D3 levels have none: 0

    pub fn surface(w: *World) ground.Ground {
        return if (w.track) |*t| .{ .track = t } else .{ .planes = .{ .each = w.planes, .material = &w.plane_material } };
    }

    // the car's hull against the track, if both exist
    pub fn collide(w: *World, c: Car, b: *body.State, dt: f32) void {
        const t = if (w.track) |*t| t else return;
        if (c.hull) |h| chassis.step(c.rigid(), h, b, t, dt);
    }

    // a D3 level's materials: a list of Material (import/d3.py, sim/data/materials); a DR1 surface_materials.xml gives
    // only the codes: stand-ins until fill
    pub fn load(w: *World, jpk_path: [*:0]const u8, materials_path: [*:0]const u8, packing: jpk.Packing) !void {
        const track = try readFile(jpk_path);
        defer allocator.free(track);
        const materials = switch (packing) {
            .d3 => try zon.load([]Material, allocator, std.mem.span(materials_path)),
            .dr1 => try standIns(materials_path),
        };
        defer allocator.free(materials);
        try w.setTrack(try jpk.load(allocator, track, materials, packing));
    }

    // sea level 0 until the level gives one
    pub fn setTrack(w: *World, t: Mesh) !void {
        var track = t;
        errdefer track.deinit();
        const level = try allocator.dupe(Material, t.materials);
        w.deinit();
        w.track = t;
        w.level = level;
        w.sea_level = 0;
    }

    // the stand-ins take the real material of their code from list
    pub fn fill(w: *World, list: []const Material) void {
        for (w.level) |*m| {
            if (!m.isBare()) continue;
            const i = material.find(list, m.code()) orelse continue;
            if (std.mem.eql(u8, list[i].origin(), m.origin())) m.* = list[i];
        }
    }

    pub fn deinit(w: *World) void {
        if (w.track) |*t| t.deinit();
        allocator.free(w.level);
        w.track = null;
        w.level = &.{};
    }
};

fn standIns(codes_path: [*:0]const u8) ![]Material {
    const bxml = try readFile(codes_path);
    defer allocator.free(bxml);
    const codes = try surface_xml.codes(allocator, bxml);
    defer allocator.free(codes);
    const out = try allocator.alloc(Material, codes.len);
    for (out, codes) |*m, code| m.* = material.bare("dr1", code);
    return out;
}

pub const d3_dt: f32 = 0.04; // s, one D3 vehicle step; force.step runs its substeps
pub const dr1_dt: f32 = 1.0 / 60.0;

pub const Sim = struct {
    car: Car = default_car,
    game: Game = .{ .d3 = .{ .body = start, .engine = .{ .rate = default_car.engine.idle, .mean = default_car.engine.idle } } },
    world: World = .{},
    materials: []const Material = &.{}, // the car's own, to borrow from on another level; sim_create's car has none
    from_files: bool = false, // car and materials; sim_create's car is static

    fn unload(s: *Sim) void {
        switch (s.game) {
            .dr1 => |*d| allocator.free(d.track_materials),
            .d3, .fh => {},
        }
        if (!s.from_files) return;
        allocator.free(s.materials);
        std.zon.parse.free(allocator, s.car);
        s.* = .{ .world = s.world };
    }

    fn own(s: *Sim, c: Car, materials: []Material) void {
        s.unload();
        s.car = c;
        s.materials = materials;
        s.from_files = true;
    }

    // a new level or car: the car's step reads the level's materials, borrowing what they lack (material.borrow),
    // and a DR1 car takes the level's sea level
    pub fn fitLevel(s: *Sim) bool {
        if (s.game == .dr1) setSeaLevel(&s.game.dr1.state, spec.seaLevelRaw(s.world.sea_level));
        const t = if (s.world.track) |*t| t else return true;
        s.world.fill(s.materials);
        var borrowed: std.EnumSet(material.Field) = .initEmpty();
        var counts: std.EnumArray(material.How, u32) = .initFill(0);
        for (t.materials, s.world.level) |*view, m| view.* = s.borrowFor(m, &borrowed, &counts);
        if (borrowed.count() > 0) logBorrowed(@tagName(s.game), s.world.level[0].origin(), borrowed, counts);
        const d = switch (s.game) {
            .dr1 => |*d| d,
            .d3, .fh => return true,
        };
        const by_track = allocator.alloc(dr1_ground.Material, t.materials.len) catch return false;
        for (by_track, t.materials) |*g, m| g.* = spec.groundMaterial(m);
        allocator.free(d.track_materials);
        d.track_materials = by_track;
        return true;
    }

    // m as the car's step reads it: a material of the car's own list stays as it is, unless its bumps are in
    // another model
    fn borrowFor(s: *const Sim, m: Material, borrowed: *std.EnumSet(material.Field), counts: *std.EnumArray(material.How, u32)) Material {
        const model = bumpsModel(s.game);
        const fits = if (model) |b| std.meta.activeTag(m.bumps) == b else true;
        if (fits and s.materials.len > 0 and std.mem.eql(u8, m.origin(), s.materials[0].origin())) return m;
        const i, const how = material.match(m, s.materials) orelse return withBumpsOf(m, model);
        if (how == .default) log.warn("{s}: no {s} match, {s}", .{ std.mem.sliceTo(&m.name, 0), @tagName(s.game), std.mem.sliceTo(&s.materials[i].name, 0) });
        counts.getPtr(how).* += 1;
        const out, const taken = material.borrow(m, s.materials[i]);
        borrowed.setUnion(taken);
        return withBumpsOf(out, model);
    }
};

// the bumps model a step reads; null: none
fn bumpsModel(g: Game) ?material.Model {
    return switch (g) {
        .d3 => .by_time,
        .dr1 => .by_distance,
        .fh => null,
    };
}

// bumps in another model than the step reads: none
fn withBumpsOf(m: Material, model: ?material.Model) Material {
    const b = model orelse return m;
    if (std.meta.activeTag(m.bumps) == b) return m;
    var out = m;
    out.bumps = material.smooth(b);
    return out;
}

const log = std.log.scoped(.material);

fn logBorrowed(car: []const u8, level: []const u8, borrowed: std.EnumSet(material.Field), counts: std.EnumArray(material.How, u32)) void {
    var names: [256]u8 = undefined;
    var w: std.Io.Writer = .fixed(&names);
    var it = borrowed.iterator();
    while (it.next()) |f| w.print(" {s}", .{@tagName(f)}) catch {};
    log.info("{s} car on a {s} level borrows{s}; matches: {} exact, {} family, {} table, {} DEFA", .{
        car, level, w.buffered(), counts.get(.exact), counts.get(.family), counts.get(.table), counts.get(.default),
    });
}

fn setSeaLevel(s: *dr1_car.State, raw: u64) void {
    s.sea_level_raw = raw;
    s.aero.sea_level_raw = raw;
}

pub const Game = union(enum) { d3: force.State, dr1: Dr1, fh: Fh }; // the tag is the origin of the car's materials

pub const Fh = struct { params: fh_car.Params, state: fh_car.State };

pub const Dr1 = struct {
    state: dr1_car.State,
    track_materials: []dr1_ground.Material = &.{}, // by the track's material index

    pub fn ground(d: *const Dr1, c: Car, track: *const Mesh) dr1_ground.Ground {
        return spec.groundOf(c, d.state, track, d.track_materials.ptr);
    }
};

pub const allocator = std.heap.page_allocator;

pub fn readFile(path: [*:0]const u8) ![]u8 {
    const io = std.Io.Threaded.global_single_threaded.io();
    return std.Io.Dir.cwd().readFileAlloc(io, std.mem.span(path), allocator, .unlimited);
}

pub fn fromAbi(st: State) body.State {
    return .{ .pos = st.pos, .axes = .{ st.axes[0], st.axes[1], st.axes[2] }, .vel = st.vel, .angvel = st.angvel };
}

pub fn toAbi(s: body.State) State {
    return .{ .pos = s.pos, .axes = .{ s.axes[0], s.axes[1], s.axes[2] }, .vel = s.vel, .angvel = s.angvel };
}

// the made-up friction-circle car (sim/data/cars/hatch.zon) on flat ground, without surfaces to borrow from
export fn sim_create() ?*Sim {
    const s = allocator.create(Sim) catch return null;
    s.* = .{};
    force.rest(s.car, &s.game.d3);
    return s;
}

export fn sim_destroy(s: *Sim) void {
    s.unload();
    s.world.deinit();
    allocator.destroy(s);
}

// a D3 level: its track.jpk and its materials (import/d3.py: <out>/d3/materials.zon); no sea level (0)
export fn sim_load_track(s: *Sim, jpk_path: [*:0]const u8, materials_path: [*:0]const u8) bool {
    s.world.load(jpk_path, materials_path, .d3) catch return false;
    return s.fitLevel();
}

// a DR1 track.jpk and a surface_materials.xml for its material codes: stand-ins until a DR1 car brings its materials;
// sea level 0 until sim_load_dr1_car gives one
export fn sim_load_dr1_track(s: *Sim, jpk_path: [*:0]const u8, materials_path: [*:0]const u8) bool {
    s.world.load(jpk_path, materials_path, .dr1) catch return false;
    return s.fitLevel();
}

// a DR1 level: the track, its real materials (materials.zon, spec.save) and its sea level in m (Engine::s_seaLevel)
export fn sim_load_dr1_level(s: *Sim, jpk_path: [*:0]const u8, codes_path: [*:0]const u8, materials_path: [*:0]const u8, sea_level: f64) bool {
    const all = zon.load([]Material, allocator, std.mem.span(materials_path)) catch return false;
    defer allocator.free(all);
    s.world.load(jpk_path, codes_path, .dr1) catch return false;
    s.world.fill(all);
    s.world.sea_level = sea_level;
    return s.fitLevel();
}

// a car for the friction-circle step (sim/data/cars/hatch.zon, import/d3.py) at rest at the pose, with its surfaces
// to borrow from on another level
export fn sim_load_d3_car(s: *Sim, car_path: [*:0]const u8, materials_path: [*:0]const u8, pose: *const State) bool {
    loadD3(s, car_path, materials_path, pose.*) catch return false;
    return s.fitLevel();
}

fn loadD3(s: *Sim, car_path: [*:0]const u8, materials_path: [*:0]const u8, pose: State) !void {
    const c = try zon.load(Car, allocator, std.mem.span(car_path));
    errdefer std.zon.parse.free(allocator, c);
    const all = try zon.load([]Material, allocator, std.mem.span(materials_path));
    s.own(c, all);
    s.game = (Sim{}).game;
    s.game.d3.engine = .{ .rate = c.engine.idle, .mean = c.engine.idle };
    s.game.d3.body = fromAbi(pose);
    force.rest(c, &s.game.d3);
}

// a DR1 car from its files (spec.save), at rest at the pose, undamaged, on any level or none.
// sea_level: m, set as the level's (sim_load_dr1_car_on_level takes the level's)
export fn sim_load_dr1_car(s: *Sim, car_path: [*:0]const u8, materials_path: [*:0]const u8, pose: *const State, sea_level: f64) bool {
    s.world.sea_level = sea_level;
    return sim_load_dr1_car_on_level(s, car_path, materials_path, pose);
}

export fn sim_load_dr1_car_on_level(s: *Sim, car_path: [*:0]const u8, materials_path: [*:0]const u8, pose: *const State) bool {
    loadDr1(s, car_path, materials_path, pose.*) catch return false;
    return s.fitLevel();
}

fn loadDr1(s: *Sim, car_path: [*:0]const u8, materials_path: [*:0]const u8, pose: State) !void {
    const c = try zon.load(Car, allocator, std.mem.span(car_path));
    errdefer std.zon.parse.free(allocator, c);
    const all = try zon.load([]Material, allocator, std.mem.span(materials_path));
    s.own(c, all);
    const state = spec.init(c, rowsOf(pose), spec.seaLevelRaw(s.world.sea_level), spec.undamaged);
    s.game = .{ .dr1 = .{ .state = state } };
}

// a car for the curve-table tyre step from its file (sim/data/cars, import/fh1.py) at rest at the pose, with the
// surfaces it borrows from on another level. Its gearbox starts in the manual mode (sim_fh_gearbox).
export fn sim_load_fh_car(s: *Sim, car_path: [*:0]const u8, materials_path: [*:0]const u8, pose: *const State) bool {
    loadFh(s, car_path, materials_path, pose.*) catch return false;
    return s.fitLevel();
}

fn loadFh(s: *Sim, car_path: [*:0]const u8, materials_path: [*:0]const u8, pose: State) !void {
    const c = try zon.load(Car, allocator, std.mem.span(car_path));
    errdefer std.zon.parse.free(allocator, c);
    const p = try fh_spec.params(c);
    const all = try zon.load([]Material, allocator, std.mem.span(materials_path));
    s.own(c, all);
    s.game = .{ .fh = .{ .params = p, .state = fh_car.rest(p, fromAbi(pose)) } };
}

// the step sim_step expects: 1/60 s for DR1 and FH
export fn sim_dt(s: *const Sim) f32 {
    return switch (s.game) {
        .d3 => d3_dt,
        .dr1, .fh => dr1_dt,
    };
}

// the DR1 car's state, for the tools; null for D3
export fn sim_dr1_state(s: *Sim) ?*dr1_car.State {
    return switch (s.game) {
        .dr1 => |*d| &d.state,
        .d3, .fh => null,
    };
}

// the track height under x, z (a ray from 1 km above), or ground.none
export fn sim_track_height(s: *Sim, x: f32, z: f32) f32 {
    const t = if (s.world.track) |*t| t else return ground.none;
    const hit = t.raycast(.{ x, 1000, z }, .{ x, -1000, z }) orelse return ground.none;
    return 1000 - 2000 * hit.t;
}

// body rows: right, up, forward, position
fn rowsOf(st: State) [4][4]f32 {
    const a = st.axes;
    return .{ .{ a[0][0], a[0][1], a[0][2], 0 }, .{ a[1][0], a[1][1], a[1][2], 0 }, .{ a[2][0], a[2][1], a[2][2], 0 }, .{ st.pos[0], st.pos[1], st.pos[2], 1 } };
}

fn xyz(v: [4]f32) [3]f32 {
    return v[0..3].*;
}

// DR1: the car at rest at the pose again (engine, gearbox, wheels), then moving as st says; the damage stays
export fn sim_set_state(s: *Sim, st: *const State) void {
    switch (s.game) {
        .d3 => |*d| {
            d.body = fromAbi(st.*);
            force.rest(s.car, d);
        },
        .dr1 => |*d| {
            d.state = spec.init(s.car, rowsOf(st.*), d.state.sea_level_raw, spec.damageOf(d.state));
            d.state.body.vel = .{ st.vel[0], st.vel[1], st.vel[2], 0 };
            d.state.body.angvel = .{ st.angvel[0], st.angvel[1], st.angvel[2], 0 };
            dr1_car.postUpdate(&d.state);
            dr1_car.endFrame(&d.state);
        },
        .fh => |*f| {
            f.state = fh_car.rest(f.params, fromAbi(st.*));
            f.state.body.vel = st.vel;
            f.state.body.angvel = st.angvel;
        },
    }
}

export fn sim_get_state(s: *const Sim, out: *State) void {
    out.* = switch (s.game) {
        .d3 => |d| toAbi(d.body),
        .dr1 => |d| blk: {
            const b = d.state.body;
            break :blk .{ .pos = xyz(b.rows[3]), .axes = .{ xyz(b.rows[0]), xyz(b.rows[1]), xyz(b.rows[2]) }, .vel = xyz(b.vel), .angvel = xyz(b.angvel) };
        },
        .fh => |f| toAbi(f.state.body),
    };
}

// DR1: the car, then the physics world (its integration, the hull if the car has one); no step without a level
pub export fn sim_step(s: *Sim, input: *const Input, dt: f32) void {
    switch (s.game) {
        .d3 => |*d| {
            force.step(s.car, d, input.*, s.world.surface(), dt);
            force.drawWheels(s.car, d);
            s.world.collide(s.car, &d.body, dt);
        },
        .dr1 => |*d| {
            const t = if (s.world.track) |*t| t else return; // DR1 needs a mesh
            dr1_input.apply(&d.state, input.*);
            const pose = d.state.body.rows;
            dr1_car.frame(&d.state, dt, d.ground(s.car, t));
            dr1_world.step(s.car, &d.state, pose, t, dt);
        },
        .fh => |*f| {
            const in = input.*;
            const box = f.params.gearbox;
            const gear = if (box.mode == .automatic) automaticRequest(in.gear, f.state.box.gear) else manualRequest(in.gear, box.top);
            const step: fh_car.Input = .{ .throttle = in.throttle, .brake = in.brake, .handbrake = if (in.handbrake) 1 else 0, .steer = in.steer, .gear = gear, .clutch = in.clutch };
            fh_car.step(f.params, &f.state, step, s.world.surface(), dt);
            s.world.collide(s.car, &f.state.body, dt);
            fh_car.syncMomentum(f.params, &f.state);
        },
    }
}

// FH gear requests. The automatic box picks the forward gear: the input only selects reverse (10), neutral (0)
// or drive (1..9). The manual boxes take the gear as it is; past the top gear or below 0 it is neutral.
fn automaticRequest(gear: i32, current: usize) ?usize {
    if (gear == 10) return 10;
    if (gear < 1 or gear > 9) return 0;
    return if (current == 0 or current == 10) 1 else null;
}

fn manualRequest(gear: i32, top: usize) usize {
    if (gear == 10) return 10;
    return if (gear >= 1 and gear <= top) @intCast(gear) else 0;
}

// the FH gearbox mode: 0 automatic, 1 manual (the box works the clutch), 2 manual with the clutch pedal
// (Input.clutch). false for other cars
export fn sim_fh_gearbox(s: *Sim, mode: u32) bool {
    const f = switch (s.game) {
        .fh => |*f| f,
        else => return false,
    };
    if (mode > 2) return false;
    f.params.gearbox.mode = @enumFromInt(mode);
    fh_gearbox.shiftPoints(&f.params.gearbox);
    f.state.box.target = f.state.box.gear;
    return true;
}

test {
    _ = @import("shared/body.zig");
    _ = @import("shared/bumps.zig");
    _ = @import("shared/chassis.zig");
    _ = @import("shared/coupling.zig");
    _ = @import("shared/d3math.zig");
    _ = @import("shared/ground.zig");
    _ = @import("shared/surface.zig");
    _ = @import("material.zig");
    _ = @import("shared/mesh.zig");
    _ = @import("d3/drivetrain.zig");
    _ = @import("d3/engine.zig");
    _ = @import("d3/force.zig");
    _ = @import("d3/integrate.zig");
    _ = @import("d3/longitudinal.zig");
    _ = @import("d3/steering.zig");
    _ = @import("d3/suspension.zig");
    _ = @import("d3/tyre.zig");
    _ = @import("dr1/tyre.zig");
    _ = @import("dr1/input.zig");
    _ = @import("dr1/spec.zig");
}
