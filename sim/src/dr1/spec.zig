// A DR1 car's constants (the game-neutral Car, ../car.zig) into car.State and back. init copies them in, as
// Car::processCarType (0xf57b00) copies CarType into the Car; specOf reads them back. Car wheel order RL RR FL FR:
// a per-axle value comes from wheel 2 (front) or wheel 0 (rear). Also the step's ground.Material from the neutral
// Material (../material.zig) and back, and the car and material files (ZON).
const std = @import("std");
const spec = @import("../car.zig");
const material = @import("../material.zig");
const mesh = @import("../shared/mesh.zig");
const car = @import("car.zig");
const drivetrain = @import("drivetrain.zig");
const ground = @import("ground.zig");
const suspension = @import("suspension.zig");
const wheel = @import("wheel.zig");

const Car = spec.Car;
const Material = material.Material;
const State = car.State;

const drivetrain_wheel = [4]usize{ 2, 3, 0, 1 }; // the drivetrain's FL FR RL RR

// the damage state: not car data, so init takes it from outside (undamaged for a new car)
pub const Damage = struct {
    car: car.Damage, // its wear template is the Car's
    springs: [4]suspension.Damage,
    wing_effect: [2]f32,
    rotational_resistance: [4]f32,
    clutch_effect: f32,
    cutout_frequency: f32,
    exhaust: f32,
    turbo: f32,
    visual_angle: [4]f32, // W+0x1bc
    render_offsets: [4][4]f32, // car+0x60, Car::updateDamagedWheelRenderOffsets 0xf0cfe0 sets it from the damaged model
};

// a new car: every scale 1, nothing broken (gearbox_state 8: working)
pub const undamaged: Damage = .{
    .car = .{
        .steer_wobble = 0,
        .gearbox = 0,
        .gear_drop = 0,
        .gearbox_state = 8,
        .misalign = @splat(0),
        .disconnect_steering = @splat(false),
        .brake_scale = @splat(1),
        .detached = @splat(false),
        .burst = @splat(false),
        .wear = std.mem.zeroes(wheel.Wear),
    },
    .springs = @splat(.{ .spring = 1, .anti_roll = 1, .damper = 1 }),
    .wing_effect = @splat(1),
    .rotational_resistance = @splat(0),
    .clutch_effect = 1,
    .cutout_frequency = 0,
    .exhaust = 0,
    .turbo = 0,
    .visual_angle = @splat(0),
    .render_offsets = std.mem.zeroes([4][4]f32),
};

// Engine::s_seaLevel of a sea level in m: whole mm, obfuscated as aero.airDensity reads it
pub fn seaLevelRaw(metres: f64) u64 {
    const mm: i64 = @intFromFloat(@round(metres * 1000));
    return @as(u64, @bitCast(mm)) *% 0x7c2a7a0ba017ee15; // the inverse of airDensity's factor
}

pub fn damageOf(s: State) Damage {
    var angle: [4]f32 = undefined;
    for (&angle, s.wheels) |*a, w| a.* = w.visual_damage_angle;
    return .{
        .car = s.damage,
        .springs = s.springs.damage,
        .wing_effect = s.aero.wing_effect,
        .rotational_resistance = s.side.rotational_resistance,
        .clutch_effect = s.side.clutch_effect,
        .cutout_frequency = s.engine.cutout_frequency,
        .exhaust = s.engine.exhaust_damage,
        .turbo = s.engine.turbo.damage_scale,
        .visual_angle = angle,
        .render_offsets = s.render_offsets,
    };
}

// the car at rest at pose (body rows: right, up, forward, position); sea_level_raw: the stage's Engine::s_seaLevel
pub fn init(c: Car, pose: [4][4]f32, sea_level_raw: u64, damage: Damage) State {
    var s = std.mem.zeroes(State);
    setDamage(&s, damage);
    apply(c, &s);
    rest(&s, c, pose, sea_level_raw);
    car.syncSide(&s);
    return s;
}

fn setDamage(s: *State, d: Damage) void {
    s.damage = d.car;
    s.springs.damage = d.springs;
    s.aero.wing_effect = d.wing_effect;
    s.side.rotational_resistance = d.rotational_resistance;
    s.side.clutch_effect = d.clutch_effect;
    s.engine.cutout_frequency = d.cutout_frequency;
    s.engine.exhaust_damage = d.exhaust;
    s.engine.turbo.damage_scale = d.turbo;
    for (&s.wheels, d.visual_angle) |*w, a| w.visual_damage_angle = a;
    s.render_offsets = d.render_offsets;
}

// every field the step only reads, from c; the state is left alone. The step has no nulls: an absent part becomes
// 0 or false, unless noted
pub fn apply(c: Car, s: *State) void {
    s.body.mass = c.mass;
    s.body.inverse_mass = 1 / c.mass;
    s.body.inertia = c.inertia;
    s.wheel_count = 4;
    s.sys.wheel_count = 4;
    s.sys.ai_com_offset = 0;
    s.sys.min_speed_for_normal_model = 0;
    s.sys.ground_geometries = 0;
    s.sys.abs_period = c.abs_period orelse 0;
    s.wheel_geometry = .{ true, true, true, true };
    for (c.wheels, &s.wheels, &s.data, 0..) |w, *x, *d, i| {
        applyWheel(w, x, d);
        s.side.radius[i] = w.radius;
        s.side.max_brake_torque[i] = w.max_brake_torque;
    }
    for (&s.drivetrain.wheels, drivetrain_wheel) |*w, i| w.state.inertia = c.wheels[i].inertia;
    applySetup(c, s);
    applyEngine(c, s);
    applyDrivetrain(c, s);
    applyAero(c, s);

    s.steering_lock = c.steer_lock orelse 0;
    s.pad_min_lock = c.pad_min_lock orelse 0;
    s.lowest_speed_max_lock = c.full_lock_speed orelse 0;
    s.steering_sensitivity = c.steer_sensitivity orelse 0;
    s.steering_linearity = c.pad_linearity.?;
    s.pad_assists = c.pad_assists orelse false;
    s.visual_turn_rate = c.visual_turn_rate orelse 0;
    s.ackerman = .{ .wheels = c.steered_wheels orelse [2]u32{ 3, 2 }, .track = c.track, .wheelbase = c.wheelbase }; // the front wheels
    s.aids = c.aids orelse std.mem.zeroes(car.Aids);
    s.stability = c.stability orelse 0;
    const launch = c.launch orelse std.mem.zeroes(spec.Launch);
    s.launch.enabled = launch.enabled;
    s.optimum_launch_rpm = launch.optimum_rpm;
    s.auto_handbrake_enabled = c.auto_handbrake orelse false;
    s.springs.axles = .{ c.wheels[2].spring.two_segment.axle, c.wheels[0].spring.two_segment.axle };
    s.springs.load_scale = c.spring_load_scale orelse 1;
    s.damage.wear = c.tyre_wear orelse std.mem.zeroes(wheel.Wear);
    s.tyre_wear_rate = c.tyre_wear_rate orelse 0;
    s.settings = c.tyre_effects orelse 0;
    s.locked_axles = c.rigid_axles orelse 0;
    const wiper = c.wiper orelse std.mem.zeroes(spec.Wiper);
    s.wiper.visual_offset = wiper.visual_offset;
    s.wiper.hold_time = wiper.hold_time;
    s.wiper.sweep_time = wiper.sweep_time;
    s.wiper.only_when_moving = wiper.only_when_moving;
    s.auto_recover = c.auto_recover orelse false;
}

fn applyWheel(w: spec.Wheel, x: *suspension.Wheel, d: *suspension.WheelData) void {
    const g = w.tyre.graphs;
    x.rest_position = .{ w.rest_position[0], w.rest_position[1], w.rest_position[2], 1 };
    d.spring_axis = .{ w.spring_axis[0], w.spring_axis[1], w.spring_axis[2], 0 };
    x.radius = w.radius;
    x.tyre_radius = w.radius;
    x.radius_modifier = 0;
    x.inertia = w.inertia;
    x.min_height = w.min_height;
    x.max_height = w.max_height;
    x.bump_stop_height = w.bump_stop_height;
    x.spring_top_height = w.spring.two_segment.top_height;
    x.slow_bump = w.slow_bump;
    x.fast_bump = w.fast_bump.?;
    x.slow_rebound = w.slow_rebound;
    x.fast_rebound = w.fast_rebound.?;
    x.bump_zone = w.bump_zone.?;
    x.rebound_zone = w.rebound_zone.?;
    x.anti_roll = w.anti_roll;
    d.base_tyre_spring = w.tyre_spring.?;
    d.base_tyre_damping = w.tyre_damping.?;
    d.maximum_depth = w.tyre_depth.?;
    d.tan_peak_slip = w.tan_peak_slip.?;
    d.peak_slip_ratio = w.peak_slip_ratio.?;
    x.grip_coefficient = w.grip.?;
    x.tyre.legacy = g.legacy;
    x.camber = g.camber;
    x.burst_grip = g.burst_grip;
    x.max_brake_torque = w.max_brake_torque;
    x.handbrake = w.handbrake;
    x.drive_proportion = w.drive_proportion;
    x.width = w.width orelse 0;
    x.max_radius = w.max_radius orelse w.radius;
    x.critical_cos = w.side_cos orelse 0;
    x.toe_in = w.toe_in orelse 0;
}

fn applySetup(c: Car, s: *State) void {
    const t = c.engine.curve.torque;
    const g = c.gearbox;
    const front, const rear = .{ c.wheels[2], c.wheels[0] };
    const no_toe = std.mem.zeroes(suspension.DynamicCamber);
    const no_roll = [3]f32{ 0, 0, 0 };
    const return_rates = c.steer_return_rates orelse [2]f32{ 0, 0 };
    s.setup = .{
        .jump = .{ c.jump_rates.?[1], c.jump_cap.?, c.jump_rates.?[0] },
        .drift_friction_control = c.drift_friction_control orelse false,
        .donut_damping = c.donut_damping orelse false,
        .momentum_boost = t.momentum_boost,
        .engine_layout = t.layout,
        .clutch = g.clutch_rates orelse std.mem.zeroes([2]car.ClutchRates),
        .clutch_points = g.clutch_points orelse std.mem.zeroes([4]f32),
        .steering = .{
            .low_rate = c.steer_rate.?[0],
            .low_speed = c.steer_rate_speeds.?[0],
            .high_rate = c.steer_rate.?[1],
            .high_speed = c.steer_rate_speeds.?[1],
            .return_centre = return_rates[0],
            .return_desired = return_rates[1],
        },
        .lock_by_graph = c.lock_by_graph orelse false,
        .pad_lock = c.pad_lock.?,
        .pad_lock_scale = c.pad_lock_step.?,
        .counter_steer = c.counter_steer orelse false,
        .toe = .{ front.toe_change orelse no_toe, rear.toe_change orelse no_toe },
        .camber = .{ front.tyre.graphs.dynamic_camber, rear.tyre.graphs.dynamic_camber },
        .roll_centre = .{ front.roll_centre orelse no_roll, rear.roll_centre orelse no_roll },
        .heat = c.tyre_heat orelse std.mem.zeroes(wheel.Heat),
        .reverse = g.reverse orelse std.mem.zeroes(car.Reverse),
        .handbrake_strength = c.handbrake_strength,
    };
}

const no_fuel = std.mem.zeroes(spec.Fuel);

fn applyEngine(c: Car, s: *State) void {
    const t = c.engine.curve.torque;
    const starter = c.engine.starter orelse std.mem.zeroes(spec.Starter);
    const fuel = c.fuel orelse no_fuel;
    const e = &s.engine;
    e.inertia = c.engine.inertia;
    e.idle = c.engine.idle;
    e.rev_limit = c.engine.rev_limit;
    e.starter_strength = starter.strength;
    e.starter.time_scale = starter.time_scale;
    e.friction = t.friction;
    e.friction_drive = t.friction_drive;
    e.torque = t.table;
    e.restrictor = t.restrictor;
    e.rev_step = t.step;
    e.max_power = t.max_power;
    e.max_torque = t.max_torque;
    e.power_scale = t.power_scale;
    e.difficulty = t.difficulty;
    e.drivetrain_friction_scale = t.drivetrain_friction_scale;
    e.fuel.tank = fuel.burned;
    e.fuel.capacity = if (fuel.burned) fuel.capacity else 0;
    e.fuel.full_tank_weight = if (fuel.burned) fuel.full_tank_weight else 0;

    const b = &e.turbo;
    b.capacity = t.capacity;
    b.enabled = c.engine.turbo != null;
    const x = c.engine.turbo orelse std.mem.zeroes(spec.Turbo);
    b.inlet_volume = x.inlet_volume;
    b.inlet_torque = x.inlet_torque;
    b.inlet_pressure_cap = x.inlet_pressure_cap;
    b.outlet_volume = x.outlet_volume;
    b.outlet_torque = x.outlet_torque;
    b.outlet_pressure_cap = x.outlet_pressure_cap;
    b.inertia = x.inertia;
    b.inlet_rate = x.inlet_rate;
    b.sqr_factor = x.sqr_factor;
    b.outlet_input_rate = x.outlet_input_rate;
    b.rpm_for_boost = x.rpm_for_boost;
    b.spool_rpm_for_boost = x.spool_rpm_for_boost;
    b.spool_rpm_max = x.spool_rpm_max;
    b.max_boost = x.max_boost;
    b.peak = c.engine.rev_limit;
}

fn applyDrivetrain(c: Car, s: *State) void {
    const tree = c.diffs.tree;
    const d = &s.drivetrain;
    d.engine_inertia = c.engine.inertia;
    d.rev_limit = c.engine.rev_limit;
    d.clutch.friction = c.gearbox.clutch_torque;
    applyDiff(&d.centre, tree.centre);
    applyDiff(&d.front, tree.front);
    applyDiff(&d.rear, tree.rear);
    s.side.centre_index = @intFromEnum(tree.centre.kind);
    s.side.clutch_on_handbrake = tree.clutch_on_handbrake;
    s.side.handbrake_uses_rear_clutch = tree.handbrake_uses_rear_clutch;

    const g = c.gearbox;
    const t = &s.transmission;
    t.ratios = g.ratios;
    t.top_gear = g.top_gear;
    t.crossover = g.crossover orelse std.mem.zeroes([11]f32);
    t.shift_time = g.shift_time orelse [2]f32{ 0, 0 };
    t.auto_gearbox = g.auto_gearbox orelse false;
    t.auto_clutch = g.auto_clutch orelse false;
    t.auto_reverse = g.auto_reverse orelse false;
    t.manual_override = g.manual_override orelse false;
    t.manual_upshift = g.manual_upshift orelse false;
    t.dual_clutch = g.dual_clutch orelse false;
}

fn applyDiff(x: *drivetrain.Diff, t: spec.TreeDiff) void {
    x.kind = t.kind;
    x.bias = t.bias;
    x.driving_bias = t.driving_bias;
    x.braking_bias = t.braking_bias;
    x.preload = t.preload;
    x.viscosity = t.viscosity;
    x.clutch_torque = t.clutch_torque;
    x.clutch_bias = t.clutch_bias;
}

fn applyAero(c: Car, s: *State) void {
    const p = c.aero.directional;
    const a = &s.aero;
    a.side_drag, a.vertical_drag, a.forward_drag = @as([3]f32, c.drag.?);
    a.reverse_drag = p.reverse_drag;
    a.centre_x = p.centre_x;
    a.centre_y = p.centre_y;
    a.downforce_central = p.downforce_central;
    a.downforce = p.downforce;
    a.wing_drag = p.wing_drag;
    a.pressure_centre = p.pressure_centre;
    a.pressure_centre_air = p.pressure_centre_air;
    a.legacy_rolling = p.legacy_rolling;
    a.legacy_downforce = p.legacy_downforce;
    a.pikes_peak = p.pikes_peak;
    a.drag_scale = p.drag_scale;
    a.legacy_downforce_central = p.downforce_central[0];
    a.legacy_centre = .{ p.pressure_centre, p.pressure_centre_air };
    a.axle_z = .{ c.wheels[2].rest_position[2], c.wheels[0].rest_position[2] };
}

// the state at rest, as the resets leave it (Engine::reset 0xf1a1d0, SuspensionSystem::reset 0xf32da0, the Car
// constructor's first gear); the per-frame copies of the constants too
fn rest(s: *State, c: Car, pose: [4][4]f32, sea_level_raw: u64) void {
    const frame: suspension.Frame = .{ .rows = pose, .vel = .{ 0, 0, 0, 0 }, .angvel = .{ 0, 0, 0, 0 } };
    s.body.rows = pose;
    s.start = frame;
    s.end = frame;
    s.world_body.com = pose[3];
    s.altitude = pose[3][1];
    s.aero.altitude = pose[3][1];
    s.sea_level_raw = sea_level_raw;
    s.aero.sea_level_raw = sea_level_raw;
    s.normal_inertia = s.body.inertia;

    for (&s.wheels, &s.data) |*w, *d| {
        d.external_grip = 1;
        d.height_if_on_ground = -1000;
        d.height_if_on_ground_next = -1000;
        d.perceived_inertia = w.inertia;
        d.tyre_spring = d.base_tyre_spring;
        d.tyre_damping = d.base_tyre_damping;
        d.max_depth = d.maximum_depth;
        d.rebound_rate = (0 * w.slow_rebound + w.fast_rebound) / w.slow_rebound;
        d.bump_rate = w.fast_bump / w.slow_bump;
        w.direction = .{ 1, 0 };
        w.render_direction = .{ 1, 0 };
        w.previous_render_direction = .{ 1, 0 };
        w.grip_for_ai = .{ 1, 1 };
        w.contact_surface = ground.none;
        w.contacted = .{ ground.none, ground.none, ground.none };
        w.temperature = @splat(s.setup.heat.ambient);
        w.brake_temperature = 293.15;
        w.cos_up = 1;
        w.damper_bump_multiple = 1;
        w.tyre.slip_window = 1;
        w.tyre.forces = .{};
    }
    wheel.precalculate(&s.solver, &s.wheels, pose); // the patch points under the hubs
    for (&s.wheels, s.solver.rel_pos) |*w, r| w.patch_point = @as(@Vector(4, f32), r) + @as(@Vector(4, f32), pose[3]);

    const e = &s.engine;
    s.side.engine_rate = e.idle;
    e.rate = e.idle;
    e.previous_rate = e.idle;
    e.delayed_rate = e.idle;
    e.since_rev_cut = 9999;
    e.drift_friction = .{ 1, 1 };
    e.starter.strength = 1;
    e.turbo.rotation_rate = @as(f32, std.math.pi) * e.turbo.spool_rpm_for_boost / 30;
    const fuel = c.fuel orelse no_fuel;
    s.fuel_level = fuel.capacity;
    if (fuel.burned) e.fuel.current = fuel.capacity;

    const t = &s.transmission;
    t.gear = 1;
    t.previous = 1;
    s.sys.gear_ratio = t.ratios[1];
    s.gear_request = 11;
    s.best_gear = 1;
    s.since_gear_up = std.math.floatMax(f32);
    s.since_gear_down = std.math.floatMax(f32);
    s.permit_reverse = true;
    s.lock = .{ -s.steering_lock, s.steering_lock };
    s.jump_inertia = 1;
    s.wall_material = ground.none;
    s.wiper.forward = true;

    const d = &s.drivetrain;
    d.engine = .{ .inertia = d.engine_inertia }; // drivetrain::Engine::reset 0xedbef0; integrate keeps it
    d.random.index = 625; // seeds itself on the first draw
    d.clutch.load = 1;
    d.clutch.damage_scale = 1;
    d.rear_clutch.ratio = 1;
    for ([_]*drivetrain.Diff{ &d.centre, &d.front, &d.rear }) |x| x.override = 1;
}

// the Car of a state; g: its tyre compounds and the car fields the ground query reads
pub fn specOf(s: State, g: ground.Ground) Car {
    const e = s.engine;
    const t = s.transmission;
    const st = s.setup;
    const a = s.aero;
    const b = e.turbo;
    var wheels: [4]spec.Wheel = undefined;
    for (&wheels, 0..) |*w, i| w.* = wheelOf(s, g, i);
    return .{
        .mass = s.body.mass,
        .inertia = s.body.inertia,
        .wheels = wheels,
        .drag = .{ a.side_drag, a.vertical_drag, a.forward_drag },
        .aero = .{ .directional = .{
            .reverse_drag = a.reverse_drag,
            .centre_x = a.centre_x,
            .centre_y = a.centre_y,
            .downforce_central = a.downforce_central,
            .downforce = a.downforce,
            .wing_drag = a.wing_drag,
            .pressure_centre = a.pressure_centre,
            .pressure_centre_air = a.pressure_centre_air,
            .legacy_rolling = a.legacy_rolling,
            .legacy_downforce = a.legacy_downforce,
            .pikes_peak = a.pikes_peak,
            .drag_scale = a.drag_scale,
        } },
        .engine = .{ .inertia = e.inertia, .idle = e.idle, .rev_limit = e.rev_limit, .curve = .{ .torque = .{
            .step = e.rev_step,
            .table = e.torque,
            .friction = e.friction,
            .friction_drive = e.friction_drive,
            .restrictor = e.restrictor,
            .capacity = e.turbo.capacity,
            .max_power = e.max_power,
            .max_torque = e.max_torque,
            .power_scale = e.power_scale,
            .difficulty = e.difficulty,
            .drivetrain_friction_scale = e.drivetrain_friction_scale,
            .layout = st.engine_layout,
            .momentum_boost = st.momentum_boost,
        } }, .starter = .{ .strength = e.starter_strength, .time_scale = e.starter.time_scale }, .turbo = if (!b.enabled) null else .{
            .inlet_volume = b.inlet_volume,
            .inlet_torque = b.inlet_torque,
            .inlet_pressure_cap = b.inlet_pressure_cap,
            .outlet_volume = b.outlet_volume,
            .outlet_torque = b.outlet_torque,
            .outlet_pressure_cap = b.outlet_pressure_cap,
            .inertia = b.inertia,
            .inlet_rate = b.inlet_rate,
            .sqr_factor = b.sqr_factor,
            .outlet_input_rate = b.outlet_input_rate,
            .rpm_for_boost = b.rpm_for_boost,
            .spool_rpm_for_boost = b.spool_rpm_for_boost,
            .spool_rpm_max = b.spool_rpm_max,
            .max_boost = b.max_boost,
        } },
        // without the engine's tank nothing burns the fuel, so it is still full; its weight is then not in State
        .fuel = if (e.fuel.tank)
            .{ .capacity = e.fuel.capacity, .full_tank_weight = e.fuel.full_tank_weight, .burned = true }
        else
            .{ .capacity = s.fuel_level, .full_tank_weight = 0, .burned = false },
        .gearbox = .{
            .ratios = t.ratios,
            .top_gear = t.top_gear,
            .clutch_torque = s.drivetrain.clutch.friction,
            .shift_time = t.shift_time,
            .crossover = t.crossover,
            .clutch_rates = st.clutch,
            .clutch_points = st.clutch_points,
            .reverse = st.reverse,
            .auto_gearbox = t.auto_gearbox,
            .auto_clutch = t.auto_clutch,
            .auto_reverse = t.auto_reverse,
            .manual_override = t.manual_override,
            .manual_upshift = t.manual_upshift,
            .dual_clutch = t.dual_clutch,
        },
        .diffs = .{ .tree = .{
            .centre = treeDiffOf(s.drivetrain.centre),
            .front = treeDiffOf(s.drivetrain.front),
            .rear = treeDiffOf(s.drivetrain.rear),
            .handbrake_uses_rear_clutch = s.side.handbrake_uses_rear_clutch,
            .clutch_on_handbrake = s.side.clutch_on_handbrake,
        } },
        .launch = .{ .enabled = s.launch.enabled, .optimum_rpm = s.optimum_launch_rpm },
        .handbrake_strength = st.handbrake_strength,
        .auto_handbrake = s.auto_handbrake_enabled,
        .abs_period = s.sys.abs_period,
        .aids = s.aids,
        .stability = s.stability,
        .drift_friction_control = st.drift_friction_control,
        .donut_damping = st.donut_damping,
        .auto_recover = s.auto_recover,
        .bumps = .{ .surface_factor = 0, .amplification = g.car.bump_scale }, // DR1 does not read the factor (setup+0x2e8)
        .spring_load_scale = s.springs.load_scale,
        .rigid_axles = s.locked_axles,
        .tyre_heat = st.heat,
        .tyre_wear = s.damage.wear,
        .tyre_wear_rate = s.tyre_wear_rate,
        .tyre_effects = s.settings,
        .contact_mode = g.car.simulation_type,
        .pad_lock = st.pad_lock,
        .pad_lock_step = st.pad_lock_scale,
        .pad_linearity = s.steering_linearity,
        .steer_rate = .{ st.steering.low_rate, st.steering.high_rate },
        .steer_rate_speeds = .{ st.steering.low_speed, st.steering.high_speed },
        .track = s.ackerman.track,
        .wheelbase = s.ackerman.wheelbase,
        .steer_lock = s.steering_lock,
        .pad_min_lock = s.pad_min_lock,
        .full_lock_speed = s.lowest_speed_max_lock,
        .lock_by_graph = st.lock_by_graph,
        .counter_steer = st.counter_steer,
        .steer_sensitivity = s.steering_sensitivity,
        .pad_assists = s.pad_assists,
        .steer_return_rates = .{ st.steering.return_centre, st.steering.return_desired },
        .steered_wheels = s.ackerman.wheels,
        .visual_turn_rate = s.visual_turn_rate,
        .jump_rates = .{ st.jump[2], st.jump[0] },
        .jump_cap = st.jump[1],
        .wiper = .{
            .visual_offset = s.wiper.visual_offset,
            .hold_time = s.wiper.hold_time,
            .sweep_time = s.wiper.sweep_time,
            .only_when_moving = s.wiper.only_when_moving,
        },
    };
}

fn wheelOf(s: State, g: ground.Ground, i: usize) spec.Wheel {
    const w = s.wheels[i];
    const d = s.data[i];
    const x: usize = if (i < 2) 1 else 0; // the axle: 0 front, 1 rear
    return .{
        .rest_position = w.rest_position[0..3].*,
        .spring_axis = d.spring_axis[0..3].*,
        .radius = w.radius,
        .max_radius = w.max_radius,
        .width = w.width,
        .side_cos = w.critical_cos,
        .inertia = w.inertia,
        .min_height = w.min_height,
        .max_height = w.max_height,
        .bump_stop_height = w.bump_stop_height,
        .spring = .{ .two_segment = .{ .axle = s.springs.axles[x], .top_height = w.spring_top_height } },
        .slow_bump = w.slow_bump,
        .fast_bump = w.fast_bump,
        .slow_rebound = w.slow_rebound,
        .fast_rebound = w.fast_rebound,
        .bump_zone = w.bump_zone,
        .rebound_zone = w.rebound_zone,
        .anti_roll = w.anti_roll,
        .roll_centre = s.setup.roll_centre[x],
        .toe_in = w.toe_in,
        .toe_change = s.setup.toe[x],
        .tyre_spring = d.base_tyre_spring,
        .tyre_damping = d.base_tyre_damping,
        .tyre_depth = d.maximum_depth,
        .tan_peak_slip = d.tan_peak_slip,
        .peak_slip_ratio = d.peak_slip_ratio,
        .grip = w.grip_coefficient,
        .tyre = .{ .graphs = .{
            .compound = g.compounds[i][0..g.compound_lens[i]],
            .legacy = w.tyre.legacy,
            .camber = w.camber,
            .dynamic_camber = s.setup.camber[x],
            .burst_grip = w.burst_grip,
        } },
        .max_brake_torque = w.max_brake_torque,
        .handbrake = w.handbrake,
        .drive_proportion = w.drive_proportion,
    };
}

fn treeDiffOf(x: drivetrain.Diff) spec.TreeDiff {
    return .{
        .kind = x.kind,
        .bias = x.bias,
        .driving_bias = x.driving_bias,
        .braking_bias = x.braking_bias,
        .preload = x.preload,
        .viscosity = x.viscosity,
        .clutch_torque = x.clutch_torque,
        .clutch_bias = x.clutch_bias,
    };
}

// the ground updateAll takes for c in state s on track; materials: by the track's material index
pub fn groundOf(c: Car, s: State, track: *const mesh.Mesh, materials: [*]const ground.Material) ground.Ground {
    var g: ground.Ground = .{
        .track = track,
        .materials = materials,
        .compounds = undefined,
        .compound_lens = undefined,
        .car = .{
            .render_offsets = s.render_offsets,
            .detached = s.damage.detached,
            .speed = s.speed,
            .bump_scale = c.bumps.?.amplification,
            .simulation_type = c.contact_mode orelse 0,
        },
    };
    for (c.wheels, &g.compounds, &g.compound_lens) |w, *p, *n| {
        p.* = w.tyre.graphs.compound.ptr;
        n.* = @intCast(w.tyre.graphs.compound.len);
    }
    return g;
}

// all materials, picked by the track's material codes; a code not in the list gets zeroes
pub fn trackMaterials(gpa: std.mem.Allocator, track: *const mesh.Mesh, all: []const Material) ![]ground.Material {
    const out = try gpa.alloc(ground.Material, track.materials.len);
    for (out, track.materials) |*m, t| {
        m.* = std.mem.zeroes(ground.Material);
        for (all) |x| {
            if (std.mem.eql(u8, &x.code(), &t.code())) m.* = groundMaterial(x);
        }
    }
    return out;
}

// the step's material: the name's code, the bumps by distance, the loose ground and tyre class as one Surface.
// The step has no nulls: an absent effect becomes the value without it
pub fn groundMaterial(m: Material) ground.Material {
    var simulation: suspension.Surface = undefined;
    const loose = m.loose orelse std.mem.zeroes(material.Loose); // firm ground
    inline for (@typeInfo(material.Loose).@"struct".fields) |f| @field(simulation, f.name) = @field(loose, f.name);
    simulation.tyre_code = m.tyre_class orelse 0; // the tyre's graphs off the ground
    const none: material.Slowdown = .{ .full_speed = 1 }; // 0, without a 0/0
    return .{
        .code = m.code(),
        .ellipsoid = m.ellipsoid orelse false,
        .bumps_wavelength = m.bumps.by_distance.wavelength,
        .bumps_magnitude = m.bumps.by_distance.magnitude,
        .depth = m.depth,
        .simulation = simulation,
        .slowdown = .{ slowdownTo(m.slowdown), slowdownTo(m.wall_slowdown orelse none), slowdownTo(m.edge_slowdown orelse none) },
        .wear_multiple = m.wear orelse 1,
        .dirt_buildup = m.dirt_buildup orelse false,
        .roughness = m.roughness orelse 0,
    };
}

// back: groundMaterial(materialOf(g)) has g's bits; grip, hardness and side drag stay null
pub fn materialOf(g: ground.Material) Material {
    var loose: material.Loose = undefined;
    inline for (@typeInfo(material.Loose).@"struct".fields) |f| @field(loose, f.name) = @field(g.simulation, f.name);
    return .{
        .name = material.named("dr1", g.code),
        .depth = g.depth,
        .bumps = .{ .by_distance = .{ .wavelength = g.bumps_wavelength, .magnitude = g.bumps_magnitude } },
        .slowdown = slowdownOf(g.slowdown[0]),
        .loose = loose,
        .tyre_class = g.simulation.tyre_code,
        .ellipsoid = g.ellipsoid,
        .wall_slowdown = slowdownOf(g.slowdown[1]),
        .edge_slowdown = slowdownOf(g.slowdown[2]),
        .wear = g.wear_multiple,
        .dirt_buildup = g.dirt_buildup,
        .roughness = g.roughness,
    };
}

fn slowdownTo(s: material.Slowdown) ground.Slowdown {
    return .{ .factor = s.amount, .min_speed = s.from_speed, .full_speed = s.full_speed };
}

fn slowdownOf(s: ground.Slowdown) material.Slowdown {
    return .{ .from_speed = s.min_speed, .full_speed = s.full_speed, .amount = s.factor };
}

// the files are ZON: a Car (its tyre compounds inside the wheels), and a list of Material
pub const save = @import("../zon.zig").save;
pub const load = @import("../zon.zig").load;
