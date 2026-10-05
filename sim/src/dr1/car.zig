// Car::updateAll (0xf6f5e0): one 1/60 s step of a DR1 car. The car's state the step reads and writes, in one place;
// the suspension, engine and drivetrain keep their own structs inside it
const body = @import("body.zig");
const math = @import("math.zig");
const aero = @import("aero.zig");
const engine = @import("engine.zig");
const drivetrain = @import("drivetrain.zig");
const ground = @import("ground.zig");
const suspension = @import("suspension.zig");
const wheel = @import("wheel.zig");
const wiper = @import("wiper.zig");
const controls = @import("controls.zig");

const V4 = body.V4;
const dot4 = body.dot4;

fn splat(x: f32) V4 {
    return @splat(x);
}

// ClutchRates (setup+0x238 standard, +0x254 manual sequential)
pub const ClutchRates = extern struct {
    lift: f32, // m_clutchLiftRate, per s
    gear_time: f32, // m_clutchGearTime
    semi_auto_time: f32,
    clutch_delay: f32, // m_autoClutchGearDelay
    off_throttle_up: f32, // m_timeOffThrottleGearUp
    on_throttle_atc: f32, // m_timeOnThrottleATCTime
    on_throttle_down: f32, // m_timeOnThrottleGearDown
};

// setup+0x280..0x294
pub const SteeringRates = extern struct {
    low_rate: f32,
    low_speed: f32,
    high_rate: f32,
    high_speed: f32,
    return_centre: f32, // m_steeringRateReturnToCentreMultiple
    return_desired: f32,
};

// setup+0x480: the auto reverse
pub const Reverse = extern struct {
    reverse_to_first_speed: f32, // fMaxSpeedForReverseToFirstTransition
    first_to_reverse_speed: f32,
    threshold: f32, // brake over throttle
};

// what updateAll reads of the setup (CarType, car+0x66c0)
pub const Setup = extern struct {
    jump: [3]f32, // +0x14c m_jumpAssistFallRate, MultipleCap, RiseRate
    drift_friction_control: bool, // +0x1ec m_driftFrictionMultipleControl
    donut_damping: bool, // +0x469
    momentum_boost: f32, // +0x628 m_lowSpeedEngineMomentumTransferBoost
    engine_layout: u32, // engine+0 m_engineLayout: 2 and 4 turn the body about its forward axis
    clutch: [2]ClutchRates, // standard, manual sequential
    clutch_points: [4]f32, // +0x270 full, none, reverse full, reverse none: x idle
    steering: SteeringRates,
    lock_by_graph: bool, // +0xa0 m_lockReductionUsesGraph
    pad_lock: [25]f32, // +0xa4 m_padLockVsSpeed
    pad_lock_scale: f32, // +0x108 m/s per graph step
    counter_steer: bool, // +0x664 m_counterSteerLockEnabled
    toe: [2]suspension.DynamicCamber, // +0x3b8 front, +0x40c rear
    camber: [2]suspension.DynamicCamber, // +0x39c front, +0x3f0 rear
    roll_centre: [2][3]f32, // +0x11c front, +0x128 rear
    heat: wheel.Heat,
    reverse: Reverse,
    handbrake_strength: f32, // +0x46c
};

// the driver aids (car and setup)
pub const Aids = extern struct {
    abs_control: f32, // car+0x50
    abs_level: [2]f32, // setup+0x634
    abs_strength: [2]f32, // setup+0x63c
    atc_control: f32, // car+0x54
    atc_strength: [2]f32, // setup+0x62c
    esp_strength: [2]f32, // setup+0x64c
};

// CarDamage (car+0x8340) fields
pub const Damage = extern struct {
    steer_wobble: f32, // +0x11ef0 m_randomSteerWobbleScale
    gearbox: f32, // +0x11ef8 m_gearboxEffect: lengthens the shift
    gear_drop: f32, // +0x11af4: past 0.8 the top gear is skipped, past 0.5 upshifts may crunch [likely]
    gearbox_state: i32, // +0x11fac: 8 working
    misalign: [4]f32, // +0x11f20 + 0x20 i
    disconnect_steering: [4]bool, // +0x11f28 + 0x20 i
    brake_scale: [4]f32, // +0x11f24 + 0x20 i
    detached: [4]bool, // +0x11fd4
    burst: [4]bool, // +0x11fd0
    wear: wheel.Wear, // +0x11e98
};

// Car m_launch*
pub const Launch = extern struct {
    enabled: bool, // car+0x6dc0
    available: bool, // +0x6dc8: armed
    active: bool, // +0x6dc9
    standing: f32, // +0x6dcc m_launchStandingTimer
    sweet_spot: f32, // +0x6dd0: time the revs were in the window
};

// DriveTrain (car+0x7028)
pub const Transmission = extern struct {
    ratios: [11]f32, // +0x34: 0 neutral, 1..9 forward, 10 reverse
    crossover: [11]f32, // +0x60 m_torqueCrossover
    previous: i32, // +0x8c
    gear: i32, // +0x90
    auto_gearbox: bool, // +0x94
    auto_clutch: bool, // +0x95
    auto_reverse: bool, // +0x96
    manual_override: bool, // +0x97
    manual_upshift: bool, // +0x98
    seamless: bool, // +0x99: the ratio blends through a shift
    since_change: f32, // +0xa0
    damaged_drop: bool, // +0xa4
    top_gear: i32, // +0xa8
    jammed: bool, // +0xac
    dual_clutch: bool, // +0xad
    swing: f32, // +0xc0 m_gearShiftSwing: the auto box's timer toward a shift, +-2
    shift_time: [2]f32, // +0xc4 semi auto, +0xc8 manual
};

// the car's view of SurfaceSpec slowdowns, per wheel; null: none
pub const Slowdowns = [4]?ground.Slowdown;

pub const State = extern struct {
    body: body.RigidBody, // car+0x6590
    sys: suspension.System, // T = car+0x610
    data: [4]suspension.WheelData,
    wheels: [4]suspension.Wheel, // car+0x7100
    drivetrain: drivetrain.Drivetrain, // *(T+0x840)
    side: drivetrain.CarSide,
    engine: engine.Engine, // car+0x6e00; engine.random is the car's generator (car+0x6db4)
    transmission: Transmission,
    wiper: wiper.Wiper, // car+0x5b4
    solver: wheel.Solver, // *(car+0x210), CarJob's stack: kept from frame to frame

    start: suspension.Frame, // car+0xe80
    acceleration: [4]f32, // car+0x2a0, smoothed
    speed: f32, // car+0x8264
    recent_speeds: [2][10]f32, // car+0x550, +0x578
    next_speed: i32, // car+0x5a0
    time_steps: [5]f32, // car+0x5ec
    next_time_step: i32, // car+0x600
    internal_clock: f32, // car+0x6db8
    controller_clock: f32, // car+0x6dbc
    render_clock: f32, // car+0x44 [likely]
    stall_timer: f32, // car+0x4c: up below 0.8 idle, 0..2 [likely]
    since_gear_up: f32, // car+0x5a4
    since_gear_down: f32, // car+0x5a8
    anti_stall: f32, // car+0x5ac m_antiStallWeight
    permit_reverse: bool, // car+0x5b0
    drift_angle: f32, // car+0x1d0
    lock: [2]f32, // car+0x1d4 min, +0x1d8 max steering
    controls: [14]u32, // car+0x6648 m_controls, see controls.Control
    gear_request: i32, // car+0x6684 m_n32NewGearRequest, 11 none
    best_gear: i32, // car+0x6688 m_bestGearFromAutoBox
    steering_lock: f32, // car+0x66a0
    wheel_turn: f32, // car+0x66a4 m_steeringWheelTurn
    pad_min_lock: f32, // car+0x66a8
    pad_assists: bool, // car+0x66ac m_enablePadAssistances
    lowest_speed_max_lock: f32, // car+0x66b0
    steering_sensitivity: f32, // car+0x66b4
    steering_linearity: f32, // car+0x66b8
    launch: Launch,
    auto_handbrake: bool, // car+0x6dd4
    auto_handbrake_enabled: bool, // car+0x6dd5
    horn: u8, // car+0x6dd6
    stability: f32, // car+0x6dc4 m_stabilityControl
    brake: f32, // car+0x6dd8 m_brake
    steering: f32, // car+0x6ddc m_steering: the steering angle
    wheel_count: u32, // car+0x6de0
    jump_inertia: f32, // car+0x8240 m_jumpAssistTyreInertia
    air_factor: f32, // car+0x8274 m_airFactorForVisualPitchAndRoll
    normal_inertia: [3]f32, // car+0x8278
    momentum_rate: f32, // engine+0x1b8: the rate the reaction torque last saw
    autogear: extern struct { throttle: f32, timer: f32 }, // car+0xd13f0: the throttle last frame, time since it last lifted
    wheel_transform: [4][4]f32, // car+0x510 m_localWheelTransform, visual
    gear_crunch: u32, // car+0
    gear_clunk: u32, // car+4
    landed: u32, // car+8

    // read only in the step
    end: suspension.Frame, // car+0xf60, left by the last frame
    world_body: body.Body, // car+0x17c0: the physics engine's body
    setup: Setup,
    aids: Aids,
    damage: Damage,
    springs: suspension.Springs,
    aero: aero.Aero, // the setup part; updateAll fills the rest
    ackerman: extern struct { wheels: [2]u32, track: f32, wheelbase: f32 }, // car+0x8290: the steered wheels, outer when turning right first
    recorder: controls.Recorder,
    wall_material: u16, // car+0x82b0 m_wallRidingContactMaterial (the physics world sets it), ground.none: none
    locked_axles: u32, // car+0x82a8 nLockedAxlesFlag: 1 rear, 2 front
    settings: u32, // car+0xe70 SimulationSettings flags, see wheel.settings_*
    wheel_geometry: [4]bool, // car+0x17d0: the wheel has a physics geometry, for the visual transform
    render_offsets: [4][4]f32, // car+0x60
    fuel_level: f32, // car+0x8228
    tyre_wear_rate: f32, // car+0x8238
    optimum_launch_rpm: f32, // car+0x826c
    visual_turn_rate: f32, // car+0x8234 fVisualTurnRateMultiple
    altitude: f32, // car+0x1074, the mid frame height
    sea_level_raw: u64, // Engine::s_seaLevel
    auto_recover: bool, // car+0x6638: Car::updateAutoRecovery, not ported
};

const soft_edge_blend: f32 = 0; // .bss 0x338f3bc
const wall_blend: f32 = 0.95; // .data 0x32ec590
const surface_blend: f32 = 0.95; // .data 0x32ec58c

// 0xf6f5e0 Car::updateAll. Not ported: Car::updateAutoRecovery (needs the physics world's ray casts; skipped),
// updateAutoSpoiler and updateVisualOffsetMatrix (visual, nothing in the step reads them)
pub fn updateAll(s: *State, dt: f32, g: ground.Ground) void {
    const v: V4 = s.body.vel;
    s.acceleration = (splat(1 / dt) * (v - @as(V4, s.start.vel)) + @as(V4, s.acceleration)) * splat(0.5);
    s.start = .{ .rows = s.body.rows, .vel = s.body.vel, .angvel = s.body.angvel };
    s.speed = @sqrt(dot4(v, v));
    s.time_steps[@intCast(s.next_time_step)] = dt;
    s.internal_clock = s.internal_clock + dt;
    s.next_time_step = @rem(s.next_time_step + 1, 5);
    s.controller_clock = s.controller_clock + dt;
    s.since_gear_up = s.since_gear_up + dt;
    s.since_gear_down = s.since_gear_down + dt;
    s.transmission.since_change = s.transmission.since_change + dt;
    s.render_clock = s.render_clock + dt;
    controls.update(s, dt);

    driftAngle(s);
    if (s.setup.drift_friction_control) driftFriction(s);
    if (s.setup.donut_damping) donutDamping(s, dt);
    jumpAssist(s, dt);
    s.body.force = V4{ 0, -9.81 * s.body.mass, 0, 0 } + @as(V4, s.body.force);
    aero.update(aeroOf(s.*), &s.body);
    wiper.update(&s.wiper, &s.engine.random, dt, s.speed);
    reactionTorque(s);

    const n = s.wheel_count;
    wheel.precalculate(&s.solver, s.wheels[0..n], s.body.rows);
    for (&s.wheels, 0..) |*w, i| wheel.patchPoints(w, s.solver, i, s.body.rows[3], dt);
    var params: Slowdowns = undefined;
    for (s.wheels[0..n], params[0..n]) |w, *p| p.* = material(g, w.contacted[w.outermost_contacted], 2);
    slowDown(s, dt, soft_edge_blend, params);
    if (s.wall_material != ground.none) {
        const p = g.materials[s.wall_material].slowdown[2];
        if (p.factor != 0) slowDown(s, dt, wall_blend, .{ p, p, p, p });
        s.wall_material = ground.none;
    }
    suspensionAndIntegration(s, dt, g);
    stallTimer(s, dt);
    for (s.wheels[0..n], params[0..n]) |w, *p| p.* = if (w.contact) material(g, w.contact_surface, 0) else null;
    slowDown(s, dt, surface_blend, params);
    surfaceEffects(s, dt, g);
    if (n != 0) {
        smoothSkidding(s.wheels[0..n], dt);
        for (s.wheels[0..n]) |*w| turnWheel(w, dt);
    }
    lockedAxles(s);
    wheel.brakeTemperatures(s.wheels[0..n], s.body.inverse_mass, s.speed, s.brake, dt);
    for (s.wheels[0..n]) |*w| w.visual_skid = 0;

    s.speed = @sqrt(dot4(s.body.vel, s.body.vel));
    const k: usize = @intCast(s.next_speed);
    s.recent_speeds[0][k] = s.speed;
    s.recent_speeds[1][k] = s.speed;
    s.next_speed = @rem(s.next_speed + 1, 10);
    for (s.wheels[0..n]) |*w| w.cos_up = if (w.contact) cosUp(w.surface_normal, s.body.rows) else 1;
    for (s.wheels[0..n]) |*w| w.damper_bump_multiple = 1;
    syncSide(s);
}

// One frame of the car as Car::updateAllCars (0xf75f70) runs it, with the physics world stubbed: no contacts, and the
// world body ends each frame where the car is. CarJob::doJob (0xf75e60) runs pre_update, updateAll and post_update.
// Not ported: the bump stop contacts (updateAllCars adds a world contact for a wheel past W+0x9c), small objects,
// the recorder, the damage model and the visual parts of pre_update, post_update and updateBodyState.
pub fn frame(s: *State, dt: f32, g: ground.Ground) void {
    preUpdate(s);
    if (dt > 0) updateAll(s, dt, g);
    postUpdate(s);
    endFrame(s);
}

const at_rest: f32 = @bitCast(@as(u32, 0x29e12e13)); // 1e-13, .rodata 0x2736234

// 0xef6ec0 Car::pre_update: the velocities back from the world body; a speed below 1e-13 on every axis is 0
pub fn preUpdate(s: *State) void {
    s.body.vel = s.world_body.vel;
    s.body.angvel = s.world_body.angvel;
    if (resting(s.body.vel)) s.body.vel = .{ 0, 0, 0, 0 };
    if (resting(s.body.angvel)) s.body.angvel = .{ 0, 0, 0, s.body.angvel[3] };
}

fn resting(v: [4]f32) bool {
    return at_rest > @abs(v[0]) and at_rest > @abs(v[1]) and at_rest > @abs(v[2]);
}

// 0xf2c880 Car::post_update: the velocities to the world body, angular w cleared
pub fn postUpdate(s: *State) void {
    s.body.angvel[3] = 0;
    s.world_body.vel = s.body.vel;
    s.world_body.angvel = s.body.angvel;
}

// the rest of updateAllCars for the car: the pose made orthonormal, then Car::updateBodyState (0xeeabf0) keeps the
// end frame (setupForFrame expects the next frame to start there) and the render frame. Stubs: the render frame is
// interpolated at 1 (the renderer's value, .data 0x32ec0d0), so its height (the altitude) and wheel angles are the
// newest; the world body moves with the car, and CarSide.body is a view of it.
pub fn endFrame(s: *State) void {
    s.body.rows = body.orthonormal(s.body.rows);
    s.end = .{ .rows = s.body.rows, .vel = s.body.vel, .angvel = s.body.angvel };
    s.altitude = s.end.rows[3][1];
    s.aero.altitude = s.altitude;
    for (s.wheels[0..s.wheel_count]) |*w| renderAngle(w);
    s.world_body.com = s.body.rows[3];
    s.side.body = s.world_body;
}

// a wheel's render angle from the last one to the newest, the short way round, then on in the direction it spins
fn renderAngle(w: *suspension.Wheel) void {
    const turn: f32 = 6.2831855;
    const last = w.rotation_angle[2];
    var d = @rem(w.rotation_angle[0] - last, turn); // fmodf
    if (3.1415927 < d) d -= turn;
    var to = d + last;
    if (w.rotation_rate <= 0) {
        while (last < to) to -= turn;
    } else {
        while (to < last) to += turn;
    }
    w.rotation_angle[1] = (to - last) * 1 + last; // interpolated at 1
}

// the drivetrain's CarSide is a view of fields the car owns (the same memory in the game): copied over by hand
pub fn syncSide(s: *State) void {
    const c = &s.side;
    for (s.data, s.wheels, 0..) |d, w, i| {
        c.traction_torque[i], c.rolling_torque[i] = d.torque;
        c.rotation_rate[i] = d.rotation_rate;
        c.forward_velocity[i] = d.forward_velocity;
        c.abs_active[i] = d.abs.active;
        c.pos[i] = d.pos;
        c.forward_axis[i] = d.forward_axis;
        c.radius[i] = w.radius;
        c.max_brake_torque[i] = w.max_brake_torque;
        c.applied_brake_torque[i] = w.applied_brake_torque;
    }
    c.braking_torque = s.sys.braking_torque;
    c.gear_ratio = s.sys.gear_ratio;
    c.handbrake = controls.get(s, .handbrake);
    c.clutch_pedal = controls.get(s, .final_clutch);
    c.auto_clutch = s.transmission.auto_clutch;
    c.engine_rate_sum = s.engine.rate;
    c.since_rev_cut = s.engine.since_rev_cut;
}

fn material(g: ground.Ground, m: u16, k: usize) ?ground.Slowdown {
    return if (m == ground.none) null else g.materials[m].slowdown[k];
}

// the body slip angle: velocity against the forward axis, over 1 m/s
fn driftAngle(s: *State) void {
    const v: V4 = .{ s.body.vel[0], s.body.vel[1], s.body.vel[2], 0 };
    const q = (v[0] * v[0] + v[1] * v[1]) + v[2] * v[2];
    if (!(q > 1)) {
        s.drift_angle = 0;
        return;
    }
    var u = body.normalize4(v, q);
    u[3] = 0;
    s.drift_angle = math.atan2(dot4(s.body.rows[0], u), dot4(s.body.rows[2], u));
}

// with the drift friction control: the engine braking follows the brake while the free wheels outrun the driven
fn driftFriction(s: *State) void {
    s.engine.drift_friction[1] = 1;
    const gear = s.transmission.gear;
    if (gear == 10 or gear == 0 or s.wheel_count == 0) return;
    var driven: f32 = 0;
    var free: f32 = 0;
    var nd: i32 = 0;
    var nf: i32 = 0;
    for (s.wheels[0..s.wheel_count]) |w| {
        const x = w.rotation_rate * w.radius;
        if (w.drive_proportion > 0) {
            driven = driven + x;
            nd += 1;
        } else {
            free = free + x;
            nf += 1;
        }
    }
    if (nf > 0 and nd > 0 and free / @as(f32, @floatFromInt(nf)) - 0.2 > driven / @as(f32, @floatFromInt(nd))) s.engine.drift_friction[1] = s.brake;
}

// m_donutDamping: below 12 m/s a fast yaw is damped
fn donutDamping(s: *State, dt: f32) void {
    const x = (12 - s.speed) * 0.25;
    if (!(x > 0)) return;
    const a = if (x > 1) 1 else x;
    const y = (@abs(s.body.angvel[1]) - 3) / 3;
    const b = if (y > 1) 1 else math.maxss(y, 0);
    const d = a * (b * dt);
    s.body.angvel = @as(V4, s.body.angvel) * splat(1 - (d + d));
}

// the jump assist's tyre inertia: rises in the air above 8 m/s, falls back to 1 on the ground
fn jumpAssist(s: *State, dt: f32) void {
    const grounded = s.sys.wheels_on_ground != 0;
    const a = if (grounded) s.air_factor - dt else s.air_factor + dt;
    s.air_factor = if (a > 2) 2 else if (0 < a) a else 0;
    const fall, const cap, const rise = s.setup.jump;
    if (grounded or 8 > s.speed) {
        const j = s.jump_inertia - fall * dt;
        s.jump_inertia = if (j < 1) 1 else j;
    } else {
        const j = rise * dt + s.jump_inertia;
        s.jump_inertia = if (j > cap) cap else j;
    }
}

// the crank's change of speed turns the body the other way, more at low speed; only with every wheel on a geometry
fn reactionTorque(s: *State) void {
    for (s.wheels[0..s.wheel_count]) |w| if (!w.has_geometry) return;
    const prev = s.momentum_rate;
    s.momentum_rate = s.engine.rate;
    const d = (s.engine.rate - prev) * s.engine.inertia;
    const axis: usize = if (s.setup.engine_layout == 2 or s.setup.engine_layout == 4) 2 else 0;
    const sp = s.speed / 25;
    const k: f32 = if (sp > 1) 0 else if (sp > 0) 1 - sp else 1;
    const boost = s.setup.momentum_boost * k + 1;
    const w = d / s.body.inertia[axis];
    s.body.angvel = splat(w) * (@as(V4, s.body.rows[axis]) * splat(boost)) + @as(V4, s.body.angvel);
}

fn aeroOf(s: State) aero.Aero {
    var a = s.aero;
    a.wheels = s.wheel_count;
    a.wheels_on_ground = s.sys.wheels_on_ground;
    a.speed = s.speed;
    for (s.wheels, 0..) |w, i| {
        a.rolling[i] = if (w.tyre.has_record) w.tyre.rolling_resistance else 0;
        a.load[i] = w.load_multiple;
        a.contact[i] = w.contact;
    }
    return a;
}

// 0xf52150 Car::updateSlowDown: a soft surface drags each wheel's hub by the slowdown factor (from its minimum
// speed, in full at its full speed), blended toward the mean over the wheels, and brakes the wheel's spin
pub fn slowDown(s: *State, dt: f32, blend: f32, params: Slowdowns) void {
    const n = s.wheel_count;
    if (n == 0) return;
    var f: [4]f32 = undefined;
    var sum: f32 = 0;
    for (params[0..n], f[0..n]) |p, *x| {
        x.* = if (p) |q| unit((s.speed - q.min_speed) / (q.full_speed - q.min_speed)) * q.factor else 0;
        sum = sum + x.*;
    }
    const share = blend * (sum / @as(f32, @floatFromInt(n)));
    for (f[0..n]) |*x| x.* = x.* * (1 - blend) + share;
    for (f[0..n], s.wheels[0..n], 0..) |x, *w, i| {
        if (!(x > 0)) continue;
        const r: V4 = s.solver.rel_pos[i];
        const v = body.cross(s.body.angvel, r) + @as(V4, s.body.vel);
        const q = dot4(v, v);
        if (!(q > 0.01)) continue;
        body.applyForce(&s.body, v / splat(@sqrt(q)) * splat(-x * s.body.mass), r);
        const rr = w.rotation_rate - x * dt / w.radius;
        w.rotation_rate = if (0 <= w.rotation_rate * rr) rr else 0;
        s.data[i].rotation_rate = w.rotation_rate;
    }
}

fn unit(x: f32) f32 {
    return if (x > 1) 1 else if (0 < x) x else 0;
}

// 0xf664e0 Car::updateSuspensionAndIntegration: the grip each wheel's tyre state leaves, SuspensionSystem::update,
// then the wheels' wear, dirt and temperatures
pub fn suspensionAndIntegration(s: *State, dt: f32, g: ground.Ground) void {
    const n = s.wheel_count;
    var heights: [4]f32 = undefined;
    for (s.wheels[0..n], s.data[0..n], heights[0..n], s.damage.burst[0..n]) |w, *d, *h, burst| {
        h.* = w.height;
        d.external_grip = wheel.externalGrip(w, s.damage.wear, s.settings, burst);
    }
    s.normal_inertia = s.body.inertia;
    syncSide(s);
    var gr = g;
    gr.car.speed = s.speed;
    gr.car.detached = s.damage.detached;
    suspension.update(&s.sys, vehicle(s), &s.data, &s.wheels, dt, gr);

    for (s.wheels[0..n], s.data[0..n]) |*w, d| {
        w.grip_available = if (w.contact) w.upward_force[0] * w.grip_coefficient * w.grip_for_ai[0] * d.external_grip else 0;
    }
    for (s.wheels[0..n], s.data[0..n], heights[0..n], s.render_offsets[0..n], s.wheel_geometry[0..n], &s.wheel_transform) |w, d, h, off, has, *t| {
        if (!has) continue;
        t.* = splat(math.maxss(h, w.height)) * @as(V4, d.spring_axis) + (@as(V4, off) + @as(V4, w.rest_position));
        t[3] = 1;
    }
    const rate = if (s.settings & wheel.settings_wear != 0) s.tyre_wear_rate else 0;
    var landed = false;
    for (s.wheels[0..n], s.damage.detached[0..n]) |*w, off| {
        if (off) continue;
        const c: ?wheel.Contact = if (w.contact) .{ .wear_multiple = g.materials[w.contact_surface].wear_multiple, .dirt_buildup = g.materials[w.contact_surface].dirt_buildup } else null;
        if (wheel.suspensionNew(w, c, s.damage.wear, s.speed, s.setup.heat, s.body.rows, dt, rate)) landed = true;
    }
    if (landed) s.landed = 1;
}

// everything SuspensionSystem::update reads outside the suspension
fn vehicle(s: *State) suspension.Vehicle {
    const t = s.transmission;
    const c = s.setup.clutch;
    return .{
        .body = &s.body,
        .car = .{
            .mass = s.body.mass,
            .speed = s.speed,
            .up = s.body.rows[1],
            .body = s.world_body,
            .assists = .{ .brake = s.brake, .handbrake = controls.get(s, .handbrake), .abs_control = s.aids.abs_control, .abs_level = s.aids.abs_level, .abs_strength = s.aids.abs_strength },
            .camber = s.setup.camber,
            .start_velocity = s.start.vel,
        },
        .springs = s.springs,
        .traction = .{
            .auto_gearbox = t.auto_gearbox,
            .upshift_cut = .{ c[0].off_throttle_up, c[1].off_throttle_up },
            .upshift_ramp = .{ c[0].on_throttle_atc, c[1].on_throttle_atc },
            .since_upshift = s.since_gear_up,
            .manual_clutch = controls.get(s, .manual_clutch),
            .final_clutch = controls.get(s, .final_clutch),
            .strength = s.aids.atc_strength,
            .control = s.aids.atc_control,
        },
        .controls = .{
            .brake = s.brake,
            .handbrake = controls.get(s, .handbrake),
            .steering = s.steering,
            .clutch = controls.get(s, .final_clutch),
            .handbrake_strength = s.setup.handbrake_strength,
            .stability = s.stability,
            .esp_strength = s.aids.esp_strength,
            .brake_scale = s.damage.brake_scale,
            .altitude = s.altitude,
            .sea_level_raw = s.sea_level_raw,
        },
        .gearbox = .{
            .ratios = t.ratios,
            .previous = t.previous,
            .gear = t.gear,
            .auto_clutch = t.auto_clutch,
            .seamless = t.seamless,
            .since_change = t.since_change,
            .shift_time = t.shift_time,
            .clutch_delay = .{ c[0].clutch_delay, c[1].clutch_delay },
        },
        .start = s.start,
        .end = s.end,
        .tyre_burst = s.damage.burst,
        .roll_centre = s.setup.roll_centre,
        .drivetrain = &s.drivetrain,
        .side = &s.side,
        .engine = &s.engine,
        .motion = .{ .up = s.body.rows[1], .forward = s.body.rows[2], .velocity = s.start.vel, .gear = t.gear },
    };
}

// car+0x4c: counts up while the engine runs below 0.8 idle, down otherwise
fn stallTimer(s: *State, dt: f32) void {
    const e = s.engine;
    const x = (if (0.8 * e.idle > math.minss(e.rate, 1.1 * e.rev_limit)) dt else -dt) + s.stall_timer;
    s.stall_timer = if (x > 2) 2 else math.maxss(x, 0);
}

// 0xefa360 Car::updateSurfaceEffects: a rough surface kicks the body's spin at random above 8 m/s
pub fn surfaceEffects(s: *State, dt: f32, g: ground.Ground) void {
    const n = s.wheel_count;
    if (n == 0) return;
    var rough: f32 = 0;
    for (s.wheels[0..n]) |w| {
        if (w.contact) rough = rough + g.materials[w.contact_surface].roughness;
    }
    if (!(rough > 0) or !(s.speed > 8)) return;
    const f = rough / @as(f32, @floatFromInt(n)) * ((s.speed - 8) / 12);
    const chance = math.truncate((dt * 6 + dt * 6) * f * 256);
    if (chance <= @as(i32, @intCast(engine.random(&s.engine.random) & 0xff))) return;
    for (0..3) |k| {
        const r: f32 = @floatFromInt(engine.random(&s.engine.random) & 0xff);
        s.body.angvel[k] = (r - 128) * 0.0078125 * 0.05 * f + s.body.angvel[k];
    }
}

// the smoothed skidding (sound and marks) toward the skidding
fn smoothSkidding(wheels: []suspension.Wheel, dt: f32) void {
    const k = 10 * dt;
    const f = if (k < 1) k else 1;
    const keep: f32 = if (1 > k) 1 - k else 0;
    for (wheels) |*w| {
        w.smoothed_amount = w.smoothed_amount * keep + w.skidding[2] * f;
        w.smoothed_skidding[0] = w.smoothed_skidding[0] * keep + w.skidding[0] * f;
        w.smoothed_skidding[1] = w.smoothed_skidding[1] * keep + w.skidding[1] * f;
    }
}

// the wheel's rotation angle, wrapped to +-4 pi
fn turnWheel(w: *suspension.Wheel, dt: f32) void {
    const a0 = w.rotation_angle[0];
    w.rotation_angle = .{ a0, a0, a0 };
    var a = w.rotation_rate * dt + a0;
    if (a > 12.566371) {
        a = a - 25.132742;
    } else if (-12.566371 > a) {
        a = a + 25.132742;
    }
    w.rotation_angle[0] = a;
}

// on a rigid axle the two wheels lean with their height difference
fn lockedAxles(s: *State) void {
    for ([2]usize{ 0, 2 }, [2]u32{ 1, 2 }) |i, bit| {
        if (s.locked_axles & bit == 0) continue;
        const a, const b = .{ &s.wheels[i], &s.wheels[i + 1] };
        const lean = math.atan((a.height - b.height) / (a.rest_position[0] - b.rest_position[0]));
        b.camber = lean;
        a.camber = -lean;
    }
}

// W+0x18c: the ground normal's lean toward the car's up, within the up-forward plane
fn cosUp(n: [4]f32, rows: [4][4]f32) f32 {
    const up: V4 = rows[1];
    const forward: V4 = rows[2];
    const a = dot4(up, n);
    const p = splat(a) * up + splat(dot4(n, forward)) * forward;
    const c = a / @sqrt(dot4(p, p));
    return if (c > 1) 1 else math.maxss(c, 0);
}
