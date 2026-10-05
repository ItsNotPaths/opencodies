// SuspensionSystem (T = car+0x610): the per-wheel ground, tyre and assist work of the substeps
const std = @import("std");
const body = @import("body.zig");
const math = @import("math.zig");
const aero = @import("aero.zig");
const drivetrain = @import("drivetrain.zig");
const engine = @import("engine.zig");
const tyre_ = @import("tyre.zig");
const Tyre = tyre_.Tyre;
const V4 = body.V4;

// the driver aids as the car and its setup hold them
pub const Assists = extern struct {
    brake: f32, // car+0x6dd8 m_brake
    handbrake: f32, // car+0x6670 m_controls[10] [likely]
    abs_control: f32, // car+0x50 m_abs_control: 0 off .. 1 on
    abs_level: [2]f32, // setup+0x634 m_abs_level_on, +0x638 _off
    abs_strength: [2]f32, // setup+0x63c m_abs_strength_on, +0x640 _off

    fn blend(a: Assists, on_off: [2]f32) f32 {
        return (on_off[0] - on_off[1]) * a.abs_control + on_off[1];
    }
};

// SuspensionSystemWheelData+0x190
pub const Abs = extern struct {
    activated: bool = false, // m_absActivatedThisUpdate: set here, cleared by the caller
    active: bool = false, // m_absCurrentlyActive
};

// 0xf346c0 SuspensionSystem::updateAbs: ABS holds a forward-rolling braked wheel whose force passes the cap
// (speed-proportional tyre model) or whose slip ratio passes the peak slip, scaled by the ABS level
pub fn updateAbs(abs: *Abs, a: Assists, rotating_forwards: bool, speed_proportional: bool, forward_force: f32, abs_cap: f32, t: Tyre, load_multiple: f32, handbrake_wheel: bool, braking_torque: f32) void {
    abs.active = false;
    if (!rotating_forwards or !(a.brake > 0.1)) return;
    if (a.handbrake > 0 and handbrake_wheel) return;
    const over = if (speed_proportional) forward_force > abs_cap else blk: {
        const excess = if (t.has_record) t.peakSlipRatio(load_multiple) - 1 else -0.9;
        break :blk t.slip.ratio > a.blend(a.abs_level) * excess + 1;
    };
    if (!over) return;
    abs.active = true;
    if (braking_torque != 0) abs.activated = true;
}

// SurfaceSimulation (W+0x3b4 current): how a loose surface takes the tyre
pub const Surface = extern struct {
    coeff_high: f32, // drag coefficient past the peak slip ratio
    coeff_mid: f32,
    coeff_low: f32, // braking past the peak
    coeff_long: f32,
    max_depth: f32, // m the tyre sinks
    max_force_per_normal: f32,
    resists_torque_factor: f32,
    density: f32,
    depth_vs_weight: f32,
    velocity_limit: f32,
    tyre_code: u32,
};

// the wheel side of the loose surface force
pub const Sinking = extern struct {
    side_velocity: f32, // SuspensionSystemWheelData+0x184
    forward_velocity: f32, // +0x188
    on_ground: bool, // +0xa0
    radius: f32, // Wheel+0x140
    width: f32, // Wheel+0x150
};

pub const SurfaceForce = extern struct { forward: f32 = 0, side: f32 = 0 };

// 0xf30f90 gravelV1: the sunk tyre ploughs the surface. Sideways the drag goes with the circle segment
// under the surface, forwards with depth x width; both capped by the ground impulse
pub fn gravel(sim: Surface, w: Sinking, t: Tyre, ground_impulse: f32) SurfaceForce {
    const forward_speed = math.maxss(sim.velocity_limit, w.forward_velocity);
    const r = w.radius;
    const depth: f32 = if (w.on_ground) sim.max_depth else 0;
    const h = r - depth;
    const chord = @sqrt(r * r - h * h); // half chord at the surface
    const segment = r * (math.asin(chord / r) * r) - chord * h;
    const coeff = coefficient(sim, t);
    const side = @abs(w.side_velocity) * (sim.density * 0.5 * w.side_velocity) * coeff * segment;
    const forward = sim.coeff_long * (0.5 * sim.density * forward_speed * @abs(forward_speed)) * (depth * w.width);
    const cap = ground_impulse * sim.max_force_per_normal;
    return .{ .forward = clampTo(forward, cap), .side = clampTo(side, cap) };
}

// the drag coefficient by how far the slip ratio is past the peak: high when driving, low when braking
fn coefficient(sim: Surface, t: Tyre) f32 {
    const g = t.set.longitudinal;
    const peak: f32 = if (t.has_record) (g.peak_x4 - g.peak_x1) + g.peak_x1 else 0.1;
    var s = t.slip.ratio;
    if (s > 1) {
        if (!t.has_record) return (sim.coeff_high - sim.coeff_mid) * 1 + sim.coeff_mid;
        s = 1;
    } else {
        s = if (s > -1) s else -1;
        if (!(s > 0)) return (sim.coeff_low - sim.coeff_mid) * unit((s + peak) / (peak - 1)) + sim.coeff_mid;
    }
    return (sim.coeff_high - sim.coeff_mid) * unit((s - peak) / (1 - peak)) + sim.coeff_mid;
}

fn unit(x: f32) f32 {
    return if (x > 1) 1 else math.maxss(x, 0);
}

fn clampTo(x: f32, cap: f32) f32 {
    return if (x > cap) cap else math.maxss(x, -cap);
}


// SuspensionSystemWheelData (T+0x10 + 0x1c0 i): the fields the tyre impulses read and write
pub const WheelData = extern struct {
    pos: [4]f32, // +0x0 m_wheelWorldPos: the hub
    rest_pos: [4]f32, // +0x10 m_wheelWorldRestPos
    spring_axis_world: [4]f32, // +0x20
    ground_normal: [4]f32, // +0x30 m_surfaceNormalCapped
    spring_axis: [4]f32, // +0x40 body frame
    height: f32, // +0x50: hub travel along the spring axis
    collision_force: [2]f32, // +0x58 side, forward
    height_if_on_ground: f32, // +0x60: where the ground puts the hub
    height_if_on_ground_next: f32, // +0x64: the same at the pose a step ahead, -1000 for none
    car_velocity: f32, // +0x68: body speed along the axis at the wheel
    bump_offset: [2]f32, // +0x78 backup, +0x7c: the road bumps' height
    external_grip: f32, // +0x98 m_externalGripMultiplier
    wheel_velocity: f32, // +0x6c: hub speed along the axis
    wheel_peak_velocity: f32, // +0x70
    ground_velocity: f32, // +0x74
    maximum_depth: f32, // +0x80
    base_tyre_spring: f32, // +0x84 tyreSpringConstant: before a puncture
    base_tyre_damping: f32, // +0x88
    tyre_spring: f32, // +0x8c activeTyreSpringConstant
    tyre_damping: f32, // +0x90 activeTyreSpringDamping
    max_depth: f32, // +0x94 activeMaximumDepth: deepest the tyre may sink
    in_air_factor: f32, // +0x9c m_wheelInAirFactor: 0 on the ground .. 1 in the air
    force_response: [12]f32, // +0xd0: axis, side, forward speed change at wheels 0..3 per unit axis impulse here
    side_response: [12]f32, // +0x100 sideForceResponse: the same per unit side impulse
    forward_response: [12]f32, // +0x130 forwardForceResponse
    side_impulse_sum: f32, // +0x160 accumulatedSideImpulseOnCar
    forward_impulse_sum: f32, // +0x164
    side_impulse: f32, // +0x168 sideImpulseOnCar
    forward_impulse: f32, // +0x16c
    torque: [2]f32, // +0x170 traction, rolling resistance
    perceived_inertia: f32, // +0x178
    impulse_on_car: f32, // +0x1a4 accumulatedImpulseOnCar
    road_impulse_on_tyre: f32, // +0x1a8 accumulatedRoadImpulseOnTyre
    rebound_rate: f32, // +0x1ac m_reboundRate: damper rate past the zone, over the slow rate
    bump_rate: f32, // +0x1b0 m_bumpRate
    side_axis: [4]f32, // +0xb0 m_wheelWorldSideAxis
    forward_axis: [4]f32, // +0xc0 m_wheelWorldForwardAxis
    on_ground: bool, // +0xa0
    tan_peak_slip: f32, // +0x17c
    peak_slip_ratio: f32, // +0x180
    side_velocity: f32, // +0x184
    forward_velocity: f32, // +0x188
    abs: Abs, // +0x190
    atc_activated: bool, // +0x192 m_atcActivatedThisUpdate
    atc_active: bool, // +0x193 m_atcCurrentlyActive
    esp_active: bool, // +0x194 m_espCurrentlyActive
    rotation_rate: f32, // +0x198 m_rotationRate
    force_request: [2]f32, // +0x19c m_forceRequestedFactorX, Z: summed request x dt
    height_before_noise: f32, // +0x54: heightIfOnGround without the bumps, clamped to the travel
};

// DynamicSuspensionParams: camber by suspension travel
pub const DynamicCamber = extern struct {
    compression_limit: f32,
    compression_offset: f32,
    compression_linearity: f32,
    extension_limit: f32,
    extension_offset: f32,
    extension_linearity: f32,
    ride_height: f32,

    // CarType::DynamicSuspensionParams::evaluate
    pub fn at(p: DynamicCamber, height: f32) f32 {
        const off = height + p.ride_height;
        if (off > 0) return p.compression_offset * math.pow(below1(@abs(off / (p.ride_height + p.compression_limit))), p.compression_linearity);
        return p.extension_offset * math.pow(below1(@abs(off / (p.ride_height + p.extension_limit))), p.extension_linearity);
    }
};

// the Wheel (W = car+0x7100 + 0x440 i) fields the tyre impulses read and write
pub const Wheel = extern struct {
    rest_position: [4]f32, // +0x10
    direction: [2]f32, // +0x20 m_directionZ, m_directionX: the wheel's heading in the body
    height_velocity: f32, // +0x4c
    peak_height_velocity: f32, // +0x50
    min_height: f32, // +0x94
    max_height: f32, // +0x98
    bump_stop_height: f32, // +0x9c
    spring_top_height: f32, // +0xa4
    slow_bump: f32, // +0xa8 N s/m
    fast_bump: f32, // +0xac
    fast_rebound: f32, // +0xb0
    slow_rebound: f32, // +0xb4
    bump_zone: f32, // +0xbc m_dampingZoneDivision, m/s
    rebound_zone: f32, // +0xc0
    anti_roll: f32, // +0xc4 m_antirollStrength
    inertia: f32, // +0x16c
    max_brake_torque: f32, // +0x178
    drive_proportion: f32, // +0x17c m_driveProportion
    applied_brake_torque: f32, // +0x3ac
    esp_brake_torque: f32, // +0x3b0 m_appliedBrakeTorqueESP
    grip_coefficient: f32, // +0x164 m_gripCoEfficient
    has_material: bool, // +0x100 m_contactMaterial not null
    metal: bool, // its tag is MET*
    skidding: [3]f32, // +0xd8 lateral, longitudinal (the game's X and Z), amount
    rotation_rate: f32, // +0x174
    generated_force: [3]f32, // +0x384 longitudinal, lateral, vertical
    upward_force: [2]f32, // +0x1ac on the suspension, on the tyre: summed
    patch_offset: [2]f32, // +0x70 m_patchOffsetX, Z: the carcass deflection of the low-speed model
    patch_offset_clamp_rate: [2]f32, // +0x78
    height: f32, // +0x80
    display_height_offset: f32, // +0x90
    grip_for_ai: [2]f32, // +0xc8 longitudinal, lateral
    contact: bool, // +0xf0 m_contactSurface not null
    ground_effect: f32, // contact surface +0x34
    surface_normal: [4]f32, // +0x130
    radius: f32, // +0x140
    width: f32, // +0x150
    camber: f32, // +0x15c
    handbrake: bool, // +0x160 m_isHandbrakeWheel
    punctured: bool, // +0x190
    tyre: Tyre, // +0x1c0
    surface_force: [4]f32, // +0x390 longitudinal, lateral, vertical, patch velocity
    load_multiple: f32, // +0x3a0
    peak_load_multiple: f32, // +0x3a4
    surface_camber: f32, // +0x3a8
    surface: Surface, // +0x3b4 m_currentSurfaceSpec
    surface_target: Surface, // +0x3e0: the surface under the wheel, blended toward
    surface_rate: Surface, // +0x40c: blend speed per unit rate
    set_target: tyre_.GraphSet, // +0x24c: the tyre graphs for that surface
    set_rate: tyre_.GraphSet, // +0x2c0
    contact_surface: u16, // +0xf0 m_contactSurface as a ground material index, ground.none: in the air
    contacted: [3]u16, // +0x108 m_contactMaterialArray: beside the wheel, under it, the other side
    contacted_count: u32, // +0x120
    outermost_contacted: u32, // +0x124: 2 left, 0 right
    max_radius: f32, // +0x14c m_maxRadius
    critical_cos: f32, // +0x154 m_criticalCosAlfa: past it the tyre's side touches
    radius_modifier: f32, // +0x1b8
    patch_point: [4]f32, // +0x60 m_patchPoint: where the low-speed carcass holds the ground, world
    temperature: [3]f32, // +0x198 tread: inner, middle, outer (by the camber side)
    wear: f32, // +0x1a4 m_tyreWear 0..1
    dirt: f32, // +0x1a8 m_tyreDirtLevel 0..1
    time_in_air: f32, // +0xec
    grip_available: f32, // +0x8c m_surfaceGripAvailable
    burst_grip: f32, // +0x168 m_gripModifierAfterTyreBurst
    has_geometry: bool, // +0xf8 m_contactGeometry not null
    brake_temperature: f32, // +0x1b4 K
    render_direction: [2]f32, // +0x28 m_renderDirectionZ, X: follows direction at a limited rate
    previous_render_direction: [2]f32, // +0x30
    last_direction: f32, // +0x48 m_lastDirection: direction X after the steering
    toe_in: f32, // +0x158
    visual_damage_angle: f32, // +0x1bc: turns the heading by twice it
    tyre_radius: f32, // +0x144
    smoothed_skidding: [2]f32, // +0xd0 X, Z
    smoothed_amount: f32, // +0xe4 m_smoothedSkidding
    visual_skid: f32, // +0xe8
    rotation_angle: [3]f32, // +0x180 now, +0x184 interpolated, +0x188 previous
    cos_up: f32, // +0x18c m_cosAngleSurfaceNormalAtWheelAndCarUp
    damper_bump_multiple: f32, // +0xb8 m_edamperBumpMultiple
};

pub const Car = extern struct {
    mass: f32, // car+0x6590
    speed: f32, // car+0x8264
    up: [4]f32, // car+0x65e0
    body: body.Body, // car+0x17c0
    assists: Assists,
    camber: [2]DynamicCamber, // setup+0x39c front, +0x3f0 rear [likely]
    start_velocity: [4]f32, // car+0xf40, the body at the frame start
};

// SuspensionSystem fields
pub const System = extern struct {
    ai_com_offset: f32, // T+0x0
    wheel_count: u32, // T+0x720 m_numberOfWheels
    ground_velocity_table: [4][4]f32, // T+0x7d0: ground axis speed change at wheel j per unit impulse at wheel i
    abs_timer: f32, // T+0x810 m_timeToNextAbsUpdate
    abs_period: f32, // T+0x814
    min_speed_for_normal_model: f32, // T+0x834: below it the low-speed tyre model
    braking_torque: [4]f32, // T+0x820 m_appliedBrakingTorque
    external_acceleration: [4]f32, // T+0x7a0
    external_angular_acceleration: [4]f32, // T+0x7b0
    car_acceleration: [4]f32, // T+0x7c0: external acceleration along each spring axis
    expectation_valid: bool, // T+0x710 m_expectation_is_valid: last frame left an expected end frame
    gear_ratio: f32, // T+0x818
    atc_filter: f32, // T+0x81c: traction control's engine torque scale, 1 off
    throttle_filter: f32, // T+0x84c m_stabilityThrottleFilter
    brake_filter: f32, // T+0x850 m_stabilityBrakeFilter
    engine_torque: f32, // T+0x854
    gear_delay: f32, // T+0x77c m_delayForGearSwitch
    wheels_on_ground: u32, // T+0x838
    ground_geometries: u32, // T+0x778 m_numGroundGeometries: not 0, no track query
};

// CarBodyState: the body at the frame start (car+0xe80) and as expected at its end (car+0xf60)
pub const Frame = extern struct {
    rows: [4][4]f32,
    vel: [4]f32, // +0xc0
    angvel: [4]f32, // +0xd0
};

pub const Impulses = extern struct {
    longitudinal: [4]f32,
    lateral: [4]f32,
    ground_effect: [4]f32, // longitudinal, from loose surfaces
};

// 0xf61090 SuspensionSystem::computeTyreImpulses for one substep. ground_impulse: the tyre's vertical
// impulse this substep, in and out (the loose-surface lift is added)
pub fn tyreImpulses(sys: *System, car: Car, data: *[4]WheelData, wheels: *[4]Wheel, dt: f32, ground_impulse: *[4]f32, grip: [4]f32) Impulses {
    const quarter = 9.81 * car.mass * 0.5 * 0.5; // carHalfWeight, a quarter of the weight
    const timer = sys.abs_timer;
    sys.abs_timer = if (0 >= timer) sys.abs_period else timer - dt;
    const update_abs = 0 >= timer;
    const low_speed = sys.min_speed_for_normal_model > car.speed;
    const crawling = 1 > car.speed;
    for (data) |*d| d.esp_active = false;
    const expected = quarter * dt;
    var out: Impulses = undefined;

    for (data, wheels, 0..) |*d, *w, i| {
        const t = &w.tyre;
        const rr = if (!(@abs(d.rotation_rate) < 1e-6)) d.rotation_rate else 0;
        const roll = rr * w.radius;
        const abs_roll = @abs(roll);
        const forward = body.dot4(body.pointVelocity(car.body, d.pos), d.forward_axis);
        const upright = !(car.up[1] < 0.7);

        var x: f32 = 0; // lateral and longitudinal force request
        var z: f32 = 0;
        var blend: [2]f32 = .{ 1.1, 1 };
        if (low_speed) {
            const c = if (abs_roll < 1) 1 else abs_roll;
            const k = @abs(d.peak_slip_ratio * c);
            w.load_multiple = 1;
            t.patch_forward = roll;
            t.wheel_forward = forward;
            t.patch_side = d.side_velocity;
            if (upright) {
                x = d.side_velocity / (c * d.tan_peak_slip);
                z = d.forward_velocity / k;
            }
            w.peak_load_multiple = if (1 < w.peak_load_multiple) w.peak_load_multiple else 1;
            blend = .{ 10, 9 };
        } else {
            const load = ground_impulse[i] / expected;
            const clamped: f32 = if (load > 4) 4 else if (!(load > 0)) 0 else load;
            w.load_multiple = clamped;
            w.peak_load_multiple = math.maxss(w.peak_load_multiple, clamped);
            const camber = cambers(car, w, d.*);
            t.setSlipRatio(forward, roll, d.side_velocity);
            t.setSlipAngle(forward, d.side_velocity);
            t.slip.direction = tyre_.slipDirection(forward, roll, d.side_velocity);
            t.forces = if (t.set.longitudinal.preshape_1 > 0)
                tyre_.ellipse(t.set, t.surface, t.slip, load, camber)
            else
                tyre_.similarity(t.set, t.surface, t.slip, t.legacy, load, camber);
            t.setSkidding(forward, roll, load);
            t.patch_forward = roll;
            t.patch_side = d.side_velocity;
            t.wheel_forward = forward;
            if (upright) {
                x = t.forces.lateral;
                z = t.forces.longitudinal;
            }
        }

        const factors = patchFactors(w, d.*, dt);
        if (blend[0] > math.maxss(blend[1], abs_roll) and crawling) {
            const k = unit2((blend[0] - math.maxss(blend[1], abs_roll)) / (blend[0] - blend[1]));
            const ax = factors[0] * soft(factors[0]) + 0.5 * w.patch_offset_clamp_rate[0];
            const az = soft(factors[1]) * factors[1] - 0.5 * w.patch_offset_clamp_rate[1];
            x = x + (ax - x) * k;
            z = z + (-az - z) * k;
        }

        const x2 = x * x;
        const across = @sqrt(nonNegative(1 - x2));
        const strength = car.assists.blend(car.assists.abs_strength);
        const atc_cap = math.maxss(strength, across);
        const abs_cap = math.maxss(across, strength);
        const rotating_forwards = !(rr < 0);
        var abs_held = false;
        if (update_abs) {
            updateAbs(&d.abs, car.assists, rotating_forwards, low_speed, z, abs_cap, t.*, w.load_multiple, w.handbrake, sys.braking_torque[i]);
            abs_held = d.abs.active;
        }
        d.atc_active = (abs_held or rotating_forwards) and -atc_cap > z;
        if (d.atc_active) d.atc_activated = true;
        d.force_request[0] = dt * x + d.force_request[0];
        d.force_request[1] = dt * z + d.force_request[1];
        if (low_speed) {
            const n2 = x2 + z * z;
            if (n2 > 1) {
                const inv = 1 / @sqrt(n2);
                x = x * inv;
                z = z * inv;
            }
        }

        var lateral: f32 = 0;
        var longitudinal: f32 = 0;
        if (w.contact) {
            const f = gravel(w.surface, .{ .side_velocity = d.side_velocity, .forward_velocity = d.forward_velocity, .on_ground = d.on_ground, .radius = w.radius, .width = w.width }, t.*, ground_impulse[i]);
            const dn = body.dot4(w.surface_normal, car.up);
            const flat: f32, const tilt: f32 = if (dn > 1) .{ 1, 0 } else if (dn > 0) .{ dn, maxGround(1 - dn * dn) } else .{ 0, 1 };
            lateral = flat * f.side;
            longitudinal = f.forward;
            const vertical = @sqrt(tilt) * @abs(f.side) + 0;
            w.surface_force = .{ longitudinal, lateral, vertical, d.forward_velocity };
            ground_impulse[i] = vertical / quarter + ground_impulse[i];
        }
        const g = expected * grip[i];
        out.lateral[i] = -x * (w.grip_for_ai[1] * g) - lateral;
        out.longitudinal[i] = w.grip_for_ai[0] * g * -z - longitudinal;
        const k: f32 = if (!w.contact or 0 > w.ground_effect) -0.0 else -w.ground_effect;
        out.ground_effect[i] = k * longitudinal;
    }
    return out;
}

// the camber the tyre sees: static + dynamic (by travel) + the surface's tilt across the wheel
fn cambers(car: Car, w: *Wheel, d: WheelData) f32 {
    const params = if (w.rest_position[2] > 0) car.camber[0] else car.camber[1];
    const dynamic = params.at(w.height + w.display_height_offset);
    const side: f32 = if (0 < w.rest_position[0]) -1 else 1;
    var camber = w.camber + side * dynamic;
    if (!w.contact) {
        w.tyre.effective_camber = 0;
        return camber;
    }
    const n: V4 = w.surface_normal;
    const tangent = body.cross(n, d.forward_axis);
    const q = tangent * tangent;
    const tangent_n = body.normalize4(tangent, (q[1] + q[3]) + (q[0] + q[2]));
    const s: V4 = d.side_axis;
    const r = s * s;
    const side_n = body.normalize4(s, (r[0] + r[2]) + (r[1] + r[3]));
    const cos = body.dot4(tangent_n, side_n);
    const tilt = body.dot4(n, side_n);
    const surface = math.acos(if (cos > 1) 1 else math.maxss(cos, -1)) * math.sign(tilt);
    camber = camber + surface;
    w.surface_camber = surface;
    w.tyre.effective_camber = camber;
    return camber;
}

// the low-speed carcass: the patch offset follows the ground and springs the request factors back
fn patchFactors(w: *Wheel, d: WheelData, dt: f32) [2]f32 {
    const z = w.patch_offset[1] - d.forward_velocity * dt;
    const x = w.patch_offset[0] - dt * d.side_velocity;
    const fz: f32, w.patch_offset[1] = if (z > 0.005) .{ 1, 0.005 } else if (!(z > -0.005)) .{ -1, -0.005 } else .{ z / 0.005, z };
    const fx: f32, w.patch_offset[0] = if (x > 0.005) .{ -1, 0.005 } else if (!(x > -0.005)) .{ 1, -0.005 } else .{ -x / 0.005, x };
    return .{ unit1(4 * d.side_velocity + fx), unit1(fz - 4 * d.forward_velocity) };
}

// 1 from 0.25 up, the factor itself below
fn soft(f: f32) f32 {
    return if (0 <= @abs(f) - 0.25) 1 else @abs(f);
}

fn unit1(x: f32) f32 {
    return if (x > 1) 1 else math.maxss(x, -1);
}

fn unit2(t: f32) f32 {
    return if (t > 1) 1 else if (0 < t) t else 0;
}

fn below1(x: f32) f32 {
    return if (x < 1) x else 1;
}

fn nonNegative(x: f32) f32 {
    return if (!(x < 0)) x else 0;
}

fn maxGround(x: f32) f32 {
    return if (0 > x) 0 else x;
}

// 0xf34820 SuspensionSystem::computeGroundImpulse: the tyre's vertical impulse per wheel. The tyre is a
// spring and damper on its depth below the ground, capped at 7.5 g; past its maximum depth the hub is
// pushed out and the ground pushed down. Wheels run in order: later ones see what earlier ones changed.
pub fn groundImpulses(sys: System, mass: f32, data: *[4]WheelData, wheels: [4]Wheel, dt: f32) [4]f32 {
    const max_impulse = 9.81 * mass * dt * 7.5;
    var out: [4]f32 = undefined;
    for (data, wheels, 0..) |*d, w, i| {
        out[i] = 0;
        const start = d.height;
        const ground = d.height_if_on_ground - d.car_velocity * dt + d.ground_velocity * dt;
        d.height_if_on_ground = ground;
        d.height = if (start > w.bump_stop_height) w.bump_stop_height else math.maxss(start, w.min_height);
        if (0 > (d.height - start) * d.wheel_velocity) stopWheel(sys, data, w, i);
        const depth = ground - d.height;
        if (depth < 0 or std.math.isNan(depth)) continue;

        d.on_ground = true;
        const closing = (d.ground_velocity - d.car_velocity) - d.wheel_velocity;
        const spring = math.minss(max_impulse, math.maxss(0, d.tyre_damping * closing * dt + d.tyre_spring * depth * dt));
        out[i] = spring + out[i];
        const per_impulse = hubResponse(w);
        d.wheel_velocity = spring * per_impulse + d.wheel_velocity;
        if (depth > d.max_depth) {
            d.height = d.height_if_on_ground - d.max_depth;
            if (closing > 0) {
                const so_far = out[i];
                var need = closing / (sys.ground_velocity_table[i][i] + per_impulse);
                if (need + so_far > max_impulse) need = max_impulse - so_far;
                if (need > 0) {
                    out[i] = so_far + need;
                    d.wheel_velocity = d.wheel_velocity + per_impulse * need;
                    for (data, sys.ground_velocity_table[i]) |*o, k| o.ground_velocity = o.ground_velocity - k * need;
                }
            }
        }
        d.road_impulse_on_tyre = out[i] + d.road_impulse_on_tyre;
    }
    return out;
}

// hub speed change per unit impulse: the mass of a solid disk with the wheel's inertia, 2 I / r^2
fn hubResponse(w: Wheel) f32 {
    return 1 / ((w.inertia + w.inertia) / (w.radius * w.radius));
}

// SuspensionSystem::forceWheelVelocityToZero: the travel clamp stopped the hub; the impulse that stops it
// moves the body at every wheel (the first wheel_count of them)
fn stopWheel(sys: System, data: *[4]WheelData, w: Wheel, i: usize) void {
    const d = &data[i];
    const impulse = d.wheel_velocity / (d.force_response[i] + hubResponse(w));
    d.impulse_on_car = d.impulse_on_car + impulse;
    for (data[0..@min(sys.wheel_count, 4)], d.force_response[0..@min(sys.wheel_count, 4)]) |*o, r| {
        const dv = r * impulse;
        o.car_velocity = o.car_velocity + dv;
        o.wheel_velocity = o.wheel_velocity - dv;
    }
    d.wheel_velocity = 0;
}

// the spring and bump stop setup of one axle (setup+0x2f0.., front first in Springs.axles)
pub const Axle = extern struct {
    max_spring_length: f32,
    max_spring_load: f32,
    mid_spring_length: f32, // the spring is two straight lines, through the mid and the max point
    mid_spring_load: f32,
    damper_length: f32,
    bump_stop_max_compression: f32,
    bump_stop_max_rate: f32,
    bump_stop_mid_compression: f32,
    bump_stop_mid_rate: f32,
};

pub const Damage = extern struct { spring: f32, anti_roll: f32, damper: f32 }; // car+0x11f0c + 0x20 i, scales

pub const Springs = extern struct {
    axles: [2]Axle, // front, rear
    load_scale: f32, // setup+0x5a0 m_downgradedMassScalar: scales both spring loads
    damage: [4]Damage,
};

// 0xf34fe0 SuspensionSystem::calculateLoadImpulses: spring, bump stop, damper and anti-roll impulse of
// each wheel, upwards on the car; nothing when the system has no wheels
pub fn loadImpulses(sys: System, springs: Springs, data: [4]WheelData, wheels: [4]Wheel, dt: f32, out: *[4]f32) void {
    const rear_mean = (data[0].height + data[1].height) * 0.5;
    const front_mean = (data[2].height + data[3].height) * 0.5;
    for (0..@min(sys.wheel_count, 4)) |i| {
        const d = data[i];
        const w = wheels[i];
        const dmg = springs.damage[i];
        const a = if (i <= 1) springs.axles[1] else springs.axles[0];
        const power = math.log(a.bump_stop_max_rate / a.bump_stop_mid_rate) / math.log(a.bump_stop_max_compression / a.bump_stop_mid_compression);
        const scale = a.bump_stop_max_rate / math.pow(a.bump_stop_max_compression, power);
        const from_top = math.maxss(0, w.spring_top_height - d.height);
        const to_bump_stop = math.minss(w.bump_stop_height - d.height, from_top);
        const compressed = math.minss(a.max_spring_length, a.damper_length) - from_top;
        const spring = dmg.spring * springLoad(a, springs.load_scale, compressed);
        const stop = if (a.bump_stop_max_compression > to_bump_stop) scale * math.pow(a.bump_stop_max_compression - to_bump_stop, power) else 0;
        const bump = if (!(d.wheel_velocity <= 0)) stop else 0;
        const anti_roll = (d.height - (if (i <= 1) rear_mean else front_mean)) * w.anti_roll * dmg.anti_roll;
        out[i] = (((damping(d, w, dmg.damper) + spring) + anti_roll) + bump) * dt;
    }
}

// CarType::calculate{Front,Rear}SpringLoadForCompression: bilinear through the mid and the max point
fn springLoad(a: Axle, load_scale: f32, compressed: f32) f32 {
    const mid = a.mid_spring_load * load_scale;
    const max = a.max_spring_load * load_scale;
    if (0 > compressed) return 0;
    if (a.mid_spring_length > compressed) return compressed * mid / a.mid_spring_length;
    if (a.max_spring_length > compressed) return (max - mid) * ((compressed - a.mid_spring_length) / (a.max_spring_length - a.mid_spring_length)) + mid;
    return max;
}

// slow rate inside the zone, the rate ratio past it, bump and rebound apart
fn damping(d: WheelData, w: Wheel, scale: f32) f32 {
    const v = d.wheel_velocity;
    if (v > 0) {
        const rate = scale * w.slow_bump;
        if (!(v > w.bump_zone)) return v * rate;
        return (v * d.bump_rate + w.bump_zone * (1 - d.bump_rate)) * rate;
    }
    const rate = scale * w.slow_rebound;
    if (!(-v > w.rebound_zone)) return v * rate;
    return (v * d.rebound_rate - w.rebound_zone * (1 - d.rebound_rate)) * rate;
}

// 0xf33760 SuspensionSystem::calculateWheelPosition: the wheel's axes and hub in the world
pub fn wheelPosition(sys: System, b: body.RigidBody, d: *WheelData, w: Wheel) void {
    const axis = body.transform(b.rows, d.spring_axis);
    d.spring_axis_world = axis;
    d.side_axis = body.transform(b.rows, .{ w.direction[0], 0, -w.direction[1], 0 });
    d.forward_axis = body.transform(b.rows, .{ w.direction[1], 0, w.direction[0], 0 });
    const rest = body.transform(b.rows, .{ w.rest_position[0], (w.rest_position[1] - w.radius) - sys.ai_com_offset, w.rest_position[2], 1 });
    d.rest_pos = rest;
    d.pos = rest + @as(V4, @splat(d.height)) * axis;
}

// 0xf4fab0 SuspensionSystem::createForceResponseTables: the speed change at every hub per unit impulse
// along each hub's axes, by a test impulse of 1e-4 m; also the body speeds at the hubs now
pub fn forceResponseTables(sys: *System, b: body.RigidBody, data: *[4]WheelData, wheels: [4]Wheel) void {
    const n = @min(sys.wheel_count, 4);
    const at: V4 = b.rows[3];
    var base: [4]V4 = undefined;
    for (data[0..n], wheels[0..n], base[0..n]) |*d, w, *v| {
        v.* = body.cross(b.angvel, @as(V4, d.pos) - at) + @as(V4, b.vel);
        d.car_velocity = body.dot4(d.spring_axis_world, v.*);
        d.side_velocity = body.dot4(d.side_axis, v.*);
        d.forward_velocity = body.dot4(d.forward_axis, v.*) - w.radius * d.rotation_rate;
    }
    const eps = b.mass * 0.0001;
    for (0..3) |k| {
        for (0..n) |u| {
            const src = &data[u];
            const dir: V4 = switch (k) {
                0 => src.spring_axis_world,
                1 => src.side_axis,
                else => src.forward_axis,
            };
            const j = dir * @as(V4, @splat(eps));
            const angvel = body.angularChange(b, j, @as(V4, src.pos) - at) + @as(V4, b.angvel);
            const vel = @as(V4, @splat(b.inverse_mass)) * j + @as(V4, b.vel);
            const row = switch (k) {
                0 => &src.force_response,
                1 => &src.side_response,
                else => &src.forward_response,
            };
            for (data[0..n], wheels[0..n], base[0..n], 0..) |o, w, v0, t| {
                const v = body.cross(angvel, @as(V4, o.pos) - at) + vel;
                row[t] = (body.dot4(o.spring_axis_world, v) - o.car_velocity) / eps;
                row[4 + t] = (body.dot4(o.side_axis, v) - o.side_velocity) / eps;
                row[8 + t] = ((body.dot4(o.forward_axis, v) - w.radius * o.rotation_rate) - o.forward_velocity) / eps;
                if (k == 0) {
                    const dv = (v - v0) * @as(V4, @splat(1 / eps));
                    const axis: V4 = o.spring_axis_world;
                    const across = dv - axis * @as(V4, @splat(body.dot4(axis, dv)));
                    sys.ground_velocity_table[u][t] = body.dot4(across, src.ground_normal) / body.dot4(axis, src.ground_normal);
                }
            }
        }
    }
}

// 0xf54bd0 SuspensionSystem::copyStateBackToCar: the substeps' summed wheel impulses as forces on the body
// (the tyre force at the roll centre height for anti-dive and anti-squat), then the body step and the
// external accelerations. roll_centre: AdvancedRollCentre front (setup+0x11c) and rear (+0x128)
pub fn copyStateBack(sys: System, b: *body.RigidBody, data: *[4]WheelData, wheels: *[4]Wheel, roll_centre: [2][3]f32, dt: f32) void {
    const inv = 1 / dt;
    const up0: V4 = b.rows[1];
    const pos0: V4 = b.rows[3];
    for (data[0..@min(sys.wheel_count, 4)], wheels[0..@min(sys.wheel_count, 4)], 0..) |*d, *w, i| {
        const side = d.side_impulse_sum * inv;
        const forward = d.forward_impulse_sum * inv;
        w.generated_force[1] = side;
        w.generated_force[0] = forward;
        const f = @as(V4, @splat(side)) * @as(V4, d.side_axis) + @as(V4, @splat(forward)) * @as(V4, d.forward_axis);
        const rc = if (i > 1) roll_centre[0] else roll_centre[1];
        const r0: V4 = b.rows[0];
        const r1: V4 = b.rows[1];
        const r2: V4 = b.rows[2];
        const at: V4 = d.pos;
        const h = -(rc[2] * w.height + rc[1]);
        const fz = body.dot4(f, r2);
        const fx = body.dot4(f, r0);
        const k = body.dot4(pos0 - at, up0) * rc[0];
        const lever = h / w.rest_position[0];
        body.applyForce(b, r2 * @as(V4, @splat(fz)), (r1 * @as(V4, @splat(k)) + at) - @as(V4, b.rows[3]));
        body.applyForce(b, (r0 + @as(V4, @splat(lever)) * r1) * @as(V4, @splat(fx)), at - @as(V4, b.rows[3]));
        const impulse = d.impulse_on_car;
        d.wheel_velocity = d.wheel_velocity - fx * lever * dt / ((w.inertia + w.inertia) / (w.radius * w.radius));
        body.applyForce(b, @as(V4, @splat(impulse)) * @as(V4, d.spring_axis_world) * @as(V4, @splat(inv)), at - @as(V4, b.rows[3]));
        w.generated_force[2] = impulse / dt;
        w.height = d.height;
        w.height_velocity = d.wheel_velocity;
        w.peak_height_velocity = d.wheel_peak_velocity;
        w.rotation_rate = d.rotation_rate;
        const sunk = d.height_if_on_ground - d.maximum_depth;
        const shown = if (!(sunk > d.height)) d.height else if (sunk > w.max_height) w.max_height else math.maxss(sunk, w.min_height);
        w.display_height_offset = shown - d.height;
        const road = d.road_impulse_on_tyre * inv;
        w.upward_force[0] = impulse * inv + w.upward_force[0];
        w.upward_force[1] = w.upward_force[1] + road;
        d.impulse_on_car = 0;
        d.road_impulse_on_tyre = 0;
        d.side_impulse_sum = 0;
        d.forward_impulse_sum = 0;
        d.side_impulse = 0;
        d.forward_impulse = 0;
        const axis: V4 = d.spring_axis_world;
        const normal: V4 = d.ground_normal;
        const along = body.dot4(axis, normal);
        body.applyForce(b, @as(V4, @splat(road / along)) * (normal - @as(V4, @splat(along)) * axis), at - @as(V4, b.rows[3]));
    }
    body.integrate(b, dt);
    const acc = @as(V4, sys.external_acceleration) * @as(V4, @splat(dt));
    b.vel = acc + @as(V4, b.vel);
    b.angvel = @as(V4, sys.external_angular_acceleration) * @as(V4, @splat(dt)) + @as(V4, b.angvel);
    b.rows[3] = @as(V4, @splat(0.5 * dt)) * acc + @as(V4, b.rows[3]);
}

fn hubVelocity(b: body.RigidBody, at: V4) V4 {
    return body.cross(b.angvel, at - @as(V4, b.rows[3])) + @as(V4, b.vel);
}

// 0xf5a7e0 SuspensionSystem::subSetupForFrame: the external accelerations since the frame start, which the body
// is set back to, and the ground heights a step ahead by ground.heightChecks (doHeightChecks) on the body
// integrated with the forces summed so far
pub fn subSetup(sys: *System, b: *body.RigidBody, start: Frame, data: *[4]WheelData, wheels: *[4]Wheel, dt: f32, ground: anytype) void {
    const n = @min(sys.wheel_count, 4);
    for (data[0..n], wheels[0..n]) |*d, w| wheelPosition(sys.*, b.*, d, w);
    for (data[0..n]) |*d| d.car_velocity = body.dot4(hubVelocity(b.*, d.pos), d.spring_axis_world);
    const inv = 1 / dt;
    const dv = @as(V4, b.vel) - @as(V4, start.vel);
    const dw = @as(V4, b.angvel) - @as(V4, start.angvel);
    b.vel = start.vel;
    b.angvel = start.angvel;
    sys.external_acceleration = dv * @as(V4, @splat(inv));
    sys.external_angular_acceleration = @as(V4, @splat(inv)) * dw;
    for (data[0..n], 0..) |d, i| sys.car_acceleration[i] = (d.car_velocity - body.dot4(hubVelocity(b.*, d.pos), d.spring_axis_world)) * inv;

    const saved = b.*;
    body.integrate(b, dt);
    for (data[0..n], wheels[0..n]) |*d, w| wheelPosition(sys.*, b.*, d, w);
    ground.heightChecks(sys, b.*, data, wheels, dt);
    var predicted: [4]f32 = undefined;
    for (data[0..n], 0..) |d, i| predicted[i] = d.height_if_on_ground;
    b.* = saved;
    for (data[0..n], wheels[0..n]) |*d, w| wheelPosition(sys.*, b.*, d, w);
    forceResponseTables(sys, b.*, data, wheels.*);
    for (data[0..n], predicted[0..n]) |*d, new| {
        const old = d.height_if_on_ground_next;
        d.height_if_on_ground = old;
        d.height_if_on_ground_next = new;
        d.ground_velocity = if (new == -1000 or old == -1000) 0 else ((new - old) + d.car_velocity * dt) / dt;
    }
}

// 0xf5c590 SuspensionSystem::setupForFrame: the tyre springs for punctures and bursts, the wheels moved by how far
// the body ended from where last frame expected it, the sums cleared; then subSetupForFrame.
// tyre_burst: car+0x11fd0
pub fn setupForFrame(sys: *System, b: *body.RigidBody, start: Frame, end: Frame, tyre_burst: [4]bool, data: *[4]WheelData, wheels: *[4]Wheel, dt: f32, ground: anytype) void {
    const n = @min(sys.wheel_count, 4);
    for (data[0..n], wheels[0..n], tyre_burst[0..n], sys.braking_torque[0..n]) |*d, *w, burst, torque| {
        d.tyre_spring = d.base_tyre_spring;
        d.tyre_damping = d.base_tyre_damping;
        d.max_depth = d.maximum_depth;
        if (w.punctured) {
            d.tyre_spring = d.base_tyre_spring * 0.02;
            d.tyre_damping = d.base_tyre_damping * 0.1;
        }
        if (burst) d.max_depth = 0;
        d.in_air_factor = unit(if (w.contact) d.in_air_factor - dt else (dt + dt) + d.in_air_factor);
        d.force_request = .{ 0, 0 };
        d.atc_activated = d.atc_active;
        d.abs.activated = torque != 0 and d.abs.active;
        d.perceived_inertia = w.inertia;
        w.peak_load_multiple = 0;
    }
    if (sys.expectation_valid) {
        const actual = body.Body{ .angvel = start.angvel, .vel = start.vel, .com = b.rows[3] };
        const expected = body.Body{ .angvel = end.angvel, .vel = end.vel, .com = end.rows[3] };
        for (data[0..n], wheels[0..n]) |*d, *w| {
            const actual_axis = body.transform(b.rows, d.spring_axis);
            const expected_axis = body.transform(end.rows, d.spring_axis);
            const pos: V4 = .{ w.rest_position[0], w.rest_position[1], w.rest_position[2], 1 };
            const expected_pos = body.transform(end.rows, pos);
            const actual_pos = body.transform(b.rows, pos);
            const movement = body.dot4(actual_pos - expected_pos, actual_axis);
            const increase = body.dot4(actual_axis, body.pointVelocity(actual, actual_pos)) - body.dot4(expected_axis, body.pointVelocity(expected, expected_pos));
            w.height_velocity = w.height_velocity - increase;
            w.height = w.height - movement;
            d.height = w.height;
            if (d.height_if_on_ground_next != -1000) d.height_if_on_ground_next = d.height_if_on_ground_next - movement;
        }
    }
    for (data[0..n], wheels[0..n]) |*d, *w| {
        w.upward_force = .{ 0, 0 };
        w.generated_force = .{ 0, 0, 0 };
        d.impulse_on_car = 0;
        d.side_impulse_sum = 0;
        d.forward_impulse_sum = 0;
        d.side_impulse = 0;
        d.forward_impulse = 0;
        d.torque = .{ 0, 0 };
        d.collision_force = .{ 0, 0 };
        d.road_impulse_on_tyre = 0;
        d.on_ground = false;
        d.height = w.height;
        d.wheel_velocity = w.height_velocity;
        d.wheel_peak_velocity = w.height_velocity;
    }
    subSetup(sys, b, start, data, wheels, dt, ground);
}

// the car fields traction control reads
pub const Traction = extern struct {
    auto_gearbox: bool, // car+0x70bc DriveTrain.m_autoGearboxEnabled: the standard clutch rates, else the sequential ones
    upshift_cut: [2]f32, // ClutchRates.m_timeOffThrottleGearUp: setup+0x248, +0x264
    upshift_ramp: [2]f32, // m_timeOnThrottleATCTime: +0x24c, +0x268
    since_upshift: f32, // car+0x5a4 m_timeSinceLastGearUp
    manual_clutch: f32, // car+0x6668 m_controls[MANUAL_CLUTCH]
    final_clutch: f32, // car+0x667c m_controls[FINAL_CLUTCH]
    strength: [2]f32, // car+0x6cec m_atc_strength_on, off
    control: f32, // car+0x54 m_atc_control
};

const mph20: f32 = @bitCast(@as(u32, 0x410f0d85)); // neMPHToMPS(20)
const collision_limit = 10; // .data 0x32ec420, a tweak

// 0xf63310 SuspensionSystem::subUpdate: one substep of the wheels; the load, ground and tyre impulses, each through
// the force response tables, and the drivetrain. side: the drivetrain's view of the car, its wheel fields synced here
pub fn subUpdate(sys: *System, car: Car, springs: Springs, traction: Traction, data: *[4]WheelData, wheels: *[4]Wheel, d: *drivetrain.Drivetrain, side: *drivetrain.CarSide, dt: f32, grip: [4]f32, engine_torque: f32) void {
    const n = @min(sys.wheel_count, 4);
    const rate = @sqrt(body.dot4(car.start_velocity, car.start_velocity)) * dt * 5 / mph20;
    for (wheels[0..n]) |*w| surfaceTransition(w, rate);
    const torque = tractionLimit(sys, car, traction, wheels.*, dt, engine_torque);

    const gravity: V4 = .{ 0, -9.81, 0, 0 };
    for (data[0..n], sys.car_acceleration[0..n]) |*x, a| {
        const dv = a * dt;
        x.car_velocity = x.car_velocity + dv;
        x.wheel_velocity = body.dot4(x.spring_axis_world, gravity) * dt + (x.wheel_velocity - dv);
    }
    var load: [4]f32 = undefined;
    loadImpulses(sys.*, springs, data.*, wheels.*, dt, &load);
    for (data[0..n]) |*x| x.height = dt * x.wheel_velocity + x.height;
    for (0..n) |i| {
        data[i].impulse_on_car = data[i].impulse_on_car + load[i];
        for (0..n) |j| {
            const a = data[i].force_response[j] * load[i];
            data[j].car_velocity = data[j].car_velocity + a;
            data[j].wheel_velocity = data[j].wheel_velocity - a;
        }
        const w = wheels[i];
        data[i].wheel_velocity = data[i].wheel_velocity - load[i] / ((w.inertia + w.inertia) / (w.radius * w.radius));
    }
    var ground = groundImpulses(sys.*, car.mass, data, wheels.*, dt);
    var imp = tyreImpulses(sys, car, data, wheels, dt, &ground, grip);

    for (data[0..n], wheels[0..n], 0..) |*x, w, i| {
        const cs = if (x.side_velocity * x.collision_force[0] < collision_limit) x.collision_force[0] else 0;
        const cf = if ((w.radius * x.rotation_rate + x.forward_velocity) * x.collision_force[1] < collision_limit) x.collision_force[1] else 0;
        const response = x.side_response[4 + i];
        if (@abs(response) > 1e-6) {
            const q = -x.side_velocity / response; // the impulse that stops the sliding
            imp.lateral[i] = if (0 > q) math.maxss(q, imp.lateral[i]) else math.minss(q, imp.lateral[i]);
        }
        x.side_impulse = imp.lateral[i] + cs;
        x.forward_impulse = imp.longitudinal[i] + cf;
        x.side_impulse_sum = x.side_impulse_sum + x.side_impulse;
        x.forward_impulse_sum = x.forward_impulse_sum + x.forward_impulse;
    }
    const cap = car.mass * 9.81;
    for (data[0..n], wheels[0..n], 0..) |*x, w, i| {
        x.torque[0] = -(imp.longitudinal[i] + imp.ground_effect[i]) * w.radius / dt;
        const gl = ground[i] / dt;
        const l = if (gl > cap) cap else math.maxss(gl, 0);
        const roll = if (w.tyre.has_record) w.tyre.rolling_resistance else 0;
        x.torque[1] = l * roll * w.radius * math.sign(-x.rotation_rate);
    }

    toSide(sys.*, data.*, wheels.*, car, side);
    drivetrain.integrate(d, side, torque, dt, true);
    for (data, wheels, 0..) |*x, *w, i| {
        x.rotation_rate = side.rotation_rate[i];
        x.forward_velocity = side.forward_velocity[i];
        w.applied_brake_torque = side.applied_brake_torque[i];
    }
    if (d.has_wobbled) {
        d.has_wobbled = false;
        side.since_rev_cut = 0;
    }

    for (0..n) |i| {
        const si = data[i].side_impulse;
        const fi = data[i].forward_impulse;
        const li = load[i];
        for (0..n) |j| {
            const r = &data[i];
            const a = r.side_response[j] * si + r.forward_response[j] * fi;
            data[j].car_velocity = data[j].car_velocity + a;
            data[j].wheel_velocity = data[j].wheel_velocity - a;
            data[j].side_velocity = data[j].side_velocity + ((r.side_response[4 + j] * si + r.force_response[4 + j] * li) + r.forward_response[4 + j] * fi);
            data[j].forward_velocity = data[j].forward_velocity + ((r.side_response[8 + j] * si + r.force_response[8 + j] * li) + r.forward_response[8 + j] * fi);
        }
    }
    for (data) |*x| x.wheel_peak_velocity = math.maxss(x.wheel_peak_velocity, x.wheel_velocity); // all four, whatever n
}

// the drivetrain's copy of the wheels
fn toSide(sys: System, data: [4]WheelData, wheels: [4]Wheel, car: Car, side: *drivetrain.CarSide) void {
    for (data, wheels, 0..) |x, w, i| {
        side.traction_torque[i] = x.torque[0];
        side.rolling_torque[i] = x.torque[1];
        side.rotation_rate[i] = x.rotation_rate;
        side.forward_velocity[i] = x.forward_velocity;
        side.abs_active[i] = x.abs.active;
        side.pos[i] = x.pos;
        side.forward_axis[i] = x.forward_axis;
        side.radius[i] = w.radius;
        side.max_brake_torque[i] = w.max_brake_torque;
    }
    side.braking_torque = sys.braking_torque;
    side.gear_ratio = sys.gear_ratio;
    side.body = car.body;
}

// 0xf56110 Wheel::updateSurfaceTransition: the tyre graphs and the surface move toward the surface under the wheel;
// off the ground they jump there
fn surfaceTransition(w: *Wheel, rate: f32) void {
    if (!w.contact) {
        w.tyre.set = w.set_target;
        return;
    }
    approach(29, @ptrCast(&w.tyre.set), @ptrCast(&w.set_target), @ptrCast(&w.set_rate), rate);
    approach(10, @ptrCast(&w.surface), @ptrCast(&w.surface_target), @ptrCast(&w.surface_rate), rate); // the floats, not the tyre code
}

fn approach(comptime n: usize, cur: *[n]f32, target: *const [n]f32, k: *const [n]f32, rate: f32) void {
    for (cur, target, k) |*c, t, r| {
        const step = @abs(r * rate);
        c.* = if (t > c.*) math.minss(c.* + step, t) else math.maxss(c.* - step, t);
    }
}

// the engine torque after traction control: cut while the clutch bites after an upshift, and as the driven
// wheels pass their peak slip
fn tractionLimit(sys: *System, car: Car, tr: Traction, wheels: [4]Wheel, dt: f32, engine_torque: f32) f32 {
    const k: usize = if (tr.auto_gearbox) 0 else 1;
    const ramp = tr.upshift_ramp[k];
    const cut = tr.upshift_cut[k];
    var shift: f32 = 0;
    if (ramp > 0 and cut + ramp > tr.since_upshift) {
        const f = (tr.since_upshift - cut) / ramp;
        shift = if (f > 1) 0 else if (f > 0) 1 - f else 1;
    }
    const speed = unit(car.speed / 6);
    const strength = math.maxss((tr.strength[0] - tr.strength[1]) * tr.control + tr.strength[1], unit(1 - tr.manual_clutch) * shift);
    const ratio = sys.gear_ratio;
    if (!(engine_torque > 0) or ratio == 0) {
        sys.atc_filter = 1;
        return engine_torque * sys.throttle_filter;
    }
    const quarter = 0.25 * car.mass * 9.81;
    const n = @min(sys.wheel_count, 4);
    var peak_torque: f32 = ratio * 0;
    var max_slip: f32 = 0;
    if (n != 0) {
        var sum: f32 = undefined;
        var slip: f32 = undefined;
        for (wheels[0..n], 0..) |w, i| {
            const peak = w.tyre.set.lateral.peak_y1 * quarter * w.radius;
            if (i == 0) {
                sum = if (w.drive_proportion > 0) peak + 0 else 0;
            } else if (w.drive_proportion > 0) sum = sum + peak;
            const x1 = w.tyre.set.longitudinal.peak_x1;
            const peak_slip = (w.tyre.set.longitudinal.peak_x4 - x1) + x1;
            const r = if (w.tyre.has_record) (peak_slip + w.tyre.slip.ratio) / (peak_slip - 1) else (0.1 + w.tyre.slip.ratio) / -0.9;
            slip = if (i == 0) unit(r) else math.maxss(slip, unit(r));
        }
        peak_torque = ratio * sum;
        max_slip = slip;
    }
    const clutch = -0.95 * tr.final_clutch + 0.95;
    const threshold = (unit(1 - clutch) - 1) * speed + 1;
    const step = dt * 50;
    var level = unit(if (max_slip > threshold) sys.atc_filter - step else step + sys.atc_filter);
    const q = (clutch * (@abs(peak_torque) - engine_torque) + engine_torque) / engine_torque;
    level = math.minss(level, q + speed * (1 - q));
    sys.atc_filter = level;
    return math.minss(sys.throttle_filter, strength * (level - 1) + 1) * engine_torque;
}

// the driver's controls and the car fields update reads
pub const Controls = extern struct {
    brake: f32, // car+0x6dd8
    handbrake: f32, // car+0x6670
    steering: f32, // car+0x6ddc
    clutch: f32, // car+0x667c m_controls[FINAL_CLUTCH]
    handbrake_strength: f32, // setup+0x46c m_handbrake_strength_multiple
    stability: f32, // car+0x6dc4 m_stabilityControl
    esp_strength: [2]f32, // car+0x6d0c m_esp_strength_on, off
    brake_scale: [4]f32, // damage, car+0x11f24 + 0x20 i
    altitude: f32, // car+0x1074, the mid frame height
    sea_level_raw: u64,
};

// DriveTrain (car+0x7028) gear state, and the setup's clutch times
pub const Gearbox = extern struct {
    ratios: [11]f32, // +0x34
    previous: i32, // +0x8c
    gear: i32, // +0x90
    auto_clutch: bool, // +0x95
    seamless: bool, // +0x99 m_seamlessShift: the ratio blends between gears
    since_change: f32, // +0xa0
    shift_time: [2]f32, // +0xc4 auto gearbox, +0xc8 manual
    clutch_delay: [2]f32, // ClutchRates.m_autoClutchGearDelay: standard setup+0x244, sequential +0x260
};

// everything update reaches outside the suspension
pub const Vehicle = extern struct {
    body: *body.RigidBody,
    car: Car,
    springs: Springs,
    traction: Traction,
    controls: Controls,
    gearbox: Gearbox,
    start: Frame, // car+0xe80
    end: Frame, // car+0xf60
    tyre_burst: [4]bool, // car+0x11fd0
    roll_centre: [2][3]f32, // setup+0x11c front, +0x128 rear
    drivetrain: *drivetrain.Drivetrain,
    side: *drivetrain.CarSide,
    engine: *engine.Engine,
    motion: engine.Motion,
};

// 0xf651e0 SuspensionSystem::update: a frame of the wheels, the engine and the drivetrain in substeps of about 1 ms
pub fn update(sys: *System, v: Vehicle, data: *[4]WheelData, wheels: *[4]Wheel, dt: f32, ground: anytype) void {
    const count = math.truncate(dt / 0.001);
    const steps: u32 = if (count <= 0) 1 else @intCast(count);
    const nf: f32 = @floatFromInt(steps);
    const h = dt / nf;
    setupForFrame(sys, v.body, v.start, v.end, v.tyre_burst, data, wheels, dt, ground);
    const n = @min(sys.wheel_count, 4);
    for (data[0..n], wheels[0..n]) |*x, w| {
        x.rebound_rate = (0 * w.slow_rebound + w.fast_rebound) / w.slow_rebound;
        x.bump_rate = w.fast_bump / w.slow_bump;
    }
    const grip = gripMultiples(v, data.*, wheels.*);
    const c = v.controls;
    const hb_strength = if (c.handbrake_strength > 4) 4 else math.maxss(c.handbrake_strength, 0);
    const hb = unit(c.handbrake);
    for (wheels, c.brake_scale, &sys.braking_torque) |w, scale, *t| {
        var b = unit(c.brake);
        if (w.handbrake) b = math.maxss(b, hb_strength * hb);
        t.* = b * w.max_brake_torque * scale;
    }

    const e = v.engine;
    const prev_rate = math.minss(e.rate, 1.1 * e.rev_limit);
    const rho = aero.airDensity(c.altitude, c.sea_level_raw);
    const torque = engine.update(e, v.motion, dt, if (sys.gear_ratio == 0) 1 else c.clutch, rho);
    sys.engine_torque = torque;
    const closed = engine.power(e.*, v.motion, 0, c.clutch) / math.maxss(20, @abs(e.rate));
    shiftGear(sys, v.gearbox, v.traction.auto_gearbox, dt);

    for (wheels) |*w| w.esp_brake_torque = 0;
    sys.throttle_filter = 1;
    const esp = (c.esp_strength[0] - c.esp_strength[1]) * c.stability + c.esp_strength[1];
    if (esp > 0 and 0.01 > c.handbrake) {
        const torque_closed = (e.starter.max_torque - closed) * e.starter.weight + closed;
        stabilityAssist(sys, v, data.*, wheels, torque, torque_closed, h, esp);
    }

    v.side.engine_rate_sum = e.rate; // car+0x6ff4 is both
    v.side.since_rev_cut = e.since_rev_cut; // and engine+0x184
    for (0..steps) |_| subUpdate(sys, v.car, v.springs, v.traction, data, wheels, v.drivetrain, v.side, h, grip, torque);
    e.since_rev_cut = v.side.since_rev_cut;
    e.rate = v.side.engine_rate_sum / nf;
    v.side.engine_rate_sum = e.rate;
    const inert = (math.minss(e.rate, 1.1 * e.rev_limit) - prev_rate) * e.inertia / dt;
    e.load = if (c.clutch == 1) 0 else unit(@abs(torque - inert) / e.max_torque);
    e.applied_torque = inert;
    e.current_torque = torque;

    var on_ground: u32 = 0;
    for (data[0..n], wheels[0..n]) |*x, *w| {
        x.force_request = .{ x.force_request[0] / dt, x.force_request[1] / dt };
        const long, const lat = w.tyre.skid;
        w.skidding = .{ lat, long, math.minss(@sqrt(lat * lat + long * long), 1) };
        if (w.contact) on_ground += 1;
    }
    sys.wheels_on_ground = on_ground;
    copyStateBack(sys.*, v.body, data, wheels, v.roll_centre, dt);
    sys.expectation_valid = true;
    for (data) |*x| x.bump_offset[0] = x.bump_offset[1];
}

// the gear ratio the wheels see: blended through a seamless shift, and switched once the auto clutch's delay is out
fn shiftGear(sys: *System, g: Gearbox, auto_gearbox: bool, dt: f32) void {
    const shift_time = if (auto_gearbox) g.shift_time[0] else g.shift_time[1];
    const to = g.ratios[@intCast(g.gear)];
    var ratio = to;
    if (shift_time > 0 and g.seamless) {
        const from = g.ratios[@intCast(g.previous)];
        ratio = (to - from) * unit(g.since_change / shift_time) + from;
    }
    if (ratio == sys.gear_ratio) {
        sys.gear_delay = 0;
        return;
    }
    var t = sys.gear_delay;
    if (0 >= t) t = if (!g.auto_clutch) 0 else if (auto_gearbox) g.clutch_delay[0] else g.clutch_delay[1];
    t = t - dt;
    sys.gear_delay = t;
    if (0 >= t or g.seamless and shift_time >= g.since_change) sys.gear_ratio = ratio;
}

// 0xf37160 SuspensionSystem::computeWheelGripMultiple: the grip coefficient, less on slopes and banked ground;
// below walking pace only while driving off
fn gripMultiples(v: Vehicle, data: [4]WheelData, wheels: [4]Wheel) [4]f32 {
    const c = v.controls;
    const apply = v.car.speed > 0.5 or v.engine.throttle > 0.1 and 0.01 > c.handbrake and 0.1 > c.brake;
    var out: [4]f32 = undefined;
    for (data, wheels, &out) |x, w, *g| {
        if (!w.contact) {
            g.* = 0;
        } else if (!apply) {
            g.* = 1;
        } else {
            var grip = w.grip_coefficient;
            if (w.has_material and (!w.metal or 5 > v.car.speed)) {
                const y = w.surface_normal[1] - 0.8192;
                if (0 > y) grip = grip * math.maxss(0.2, y * 5 + 1);
            }
            const g2 = x.external_grip * grip;
            const cos = body.dot4(v.car.up, w.surface_normal);
            g.* = if (0.96 > cos) cos * cos * g2 else g2;
        }
    }
    return out;
}

// the stability assist's choice of the 16 dry runs: brakes as they are or off, throttle as it is or closed,
// one wheel braked fully
const Run = struct {
    closed_throttle: bool,
    brakes_off: bool,
    wheel: usize,

    fn of(k: usize) Run {
        return .{ .closed_throttle = k >= 8, .brakes_off = k & 4 != 0, .wheel = k & 3 }; // the table at 0x27330a0
    }
};

// the stability assist (in update): dry runs of the drivetrain for each choice, scored by the yaw the tyres
// would give; the best one's brake and throttle cut, scaled by how far front and rear slip apart
fn stabilityAssist(sys: *System, v: Vehicle, data: [4]WheelData, wheels: *[4]Wheel, torque: f32, torque_closed: f32, h: f32, esp: f32) void {
    var hub: [4]f32 = undefined;
    for (data, &hub) |x, *s| s.* = body.dot4(body.pointVelocity(v.car.body, x.pos), x.forward_axis);
    const saved = sys.braking_torque;
    const steer = v.controls.steering;
    const a2 = wheels[2].tyre.slip.angle;
    const a3 = wheels[3].tyre.slip.angle;
    const front = yawError(((if (steer * a2 < 0) a2 else 0) + (if (steer * a3 < 0) a3 else 0)) * 0.5, wheels[2].tyre);
    const rear = yawError((wheels[1].tyre.slip.angle + wheels[0].tyre.slip.angle) * 0.5, wheels[0].tyre);
    const d = @abs(front) - @abs(rear);
    const minimise = if (d > 0) front > 0 else 0 > d and 0 > rear;
    const severity = unit((@abs(d) - 5) / 15);

    const dt = v.drivetrain;
    toSide(sys.*, data, wheels.*, v.car, v.side);
    var best: f32 = 0;
    var pick: Run = undefined;
    for (0..16) |k| {
        const run = Run.of(k);
        sys.braking_torque = if (run.brakes_off) .{ 0, 0, 0, 0 } else saved;
        sys.braking_torque[run.wheel] = math.maxss(sys.braking_torque[run.wheel], wheels[run.wheel].max_brake_torque);
        v.side.braking_torque = sys.braking_torque;
        const keep = dt.*;
        drivetrain.integrate(dt, v.side, if (run.closed_throttle) torque_closed else torque, h, false);
        var score: f32 = 0;
        for (data, wheels.*, hub, 0..) |x, w, vh, i| {
            const roll = dt.wheels[(i + 2) % 4].state.angvel * w.radius; // drivetrain order FL FR RL RR
            const slip = vh - roll;
            const lat = x.side_velocity;
            const q = lat * lat + slip * slip;
            var cs: f32 = 1;
            var sn: f32 = 0;
            if (q > 0.001) {
                const inv = 1 / @sqrt(q);
                cs = slip * inv;
                sn = inv * lat;
            }
            const along = cs * w.direction[0] + w.direction[1] * sn;
            const across = w.direction[1] * -cs + sn * w.direction[0];
            score = score + (along * w.rest_position[0] - across * w.rest_position[2]) * estimateForce(vh, roll, lat);
        }
        if ((if (minimise) best > score else score > best) or k == 0) {
            pick = run;
            best = score;
        }
        const random = dt.random; // the components' backup leaves these
        const wobbled = dt.has_wobbled;
        dt.* = keep;
        dt.random = random;
        dt.has_wobbled = wobbled;
    }

    const kk = esp * severity;
    const brake: f32 = if (pick.brakes_off) unit(1 - kk) else 1;
    sys.brake_filter = brake;
    for (&sys.braking_torque, saved, wheels) |*t, s, *w| {
        t.* = brake * s;
        w.esp_brake_torque = 0;
    }
    sys.throttle_filter = if (pick.closed_throttle) unit(1 - (kk + kk)) else 1;
    const cur = sys.braking_torque[pick.wheel];
    const next = kk * (wheels[pick.wheel].max_brake_torque - cur) + cur;
    wheels[pick.wheel].esp_brake_torque = next - cur;
    sys.braking_torque[pick.wheel] = next;
}

// degrees of slip angle past the tyre's lateral peak, signed
fn yawError(angle: f32, t: Tyre) f32 {
    const p = t.set.lateral.peak_x1;
    const peak = if (t.has_record) (p - p) + p else 0.1;
    const e = @abs(angle) - peak;
    return math.sign(angle) * (if (0 > e) 0 else e * 180 / std.math.pi);
}

// 0xf34da0 estimateForce: a rough tyre force, 0 .. sqrt 2, from the slip ratio and angle of hub speed v,
// tread speed w and side speed lat
fn estimateForce(v: f32, w: f32, lat: f32) f32 {
    const c = ((@abs(lat) + math.maxss(@abs(v), @abs(w))) - 0.05) / 3.95;
    const hi: f32 = if (c > 1) 1 else if (c > 0) c else 0;
    const lo: f32 = if (c > 1) -1 else if (c > 0) -c else -0.0;
    const r = (v - w) / (if (0.05 < @abs(v)) @abs(v) else 0.05);
    const ratio = if (r > hi) hi else math.maxss(r, lo);
    var angle_t: f32 = 0;
    if (lat * lat + v * v > 0.01) {
        var a = math.atan2(lat, v);
        if (-std.math.pi / 2.0 > a) a = -std.math.pi - a else if (a > std.math.pi / 2.0) a = std.math.pi - a;
        const x = @abs(a) / 0.31415927;
        angle_t = if (x > 1) 1 else if (x > 0) x * x else 0;
    }
    const s = @abs(ratio) / 0.2;
    return @sqrt(angle_t + (if (s > 1) 1 else if (s > 0) s * s else 0));
}
