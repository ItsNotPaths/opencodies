// One car's constants, in plain physical terms (docs/car.md). Per-frame state stays in each game's State.
// Only the car's name says where it comes from. null: the car does not have that part; the step treats it as absent.
const vec = @import("shared/vec.zig");
const body = @import("shared/body.zig");
const bumps = @import("shared/bumps.zig");
const chassis = @import("shared/chassis.zig");
const car_types = @import("dr1/car.zig");
const drivetrain = @import("dr1/drivetrain.zig");
const suspension = @import("dr1/suspension.zig");
const tyre = @import("dr1/tyre.zig");
const wheel = @import("dr1/wheel.zig");

const V3 = vec.V3;
const Axle = suspension.Axle;
const DynamicCamber = suspension.DynamicCamber;
const SubTyre = tyre.SubTyre;
const Heat = wheel.Heat;
const Wear = wheel.Wear;
const Aids = car_types.Aids;
const ClutchRates = car_types.ClutchRates;
const Reverse = car_types.Reverse;
const DiffKind = drivetrain.Kind;
const Curves = @import("fh/tyre.zig").Curves;
const RearSlide = @import("fh/tyre.zig").RearSlide;
const Lsd = @import("fh/drivetrain.zig").Diff;

pub const Car = struct {
    mass: f32,
    inertia: V3, // body frame diagonal, kg m^2
    front_weight: ?f32 = null, // share of the weight on the front axle
    hull: ?chassis.Hull = null, // the shapes against the track
    wheels: [4]Wheel, // RL RR FL FR

    drag: ?V3 = null, // side, vertical, forward
    aero: Aero,
    engine: Engine,
    fuel: ?Fuel = null,
    gearbox: Gearbox,
    diffs: Diffs,
    clutch_order: ?[4]usize = null, // the clutch sums the wheel spins in this order
    driveline: ?Driveline = null,
    launch: ?Launch = null,

    handbrake_strength: f32, // times the rear brake torque
    handbrake_grip_loss: ?f32 = null, // share of the tyre grip lost on the handbrake
    auto_handbrake: ?bool = null,
    abs_slip: ?f32 = null, // the brakes release past this slip
    abs_period: ?f32 = null, // s between brake releases
    aids: ?Aids = null,
    stability: ?f32 = null, // 0..1, blends the stability assist strength
    drift_friction_control: ?bool = null, // the engine friction eases in a drift
    donut_damping: ?bool = null, // damps a spin on the spot
    auto_recover: ?bool = null, // puts a stuck car back on the road; not ported

    bumps: ?bumps.Gain = null,
    substep_jitter: ?[]const f32 = null, // the substeps' lengths over their mean, in turn; null: even
    spring_load_scale: ?f32 = null, // scales both spring loads
    rebound_relief: ?f32 = null, // share of the fast rebound damping lost while the body lifts off a drooping wheel
    rigid_axles: ?u32 = null, // bits 1 rear, 2 front: the two wheels lean with their height difference

    tyre_grip_by_load: ?[3]f32 = null, // grip slope per static load, over 0..2, 2..4 and past 4 loads
    tyre_yaw: ?TyreYaw = null,
    tyre_heat: ?Heat = null,
    tyre_wear: ?Wear = null, // the wear template
    tyre_wear_rate: ?f32 = null,
    tyre_effects: ?u32 = null, // bits 2 wear, 4 temperature, 8 dirt
    contact_mode: ?u32 = null, // 0: each tyre also reads the material under its two edges
    rear_slide_grip: ?RearSlide = null, // the rear tyres' side grip by slip angle and steering

    // steering
    pad_lock: ?[25]f32 = null, // rad by speed
    pad_lock_step: ?f32 = null, // m/s per entry
    pad_linearity: ?f32 = null,
    steer_rate: ?[2]f32 = null, // per s at the two speeds below
    steer_rate_speeds: ?[2]f32 = null, // m/s
    track: f32, // m, Ackermann
    wheelbase: f32,
    steer_lock: ?f32 = null, // rad
    pad_min_lock: ?f32 = null,
    full_lock_speed: ?f32 = null, // m/s; the lock falls as 1 / speed above it
    lock_by_graph: ?bool = null, // the pad lock follows pad_lock, not full_lock_speed
    counter_steer: ?bool = null, // the pad target follows the drift angle
    steer_sensitivity: ?f32 = null,
    pad_assists: ?bool = null, // the pad rate limits and targets apply
    steer_return_rates: ?[2]f32 = null, // to centre, to the target
    steered_wheels: ?[2]u32 = null, // outer when turning right first
    visual_turn_rate: ?f32 = null,
    speed_steering: ?SpeedSteering = null,

    // jump assist: the tyre yaw inertia multiple grows in the air
    jump_rates: ?[2]f32 = null, // per s: rise, fall
    jump_cap: ?f32 = null,

    wiper: ?Wiper = null,

    pub fn rigid(car: Car) body.Body {
        return .{ .mass = car.mass, .inertia = car.inertia };
    }
};

pub const Wheel = struct {
    rest_position: V3, // hub, body frame
    spring_axis: V3, // body frame
    radius: f32,
    max_radius: ?f32 = null, // the ground query reaches this far
    width: ?f32 = null,
    side_cos: ?f32 = null, // past this cos of the ground angle the tyre's side touches
    inertia: f32, // kg m^2
    min_height: f32, // m of travel along the axis
    max_height: f32,
    bump_stop_height: f32,
    spring: Spring,
    slow_bump: f32, // N s/m
    fast_bump: ?f32 = null, // past bump_zone
    slow_rebound: f32,
    fast_rebound: ?f32 = null,
    bump_zone: ?f32 = null, // m/s
    rebound_zone: ?f32 = null,
    anti_roll: f32, // N/m
    anti_roll_damping: ?f32 = null, // N s/m
    damper_caps: ?[2]f32 = null, // N: bump, rebound force
    bump_stop: ?BumpStop = null, // past max_height
    unsprung_mass: ?f32 = null, // kg
    roll_centre: ?[3]f32 = null, // anti squat/dive, anti-roll constant, anti-roll linear
    toe_in: ?f32 = null, // rad
    toe_change: ?DynamicCamber = null, // the toe by travel
    tyre_spring: ?f32 = null, // N/m, carcass
    tyre_damping: ?f32 = null, // N s/m
    tyre_depth: ?f32 = null, // m the tyre may sink
    tan_peak_slip: ?f32 = null,
    peak_slip_ratio: ?f32 = null,
    grip: ?f32 = null, // friction coefficient
    tyre: Tyre,
    visual_camber: ?VisualCamber = null,
    max_brake_torque: f32, // N m
    handbrake: bool,
    drive_proportion: f32,
};

pub const Spring = union(enum) {
    linear: struct { rate: f32, preload: f32 }, // N/m; m, the static load at travel 0
    two_segment: struct { axle: Axle, top_height: f32 }, // with a bump stop
};

pub const Tyre = union(enum) {
    circle: struct { rolling: f32, camber: f32, camber_grip: f32 }, // mu in a friction circle; camber rad
    graphs: struct { compound: []const SubTyre, legacy: bool, camber: f32, dynamic_camber: DynamicCamber, burst_grip: f32 }, // camber rad
    curve_table: struct { lateral: Curves, longitudinal: Curves, lateral_grip: f32, longitudinal_grip: f32, free_grip: ?FreeGrip = null }, // friction against normalized slip at two loads
};

// extra grip whose force acts without turning the body: the force scales, its torque does not
pub const FreeGrip = struct {
    lateral: f32,
    braking: f32,
    driving: [2]f32, // at the two speeds below
    driving_speeds: [2]f32, // m/s
};

pub const BumpStop = struct { rate: f32, damping: f32 }; // N/m, N s/m

// the gearbox and shafts behind the clutch
pub const Driveline = struct {
    gearbox_inertia: f32, // kg m^2
    shaft_inertia: f32,
    torque_scale: f32, // times the torque through the gearbox
};

// the lock falls with speed to hold a lateral acceleration; the wheels turn at limited rates
pub const SpeedSteering = struct {
    max_gees: f32, // times the tyres' peak lateral acceleration
    min_lock: f32, // rad
    turn_rate: f32, // rad/s away from centre
    return_rate: f32, // rad/s toward centre
    find_peak_rate: f32, // rad/s past the lock at full input, until the front tyres reach their peak slip
    rate_ramp: f32, // s to reach full rate
    slow_speed: f32, // m/s: the rate scales between these
    fast_speed: f32,
    fast_rate_scale: f32,
};

// the yaw inertia the tyre impulses see grows with speed
pub const TyreYaw = struct {
    mid_scale: f32, // at the mid speed
    speeds: [2]f32, // m/s, lowest and mid
};

// the drawn wheel leans by camber + this, and the contact query follows its lowest edge
pub const VisualCamber = struct {
    angle: f32, // rad
    gain: [2]f32, // rad/m of travel: stretched (< 0), squashed
    rim: [2]f32, // m, x extent of the wheel model along the axle from the hub
};

pub const Aero = union(enum) {
    point: struct { // downforce at one point
        downforce: f32, // N/(m/s)^2
        pressure_point: f32, // 0 rear axle, 1 front axle
        pressure_point_air: [2]f32, // a wheel off the ground: falling, rising
    },
    directional: struct { // drag and downforce by direction and by wing
        reverse_drag: f32,
        centre_x: f32, // where the side drag acts, 0 rear axle .. 1 front axle
        centre_y: f32,
        downforce_central: [2]f32, // forward, reverse
        downforce: [3][2]f32, // body, front, rear; forward, reverse
        wing_drag: [4]f32,
        pressure_centre: f32,
        pressure_centre_air: f32,
        legacy_rolling: bool,
        legacy_downforce: bool,
        pikes_peak: bool, // the newer downforce, by speed squared
        drag_scale: f32, // times the forward drag
    },
    axles: Axles,
};

// drag along each body axis, side drag and downforce at the axles, wings
pub const Axles = struct {
    forward_drag: f32, // N/(m/s)^2 at the centre of mass
    vertical_drag: f32,
    side_drag: [2]f32, // rear, front
    downforce: [2]f32, // rear, front; fades out as the velocity turns past zero_downforce_cos
    zero_downforce_cos: f32,
    wings: [2]?Wing, // rear, front
};

pub const Wing = struct {
    drag: f32, // N/(m/s)^2
    side_drag: f32,
    downforce: f32,
    zero_downforce_cos: f32,
    torque_share: f32, // share of the downforce that pitches the body; the rest acts at the centre of mass
};

pub const Engine = struct {
    inertia: f32, // kg m^2
    idle: f32, // rad/s
    rev_limit: f32,
    curve: Curve,
    starter: ?Starter = null,
    turbo: ?Turbo = null,
};

pub const Curve = union(enum) {
    power: struct {
        step: f32, // rad/s
        table: []const f32, // W; from a 0 entry to the next sample, and past the end: 0
        divisor: f32, // the power divides by it
        friction: f32,
    },
    torque: struct {
        step: f32, // rad/s
        table: [32]f32, // N m
        friction: f32, // times the friction graph
        friction_drive: f32,
        restrictor: f32,
        capacity: f32, // engine capacity
        max_power: f32, // W, rolling back in first
        max_torque: f32,
        power_scale: f32,
        difficulty: f32,
        drivetrain_friction_scale: f32,
        layout: u32, // 2 and 4 turn the body about its forward axis
        momentum_boost: f32, // the engine's spin moves the body more at low speed
    },
    sampled: struct {
        step: f32, // rad/s
        table: []const f32, // N m at full throttle
        braking: f32, // N m, times a fixed shape of the speed at zero throttle
        low_speed_braking: f32, // times the fixed shape over the first 8 samples
        stall: f32, // rad/s: below it the engine stops
        redline: f32, // rad/s: the braking shape bends here; the box shifts up near it
    },
};

pub const Starter = struct { strength: f32, time_scale: f32 };

pub const Turbo = struct {
    inlet_volume: f32,
    inlet_torque: f32,
    inlet_pressure_cap: f32,
    outlet_volume: f32,
    outlet_torque: f32,
    outlet_pressure_cap: f32,
    inertia: f32,
    inlet_rate: f32,
    sqr_factor: f32,
    outlet_input_rate: f32,
    rpm_for_boost: f32,
    spool_rpm_for_boost: f32,
    spool_rpm_max: f32,
    max_boost: f32, // the peak is the rev limit
};

// full at reset; burned: the engine burns it, so its weight falls
pub const Fuel = struct { capacity: f32, full_tank_weight: f32, burned: bool };

pub const Gearbox = struct {
    ratios: [11]f32, // wheel/engine: 0 neutral, 1..9 forward, 10 reverse
    top_gear: i32,
    clutch_torque: f32, // N m
    shift_delay: ?f32 = null, // s the ratio waits after a gear change
    shift_time: ?[2]f32 = null, // s: semi auto, manual
    crossover: ?[11]f32 = null, // engine speed where the auto box shifts up, by gear
    clutch_rates: ?[2]ClutchRates = null, // standard, manual sequential
    clutch_points: ?[4]f32 = null,
    reverse: ?Reverse = null, // when the box goes between reverse and first
    auto_gearbox: ?bool = null,
    auto_clutch: ?bool = null,
    auto_reverse: ?bool = null,
    manual_override: ?bool = null,
    manual_upshift: ?bool = null,
    dual_clutch: ?bool = null,
    clutch_speeds: ?[2]f32 = null, // per s: in, out
};

pub const Diffs = union(enum) {
    rate: struct { rear: RateDiff, front: RateDiff, centre: RateDiff, off_throttle: f32 },
    tree: struct { centre: TreeDiff, front: TreeDiff, rear: TreeDiff, handbrake_uses_rear_clutch: bool, clutch_on_handbrake: bool },
    limited_slip: struct { final_drive: f32, rear: Lsd, front: Lsd, centre: Lsd }, // ratio: shaft/wheel; the ratios include final_drive
};

pub const RateDiff = struct { no_effect: f32, full_effect: f32, max_torque: f32 }; // rad/s, rad/s, %

pub const TreeDiff = struct {
    kind: DiffKind,
    bias: f32,
    driving_bias: f32,
    braking_bias: f32,
    preload: f32,
    viscosity: f32,
    clutch_torque: f32, // crown
    clutch_bias: f32,
};

pub const Launch = struct { enabled: bool, optimum_rpm: f32 };

pub const Wiper = struct { visual_offset: f32, hold_time: f32, sweep_time: f32, only_when_moving: bool };
