// One ground material, in plain physical terms (docs/car.md, Materials). Only the name says where it comes from:
// "d3-GLD+" and "dr1-GLD+" are two materials. null: the material does not have that effect.
const std = @import("std");

pub const Name = [8]u8; // origin, '-', the 4-char track code; 0 padded

pub const Material = struct {
    name: Name,
    depth: f32, // m the contact sits below the mesh
    bumps: Bumps,
    slowdown: Slowdown, // on the surface
    grip: ?f32 = null, // times the tyre's friction
    hardness: ?f32 = null, // the tyre's peak slip angle scales by 1 / hardness
    side_drag: ?Drag = null, // sideways drag on the tyre
    loose: ?Loose = null, // the tyre sinks in and ploughs
    tyre_class: ?u32 = null, // picks the tyre's grip graphs for this surface
    ellipsoid: ?bool = null, // true: the tyre also meets the triangles as an ellipsoid (its side can touch)
    wall_slowdown: ?Slowdown = null, // the car rides a wall of it
    edge_slowdown: ?Slowdown = null, // the wheels run onto it from the side, or the car scrapes a wall of it
    wear: ?f32 = null, // times the tyre wear
    dirt_buildup: ?bool = null, // true: the tyre collects dirt on it; false: the dirt clears
    roughness: ?f32 = null, // shakes the car's spin above 8 m/s
    curve_grip: ?CurveGrip = null, // for tyres with friction curves

    pub fn code(m: Material) [4]u8 {
        const dash = std.mem.indexOfScalar(u8, &m.name, '-').?;
        return m.name[dash + 1 ..][0..4].*;
    }

    pub fn metal(m: Material) bool {
        return std.mem.eql(u8, m.code()[0..3], "MET");
    }

    // the name before the dash: "d3", "dr1"
    pub fn origin(m: *const Material) []const u8 {
        return m.name[0..std.mem.indexOfScalar(u8, &m.name, '-').?];
    }

    pub fn isBare(m: Material) bool {
        return std.meta.eql(m, bare(m.origin(), m.code()));
    }
};

pub fn find(materials: []const Material, code: [4]u8) ?usize {
    for (materials, 0..) |m, i| if (std.mem.eql(u8, &m.code(), &code)) return i;
    return null;
}

pub fn named(origin: []const u8, code: [4]u8) Name {
    var n: Name = @splat(0);
    @memcpy(n[0..origin.len], origin);
    n[origin.len] = '-';
    @memcpy(n[origin.len + 1 ..][0..4], &code);
    return n;
}

// only the name known: no depth, bumps or slowdown, every optional effect absent
pub fn bare(origin: []const u8, code: [4]u8) Material {
    return .{ .name = named(origin, code), .depth = 0, .bumps = smooth(.by_distance), .slowdown = .{ .full_speed = 1 } };
}

// how a tyre with friction curves grips on the surface
pub const CurveGrip = struct {
    friction: f32, // times the tyre's friction
    off_road: f32, // 0..1: the tyre's peak slip angle moves to peak_slip_angle
    peak_slip_angle: f32, // rad
    rear: f32, // times the rear tyres' friction
    handbrake: f32, // share of the grip a locked handbraked tyre keeps
    min_arcade: f32, // the least blend toward the arcade curves
};

// the road noise under the tyre
pub const Bumps = union(enum) {
    by_time: struct {
        wavelength: f32, // s
        magnitude: f32, // m
        contact_share: f32, // the part that moves the contact; the rest only moves the drawn wheel
        car_share: f32, // the part the car's surface factor scales
    },
    by_distance: struct {
        wavelength: f32, // m, over x and z
        magnitude: f32, // m
    },
};

pub const Model = std.meta.Tag(Bumps);

// no road noise
pub fn smooth(model: Model) Bumps {
    return switch (model) {
        .by_time => .{ .by_time = .{ .wavelength = 0, .magnitude = 0, .contact_share = 0, .car_share = 0 } },
        .by_distance => .{ .by_distance = .{ .wavelength = 1, .magnitude = 0 } },
    };
}

// a deceleration that ramps in with speed
pub const Slowdown = extern struct { from_speed: f32 = 0, full_speed: f32 = 0, amount: f32 = 0 }; // amount: m/s^2 at full_speed

// sideways drag that saturates at speed
pub const Drag = extern struct { speed: f32 = 0, coefficient: f32 = 0 };

// the tyre in loose ground: drag coefficients by slip, how deep it sinks, how hard the surface pushes back
pub const Loose = struct {
    coeff_high: f32, // past the peak slip ratio
    coeff_mid: f32,
    coeff_low: f32, // braking past the peak
    coeff_long: f32,
    max_depth: f32, // m the tyre sinks
    max_force_per_normal: f32,
    resists_torque_factor: f32,
    density: f32,
    depth_vs_weight: f32,
    velocity_limit: f32,
};

// A car on another game's level (docs/car.md, "Across games"). The car's list is its own game's materials.

pub const How = enum { exact, family, table, default };

// its suffixes in order of preference: one list has only * and +, the other only + ! _
const suffixes = "*+!_";

// past the family: a tyre class or a code, and the code it stands for in the other list
const table = [_][2][4]u8{
    .{ "GVLD".*, "GLD*".* }, .{ "GVLW".*, "GLW*".* }, .{ "GVHD".*, "GHD*".* }, .{ "GVHW".*, "GHW*".* },
    .{ "SNWL".*, "SNO*".* }, .{ "ICET".*, "ICS*".* }, .{ "TRMD".*, "TSD*".* }, .{ "TRMW".*, "TSW*".* },
    .{ "RMBL".*, "TRD+".* }, .{ "GRAV".*, "GLD+".* }, .{ "GTRP".*, "GHD+".* },
};

// m's match in list: its code, then its family (same 3 letters), then the table, then DEFA
pub fn match(m: Material, list: []const Material) ?struct { usize, How } {
    const code = m.code();
    if (find(list, code)) |i| return .{ i, .exact };
    for (suffixes) |c| {
        if (find(list, code[0..3].* ++ [1]u8{c})) |i| return .{ i, .family };
    }
    const class: [4]u8 = @bitCast(m.tyre_class orelse 0);
    for (table) |t| {
        if (!std.mem.eql(u8, &t[0], &code) and !std.mem.eql(u8, &t[0], &class)) continue;
        if (find(list, t[1])) |i| return .{ i, .table };
    }
    return .{ find(list, "DEFA".*) orelse return null, .default };
}

pub const Field = std.meta.FieldEnum(Material);

// m with each field it holds as null, or in another bumps model, taken from the car's match
pub fn borrow(m: Material, from: Material) struct { Material, std.EnumSet(Field) } {
    var out = m;
    var taken: std.EnumSet(Field) = .initEmpty();
    inline for (@typeInfo(Material).@"struct".fields) |f| {
        const mine = @field(m, f.name);
        const theirs = @field(from, f.name);
        const take = switch (@typeInfo(f.type)) {
            .optional => mine == null and theirs != null,
            .@"union" => std.meta.activeTag(mine) != std.meta.activeTag(theirs),
            else => false,
        };
        if (take) {
            @field(out, f.name) = theirs;
            taken.insert(@field(Field, f.name));
        }
    }
    return .{ out, taken };
}
