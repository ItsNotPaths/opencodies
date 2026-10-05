// SuspensionSystem::doHeightChecks 0xf59570: where each wheel finds the ground, on the shared track mesh. The game
// copies the triangles in a box around each wheel's ray from its CQTC grid into a TriangleCache (100 at most) and
// queries that; here the mesh gives the triangles, through the same triangle-box test.
const std = @import("std");
const body = @import("body.zig");
const math = @import("math.zig");
const suspension = @import("suspension.zig");
const tyre = @import("tyre.zig");
const mesh = @import("../shared/mesh.zig");

const V4 = body.V4;
const dot4 = body.dot4;
const System = suspension.System;
const WheelData = suspension.WheelData;
const Wheel = suspension.Wheel;

pub const none: u16 = 0xffff; // no material

// a NePhysicsMaterial and its SurfaceSpec (the material manager's data entry), the fields the query reads
pub const Material = extern struct {
    code: [4]u8, // m_ID
    ellipsoid: bool, // +0x38 m_useEllipsoidTyreCollision
    bumps_wavelength: f32, // SurfaceSpec+0x4
    bumps_magnitude: f32, // +0x8
    depth: f32, // +0xc
    simulation: suspension.Surface, // +0x1c, the tyre code at +0x44
    slowdown: [3]Slowdown, // +0x48 on the surface, +0x54 wall riding, +0x60 soft edges
    wear_multiple: f32, // +0xb4
    dirt_buildup: bool, // +0xb8 m_dirtTyreBuildup
    roughness: f32, // +0x6c: kicks the car's spin above 8 m/s
};

// SurfaceSlowDownParameters
pub const Slowdown = extern struct {
    factor: f32, // m/s^2 at full slowdown
    min_speed: f32, // none below it
    full_speed: f32, // full from it
};

// the car fields the query reads
pub const Car = extern struct {
    render_offsets: [4][4]f32, // car+0x60 m_wheelRenderOffsets
    detached: [4]bool, // car+0x11fd4 CarDamage::getWheelDetached
    speed: f32, // car+0x8264
    bump_scale: f32, // car+0x69ac
    simulation_type: u32, // car+0x6db0: 0 records the contacted materials
};

pub const Ground = extern struct {
    track: *const mesh.Mesh,
    materials: [*]const Material, // one per track material
    compounds: [4][*]const tyre.SubTyre, // each wheel's tyre compound (RuntimeTyre+0x28)
    compound_lens: [4]u32,
    car: Car,

    pub fn heightChecks(g: Ground, sys: *System, b: body.RigidBody, data: *[4]WheelData, wheels: *[4]Wheel, dt: f32) void {
        doHeightChecks(g, sys, b, data, wheels, true, dt);
    }

    fn compound(g: Ground, i: usize) []const tyre.SubTyre {
        return g.compounds[i][0..g.compound_lens[i]];
    }
};

pub const Triangle = extern struct { v: [3][4]f32, material: u16 };

pub const Hit = extern struct { t: f32, normal: [4]f32, material: u16 };

const capacity = 100; // TriangleCache

pub const Cache = struct {
    buf: [capacity]Triangle = undefined,
    len: usize = 0,

    pub fn items(c: *const Cache) []const Triangle {
        return c.buf[0..c.len];
    }
};

fn splat(x: f32) V4 {
    return @splat(x);
}

const cos75: f32 = 0.258819043636322; // 0x2736320

// 0xf59570; end_frame: the bumps move the contact
pub fn doHeightChecks(g: Ground, sys: *System, b: body.RigidBody, data: *[4]WheelData, wheels: *[4]Wheel, end_frame: bool, dt: f32) void {
    const n = @min(sys.wheel_count, 4);
    const right: V4 = b.rows[0];
    const up: V4 = b.rows[1];
    const forward: V4 = b.rows[2];
    var old: [4]f32 = undefined;
    var push: [4]V4 = @splat(splat(0)); // the ellipsoid's push out of the triangles
    var start: [4]V4 = undefined;
    var end: [4]V4 = undefined;
    for (data[0..n], wheels[0..n], 0..) |*d, *w, i| {
        old[i] = d.height_if_on_ground;
        d.height_if_on_ground = -1000;
        setContactSurface(g, w, i, none);
        w.has_geometry = false;
        w.has_material = false;
        w.metal = false;
        w.surface_normal = up;
        var off: V4 = g.car.render_offsets[i];
        off[3] = 0;
        var base = (splat(0) * @as(V4, b.rows[3]) + splat(off[1]) * up) + (splat(off[0]) * right + splat(off[2]) * forward);
        base = base + @as(V4, d.rest_pos) + splat(sys.ai_com_offset) * up;
        base[3] = 1;
        const top = (0 + w.max_height) + w.max_radius;
        const bottom = w.min_height - 0.1;
        const air = d.in_air_factor * 1.4 - 0.2;
        const splay = if (air > 1) 1 else math.maxss(air, 0);
        const lean = right * splat(splay) * splat(math.sign(w.rest_position[0])); // a wheel long in the air feels outward
        start[i] = splat(top) * up + base + splat(0.05) * lean;
        end[i] = (base + up * splat(bottom)) - lean * splat(0.2);
    }

    for (data[0..n], wheels[0..n], 0..) |*d, *w, i| {
        if (g.car.detached[i]) continue;
        const top = (0 + w.max_height) + w.max_radius;
        const span = (w.min_height - 0.1) - top;
        const pad = w.max_radius * 1.01;
        const pads: V4 = .{ pad, pad, pad, 0 };
        var cache: Cache = .{};
        if (sys.ground_geometries == 0) gather(g.track, minps(start[i], end[i]) - pads, pads + maxps(start[i], end[i]), &cache);
        const hit = rayCast(cache.items(), start[i], end[i]) orelse continue;
        const axis: V4 = d.spring_axis_world;
        const normal: V4 = hit.normal;
        const facing = dot4(axis, normal);
        if (cos75 >= facing) continue; // steeper than 75 deg; small entities are not queried
        d.height_if_on_ground = span * hit.t + top;

        if (cache.len != 0) {
            const center = (splat(hit.t) * (end[i] - start[i]) + start[i]) + axis * splat(w.radius);
            const e = ellipsoid(cache.items(), g.materials, w.*, center, d.side_axis, axis);
            push[i] = e.push;
            w.contacted_count = 0;
            if (g.car.simulation_type == 0) {
                w.outermost_contacted = if (0 > w.rest_position[0]) 2 else 0;
                const side = splat(w.width) * @as(V4, d.side_axis);
                const near = rayCast(cache.items(), start[i] + side, end[i] + side);
                const far = rayCast(cache.items(), start[i] - side, end[i] - side);
                w.contacted = .{ if (near) |h| h.material else e.material, hit.material, if (far) |h| h.material else none };
                w.contacted_count = 3;
            }
        }

        const ellip = if (facing >= 0.9397) 1 else (facing * 1.0641694) * (facing * 1.0641694); // past 20 deg a rise climbs slower
        const m = g.materials[hit.material];
        setContactSurface(g, w, i, hit.material);
        w.has_geometry = true;
        w.has_material = true;
        w.metal = std.mem.eql(u8, &m.code, "MET*");
        w.surface_normal = normal;
        var bump = d.bump_offset[0];
        if (end_frame) {
            const floor = d.bump_offset[0] - dt * g.car.speed * 0.1736482;
            bump = math.maxss(bumpOffset(m, start[i], g.car.bump_scale), floor);
            d.bump_offset[1] = bump;
        }
        var h = d.height_if_on_ground - m.depth - w.radius_modifier;
        d.height_before_noise = travel(w.*, h);
        h = h + bump;
        d.height_if_on_ground = h;
        const was = travel(w.*, old[i]);
        if (h >= w.min_height and old[i] >= w.min_height and @abs(dot4(push[i], d.side_axis)) / w.width >= 0.25) {
            // pushed sideways: the contact rises no faster than the wheel moves into the side
            const v = velocityAt(b, start[i] - @as(V4, b.rows[3]));
            if ((h - was) / dt > @abs(dot4(d.side_axis, v)) * 0.5) {
                const climb = dot4(up, v);
                const limit = (if (0 > climb) 0.2 * dt - climb * dt else 0.2 * dt) + was;
                if (h > limit) h = limit;
                d.height_if_on_ground = h;
            }
        }
        if (1 > ellip and h - was > 0) d.height_if_on_ground = was + (h - was) * ellip;
    }

    for (data[0..n], wheels[0..n]) |*d, w| d.ground_normal = bend(w.surface_normal, d.spring_axis_world);
}

// clamped to the wheel travel, NaN gives the bottom
fn travel(w: Wheel, h: f32) f32 {
    return if (h > w.max_height) w.max_height else math.maxss(h, w.min_height);
}

// the normal bent toward the axis while it is more than 80 deg off
fn bend(normal: V4, axis: V4) V4 {
    var n = normal;
    while (0.173 > dot4(axis, n)) {
        const v = n + splat(0.05) * axis;
        const s = v * v;
        n = body.normalize4(v, (s[0] + s[2]) + (s[1] + s[3]));
    }
    return n;
}

// SSE minps/maxps: a unless b is smaller/larger
fn minps(a: V4, b: V4) V4 {
    return @select(f32, a < b, a, b);
}

fn maxps(a: V4, b: V4) V4 {
    return @select(f32, a > b, a, b);
}

fn point(v: @Vector(3, f32)) V4 {
    return .{ v[0], v[1], v[2], 1 };
}

// 0xe0ab10 getTrianglesFromStaticCQTCs, on the mesh: the triangles testTriangleWithAABB passes, in mesh order
pub fn gather(track: *const mesh.Mesh, lo: V4, hi: V4, out: *Cache) void {
    if (!std.math.isFinite(@reduce(.Add, lo + hi))) return; // the mesh grid takes no NaN; the game's case is unchecked
    const half = (hi - lo) * splat(0.5);
    const center = half + lo;
    const margin: @Vector(3, f32) = @splat(0.01); // the mesh's own box test only narrows the candidates
    var near: [512]mesh.Triangle = undefined;
    const lo3: @Vector(3, f32) = .{ lo[0], lo[1], lo[2] };
    const hi3: @Vector(3, f32) = .{ hi[0], hi[1], hi[2] };
    for (track.faces(lo3 - margin, hi3 + margin, &near)) |f| {
        const v: [3][4]f32 = .{ point(f.corners[0]), point(f.corners[1]), point(f.corners[2]) };
        if (!boxOverlaps(center, half, .{ v[0], v[1], v[2] })) continue;
        if (out.len == capacity) return;
        out.buf[out.len] = .{ .v = v, .material = f.material };
        out.len += 1;
    }
}

// 0xdf6140 PhysicsCollisionFunctions::testTriangleWithAABB: separating axes, the box's own three first
pub fn boxOverlaps(center: V4, half: V4, tri: [3]V4) bool {
    const v0 = tri[0] - center;
    const v1 = tri[1] - center;
    const v2 = tri[2] - center;
    inline for (.{ 1, 0, 2 }) |k| {
        const lo = math.minss(v2[k], math.minss(v1[k], v0[k]));
        const hi = math.maxss(v2[k], math.maxss(v1[k], v0[k]));
        if (lo > half[k] or 0 - half[k] > hi) return false;
    }
    const nv = body.cross(v0 - v1, v0 - v2);
    const s = nv * nv;
    const n = body.normalize4(nv, (s[0] + s[2]) + (s[1] + s[3]));
    const d = dot4(v0, n);
    const r = (@abs(n[0]) * half[0] + @abs(n[1]) * half[1]) + @abs(n[2]) * half[2];
    if (d > r or -r > d) return false;

    const hx, const hy, const hz = .{ half[0], half[1], half[2] };
    var e = v1 - v0;
    if (separated(v2[1] * e[2] - v2[2] * e[1], v0[1] * e[2] - v0[2] * e[1], @abs(e[2]) * hy + @abs(e[1]) * hz)) return false;
    if (separated((0 - e[2]) * v2[0] + v2[2] * e[0], v0[0] * (0 - e[2]) + v0[2] * e[0], @abs(e[0]) * hz + @abs(e[2]) * hx)) return false;
    if (separated(v1[0] * e[1] - v1[1] * e[0], e[1] * v2[0] - e[0] * v2[1], @abs(e[1]) * hx + @abs(e[0]) * hy)) return false;
    e = v2 - v1;
    if (separated(v2[1] * e[2] - v2[2] * e[1], v0[1] * e[2] - v0[2] * e[1], @abs(e[1]) * hz + @abs(e[2]) * hy)) return false;
    if (separated((0 - e[2]) * v2[0] + v2[2] * e[0], v0[0] * (0 - e[2]) + v0[2] * e[0], @abs(e[0]) * hz + @abs(e[2]) * hx)) return false;
    if (separated(v1[0] * e[1] - e[0] * v1[1], v0[0] * e[1] - v0[1] * e[0], @abs(e[1]) * hx + @abs(e[0]) * hy)) return false;
    e = v0 - v2;
    if (separated(v1[1] * e[2] - v1[2] * e[1], v0[1] * e[2] - v0[2] * e[1], hz * @abs(e[1]) + hy * @abs(e[2]))) return false;
    if (separated(v1[2] * e[0] + (0 - e[2]) * v1[0], v0[2] * e[0] + v0[0] * (0 - e[2]), @abs(e[2]) * hx + hz * @abs(e[0]))) return false;
    return !separated(v1[0] * e[1] - v1[1] * e[0], e[1] * v2[0] - e[0] * v2[1], hx * @abs(e[1]) + @abs(e[0]) * hy);
}

// the projections p, q of the triangle on an edge axis miss -rad..rad; p, q in the game's compare order
fn separated(p: f32, q: f32, rad: f32) bool {
    const lo, const hi = if (p > q) .{ q, p } else .{ p, q };
    return lo > rad or -rad > hi;
}

// 0xdeca20: two-sided Moller-Trumbore over 4 lanes, the fraction of ray at the hit
fn hitFraction(origin: V4, ray: V4, tri: [3]V4) ?f32 {
    const v0: V4 = tri[0];
    const e1 = @as(V4, tri[1]) - v0;
    const e2 = @as(V4, tri[2]) - v0;
    const p = body.cross(ray, e2);
    const det = dot4(p, e1);
    if (det > -1e-6 and 1e-6 > det) return null;
    const inv = 1 / det;
    const s = origin - v0;
    const u = dot4(p, s) * inv;
    if (-1e-6 > u or u > just_past_one) return null;
    const q = body.cross(s, e1);
    const w = dot4(ray, q) * inv;
    if (-1e-6 > w or w + u > just_past_one) return null;
    const t = inv * dot4(e2, q);
    if (-1e-6 > t or t > just_past_one) return null;
    return t;
}

const just_past_one: f32 = 0x1.000010p0; // 0x272f9e0

// 0xdfdda0 TriangleCache::rayCast: the nearest hit from start to end, the first triangle on a tie
pub fn rayCast(tris: []const Triangle, start: V4, end: V4) ?Hit {
    const ray = end - start;
    var best: f32 = 1e38;
    var at: ?usize = null;
    for (tris, 0..) |tri, k| {
        const t = hitFraction(start, ray, .{ tri.v[0], tri.v[1], tri.v[2] }) orelse continue;
        if (best > t) {
            best = t;
            at = k;
        }
    }
    const tri = tris[at orelse return null];
    const v0: V4 = tri.v[0];
    const nv = body.cross(@as(V4, tri.v[1]) - v0, @as(V4, tri.v[2]) - v0);
    const s = nv * nv;
    return .{ .t = best, .normal = body.normalize4(nv, (s[0] + s[2]) + (s[1] + s[3])), .material = tri.material };
}

pub const Push = extern struct { depth: f32 = 0, push: [4]f32 = .{ 0, 0, 0, 0 }, normal: [4]f32 = .{ 0, 0, 0, 0 }, material: u16 = none };

// 0xf355a0 SuspensionSystem::ellipsoidWheelCollision: the tyre as an ellipsoid about center against the triangles
// whose material asks for it; the deepest along the suspension axis wins (the first on a tie). side: the wheel's
// side axis, axis: the suspension axis. push = depth along the normal less that along the axis
pub fn ellipsoid(tris: []const Triangle, materials: [*]const Material, w: Wheel, center: V4, side: V4, axis: V4) Push {
    var out: Push = .{};
    for (tris) |tri| {
        if (!materials[tri.material].ellipsoid) continue;
        const v = [3]V4{ tri.v[0], tri.v[1], tri.v[2] };
        const nv = body.cross(v[1] - v[0], v[2] - v[0]);
        const sq = nv * nv;
        const n = body.normalize4(nv, (sq[0] + sq[2]) + (sq[1] + sq[3]));
        var p = center - splat(dot4(n, center) - dot4(v[0], n)) * n; // center in the plane, then into the triangle
        for (0..3) |k| {
            const a = v[k];
            const e = v[(k + 1) % 3] - a;
            const mv = body.cross(e, n);
            const ms = mv * mv;
            const m = body.normalize4(mv, (ms[0] + ms[2]) + (ms[1] + ms[3]));
            const r = p - a;
            const out_of_edge = dot4(m, r);
            if (!(out_of_edge > 0)) continue;
            if (0 > dot4(e, r)) {
                p = a;
            } else if (0 > dot4(e, e - r)) {
                p = e + a;
            } else {
                p = p - m * splat(out_of_edge);
            }
        }
        const diff = center - p;
        const ds = diff * diff;
        const dist = @sqrt((ds[0] + ds[2]) + (ds[1] + ds[3]));
        if (0.0001 > dist or dist > w.max_radius) continue;
        var dir = diff / splat(dist);
        const c = @abs(dot4(side, dir));
        const reach = if (c > w.critical_cos) w.width / c else w.radius / @sqrt(1 - c * c);
        const depth = reach - dist;
        if (0 > depth) continue;
        var along = dot4(dir, axis) * depth;
        if (0 > along) {
            along = reach - along;
            dir = -dir;
        }
        if (!(along > out.depth)) continue;
        out = .{ .depth = along, .push = splat(depth) * dir - axis * splat(along), .normal = dir, .material = tri.material };
    }
    return out;
}

// 0xf7a880 RigidBody::calculateVelocityAtRelativePosition
pub fn velocityAt(b: body.RigidBody, r: V4) V4 {
    return body.cross(b.angvel, r) + @as(V4, b.vel);
}

// 0xf34430 SuspensionSystem::getBumpOffsetForSurface: value noise on a 256 x 256 lattice over x, z in wavelengths,
// the lattice values hashed from square roots
pub fn bumpOffset(m: Material, at: V4, scale: f32) f32 {
    const x: u32 = @bitCast(math.truncate(at[0] / m.bumps_wavelength * 256));
    const z: u32 = @bitCast(math.truncate(at[2] / m.bumps_wavelength * 256));
    const fx, const cx = .{ x & 0xff, x >> 8 & 0xff };
    const fz, const cz = .{ z & 0xff, z >> 8 & 0xff };
    const cx1 = (cx + 1) & 0xff;
    const cz1 = (cz + 1) & 0xff;
    const near_x = (lattice(cx1 << 8 | cz) * fx + lattice(cx << 8 | cz) * (256 - fx)) >> 8;
    const far_x = (lattice(cx1 << 8 | cz1) * fx + lattice(cx << 8 | cz1) * (256 - fx)) >> 8;
    const v: i32 = @bitCast((far_x * fz + near_x * (256 - fz)) >> 8);
    return @as(f32, @floatFromInt(v - 128)) * m.bumps_magnitude * 0.0078125 * scale;
}

fn lattice(k: u32) u32 {
    const r = @sqrt(@as(f32, @floatFromInt(k)) + 2.158833);
    return @as(u32, @bitCast(math.truncate(r * 987345.6))) & 0xff;
}

// 0xf59230 Wheel::setContactSurface: the surface under the wheel becomes the target, blended toward at
// |target - current| per unit rate; from the air it applies at once. Also the tyre's graphs for it
fn setContactSurface(g: Ground, w: *Wheel, i: usize, k: u16) void {
    if (w.contact_surface == k) return;
    if (k == none) {
        w.surface = std.mem.zeroes(suspension.Surface);
        w.surface_target = w.surface;
        tyre.setActiveSurface(&w.tyre, &w.set_target, &w.set_rate, g.compound(i), 0);
        w.contact_surface = none;
        w.contact = false;
        w.ground_effect = 0;
        return;
    }
    const s = g.materials[k].simulation;
    w.surface_target = s;
    tyre.setActiveSurface(&w.tyre, &w.set_target, &w.set_rate, g.compound(i), s.tyre_code);
    if (w.contact_surface == none) {
        w.surface_rate = std.mem.zeroes(suspension.Surface);
        w.surface = s;
    } else {
        const rate: *[10]f32 = @ptrCast(&w.surface_rate);
        for (rate, @as(*const [10]f32, @ptrCast(&w.surface_target)), @as(*const [10]f32, @ptrCast(&w.surface))) |*r, a, b| r.* = @abs(a - b);
    }
    w.contact_surface = k;
    w.contact = true;
    w.ground_effect = s.resists_torque_factor;
}
