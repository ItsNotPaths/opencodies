#!/usr/bin/env python3
"""DiRT 3 cars and surfaces to the sim's dataset (sim/src/car.zig, sim/src/material.zig), as ZON.

usage: d3.py [CAR ...] [--install DIR] [--out DIR] [--abs-assist]

CAR is a folder name under cars/models (default: ffr). The install comes from --install or the D3 environment
variable. For each car it writes <out>/d3-<car>/car.zon, and once <out>/d3/materials.zon (default out: runs/).

The script does what the game does when it loads a car: the .ctf loader (0x7f6090), the setup apply (0x7ea700) with
the wheel positions (0x7d6070), the wheel init (0x7d31e0), the suspension axes (0x7e5f30) and the spring preloads
(0x7dbe50). The hull comes from the .nd2 (SIMPLECOLL, COLLISION; 0x7eb0f0), the drawn wheel extents from the
_highLOD.pssg, the substep lengths from dirt3_game.exe. Floats follow the game's f32 order of operations.
Stdlib only.
"""
import argparse
import math
import os
import re
import struct
import sys
from decimal import Decimal
from fractions import Fraction
from pathlib import Path

DEFAULT_INSTALL = "/run/media/paths/SSS-Games/SteamLibrary/steamapps/common/DiRT 3 Complete Edition"
ROOT = Path(__file__).resolve().parent.parent


# f32 arithmetic: an f64 operation on f32 values, rounded once more, gives the f32 result for + - * / sqrt


def f(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


def add(a, b):
    return f(a + b)


def sub(a, b):
    return f(a - b)


def mul(a, b):
    return f(a * b)


def div(a, b):
    return f(a / b)


def tanf(x):
    return f(math.tan(x))


# the .ctf: a version, then the fields in alphabetical order, each read only from its version on (@lo) or up to a
# version (@lo-hi). i: int, s: string, curve: count, step, values. ctfwalk.py found this order in the loader.
CTF_LAYOUT = """
abs_strength@10 air_density_multiple@8 air_resistance_x air_resistance_y air_resistance_z ambient_temperature@12
anti_roll_front anti_roll_rear atc_strength@10 auto_clutch_full_point auto_clutch_gear_delay@14 auto_clutch_zero_point
boost_duration@28 boost_multiplier@28 i:boost_enabled brake_bias@9 brake_torque_total@9 bumpy_amplification@21
brake_torque_front@1-8 brake_torque_rear@1-8 bumpy_surface_factor camber_back_grip@30 camber_back_left
camber_back_right camber_front_grip@30 camber_front_left camber_front_right centre_diff_full_effect_rate
centre_diff_max_torque centre_diff_no_effect_rate chase_cam_velocity_prop clutch_gear_time clutch_lift_rate
clutch_max_torque s:name com_y com_z cs_history_time@24 cs_maximum_boost@24 cs_tighest_corner_radius@24
da_additional_acc@26 da_drift_angle@26 da_enable_speed@26 da_full_speed@26 da_proportion_to_turn_acc@26 damping_front
damping_rear diff_off_throttle_multiplier i:downforce_enabled downforce_central downforce_directionality
downforce_pressure_point downforce_pressure_point_in_air downforce_pressure_point_in_air_upwards
downforce_proportion_front_bumper downforce_proportion_front_wing downforce_proportion_spoiler downforce_rear_and_front
i:drive_int s:drive engine_friction engine_inertia s:layout i:int63 i:int64 front_burst_tyre_grip_multiple front_castor
front_damper_post_zone_scale@1-6 front_damper_zone_division front_diff_full_effect_rate front_diff_max_torque
front_diff_no_effect_rate front_fast_bump@7 front_fast_rebound@7 front_peak_slip front_peak_slp_ratio@22
front_peak_tyre_depth front_pressure@4 front_radius front_rebound_damper_multiplier@1-6 i:int80 front_roll_centre
front_rolling_resistance front_slow_rebound@7 front_tyre_spring_constant@1-3 front_visual_camber front_visual_sqsh_cam
front_visual_sqsh_lin front_visual_sqsh_quad front_visual_stch_cam front_visual_stch_lin front_visual_stch_quad
front_width@4 i:engine_life i:engine_life_power_percent gear_1st gear_1st_downshift_multiple gear_2nd gear_3rd gear_4th
gear_5th gear_6th gear_7th gear_8th gear_all_downshift_multiple@17 gear_change_throttle_off_time
gear_change_throttle_on_time gear_power_mod_1@35 gear_power_mod_2@35 gear_power_mod_3@35 gear_power_mod_4@35
gear_power_mod_5@35 gear_power_mod_6@35 gear_power_mod_7@35 gear_reverse grip_front grip_rear grip_weight_1@29
grip_weight_2@29 grip_weight_3@29 gsteer_lock_reduce_rate@27 gsteer_proportion@27 handbrake_grip_loss
handbrake_strength_multiple i:int124 high_speed_for_steering_rate high_speed_steering_rate idle_rate inerter_front@23
inerter_rear@23 inertia_x inertia_y inertia_z jump_anti_gravity@19 jumpassist_fall_rate@18 jumpassist_multiple_cap@18
jumpassist_rise_rate@18 i:int137 i:int138 i:int139@13 low_speed_for_steering_rate low_speed_steering_rate
lowspeed_engine_momentum_boost magic_brake mass max_steering_lock max_wheel_height_front max_wheel_height_rear
min_wheel_height_front min_wheel_height_rear s:full_name i:pad_curve_enabled pad_auto_steer pad_linearity curve:pad_lock
pad_min_lock i:int156 curve:power rear_burst_tyre_grip_multiple rear_castor rear_damper_post_zone_scale@1-6
rear_damper_zone_division rear_diff_full_effect_rate rear_diff_max_torque rear_diff_no_effect_rate rear_drive_proportion
rear_fast_bump@7 rear_fast_rebound@7 rear_peak_slip rear_peak_slp_ratio@22 rear_peak_tyre_depth rear_pressure@4
rear_radius i:int173 rear_roll_centre rear_rolling_resistance rear_slow_rebound@7 rear_tyre_spring_constant@1-3
rear_visual_camber rear_visual_sqsh_cam rear_visual_sqsh_lin rear_visual_sqsh_quad rear_visual_stch_cam
rear_visual_stch_lin rear_visual_stch_quad rear_width@4 rebound_damping_multiple@1-6 rev_limit
reverse_to_1st_transition_speed ride_height_adjust_front ride_height_adjust_rear@5 road_angle_for_no_grip
selfsteer_angle_threshold i:int193 selfsteer_low_speeds selfsteer_lowangle_threshold selfsteer_max_return_rate@16
selfsteer_min_return_rate@16 selfsteer_only_if_correcting@2 selfsteer_only_when_needed i:int200@16
selfsteer_speed_threshold skidding_grip_drop speed_for_full_pad_lock speed_for_full_starting_wheelspin
speed_for_no_starting_wheelspin speed_for_optimal_launch@34 stability_control_max_angle@25 stability_control_max_atc@25
stability_control_off_speed@25 stability_control_on_speed@25 starter_motor_strength_multiple stating_wheelspin_grip_loss
steering_rate_to_centre_mult steering_rate_to_desired_mult suspension_strength_front suspension_strength_rear
third_spring_dmp_front@23 third_spring_dmp_rear@23 third_spring_str_front@23 third_spring_str_rear@23 toe_in_front
toe_in_rear s:locked_tuning@11 s:hidden_tuning@11 tyre_damping_factor_front@6 tyre_damping_factor_rear@6
tyre_inertia_multiple_with_drift_steer@13 tyre_inertias_high_speed tyre_inertias_lowest_speed tyre_inertias_mid_speed
tyre_patch_shape_front@12 tyre_patch_shape_rear@12 tyre_patch_temp_road_rate@12 tyre_patch_temp_roll_rate@12
tyre_patch_temp_spread_rate@12 tyre_patch_temp_work_rate@12 tyre_x_inertia_multiple_high tyre_x_inertia_multiple_mid
tyre_y_inertia_multiple_high tyre_y_inertia_multiple_mid tyre_z_inertia_multiple_high tyre_z_inertia_multiple_mid
i:int243@15 s:version_tag visual_acc_cap_pitch visual_acc_cap_roll visual_acc_new_rate visual_acc_scale_pitch
visual_acc_scale_roll visual_amplification_pitch visual_amplification_roll visual_front_max_height
visual_rear_max_height visual_turn_rate visual_vib_bottom_amount visual_vib_bottom_speed visual_vib_frequency
visual_vib_interval visual_vib_max_threshold visual_vib_min_threshold visual_vib_top_amount visual_vib_top_speed
visual_wheel_xv_limit wear_rate_multiple weather_power_multiple@8 weight_grip_power wheel_force_bottom_factor@20
wheel_inertia_front wheel_inertia_rear wheel_max_side_shift@3 wheel_side_force_to_shift@3 wiper_cycle_offset
edamp_accel_damping@32 edamp_accel_threshold@32 edamp_deccel_damping@32 edamp_deccel_threshold@32 edamp_g_sensor@32
edamp_lateral_multiple@32 edamp_lateral_threshold@32 front_rebound_cutoff@30 front_rebound_zone_division@31
gear_downshift_skid@33 rear_rebound_cutoff@30 rear_rebound_zone_division@31
"""

# the loader's default for a field the file's version does not have (0 if not listed)
CTF_DEFAULTS = dict(abs_strength=1, air_density_multiple=1, ambient_temperature=25, auto_clutch_gear_delay=-1,
                    boost_duration=6, boost_multiplier=2, bumpy_amplification=1, camber_back_grip=1, camber_front_grip=1,
                    cs_history_time=0.3, cs_tighest_corner_radius=60, da_drift_angle=0.25, da_enable_speed=4,
                    da_full_speed=8, gear_all_downshift_multiple=0.75, grip_weight_1=0.7, grip_weight_2=0.2,
                    gsteer_lock_reduce_rate=0.1, jumpassist_fall_rate=1, jumpassist_multiple_cap=1,
                    stability_control_max_angle=0.125, stability_control_off_speed=10, stability_control_on_speed=20,
                    selfsteer_max_return_rate=999999, selfsteer_min_return_rate=999998, speed_for_optimal_launch=-1,
                    tyre_damping_factor_front=0.2, tyre_damping_factor_rear=0.2, tyre_inertia_multiple_with_drift_steer=1,
                    weather_power_multiple=1, wheel_max_side_shift=1, wheel_side_force_to_shift=6.66e-05,
                    edamp_lateral_multiple=1, gear_downshift_skid=10, **{"gear_power_mod_%d" % i: 1 for i in range(1, 8)})


def read_ctf(path):
    """{field: f32, string or int; curves as (step, [values])}."""
    d = Path(path).read_bytes()
    _, version = struct.unpack("<II", d[:8])
    if version < 7:
        sys.exit("%s: CTF version %d is older than the dampers this script reads (7)" % (path, version))
    p, out, present = 8, {}, set()
    for item in CTF_LAYOUT.split():
        kind, _, name = item.rpartition(":")
        name, _, span = name.partition("@")
        lo, _, hi = span.partition("-")
        if version < int(lo or 0) or version > int(hi or 99):
            if not kind:
                out.setdefault(name, f(CTF_DEFAULTS.get(name, 0)))
            continue
        present.add(name)
        if kind == "":
            out[name] = struct.unpack_from("<f", d, p)[0]
            p += 4
        elif kind == "i":
            out[name] = struct.unpack_from("<i", d, p)[0]
            p += 4
        elif kind == "s":
            end = d.index(b"\0", p)
            out[name] = d[p:end].decode("latin1")
            p = end + 1
        else:
            n, step = struct.unpack_from("<if", d, p)
            out[name] = (step, list(struct.unpack_from("<%df" % n, d, p + 8)))
            p += 8 + 4 * n
    return out, present, version


def read_tng(path):
    """tuning.tng: the setup sliders. Each moves CTF fields along points (x: slider, y: value); a point in percent
    scales the car's own value. Layout from the loader 0x7d2b60."""
    d = Path(path).read_bytes()
    _, count, size = struct.unpack_from("<III", d, 0)
    strings = d[12:12 + size]
    text = lambda o: None if o == -1 else strings[o:strings.index(b"\0", o)].decode("latin1")
    p, sliders = 12 + size, []
    for _ in range(count):
        title, ident, named = struct.unpack_from("<iHi", d, p)
        if named:
            field, scale = struct.unpack_from("<if", d, p + 10)
            field, default, p = text(field), 0.0, p + 18
        else:
            field, scale, default, p = None, 1.0, struct.unpack_from("<f", d, p + 10)[0], p + 14
        (kind,) = struct.unpack_from("<I", d, p)
        p += 4 + {0: 8, 1: 8, 4: 8, 2: 4, 3: 4}.get(kind, 0)
        (n,) = struct.unpack_from("<I", d, p)
        points = []
        for k in range(n):
            name, x, y, flags = struct.unpack_from("<iffI", d, p + 4 + 16 * k)
            points.append((text(name), x, y, flags & 1, flags >> 1 & 1))
        p += 4 + 16 * n
        sliders.append(dict(title=text(title), id=ident, field=field, scale=f(scale), default=f(default), points=points))
    return sliders


def tune(c, present, sliders, simple):
    """0x7fdd40, 0x4ed940 and 0x8017a0: each slider starts at its field's file value times its scale (or its
    default); the six simple sliders then take the player's settings (simple: {title: value}); the fields each
    slider moves take the value at its position. Points in percent scale with the start position and the field's
    file value, so a slider that did not move can still change a value by one ulp ((y + 100) x value x 0.01)."""
    locked = {int(n) for n in c.get("locked_tuning", "").split("/") if n.strip()}
    raw = dict(c)
    for s in sliders:
        if s["id"] in locked or s["field"] is not None and s["field"] not in present:
            continue
        start = s["default"] if s["field"] is None else mul(s["scale"], raw[s["field"]])
        at = f(simple.get(s["title"], start))
        done = set()
        for name, *_ in s["points"]:
            if name in done or name not in present:
                continue
            done.add(name)
            base = c[name]
            xs, ys = [], []
            for n, x, y, x_pct, y_pct in s["points"]:
                if n == name:
                    xs.append(mul(mul(add(x, f(100)), start), f(0.01)) if x_pct else add(start, x))
                    ys.append(mul(mul(add(y, f(100)), base), f(0.01)) if y_pct else add(y, base))
            for _ in range(len(xs)):  # the game's bubble sort by x
                for k in range(len(xs) - 1):
                    if xs[k + 1] < xs[k]:
                        xs[k], xs[k + 1], ys[k], ys[k + 1] = xs[k + 1], xs[k], ys[k + 1], ys[k]
            k = next((k for k in range(len(xs) - 2) if at <= xs[k + 1]), len(xs) - 2)
            span = sub(xs[k + 1], xs[k])
            t = f(0.5) if span == 0 else div(sub(at, xs[k]), span)
            c[name] = add(mul(sub(f(1), t), ys[k]), mul(ys[k + 1], t))


def peak_slip(c, axle):
    """deg to rad as the loader does it (x87, then stored)"""
    return f(c[axle + "_peak_slip"] * f(math.pi) * f(1 / 180))


# binary XML (.nd2, surface_materials.xml): \0BXML\0, then elements: u32 size, attribute count, name, the attribute
# pairs, one byte; children follow, then the end mark 04 05 00 00 00 00 (04 06 .. ends the file)


def read_bxml(path):
    """[(name, {attribute: text})] in file order."""
    d = Path(path).read_bytes()
    if not d.startswith(b"\0BXML\0"):
        sys.exit("%s: not binary XML" % path)
    p, out = 6, []

    def string():
        nonlocal p
        end = d.index(b"\0", p)
        s = d[p:end].decode("latin1")
        p = end + 1
        return s

    while p < len(d):
        if d[p] == 4 and d[p + 1] in (5, 6):
            p += 6
            continue
        count = d[p + 4]
        p += 5
        name = string()
        attrs = {}
        for _ in range(count):
            key = string()
            attrs[key] = string()
        p += 1
        out.append((name, attrs))
    return out


def element(xml, name):
    return next(a for n, a in xml if n == name)


def number(attrs, key, default=0.0):
    """atof, then a float cast, as the loaders do"""
    return f(float(attrs[key])) if key in attrs else f(default)


# the player's simple setup (0x4ed940 sets these sliders from the profile, 3 bits each times a scale); the final drive
# of a new profile, as read in the benchmark snapshots. The other five keep their start positions.
NEW_PROFILE_FINAL_DRIVE = 1 - 2 ** -24

# the substep lengths over the mean (0xefd888, 21 floats)
JITTER_VA, JITTER_COUNT = 0xEFD888, 21


def read_exe_floats(exe, va, count):
    d = Path(exe).read_bytes()
    pe = struct.unpack_from("<I", d, 0x3C)[0]
    sections, opt_size = struct.unpack_from("<H", d, pe + 6)[0], struct.unpack_from("<H", d, pe + 20)[0]
    image_base = struct.unpack_from("<I", d, pe + 52)[0]
    at = pe + 24 + opt_size
    for i in range(sections):
        vsize, vaddr, rsize, raw = struct.unpack_from("<IIII", d, at + 40 * i + 8)
        if vaddr <= va - image_base < vaddr + rsize:
            return list(struct.unpack_from("<%df" % count, d, raw + va - image_base - vaddr))
    sys.exit("%s: no section holds 0x%x" % (exe, va))


def wheel_extents(pssg):
    """{corner: (min x, max x)} of the x0_wheel_<corner> bounding boxes: the drawn wheel along its axle"""
    d = Path(pssg).read_bytes()
    if not d.startswith(b"PSSG"):
        return wheel_extents_text(d)
    _, _, node_count = struct.unpack_from(">III", d, 4)
    p, names, attrs = 16, {}, {}
    for _ in range(node_count):
        nid, n = struct.unpack_from(">II", d, p)
        names[nid] = d[p + 8:p + 8 + n].decode()
        p += 8 + n
        (count,) = struct.unpack_from(">I", d, p)
        p += 4
        for _ in range(count):
            aid, n = struct.unpack_from(">II", d, p)
            attrs[aid] = d[p + 8:p + 8 + n].decode()
            p += 8 + n
    out = {}

    def walk(p, end):
        while p + 12 <= end:
            nid, size, asize = struct.unpack_from(">III", d, p)
            name, body, stop = names.get(nid), p + 12 + asize, p + 8 + size
            if name is None or stop > end or body > stop:
                return  # not a node list
            ident = node_id(d, p + 12, body, attrs)
            if name == "MATRIXPALETTEJOINTNODE" and ident and ident.startswith("x0_wheel_"):
                box = child(d, body, stop, names, "BOUNDINGBOX")
                if box is not None:
                    lo_x, _, _, hi_x, _, _ = struct.unpack_from(">6f", d, box)
                    out[ident[len("x0_wheel_"):]] = (lo_x, hi_x)
            elif name in ("PSSGDATABASE", "ROOTNODE", "NODE", "MATRIXPALETTEJOINTNODE", "MATRIXPALETTENODE",
                          "MATRIXPALETTEBUNDLENODE", "LIBRARY"):
                walk(body, stop)
            p = stop

    walk(p, len(d))
    return out


def wheel_extents_text(d):
    """the same from a PSSG written as XML text (9 numbers: 9 significant digits round trip an f32)"""
    out = {}
    for m in re.finditer(rb'id="x0_wheel_(\w\w)"[^<]*>\s*<TRANSFORM[^<]*</TRANSFORM>\s*<BOUNDINGBOX\s*>([^<]*)<', d):
        box = [f(float(x)) for x in m.group(2).split()]
        out[m.group(1).decode()] = (box[0], box[3])
    return out


def node_id(d, p, end, attrs):
    while p < end:
        aid, n = struct.unpack_from(">II", d, p)
        if attrs.get(aid) == "id" and n > 4:
            return d[p + 12:p + 8 + n].decode("latin1")
        p += 8 + n
    return None


def child(d, p, end, names, want):
    while p < end:
        nid, size, asize = struct.unpack_from(">III", d, p)
        if names.get(nid) == want:
            return p + 12 + asize
        p += 8 + size
    return None


# the hull: 0x7eb0f0 cuts the SIMPLECOLL outline into cells (shared/chassis.zig split, here at import time)
HULL_MARGIN = f(0.125)  # shape+0xd8
TRACK_MARGIN = f(0.02)  # shape+0xd8 of a track triangle
HULL_FRICTION = f(0.4)  # the body material
HULL_SOFTNESS = f(4.83e4)  # N s/m per m of depth [measured at touchdown]; the game derives it from the damage zones
CORNER_MERGE = f(0.05)  # m, 0x7e2970 drops a corner this close to an earlier one
MIN_FLOOR = f(0.176)  # m, 0xefe764


def hull(nd2, origin):
    planes_attrs, coll = element(nd2, "SIMPLECOLL"), element(nd2, "COLLISION")
    planes = []
    for i in range(int(planes_attrs.get("m_numPlanes", 0))):
        planes.append([number(planes_attrs, "p%d_%s" % (i, k)) for k in "xyzd"])
    floor = hull_floor(number(coll, "m_cs_bottom_y"), number(coll, "m_groundClearence", 0.17))
    cuts = [[f(float(x)) for x in coll[key].split()][:int(coll[n]) - 1] for key, n in
            (("m_subdivX_pos", "m_subdivX"), ("m_subdivZ_pos", "m_subdivZ"))]
    across = (number(coll, "m_cs_right_x"), number(coll, "m_cs_left_x"))
    along = (number(coll, "m_cs_rear_centre_z"), number(coll, "m_cs_front_centre_z"))
    pieces = cut(planes, floor, across, along, cuts, TRACK_MARGIN, HULL_MARGIN, origin)
    return dict(pieces=pieces, margin=HULL_MARGIN, friction=HULL_FRICTION, softness=HULL_SOFTNESS)


def cut(planes, floor, across, along, cuts, overlap, margin, origin):
    """the outline (planes [x, y, z, d], inside where n.x <= d) in cells by x, then z: each face moves in by the
    margin less the track's, a floor face no lower than the floor, a cut face out by the overlap; corners in the
    body frame (+ origin)"""
    inset = sub(TRACK_MARGIN, margin)
    pieces = []
    for ix in range(len(cuts[0]) + 1):
        for iz in range(len(cuts[1]) + 1):
            cell = []
            for p in planes:
                flat = p[1] <= -0.99 and abs(p[0]) < 0.017 and abs(p[2]) < 0.017
                cell.append(((p[0], p[1], p[2]), (min(p[3], -floor) if flat else p[3]) + inset))
            cell += slabs((1.0, 0.0, 0.0), across, cuts[0], ix, overlap)
            cell += slabs((0.0, 0.0, 1.0), along, cuts[1], iz, overlap)
            pieces.append([[add(c, o) for c, o in zip(v, origin)] for v in corners(cell)])
    return pieces


def hull_floor(bottom, clearance, extra=f(0)):
    """0x7eb100: the floor sits at the bottom plus the clearance, and at least MIN_FLOOR up; extra is the game's
    additional clearance (0x1029cf8, 0 outside the menus)"""
    if clearance == -1000:
        return f(-2)
    lift = sub(add(extra, MIN_FLOOR), add(clearance, bottom))
    return add(add(clearance, min(max(lift, f(0)), f(9999))), bottom)


def slabs(axis, extent, cuts, i, overlap):
    lo, hi = extent
    lerp = lambda t: add(mul(sub(hi, lo), t), lo)
    out = []
    if i > 0:
        out.append((tuple(-a for a in axis), -lerp(cuts[i - 1]) + overlap))
    if i < len(cuts):
        out.append((axis, lerp(cuts[i]) + overlap))
    return out


def corners(planes):
    out = []
    for i, a in enumerate(planes):
        for j in range(i + 1, len(planes)):
            for c in planes[j + 1:]:
                x = meet(a, planes[j], c)
                if x is None or not all(dot(n, x) - d <= 1e-5 for n, d in planes):
                    continue
                v = [f(t) for t in x]
                if not any(length32([sub(p, q) for p, q in zip(v, w)]) < CORNER_MERGE for w in out):
                    out.append(v)
    return out


def meet(a, b, c):
    """the point on three planes (Cramer's rule), f64"""
    bc = cross(b[0], c[0])
    det = a[0][0] * bc[0] + a[0][1] * bc[1] + a[0][2] * bc[2]
    if abs(det) < 1e-9:
        return None
    ca, ab = cross(c[0], a[0]), cross(a[0], b[0])
    return [(bc[k] * a[1] + ca[k] * b[1] + ab[k] * c[1]) / det for k in range(3)]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def length32(v):
    return f(math.sqrt(add(add(mul(v[0], v[0]), mul(v[1], v[1])), mul(v[2], v[2]))))


# the car


def wheel_positions(c, nd2):
    """0x7d6070: the .nd2 wheels less the ride height, moved so the body origin sits at the centre of mass.
    Returns the four positions (RL RR FL FR, +x left) and the model-to-body shift."""
    w = element(nd2, "WHEELS")
    rx, ry, rz, fx, fy, fz = (number(w, "m_%s_%s" % (a, k)) for a in ("rear", "front") for k in "xyz")
    radius = {a: number(w, "m_%s_r" % a) for a in ("rear", "front")}  # the gravel tyres
    rh_r, rh_f = c["ride_height_adjust_rear"], c["ride_height_adjust_front"]
    p = [[rx, sub(ry, rh_r), rz], [-rx, sub(ry, rh_r), rz], [fx, sub(fy, rh_f), fz], [-fx, sub(fy, rh_f), fz]]
    r = [radius["rear"], radius["rear"], radius["front"], radius["front"]]
    track = mul(add(sub(p[2][0], p[3][0]), sub(p[0][0], p[1][0])), f(0.5))
    up = -add(mul(track, c["com_y"]), mul(add(rh_r, rh_f), f(0.5)))
    sag = mul(add(add(add(sub(p[1][1], r[1]), sub(p[0][1], r[0])), sub(p[2][1], r[2])), sub(p[3][1], r[3])), f(0.25))
    rear_z, front_z = mul(add(p[1][2], p[0][2]), f(0.5)), mul(add(p[2][2], p[3][2]), f(0.5))
    shift = [-mul(add(add(add(p[0][0], p[1][0]), p[2][0]), p[3][0]), f(0.25)), sub(up, sag),
             -add(mul(sub(front_z, rear_z), c["com_z"]), rear_z)]
    return [[add(s, x) for s, x in zip(shift, q)] for q in p], shift


def axis(c, axle, side):
    """0x7e5f30: normalize(2 roll centre toward the wheel's side, 1, tan castor). The game uses rsqrtps and one
    Newton step; this is the exact square root, equal on the cars checked."""
    v = [mul(c[axle + "_roll_centre"], f(2 * side)), f(1), tanf(f(c[axle + "_castor"] * f(math.pi) * f(1 / 180)))]
    s = add(add(add(mul(v[1], v[1]), mul(v[0], v[0])), f(0)), add(mul(v[2], v[2]), f(0)))
    n = math.sqrt(s)
    return [f(x / n) for x in v]


def tyre_spring(c, axle):
    """the loader (version > 3): the tyre's vertical stiffness from its pressure, width and radius, and its damping"""
    share = c["com_z"] if axle == "front" else sub(f(1), c["com_z"])
    load = mul(mul(mul(share, c["mass"]), f(0.5)), f(9.81))
    length = div(div(load, mul(c[axle + "_pressure"], f(6894.757))), c[axle + "_width"])
    r = c[axle + "_radius"]
    k = div(load, sub(r, f(math.sqrt(sub(mul(r, r), mul(mul(length, f(0.25)), length)))))) if length < mul(r, f(2)) else f(0)
    damping = mul(f(math.sqrt(mul(mul(mul(mul(c["mass"], share), f(0.5)), c["tyre_damping_factor_" + axle]), k))), f(2))
    return k, damping


def preloads(c, positions, axes):
    """0x7dbe50: at travel 0 each spring carries its static share of the weight, along its axis"""
    rear_z = mul(abs(add(positions[0][2], positions[1][2])), f(0.5))
    front_z = mul(abs(add(positions[2][2], positions[3][2])), f(0.5))
    total = add(front_z, rear_z)
    weight = c["mass"] * 0.5 * f(9.81)
    rear = -(weight * front_z / total / c["suspension_strength_rear"])  # x87, rounded once
    front = -div(f(weight * rear_z / total), c["suspension_strength_front"])
    return [mul(f(rear), axes[0][1]), mul(f(rear), axes[1][1]), mul(front, axes[2][1]), mul(front, axes[3][1])]


def abs_slip(assist):
    """the slip the ABS releases at: 0x7d0520 at 1.0 with the ABS assist, 0.4 without (0xb2a0b0)"""
    table = [1e6, 64, 32, 16, 8, 5, 4, 3, 2, 1, 0, 0]
    x = mul(f(1.0 if assist else 0.4), f(10))
    i = int(x)
    frac = sub(x, f(i))
    return f((1 - frac) * table[i] + frac * table[i + 1])


def car(install, name, abs_assist, simple):
    folder = Path(install) / "cars/models" / name
    c, present, version = read_ctf(folder / (name + ".ctf"))
    tune(c, present, read_tng(Path(install) / "cars/settings/tuning.tng"), simple)
    if version < 22:  # the loader's default: the peak slip angle's tangent
        for axle in ("front", "rear"):
            c[axle + "_peak_slp_ratio"] = tanf(peak_slip(c, axle))
    nd2 = read_bxml(folder / (name + ".nd2"))
    extents = wheel_extents(folder / (name + "_highLOD.pssg"))
    if len(extents) < 4:
        sys.exit("%s: the _highLOD.pssg has no x0_wheel_<corner> bounding boxes" % name)
    positions, shift = wheel_positions(c, nd2)
    axes = [axis(c, "rear", 1), axis(c, "rear", -1), axis(c, "front", 1), axis(c, "front", -1)]
    pre = preloads(c, positions, axes)
    springs = {a: tyre_spring(c, a) for a in ("front", "rear")}
    if c["drive"] != "Four":
        sys.exit("%s: %s-wheel drive; the friction-circle step drives four wheels (sim/src/d3/drivetrain.zig)" % (name, c["drive"]))
    rear_share = c["rear_drive_proportion"]
    split = (mul(rear_share, f(0.5)), mul(sub(f(1), rear_share), f(0.5)))  # 0x7ea700: each rear wheel, each front
    coll = element(nd2, "COLLISION")
    raise_to = sub(add(add(number(coll, "m_cs_bottom_y"), number(coll, "m_groundClearence", 0.17)), f(0.01)), f(0.25))
    brakes = brake_torque(c, version)
    wheels = []
    for i, (corner, side) in enumerate((("bl", 1), ("br", -1), ("fl", 1), ("fr", -1))):
        axle = "rear" if i < 2 else "front"
        top = max(c["max_wheel_height_" + axle], f(0.3))
        k, damping = springs[axle]
        lo, hi = extents[corner]
        wheels.append(dict(
            rest_position=positions[i], spring_axis=axes[i], radius=c[axle + "_radius"],
            inertia=c["wheel_inertia_" + axle], min_height=c["min_wheel_height_" + axle],
            max_height=raise_to if raise_to > top else top, bump_stop_height=c["max_wheel_height_" + axle],
            spring={"linear": dict(rate=c["suspension_strength_" + axle], preload=pre[i])},
            slow_bump=c["damping_" + axle], fast_bump=c[axle + "_fast_bump"], slow_rebound=c[axle + "_slow_rebound"],
            fast_rebound=c[axle + "_fast_rebound"], bump_zone=c[axle + "_damper_zone_division"],
            rebound_zone=c[axle + "_rebound_zone_division"], anti_roll=c["anti_roll_" + axle],
            tyre_spring=k, tyre_damping=damping, tyre_depth=c[axle + "_peak_tyre_depth"],
            tan_peak_slip=tanf(peak_slip(c, axle)), peak_slip_ratio=tanf(c[axle + "_peak_slp_ratio"]),
            grip=c["grip_" + axle],
            tyre={"circle": dict(rolling=c[axle + "_rolling_resistance"],
                                 camber=c["camber_%s_%s" % ("back" if i < 2 else "front", "left" if side > 0 else "right")],
                                 camber_grip=c["camber_%s_grip" % ("back" if i < 2 else "front")])},
            visual_camber=dict(angle=c[axle + "_visual_camber"],
                               gain=[c[axle + "_visual_stch_cam"], c[axle + "_visual_sqsh_cam"]], rim=[lo, hi]),
            max_brake_torque=brakes[axle], handbrake=i < 2, drive_proportion=split[0] if i < 2 else split[1]))
    step, power = c["power"]
    peak_torque = max((div(p, mul(f(i), step)) for i, p in enumerate(power) if i > 0), default=f(0))
    gears = [c["gear_%s" % n] for n in ("1st", "2nd", "3rd", "4th", "5th", "6th", "7th", "8th")]
    ratios = [gears[0]] + gears + [f(0)] + [c["gear_reverse"]]
    top_gear = max((i + 1 for i, g in enumerate(gears) if g > 0), default=0)
    life = f(c["engine_life"])
    pad_step, pad_curve = c["pad_lock"]
    return {
        "mass": c["mass"],
        "inertia": [c["inertia_x"], c["inertia_y"], c["inertia_z"]],
        "front_weight": c["com_z"],
        "hull": hull(nd2, [f(0), shift[1], shift[2]]),
        "wheels": wheels,
        "drag": [c["air_resistance_x"], c["air_resistance_y"], c["air_resistance_z"]],
        "aero": {"point": dict(downforce=c["downforce_central"], pressure_point=c["downforce_pressure_point"],
                               pressure_point_air=[c["downforce_pressure_point_in_air"],
                                                   f(c["downforce_pressure_point_in_air_upwards"] - 10)])},
        "engine": dict(inertia=c["engine_inertia"], idle=c["idle_rate"], rev_limit=c["rev_limit"],
                       curve={"power": dict(step=step, table=power,
                                            divisor=add(f(1), mul(div(life, life), mul(f(c["engine_life_power_percent"]), f(0.01)))),
                                            friction=mul(c["engine_friction"], f(3000)))}),
        "gearbox": dict(ratios=ratios, top_gear=top_gear, clutch_torque=mul(c["clutch_max_torque"], peak_torque),
                        shift_delay=c["auto_clutch_gear_delay"]),
        "diffs": {"rate": dict(**{a: dict(no_effect=c[a + "_diff_no_effect_rate"], full_effect=c[a + "_diff_full_effect_rate"],
                                          max_torque=c[a + "_diff_max_torque"]) for a in ("rear", "front", "centre")},
                               off_throttle=c["diff_off_throttle_multiplier"])},
        "clutch_order": [2, 3, 0, 1],  # 0x7ea700: four-wheel drive lists the fronts first
        "handbrake_strength": c["handbrake_strength_multiple"],
        "handbrake_grip_loss": c["handbrake_grip_loss"],
        "abs_slip": abs_slip(abs_assist),
        "bumps": dict(surface_factor=c["bumpy_surface_factor"], amplification=c["bumpy_amplification"]),
        "rebound_relief": c["edamp_g_sensor"],
        "tyre_grip_by_load": [c["grip_weight_1"], c["grip_weight_2"], c["grip_weight_3"]],
        "tyre_yaw": dict(mid_scale=c["tyre_y_inertia_multiple_mid"],
                         speeds=[c["tyre_inertias_lowest_speed"], c["tyre_inertias_mid_speed"]]),
        "substep_jitter": read_exe_floats(Path(install) / "dirt3_game.exe", JITTER_VA, JITTER_COUNT),
        "pad_lock": [mul(mul(v, f(math.pi)), f(1 / 180)) for v in pad_curve],
        "pad_lock_step": pad_step,
        "pad_linearity": c["pad_linearity"],
        "steer_rate": [c["low_speed_steering_rate"], c["high_speed_steering_rate"]],
        "steer_rate_speeds": [c["low_speed_for_steering_rate"], c["high_speed_for_steering_rate"]],
        "track": mul(positions[2][0], f(2)),
        "wheelbase": sub(positions[2][2], positions[0][2]),
        "jump_rates": [c["jumpassist_rise_rate"], c["jumpassist_fall_rate"]],
        "jump_cap": c["jumpassist_multiple_cap"],
    }, c["full_name"]


def brake_torque(c, version):
    """version 9 on: a total and a front bias (x87, rounded once)"""
    if version < 9:
        return {"front": c["brake_torque_front"], "rear": c["brake_torque_rear"]}
    total, bias = c["brake_torque_total"], c["brake_bias"]
    return {"front": f(total * bias), "rear": f((1 - bias) * total)}


# surfaces: surface_materials.xml as the loader 0x7ea0a0 fills it; only MECHANICS sets physics, a MATERIAL without it
# has 0 in every field


def materials(install):
    out = []
    for name, a in read_bxml(Path(install) / "surface_materials.xml"):
        if name == "MATERIAL":
            out.append(material(a["name"], {}))
        elif name == "MECHANICS":
            out[-1] = material(out[-1]["code"], a)
    return [m["value"] for m in out]


def material(code, a):
    value = {
        "name": name_bytes("d3", code),
        "depth": number(a, "depth"),
        "bumps": {"by_time": dict(wavelength=number(a, "bumpswavelength"), magnitude=number(a, "bumpsmagnitude"),
                                  contact_share=min(max(number(a, "bumpsrealproportion"), f(0)), f(1)),
                                  car_share=number(a, "bumpsproportionCarDependent"))},
        "slowdown": dict(from_speed=number(a, "minimumSpeedForSlowdown"), full_speed=number(a, "speedForMaximumSlowdown"),
                         amount=number(a, "slowdown")),
        "grip": number(a, "grip"),
        "hardness": number(a, "hardness"),
        "side_drag": dict(speed=number(a, "lss_speed"), coefficient=number(a, "lss_coefficent")),
    }
    return dict(code=code, value=value)


def name_bytes(origin, code):
    """Material.name: origin, '-', the 4-char code, 0 padded to 8"""
    b = (origin + "-" + code).encode("latin1")
    return Bytes(b + b"\0" * (8 - len(b)))


# ZON out: floats as the shortest decimal that parses to the same f32


class Bytes(bytes):
    pass


def f32_text(x):
    if x == 0:
        return "-0.0" if math.copysign(1, x) < 0 else "0"
    if math.isinf(x):
        return "-inf" if x < 0 else "inf"
    for digits in range(1, 10):
        s = "%.*g" % (digits, x)
        if nearest_f32(Fraction(s)) == x:
            if "e" in s and -5 <= math.floor(math.log10(abs(x))) < 16:
                s = format(Decimal(s), "f")
            return s.replace("e+", "e")
    raise ValueError(x)


def nearest_f32(q):
    """the f32 nearest to the exact rational q (ties to even), as an f64"""
    x = f(float(q))  # within one f32 step of the answer
    best = None
    for cand in (x, next_f32(x, -1), next_f32(x, 1)):
        d = abs(Fraction(cand) - q)
        even = struct.unpack("<I", struct.pack("<f", cand))[0] % 2 == 0
        if best is None or d < best[0] or d == best[0] and even:
            best = (d, cand)
    return best[1]


def next_f32(x, way):
    i = struct.unpack("<i", struct.pack("<f", x))[0]
    i += way if x > 0 or x == 0 and way > 0 else -way
    return struct.unpack("<f", struct.pack("<i", i))[0]


def zon(v, indent=0):
    pad = "    " * (indent + 1)
    if v is None:
        return "null"
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        return f32_text(v)
    if isinstance(v, Bytes):
        return ".{ " + ", ".join("'%s'" % chr(b) if 32 < b < 127 and chr(b) not in "'\\" else str(b) for b in v) + " }"
    if isinstance(v, dict):
        return ".{\n" + "".join("%s.%s = %s,\n" % (pad, k, zon(x, indent + 1)) for k, x in v.items()) + "    " * indent + "}"
    if all(isinstance(x, (int, float)) and not isinstance(x, bool) for x in v):
        return ".{ " + ", ".join(zon(x) for x in v) + " }"
    return ".{\n" + "".join("%s%s,\n" % (pad, zon(x, indent + 1)) for x in v) + "    " * indent + "}"


def write(path, header, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text("// %s\n%s\n" % (header, zon(value)))
    print(path)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cars", nargs="*", default=["ffr"])
    ap.add_argument("--install", default=os.environ.get("D3", DEFAULT_INSTALL))
    ap.add_argument("--out", default=str(ROOT / "runs"))
    ap.add_argument("--abs-assist", action="store_true", help="the game's ABS assist on (off in the benchmark runs)")
    ap.add_argument("--final-drive", type=float, default=NEW_PROFILE_FINAL_DRIVE,
                    help="the simple-setup final drive slider, 0..1 (default: a new profile's)")
    a = ap.parse_args()
    out = Path(a.out)
    for name in a.cars:
        value, full_name = car(a.install, name, a.abs_assist, {"Gears/Final Drive": a.final_drive})
        write(out / ("d3-" + name) / "car.zon", "%s (%s), from the DiRT 3 install by import/d3.py" % (name, full_name), value)
    write(out / "d3" / "materials.zon", "DiRT 3 surface_materials.xml by import/d3.py", materials(a.install))


if __name__ == "__main__":
    main()
