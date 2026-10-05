#!/usr/bin/env python3
"""Forza Horizon 1 cars and surfaces to the sim's dataset (sim/src/car.zig, sim/src/material.zig), as ZON.

usage: fh1.py [CAR ...] [--db gamedb.slt] [--physics physics.zip] [--out DIR] [--list] [--no-surfaces]

CAR is a Data_Car Id or MediaName (MAZ_Miata_94, FER_F40Competizione_89, AUD_R8GT_11, FOR_FocusRS500_10). The game
files come from --db / --physics or the FH1_DB / FH1_PHYSICS environment variables. For each car it writes
<out>/fh-<name>/car.zon (name: --name, else the lower-case model part of MediaName), and <out>/fh/materials.zon (default out: runs/).

Cars: the chain of InitCarDynInitFromDB and the Set*FromInit functions (docs/forza-horizon.md section 9). The game
runs in units of 100 kg (100 N, 100 N m, 100 kg m^2); this script multiplies those back to SI.
Surfaces: one fh-<code> Material per surface code of the D3 and DR1 lists, with curve_grip from the FH1 surface the
code stands for (surfaceTypes.xml in physics.zip, read with tools/fh/fhzip.py for the Xbox LZX entries). The code
to surface table is a [guess] by the code's letters: T tarmac, C concrete, G gravel, D dirt, M mud, S sand/snow/silt,
I ice, R rock.
"""
import argparse
import math
import os
import re
import sqlite3
import struct
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
FH1 = "/run/media/paths/SSS-Core/fh1-recomp/fh1-recompiled-release/game/media"
D3_SURFACES = Path(os.environ.get("D3", "/run/media/paths/SSS-Games/SteamLibrary/steamapps/common/DiRT 3 Complete Edition")) / "surface_materials.xml"
DR1_SURFACES = ROOT / "runs/dr1-555/materials.zon"
TYRE_GIVE = 0.01  # m the tyre gives before the travel moves [guess: the game derives it, not traced]


def f32(x):
    return struct.unpack("<f", struct.pack("<f", x))[0]


RPM = 0.10472  # rad/s per rpm, the game's constant
DEG = 0.017453292
G = 9.80665
KG = G * 0.01  # DB kg to game force
AERO = G * 0.0002223948516610489  # DB kgf at 150 mph to N/(m/s)^2

# PhysicsSettings.ini (FH1) where present, else the FH2 defaults
TUNE = dict(final_drive=(2.2, 6.1), gear=(0.48, 6), pressure=(15, 55), lsd=(0, 2000), lsd_decel=(0, 1000))
SIDEWALL = ((0.0781, 0.1143), (1, 1.5))  # m of sidewall -> slip scale
HEAVY_PEAK_SA = (4 * DEG, 18 * DEG)
WHEEL_INERTIA = dict(add=0.25, scale=1, floor=1.82)  # kg m^2 [likely: FH2 defaults]
UNSPRUNG = dict(add=4, scale=1, floor=27.2, clamp=1.5)  # kg [likely: FH2 defaults]
UPDATE_HZ = 359
BRAKE_BIAS_REAR_OFFSET = 0.03
HULL_MARGIN = 0.125  # m; margin, friction and softness are the D3 car's [guess]


def f32_redline(rev_limit, step, n):
    """the redline as the f32 the sim derived from the rev limit before (2 rev_limit - (n - 1) step), so the step's
    results stay bit for bit; the database's own redline differs by an ulp or so"""
    return f32(2 * f32(rev_limit) - f32((n - 1) * f32(step)))


def lerpc(x, x0, x1, y0, y1):
    if x1 == x0:
        return y0
    return y0 + min(1, max(0, (x - x0) / (x1 - x0))) * (y1 - y0)


def tuned(default, lo, hi):
    """A stock car's tune value: the default clamped to the range (GenSetTuneValueFromInit with tune -1)."""
    if lo == hi:
        return default
    lo, hi = min(lo, hi), max(lo, hi)
    return lo + min(1, max(0, (default - lo) / (hi - lo))) * (hi - lo)


class Db:
    def __init__(self, path):
        self.c = sqlite3.connect(path)
        self.c.row_factory = sqlite3.Row

    def one(self, q, *a):
        r = self.c.execute(q, a).fetchone()
        return dict(r) if r else None


def stock_parts(d, car_id):
    stock = lambda t, key, v: d.one(f'select * from {t} where {key}=? and IsStock=1', v)
    eng, dt, body = (stock('List_Upgrade' + t, 'Ordinal', car_id) for t in ('Engine', 'Drivetrain', 'CarBody'))
    P = {'Engine': eng, 'Drivetrain': dt, 'CarBody': body}
    for t in ('Brakes', 'SpringDamper', 'AntiSwayFront', 'AntiSwayRear', 'TireCompound', 'RearWing', 'RimSizeFront', 'RimSizeRear'):
        P[t] = stock('List_Upgrade' + t, 'Ordinal', car_id)
    for t in ('Camshaft', 'Valves', 'Displacement', 'PistonsCompression', 'FuelSystem', 'Ignition', 'Exhaust', 'Intake',
              'Flywheel', 'Manifold', 'RestrictorPlate', 'OilCooling', 'TurboSingle', 'TurboTwin', 'TurboQuad', 'CSC', 'DSC',
              'Intercooler'):
        P[t] = stock('List_UpgradeEngine' + t, 'EngineID', eng['EngineID'])
    for t in ('Clutch', 'Transmission', 'Driveline', 'Differential'):
        P[t] = stock('List_UpgradeDrivetrain' + t, 'DrivetrainID', dt['DrivetrainID'])
    for t in ('FrontBumper', 'RearBumper', 'Hood', 'SideSkirt', 'TireWidthFront', 'TireWidthRear', 'Weight', 'ChassisStiffness'):
        P[t] = stock('List_UpgradeCarBody' + t, 'CarBodyID', body['CarBodyID'])
    return P


def tyre_size(stock_w, stock_aspect, stock_rim, w, rim):
    """The installed size keeps the stock outer diameter."""
    ratio = ((stock_w * 0.001 * stock_aspect * 0.01 + stock_rim * 0.0127) * 2 - rim * 0.0254) * 0.5 / (w * 0.001) * 100
    return dict(width_mm=w, ratio=ratio, radius=w * 0.001 * ratio * 0.01 + rim * 0.0127, width=w * 0.001)


def curves(d, comp, size, pressure):
    """Lateral and longitudinal Curves (fh/tyre.zig) of one axle, after CTirePhysics::Init's width, sidewall and pressure scales."""
    width_scale = lerpc(size['width_mm'], comp['TireFricWidth0'], comp['TireFricWidth1'], comp['TireFricScale0'], comp['TireFricScale1'])
    sidewall = size['width_mm'] * size['ratio'] * 1e-5
    slip_scale = lerpc(sidewall, *SIDEWALL[0], *SIDEWALL[1])
    ac = d.one('select * from List_TireAffectCurve where AffectCurveID=?', comp['AffectCurveStartPressureAffectPeakSASRID'])
    if ac:
        vals = [ac['v%d' % i] * ac['OutputScale'] for i in range(ac['NumValues'])]
        x = (pressure - ac['MinInput']) * (ac['NumValues'] - 1) / (ac['MaxInput'] - ac['MinInput'])
        j = min(ac['NumValues'] - 2, max(0, int(x)))
        slip_scale *= vals[j] + (vals[j + 1] - vals[j]) * min(1, max(0, x - j))
    out = {}
    for key, unit in (('lateral', DEG), ('longitudinal', 1)):
        mc = d.one('select * from List_TireFrictionMultiCurve where FrictionMultiCurveID=?', comp['FrictionMultiCurve%sID' % key.title()])
        rows = [d.one('select * from List_TireFrictionCurve where FrictionCurveID=?', mc['TireFrictionCurveID%d' % c]) for c in (0, 1)]
        out[key] = dict(max_slip=mc['MaxSlip'] * unit * slip_scale,
                        loads=[mc['MinLoadCurve'] * G, mc['MaxLoadCurve'] * G],
                        load_cap=mc['LoadClamp'] * G,
                        scale=[r['FrictionScale'] for r in rows],
                        shape=[[r['v%d' % i] * width_scale for i in range(r['NumCurveValues'])] for r in rows])
    # the heavy-load peak slip angle stays within 4..18 deg: both directions rescale
    lat = out['lateral']
    shape = lat['shape'][1]
    peak = max(range(len(shape)), key=lambda i: shape[i] * lat['scale'][1]) * lat['max_slip'] / (len(shape) - 1)
    lo, hi = HEAVY_PEAK_SA
    if peak > hi or peak < lo:
        f = (hi if peak > hi else lo) / peak
        for k in out:
            out[k]['max_slip'] *= f
    return out


def peak_at(c, load):
    """GetPeakFrictionAtLoad of a Curves dict: the peak friction lerped between the two loads."""
    peaks = [max(v * c['scale'][k] for v in c['shape'][k]) for k in (0, 1)]
    f = min(1, max(0, (load - c['loads'][0]) / (c['loads'][1] - c['loads'][0])))
    return peaks[0] + (peaks[1] - peaks[0]) * f


def zero_throttle(w, step, n, braking, low, redline, max_power_speed, max_speed):
    """CCarEngine::CalcZeroThrottleTorque, N m (fh/drivetrain.zig zeroThrottle)."""
    shape = [0.1, 0.09, 0.08, 0.07, 0.06, 0.05, 0.0385, 0.0192, 0]
    wc = min(max(w, 0), max_speed)
    x = min(max(wc / step, 0), n - 1)
    if x < 8:
        i = int(x)
        return (shape[i] + (shape[i + 1] - shape[i]) * (x - i)) * low * braking
    if wc < max_power_speed and wc < redline:
        s = -(wc - 83.77576) / (max_power_speed - 83.77576)
    elif wc < redline:
        s = -1
    elif redline < max_speed:
        s = (wc - redline) / (max_speed - redline) * -1.63 - 1
    else:
        s = -2.63
    return s * braking


def car(d, car_id):
    row = d.one('select * from Data_Car where Id=? or MediaName=?', car_id, car_id)
    if not row:
        sys.exit('no car %s' % car_id)
    P = stock_parts(d, row['Id'])
    parts = [p for p in P.values() if p]
    drive = d.one('select DriveType from Data_Drivetrain join List_DriveType on Data_Drivetrain.DriveTypeID=List_DriveType.ID '
                  'where DrivetrainID=?', P['Drivetrain']['DrivetrainID'])['DriveType']

    # mass, centre of mass, inertia of a uniform box
    w = P['Weight']
    front_weight = min(0.99, max(0.01, w['CMBackFront'] + sum(p.get('WeightDistDiff') or 0 for p in parts)))
    mass = w['Mass'] + sum(p.get('MassDiff') or 0 for p in parts)
    dims = (w['BlockDimX'], w['BlockDimY'], w['BlockDimZ'])
    inertia = [mass / 12 * (a * a + b * b) for a, b in ((dims[1], dims[2]), (dims[0], dims[2]), (dims[0], dims[1]))]
    cm_height = w['CMHeight'] - ((row['FrontStockRideHeight'] - row['RearStockRideHeight']) * front_weight + row['RearStockRideHeight'])
    body = d.one('select * from Data_CarBody where Id=?', P['CarBody']['CarBodyID'])
    wheelbase, tracks = body['ModelWheelbase'], (body['ModelFrontTrackOuter'], body['ModelRearTrackOuter'])
    cg_x = tracks[0] * (w['CMLeftRight'] - 0.5)
    cg_z = (front_weight - 0.5) * wheelbase

    # tyres
    comp = d.one('select * from List_TireCompound where TireCompoundID=?', P['TireCompound']['TireCompoundID'])
    sizes = [tyre_size(row['%sTireWidthMM' % a], row['%sTireAspect' % a], row['%sWheelDiameterIN' % a],
                       P['TireWidth' + a]['%sTireWidth' % a], P['RimSize' + a]['%sWheelDiameter' % a]) for a in ('Front', 'Rear')]
    pressure = tuned(30, *TUNE['pressure'])
    tyres = [curves(d, comp, s, pressure) for s in sizes]

    # unsprung masses: a quarter of the wheel and half the rim and tyre upgrades of the axle
    wheel_mass = d.one('select Mass from List_Wheels where Id=?', row['StockWheelID'])['Mass']
    corner_mass = [0.25 * wheel_mass + 0.5 * ((P['RimSize' + a] or {}).get('MassDiff', 0) + P['TireWidth' + a]['MassDiff']) for a in ('Front', 'Rear')]

    # suspension, front then rear
    susp = [d.one('select * from List_SpringDamperPhysics where SpringDamperPhysicsID=?', P['SpringDamper']['%sSpringDamperPhysicsID' % a])
            for a in ('Front', 'Rear')]
    sway = [d.one('select * from List_AntiSwayPhysics where AntiSwayPhysicsID=?', P['AntiSway' + a]['AntiSwayPhysicsID']) for a in ('Front', 'Rear')]
    stiff = P['ChassisStiffness'] or {}
    h = 1.0 / UPDATE_HZ
    axles = []
    for a, s in enumerate(susp):
        r = sizes[a]['radius']
        k = tuned(s['DefSpringRate'], s['MinSpringRate'], s['MaxSpringRate']) * mass
        bump = tuned(s['DefDampenBumpRate'], s['MinDampenBumpRate'], s['MaxDampenBumpRate']) * mass
        rebound = tuned(s['DefDampenReboundRate'], s['MinDampenReboundRate'], s['MaxDampenReboundRate']) * mass
        ride = tuned(s['DefRideHeight'], s['MinRideHeight'], s['MaxRideHeight'])
        mw = corner_mass[a]
        unsprung = max(mw * UNSPRUNG['scale'] + UNSPRUNG['add'], UNSPRUNG['floor'])
        unsprung = max(unsprung, UNSPRUNG['clamp'] * (k * h * h * 0.25 + max(bump, rebound) * h * 0.5))
        side = 'Front' if a == 0 else 'Rear'
        axles.append(dict(
            k=k, bump=bump, rebound=rebound, caps=[s['DampenBumpClamp'] * mass, s['DampenReboundClamp'] * mass],
            stop=dict(rate=s['BumpstopStiffness'] * mass, damping=s['BumpstopDamping'] * mass),
            y=r - (ride + cm_height), z=(wheelbase * 0.5 if a == 0 else -wheelbase * 0.5) - cg_z,
            compress=r - (s['MaxCompressHeight'] + cm_height), stretch_delta=s['MaxStretchDeltaFromRideHeight'],
            x=tracks[a] * 0.5 - sizes[a]['width'] * 0.5, radius=r, width=sizes[a]['width'],
            inertia=max(WHEEL_INERTIA['scale'] * 0.5 * mw * r * r + WHEEL_INERTIA['add'], WHEEL_INERTIA['floor']),
            unsprung=unsprung, lat_grip=stiff.get(side + 'LatFrictionScale', 1), long_grip=stiff.get(side + 'LongFrictionScale', 1),
            sway=tuned(sway[a]['DefSwaybarStiffness'], sway[a]['MinSwaybarStiffness'], sway[a]['MaxSwaybarStiffness']) * mass,
            sway_damping=sway[a]['SwaybarDamping'] * mass))
    # spring free positions from the force and pitch balance at ride height
    f, r_ = axles
    a_ = mass * G * 0.5 - r_['k'] * r_['y'] - f['k'] * f['y']
    b_ = r_['y'] * (-r_['z'] * r_['k']) - f['y'] * (f['z'] * f['k'])
    c_ = f['k'] * (-r_['z'] * r_['k']) + r_['k'] * (f['z'] * f['k'])
    f['free'] = -(a_ * (-r_['z'] * r_['k']) + b_ * r_['k']) / c_
    r_['free'] = (b_ * f['k'] - a_ * f['z'] * f['k']) / c_
    for ax in axles:
        dd = ax['y'] - ax['free']
        stretch = ax['free'] if dd <= 0.07 else ax['y'] - max(0.07, 0.75 * dd)
        ax['stretch'] = max(stretch, ax['y'] - ax['stretch_delta'])

    # brakes: the pressure that gives the car's own peak braking decel, split by the tyres' peak forces
    br = P['Brakes']
    static = [mass * G * front_weight / 2, mass * G * (1 - front_weight) / 2]
    brake_grip = comp['TorqueFreeLongFrictionScaleBrake'] * br['GameFrictionScaleBraking']
    peak_f = [peak_at(tyres[a]['longitudinal'], static[a]) * static[a] * 2 * brake_grip for a in (0, 1)]
    decel = max(KG, sum(peak_f) / mass)
    front_share = min(0.85, max(0.15, peak_f[0] / sum(peak_f)))
    bias = min(1, max(0, br['BrakeBiasSlider'] - BRAKE_BIAS_REAR_OFFSET * 0.5)) * 2
    t = br['BrakeTorqueSlider']
    pres = 1.81818 * t * 2 if t <= 0.5 else 1.81818 + (4.34783 - 1.81818) * (t - 0.5) * 2
    pressure_by_axle = [front_share * decel / G * 2 * pres * bias, (1 - front_share) * decel / G * 2 * pres * (2 - bias)]
    handbrake = tuned(br['BiasHandbrake'], 0, 5.5) * decel / G

    # engine
    eng = d.one('select * from Data_Engine where EngineID=?', P['Engine']['EngineID'])
    cam = P['Camshaft']
    tc = d.one('select * from List_TorqueCurve where TorqueCurveID=?', cam['TorqueCurveFullThrottleID'])
    ts = lambda p: (P[p] or {}).get('TorqueScale', 1) - 1
    total = 1 + ts('Valves') + ts('Displacement') + ts('PistonsCompression') + ts('Intake') + ts('Exhaust') \
        + ts('FuelSystem') + ts('Ignition') + ts('Manifold') + ts('RestrictorPlate') + ts('OilCooling')
    n = cam['NumRPMEntriesArray']
    max_speed = cam['TorqueCurveMaxRPM'] * RPM
    step = max_speed / (n - 1)
    table = [tc['v%d' % i] * tc['TorqueScale'] for i in range(n)]
    table = [v * total if v > 0 else v for v in table]
    redline = cam['RedlineRPM'] * RPM
    max_power_speed = max(range(n), key=lambda i: table[i] * i) * step
    braking = tc['ZeroThrottleTorqueScale']
    idle = next((i * step for i in range(n) if i * step >= 41.89 and
                 zero_throttle(i * step, step, n, braking, total, redline, max_power_speed, max_speed) <= 0), redline * 0.15)

    # drivetrain
    tm, cl, dl, df = P['Transmission'], P['Clutch'], P['Driveline'], P['Differential']
    final = tuned(tm['FinalDriveRatio'], *TUNE['final_drive'])
    gears = [tm['GearRatio0']] + [tuned(tm['GearRatio%d' % i], *TUNE['gear']) for i in range(1, tm['NumGears'])]
    ratios = [0.0] * 11
    for i, g in enumerate(gears[1:], 1):
        ratios[i] = 1 / (g * final)
    ratios[10] = 1 / (gears[0] * final)
    split = df['RearToqueSplit'] if drive == 'AWD' else (1.0 if drive == 'RWD' else 0.0)

    def lsd(k, ratio):
        return dict(ratio=ratio, accel_lock=tuned(df[k + 'LimitedSlipTorqueAccel'], *TUNE['lsd']),
                    decel_lock=tuned(df[k + 'LimitedSlipTorqueDecel'], *TUNE['lsd_decel']),
                    full_lock_torque=df[k + 'LimitedSlipAccelDefInputTorque'], full_lock_slip=df[k + 'LimitedSlipRelVelClamp'] * RPM)
    if drive == 'AWD':
        rf, rr = sizes[0]['radius'], sizes[1]['radius']
        den = rf * split + rr * (1 - split)
        front_ratio, rear_ratio = 2 * (1 - split) * rr * final / den, 2 * split * rf * final / den
        centre = lsd('Center', 1.0)
        centre['accel_lock'] = df['CenterLimitedSlipTorqueAccel']
        centre['decel_lock'] = df['CenterLimitedSlipTorqueDecel']
    else:
        front_ratio = rear_ratio = final
        centre = dict(ratio=1.0, accel_lock=0.0, decel_lock=0.0, full_lock_torque=1.0, full_lock_slip=1.0)

    # aero: body, bumper parts, wings
    rb = P['RearBumper'] or {}
    drag_scale = math.prod(p.get('DragScale') or 1 for p in parts if 'DragScale' in p) * min(1.5, max(0.5, row['GameDragScale']))

    def wing(part, other):
        a = d.one('select * from List_AeroPhysics where AeroPhysicsID=?', (part or {}).get('AeroPhysicsID'))
        if not a:
            return None
        s = a['DefaultTuneSlider']
        key = 'DFTorqueScaleTwo' if other else 'DFTorqueScaleOne'
        xs = [a['%sSliderInput%d' % (key, i)] for i in range(3)]
        ys = [a['%sTorqueScaleOutput%d' % (key, i)] for i in range(3)]
        share = ys[0] if s <= xs[0] else ys[2] if s >= xs[2] else (lerpc(s, xs[0], xs[1], ys[0], ys[1]) if s < xs[1] else lerpc(s, xs[1], xs[2], ys[1], ys[2]))
        return dict(drag=(a['Drag0'] + (a['Drag1'] - a['Drag0']) * s) * AERO, side_drag=a['LateralDrag'] * AERO,
                    downforce=(a['Downforce0'] + (a['Downforce1'] - a['Downforce0']) * s) * AERO,
                    zero_downforce_cos=math.cos(a['AngleZeroDownforce'] * DEG), torque_share=share)
    front_aero = d.one('select * from List_AeroPhysics where AeroPhysicsID=?', (P['FrontBumper'] or {}).get('AeroPhysicsID'))
    rear_aero = d.one('select * from List_AeroPhysics where AeroPhysicsID=?', (P['RearWing'] or {}).get('AeroPhysicsID'))
    df_body = [row['BodyAeroForwardDownforceRear'] + (rb.get('BodyAeroForwardDownforceRear') or 0),
               row['BodyAeroForwardDownforceFront'] + (rb.get('BodyAeroForwardDownforceFront') or 0)]
    aero = dict(forward_drag=row['BodyAeroLongitudinalDrag'] * AERO * drag_scale, vertical_drag=row['BodyAeroVerticalDrag'] * AERO,
                side_drag=[row['BodyAeroLateralDragRear'] * AERO, row['BodyAeroLateralDragFront'] * AERO],
                downforce=[v * AERO for v in df_body], zero_downforce_cos=math.cos(row['BodyAeroAngleZeroDownforce'] * DEG),
                wings=[wing(P['RearWing'], front_aero), wing(P['FrontBumper'], rear_aero)])

    # the compound's torque-free scales; braking also takes the brakes' game friction scale
    free_grip = dict(lateral=comp['TorqueFreeLatFrictionScale'],
                     braking=comp['TorqueFreeLongFrictionScaleBrake'] * br['GameFrictionScaleBraking'],
                     driving=[comp['TorqueFreeLongFrictionScaleAccel0'], comp['TorqueFreeLongFrictionScaleAccel1']],
                     driving_speeds=[comp['TorqueFreeLongFrictionScaleAccelSpeed0'], comp['TorqueFreeLongFrictionScaleAccelSpeed1']])

    # wheels in the sim order RL RR FL FR. The sim's body x points to the car's left, the game's to its right
    wheels = []
    cg_x = -cg_x
    for a, sx in ((1, 1), (1, -1), (0, 1), (0, -1)):
        ax = axles[a]
        driven = drive == 'AWD' or (drive == 'RWD') == (a == 1)
        share = (split if a == 1 else 1 - split) * 0.5 if driven else 0.0
        tyre = tyres[a]
        wheels.append({
            'rest_position': [sx * ax['x'] + cg_x, ax['y'], ax['z']],
            'spring_axis': [0, 1, 0],
            'radius': ax['radius'], 'width': ax['width'], 'inertia': ax['inertia'],
            'min_height': ax['stretch'], 'max_height': ax['compress'], 'bump_stop_height': ax['compress'],
            'spring': {'linear': dict(rate=ax['k'], preload=ax['free'])},
            'slow_bump': ax['bump'], 'slow_rebound': ax['rebound'],
            'anti_roll': ax['sway'], 'anti_roll_damping': ax['sway_damping'],
            'damper_caps': ax['caps'], 'bump_stop': ax['stop'], 'unsprung_mass': ax['unsprung'], 'tyre_depth': TYRE_GIVE,
            'tyre': {'curve_table': dict(lateral=tyre['lateral'], longitudinal=tyre['longitudinal'],
                                         lateral_grip=ax['lat_grip'], longitudinal_grip=ax['long_grip'], free_grip=free_grip)},
            'max_brake_torque': mass * G / 4 * ax['radius'] * pressure_by_axle[a],
            'handbrake': a == 1, 'drive_proportion': share,
        })
    deg_rate = 2 * DEG  # the stock steering speed tune doubles the rates
    rev_limit = redline + 0.5 * (max_speed - redline)  # the rev limit is half way from the redline to the end
    # [simplified] the hull: the mass block of the body, from the floor up, its faces in by the margin less the track's
    inset = HULL_MARGIN - 0.02
    hx, hz = dims[0] / 2 - inset, dims[2] / 2 - inset
    y0, y1 = -cm_height + inset, dims[1] - cm_height - inset
    box = [[x + cg_x, y, z - cg_z] for x in (-hx, hx) for y in (y0, y1) for z in (-hz, hz)]
    return row, {
        'mass': mass, 'inertia': inertia, 'front_weight': front_weight, 'wheels': wheels,
        'hull': dict(pieces=[box], margin=HULL_MARGIN, friction=0.4, softness=48300.0),
        'aero': {'axles': aero},
        'engine': dict(inertia=eng['MomentInertia'] + P['Flywheel']['MomentInertia'], idle=idle, rev_limit=rev_limit,
                       curve={'sampled': dict(step=step, table=table, braking=braking, low_speed_braking=total,
                                              stall=max(min(cam['StallRPM'] * RPM, idle - step), 36.65),
                                              redline=f32_redline(rev_limit, step, n))}),
        'gearbox': dict(ratios=ratios, top_gear=len(gears) - 1, clutch_torque=cl['ClutchMaxTorque'],
                        shift_time=[tm['GearShiftTime'], tm['GearShiftTime'] * 0.75],
                        clutch_speeds=[1 / max(cl['ClutchInTime'], 0.001), 1 / max(cl['ClutchOutTime'], 0.001)]),
        'driveline': dict(gearbox_inertia=tm['MomentInertia'], shaft_inertia=dl['MomentInertia'],
                          torque_scale=min(1.5, max(0.5, row['GameTorqueScale']))),
        'diffs': {'limited_slip': dict(final_drive=final, rear=lsd('Rear', rear_ratio), front=lsd('Front', front_ratio), centre=centre)},
        'launch': dict(enabled=True, optimum_rpm=cam['StartRPM']),
        'handbrake_strength': handbrake / pressure_by_axle[1],
        'rear_slide_grip': dict(min_scale=row['FixListingRearFricScale'], slip=[row['FixListingNormSlip0'], row['FixListingNormSlip1']],
                                steer=[row['FixListingSteerAngle0'] * DEG, row['FixListingSteerAngle1'] * DEG]),
        'track': tracks[0], 'wheelbase': wheelbase,
        'steer_lock': row['SteerMaxAngle'] * DEG,
        'speed_steering': dict(max_gees=row['SteerSpeedSensitiveMaxGees'], min_lock=row['SteerSpeedSensitiveMinMaxAngle'] * DEG,
                               turn_rate=row['SteerMaxAngVelTurning'] * deg_rate, return_rate=row['SteerMaxAngVelStraighten'] * deg_rate,
                               find_peak_rate=row['SteerAngVelDynFindPeak'] * deg_rate, rate_ramp=row['SteerAccelTimeToMaxRate'],
                               slow_speed=row['SteerSpeedSensitiveSlowSpeed'] * 0.44704, fast_speed=row['SteerSpeedSensitiveFastSpeed'] * 0.44704,
                               fast_rate_scale=row['SteerSpeedSensitiveFastRateScale']),
    }


# FH1 has no snow or ice: SlowGrass (friction 0.3) stands in for both [guess]
BY_FAMILY = {
    'GRS': 'Grass', 'GRA': 'Grass', 'GRSH': 'Grass', 'VEG': 'Grass', 'LEA': 'Leaf1',
    'SND': 'Sand', 'SAN': 'Sand', 'SDT': 'Sand', 'SDW': 'Sand', 'SDC': 'Sand',
    'SNO': 'SlowGrass', 'SNB': 'SlowGrass', 'SNW': 'SlowGrass', 'ICS': 'SlowGrass', 'ICR': 'SlowGrass', 'ICE': 'SlowGrass',
    'MET': 'Asphalt', 'RMB': 'RumbleStrip', 'COB': 'CobblestoneSmall', 'CON': 'Concrete',
}
BY_LETTER = {'T': 'Asphalt', 'C': 'Concrete', 'G': 'Gravel', 'D': 'Dirt', 'M': 'Dirt', 'S': 'Dirt', 'I': 'SlowGrass', 'R': 'Gravel'}


def surface_for(code):
    return BY_FAMILY.get(code) or BY_FAMILY.get(code[:3]) or BY_LETTER.get(code[0], 'Asphalt')


def curve_grip(xml, name):
    node = xml.find('SurfaceTypes/' + name)
    f = node.find('Friction')
    v = lambda key, default: float(f.find(key).get('value')) if f is not None and f.find(key) is not None else default
    return dict(friction=v('FrictionScale', 0.85), off_road=v('OffRoadness', 0), peak_slip_angle=v('OffRoadDryPeakSA', 10) * math.pi / 180,
                rear=v('RearGripMultiplier', 1), handbrake=v('HandbrakeGripMultiplier', 1), min_arcade=v('MinimumArcadeGripValue', 0))


def d3_codes(path):
    data = open(path, 'rb').read()
    return set(m.decode() for m in re.findall(rb'MATERIAL\x00name\x00([A-Z0-9][A-Z0-9*+!_]{3})\x00', data)) or \
        set(m.decode() for m in re.findall(rb'(?<![A-Za-z0-9])([A-Z][A-Z0-9]{2}[A-Z0-9*+!_])(?![A-Za-z0-9])', data))


def dr1_codes(path):
    names = re.findall(r'\.name = \.\{(.*?)\}', open(path).read(), re.S)
    char = lambda t: chr(int(t)) if t[0].isdigit() else t[1]
    return set(''.join(char(t) for t in re.findall(r"\d+|'[^']'", n)).rstrip('\0')[4:] for n in names)


def surfaces(physics_zip, d3_surfaces, dr1_materials):
    with tempfile.TemporaryDirectory() as tmp:
        subprocess.run([sys.executable, str(ROOT / "tools/fh/fhzip.py"), "extract", str(physics_zip), tmp, "surfaceTypes.xml"],
                       check=True, stdout=subprocess.DEVNULL)
        xml = ET.fromstring(Path(tmp, "surfaceTypes.xml").read_text("utf-8", "replace").replace("\r", ""))
    codes = sorted(d3_codes(d3_surfaces) | dr1_codes(dr1_materials) | {"DEFA"})
    return "// FH1 surfaces by D3 and DR1 code (import/fh1.py)\n.{\n" + "\n".join(material(c, curve_grip(xml, surface_for(c))) for c in codes) + "\n}\n"


def material(code, grip):
    name = ", ".join("'%s'" % ch for ch in "fh-" + code) + ", 0" * (8 - 3 - len(code))
    fields = ", ".join(f".{k} = {v!r}" for k, v in grip.items())
    return (f"    .{{ .name = .{{ {name} }}, .depth = 0, .bumps = .{{ .by_distance = .{{ .wavelength = 1, .magnitude = 0 }} }},"
            f" .slowdown = .{{ .full_speed = 1 }}, .curve_grip = .{{ {fields} }} }},")


def zon(v, ind=0):
    pad = '    ' * (ind + 1)
    if v is None:
        return 'null'
    if isinstance(v, bool):
        return 'true' if v else 'false'
    if isinstance(v, (int, float)):
        return repr(float(v)) if isinstance(v, float) else str(v)
    if isinstance(v, dict):
        return '.{\n' + ''.join(f'{pad}.{k} = {zon(x, ind + 1)},\n' for k, x in v.items()) + '    ' * ind + '}'
    if all(isinstance(x, (int, float)) for x in v):
        return '.{ ' + ', '.join(zon(x) for x in v) + ' }'
    return '.{\n' + ''.join(f'{pad}{zon(x, ind + 1)},\n' for x in v) + '    ' * ind + '}'


def short_name(media):
    """MAZ_Miata_94 -> miata"""
    parts = media.split("_")
    return (parts[1] if len(parts) > 2 else media).lower()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("cars", nargs="*")
    ap.add_argument("--db", default=os.environ.get("FH1_DB", FH1 + "/db/gamedb.slt"))
    ap.add_argument("--physics", default=os.environ.get("FH1_PHYSICS", FH1 + "/physics.zip"))
    ap.add_argument("--d3-surfaces", default=str(D3_SURFACES), help="the D3 surface_materials.xml (its codes)")
    ap.add_argument("--dr1-materials", default=str(DR1_SURFACES), help="a DR1 materials.zon (its codes)")
    ap.add_argument("--out", default=str(ROOT / "runs"))
    ap.add_argument("--name", action="append", default=[], help="the folder name per car (fh-<name>), in order")
    ap.add_argument("--list", action="store_true")
    ap.add_argument("--no-surfaces", action="store_true")
    a = ap.parse_args()
    d = Db(a.db)
    if a.list:
        for r in d.c.execute("select Id, MediaName from Data_Car order by Id"):
            print(r["Id"], r["MediaName"])
        return
    out = Path(a.out)
    for k, key in enumerate(a.cars):
        row, c = car(d, int(key) if key.isdigit() else key)
        path = out / ("fh-" + (a.name[k] if k < len(a.name) else short_name(row["MediaName"]))) / "car.zon"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("// %s (Data_Car %d), stock, from gamedb.slt by import/fh1.py\n%s\n" % (row["MediaName"], row["Id"], zon(c)))
        print(path)
    if not a.no_surfaces:
        path = out / "fh" / "materials.zon"
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(surfaces(a.physics, a.d3_surfaces, a.dr1_materials))
        print(path)


if __name__ == "__main__":
    main()
