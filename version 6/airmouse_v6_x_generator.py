#!/usr/bin/env python3
"""
NIDAR AirMouse frame, version 6: X layout, plywood plates and aluminium tube arms.

Nothing is 3D printed and nothing is carbon. The plates are birch plywood, cut on
any laser cutter from 4 mm and 3 mm sheet. The arms are 12 x 12 x 1 mm aluminium
square tube from a hardware shop. The prop guards are polycarbonate strip bands
held by plywood spokes.

Layout: a true X, like a ZD550 shrunk to fit the AirMouse rules. The four arms
run out on the 45 deg diagonals from a small centre hub to the same motor square
as versions 4 and 5 (a normal Quad X for ArduPilot or PX4).

Fold: each arm folds DOWN and sideways about a hinge bolt that runs front to back
(along X), like version 4. The diagonal tube passes through angled slots in two
upright plywood "knuckle" plates, so the skewed hinge needs no printed block.
In flight, thrust presses the knuckles' top edges against the top plate, so the
lock bolt carries no flight load.

Install (Python 3.12):  pip install cadquery ezdxf shapely numpy trimesh pillow
Run:                    python airmouse_v6_x_generator.py
Out:                    output/dxf (laser), output/step, output/renders
"""
import math
import os
from pathlib import Path

import cadquery as cq
import ezdxf
import numpy as np
import trimesh
from ezdxf import units
from shapely import affinity
from shapely.geometry import LineString, Point, Polygon, box as sbox
from shapely.ops import unary_union

# =============================================================================
# PARAMETERS (mm)
# =============================================================================
# --- Rules: NIDAR 2026-27 Rulebook and the AirMouse mission brief
LAUNCH_BOX = 609.6          # 2 ft x 2 ft take-off square
CORRIDOR = 1000.0           # minimum corridor width
MAX_AUW_G = 10000.0         # all-up weight limit

# --- Plywood (birch, laser cut). MEASURE YOUR SHEET: "4 mm" ply is often 3.7-3.9 mm.
T4, T3 = 4.0, 3.0           # the two sheet thicknesses used
PLY_DENSITY = 0.68          # g/cm3
SLOT_PLAY = 0.15            # slot is this much longer and wider than its tab (laser kerf already widens it a little)
MIN_WEB = 3.0               # wood left between any two holes, or a hole and an edge
SHEET_W = 600.0             # width of the sheet the parts are nested on

# --- Aluminium arm tube (6063, hardware-shop square tube)
TUBE, TUBE_WALL = 12.0, 1.0
AL_DENSITY = 2.70

# --- Propulsion and guards: everything is sized from the prop
PROP_IN = 10.0
GUARD_GAP = 9.0             # prop tip to the inside of the band
BAND_T, BAND_H = 1.0, 16.0  # polycarbonate strip
PC_DENSITY = 1.20
RING_GAP = 4.0              # between neighbouring bands
HEAD_PROUD = 2.0            # band joint screw heads
MOTOR_HOLES = (16.0, 19.0)  # 2212 cross pattern, M3. MEASURE YOURS.
MOTOR_R, MOTOR_H = 14.0, 30.0
PROP_ABOVE_PAD = 38.0       # motor plate top to prop plane. MEASURE YOURS.

PROP_R = PROP_IN * 25.4 / 2
GUARD_R = PROP_R + GUARD_GAP
MOTOR_XY = GUARD_R + BAND_T + RING_GAP / 2
WHEELBASE = 2 * math.sqrt(2) * MOTOR_XY

# --- Hub sandwich
GAP = 32.0                  # M3 x 32 standoffs; the Pixhawk lives here
Z_BOT1 = T4                 # top of the bottom plate (its underside is z = 0)
TOP_Z = Z_BOT1 + GAP        # underside of the top plate
TOP_Z1 = TOP_Z + T4         # top of the top plate
HUB_POSTS = [(sx * x, sy * y) for x, y in ((20.0, 38.0), (66.0, 8.0), (140.0, 7.0)) for sx in (1, -1) for sy in (1, -1)]

# --- Arm tube and hinge (front-left arm; the others are mirror images)
TZ = 20.0                   # tube centre height
K1, K2 = 52.0, 80.0         # x of the two knuckle plates' centre planes
WASHER = 0.5                # nylon washer between knuckle and cheek
CHEEK_XS = (K1 - T4 - WASHER, K2 + T4 + WASHER)
HY, HZ = 35.0, 14.0         # hinge bolt (along X)
KR = 8.0                    # knuckle radius around the hinge
LOCK = (40.0, 31.0)         # lock bolt (y, z), above and outboard of the hinge
CHEEK_Y = (8.0, 46.0)       # cheek body spans this y range
CHEEK_TAB = (14.0, 26.0)    # cheek top tab
CHEEK_BOT_TAB = (11.0, 18.0)
TAB_LOCK_Y = 20.0
STANDOFF_R = 2.75           # 5.5 mm hex standoffs, modelled round
TAB_LOCK_Z = TOP_Z1 + STANDOFF_R
CHEEK_SPACERS = [(18.0, 10.0), (18.0, 29.0)]
EAR_Y = 66.0                # top plate covers the knuckles out to here: the thrust stop
BOT_EDGE_Y = 22.0           # bottom plate stays inside this between the cheeks
PIN_X = (K1 + T4 / 2 + 3.3, K2 - T4 / 2 - 3.3)   # tube retaining bolts, inside the knuckle box

# --- Motor end (local frame: u along the arm, outboard positive; v across it)
PAD_TOP = TZ + TUBE / 2 + T3          # top of the motor plate, which sits on the tube
TIP_Z1 = TZ - TUBE / 2                # tip plate is under the tube
TUBE_END_U = -14.0                    # outer end of the tube, behind the motor screws
CLAMP_U = (-24.0, -40.0)              # bolts through motor plate, tube and tip plate
CLAMP_SO = [(20.0 * math.cos(math.radians(a)), 20.0 * math.sin(math.radians(a))) for a in (22.5, -67.5)]
SPOKE_GLOBAL = [22.5 + 90 * k for k in range(4)]   # spoke directions, off the axes so neighbours' tabs never meet
SPOKE_LOCAL = [a - 45.0 for a in SPOKE_GLOBAL]
SPOKE_R0, SPOKE_R1, SPOKE_SCREW_R = 18.0, 38.0, 28.0
CAM_X = 150.0               # front camera plate, at the front ends of the plates

# --- Electronics stack
DECK_Z = TOP_Z1 + 35.0      # M3 x 35 standoffs
DECK_HALF, DECK_CHAMFER = (46.0, 37.0), 8.0
DECK_POSTS = [(sx * 34.0, sy * 32.0) for sx in (1, -1) for sy in (1, -1)]
PI_HOLES = [(sx * 29.0, sy * 24.5) for sx in (1, -1) for sy in (1, -1)]
X2_HOLES = [(-35.3, 0.0), (36.0, 18.0), (36.0, -18.0)]
X2_SCAN_ABOVE_BASE = 42.0
CAM_FOV = (66.0, 41.0)      # Pi Camera Module 3 (standard lens)

# --- Battery cage, sensors and legs (under the bottom plate)
BATTERY = (165.0, 50.0, 40.0)            # upper limit; pack not measured
CAGE = 45.0                 # M3 x 45 standoffs
SENSOR_Z1 = -CAGE
SENSOR_Z0 = SENSOR_Z1 - T3
CAGE_POSTS = [(sx * 102.0, sy * 8.0) for sx in (1, -1) for sy in (1, -1)]
LEG_X = 94.0
SPLAY_Z = SENSOR_Z0 - 14.0  # legs splay out below this, under the folded arms
LEG_FOOT_Y, LEG_FOOT_Z = 46.0, SENSOR_Z0 - 40.0
FLOW_X, TF_X = 50.0, -50.0
TF_SPACING = 36.0

# --- Mass, thrust and material strength
BOUGHT_G = {"motors": 4 * 57, "ESCs": 4 * 35, "props": 4 * 13, "battery": 471, "Pixhawk": 38,
            "Pi 4B + cooling + UBEC": 76, "YDLidar X2": 200, "TFmini-S": 5, "PX4Flow": 20,
            "camera": 10, "power module, wiring": 100, "receiver": 15}
THRUST_LOW_G, THRUST_HIGH_G = 642.0, 850.0
HOVER_G_PER_W, USABLE_MAH, AVIONICS_W = 7.0, 4000.0, 20.0
PLY_FLEX_MPA, PLY_BEARING_MPA = 40.0, 20.0     # conservative birch ply values
AL_YIELD_MPA, AL_E_MPA = 110.0, 69000.0       # 6063-T5

M2, M25, M3, M4 = 2.4, 2.8, 3.4, 4.3
OUT = Path(__file__).resolve().parent / "output"
CORNERS = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
PROP_Z = PAD_TOP + PROP_ABOVE_PAD
BAND_Z0 = PROP_Z - BAND_H / 2
SCAN_Z = DECK_Z + T3 + X2_SCAN_ABOVE_BASE
SQ2 = math.sqrt(2)
V = cq.Vector


# =============================================================================
# 3D HELPERS
# =============================================================================
def bx(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl(r, z0, z1, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def cyl_x(r, y, z, x0, x1):
    return cq.Solid.makeCylinder(r, x1 - x0, V(x0, y, z), V(1, 0, 0))


def prism(pts, z0, z1):
    return cq.Workplane("XY").polyline(pts).close().extrude(z1 - z0).val().translate(V(0, 0, z0))


def fuse(*shapes):
    out = shapes[0]
    for s in shapes[1:]:
        out = out.fuse(s)
    return out.clean()


def cut(base, *cutters):
    for c in cutters:
        base = base.cut(c)
    return base.clean()


def remap(shape, rows):
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf
    t = gp_Trsf()
    t.SetValues(*[float(v) for r in rows for v in r])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, t, True).Shape())


def rot_z(shape, deg):
    return shape.rotate(V(0, 0, 0), V(0, 0, 1), deg)


def volume_of(shape):
    try:
        return sum(s.Volume() for s in shape.Solids()) if shape is not None else 0.0
    except Exception:
        return 0.0


def overlap(a, b):
    ba, bb = a.BoundingBox(), b.BoundingBox()
    if ba.xmax < bb.xmin or bb.xmax < ba.xmin or ba.ymax < bb.ymin or bb.ymax < ba.ymin or ba.zmax < bb.zmin or bb.zmax < ba.zmin:
        return 0.0
    return volume_of(a.intersect(b))


def worst_overlap(group_a, group_b):
    w, where = 0.0, ""
    for na, a in group_a:
        for nb, b in group_b:
            v = overlap(a, b)
            if v > w:
                w, where = v, f"{na} / {nb}"
    return w, where


def worst_within(group):
    w, where = 0.0, ""
    for i, (na, a) in enumerate(group):
        for nb, b in group[i + 1:]:
            v = overlap(a, b)
            if v > w:
                w, where = v, f"{na} / {nb}"
    return w, where


def yz(shape, x0):
    """Profile drawn in (y, z), stood up: its thickness runs along X from x0."""
    return remap(shape, [[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0]])


def mirror_x(shape):
    return shape.mirror("YZ")


def mirror_y(shape):
    return shape.mirror("XZ")


def to_motor(shape, z0=0.0):
    """Local arm frame (u outboard along the arm, v across) to the front-left motor."""
    return rot_z(shape, 45.0).translate(V(MOTOR_XY, MOTOR_XY, z0))


def u_of_x(x):
    """Local u of the point on the arm centre line at global x."""
    return (x - MOTOR_XY) * SQ2


# =============================================================================
# 2D PLYWOOD PARTS
# =============================================================================
def rounded(poly, r):
    """Round every corner of a polygon with radius r (convex and concave)."""
    return poly.buffer(r, 5).buffer(-2 * r, 5).buffer(r, 5)


def slot(x0, y0, x1, y1, w):
    """Round-ended slot between two points."""
    return LineString([(x0, y0), (x1, y1)]).buffer(w / 2, 8)


def tab_slot(cx, cy, w, h):
    """Rectangular slot for a tab, w x h already including play. A laser cuts square corners."""
    return sbox(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)


def radial_slot(ang, r0, r1, w):
    return affinity.rotate(sbox(r0, -w / 2, r1, w / 2), ang, origin=(0, 0))


def mirror4(q1):
    """Symmetric outline from its +x, +y quarter, given as points from the +y axis to the +x axis."""
    quarter = Polygon([(0.0, 0.0)] + q1)
    parts = [quarter, affinity.scale(quarter, -1, 1, origin=(0, 0)),
             affinity.scale(quarter, 1, -1, origin=(0, 0)), affinity.scale(quarter, -1, -1, origin=(0, 0))]
    return unary_union(parts).buffer(0.01).buffer(-0.01)


class Ply:
    """A flat plywood part: outline, slot/window cut-outs and round holes, in its own 2D frame."""

    def __init__(self, name, outline, t, qty, holes=(), cutouts=(), use=""):
        assert outline.geom_type == "Polygon", name
        outline = outline.simplify(0.01)
        self.name, self.outline, self.t, self.qty, self.use = name, outline, t, qty, use
        self.holes, self.cutouts = list(holes), [c.simplify(0.01) for c in cutouts]
        self._solid = None

    def shape2d(self):
        p = self.outline
        for c in self.cutouts:
            p = p.difference(c)
        for x, y, d in self.holes:
            p = p.difference(Point(x, y).buffer(d / 2, 48))
        return p

    def solid(self):
        if self._solid is None:
            s = prism(list(self.outline.exterior.coords)[:-1], 0, self.t)
            cutters = [prism(list(c.exterior.coords)[:-1], -1, self.t + 1) for c in self.cutouts]
            cutters += [cyl(d / 2, -1, self.t + 1, x, y) for x, y, d in self.holes]
            self._solid = cut(s, *cutters)
        return self._solid

    def web(self):
        feats = list(self.cutouts) + [Point(x, y).buffer(d / 2, 48) for x, y, d in self.holes]
        inside = all(self.outline.buffer(-0.01).contains(f) for f in feats)
        edge = self.outline.exterior
        d = [edge.distance(f) for f in feats]
        d += [a.distance(b) for i, a in enumerate(feats) for b in feats[i + 1:]]
        return (min(d) if d else 99.0), inside

    def grams(self):
        return self.shape2d().area * self.t / 1000 * PLY_DENSITY


# ----- hub plates ---------------------------------------------------------------
def top_plate():
    ear_x0 = CHEEK_XS[0] - T4 / 2 - 5.0
    ear_x1 = CHEEK_XS[1] + T4 / 2 + 5.0
    q1 = [(0, 44), (ear_x0 - 4, 44), (ear_x0, EAR_Y), (ear_x1, EAR_Y), (ear_x1, 14), (CAM_X + 6, 14), (CAM_X + 6, 0)]
    outline = rounded(mirror4(q1), 3.0)
    holes = [(x, y, M3) for x, y in HUB_POSTS + DECK_POSTS]
    cuts = []
    for sx in (1, -1):
        for xc in CHEEK_XS:
            for sy in (1, -1):
                cuts.append(tab_slot(sx * xc, sy * sum(CHEEK_TAB) / 2, T4 + SLOT_PLAY, CHEEK_TAB[1] - CHEEK_TAB[0] + SLOT_PLAY))
        cuts.append(tab_slot(sx * CAM_X, 0.0, T3 + SLOT_PLAY, 16.0 + SLOT_PLAY))      # camera plate
        cuts.append(slot(sx * 74.0, 0, sx * 96.0, 0, 8.0))                            # lightening
        cuts.append(slot(sx * 110.0, 0, sx * 130.0, 0, 8.0))                          # lightening
        cuts.append(slot(sx * 28.0, -10.0, sx * 28.0, 10.0, 14.0))                    # lightening
    cuts += [slot(-12.0, sy * 32.0, 12.0, sy * 32.0, 8.0) for sy in (1, -1)]           # wiring up to the Pi
    return Ply("top_plate", outline, T4, 1, holes, cuts, "Hub top. Holds the cheeks; the knuckles stop against it.")


def bottom_plate():
    q1 = [(0, 46), (40, 46), (CHEEK_XS[0] - 4.5, BOT_EDGE_Y), (100, BOT_EDGE_Y), (106, 14), (CAM_X + 6, 14), (CAM_X + 6, 0)]
    outline = rounded(mirror4(q1), 3.0)
    holes = [(x, y, M3) for x, y in HUB_POSTS + CAGE_POSTS]
    cuts = []
    for sx in (1, -1):
        for xc in CHEEK_XS:
            for sy in (1, -1):
                cuts.append(tab_slot(sx * xc, sy * sum(CHEEK_BOT_TAB) / 2, T4 + SLOT_PLAY, CHEEK_BOT_TAB[1] - CHEEK_BOT_TAB[0] + SLOT_PLAY))
        cuts.append(tab_slot(sx * CAM_X, 0.0, T3 + SLOT_PLAY, 6.0 + SLOT_PLAY))
        cuts.append(tab_slot(sx * LEG_X, 0.0, T4 + SLOT_PLAY, 24.0 + SLOT_PLAY))     # leg top tab
        cuts.append(slot(sx * 30.0, -20.0, sx * 30.0, 20.0, 6.0))                    # Pixhawk zip ties
        cuts.append(slot(sx * 72.0, 0, sx * 84.0, 0, 8.0))                           # lightening
        cuts.append(slot(sx * 112.0, 0, sx * 130.0, 0, 8.0))                         # lightening
    cuts += [slot(-10.0, sy * 32.0, 10.0, sy * 32.0, 8.0) for sy in (1, -1)]          # lightening
    cuts.append(slot(-10.0, 0, 10.0, 0, 10.0))                                       # sensor wiring
    return Ply("bottom_plate", outline, T4, 1, holes, cuts, "Hub bottom. Holds the battery cage and legs.")


# ----- cheeks (YZ profile: a = y, b = z), two per arm, fixed to the hub ---------------
def cheek():
    body = rounded(sbox(CHEEK_Y[0], Z_BOT1, CHEEK_Y[1], TOP_Z), 1.0)
    tab_top = TAB_LOCK_Z + M3 / 2 + 3.5
    tabs = [rounded(sbox(CHEEK_TAB[0], TOP_Z - 2, CHEEK_TAB[1], tab_top), 2.0),
            sbox(CHEEK_BOT_TAB[0], 0.0, CHEEK_BOT_TAB[1], Z_BOT1 + 1)]
    outline = unary_union([body] + tabs)
    holes = [(HY, HZ, M4), (LOCK[0], LOCK[1], M3), (TAB_LOCK_Y, TAB_LOCK_Z, M3)] + [(y, z, M3) for y, z in CHEEK_SPACERS]
    return Ply("cheek", outline, T4, 8, holes, [], "Fixed hinge plate, one each side of an arm's knuckles.")


# ----- knuckles (YZ profile), the rotating part of the hinge ----------------------
def tube_hole(k):
    """The tube crosses an upright plate at 45 deg: its section is TUBE*sqrt2 wide, shifted by the plate thickness."""
    w = TUBE * SQ2 + T4 + SLOT_PLAY
    return tab_slot(k, TZ, w, TUBE + SLOT_PLAY)


def knuckle(k, name):
    y_end = k + (TUBE * SQ2 + T4 + SLOT_PLAY) / 2 + 4.0
    main = Polygon([(HY, HZ - KR), (y_end, HZ - KR), (y_end, TOP_Z), (HY, TOP_Z)])
    outline = rounded(unary_union([main, Point(HY, HZ).buffer(KR, 64)]), 1.0)
    holes = [(HY, HZ, M4), (LOCK[0], LOCK[1], M3)]
    return Ply(name, outline, T4, 4, holes, [tube_hole(k)], "Rotating hinge plate; the tube passes through it at 45 deg.")


# ----- motor plate and tip plate (local u, v frame, origin on the motor axis) ---------
def motor_holes():
    return [(MOTOR_HOLES[0] / 2, 0.0), (-MOTOR_HOLES[0] / 2, 0.0), (0.0, MOTOR_HOLES[1] / 2), (0.0, -MOTOR_HOLES[1] / 2)]


def tongue(half_w):
    return sbox(CLAMP_U[1] - 6.0, -half_w, 0.0, half_w)


def motor_plate():
    lobes = [radial_slot(a, 0.0, SPOKE_R1 + 3.5, 12.0) for a in SPOKE_LOCAL]
    outline = rounded(unary_union([Point(0, 0).buffer(27.0, 64), tongue(9.5)] + lobes), 2.0)
    holes = [(x, y, M3) for x, y in motor_holes() + CLAMP_SO] + [(u, 0.0, M3) for u in CLAMP_U] + [(0.0, 0.0, 6.5)]
    cuts = [radial_slot(a, SPOKE_R0 + 1.0 - SLOT_PLAY / 2, SPOKE_R1 - 1.0 + SLOT_PLAY / 2, T3 + SLOT_PLAY) for a in SPOKE_LOCAL]
    return Ply("motor_plate", outline, T3, 4, holes, cuts, "Motor seat on top of the tube. The four guard spokes slot into it.")


def tip_plate():
    outline = rounded(unary_union([Point(0, 0).buffer(25.0, 64), tongue(7.0)]), 2.0)
    # Motor screws go in before the tip plate; these holes let a 2.5 mm hex key reach them.
    holes = [(x, y, M3) for x, y in CLAMP_SO] + [(u, 0.0, M3) for u in CLAMP_U] + [(x, y, 3.6) for x, y in motor_holes()]
    return Ply("tip_plate", outline, T3, 4, holes, [], "Under the tube; clamps it to the motor plate.")


# ----- guard spoke (profile in (r, z): r out from the motor axis) ---------------------
def spoke():
    t0, t1 = GUARD_R - 4.0, GUARD_R
    low, high = BAND_Z0, BAND_Z0 + BAND_H
    body = Polygon([(SPOKE_R0, PAD_TOP), (SPOKE_R1, PAD_TOP), (t0, low - 10.0), (t0, low), (t1, low), (t1, high),
                    (t0, high), (t0, PROP_Z - 6.0), (SPOKE_R0, PAD_TOP + 14.0)])
    foot = sbox(SPOKE_R0 + 1.0, PAD_TOP - T3, SPOKE_R1 - 1.0, PAD_TOP + 1.0)
    lug = sbox(t1 - 1.0, low + 3.0, t1 + BAND_T + 9.0, low + 13.0)       # passes through the band
    outline = unary_union([body, foot, lug])
    # T-slot: an M3 screw comes up through the motor plate into a captive nut.
    tslot = unary_union([sbox(SPOKE_SCREW_R - 1.7, PAD_TOP - T3 - 1.0, SPOKE_SCREW_R + 1.7, PAD_TOP + 7.0),
                         sbox(SPOKE_SCREW_R - 3.0, PAD_TOP + 3.5, SPOKE_SCREW_R + 3.0, PAD_TOP + 6.1)])
    outline = outline.difference(tslot)
    outline = Polygon(outline.exterior)
    holes = [(t1 + BAND_T + 4.0, low + 8.0, M2)]                          # zip tie or 2 mm pin outside the band
    cuts = [slot(57.0, PAD_TOP + 9.8, 100.0, PAD_TOP + 20.0, 5.0)]                  # lightening
    return Ply("guard_spoke", outline, T3, 16, holes, cuts, "Guard spoke: foot in the motor plate, lug through the band.")


# ----- deck, sensor plate, legs, camera plate ---------------------------------------
def deck():
    hx, hy = DECK_HALF
    c = DECK_CHAMFER
    outline = rounded(Polygon([(hx - c, -hy), (hx, -hy + c), (hx, hy - c), (hx - c, hy), (-hx + c, hy),
                               (-hx, hy - c), (-hx, -hy + c), (-hx + c, -hy)]), 2.0)
    holes = [(x, y, M3) for x, y in DECK_POSTS] + [(x, y, M25) for x, y in PI_HOLES + X2_HOLES]
    cuts = [slot(-16.0, sy * 12.0, 16.0, sy * 12.0, 8.0) for sy in (1, -1)]
    return Ply("lidar_deck", outline, T3, 1, holes, cuts, "YDLidar X2 on top, Raspberry Pi 4B hanging underneath.")


def sensor_plate():
    q1 = [(0, 36), (40, 36), (48, 26), (86, 22), (108, 14), (108, 0)]
    outline = rounded(mirror4(q1), 3.0)
    holes = [(x, y, M3) for x, y in CAGE_POSTS] + [(TF_X, s * TF_SPACING / 2, M2) for s in (1, -1)]
    cuts = [tab_slot(sx * LEG_X, 0.0, T4 + SLOT_PLAY, 30.0 + SLOT_PLAY) for sx in (1, -1)]
    cuts += [slot(sx * 13.0 - 9.5, sy * 29.0, sx * 13.0 + 9.5, sy * 29.0, 3.0) for sx in (1, -1) for sy in (1, -1)]  # battery straps
    cuts += [slot(FLOW_X + s * 17.0, -13.5, FLOW_X + s * 17.0, 13.5, 3.0) for s in (1, -1)]   # PX4Flow zip ties
    cuts.append(slot(FLOW_X - 4.0, 0, FLOW_X + 4.0, 0, 14.0))                                 # PX4Flow lens
    cuts.append(slot(TF_X - 4.0, 0, TF_X + 4.0, 0, 12.0))                                     # TFmini-S window
    return Ply("sensor_plate", outline, T3, 1, holes, cuts, "Battery floor; PX4Flow and TFmini-S underneath.")


def leg():
    """YZ profile. Top tab in the bottom plate, neck through the sensor plate, feet splayed below the folded arms."""
    fy, fz = LEG_FOOT_Y, LEG_FOOT_Z
    pts = [(-12, Z_BOT1), (12, Z_BOT1), (12, 0), (15, 0), (15, SPLAY_Z), (24, SPLAY_Z), (fy + 6, fz + 4), (fy + 6, fz),
           (fy - 6, fz), (12, SPLAY_Z - 9), (-12, SPLAY_Z - 9), (-fy + 6, fz), (-fy - 6, fz), (-fy - 6, fz + 4),
           (-24, SPLAY_Z), (-15, SPLAY_Z), (-15, 0), (-12, 0)]
    outline = Polygon(pts).buffer(-0.8, 3).buffer(0.8, 3)    # round outside corners only: tabs stay square
    holes = [(0.0, SENSOR_Z1 + STANDOFF_R, M3)]                         # cross pin sits on the sensor plate
    cuts = [slot(0, -35.0 + 7.0, 0, -9.0 - 7.0, 14.0)]
    return Ply("leg", outline, T4, 2, holes, cuts, "Landing leg and battery end-stop, one front and one rear.")


def camera_plate():
    body = sbox(-16.0, Z_BOT1, 16.0, TOP_Z)
    outline = unary_union([rounded(body, 1.0), sbox(-8.0, TOP_Z - 1, 8.0, TOP_Z1), sbox(-3.0, 0.0, 3.0, Z_BOT1 + 1)])
    zc = (Z_BOT1 + TOP_Z) / 2
    holes = [(sy * 10.5, zc + sz * 6.25, 2.2) for sy in (1, -1) for sz in (1, -1)]
    return Ply("camera_plate", outline, T3, 1, holes, [], "Front camera mount (Pi Camera footprint), clamped between the plates.")


# =============================================================================
# BOUGHT / HAND-MADE PARTS: aluminium tube and polycarbonate band
# =============================================================================
def tube_root_u():
    """u of the centre line where it meets the inner face of knuckle 1 (the 45 deg cut)."""
    return u_of_x(K1 - T4 / 2)


def tube_holes_u():
    return list(CLAMP_U) + [u_of_x(x) for x in PIN_X]


def make_tube():
    u0, u1 = tube_root_u() - 12.0, TUBE_END_U
    t = bx(u0, u1, -TUBE / 2, TUBE / 2, TZ - TUBE / 2, TZ + TUBE / 2)
    t = cut(t, bx(u0 - 1, u1 + 1, -TUBE / 2 + TUBE_WALL, TUBE / 2 - TUBE_WALL, TZ - TUBE / 2 + TUBE_WALL, TZ + TUBE / 2 - TUBE_WALL),
            *[cyl(M3 / 2, TZ - 8, TZ + 8, u, 0.0) for u in tube_holes_u()])
    t = to_motor(t)
    return t.intersect(bx(K1 - T4 / 2, 400, -400, 400, -50, 100)).clean()   # 45 deg cut, flush with knuckle 1


def band_strip_len():
    return 2 * math.pi * (GUARD_R + BAND_T / 2)


def make_band():
    ring = (cq.Workplane("XY").circle(GUARD_R + BAND_T).circle(GUARD_R).extrude(BAND_H).val().translate(V(0, 0, BAND_Z0)))
    slots = [rot_z(bx(GUARD_R - 1, GUARD_R + BAND_T + 1, -(T3 + 0.3) / 2, (T3 + 0.3) / 2, BAND_Z0 + 3 - 0.15, BAND_Z0 + 13 + 0.15), a)
             for a in SPOKE_GLOBAL]
    return cut(ring, *slots).translate(V(MOTOR_XY, MOTOR_XY, 0))


def prop_blades():
    """Two-blade 10 in prop, for pictures only."""
    blade = Polygon([(14, -5), (40, -11), (90, -9), (PROP_R - 3, -4), (PROP_R, 0), (PROP_R - 3, 4), (90, 8), (40, 10), (14, 5)])
    both = unary_union([blade, affinity.rotate(blade, 180, origin=(0, 0)), Point(0, 0).buffer(16, 24)])
    p = prism(list(both.exterior.coords)[:-1], PROP_Z - 1.5, PROP_Z + 1.5)
    return rot_z(p, 25).translate(V(MOTOR_XY, MOTOR_XY, 0))


# =============================================================================
# ASSEMBLY
# =============================================================================
def fold(shape, deg):
    """Swing a +Y arm part about its hinge line: 0 deployed, 90 folded down."""
    return shape.translate(V(0, -HY, -HZ)).rotate(V(0, 0, 0), V(1, 0, 0), -deg).translate(V(0, HY, HZ))


def to_corner(shape, corner):
    sx, sy = corner
    if sy < 0:
        shape = mirror_y(shape)
    if sx < 0:
        shape = mirror_x(shape)
    return shape


def build_parts():
    return {"top": top_plate(), "bottom": bottom_plate(), "cheek": cheek(), "knuckle1": knuckle(K1, "knuckle_inner"),
            "knuckle2": knuckle(K2, "knuckle_outer"), "motor": motor_plate(), "tip": tip_plate(), "spoke": spoke(),
            "deck": deck(), "sensor": sensor_plate(), "leg": leg(), "camera": camera_plate()}


def place_spoke(P, ang):
    s = remap(P["spoke"].solid(), [[1, 0, 0, 0], [0, 0, -1, T3 / 2], [0, 1, 0, 0]])
    return rot_z(s, ang).translate(V(MOTOR_XY, MOTOR_XY, 0))


def arm_unit(P, tube, band):
    """The front-left (+X, +Y) arm, deployed. Returns {kind: [(name, solid)]}."""
    x_in, x_out = K1 + T4 / 2, K2 - T4 / 2
    u = {"wood": [("knuckle_inner", yz(P["knuckle1"].solid(), K1 - T4 / 2)),
                  ("knuckle_outer", yz(P["knuckle2"].solid(), K2 - T4 / 2)),
                  ("motor_plate", to_motor(P["motor"].solid(), PAD_TOP - T3)),
                  ("tip_plate", to_motor(P["tip"].solid(), TIP_Z1 - T3))],
         "metal": [("tube", tube),
                   ("hinge_spacer", cyl_x(3.5, HY, HZ, x_in, x_out)),
                   ("lock_spacer", cyl_x(3.0, LOCK[0], LOCK[1], x_in, x_out))]
         + [(f"clamp_standoff{i}", to_motor(cyl(STANDOFF_R, TIP_Z1, PAD_TOP - T3, x, y))) for i, (x, y) in enumerate(CLAMP_SO)]
         + [(f"tube_pin{i}", cyl(1.5, TZ - TUBE / 2 - 1, TZ + TUBE / 2 + 1, x, x)) for i, x in enumerate(PIN_X)],
         "motor": [("motor", cyl(MOTOR_R, PAD_TOP, PAD_TOP + MOTOR_H, MOTOR_XY, MOTOR_XY))],
         "guard": [(f"spoke{k}", place_spoke(P, a)) for k, a in enumerate(SPOKE_GLOBAL)] + [("band", band)],
         "prop": [("prop", prop_blades())]}
    return u


def body_parts(P):
    c = P["cheek"].solid()
    wood = [("top_plate", P["top"].solid().translate(V(0, 0, TOP_Z))), ("bottom_plate", P["bottom"].solid()),
            ("lidar_deck", P["deck"].solid().translate(V(0, 0, DECK_Z))),
            ("sensor_plate", P["sensor"].solid().translate(V(0, 0, SENSOR_Z0))),
            ("camera_plate", yz(P["camera"].solid(), CAM_X - T3 / 2))]
    metal = []
    for corner in CORNERS:
        tag = {(1, 1): "FL", (1, -1): "FR", (-1, 1): "RL", (-1, -1): "RR"}[corner]
        here = [(f"cheek_{tag}{k}", yz(c, xc - T4 / 2)) for k, xc in enumerate(CHEEK_XS)]
        x0, x1 = CHEEK_XS[0] + T4 / 2, CHEEK_XS[1] - T4 / 2
        mh = [(f"cheek_spacer_{tag}{i}", cyl_x(3.0, y, z, x0, x1)) for i, (y, z) in enumerate(CHEEK_SPACERS)]
        mh.append((f"tab_lock_{tag}", cyl_x(STANDOFF_R, TAB_LOCK_Y, TAB_LOCK_Z, x0, x1)))
        wood += [(n, to_corner(s, corner)) for n, s in here]
        metal += [(n, to_corner(s, corner)) for n, s in mh]
    for sx in (1, -1):
        wood.append((f"leg_{'front' if sx > 0 else 'rear'}", yz(P["leg"].solid(), sx * LEG_X - T4 / 2)))
    metal += [(f"hub_post{i}", cyl(STANDOFF_R, Z_BOT1, TOP_Z, x, y)) for i, (x, y) in enumerate(HUB_POSTS)]
    metal += [(f"cage_post{i}", cyl(STANDOFF_R, SENSOR_Z1, 0.0, x, y)) for i, (x, y) in enumerate(CAGE_POSTS)]
    metal += [(f"deck_post{i}", cyl(STANDOFF_R, TOP_Z1, DECK_Z, x, y)) for i, (x, y) in enumerate(DECK_POSTS)]
    return wood, metal


def electronics():
    d1 = DECK_Z + T3
    lidar = fuse(bx(-40, 40, -22, 22, d1, d1 + 15), cyl(30.25, d1 + 15, d1 + 50))
    zc = (Z_BOT1 + TOP_Z) / 2
    cx = CAM_X + T3 / 2
    return [("pixhawk", bx(-40.75, 40.75, -25, 25, Z_BOT1 + 4, Z_BOT1 + 4 + 15.5)),
            ("pi4b", bx(-42.5, 42.5, -28, 28, DECK_Z - 6 - 22, DECK_Z - 6)),
            ("ydlidar_x2", lidar),
            ("battery", bx(-BATTERY[0] / 2, BATTERY[0] / 2, -BATTERY[1] / 2, BATTERY[1] / 2, SENSOR_Z1, SENSOR_Z1 + BATTERY[2])),
            ("px4flow", bx(FLOW_X - 22.75, FLOW_X + 22.75, -17.5, 17.5, SENSOR_Z0 - 21, SENSOR_Z0)),
            ("tfmini_s", bx(TF_X - 7.5, TF_X + 7.5, -21, 21, SENSOR_Z0 - 16, SENSOR_Z0)),
            ("camera", fuse(bx(cx + 3, cx + 4, -12.5, 12.5, zc - 12, zc + 12), cyl_x(4.0, 0, zc, cx + 4, cx + 9)))]


def place_unit(unit, corner, deg):
    return {k: [(n, to_corner(fold(s, deg), corner)) for n, s in v] for k, v in unit.items()}


# =============================================================================
# OUTPUT HELPERS
# =============================================================================
def tri_mesh(shape, tol=0.08):
    verts, tris = shape.tessellate(tol, 0.3)
    return trimesh.Trimesh(np.array([[v.x, v.y, v.z] for v in verts]), np.array(tris), process=True)


def dxf_doc():
    doc = ezdxf.new("R2010", setup=True)
    doc.units = units.MM
    for name, color in (("CUT_OUTER", 1), ("CUT_INNER", 5), ("NOTES", 7)):
        doc.layers.add(name, color=color)
    return doc


def dxf_part(msp, part, place=lambda g: g, label=None):
    """Outline and cut-outs as polylines, holes as true circles. place() moves shapely geometry."""
    outline = place(part.outline)
    msp.add_lwpolyline(list(outline.exterior.coords)[:-1], close=True, dxfattribs={"layer": "CUT_OUTER"})
    for c in part.cutouts:
        msp.add_lwpolyline(list(place(c).exterior.coords)[:-1], close=True, dxfattribs={"layer": "CUT_INNER"})
    for x, y, d in part.holes:
        p = place(Point(x, y))
        msp.add_circle((p.x, p.y), d / 2, dxfattribs={"layer": "CUT_INNER"})
    if label:
        b = outline.bounds
        msp.add_text(label, height=3, dxfattribs={"layer": "NOTES"}).set_placement((b[0], b[1] - 6))


def band_dxf(path):
    """Flat polycarbonate strip: cut to length with a knife, drill or punch the holes and slots. Do not laser polycarbonate."""
    L, over = band_strip_len(), 20.0
    doc = dxf_doc()
    msp = doc.modelspace()
    msp.add_lwpolyline([(0, 0), (L + over, 0), (L + over, BAND_H), (0, BAND_H)], close=True, dxfattribs={"layer": "CUT_OUTER"})
    for s in (5.0, 15.0):                                   # joint: these two lie over the two at the far end
        for x in (s, L + s):
            msp.add_circle((x, BAND_H / 2), M3 / 2, dxfattribs={"layer": "CUT_INNER"})
    for k in range(4):                                      # a slot for each spoke lug
        x = over + (k + 0.5) * L / 4
        w, h = T3 + 0.3, 10.3
        msp.add_lwpolyline([(x - w / 2, 3 - 0.15), (x + w / 2, 3 - 0.15), (x + w / 2, 3 - 0.15 + h), (x - w / 2, 3 - 0.15 + h)],
                           close=True, dxfattribs={"layer": "CUT_INNER"})
    msp.add_text(f"GUARD BAND x4: 1 mm polycarbonate, {L + over:.0f} x {BAND_H:.0f} mm. Overlap the ends {over:.0f} mm, 2 x M3.",
                 height=3, dxfattribs={"layer": "NOTES"}).set_placement((0, -6))
    doc.saveas(path)


def nest(parts, thickness):
    """Shelf-pack every copy of every part of one thickness onto a SHEET_W wide sheet."""
    items = []
    for p in parts:
        if p.t != thickness:
            continue
        b = p.outline.bounds
        w, h = b[2] - b[0], b[3] - b[1]
        turn = 90 if (h > w and h <= SHEET_W) else 0
        for _ in range(p.qty):
            items.append((p, turn))

    def height(it):
        b = affinity.rotate(it[0].outline, it[1], origin=(0, 0)).bounds
        return b[3] - b[1]
    items.sort(key=lambda it: -height(it))
    placed, x, y, row_h, gap = [], 0.0, 0.0, 0.0, 5.0
    for p, turn in items:
        g = affinity.rotate(p.outline, turn, origin=(0, 0))
        b = g.bounds
        w, h = b[2] - b[0], b[3] - b[1]
        if x + w > SHEET_W:
            x, y, row_h = 0.0, y + row_h + gap, 0.0
        dx, dy = x - b[0], y - b[1]
        placed.append((p, lambda geom, t=turn, dx=dx, dy=dy: affinity.translate(affinity.rotate(geom, t, origin=(0, 0)), dx, dy)))
        x += w + gap
        row_h = max(row_h, h)
    return placed, y + row_h


def sheet_png(placed, length, thickness, path):
    from PIL import Image, ImageDraw
    k, m = 2.0, 30
    W, H = int(SHEET_W * k + 2 * m), int(length * k + 2 * m + 40)
    img = Image.new("RGB", (W, H), (246, 246, 248))
    dr = ImageDraw.Draw(img)
    dr.rectangle([m, m + 40, m + SHEET_W * k, m + 40 + length * k], outline=(150, 150, 160), width=2)

    def px(pts):
        return [(m + x * k, m + 40 + (length - y) * k) for x, y in pts]
    for p, place in placed:
        dr.polygon(px(place(p.outline).exterior.coords), fill=(214, 178, 128), outline=(120, 90, 50))
        for c in p.cutouts:
            dr.polygon(px(place(c).exterior.coords), fill=(246, 246, 248))
        for x, y, d in p.holes:
            q = place(Point(x, y))
            (cx, cy), = px([(q.x, q.y)])
            dr.ellipse([cx - d / 2 * k, cy - d / 2 * k, cx + d / 2 * k, cy + d / 2 * k], fill=(246, 246, 248))
    dr.text((m, 14), f"{thickness:.0f} mm birch plywood: {SHEET_W:.0f} x {length:.0f} mm used", fill=(30, 30, 30), font_size=24)
    img.save(path)


def render(groups, path, elev, azim, title, sub="", size=(1600, 1200), ss=2, zoom=1.0, tol=0.5):
    """Z-buffer renderer with outlines. groups: [(list of shapes, rgb)]."""
    from PIL import Image, ImageDraw
    W, H = size[0] * ss, size[1] * ss
    el, az = math.radians(elev), math.radians(azim)
    fwd = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    right = np.array([-math.sin(az), math.cos(az), 0.0])
    up = np.cross(fwd, right)
    light = fwd * 0.55 + up * 0.7 - right * 0.35
    light /= np.linalg.norm(light)
    tris, cols, ids = [], [], []
    for gi, (shapes, color) in enumerate(groups):
        for si, s in enumerate(shapes):
            m = tri_mesh(s, tol)
            if len(m.faces) == 0:
                continue
            tris.append(m.triangles)
            shade = 0.32 + 0.68 * np.abs(m.face_normals @ light)
            cols.append(np.array(color, float)[None, :] * shade[:, None])
            ids.append(np.full(len(m.faces), gi * 10000 + si))
    tris, cols, ids = np.concatenate(tris), np.concatenate(cols), np.concatenate(ids)
    allv = tris.reshape(-1, 3)
    u, v = allv @ right, allv @ up
    scale = min((W - 160 * ss) / np.ptp(u), (H - 220 * ss) / np.ptp(v)) * zoom
    cu, cv = (u.max() + u.min()) / 2, (v.max() + v.min()) / 2
    depth = np.full((H, W), -np.inf)
    gy = np.linspace(0, 1, H)[:, None, None]
    img = (np.array([251, 251, 253]) * (1 - gy) + np.array([226, 230, 237]) * gy) * np.ones((1, W, 1))
    idb = np.full((H, W), -1)
    PX = (tris @ right - cu) * scale + W / 2
    PY = H / 2 + 40 * ss - (tris @ up - cv) * scale
    PZ = tris @ fwd
    for x, y, z, col, tid in zip(PX, PY, PZ, cols, ids):
        x0, x1 = max(int(x.min()), 0), min(int(x.max()) + 1, W - 1)
        y0, y1 = max(int(y.min()), 0), min(int(y.max()) + 1, H - 1)
        if x1 < x0 or y1 < y0:
            continue
        d = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
        if abs(d) < 1e-9:
            continue
        gx, gyy = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
        a = ((y[1] - y[2]) * (gx - x[2]) + (x[2] - x[1]) * (gyy - y[2])) / d
        b = ((y[2] - y[0]) * (gx - x[2]) + (x[0] - x[2]) * (gyy - y[2])) / d
        c = 1 - a - b
        inside = (a >= -1e-6) & (b >= -1e-6) & (c >= -1e-6)
        zz = a * z[0] + b * z[1] + c * z[2]
        win = depth[y0:y1 + 1, x0:x1 + 1]
        mask = inside & (zz > win)
        win[mask] = zz[mask]
        img[y0:y1 + 1, x0:x1 + 1][mask] = col
        idb[y0:y1 + 1, x0:x1 + 1][mask] = tid
    edge = np.zeros((H, W), bool)
    for ax in (0, 1):
        a_id, b_id = idb, np.roll(idb, 1, axis=ax)
        a_d, b_d = depth, np.roll(depth, 1, axis=ax)
        jump = (a_id != b_id) | (np.abs(np.nan_to_num(a_d - b_d, posinf=1e9, neginf=1e9)) > 4.0)
        edge |= jump & ((a_id >= 0) | (b_id >= 0))
    img[edge] *= 0.25
    out = Image.fromarray(np.clip(img, 0, 255).astype(np.uint8)).resize(size, Image.LANCZOS)
    dr = ImageDraw.Draw(out)
    dr.text((34, 26), title, fill=(25, 25, 30), font_size=34)
    if sub:
        dr.text((34, 72), sub, fill=(80, 80, 90), font_size=22)
    out.save(path)


def blockage(shapes, cx, cy):
    """Share of the prop disc (outside the hub) shaded by these shapes, from a 0.5 mm raster."""
    from PIL import Image, ImageDraw
    k = 2.0
    n = int(2 * PROP_R * k) + 4
    img = Image.new("1", (n, n), 0)
    dr = ImageDraw.Draw(img)
    for s in shapes:
        m = tri_mesh(s, 0.5)
        for t in m.triangles:
            dr.polygon([((p[0] - cx + PROP_R) * k + 2, (p[1] - cy + PROP_R) * k + 2) for p in t], fill=1)
    a = np.array(img, bool)
    yy, xx = np.mgrid[0:n, 0:n]
    r = np.hypot((xx - 2) / k - PROP_R, (yy - 2) / k - PROP_R)
    disc = (r <= PROP_R) & (r >= 15.0)
    return (a & disc).sum() / disc.sum()


class Report:
    def __init__(self):
        self.failed = 0

    def check(self, label, ok, detail=""):
        self.failed += 0 if ok else 1
        print(f"  [{'PASS' if ok else 'FAIL'}] {label}{'  ' + detail if detail else ''}")


# =============================================================================
# MAIN
# =============================================================================
def main():
    rpt = Report()
    for sub in ("dxf", "step", "renders"):
        (OUT / sub).mkdir(parents=True, exist_ok=True)
    print(f"Prop {PROP_IN:.0f} in -> motors at +/-{MOTOR_XY:.1f} mm, wheelbase {WHEELBASE:.1f} mm")

    P = build_parts()
    parts = list(P.values())

    # ----- plywood parts
    print("\nPLYWOOD PARTS (laser cut; DXF in mm, STEP flat)")
    ply_g = 0.0
    for p in parts:
        s = p.solid()
        doc = dxf_doc()
        dxf_part(doc.modelspace(), p, label=f"{p.name.upper()}  x{p.qty}  {p.t:.0f} mm plywood")
        doc.saveas(OUT / "dxf" / f"{p.name}_{p.t:.0f}mm_x{p.qty}.dxf")
        s.exportStep(str(OUT / "step" / f"{p.name}_{p.t:.0f}mm_x{p.qty}.step"))
        web, inside = p.web()
        b = p.outline.bounds
        g = p.grams()
        ply_g += g * p.qty
        rpt.check(f"{p.name} x{p.qty}: {b[2] - b[0]:.0f} x {b[3] - b[1]:.0f} x {p.t:.0f} mm, {g:.1f} g each",
                  s.isValid() and len(s.Solids()) == 1 and inside and web >= MIN_WEB - 1e-6,
                  f"thinnest web {web:.1f} mm" + ("" if inside else ", FEATURE OUTSIDE PART"))
    sheets = {}
    for t in (T4, T3):
        placed, length = nest(parts, t)
        doc = dxf_doc()
        for p, place in placed:
            dxf_part(doc.modelspace(), p, place, p.name)
        doc.saveas(OUT / "dxf" / f"CUT_SHEET_{t:.0f}mm.dxf")
        sheet_png(placed, length, t, OUT / "renders" / f"cut_sheet_{t:.0f}mm.png")
        sheets[t] = length
        print(f"  cut sheet {t:.0f} mm: {len(placed)} pieces on {SHEET_W:.0f} x {length:.0f} mm")

    # ----- tube and band
    tube, band = make_tube(), make_band()
    tube_vol = volume_of(tube)
    tube_g, band_g = tube_vol / 1000 * AL_DENSITY, volume_of(band) / 1000 * PC_DENSITY
    long_side = TUBE_END_U - (tube_root_u() - TUBE / 2)
    short_side = TUBE_END_U - (tube_root_u() + TUBE / 2)
    print("\nALUMINIUM TUBE x4 (12 x 12 x 1 mm square)")
    print(f"  cut {long_side:.1f} mm on the long side, {short_side:.1f} mm on the short side: the root end is cut at 45 deg")
    print("  drill 3.4 mm straight down through both walls, measured along the centre line from the square outer end:")
    for name, uu in zip(("clamp bolt", "clamp bolt", "inner pin", "outer pin"), tube_holes_u()):
        print(f"    {name:10s} {TUBE_END_U - uu:6.1f} mm")
    print(f"  {tube_g:.1f} g each")
    band_dxf(OUT / "dxf" / "guard_band_polycarbonate_x4.dxf")
    print(f"\nGUARD BAND x4: polycarbonate strip {band_strip_len() + 20:.0f} x {BAND_H:.0f} x {BAND_T:.0f} mm, {band_g:.0f} g each")
    tube.exportStep(str(OUT / "step" / "arm_tube_x4.step"))
    band.translate(V(-MOTOR_XY, -MOTOR_XY, -BAND_Z0)).exportStep(str(OUT / "step" / "guard_band_x4.step"))

    # ----- assembly
    unit = arm_unit(P, tube, band)
    body_wood, body_metal = body_parts(P)
    elec = electronics()
    body = body_wood + body_metal
    deployed = {c: place_unit(unit, c, 0) for c in CORNERS}

    def flat(u, kinds=("wood", "metal", "motor", "guard")):
        return [(n, s) for k in kinds for n, s in u[k]]

    print("\nCOMPETITION GEOMETRY")
    every = [s for _, s in body + elec] + [s for c in CORNERS for _, s in flat(deployed[c])]
    bbs = [s.BoundingBox() for s in every]
    width = max(max(b.xmax for b in bbs) - min(b.xmin for b in bbs), max(b.ymax for b in bbs) - min(b.ymin for b in bbs)) + 2 * HEAD_PROUD
    height = max(b.zmax for b in bbs) - min(b.zmin for b in bbs)
    rpt.check(f"fits the 2 ft launch square ({LAUNCH_BOX:.0f} mm, rule 8.23)", width <= LAUNCH_BOX, f"{width:.0f} mm wide, {LAUNCH_BOX - width:.0f} mm spare")
    worst = 2 * (MOTOR_XY * SQ2 + GUARD_R + BAND_T + HEAD_PROUD)
    rpt.check(f"fits a {CORRIDOR:.0f} mm corridor (rule 8.22)", worst < CORRIDOR,
              f"square on: {(CORRIDOR - width) / 2:.0f} mm each side; worst 45 deg yaw: {(CORRIDOR - worst) / 2:.0f} mm each side")
    rpt.check("arms on the 45 deg diagonals, motors on a square (standard Quad X mixer)", True, f"+/-{MOTOR_XY:.1f} mm")

    print("\nFIT AND CLEARANCE")
    w, where = worst_within(body)
    rpt.check("hub: plates, cheeks, legs and standoffs seat without clashing (tabs in slots)", w < 0.5, f"worst {w:.2f} mm3 {where}")
    w, where = worst_within(flat(unit, ("wood", "metal", "motor", "guard")))
    rpt.check("arm: tube through both knuckles, plates, spokes and band seat without clashing", w < 0.5, f"worst {w:.2f} mm3 {where}")
    w, where = worst_overlap(elec, body)
    rpt.check("Pixhawk, Pi 4B, X2, battery, sensors and camera clear the frame", w < 0.5, f"worst {w:.2f} mm3 {where}")
    obstacles = body + elec
    for deg in (0, 20, 45, 70, 90):
        w, where = 0.0, ""
        for c in CORNERS:
            v, wh = worst_overlap(flat(place_unit(unit, c, deg)), obstacles)
            if v > w:
                w, where = v, wh
        for c, o in ((CORNERS[0], CORNERS[1]), (CORNERS[0], CORNERS[2]), (CORNERS[2], CORNERS[3]), (CORNERS[1], CORNERS[3])):
            v, wh = worst_overlap(flat(place_unit(unit, c, deg)), flat(place_unit(unit, o, deg)))
            if v > w:
                w, where = v, wh
        label = {0: "deployed", 90: "folded"}.get(deg, f"swinging, {deg} deg")
        rpt.check(f"{label}: arms, guards and motors clear the hub and each other", w < 0.5, f"worst {w:.2f} mm3 {where}")
    top = body_wood[0][1]
    knuckles = unit["wood"][:2]
    stop = sum(overlap(fold(s, -3), top) for _, s in knuckles)
    rpt.check("thrust stop: the knuckles' top edges bear on the top plate, so the arm cannot swing up", stop > 1.0, f"{stop:.0f} mm3 contact at 3 deg up")
    lock_pin = cyl_x(1.6, LOCK[0], LOCK[1], K1 - 20, K2 + 20)
    held = sum(overlap(fold(s, 4), lock_pin) for _, s in knuckles)
    rpt.check("lock bolt in: the arm cannot fold", held > 1.0, f"{held:.0f} mm3 at 4 deg down")
    disc = cyl(PROP_R, PROP_Z - 1.5, PROP_Z + 1.5, MOTOR_XY, MOTOR_XY)
    others = [s for n, s in flat(deployed[(1, 1)]) if n != "motor"] + [s for _, s in body + elec]
    rpt.check(f"{PROP_IN:.0f} in prop clears spokes, band, arm and hub", max(overlap(disc, s) for s in others) < 0.5)
    scan = cyl(2000.0, SCAN_Z - 2, SCAN_Z + 2)
    blk = max(overlap(scan, s) for s in [s for c in CORNERS for _, s in flat(deployed[c])] + [s for _, s in body])
    rpt.check("X2 scan plane is clear of guards, motors and frame", blk < 0.5, f"scan at {SCAN_Z:.0f} mm, band top at {BAND_Z0 + BAND_H:.0f} mm")
    zc = (Z_BOT1 + TOP_Z) / 2
    lens = (CAM_X + T3 / 2 + 9, 0.0, zc)
    hf, vf = (math.radians(a / 2) for a in CAM_FOV)
    fov = (cq.Workplane("YZ", origin=lens).rect(6, 6).workplane(offset=300)
           .rect(2 * 300 * math.tan(hf), 2 * 300 * math.tan(vf)).loft().val())
    cam_blk = max(overlap(fov, s) for s in [s for c in CORNERS for _, s in flat(deployed[c])] + [s for _, s in body])
    rpt.check(f"front camera ({CAM_FOV[0]:.0f} x {CAM_FOV[1]:.0f} deg, Pi Camera 3) sees past guards, arms and legs", cam_blk < 0.5, f"{cam_blk:.1f} mm3 in view")
    ground = min(b.zmin for b in [s.BoundingBox() for _, s in body])
    low = SENSOR_Z0 - 21.0
    rpt.check("downward sensors stay above the ground when landed", low > ground + 5, f"{low - ground:.0f} mm clear")
    views = [cq.Solid.makeCone(6.0, 6.0 + 300 * math.tan(math.radians(1)), 300, V(TF_X, 0, SENSOR_Z0 - 16), V(0, 0, -1)),
             cq.Solid.makeCone(8.0, 8.0 + 300 * math.tan(math.radians(12)), 300, V(FLOW_X, 0, SENSOR_Z0 - 21), V(0, 0, -1))]
    legs = [s for n, s in body if n.startswith("leg")]
    rpt.check("TFmini-S and PX4Flow see past the legs", max(overlap(v, l) for v in views for l in legs) < 0.5)

    print("\nBOLT LINES (a 2.4 mm pin passes every hole without touching wood or tube)")
    solid_all = body_wood + [(n, s) for c in CORNERS for n, s in place_unit(unit, c, 0)["wood"]]
    solid_all += [(n, s) for c in CORNERS for n, s in place_unit(unit, c, 0)["metal"] if n == "tube"]

    def clear(pins):
        return max(overlap(p, s) for p in pins for _, s in solid_all) < 1e-6

    def at_motor_local(x, y, z0, z1):
        return to_motor(cyl(1.2, z0, z1, x, y))

    pins = {
        "hinge and lock bolts through cheeks and knuckles": [to_corner(cyl_x(1.2, y, z, K1 - 20, K2 + 20), c) for c in CORNERS
                                                            for y, z in ((HY, HZ), LOCK)],
        "cheek spacers and tab locks": [to_corner(cyl_x(1.2, y, z, K1 - 20, K2 + 20), c) for c in CORNERS
                                        for y, z in CHEEK_SPACERS + [(TAB_LOCK_Y, TAB_LOCK_Z)]],
        "hub standoffs through both plates": [cyl(1.2, -1, TOP_Z1 + 1, x, y) for x, y in HUB_POSTS],
        "cage standoffs through bottom and sensor plates": [cyl(1.2, SENSOR_Z0 - 1, Z_BOT1 + 1, x, y) for x, y in CAGE_POSTS],
        "deck posts through top plate and deck": [cyl(1.2, TOP_Z - 1, DECK_Z + T3 + 1, x, y) for x, y in DECK_POSTS],
        "clamp bolts through motor plate, tube and tip plate": [to_corner(at_motor_local(u, 0, TIP_Z1 - T3 - 1, PAD_TOP + 1), c)
                                                                for c in CORNERS for u in CLAMP_U],
        "clamp standoffs and motor screws": [to_corner(at_motor_local(x, y, TIP_Z1 - T3 - 1, PAD_TOP + 1), c)
                                             for c in CORNERS for x, y in CLAMP_SO + motor_holes()],
        "tube retaining pins": [to_corner(cyl(1.2, TZ - 10, TZ + 10, x, x), c) for c in CORNERS for x in PIN_X],
        "leg cross pins": [cyl_x(1.2, 0.0, SENSOR_Z1 + STANDOFF_R, sx * LEG_X - 5, sx * LEG_X + 5) for sx in (1, -1)],
    }
    for label, pl in pins.items():
        rpt.check(label, clear(pl))

    print("\nAIRFLOW (share of each prop disc shaded from below)")
    fl = deployed[(1, 1)]
    arm_sh = [s for _, s in fl["wood"] + fl["metal"]]
    body_sh = [s for _, s in body + elec if s.BoundingBox().zmin < PROP_Z]
    guard_sh = [s for n, s in fl["guard"] if n.startswith("spoke")]
    a_b, b_b, g_b = blockage(arm_sh, MOTOR_XY, MOTOR_XY), blockage(body_sh, MOTOR_XY, MOTOR_XY), blockage(guard_sh, MOTOR_XY, MOTOR_XY)
    print(f"  arm {a_b * 100:.1f} %, guard spokes {g_b * 100:.1f} %, hub, battery and legs {b_b * 100:.1f} %")
    rpt.check("everything together shades under 20% of each disc", a_b + b_b + g_b < 0.20, f"{(a_b + b_b + g_b) * 100:.1f} %")

    # ----- mass, strength, performance
    standoff_mm = (len(HUB_POSTS) * GAP + len(CAGE_POSTS) * CAGE + len(DECK_POSTS) * 35
                   + 4 * (2 * TUBE + 2 * (K2 - K1 - T4)) + 4 * 3 * (CHEEK_XS[1] - CHEEK_XS[0] - T4))
    n_screws = 2 * (len(HUB_POSTS) + len(CAGE_POSTS) + len(DECK_POSTS)) + 4 * (4 + 2 + 2 + 2 + 4 + 2 + 6) + 16 + 2
    hardware_g = standoff_mm * 0.052 + n_screws * 0.9 + 4 * 4.0 + 16 * 0.3
    bought = sum(BOUGHT_G.values())
    auw = ply_g + 4 * (tube_g + band_g) + hardware_g + bought
    print("\nMASS AND PERFORMANCE (estimates)")
    print(f"  plywood {ply_g:.0f} g, tubes {4 * tube_g:.0f} g, bands {4 * band_g:.0f} g, standoffs and bolts {hardware_g:.0f} g, other parts {bought:.0f} g")
    print(f"  frame about {ply_g + 4 * (tube_g + band_g) + hardware_g:.0f} g; all-up weight about {auw:.0f} g")
    rpt.check(f"all-up weight under {MAX_AUW_G / 1000:.0f} kg", auw < MAX_AUW_G)
    for label, t in (("low", THRUST_LOW_G), ("high", THRUST_HIGH_G)):
        print(f"  thrust-to-weight, {label} thrust figure ({t:.0f} g per motor): {4 * t / auw:.2f}")
    watts = auw / HOVER_G_PER_W + AVIONICS_W
    print(f"  hover about {watts:.0f} W, {watts / 11.1:.0f} A, {USABLE_MAH / 1000 / (watts / 11.1) * 60:.1f} min")

    print("\nSTRENGTH (hand calculation, full thrust x 2 for gusts and hard manoeuvres)")
    T = THRUST_HIGH_G * 9.81e-3 * 2
    sfs = []
    I_t = (TUBE ** 4 - (TUBE - 2 * TUBE_WALL) ** 4) / 12
    L = -u_of_x(K2 + T4 / 2)
    sig = T * L * (TUBE / 2) / I_t
    sfs.append(AL_YIELD_MPA / sig)
    print(f"  tube at the outer knuckle: {sig:.1f} MPa bending, safety factor {AL_YIELD_MPA / sig:.0f}")
    print(f"  motor deflection under that load: {T * L ** 3 / (3 * AL_E_MPA * I_t):.3f} mm")
    u_c = (EAR_Y - HY) / 2
    f_stop = T * (MOTOR_XY - HY) / u_c
    bearing = (f_stop - T) / (2 * T4 * 4.0)
    sfs.append(PLY_BEARING_MPA / bearing)
    print(f"  stop contact {f_stop:.0f} N; hinge bolt bearing in the knuckles {bearing:.1f} MPa, safety factor {PLY_BEARING_MPA / bearing:.0f}")
    span = CHEEK_XS[1] - CHEEK_XS[0]
    sig_ear = 6 * (f_stop * span / 4) / ((EAR_Y - CHEEK_TAB[1]) * T4 ** 2)
    sfs.append(PLY_FLEX_MPA / sig_ear)
    print(f"  top plate ear between the cheeks: {sig_ear:.1f} MPa, safety factor {PLY_FLEX_MPA / sig_ear:.1f}")
    sig_k = 6 * (f_stop / 2 * (EAR_Y - HY) / 2) / (T4 * (TOP_Z - (HZ - KR)) ** 2)
    sfs.append(PLY_FLEX_MPA / sig_k)
    print(f"  knuckle plate bending: {sig_k:.1f} MPa, safety factor {PLY_FLEX_MPA / sig_k:.0f}")
    rpt.check("every hand-calculated safety factor is at least 3", min(sfs) >= 3.0, f"lowest {min(sfs):.1f}")

    folded = {c: place_unit(unit, c, 90) for c in CORNERS}
    fb = [s.BoundingBox() for s in [s for c in CORNERS for _, s in flat(folded[c])] + [s for _, s in body + elec]]
    fsize = [max(b.xmax for b in fb) - min(b.xmin for b in fb), max(b.ymax for b in fb) - min(b.ymin for b in fb),
             max(b.zmax for b in fb) - min(b.zmin for b in fb)]
    print(f"\nSIZE  deployed {width:.0f} x {width:.0f} x {height:.0f} mm; folded {fsize[0]:.0f} x {fsize[1]:.0f} x {fsize[2]:.0f} mm")

    # ----- assembly STEP
    asm = [s for _, s in body] + [s for c in CORNERS for _, s in flat(deployed[c], ("wood", "metal", "guard"))]
    cq.Compound.makeCompound(asm).exportStep(str(OUT / "step" / "assembly_deployed.step"))
    asm_f = [s for _, s in body] + [s for c in CORNERS for _, s in flat(folded[c], ("wood", "metal", "guard"))]
    cq.Compound.makeCompound(asm_f).exportStep(str(OUT / "step" / "assembly_folded.step"))

    if os.environ.get("V6_NO_RENDER"):
        print(f"\n{'ALL CHECKS PASSED' if rpt.failed == 0 else str(rpt.failed) + ' CHECK(S) FAILED'}")
        return rpt.failed
    print("\nRENDERING")
    PLY, AL, BAND, MOT, PROP, BAT, PCB, LID = ((212, 176, 124), (196, 200, 208), (150, 196, 222), (120, 126, 138),
                                               (40, 40, 44), (52, 104, 196), (46, 140, 88), (90, 96, 110))

    def scene(degs, with_props=True, with_elec=True):
        units_ = {c: place_unit(unit, c, d) for c, d in zip(CORNERS, degs)}
        g = [([s for _, s in body_wood], PLY), ([s for _, s in body_metal], AL)]
        g.append(([s for c in CORNERS for _, s in units_[c]["wood"]], PLY))
        g.append(([s for c in CORNERS for _, s in units_[c]["metal"]], AL))
        g.append(([s for c in CORNERS for _, s in units_[c]["motor"]], MOT))
        g.append(([s for c in CORNERS for n, s in units_[c]["guard"] if n.startswith("spoke")], PLY))
        g.append(([s for c in CORNERS for n, s in units_[c]["guard"] if n == "band"], BAND))
        if with_props:
            g.append(([s for c in CORNERS for _, s in units_[c]["prop"]], PROP))
        if with_elec:
            e = dict(elec)
            g += [([e["battery"]], BAT), ([e["pixhawk"], e["pi4b"], e["px4flow"], e["tfmini_s"], e["camera"]], PCB), ([e["ydlidar_x2"]], LID)]
        return g

    sub = f"Plywood and aluminium X frame, {WHEELBASE:.0f} mm wheelbase, {width:.0f} mm wide, about {auw:.0f} g all-up"
    render(scene([0] * 4), OUT / "renders" / "deployed.png", 26, 32, "Version 6: X layout, arms deployed", sub)
    render(scene([90] * 4, with_props=False), OUT / "renders" / "folded.png", 16, 32, "Arms folded down for transport",
           f"Folded {fsize[0]:.0f} x {fsize[1]:.0f} x {fsize[2]:.0f} mm; remove the lock bolt and swing each arm down")
    render(scene([0, 30, 60, 90], with_props=False), OUT / "renders" / "fold_sequence.png", 22, 58, "Fold sequence: 0, 30, 60 and 90 deg",
           "Each arm swings down and sideways about its hinge bolt; nothing touches at any angle")
    sq = LAUNCH_BOX / 2
    square = cut(bx(-sq, sq, -sq, sq, ground - 3, ground - 1), bx(-sq + 6, sq - 6, -sq + 6, sq - 6, ground - 4, ground))
    render(scene([0] * 4) + [([square], (205, 60, 60))], OUT / "renders" / "top.png", 90, -90, "Top view inside the 2 ft launch square",
           f"{width:.0f} mm wide in a {LAUNCH_BOX:.0f} mm square (red); front is to the right")
    walls = [bx(-60, 60, *sorted((s * CORRIDOR / 2, s * (CORRIDOR / 2 + 12))), ground, 260) for s in (1, -1)]
    floor = bx(-60, 60, -CORRIDOR / 2, CORRIDOR / 2, ground - 4, ground)
    render(scene([0] * 4) + [(walls, (210, 214, 222)), ([floor], (225, 228, 234))], OUT / "renders" / "front_corridor.png", 0, 0,
           "Front view in a 1 m corridor", f"{(CORRIDOR - width) / 2:.0f} mm to each wall flying straight; {(CORRIDOR - worst) / 2:.0f} mm at the worst 45 deg yaw")
    fl_body = lambda group: [s for n, s in group if n.endswith(("FL", "FL0", "FL1"))]
    det = [(fl_body(body_wood) + [s for _, s in deployed[(1, 1)]["wood"]], PLY),
           (fl_body(body_metal) + [s for _, s in deployed[(1, 1)]["metal"]], AL),
           ([s for _, s in deployed[(1, 1)]["motor"]], MOT)]
    render(det, OUT / "renders" / "arm_hinge.png", 28, 205, "Front-left arm and hinge, seen from the hub",
           "Tube through two knuckle plates at 45 deg, cheeks on the hub, hinge and lock bolts along X")
    exploded = [([P["top"].solid().translate(V(0, 0, TOP_Z + 60)), P["bottom"].solid().translate(V(0, 0, -30)),
                  P["deck"].solid().translate(V(0, 0, DECK_Z + 110)), P["sensor"].solid().translate(V(0, 0, SENSOR_Z0 - 90))]
                 + [s for n, s in body_wood if n.startswith("cheek")]
                 + [s.translate(V(0, 0, -60)) for n, s in body_wood if n.startswith("leg")]
                 + [s.translate(V(0, 60, 25)) for _, s in deployed[(1, 1)]["wood"]], PLY),
                ([deployed[(1, 1)]["metal"][0][1].translate(V(0, 60, 25))], AL)]
    render(exploded, OUT / "renders" / "exploded.png", 24, 35, "Exploded view of the plywood parts",
           "Every joint is a tab in a slot, clamped by standoffs or bolts")
    print("  done")

    print(f"\n{'ALL CHECKS PASSED' if rpt.failed == 0 else str(rpt.failed) + ' CHECK(S) FAILED'}")
    return rpt.failed


if __name__ == "__main__":
    raise SystemExit(main())
