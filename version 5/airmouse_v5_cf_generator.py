#!/usr/bin/env python3
"""
NIDAR AirMouse frame, version 5: carbon fibre plate frame.

Every structural part is a flat carbon fibre plate, cut by CNC from 2 mm and
3 mm sheet, joined by tabs, slots and aluminium standoffs. No glue is needed:
every tab is clamped between two plates or pinned by a standoff.

Layout: an "H" spine. The body is a long, narrow sandwich along X (front is +X).
At each end a carbon clevis (two cheek plates) carries two box arms that run
sideways (+/-Y) to the motors. Each arm is two vertical carbon side plates, so
it folds DOWN about a bolt along X, like versions 2 and 4. The motors sit on the
same square as version 4 (a normal Quad X for PX4), so the guards, launch-square
fit and corridor clearance are unchanged. Only the prop guards are printed.

Install (Python 3.12):  pip install cadquery ezdxf shapely numpy trimesh pillow
Run:                    python airmouse_v5_cf_generator.py
Out:                    output/dxf (CNC), output/step, output/stl (guards), output/renders
"""
import math
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
# --- Rules: NIDAR 2026-27 Rulebook 8.22, 8.23, 8.30 and the AirMouse brief
LAUNCH_BOX = 609.6          # 2 ft x 2 ft take-off square
CORRIDOR = 1000.0           # minimum corridor width
MAX_AUW_G = 10000.0         # all-up weight limit

# --- Carbon sheet (3K twill, CNC cut)
T3, T2 = 3.0, 2.0           # the two sheet thicknesses used
CF_DENSITY = 1.55
SLOT_PLAY = 0.2             # slot is this much longer and wider than its tab
DOGBONE_R = 0.8             # corner relief so square tabs seat in milled slots
MIN_WEB = 3.0               # carbon left between any two holes, or a hole and an edge
SHEET_W = 400.0             # width of the sheet the parts are nested on

# --- Printer (guards only): Creality Ender 3 V3 SE
BED, BED_H, BED_MARGIN = 220.0, 250.0, 2.5

# --- Propulsion: everything is sized from the prop, as in version 4
PROP_IN = 10.0
GUARD_GAP = 8.0
BAND_T, BAND_H = 1.2, 16.0
RING_GAP = 4.0
HEAD_PROUD = 3.0
MOTOR_HOLES = (16.0, 19.0)  # 2212 cross pattern, M3. MEASURE YOURS.
MOTOR_R, MOTOR_H = 14.0, 30.0
PROP_ABOVE_PAD = 38.0       # motor plate top to prop plane. MEASURE YOURS.

PROP_R = PROP_IN * 25.4 / 2
GUARD_R = PROP_R + GUARD_GAP
MOTOR_XY = GUARD_R + BAND_T + RING_GAP / 2
WHEELBASE = 2 * math.sqrt(2) * MOTOR_XY

# --- Body sandwich
GAP = 25.0                  # M3 x 25 standoffs; the Pixhawk lives here
Z_BOT1 = T3                 # top of the bottom plate (its underside is z = 0)
TOP_Z = Z_BOT1 + GAP        # underside of the top plate
TOP_Z1 = TOP_Z + T3         # top of the top plate
SPINE_POSTS = [(sx * x, sy * y) for x, y in ((20.0, 40.0), (62.0, 22.0), (106.0, 8.0)) for sx in (1, -1) for sy in (1, -1)]

# --- Arm (box of two vertical side plates) and clevis (two cheek plates)
AX = MOTOR_XY               # the arm runs along Y at this X
ARM_IN = 25.0               # gap between side plates: M3 x 25 standoffs
SIDE_C = ARM_IN / 2 + T3 / 2
ARM_OUT = ARM_IN / 2 + T3
WASHER = 0.5                # PTFE / nylon washer between arm and cheek
CHEEK_C = ARM_OUT + WASHER + T3 / 2
CHEEK_GAP = 2 * (ARM_OUT + WASHER)
HY, HZ = 18.0, 15.0         # hinge bolt (along X) position for the +Y arms
KR = 9.0                    # knuckle radius around the hinge
LOCK_DY, LOCK_DZ = 7.0, 7.0  # lock bolt, outboard of and above the hinge
CHEEK_Y = HY + LOCK_DY + 8.0
CHEEK_TAB = (13.0, 27.0)    # cheek top tabs span this |y| range
TAB_LOCK_Y = 20.0
STANDOFF_R = 2.75           # 5.5 mm hex standoffs, modelled round
TAB_LOCK_Z = TOP_Z1 + STANDOFF_R
CHEEK_SPACERS = [(0.0, 8.0), (0.0, 21.0)]
TIP_H = 15.0                # M3 x 15 standoffs between motor plate and tip plate
PAD_TOP = TOP_Z + T2        # top of the motor plate
TIP_BOT = TOP_Z - TIP_H     # bottom of the side plates at the tip
TIP_BACK, TIP_FRONT = 25.0, 25.0
TAB_U = ((-20.0, -10.0), (8.0, 16.0))   # side plate tabs, along the arm from the motor axis
CLAMP = 21.5                # motor-plate clamp standoffs at (+/-CLAMP, +/-CLAMP) from the motor
ARM_SPACER_Y = (HY + 40.0, HY + 80.0)
SPOKE_R, SPOKE_T = 24.0, 5.0
CAM_X = 165.0               # front camera plate (between the plate ends)

# --- Electronics stack
DECK_Z = TOP_Z1 + 35.0      # M3 x 35 standoffs
DECK_HALF, DECK_CHAMFER = (46.0, 37.0), 8.0
DECK_POSTS = [(sx * 34.0, sy * 32.0) for sx in (1, -1) for sy in (1, -1)]
PI_HOLES = [(sx * 29.0, sy * 24.5) for sx in (1, -1) for sy in (1, -1)]
X2_HOLES = [(-35.3, 0.0), (36.0, 18.0), (36.0, -18.0)]
X2_SCAN_ABOVE_BASE = 42.0

# --- Battery cage, sensors and legs (under the bottom plate)
BATTERY = (165.0, 50.0, 40.0)            # upper limit; pack not measured
CAGE = 45.0                 # M3 x 45 standoffs: room for the battery plus foam
SENSOR_Z1 = -CAGE           # top of the sensor plate
SENSOR_Z0 = SENSOR_Z1 - T2
CAGE_POSTS = [(sx * 96.0, sy * 8.0) for sx in (1, -1) for sy in (1, -1)]
LEG_X = 88.0
LEG_FOOT_Y, LEG_FOOT_Z = 46.0, SENSOR_Z0 - 40.0
FLOW_X, TF_X = 50.0, -50.0
TF_SPACING = 36.0

# --- Mass and thrust (same assumptions as version 4)
PETG = 1.27
BOUGHT_G = {"motors": 4 * 57, "ESCs": 4 * 35, "props": 4 * 13, "battery": 471, "Pixhawk": 38,
            "Pi 4B + cooling + regulator": 76, "YDLidar X2": 200, "TFmini-S": 5, "PX4Flow": 20,
            "camera": 10, "power module, wiring": 100, "receiver + telemetry": 40}
THRUST_LOW_G, THRUST_HIGH_G = 642.0, 850.0
HOVER_G_PER_W, USABLE_MAH, AVIONICS_W = 7.0, 4000.0, 20.0
CF_FLEX_MPA, CF_BEARING_MPA, CF_E_MPA = 300.0, 150.0, 45000.0   # conservative plate values

M2, M25, M3, M4 = 2.4, 2.8, 3.4, 4.3
OUT = Path(__file__).resolve().parent / "output"
CORNERS = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
PROP_Z = PAD_TOP + PROP_ABOVE_PAD
BAND_Z0 = PROP_Z - BAND_H / 2
SCAN_Z = DECK_Z + T2 + X2_SCAN_ABOVE_BASE
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


def teardrop(a, b, d):
    r = d / 2
    pts = [(a + r * math.cos(math.radians(t)), b + r * math.sin(math.radians(t))) for t in range(-225, 46, 15)]
    return pts + [(a, b + r * math.sqrt(2))]


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


def yz(shape, x0):
    """Profile drawn in (y, z) and extruded along +local z, stood up: thickness runs along X from x0."""
    return remap(shape, [[0, 0, 1, x0], [1, 0, 0, 0], [0, 1, 0, 0]])


def mirror_x(shape):
    return shape.mirror("YZ")


def mirror_y(shape):
    return shape.mirror("XZ")


# =============================================================================
# 2D CARBON PARTS
# =============================================================================
def rounded(poly, r):
    """Round every corner of a polygon with radius r (convex and concave)."""
    return poly.buffer(r, 5).buffer(-2 * r, 5).buffer(r, 5)


def slot(x0, y0, x1, y1, w):
    """Round-ended slot between two points."""
    return LineString([(x0, y0), (x1, y1)]).buffer(w / 2, 8)


def tab_slot(cx, cy, w, h):
    """Rectangular slot for a tab, w x h already including play, with dogbone reliefs."""
    rect = sbox(cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
    bones = [Point(cx + sx * w / 2, cy + sy * h / 2).buffer(DOGBONE_R, 4) for sx in (1, -1) for sy in (1, -1)]
    return unary_union([rect] + bones)


def mirror4(q1):
    """Build a symmetric outline from its +x, +y quarter, given as points from the +y axis to the +x axis."""
    quarter = Polygon([(0.0, 0.0)] + q1)
    parts = [quarter, affinity.scale(quarter, -1, 1, origin=(0, 0)),
             affinity.scale(quarter, 1, -1, origin=(0, 0)), affinity.scale(quarter, -1, -1, origin=(0, 0))]
    return unary_union(parts).buffer(0.01).buffer(-0.01)


class CF:
    """A flat carbon part: outline, slot/window cut-outs and round holes, in its own 2D frame."""

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
        return self.shape2d().area * self.t / 1000 * CF_DENSITY


# ----- top and bottom plates ----------------------------------------------------
CHEEK_XS = (AX - CHEEK_C, AX + CHEEK_C)   # cheek centre planes at the +X end


def top_plate():
    q1 = [(0, 46), (48, 46), (64, 30), (100, 14), (114, 14), (114, CHEEK_Y + 1), (162, CHEEK_Y + 1),
          (162, 18), (170, 18), (170, 0)]
    outline = rounded(mirror4(q1), 3.0)
    holes = [(x, y, M3) for x, y in SPINE_POSTS + DECK_POSTS]
    cuts = []
    for sx in (1, -1):
        for xc in CHEEK_XS:
            for sy in (1, -1):
                cy = sy * (CHEEK_TAB[0] + CHEEK_TAB[1]) / 2
                cuts.append(tab_slot(sx * xc, cy, T3 + SLOT_PLAY, CHEEK_TAB[1] - CHEEK_TAB[0] + SLOT_PLAY))
        cuts.append(tab_slot(sx * CAM_X, 0.0, T2 + SLOT_PLAY, 16.0 + SLOT_PLAY))      # camera plate
        cuts.append(slot(sx * 72.0, 0, sx * 96.0, 0, 8.0))                            # lightening
        cuts.append(slot(sx * 30.0, -8.0, sx * 30.0, 8.0, 14.0))                      # lightening
        cuts.append(slot(sx * (AX - 7), 0, sx * (AX + 7), 0, 9.0))                    # lightening between the cheeks
    cuts += [slot(-12.0, sy * 32.0, 12.0, sy * 32.0, 8.0) for sy in (1, -1)]           # wiring up to the Pi
    return CF("top_plate", outline, T3, 1, holes, cuts, "Body top. Carries the arms through the cheeks.")


def bottom_plate():
    q1 = [(0, 46), (48, 46), (64, 30), (100, 14), (114, 14), (114, 7.5), (170, 7.5), (170, 0)]
    outline = rounded(mirror4(q1), 3.0)
    holes = [(x, y, M3) for x, y in SPINE_POSTS + CAGE_POSTS]
    cuts = []
    for sx in (1, -1):
        for xc in CHEEK_XS:
            cuts.append(tab_slot(sx * xc, 0.0, T3 + SLOT_PLAY, 7.0 + SLOT_PLAY))
        cuts.append(tab_slot(sx * CAM_X, 0.0, T2 + SLOT_PLAY, 6.0 + SLOT_PLAY))
        cuts.append(tab_slot(sx * LEG_X, 0.0, T3 + SLOT_PLAY, 24.0 + SLOT_PLAY))     # leg top tab
        cuts.append(slot(sx * 30.0, -20.0, sx * 30.0, 20.0, 6.0))                    # Pixhawk zip ties
        cuts.append(slot(sx * 66.0, 0, sx * 78.0, 0, 8.0))                           # lightening
    cuts += [slot(-10.0, sy * 32.0, 10.0, sy * 32.0, 8.0) for sy in (1, -1)]          # lightening
    cuts.append(slot(-10.0, 0, 10.0, 0, 10.0))                                       # sensor wiring
    return CF("bottom_plate", outline, T3, 1, holes, cuts, "Body bottom. Holds the battery cage and legs.")


# ----- clevis cheeks (YZ profile: a = y, b = z) ------------------------------------
def cheek():
    body = rounded(sbox(-CHEEK_Y, Z_BOT1, CHEEK_Y, TOP_Z), 1.0)
    tab_top = TAB_LOCK_Z + M3 / 2 + 3.5
    tabs = [rounded(sbox(s * CHEEK_TAB[0], TOP_Z - 2, s * CHEEK_TAB[1], tab_top), 2.0) for s in (1, -1)]
    tabs.append(sbox(-3.5, 0.0, 3.5, Z_BOT1 + 1))
    outline = unary_union([body] + tabs)
    holes = [(s * HY, HZ, M4) for s in (1, -1)] + [(s * (HY + LOCK_DY), HZ + LOCK_DZ, M3) for s in (1, -1)]
    holes += [(y, z, M3) for y, z in CHEEK_SPACERS] + [(s * TAB_LOCK_Y, TAB_LOCK_Z, M3) for s in (1, -1)]
    cuts = [Point(s * 10.0, 11.0).buffer(3.5, 8) for s in (1, -1)]                  # wire windows
    return CF("cheek", outline, T3, 4, holes, cuts, "Clevis plate. Two per end, carrying both arms on that end.")


# ----- arm side plate (YZ profile, +Y arm) -----------------------------------------
def arm_bottom(y):
    y1 = MOTOR_XY - TIP_BACK
    if y >= y1:
        return TIP_BOT
    return (HZ - KR) + (TIP_BOT - (HZ - KR)) * (y - HY) / (y1 - HY)


def arm_spacers():
    return [(y, (arm_bottom(y) + TOP_Z) / 2) for y in ARM_SPACER_Y]


def side_plate():
    main = Polygon([(HY, HZ - KR), (MOTOR_XY - TIP_BACK, TIP_BOT), (MOTOR_XY + TIP_FRONT, TIP_BOT),
                    (MOTOR_XY + TIP_FRONT, TOP_Z), (HY, TOP_Z)])
    body = unary_union([main, Point(HY, HZ).buffer(KR, 64)])
    body = body.buffer(-1.5, 5).buffer(1.5, 5)          # round the outer corners
    tabs = []
    for a0, a1 in TAB_U:
        tabs.append(sbox(MOTOR_XY + a0, TOP_Z - 1, MOTOR_XY + a1, TOP_Z + T2))
        tabs.append(sbox(MOTOR_XY + a0, TIP_BOT - T2, MOTOR_XY + a1, TIP_BOT + 1))
    outline = unary_union([body] + tabs)
    holes = [(HY, HZ, M4), (HY + LOCK_DY, HZ + LOCK_DZ, M3)] + [(y, z, M3) for y, z in arm_spacers()]
    s1, s2 = ARM_SPACER_Y
    spans = [(HY + LOCK_DY + 8.0, s1 - 9.5), (s1 + 9.5, s2 - 9.5), (s2 + 9.5, MOTOR_XY - 12.0), (MOTOR_XY - 6.0, MOTOR_XY + 14.0)]
    cuts = []
    for y0, y1 in spans:
        h = TOP_Z - max(arm_bottom(y0), arm_bottom(y1))
        w = min(9.0, h - 8.0)
        zc = lambda y: (arm_bottom(y) + TOP_Z) / 2
        cuts.append(slot(y0 + w / 2, zc(y0), y1 - w / 2, zc(y1), w))
    return CF("arm_side", outline, T3, 8, holes, cuts, "Arm box wall. Two per arm.")


# ----- motor plate and tip plate (XY, origin on the motor axis, +y outboard) ----------
def motor_holes():
    pts = []
    for k, span in enumerate(MOTOR_HOLES):
        for s in (1, -1):
            x, y = (s * span / 2, 0.0) if k == 0 else (0.0, s * span / 2)
            c = math.cos(math.radians(45))
            pts.append(((x - y) * c, (x + y) * c))
    return pts


def spoke_points():
    """Guard spoke bolts, spokes along +x, +y, -x, -y from the motor."""
    pts = []
    for k in range(4):
        a = math.radians(90 * k)
        for t in (-SPOKE_T, SPOKE_T):
            pts.append((SPOKE_R * math.cos(a) - t * math.sin(a), SPOKE_R * math.sin(a) + t * math.cos(a)))
    return pts


def clamp_points():
    return [(sx * CLAMP, sy * CLAMP) for sx in (1, -1) for sy in (1, -1)]


def side_tab_slots():
    return [tab_slot(sx * SIDE_C, (a0 + a1) / 2, T3 + SLOT_PLAY, a1 - a0 + SLOT_PLAY) for sx in (1, -1) for a0, a1 in TAB_U]


def motor_plate():
    sq = rounded(sbox(-27.0, -27.0, 27.0, 27.0), 3.0)
    lobes = [affinity.rotate(rounded(sbox(14.0, -12.0, 33.0, 12.0), 3.0), 90 * k, origin=(0, 0)) for k in range(4)]
    outline = rounded(unary_union([sq] + lobes), 2.0)
    holes = [(x, y, M3) for x, y in motor_holes() + spoke_points() + clamp_points()] + [(0.0, 0.0, 6.5)]
    cuts = side_tab_slots() + [slot(-2.5, -15.0, 2.5, -15.0, 5.0)]     # motor wires into the arm
    return CF("motor_plate", outline, T2, 4, holes, cuts, "Motor seat, top of the arm tip. Guard spokes bolt here.")


def tip_plate():
    outline = rounded(sbox(-27.0, -27.0, 27.0, 27.0), 3.0)
    # Motor screws go in before the tip plate; these holes let a 2.5 mm hex key reach them.
    holes = [(x, y, M3) for x, y in clamp_points()] + [(x, y, 3.6) for x, y in motor_holes()]
    return CF("tip_plate", outline, T2, 4, holes, side_tab_slots(), "Closes the arm tip from below; hex-key access to the motor screws.")


# ----- deck, sensor plate, legs, camera plate ---------------------------------------
def deck():
    hx, hy = DECK_HALF
    c = DECK_CHAMFER
    outline = rounded(Polygon([(hx - c, -hy), (hx, -hy + c), (hx, hy - c), (hx - c, hy), (-hx + c, hy),
                               (-hx, hy - c), (-hx, -hy + c), (-hx + c, -hy)]), 2.0)
    holes = [(x, y, M3) for x, y in DECK_POSTS] + [(x, y, M25) for x, y in PI_HOLES + X2_HOLES]
    cuts = [slot(-16.0, sy * 12.0, 16.0, sy * 12.0, 8.0) for sy in (1, -1)]
    return CF("lidar_deck", outline, T2, 1, holes, cuts, "YDLidar X2 on top, Raspberry Pi 4B hanging underneath.")


def sensor_plate():
    q1 = [(0, 36), (58, 36), (80, 25), (101, 16), (101, 0)]
    outline = rounded(mirror4(q1), 3.0)
    holes = [(x, y, M3) for x, y in CAGE_POSTS] + [(TF_X, s * TF_SPACING / 2, M2) for s in (1, -1)]
    cuts = [tab_slot(sx * LEG_X, 0.0, T3 + SLOT_PLAY, 30.0 + SLOT_PLAY) for sx in (1, -1)]
    cuts += [slot(sx * 13.0 - 9.5, sy * 22.0, sx * 13.0 + 9.5, sy * 22.0, 3.0) for sx in (1, -1) for sy in (1, -1)]  # battery straps
    cuts += [slot(FLOW_X + s * 17.0, -13.5, FLOW_X + s * 17.0, 13.5, 3.0) for s in (1, -1)]   # PX4Flow zip ties
    cuts.append(slot(FLOW_X - 4.0, 0, FLOW_X + 4.0, 0, 14.0))                                 # PX4Flow lens
    cuts.append(slot(TF_X - 4.0, 0, TF_X + 4.0, 0, 12.0))                                     # TFmini-S window
    return CF("sensor_plate", outline, T2, 1, holes, cuts, "Battery floor; PX4Flow and TFmini-S underneath.")


def leg():
    """YZ profile: a = y, b = z. Top tab in the bottom plate, neck through the sensor plate, feet below."""
    fy, fz = LEG_FOOT_Y, LEG_FOOT_Z
    pts = [(-12, Z_BOT1), (12, Z_BOT1), (12, 0), (15, 0), (15, SENSOR_Z0), (24, SENSOR_Z0), (fy + 6, fz + 4), (fy + 6, fz),
           (fy - 6, fz), (12, SENSOR_Z0 - 9), (-12, SENSOR_Z0 - 9), (-fy + 6, fz), (-fy - 6, fz), (-fy - 6, fz + 4),
           (-24, SENSOR_Z0), (-15, SENSOR_Z0), (-15, 0), (-12, 0)]
    outline = Polygon(pts).buffer(0.8, 3).buffer(-0.8, 3)
    holes = [(0.0, SENSOR_Z1 + STANDOFF_R, M3)]                         # cross pin sits on the sensor plate
    cuts = [slot(0, -35.0 + 7.0, 0, -9.0 - 7.0, 14.0)]
    return CF("leg", outline, T3, 2, holes, cuts, "Landing leg and battery end-stop, one front and one rear.")


def camera_plate():
    body = sbox(-16.0, Z_BOT1, 16.0, TOP_Z)
    outline = unary_union([rounded(body, 1.0), sbox(-8.0, TOP_Z - 1, 8.0, TOP_Z1), sbox(-3.0, 0.0, 3.0, Z_BOT1 + 1)])
    zc = (Z_BOT1 + TOP_Z) / 2
    holes = [(sy * 10.5, zc + sz * 6.25, 2.2) for sy in (1, -1) for sz in (1, -1)]
    return CF("camera_plate", outline, T2, 1, holes, [], "Front camera mount (Pi Camera footprint), clamped between the plates.")


# =============================================================================
# PRINTED GUARDS (from version 4, spokes turned to line up with the arm)
# =============================================================================
def make_spoke():
    r0, r1 = 18.0, 31.0
    t0, t1 = GUARD_R - 3.0, GUARD_R
    low, high = BAND_Z0, BAND_Z0 + BAND_H
    outer = [(r0, PAD_TOP), (r1, PAD_TOP), (t0, low), (t1, low), (t1, high), (t0, high),
             (t0, PROP_Z - 6.0), (r0, PAD_TOP + 12.0)]
    web = prism(outer, 0, 2.4)
    window = prism([(r1 + 8, PAD_TOP + 5.0), (t0 - 14, low + 1.0), (r1 + 8, PAD_TOP + 10.0)], -1, 4)
    spoke = fuse(cut(web, window), bx(r0, r1, PAD_TOP, PAD_TOP + 3.0, 0, 20.0), bx(t0, t1, low, high, 0, 20.0))
    holes = []
    for w in (10.0 - SPOKE_T, 10.0 + SPOKE_T):
        holes.append(remap(prism(teardrop(SPOKE_R, w, M3), 0, 8), [[1, 0, 0, 0], [0, 0, -1, PAD_TOP + 6], [0, 1, 0, 0]]))
    for w in (3.0, 17.0):
        holes.append(remap(prism(teardrop(PROP_Z, w, M3), t0 - 1, t1 + 1), [[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]]))
    return cut(spoke, *holes)


def make_arc():
    r0, r1 = GUARD_R, GUARD_R + BAND_T
    prof = [(r0 - 2.0, 0.0), (r1, 0.0), (r1, BAND_H), (r0 - 2.0, BAND_H), (r0, BAND_H - 2.0), (r0, 2.0)]
    gap = math.degrees(0.15 / GUARD_R)
    ring = cq.Workplane("XZ").polyline(prof).close().revolve(90.0 - 2 * gap, (0, 0, 0), (0, 1, 0)).val()
    bb = ring.BoundingBox()
    mid = math.degrees(math.atan2((bb.ymin + bb.ymax) / 2, (bb.xmin + bb.xmax) / 2))
    arc = rot_z(ring, -mid)
    for side in (1, -1):
        ang = side * (45 - gap - math.degrees(7.0 / GUARD_R))
        arc = arc.fuse(rot_z(bx(r0 - 0.5, r1 + 0.8, -6.0, 6.0, 0.0, BAND_H), ang))
        hole = remap(prism(teardrop(0.0, BAND_H / 2, M3), 0, 12), [[0, 0, 1, r0 - 4], [1, 0, 0, 0], [0, 1, 0, 0]])
        arc = arc.cut(rot_z(hole, ang))
    return arc.clean()


def guard_pieces(spoke, arc):
    placed = remap(spoke, [[1, 0, 0, 0], [0, 0, -1, 10.0], [0, 1, 0, 0]])
    out = []
    for k in range(4):
        a = 90 * k
        out.append((f"spoke{k}", rot_z(placed, a).translate(V(AX, MOTOR_XY, 0))))
        out.append((f"arc{k}", rot_z(arc.translate(V(0, 0, BAND_Z0)), a + 45).translate(V(AX, MOTOR_XY, 0))))
    return out


def prop_blades():
    """Two-blade 10 in prop, for pictures only."""
    blade = Polygon([(14, -5), (40, -11), (90, -9), (PROP_R - 3, -4), (PROP_R, 0), (PROP_R - 3, 4), (90, 8), (40, 10), (14, 5)])
    both = unary_union([blade, affinity.rotate(blade, 180, origin=(0, 0)), Point(0, 0).buffer(16, 24)])
    p = prism(list(both.exterior.coords)[:-1], PROP_Z - 1.5, PROP_Z + 1.5)
    return rot_z(p, 25).translate(V(AX, MOTOR_XY, 0))


# =============================================================================
# ASSEMBLY
# =============================================================================
def fold(shape, deg):
    """Swing a +Y arm part about its hinge line: 0 deployed, 90 folded down."""
    return (shape.translate(V(0, -HY, -HZ)).rotate(V(0, 0, 0), V(1, 0, 0), -deg).translate(V(0, HY, HZ)))


def to_corner(shape, corner):
    sx, sy = corner
    if sy < 0:
        shape = mirror_y(shape)
    if sx < 0:
        shape = mirror_x(shape)
    return shape


def build_parts():
    return {"top": top_plate(), "bottom": bottom_plate(), "cheek": cheek(), "side": side_plate(),
            "motor": motor_plate(), "tip": tip_plate(), "deck": deck(), "sensor": sensor_plate(),
            "leg": leg(), "camera": camera_plate()}


def arm_unit(P, spoke, arc):
    """The front-left (+X, +Y) arm, deployed. Returns {kind: [(name, solid)]}."""
    side = P["side"].solid()
    u = {"cf": [("side_in", yz(side, AX - SIDE_C - T3 / 2)), ("side_out", yz(side, AX + SIDE_C - T3 / 2)),
                ("motor_plate", P["motor"].solid().translate(V(AX, MOTOR_XY, TOP_Z))),
                ("tip_plate", P["tip"].solid().translate(V(AX, MOTOR_XY, TIP_BOT - T2)))],
         "metal": [("hinge_tube", cyl_x(3.5, HY, HZ, AX - ARM_IN / 2, AX + ARM_IN / 2)),
                   ("lock_tube", cyl_x(3.0, HY + LOCK_DY, HZ + LOCK_DZ, AX - ARM_IN / 2, AX + ARM_IN / 2))]
         + [(f"arm_standoff{i}", cyl_x(STANDOFF_R, y, z, AX - ARM_IN / 2, AX + ARM_IN / 2)) for i, (y, z) in enumerate(arm_spacers())]
         + [(f"clamp{i}", cyl(STANDOFF_R, TIP_BOT, TOP_Z, AX + x, MOTOR_XY + y)) for i, (x, y) in enumerate(clamp_points())],
         "motor": [("motor", cyl(MOTOR_R, PAD_TOP, PAD_TOP + MOTOR_H, AX, MOTOR_XY))],
         "guard": guard_pieces(spoke, arc),
         "prop": [("prop", prop_blades())]}
    return u


def body_parts(P):
    c = P["cheek"].solid()
    cf = [("top_plate", P["top"].solid().translate(V(0, 0, TOP_Z))), ("bottom_plate", P["bottom"].solid()),
          ("lidar_deck", P["deck"].solid().translate(V(0, 0, DECK_Z))),
          ("sensor_plate", P["sensor"].solid().translate(V(0, 0, SENSOR_Z0))),
          ("camera_plate", yz(P["camera"].solid(), CAM_X - T2 / 2))]
    metal = []
    for sx in (1, -1):
        e = "front" if sx > 0 else "rear"
        for k, xc in enumerate(CHEEK_XS):
            cf.append((f"cheek_{e}{k}", yz(c, sx * xc - T3 / 2)))
        x0, x1 = sorted((sx * (AX - CHEEK_GAP / 2), sx * (AX + CHEEK_GAP / 2)))
        metal += [(f"cheek_spacer_{e}{i}", cyl_x(3.0, y, z, x0, x1)) for i, (y, z) in enumerate(CHEEK_SPACERS)]
        metal += [(f"tab_lock_{e}{s}", cyl_x(STANDOFF_R, s * TAB_LOCK_Y, TAB_LOCK_Z, x0, x1)) for s in (1, -1)]
        cf.append((f"leg_{e}", yz(P["leg"].solid(), sx * LEG_X - T3 / 2)))
    metal += [(f"spine_post{i}", cyl(STANDOFF_R, Z_BOT1, TOP_Z, x, y)) for i, (x, y) in enumerate(SPINE_POSTS)]
    metal += [(f"cage_post{i}", cyl(STANDOFF_R, SENSOR_Z1, 0.0, x, y)) for i, (x, y) in enumerate(CAGE_POSTS)]
    metal += [(f"deck_post{i}", cyl(STANDOFF_R, TOP_Z1, DECK_Z, x, y)) for i, (x, y) in enumerate(DECK_POSTS)]
    return cf, metal


def electronics():
    d1 = DECK_Z + T2
    lidar = fuse(bx(-40, 40, -22, 22, d1, d1 + 15), cyl(30.25, d1 + 15, d1 + 50))
    return [("pixhawk", bx(-40.75, 40.75, -25, 25, Z_BOT1 + 4, Z_BOT1 + 4 + 15.5)),
            ("pi4b", bx(-42.5, 42.5, -28, 28, DECK_Z - 6 - 22, DECK_Z - 6)),
            ("ydlidar_x2", lidar),
            ("battery", bx(-BATTERY[0] / 2, BATTERY[0] / 2, -BATTERY[1] / 2, BATTERY[1] / 2, SENSOR_Z1, SENSOR_Z1 + BATTERY[2])),
            ("px4flow", bx(FLOW_X - 22.75, FLOW_X + 22.75, -17.5, 17.5, SENSOR_Z0 - 21, SENSOR_Z0)),
            ("tfmini_s", bx(TF_X - 7.5, TF_X + 7.5, -21, 21, SENSOR_Z0 - 16, SENSOR_Z0)),
            ("camera", fuse(bx(CAM_X + T2 / 2 + 3, CAM_X + T2 / 2 + 4, -12.5, 12.5, 4, 27),
                            cyl_x(4.0, 0, 19.0, CAM_X + T2 / 2 + 4, CAM_X + T2 / 2 + 9)))]


def place_unit(unit, corner, deg):
    return {k: [(n, to_corner(fold(s, deg), corner)) for n, s in v] for k, v in unit.items()}


# =============================================================================
# OUTPUT HELPERS
# =============================================================================
def tri_mesh(shape, tol=0.08):
    verts, tris = shape.tessellate(tol, 0.3)
    return trimesh.Trimesh(np.array([[v.x, v.y, v.z] for v in verts]), np.array(tris), process=True)


def best_turn(shape):
    limit = BED - 2 * BED_MARGIN
    best = None
    for deg in [0, 45] + list(range(5, 90, 5)):
        bb = rot_z(shape, deg).BoundingBox()
        span = max(bb.xlen, bb.ylen)
        if span <= limit:
            return deg
        if best is None or span < best[0]:
            best = (span, deg)
    return best[1]


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


def nest(parts, thickness):
    """Shelf-pack every copy of every part of one thickness onto a SHEET_W wide sheet."""
    items = []
    for p in parts:
        if p.t != thickness:
            continue
        b = p.outline.bounds
        w, h = b[2] - b[0], b[3] - b[1]
        turn = 90 if (h > w and h <= SHEET_W) else 0
        for i in range(p.qty):
            items.append((p, turn))
    items.sort(key=lambda it: -(it[0].outline.bounds[2] - it[0].outline.bounds[0] if it[1] else it[0].outline.bounds[3] - it[0].outline.bounds[1]))
    placed, x, y, row_h, gap = [], 0.0, 0.0, 0.0, 6.0
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
    k, m = 2.5, 30
    W, H = int(SHEET_W * k + 2 * m), int(length * k + 2 * m + 40)
    img = Image.new("RGB", (W, H), (246, 246, 248))
    dr = ImageDraw.Draw(img)
    dr.rectangle([m, m + 40, m + SHEET_W * k, m + 40 + length * k], outline=(150, 150, 160), width=2)

    def px(pts):
        return [(m + x * k, m + 40 + (length - y) * k) for x, y in pts]
    for p, place in placed:
        dr.polygon(px(place(p.outline).exterior.coords), fill=(52, 55, 62), outline=(20, 20, 24))
        for c in p.cutouts:
            dr.polygon(px(place(c).exterior.coords), fill=(246, 246, 248))
        for x, y, d in p.holes:
            q = place(Point(x, y))
            (cx, cy), = px([(q.x, q.y)])
            dr.ellipse([cx - d / 2 * k, cy - d / 2 * k, cx + d / 2 * k, cy + d / 2 * k], fill=(246, 246, 248))
    dr.text((m, 14), f"{thickness:.0f} mm carbon: {SHEET_W:.0f} x {length:.0f} mm used", fill=(30, 30, 30), font_size=24)
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
    # Outlines where the part changes or the depth jumps.
    edge = np.zeros((H, W), bool)
    for ax in (0, 1):
        a_id, b_id = idb, np.roll(idb, 1, axis=ax)
        a_d, b_d = depth, np.roll(depth, 1, axis=ax)
        jump = (a_id != b_id) | (np.abs(np.nan_to_num(a_d - b_d, posinf=1e9, neginf=1e9)) > 4.0 / 1.0)
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
    for sub in ("dxf", "step", "stl", "renders"):
        (OUT / sub).mkdir(parents=True, exist_ok=True)
    print(f"Prop {PROP_IN:.0f} in -> motors at +/-{MOTOR_XY:.1f} mm, wheelbase {WHEELBASE:.1f} mm, arm {MOTOR_XY - HY:.1f} mm hinge to motor")

    P = build_parts()
    parts = list(P.values())
    spoke, arc = make_spoke(), make_arc()

    # ----- carbon parts: DXF for CNC, STEP for SOLIDWORKS, web check
    print("\nCARBON PARTS (CNC from sheet; DXF in mm, STEP flat)")
    cf_g = 0.0
    for p in parts:
        s = p.solid()
        doc = dxf_doc()
        dxf_part(doc.modelspace(), p, label=f"{p.name.upper()}  x{p.qty}  {p.t:.0f} mm carbon")
        doc.saveas(OUT / "dxf" / f"{p.name}_{p.t:.0f}mm_x{p.qty}.dxf")
        s.exportStep(str(OUT / "step" / f"{p.name}_{p.t:.0f}mm_x{p.qty}.step"))
        web, inside = p.web()
        b = p.outline.bounds
        g = p.grams()
        cf_g += g * p.qty
        rpt.check(f"{p.name} x{p.qty}: {b[2] - b[0]:.0f} x {b[3] - b[1]:.0f} x {p.t:.0f} mm, {g:.1f} g each",
                  s.isValid() and len(s.Solids()) == 1 and inside and web >= MIN_WEB - 1e-6,
                  f"thinnest web {web:.1f} mm" + ("" if inside else ", FEATURE OUTSIDE PART"))
    for t in (T3, T2):
        placed, length = nest(parts, t)
        doc = dxf_doc()
        for p, place in placed:
            dxf_part(doc.modelspace(), p, place, p.name)
        doc.saveas(OUT / "dxf" / f"CUT_SHEET_{t:.0f}mm.dxf")
        sheet_png(placed, length, t, OUT / "renders" / f"cut_sheet_{t:.0f}mm.png")
        print(f"  cut sheet {t:.0f} mm: {len(placed)} pieces on {SHEET_W:.0f} x {length:.0f} mm")

    # ----- printed guards
    print("\nPRINTED GUARD PARTS (PETG, Ender 3 V3 SE)")
    guard_g = 0.0
    limit = BED - 2 * BED_MARGIN
    for name, part, qty in (("guard_arc_x16", arc, 16), ("guard_spoke_x16", spoke, 16)):
        p = rot_z(part, best_turn(part))
        bb = p.BoundingBox()
        p = p.translate(V(-bb.center.x, -bb.center.y, -bb.zmin))
        bb = p.BoundingBox()
        p.exportStep(str(OUT / "step" / f"{name}.step"))
        mesh = tri_mesh(p)
        mesh.export(OUT / "stl" / f"{name}.stl")
        g = p.Volume() / 1000 * PETG
        guard_g += g * qty
        rpt.check(f"{name}: {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm, {g:.0f} g each",
                  bb.xlen <= limit and bb.ylen <= limit and bb.zlen <= BED_H and mesh.is_watertight and p.isValid())

    # ----- assembly
    unit = arm_unit(P, spoke, arc)
    body_cf, body_metal = body_parts(P)
    elec = electronics()
    body = body_cf + body_metal
    deployed = {c: place_unit(unit, c, 0) for c in CORNERS}

    def flat(u, kinds=("cf", "metal", "motor", "guard")):
        return [(n, s) for k in kinds for n, s in u[k]]

    print("\nCOMPETITION GEOMETRY")
    every = [s for _, s in body + elec] + [s for c in CORNERS for _, s in flat(deployed[c])]
    bbs = [s.BoundingBox() for s in every]
    width = max(max(b.xmax for b in bbs) - min(b.xmin for b in bbs), max(b.ymax for b in bbs) - min(b.ymin for b in bbs)) + 2 * HEAD_PROUD
    height = max(b.zmax for b in bbs) - min(b.zmin for b in bbs)
    rpt.check(f"fits the 2 ft launch square ({LAUNCH_BOX:.0f} mm, rule 8.23)", width <= LAUNCH_BOX, f"{width:.0f} mm wide, {LAUNCH_BOX - width:.0f} mm spare")
    worst = 2 * (MOTOR_XY * math.sqrt(2) + GUARD_R + BAND_T + HEAD_PROUD)
    rpt.check(f"fits a {CORRIDOR:.0f} mm corridor (rule 8.22)", worst < CORRIDOR,
              f"square on: {(CORRIDOR - width) / 2:.0f} mm each side; worst 45 deg yaw: {(CORRIDOR - worst) / 2:.0f} mm each side")
    rpt.check("motors on a square (standard PX4 Quad X mixer)", True, f"+/-{MOTOR_XY:.1f} mm")

    print("\nFIT AND CLEARANCE")
    w, where = 0.0, ""
    for i, (na, a) in enumerate(body):
        for nb, b in body[i + 1:]:
            v = overlap(a, b)
            if v > w:
                w, where = v, f"{na} / {nb}"
    rpt.check("body: plates, cheeks, legs and standoffs seat without clashing (tabs in slots)", w < 0.5, f"worst {w:.2f} mm3 {where}")
    arm_list = flat(unit, ("cf", "metal", "motor"))
    w, where = 0.0, ""
    for i, (na, a) in enumerate(arm_list):
        for nb, b in arm_list[i + 1:]:
            v = overlap(a, b)
            if v > w:
                w, where = v, f"{na} / {nb}"
    rpt.check("arm: side plates, motor plate, tip plate and standoffs seat without clashing", w < 0.5, f"worst {w:.2f} mm3 {where}")
    w, where = worst_overlap(elec, body)
    rpt.check("Pixhawk, Pi 4B, X2, battery, sensors and camera clear the frame", w < 0.5, f"worst {w:.2f} mm3 {where}")
    obstacles = body + elec
    for deg in (0, 20, 45, 70, 90):
        w, where = 0.0, ""
        for c in CORNERS:
            arm = flat(place_unit(unit, c, deg))
            v, wh = worst_overlap(arm, obstacles)
            if v > w:
                w, where = v, wh
        for c, o in ((CORNERS[0], CORNERS[1]), (CORNERS[0], CORNERS[2]), (CORNERS[2], CORNERS[3]), (CORNERS[1], CORNERS[3])):
            v, wh = worst_overlap(flat(place_unit(unit, c, deg)), flat(place_unit(unit, o, deg)))
            if v > w:
                w, where = v, wh
        label = {0: "deployed", 90: "folded"}.get(deg, f"swinging, {deg} deg")
        rpt.check(f"{label}: arms, guards and motors clear the body and each other", w < 0.5, f"worst {w:.2f} mm3 {where}")
    top = body_cf[0][1]
    stop = sum(overlap(fold(s, -3), top) for _, s in unit["cf"][:2])
    rpt.check("thrust stop: the arm's top edge bears on the top plate, so it cannot swing up", stop > 1.0, f"{stop:.0f} mm3 contact at 3 deg up")
    lock_pin = cyl_x(1.6, HY + LOCK_DY, HZ + LOCK_DZ, AX - 25, AX + 25)
    held = sum(overlap(fold(s, 4), lock_pin) for _, s in unit["cf"][:2])
    rpt.check("lock bolt in: the arm cannot fold", held > 1.0, f"{held:.0f} mm3 at 4 deg down")
    disc = cyl(PROP_R, PROP_Z - 1.5, PROP_Z + 1.5, AX, MOTOR_XY)
    others = [s for n, s in flat(deployed[(1, 1)]) if n != "motor"] + [s for _, s in body + elec]
    rpt.check(f"{PROP_IN:.0f} in prop clears guard, arm and body", max(overlap(disc, s) for s in others) < 0.5)
    scan = cyl(2000.0, SCAN_Z - 2, SCAN_Z + 2)
    blk = max(overlap(scan, s) for s in [s for c in CORNERS for _, s in flat(deployed[c])] + [s for _, s in body])
    rpt.check("X2 scan plane is clear of guards, motors and frame", blk < 0.5, f"scan at {SCAN_Z:.0f} mm, guard top at {BAND_Z0 + BAND_H:.0f} mm")
    lens = (CAM_X + T2 / 2 + 9, 0.0, 19.0)
    fov = (cq.Workplane("YZ", origin=lens).rect(6, 6).workplane(offset=300)
           .rect(2 * 300 * math.tan(math.radians(31)), 2 * 300 * math.tan(math.radians(24))).loft().val())
    cam_blk = max(overlap(fov, s) for s in [s for c in CORNERS for _, s in flat(deployed[c])] + [s for _, s in body])
    rpt.check("front camera (62 x 48 deg view) sees past guards, arms and legs (rule 8.27 live feed)", cam_blk < 0.5, f"{cam_blk:.1f} mm3 in view")
    ground = min(b.zmin for b in [s.BoundingBox() for _, s in body])
    low = SENSOR_Z0 - 21.0
    rpt.check("downward sensors stay above the ground when landed", low > ground + 5, f"{low - ground:.0f} mm clear")
    views = [cq.Solid.makeCone(6.0, 6.0 + 300 * math.tan(math.radians(1)), 300, V(TF_X, 0, SENSOR_Z0 - 16), V(0, 0, -1)),
             cq.Solid.makeCone(8.0, 8.0 + 300 * math.tan(math.radians(12)), 300, V(FLOW_X, 0, SENSOR_Z0 - 21), V(0, 0, -1))]
    legs = [s for n, s in body if n.startswith("leg")]
    rpt.check("TFmini-S and PX4Flow see past the legs", max(overlap(v, l) for v in views for l in legs) < 0.5)

    print("\nBOLT LINES (a 2.4 mm pin passes every hole without touching carbon)")
    cf_all = body_cf + [(n, s) for c in CORNERS for n, s in place_unit(unit, c, 0)["cf"]]

    def clear(pins):
        return max(overlap(p, s) for p in pins for _, s in cf_all) < 1e-6

    pins = {
        "hinge and lock bolts through cheeks and arm": [to_corner(cyl_x(1.2, y, z, AX - 25, AX + 25), c) for c in CORNERS
                                                       for y, z in ((HY, HZ), (HY + LOCK_DY, HZ + LOCK_DZ))],
        "cheek spacers and tab locks": [cyl_x(1.2, y, z, sx * AX - 25, sx * AX + 25) for sx in (1, -1)
                                        for y, z in CHEEK_SPACERS + [(s * TAB_LOCK_Y, TAB_LOCK_Z) for s in (1, -1)]],
        "spine standoffs through both plates": [cyl(1.2, -1, TOP_Z1 + 1, x, y) for x, y in SPINE_POSTS],
        "cage standoffs through bottom and sensor plates": [cyl(1.2, SENSOR_Z0 - 1, Z_BOT1 + 1, x, y) for x, y in CAGE_POSTS],
        "deck posts through top plate and deck": [cyl(1.2, TOP_Z - 1, DECK_Z + T2 + 1, x, y) for x, y in DECK_POSTS],
        "arm standoffs through both side plates": [to_corner(cyl_x(1.2, y, z, AX - 25, AX + 25), c) for c in CORNERS for y, z in arm_spacers()],
        "motor-plate clamp standoffs": [to_corner(cyl(1.2, TIP_BOT - T2 - 1, PAD_TOP + 1, AX + x, MOTOR_XY + y), c) for c in CORNERS for x, y in clamp_points()],
        "motor screws and guard spoke bolts": [to_corner(cyl(1.2, TOP_Z - 0.5, PAD_TOP + 1, AX + x, MOTOR_XY + y), c) for c in CORNERS for x, y in motor_holes() + spoke_points()],
        "leg cross pins": [cyl_x(1.2, 0.0, SENSOR_Z1 + STANDOFF_R, sx * LEG_X - 5, sx * LEG_X + 5) for sx in (1, -1)],
    }
    for label, pl in pins.items():
        rpt.check(label, clear(pl))
    spk = [to_corner(cyl(1.0, PAD_TOP - 1, PAD_TOP + 4, AX + x, MOTOR_XY + y), c) for c in CORNERS for x, y in spoke_points()]
    rpt.check("guard spokes line up with the motor plate holes", max(overlap(p, s) for p in spk for c in CORNERS for _, s in flat(deployed[c], ("guard",))) < 1e-6)

    print("\nAIRFLOW (share of each prop disc shaded from below)")
    fl = deployed[(1, 1)]
    arm_sh = [s for _, s in fl["cf"] + fl["metal"]]
    body_sh = [s for _, s in body + elec if s.BoundingBox().zmin < PROP_Z]
    guard_sh = [s for _, s in fl["guard"] if "spoke" in _]
    a_b, b_b, g_b = blockage(arm_sh, AX, MOTOR_XY), blockage(body_sh, AX, MOTOR_XY), blockage(guard_sh, AX, MOTOR_XY)
    print(f"  arm {a_b * 100:.1f} %, guard spokes {g_b * 100:.1f} %, body, battery and legs {b_b * 100:.1f} %")
    rpt.check("body, battery and legs shade under 3% (the H layout keeps the body out from under the props)", b_b < 0.03, f"{b_b * 100:.1f} %")
    rpt.check("everything together shades under 20% of each disc", a_b + b_b + g_b < 0.20, f"{(a_b + b_b + g_b) * 100:.1f} %")

    # ----- mass, strength, performance
    standoff_mm = (len(SPINE_POSTS) * GAP + len(CAGE_POSTS) * CAGE + len(DECK_POSTS) * 35
                   + 4 * (2 * ARM_IN + 4 * TIP_H) + 2 * 4 * CHEEK_GAP + 2 * (ARM_IN * 2))
    hardware_g = standoff_mm * 0.052 + 160 * 0.9 + 8 * 2.0
    bought = sum(BOUGHT_G.values())
    auw = cf_g + guard_g + hardware_g + bought
    print("\nMASS AND PERFORMANCE (estimates)")
    print(f"  carbon {cf_g:.0f} g, printed guards {guard_g:.0f} g, standoffs and bolts {hardware_g:.0f} g, other parts {bought:.0f} g")
    print(f"  all-up weight about {auw:.0f} g")
    rpt.check(f"all-up weight under {MAX_AUW_G / 1000:.0f} kg", auw < MAX_AUW_G)
    for label, t in (("low", THRUST_LOW_G), ("high", THRUST_HIGH_G)):
        print(f"  thrust-to-weight, {label} thrust figure ({t:.0f} g per motor): {4 * t / auw:.2f}")
    watts = auw / HOVER_G_PER_W + AVIONICS_W
    print(f"  hover about {watts:.0f} W, {watts / 11.1:.0f} A, {USABLE_MAH / 1000 / (watts / 11.1) * 60:.1f} min")

    print("\nSTRENGTH (hand calculation, full thrust x 2 for gusts and hard manoeuvres)")
    T = THRUST_HIGH_G * 9.81e-3 * 2
    worst_sf = 99.0
    for y, w_slot in ((HY + LOCK_DY + 8.0 + 4.5, 9.0), (CHEEK_Y, 0.0)):
        h = TOP_Z - arm_bottom(y)
        I = 2 * T3 * (h ** 3 - w_slot ** 3) / 12
        sig = T * (MOTOR_XY - y) * (h / 2) / I
        worst_sf = min(worst_sf, CF_FLEX_MPA / sig)
        print(f"  arm at y={y:.0f}: {sig:.1f} MPa bending, safety factor {CF_FLEX_MPA / sig:.0f}")
    h = TOP_Z - arm_bottom(HY + 40)
    I = 2 * T3 * h ** 3 / 12
    L = MOTOR_XY - CHEEK_Y
    print(f"  motor deflection under that load: {T * L ** 3 / (3 * CF_E_MPA * I):.3f} mm")
    u_c = (CHEEK_Y - HY) / 2
    f_stop = T * (MOTOR_XY - HY) / u_c
    bearing = (f_stop - T) / (2 * T3 * 4.0)
    worst_sf = min(worst_sf, CF_BEARING_MPA / bearing)
    print(f"  stop contact {f_stop:.0f} N; hinge bolt bearing {bearing:.1f} MPa, safety factor {CF_BEARING_MPA / bearing:.0f}")
    m_end = 2 * T * (AX - 106.0)
    sig_end = 6 * m_end / (2 * 14.0 * T3 ** 2)
    worst_sf = min(worst_sf, CF_FLEX_MPA / sig_end)
    print(f"  top plate between last standoff and cheeks: {sig_end:.0f} MPa, safety factor {CF_FLEX_MPA / sig_end:.0f}")
    rpt.check("every hand-calculated safety factor is at least 3", worst_sf >= 3.0, f"lowest {worst_sf:.0f}")

    folded = {c: place_unit(unit, c, 90) for c in CORNERS}
    fb = [s.BoundingBox() for s in [s for c in CORNERS for _, s in flat(folded[c])] + [s for _, s in body + elec]]
    fsize = [max(b.xmax for b in fb) - min(b.xmin for b in fb), max(b.ymax for b in fb) - min(b.ymin for b in fb),
             max(b.zmax for b in fb) - min(b.zmin for b in fb)]
    print(f"\nSIZE  deployed {width:.0f} x {width:.0f} x {height:.0f} mm; folded {fsize[0]:.0f} x {fsize[1]:.0f} x {fsize[2]:.0f} mm")

    # ----- assembly STEP
    asm = [s for _, s in body] + [s for c in CORNERS for _, s in flat(deployed[c], ("cf", "metal", "guard"))]
    cq.Compound.makeCompound(asm).exportStep(str(OUT / "step" / "assembly_deployed.step"))
    asm_f = [s for _, s in body] + [s for c in CORNERS for _, s in flat(folded[c], ("cf", "metal", "guard"))]
    cq.Compound.makeCompound(asm_f).exportStep(str(OUT / "step" / "assembly_folded.step"))

    # ----- renders (set V5_NO_RENDER=1 to skip while editing)
    import os
    if os.environ.get("V5_NO_RENDER"):
        print(f"\n{'ALL CHECKS PASSED' if rpt.failed == 0 else str(rpt.failed) + ' CHECK(S) FAILED'}")
        return rpt.failed
    print("\nRENDERING")
    CFC, AL, GUARD, MOT, PROP, BAT, PCB, LID = ((58, 62, 70), (196, 200, 208), (236, 176, 52), (120, 126, 138),
                                               (40, 40, 44), (52, 104, 196), (46, 140, 88), (90, 96, 110))

    def scene(degs, with_props=True, with_elec=True):
        units_ = {c: place_unit(unit, c, d) for c, d in zip(CORNERS, degs)}
        g = [([s for _, s in body_cf], CFC), ([s for _, s in body_metal], AL)]
        g.append(([s for c in CORNERS for _, s in units_[c]["cf"]], CFC))
        g.append(([s for c in CORNERS for _, s in units_[c]["metal"]], AL))
        g.append(([s for c in CORNERS for _, s in units_[c]["motor"]], MOT))
        g.append(([s for c in CORNERS for _, s in units_[c]["guard"]], GUARD))
        if with_props:
            g.append(([s for c in CORNERS for _, s in units_[c]["prop"]], PROP))
        if with_elec:
            e = dict(elec)
            g += [([e["battery"]], BAT), ([e["pixhawk"], e["pi4b"], e["px4flow"], e["tfmini_s"], e["camera"]], PCB), ([e["ydlidar_x2"]], LID)]
        return g

    sub = f"Carbon plate H-frame, {WHEELBASE:.0f} mm wheelbase, {width:.0f} mm wide, about {auw:.0f} g all-up"
    render(scene([0] * 4), OUT / "renders" / "deployed.png", 26, 32, "Version 5: carbon fibre, arms deployed", sub)
    render(scene([90] * 4, with_props=False), OUT / "renders" / "folded.png", 16, 32, "Arms folded down for transport",
           f"Folded {fsize[0]:.0f} x {fsize[1]:.0f} x {fsize[2]:.0f} mm; remove the lock bolt and swing each arm down")
    render(scene([0, 30, 60, 90], with_props=False), OUT / "renders" / "fold_sequence.png", 22, 58, "Fold sequence: 0, 30, 60 and 90 deg",
           "Each arm swings down about its hinge bolt; nothing touches at any angle")
    sq = LAUNCH_BOX / 2
    square = cut(bx(-sq, sq, -sq, sq, ground - 3, ground - 1), bx(-sq + 6, sq - 6, -sq + 6, sq - 6, ground - 4, ground))
    render(scene([0] * 4) + [([square], (205, 60, 60))], OUT / "renders" / "top.png", 90, -90, "Top view inside the 2 ft launch square",
           f"{width:.0f} mm wide in a {LAUNCH_BOX:.0f} mm square (red); front is to the right")
    walls = [bx(-60, 60, *sorted((s * CORRIDOR / 2, s * (CORRIDOR / 2 + 12))), ground, 260) for s in (1, -1)]
    floor = bx(-60, 60, -CORRIDOR / 2, CORRIDOR / 2, ground - 4, ground)
    render(scene([0] * 4) + [(walls, (210, 214, 222)), ([floor], (225, 228, 234))], OUT / "renders" / "front_corridor.png", 0, 0,
           "Front view in a 1 m corridor", f"{(CORRIDOR - width) / 2:.0f} mm to each wall flying straight; {(CORRIDOR - worst) / 2:.0f} mm at the worst 45 deg yaw")
    near = lambda s: s.BoundingBox().xmin > 100 and s.BoundingBox().ymin > -45
    det = [([s for _, s in body_cf if near(s)] + [s for _, s in deployed[(1, 1)]["cf"]], CFC),
           ([s for _, s in body_metal if near(s)] + [s for _, s in deployed[(1, 1)]["metal"]], AL),
           ([s for _, s in deployed[(1, 1)]["motor"]], MOT)]
    render(det, OUT / "renders" / "arm_hinge.png", 24, 62, "Front-left arm and hinge",
           "Two side plates, cheek clevis, hinge and lock bolts, motor plate clamped by four standoffs")
    exploded = [([P["top"].solid().translate(V(0, 0, TOP_Z + 60)), P["bottom"].solid().translate(V(0, 0, -30)),
                  P["deck"].solid().translate(V(0, 0, DECK_Z + 110)), P["sensor"].solid().translate(V(0, 0, SENSOR_Z0 - 90))]
                 + [s.translate(V(0, 0, 0)) for n, s in body_cf if n.startswith("cheek")]
                 + [s.translate(V(0, 0, -60)) for n, s in body_cf if n.startswith("leg")]
                 + [s.translate(V(0, 60, 25)) for _, s in deployed[(1, 1)]["cf"]], CFC)]
    render(exploded, OUT / "renders" / "exploded.png", 24, 35, "Exploded view of the carbon parts",
           "Every joint is a tab in a slot, clamped by standoffs: no glue")
    print("  done")

    print(f"\n{'ALL CHECKS PASSED' if rpt.failed == 0 else str(rpt.failed) + ' CHECK(S) FAILED'}")
    return rpt.failed


if __name__ == "__main__":
    raise SystemExit(main())
