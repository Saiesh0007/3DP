#!/usr/bin/env python3
"""
NIDAR AirMouse frame, version 4: the smallest frame the current parts allow.

Round tube arms (bought aluminium or carbon tube), printed hinges, mounts,
plates, guards and legs. Arms fold downward. Frame size is worked out from the
prop size, so changing PROP_IN resizes everything and re-runs every check.

Install (Python 3.12):  pip install cadquery ezdxf shapely numpy trimesh pillow
Run:                    python airmouse_v4_generator.py
Out:                    output/step, output/stl, output/dxf, output/renders
"""
import math
from pathlib import Path

import cadquery as cq
import ezdxf
import numpy as np
import trimesh
from ezdxf import units
from shapely.geometry import Polygon, box as sbox

# =============================================================================
# PARAMETERS (mm)
# =============================================================================
# --- Rules (NIDAR 2026-27 Rulebook 8.22, 8.23)
LAUNCH_BOX = 609.6
CORRIDOR = 1000.0

# --- Printer (Creality Ender 3 V3 SE)
BED, BED_H, BED_MARGIN = 220.0, 250.0, 2.5

# --- Propulsion: everything below is sized from the prop
PROP_IN = 10.0
GUARD_GAP = 8.0             # prop tip to guard band
BAND_T, BAND_H = 1.2, 16.0
RING_GAP = 4.0              # space between neighbouring guards
HEAD_PROUD = 3.0            # guard bolt heads outside the band
MOTOR_HOLES = (16.0, 19.0)  # 2212 cross pattern, M3. MEASURE YOURS.
MOTOR_R, MOTOR_H = 14.0, 30.0
PROP_ABOVE_PAD = 38.0       # motor mount face to prop plane. MEASURE YOURS.

PROP_R = PROP_IN * 25.4 / 2
GUARD_R = PROP_R + GUARD_GAP                   # inside of the band
# Neighbouring guards sit RING_GAP apart, so each motor is this far out in X and Y:
MOTOR_XY = GUARD_R + BAND_T + RING_GAP / 2
WHEELBASE = 2 * math.sqrt(2) * MOTOR_XY        # diagonal, motor axis to motor axis

# --- Body plates (printed PETG; the DXF can also be cut from 1.5 mm carbon)
PLATE = 150.0
PLATE_T = 3.0
CHAMFER = 5.0
GAP = 24.0                  # between plates: the Pixhawk lives here
TOP_Z = PLATE_T + GAP       # underside of top plate

# --- Arm tube (bought: 6061 aluminium 16 x 1 mm, or carbon 16 x 14 mm)
TUBE_OD, TUBE_ID = 16.0, 14.0
TUBE_BORE = TUBE_OD + 0.3   # printed bore, slip fit

# --- Hinge (folds down about an X-direction axis, like version 2)
HINGE_X, HINGE_Y, HINGE_Z = 55.0, 58.0, 11.0
KNUCKLE_R = 7.5
BLOCK_W, BLOCK_OUT = 26.0, 34.0
EAR_T, EAR_GAP = 5.0, 0.3
EAR_END = 12.0              # ear length past the hinge axis
LOCK_DY, LOCK_DZ = 6.0, 9.0
STOP_T = 10.0              # thick enough that its bolt holes keep a 3 mm web to the plate cut-out
TUBE_SWEEP_Y = HINGE_Y + 3.0   # bottom plate is cut away beyond this, outboard of the block
TUBE_Z = PLATE_T + GAP / 2  # tube centre height
SOCKET_Y = HINGE_Y + 14.0   # where the tube end sits inside the block
BLOCK_TOP = TOP_Z - 0.4

# --- Motor mount
MOUNT_BACK, MOUNT_FRONT = 40.0, 18.0     # block length behind and ahead of the motor axis
MOUNT_W = TUBE_OD + 8.0
TUBE_SHORT_OF_MOTOR = 12.0  # tube ends this far behind the motor axis
PAD_T = 4.0
PAD_TOP = TUBE_Z + TUBE_OD / 2 + PAD_T
SPOKE_R, SPOKE_T = 24.0, 5.0             # spoke bolt holes on the motor pad

# --- Electronics stack
DECK_Z = TOP_Z + PLATE_T + 35.0          # bought M3 x 35 standoffs
DECK_T = 3.0
DECK_HALF = (46.0, 38.0)
DECK_CHAMFER = 10.0
DECK_POSTS = [(sx * 41.0, sy * 33.0) for sx in (1, -1) for sy in (1, -1)]
PI_HOLES = [(sx * 29.0, sy * 24.5) for sx in (1, -1) for sy in (1, -1)]
X2_HOLES = [(-35.3, 0.0), (36.0, 18.0), (36.0, -18.0)]
X2_SCAN_ABOVE_BASE = 42.0               # scaled from the YDLidar X2 drawing
SPACERS = [(0.0, 60.0), (0.0, -60.0)]   # bought M3 x 24 standoffs between plates

# --- Battery tray and sensors (under the bottom plate)
BATTERY = (165.0, 50.0, 40.0)            # upper limit; pack not measured
TRAY_LEN, TRAY_WALL, TRAY_FLOOR = 170.0, 3.0, 3.0
TRAY_IN = BATTERY[1] / 2 + 1.0
TRAY_Z0 = -(BATTERY[2] + 4.0) - TRAY_FLOOR   # underside of the tray floor
TRAY_BOSS_R = 4.5
TRAY_BOLTS = [(sx * 60.0, sy * (TRAY_IN + TRAY_BOSS_R)) for sx in (1, -1) for sy in (1, -1)]
FLOW_X, TF_X = 45.0, -45.0
PX4FLOW = (45.5, 35.0)
TF_SPACING = 36.0

# --- Legs
LEG_ROOT = (68.0, 18.0)
LEG_DIR = 40.0
LEG_REACH, LEG_DROP = 62.0, 40.0
LEG_T, FLANGE_W, FLANGE_T = 6.0, 16.0, 4.0

# --- Mass and thrust (assumptions marked in README)
DENSITY = 1.27              # PETG
TUBE_DENSITY = 2.70         # aluminium; carbon is about 1.55
BOUGHT_G = {"motors": 4 * 57, "ESCs": 4 * 35, "props": 4 * 13, "battery": 471, "Pixhawk": 38,
            "Pi 4B + cooling + regulator": 76, "YDLidar X2": 200, "TFmini-S": 5, "PX4Flow": 20,
            "camera": 10, "power module, wiring": 100, "receiver + telemetry": 40, "bolts, nuts, standoffs": 70}
THRUST_LOW_G, THRUST_HIGH_G = 642.0, 850.0
HOVER_G_PER_W, USABLE_MAH, AVIONICS_W = 7.0, 4000.0, 20.0

M3, M4, M25, HEAD_D = 3.4, 4.3, 2.8, 6.5
OUT = Path(__file__).resolve().parent / "output"
CORNERS = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
PROP_Z = PAD_TOP + PROP_ABOVE_PAD
BAND_Z0 = PROP_Z - BAND_H / 2
SCAN_Z = DECK_Z + DECK_T + X2_SCAN_ABOVE_BASE


# =============================================================================
# GEOMETRY HELPERS (CadQuery solids)
# =============================================================================
V = cq.Vector


def bx(x0, x1, y0, y1, z0, z1):
    return cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, V(x0, y0, z0))


def cyl(r, z0, z1, x=0.0, y=0.0):
    return cq.Solid.makeCylinder(r, z1 - z0, V(x, y, z0), V(0, 0, 1))


def cyl_along(r, start, direction, length):
    return cq.Solid.makeCylinder(r, length, V(*start), V(*direction))


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
    """Apply a 3 x 4 rigid transform (rows of [a, b, c, t]); rotations only, no scaling."""
    from OCP.BRepBuilderAPI import BRepBuilderAPI_Transform
    from OCP.gp import gp_Trsf
    t = gp_Trsf()
    t.SetValues(*[float(v) for r in rows for v in r])
    return cq.Shape.cast(BRepBuilderAPI_Transform(shape.wrapped, t, True).Shape())


def rot_z(shape, deg):
    return shape.rotate(V(0, 0, 0), V(0, 0, 1), deg)


def teardrop(a, b, d):
    """Hole outline in a plane, point toward +b, so it prints without support."""
    r = d / 2
    pts = [(a + r * math.cos(math.radians(t)), b + r * math.sin(math.radians(t))) for t in range(-225, 46, 15)]
    return pts + [(a, b + r * math.sqrt(2))]


def bore_x(y, z, d, x0, x1):
    """Teardrop bore along X, point toward +Z."""
    return remap(prism(teardrop(y, z, d), x0, x1), [[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]])


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


# =============================================================================
# ARM: tube, root block and motor mount, built for the front-left corner
# =============================================================================
TUBE_DIR = math.atan2(MOTOR_XY - SOCKET_Y, MOTOR_XY - HINGE_X)
TUBE_TO_MOTOR = math.hypot(MOTOR_XY - SOCKET_Y, MOTOR_XY - HINGE_X)
TUBE_LEN = TUBE_TO_MOTOR - TUBE_SHORT_OF_MOTOR
UX, UY = math.cos(TUBE_DIR), math.sin(TUBE_DIR)


def along(u, side=0.0):
    """Point on the arm line, u from the tube start, side to the left of it."""
    return (HINGE_X + u * UX - side * UY, SOCKET_Y + u * UY + side * UX)


def make_tube():
    s = along(0.0)
    tube = cyl_along(TUBE_OD / 2, (s[0], s[1], TUBE_Z), (UX, UY, 0), TUBE_LEN)
    return cut(tube, cyl_along(TUBE_ID / 2, (s[0], s[1], TUBE_Z), (UX, UY, 0), TUBE_LEN))


def cross_bolt(u):
    x, y = along(u)
    return cyl(M3 / 2, -5, 60, x, y)


def make_root():
    y0, y1 = HINGE_Y - KNUCKLE_R, HINGE_Y + BLOCK_OUT
    # Behind the hinge (y < HINGE_Y) the block swings upward as the arm folds, so it is
    # trimmed to a circle that stays 0.5 mm under the top plate.
    swing_r = TOP_Z - HINGE_Z - 0.5
    rear = prism([(y0, HINGE_Z), (HINGE_Y, HINGE_Z), (HINGE_Y, BLOCK_TOP), (y0, BLOCK_TOP)], 0, BLOCK_W).intersect(
        cq.Workplane("XY").center(HINGE_Y, HINGE_Z).circle(swing_r).extrude(BLOCK_W).val())
    profile = (cq.Workplane("XY").center(HINGE_Y, HINGE_Z).circle(KNUCKLE_R).extrude(BLOCK_W).val()
               .fuse(rear)
               .fuse(prism([(HINGE_Y, HINGE_Z), (y1, HINGE_Z), (y1, BLOCK_TOP), (HINGE_Y, BLOCK_TOP)], 0, BLOCK_W))
               .fuse(prism([(HINGE_Y, HINGE_Z - KNUCKLE_R), (y1, HINGE_Z - KNUCKLE_R), (y1, HINGE_Z), (HINGE_Y, HINGE_Z)], 0, BLOCK_W)))
    block = remap(profile, [[0, 0, 1, HINGE_X - BLOCK_W / 2], [1, 0, 0, 0], [0, 1, 0, 0]])
    s = along(0.0)
    bore = cyl_along(TUBE_BORE / 2, (s[0], s[1], TUBE_Z), (UX, UY, 0), 80)
    return cut(block, bore, bore_x(HINGE_Y, HINGE_Z, M4, HINGE_X - 20, HINGE_X + 20),
               bore_x(HINGE_Y + LOCK_DY, HINGE_Z + LOCK_DZ, M4, HINGE_X - 20, HINGE_X + 20), cross_bolt(8.0))


def spoke_points():
    """Guard spoke bolt holes around the motor axis, in arm-aligned coordinates."""
    pts = []
    for k in range(4):
        a = math.radians(45 + 90 * k)
        for t in (-SPOKE_T, SPOKE_T):
            pts.append((SPOKE_R * math.cos(a) - t * math.sin(a), SPOKE_R * math.sin(a) + t * math.cos(a)))
    return pts


def to_motor_frame(shape):
    """Arm-aligned frame (x along the arm, origin at the motor axis) to assembly."""
    return rot_z(shape, math.degrees(TUBE_DIR)).translate(V(MOTOR_XY, MOTOR_XY, 0))


def make_mount():
    z0 = TUBE_Z - TUBE_OD / 2 - 4.0
    block = bx(-MOUNT_BACK, MOUNT_FRONT, -MOUNT_W / 2, MOUNT_W / 2, z0, PAD_TOP)
    pad = cyl(18.0, PAD_TOP - PAD_T, PAD_TOP)
    for k in range(4):
        lobe = bx(14.0, SPOKE_R + 7.0, -SPOKE_T - 5.0, SPOKE_T + 5.0, PAD_TOP - PAD_T, PAD_TOP)
        pad = pad.fuse(rot_z(lobe, 45 + 90 * k))
    mount = fuse(block, pad)
    tube_end = -TUBE_SHORT_OF_MOTOR
    cutters = [remap(prism(teardrop(0.0, TUBE_Z, TUBE_BORE), -MOUNT_BACK - 1, tube_end),
                     [[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]]),
               cyl(M3 / 2, z0 - 1, PAD_TOP + 1, -26.0, 0.0),        # cross bolt through the tube
               cyl(5.0, z0 - 1, PAD_TOP + 1)]                        # shaft and circlip clearance
    for k, span in enumerate(MOTOR_HOLES):
        for s in (1, -1):
            x, y = (s * span / 2, 0.0) if k == 0 else (0.0, s * span / 2)
            # Rotate the cross 45 deg so no bolt sits on the arm line.
            c, sn = math.cos(math.radians(45)), math.sin(math.radians(45))
            x, y = x * c - y * sn, x * sn + y * c
            cutters.append(cyl(M3 / 2, PAD_TOP - PAD_T - 1, PAD_TOP + 1, x, y))
            cutters.append(cyl(HEAD_D / 2, z0 - 1, PAD_TOP - PAD_T, x, y))
    for x, y in spoke_points():
        cutters.append(cyl(M3 / 2, PAD_TOP - PAD_T - 1, PAD_TOP + 1, x, y))
    # Wire exit beside the motor.
    cutters.append(bx(-22.0, -14.0, -6.0, 6.0, TUBE_Z, PAD_TOP + 1))
    return cut(mount, *cutters)


def make_spoke():
    """Print frame: U radial, V height, W sideways (web on the bed, flanges rising)."""
    r0, r1 = 18.0, 31.0
    t0, t1 = GUARD_R - 3.0, GUARD_R
    low, high = BAND_Z0, BAND_Z0 + BAND_H
    outer = [(r0, PAD_TOP), (r1, PAD_TOP), (t0, low), (t1, low), (t1, high), (t0, high),
             (t0, PROP_Z - 6.0), (r0, PAD_TOP + 12.0)]
    web = prism(outer, 0, 2.4)
    window = prism([(r1 + 8, PAD_TOP + 5.0), (t0 - 14, low + 1.0), (r1 + 8, PAD_TOP + 10.0)], -1, 4)
    spoke = fuse(cut(web, window), bx(r0, r1, PAD_TOP, PAD_TOP + 3.0, 0, 20.0), bx(t0, t1, low, high, 0, 20.0))
    holes = []
    for w in (10.0 - SPOKE_T, 10.0 + SPOKE_T):      # root bolts, vertical in the assembly
        holes.append(remap(prism(teardrop(SPOKE_R, w, M3), 0, 8), [[1, 0, 0, 0], [0, 0, -1, PAD_TOP + 6], [0, 1, 0, 0]]))
    for w in (3.0, 17.0):                            # band bolts, radial in the assembly
        holes.append(remap(prism(teardrop(PROP_Z, w, M3), t0 - 1, t1 + 1), [[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]]))
    return cut(spoke, *holes)


def spoke_to_assembly(spoke):
    return remap(spoke, [[1, 0, 0, 0], [0, 0, -1, 10.0], [0, 1, 0, 0]])


def make_arc():
    """A quarter of the guard ring, printed standing on its rim."""
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
    pieces = []
    placed = spoke_to_assembly(spoke)
    for k in range(4):
        a = 45 + 90 * k
        pieces.append(rot_z(placed, a))
        pieces.append(rot_z(arc.translate(V(0, 0, BAND_Z0)), a + 45))
    return pieces


# =============================================================================
# BODY
# =============================================================================
def hinge_points(corner):
    sx, sy = corner
    return [(sx * (HINGE_X + dx), sy * (HINGE_Y - KNUCKLE_R - 0.2 - STOP_T / 2)) for dx in (-6.0, 6.0)]


def ear_outer():
    return BLOCK_W / 2 + EAR_GAP + EAR_T


def make_hinge_base():
    """Clevis for the front-left corner: two ears and a stop wall. Also spaces the plates."""
    xo = ear_outer()
    stop_y1 = HINGE_Y - KNUCKLE_R - 0.2
    parts = [bx(HINGE_X - xo, HINGE_X + xo, stop_y1 - STOP_T, stop_y1, PLATE_T, TOP_Z)]
    for s in (1, -1):
        a, b = sorted((HINGE_X + s * (BLOCK_W / 2 + EAR_GAP), HINGE_X + s * xo))
        parts.append(bx(a, b, stop_y1 - STOP_T, HINGE_Y + EAR_END, PLATE_T, TOP_Z))
    base = fuse(*parts)
    holes = [cyl(M3 / 2, 0, TOP_Z + 1, x, y) for x, y in hinge_points((1, 1))]
    holes += [bore_x(HINGE_Y, HINGE_Z, M4, HINGE_X - 30, HINGE_X + 30),
              bore_x(HINGE_Y + LOCK_DY, HINGE_Z + LOCK_DZ, M4, HINGE_X - 30, HINGE_X + 30)]
    return cut(base, *holes)


def plate_outline(notched):
    h, c = PLATE / 2, CHAMFER
    poly = Polygon([(h - c, -h), (h, -h + c), (h, h - c), (h - c, h), (-h + c, h), (-h, h - c), (-h, -h + c), (-h + c, -h)])
    if notched:   # the arm root swings down through these
        for sx, sy in CORNERS:
            x0, x1 = sorted((sx * (HINGE_X - BLOCK_W / 2 - 0.5), sx * (HINGE_X + BLOCK_W / 2 + 0.5)))
            y0, y1 = sorted((sy * (HINGE_Y - KNUCKLE_R - 0.2), sy * 200))
            poly = poly.difference(sbox(x0, y0, x1, y1))
            # The tube leaves the block toward the plate corner and sweeps through here.
            x0, x1 = sorted((sx * (HINGE_X + BLOCK_W / 2), sx * 200))
            y0, y1 = sorted((sy * TUBE_SWEEP_Y, sy * 200))
            poly = poly.difference(sbox(x0, y0, x1, y1))
    return list(poly.exterior.coords)[:-1]


def plate_features(which):
    """(holes, slots) for the plate: holes are (x, y, d), slots (cx, cy, length, width, angle)."""
    holes = [(x, y, M3) for c in CORNERS for x, y in hinge_points(c)]
    holes += [(x, y, M3) for x, y in SPACERS]
    slots = []
    if which == "top":
        holes += [(x, y, M3) for x, y in DECK_POSTS]
        slots += [(0.0, sy * 40.0, 50.0, 10.0, 0.0) for sy in (1, -1)]        # wiring from the bay to the Pi
        slots += [(sx * 52.0, 0.0, 40.0, 6.0, 90.0) for sx in (1, -1)]        # Pixhawk zip ties
        slots += [(sx * 25.0, 0.0, 40.0, 14.0, 90.0) for sx in (1, -1)]       # lightening
        slots += [(sx * 66.0, 0.0, 50.0, 10.0, 90.0) for sx in (1, -1)]       # lightening
    else:
        holes += [(x, y, M3) for x, y in TRAY_BOLTS]
        slots += [(sx * 30.0, 0.0, 40.0, 6.0, 90.0) for sx in (1, -1)]        # Pixhawk zip ties
        slots += [(sx * 52.0, 0.0, 30.0, 5.0, 90.0) for sx in (1, -1)]        # battery strap
        slots += [(0.0, 0.0, 30.0, 10.0, 0.0)]                                # sensor wiring
        slots += [(0.0, sy * 45.0, 60.0, 14.0, 0.0) for sy in (1, -1)]        # lightening
        slots += [(sx * 66.0, 0.0, 30.0, 10.0, 90.0) for sx in (1, -1)]       # lightening
    return holes, slots


def slot_solid(cx, cy, length, width, angle, z0, z1):
    return (cq.Workplane("XY").center(cx, cy).slot2D(length, width, angle).extrude(z1 - z0)
            .val().translate(V(0, 0, z0)))


def make_plate(which):
    plate = prism(plate_outline(which == "bottom"), 0, PLATE_T)
    holes, slots = plate_features(which)
    cutters = [cyl(d / 2, -1, PLATE_T + 1, x, y) for x, y, d in holes]
    cutters += [slot_solid(*s, -1, PLATE_T + 1) for s in slots]
    return cut(plate, *cutters)


def make_deck():
    hx, hy = DECK_HALF
    c = DECK_CHAMFER
    outline = [(hx - c, -hy), (hx, -hy + c), (hx, hy - c), (hx - c, hy), (-hx + c, hy), (-hx, hy - c), (-hx, -hy + c), (-hx + c, -hy)]
    deck = prism(outline, 0, DECK_T)
    cutters = [cyl(M3 / 2, -1, DECK_T + 1, x, y) for x, y in DECK_POSTS]
    cutters += [cyl(M25 / 2, -1, DECK_T + 1, x, y) for x, y in PI_HOLES + X2_HOLES]
    cutters += [slot_solid(0.0, sy * 12.0, 40.0, 8.0, 0.0, -1, DECK_T + 1) for sy in (1, -1)]
    return cut(deck, *cutters)


def make_tray():
    """Battery cradle. Printed floor down; the battery slides in from the front or back."""
    hl, w = TRAY_LEN / 2, TRAY_IN + TRAY_WALL
    floor = bx(-hl, hl, -w, w, 0, TRAY_FLOOR)
    walls = [bx(-hl, hl, s * TRAY_IN if s > 0 else -w, w if s > 0 else -TRAY_IN, 0, -TRAY_Z0) for s in (1, -1)]
    bosses = [cyl(TRAY_BOSS_R, 0, -TRAY_Z0, x, y) for x, y in TRAY_BOLTS]
    tray = fuse(floor, *walls, *bosses)
    cutters = [cyl(M3 / 2, -1, -TRAY_Z0 + 1, x, y) for x, y in TRAY_BOLTS]
    # Lightening windows in the walls (45 deg diamonds print without support).
    zm = -TRAY_Z0 / 2 + 1.5
    for xc in (-40.0, 0.0, 40.0):
        d = [(xc + 15, zm), (xc, zm + 15), (xc - 15, zm), (xc, zm - 15)]
        cutters.append(remap(prism(d, -w - 1, w + 1), [[1, 0, 0, 0], [0, 0, 1, 0], [0, -1, 0, 0]]))
    # Sensor fixings in the floor: PX4Flow zip-tie slots and lens window, TFmini-S screws.
    cutters += [slot_solid(FLOW_X + s * 17.0, 0.0, 30.0, 3.0, 90.0, -1, 5) for s in (1, -1)]
    cutters.append(slot_solid(FLOW_X, 0.0, 22.0, 14.0, 0.0, -1, 5))
    cutters += [cyl(1.2, -1, 5, TF_X, s * TF_SPACING / 2) for s in (1, -1)]
    cutters.append(slot_solid(TF_X, 0.0, 20.0, 12.0, 0.0, -1, 5))
    cutters += [cyl(M3 / 2, -1, 5, x, y) for x, y in leg_holes_tray()]
    cutters.append(slot_solid(0.0, 0.0, 44.0, 24.0, 0.0, -1, 5))          # lightening
    return cut(tray, *cutters)


def tray_to_assembly(tray):
    return tray.translate(V(0, 0, TRAY_Z0))


def corner_xy(x, y, corner):
    """Front-left (x, y) moved to a corner the same way to_corner moves solids."""
    sx, sy = corner
    if sx * sy < 0:
        y = -y
    if sx < 0:
        x, y = -x, -y
    return x, y


def leg_holes_tray():
    d = math.radians(LEG_DIR)
    fl = [(LEG_ROOT[0] + off * math.cos(d) - 3.0 * math.sin(d), LEG_ROOT[1] + off * math.sin(d) + 3.0 * math.cos(d))
          for off in (-9.0, 9.0)]
    return [corner_xy(x, y, c) for c in CORNERS for x, y in fl]


def make_leg():
    """Print frame: U outward, V down, W sideways. Web on the bed, flange rising."""
    prof = [(-13, 0), (13, 0), (LEG_REACH + 6, LEG_DROP - 7), (LEG_REACH + 6, LEG_DROP), (LEG_REACH - 8, LEG_DROP), (-13, 12)]
    web = prism(prof, 0, LEG_T)
    window = prism([(14, 7), (LEG_REACH - 10, LEG_DROP - 5.5), (6, 11)], -1, LEG_T + 1)
    leg = fuse(cut(web, window), bx(-13, 13, 0, FLANGE_T, 0, FLANGE_W))
    holes = [remap(prism(teardrop(off, FLANGE_W - 5.0, M3), 0, FLANGE_T + 2), [[1, 0, 0, 0], [0, 0, -1, FLANGE_T + 1], [0, 1, 0, 0]])
             for off in (-9.0, 9.0)]
    return cut(leg, *holes)


def place_leg(leg, sx, sy):
    """Print frame to the front-left corner, then to any corner like the arms."""
    m = remap(leg, [[1, 0, 0, 0], [0, 0, 1, -FLANGE_W + 8.0], [0, -1, 0, 0]])
    m = rot_z(m, LEG_DIR).translate(V(LEG_ROOT[0], LEG_ROOT[1], TRAY_Z0))
    return to_corner(m, (sx, sy))


# =============================================================================
# CORNER PLACEMENT AND FOLDING
# =============================================================================
def fold(shape, deg):
    """Rotate a front-left part about its hinge line: 0 deployed, 90 folded down."""
    return (shape.translate(V(0, -HINGE_Y, -HINGE_Z)).rotate(V(0, 0, 0), V(1, 0, 0), -deg)
            .translate(V(0, HINGE_Y, HINGE_Z)))


def to_corner(shape, corner):
    sx, sy = corner
    if sx * sy < 0:
        shape = shape.mirror("XZ")
    if sx < 0:
        shape = rot_z(shape, 180)
    return shape


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


def write_dxf(path, outline, holes, slots, label):
    doc = ezdxf.new("R2010", setup=True)
    doc.units = units.MM
    for name, color in (("CUT_OUTER", 1), ("CUT_INNER", 5), ("NOTES", 7)):
        doc.layers.add(name, color=color)
    msp = doc.modelspace()
    msp.add_lwpolyline(outline, close=True, dxfattribs={"layer": "CUT_OUTER"})
    for x, y, d in holes:
        msp.add_circle((x, y), d / 2, dxfattribs={"layer": "CUT_INNER"})
    for cx, cy, length, width, ang in slots:
        r, half = width / 2, (length - width) / 2
        ux, uy = math.cos(math.radians(ang)), math.sin(math.radians(ang))
        a, b = (cx + ux * half, cy + uy * half), (cx - ux * half, cy - uy * half)
        at = {"layer": "CUT_INNER"}
        msp.add_line((a[0] - uy * r, a[1] + ux * r), (b[0] - uy * r, b[1] + ux * r), dxfattribs=at)
        msp.add_line((a[0] + uy * r, a[1] - ux * r), (b[0] + uy * r, b[1] - ux * r), dxfattribs=at)
        msp.add_arc(a, r, ang - 90, ang + 90, dxfattribs=at)
        msp.add_arc(b, r, ang + 90, ang + 270, dxfattribs=at)
    msp.add_text(label, height=3, dxfattribs={"layer": "NOTES"}).set_placement((-PLATE / 2, -PLATE / 2 - 8))
    doc.saveas(path)


def render(groups, path, elev, azim, title, size=1200):
    from PIL import Image, ImageDraw
    el, az = math.radians(elev), math.radians(azim)
    fwd = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    right = np.array([-math.sin(az), math.cos(az), 0.0])
    up = np.cross(fwd, right)
    light = fwd * 0.75 - right * 0.35 + up * 0.55
    light /= np.linalg.norm(light)
    meshes = [(tri_mesh(s, 0.4), np.array(c, float)) for s, c in groups]
    allv = np.concatenate([m.vertices for m, _ in meshes])
    u, v = allv @ right, allv @ up
    scale = (size - 120) / max(np.ptp(u), np.ptp(v))
    cu, cv = (u.max() + u.min()) / 2, (v.max() + v.min()) / 2
    depth = np.full((size, size), -np.inf)
    img = np.full((size, size, 3), 246.0)
    for mesh, color in meshes:
        tris = mesh.triangles
        px = (tris @ right - cu) * scale + size / 2
        py = size / 2 - (tris @ up - cv) * scale
        pz = tris @ fwd
        shade = 0.3 + 0.7 * np.abs(mesh.face_normals @ light)
        for x, y, z, s in zip(px, py, pz, shade):
            x0, x1 = max(int(x.min()), 0), min(int(x.max()) + 1, size - 1)
            y0, y1 = max(int(y.min()), 0), min(int(y.max()) + 1, size - 1)
            if x1 < x0 or y1 < y0:
                continue
            d = (y[1] - y[2]) * (x[0] - x[2]) + (x[2] - x[1]) * (y[0] - y[2])
            if abs(d) < 1e-9:
                continue
            gx, gy = np.meshgrid(np.arange(x0, x1 + 1), np.arange(y0, y1 + 1))
            a = ((y[1] - y[2]) * (gx - x[2]) + (x[2] - x[1]) * (gy - y[2])) / d
            b = ((y[2] - y[0]) * (gx - x[2]) + (x[0] - x[2]) * (gy - y[2])) / d
            c = 1 - a - b
            inside = (a >= -1e-6) & (b >= -1e-6) & (c >= -1e-6)
            zz = a * z[0] + b * z[1] + c * z[2]
            win = depth[y0:y1 + 1, x0:x1 + 1]
            mask = inside & (zz > win)
            win[mask] = zz[mask]
            img[y0:y1 + 1, x0:x1 + 1][mask] = color * s
    out = Image.fromarray(img.astype(np.uint8))
    ImageDraw.Draw(out).text((28, 22), title, fill=(30, 30, 30), font_size=28)
    out.save(path)


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
    for sub in ("step", "stl", "dxf", "renders"):
        (OUT / sub).mkdir(parents=True, exist_ok=True)
    print(f"Prop {PROP_IN:.0f} in -> motors at +/-{MOTOR_XY:.1f} mm, wheelbase {WHEELBASE:.1f} mm, tube {TUBE_LEN:.1f} mm long")

    tube, root, mount = make_tube(), make_root(), make_mount()
    spoke, arc = make_spoke(), make_arc()
    hinge = make_hinge_base()
    top, bottom, deck = make_plate("top"), make_plate("bottom"), make_deck()
    tray, leg = make_tray(), make_leg()

    # Printed parts, in print orientation, with an estimate of how solid each prints.
    printed = {
        "arm_root_A_x2": (root.rotate(V(0, 0, 0), V(1, 0, 0), 180), 2, 0.6),
        "arm_root_B_x2": (root.mirror("XZ").rotate(V(0, 0, 0), V(1, 0, 0), 180), 2, 0.6),
        "motor_mount_x4": (mount.rotate(V(0, 0, 0), V(1, 0, 0), 180), 4, 0.5),
        "hinge_base_A_x2": (hinge, 2, 0.8), "hinge_base_B_x2": (hinge.mirror("XZ"), 2, 0.8),
        "top_plate_x1": (top, 1, 0.9), "bottom_plate_x1": (bottom, 1, 0.9), "lidar_deck_x1": (deck, 1, 0.9),
        "battery_tray_x1": (tray, 1, 0.7), "leg_A_x2": (leg, 2, 0.85), "leg_B_x2": (leg.mirror("XZ"), 2, 0.85),
        "guard_arc_x16": (arc, 16, 1.0), "guard_spoke_x16": (spoke, 16, 1.0),
    }
    print("\nPRINTED PARTS (Ender 3 V3 SE bed, sound STL, STEP for SOLIDWORKS)")
    printed_g = 0.0
    limit = BED - 2 * BED_MARGIN
    for name, (part, qty, fill) in printed.items():
        deg = best_turn(part)
        p = rot_z(part, deg)
        bb = p.BoundingBox()
        p = p.translate(V(-bb.center.x, -bb.center.y, -bb.zmin))
        bb = p.BoundingBox()
        p.exportStep(str(OUT / "step" / f"{name}.step"))
        mesh = tri_mesh(p)
        mesh.export(OUT / "stl" / f"{name}.stl")
        g = p.Volume() / 1000 * DENSITY * fill
        printed_g += g * qty
        fits = bb.xlen <= limit and bb.ylen <= limit and bb.zlen <= BED_H
        rpt.check(f"{name}: {bb.xlen:.0f} x {bb.ylen:.0f} x {bb.zlen:.0f} mm, about {g:.0f} g each",
                  fits and p.isValid() and mesh.is_watertight and len(p.Solids()) == 1,
                  "" if mesh.is_watertight and p.isValid() else "BAD SOLID")
    tube.exportStep(str(OUT / "step" / "arm_tube_x4_BUY.step"))
    tube_g = 4 * tube.Volume() / 1000 * TUBE_DENSITY
    for which, part in (("top", top), ("bottom", bottom)):
        holes, slots = plate_features(which)
        write_dxf(OUT / "dxf" / f"{which}_plate.dxf", plate_outline(which == "bottom"), holes, slots,
                  f"{which.upper()} PLATE  {PLATE_T} mm PETG, or 1.5 mm carbon")

    # ----- assembly
    unit_parts = [tube, root, to_motor_frame(mount),
                  cyl(MOTOR_R, PAD_TOP, PAD_TOP + MOTOR_H, MOTOR_XY, MOTOR_XY)]
    unit_parts += [to_motor_frame(p) for p in guard_pieces(spoke, arc)]
    unit = fuse(*unit_parts)
    body = {
        "hinge bases": fuse(*[to_corner(hinge, c) for c in CORNERS]),
        "bottom plate": bottom, "top plate": top.translate(V(0, 0, TOP_Z)),
        "lidar deck": deck.translate(V(0, 0, DECK_Z)), "battery tray": tray_to_assembly(tray),
    }
    legs = [place_leg(leg, *c) for c in CORNERS]
    body["legs"] = fuse(*legs)

    deployed = {c: to_corner(unit, c) for c in CORNERS}
    folded = {c: to_corner(fold(unit, 90), c) for c in CORNERS}
    everything = list(deployed.values()) + list(body.values())

    print("\nCOMPETITION GEOMETRY")
    bbs = [s.BoundingBox() for s in everything]
    width = max(max(b.xmax for b in bbs) - min(b.xmin for b in bbs), max(b.ymax for b in bbs) - min(b.ymin for b in bbs)) + 2 * HEAD_PROUD
    height = max(b.zmax for b in bbs) - min(b.zmin for b in bbs)
    rpt.check(f"fits the 2 ft launch square ({LAUNCH_BOX:.0f} mm)", width <= LAUNCH_BOX, f"{width:.0f} mm wide, {LAUNCH_BOX - width:.0f} mm spare")
    worst = 2 * (MOTOR_XY * math.sqrt(2) + GUARD_R + BAND_T + HEAD_PROUD)
    rpt.check(f"fits a {CORRIDOR:.0f} mm corridor", worst < CORRIDOR,
              f"square on: {(CORRIDOR - width) / 2:.0f} mm each side; worst 45 deg yaw: {(CORRIDOR - worst) / 2:.0f} mm each side")

    print("\nCLEARANCE CHECKS")
    for label, state in (("deployed", deployed), ("folded", folded)):
        w = 0.0
        for i, c in enumerate(CORNERS):
            for o in CORNERS[i + 1:]:
                w = max(w, overlap(state[c], state[o]))
            for b in body.values():
                w = max(w, overlap(state[c], b))
        rpt.check(f"{label}: arms, guards and motors clear each other and the body", w < 0.5, f"worst {w:.2f} mm3")
    w = 0.0
    for deg in (20, 45, 70):
        for c in CORNERS:
            arm_pose = to_corner(fold(unit, deg), c)
            for b in body.values():
                w = max(w, overlap(arm_pose, b))
    rpt.check("arms swing down without hitting the body (20, 45, 70 deg)", w < 0.5, f"worst {w:.2f} mm3")
    up = overlap(fold(root, -6), body["hinge bases"])
    rpt.check("stop wall carries the thrust: the arm cannot rotate upward", up > 1.0, f"contact {up:.0f} mm3 at 6 deg up")
    disc = cyl(PROP_R, PROP_Z - 1.5, PROP_Z + 1.5, MOTOR_XY, MOTOR_XY)
    hit = max(overlap(disc, s) for s in [deployed[(1, 1)].cut(cyl(MOTOR_R + 0.5, PAD_TOP, PAD_TOP + MOTOR_H, MOTOR_XY, MOTOR_XY))] + list(body.values()))
    rpt.check(f"{PROP_IN:.0f} in prop clears guard, arm and body", hit < 0.5, f"{hit:.1f} mm3")
    scan = cyl(2000.0, SCAN_Z - 2, SCAN_Z + 2)
    blk = max(overlap(scan, s) for s in deployed.values())
    rpt.check("X2 scan plane is clear of guards and motors", blk < 0.5, f"scan at {SCAN_Z:.0f} mm, guard top at {BAND_Z0 + BAND_H:.0f} mm")
    fc = bx(-40.75, 40.75, -25, 25, PLATE_T + 4, PLATE_T + 4 + 15.5)
    rpt.check("Pixhawk fits between the plates on 4 mm foam", max(overlap(fc, b) for b in body.values()) < 0.5)
    pi = bx(-42.5, 42.5, -28, 28, DECK_Z - 6 - 22, DECK_Z - 6)
    rpt.check("Pi 4B hangs under the lidar deck, clear of the top plate", max(overlap(pi, b) for b in body.values()) < 0.5,
              f"{DECK_Z - 28 - TOP_Z - PLATE_T:.0f} mm above the top plate")
    bat = bx(-BATTERY[0] / 2, BATTERY[0] / 2, -BATTERY[1] / 2, BATTERY[1] / 2, TRAY_Z0 + TRAY_FLOOR, TRAY_Z0 + TRAY_FLOOR + BATTERY[2])
    rpt.check(f"battery up to {BATTERY[0]:.0f} x {BATTERY[1]:.0f} x {BATTERY[2]:.0f} mm fits the tray",
              max(overlap(bat, b) for b in body.values()) < 0.5)
    ground = min(b.zmin for b in bbs)
    sensor_low = TRAY_Z0 - 21.0
    rpt.check("downward sensors stay above the ground when landed", sensor_low > ground + 5, f"{sensor_low - ground:.0f} mm clear")
    views = [cq.Solid.makeCone(6.0, 6.0 + 300 * math.tan(math.radians(1)), 300, V(TF_X, 0, TRAY_Z0 - 21), V(0, 0, -1)),
             cq.Solid.makeCone(8.0, 8.0 + 300 * math.tan(math.radians(12)), 300, V(FLOW_X, 0, TRAY_Z0 - 25), V(0, 0, -1))]
    rpt.check("TFmini-S and PX4Flow see past the legs", max(overlap(v, l) for v in views for l in legs) < 0.5)

    from shapely.geometry import LineString, Point as SPoint
    for which in ("top", "bottom"):
        holes, slots = plate_features(which)
        feats = [SPoint(x, y).buffer(d / 2, 32) for x, y, d in holes]
        for cx, cy, length, sw, ang in slots:
            half = (length - sw) / 2
            ux, uy = math.cos(math.radians(ang)), math.sin(math.radians(ang))
            feats.append(LineString([(cx - ux * half, cy - uy * half), (cx + ux * half, cy + uy * half)]).buffer(sw / 2, 32))
        edge = Polygon(plate_outline(which == "bottom")).exterior
        web = min(min(a.distance(b) for i, a in enumerate(feats) for b in feats[i + 1:]), min(edge.distance(f) for f in feats))
        rpt.check(f"{which} plate: every hole and slot leaves at least 3 mm of material", web >= 3.0, f"thinnest {web:.1f} mm")

    def aligned(points, z0, z1, solids):
        return all(overlap(cyl(0.8, z0, z1, x, y), s) < 1e-6 for x, y in points for s in solids)

    rpt.check("hinge base bolts line up with both plates",
              aligned([p for c in CORNERS for p in hinge_points(c)], -1, TOP_Z + PLATE_T + 1, [body["hinge bases"], body["bottom plate"], body["top plate"]]))
    rpt.check("tray bolts line up with the bottom plate", aligned(TRAY_BOLTS, TRAY_Z0 - 1, PLATE_T + 1, [body["battery tray"], body["bottom plate"]]))
    rpt.check("deck posts line up with top plate and deck", aligned(DECK_POSTS, TOP_Z - 1, DECK_Z + DECK_T + 1, [body["top plate"], body["lidar deck"]]))
    rpt.check("leg bolts line up with the tray", aligned(leg_holes_tray(), TRAY_Z0 - FLANGE_T - 1, TRAY_Z0 + TRAY_FLOOR + 1, [body["battery tray"], body["legs"]]))
    pin = (cyl_along(1.2, (HINGE_X - 25, HINGE_Y, HINGE_Z), (1, 0, 0), 50)
           .fuse(cyl_along(1.2, (HINGE_X - 25, HINGE_Y + LOCK_DY, HINGE_Z + LOCK_DZ), (1, 0, 0), 50)))
    rpt.check("hinge and lock bores line up through clevis and arm root", overlap(pin, root) < 1e-6 and overlap(pin, hinge) < 1e-6)
    spk = fuse(*[cyl(1.0, PAD_TOP - PAD_T - 1, PAD_TOP + 4, x, y) for x, y in spoke_points()])
    rpt.check("guard spokes bolt to the motor mount", overlap(to_motor_frame(spk), fuse(*unit_parts[2:])) < 1e-6)
    rpt.check("tube sits in the root and the mount without overlap", overlap(tube, root) < 0.5 and overlap(tube, to_motor_frame(mount)) < 0.5)

    # ----- mass and performance
    bought = sum(BOUGHT_G.values())
    auw = printed_g + tube_g + bought
    print("\nMASS AND PERFORMANCE (estimates)")
    print(f"  printed parts about {printed_g:.0f} g, arm tubes {tube_g:.0f} g (aluminium), other parts {bought:.0f} g")
    print(f"  all-up weight about {auw:.0f} g")
    for label, t in (("low", THRUST_LOW_G), ("high", THRUST_HIGH_G)):
        print(f"  thrust-to-weight, {label} thrust figure ({t:.0f} g per motor): {4 * t / auw:.2f}")
    watts = auw / HOVER_G_PER_W + AVIONICS_W
    print(f"  hover about {watts:.0f} W, {watts / 11.1:.0f} A, {USABLE_MAH / 1000 / (watts / 11.1) * 60:.1f} min")
    fold_bbs = [s.BoundingBox() for s in list(folded.values()) + list(body.values())]
    fsize = [max(b.xmax for b in fold_bbs) - min(b.xmin for b in fold_bbs), max(b.ymax for b in fold_bbs) - min(b.ymin for b in fold_bbs),
             max(b.zmax for b in fold_bbs) - min(b.zmin for b in fold_bbs)]
    print(f"\nSIZE  deployed {width:.0f} x {width:.0f} x {height:.0f} mm; folded {fsize[0]:.0f} x {fsize[1]:.0f} x {fsize[2]:.0f} mm")

    # Whole drone as one STEP for SOLIDWORKS, arms deployed.
    cq.Compound.makeCompound(everything).exportStep(str(OUT / "step" / "assembly_deployed.step"))

    print("\nRENDERING")
    dark, orange, yellow, grey, silver = (70, 76, 84), (214, 96, 40), (232, 178, 70), (150, 155, 165), (200, 204, 210)

    def scene(deg):
        arms = fuse(*[to_corner(fold(fuse(root, to_motor_frame(mount)), deg), c) for c in CORNERS])
        tubes = fuse(*[to_corner(fold(tube, deg), c) for c in CORNERS])
        guards = fuse(*[to_corner(fold(to_motor_frame(p), deg), c) for c in CORNERS for p in guard_pieces(spoke, arc)])
        motors = fuse(*[to_corner(fold(cyl(MOTOR_R, PAD_TOP, PAD_TOP + MOTOR_H, MOTOR_XY, MOTOR_XY), deg), c) for c in CORNERS])
        lidar = cyl(30.25, DECK_Z + DECK_T + 15, DECK_Z + DECK_T + 50)
        frame = fuse(body["hinge bases"], body["bottom plate"], body["top plate"], body["lidar deck"], body["battery tray"])
        return [(arms, dark), (tubes, silver), (guards, yellow), (motors, grey), (frame, orange), (body["legs"], dark), (lidar, grey)]

    render(scene(0), OUT / "renders" / "deployed.png", 25, -125, f"Version 4 deployed, {WHEELBASE:.0f} mm wheelbase")
    render(scene(90), OUT / "renders" / "folded.png", 20, -125, "Arms folded down")
    render([(fuse(root, tube, to_motor_frame(mount)), dark)], OUT / "renders" / "arm.png", 30, -60, "Arm: root, tube, motor mount")

    print(f"\n{'ALL CHECKS PASSED' if rpt.failed == 0 else str(rpt.failed) + ' CHECK(S) FAILED'}")
    return rpt.failed


if __name__ == "__main__":
    raise SystemExit(main())
