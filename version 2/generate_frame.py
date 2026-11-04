#!/usr/bin/env python3
"""NIDAR AirMouse indoor quadcopter frame, version 2. Original design.

Downward-folding arms, full-circle prop guards, and mounts for: Pixhawk 2.4.8,
Raspberry Pi 4B, YDLidar X2, TFmini-S, PX4Flow, 2212 920 KV motors, 3S 5000 mAh.

Run:  python generate_frame.py
Out:  stl/*.stl (print-oriented), renders/*.png, and a check report on stdout.

Assembly coordinates: +X forward, +Y left, +Z up, z = 0 at the underside of the
bottom deck. All lengths are in mm. Change the PARAMETERS block and re-run.
"""
import math
from pathlib import Path

import numpy as np
import trimesh
from manifold3d import CrossSection, FillRule, Manifold, OpType
from shapely.geometry import Point, Polygon, box
from shapely.ops import unary_union

# ---------------------------------------------------------------- PARAMETERS
# Competition limits (Rulebook 8.22, 8.23; Mission Brief section 2)
LAUNCH_BOX = 609.6          # 2 ft square launch area
CORRIDOR = 1000.0
ROOM = 2000.0
MAX_AUW_G = 10000.0

# Printer: Creality Ender 3 V3 SE
BED, BED_HEIGHT, BED_MARGIN = 220.0, 250.0, 2.5

# Propulsion
PROP_IN = 10.0
MOTOR_XY = 141.0            # motor centres at (+/-MOTOR_XY, +/-MOTOR_XY)
MOTOR_R, MOTOR_H = 14.0, 30.0
MOTOR_HOLES = (16.0, 19.0)  # hole-to-hole across the motor, M3. MEASURE YOURS.
PROP_Z_ABOVE_PAD = 38.0     # pad top to prop plane. MEASURE YOURS.

# Hinge: hinge lines run along X, so front and rear guards never meet when folded.
HINGE_X, HINGE_Y, HINGE_Z = 62.0, 64.0, 11.0
LOCK_DY, LOCK_DZ = 10.0, 15.0     # lock bolt position relative to the hinge
ARM_TOP = 35.0              # top of arm and motor pad
BLOCK_W, BLOCK_BACK, BLOCK_OUT = 20.0, 8.0, 34.0
EAR_T = 4.5
EAR_END = LOCK_DY + 6.0     # ear length past the hinge, kept short so the tube clears it
TUBE_START_Y = HINGE_Y + 30.0

# Arm tube
TUBE_W0, TUBE_W1 = 22.0, 18.0
TUBE_H0, TUBE_H1 = 24.0, 18.0
KEEL, WALL = 4.0, 2.0
PAD_T = 4.0
SPOKE_HOLE_R, SPOKE_HOLE_T = 25.0, 5.0

# Body
DECK_X, DECK_Y = 90.0, 81.0
DECK_T = 2.6
RAIL_Y, RAIL_T = 45.0, 2.4
TOP_DECK_Z = 44.0           # underside of top deck; battery bay is 41 mm tall
POST_H = 60.0               # top deck to upper deck; Pi 4B hangs in this space
UPPER_T = 3.0
BOSS_R = 4.5
BOSS_POINTS = [(sx * x, sy * RAIL_Y) for x in (20.0, 62.0, 86.0) for sx in (1, -1) for sy in (1, -1)]
POST_POINTS = [(sx * 44.0, sy * 34.0) for sx in (1, -1) for sy in (1, -1)]

# Prop guard
GUARD_CLEAR = 8.0           # radial gap from prop tip to guard
BAND_H, BAND_T = 20.0, 1.2
HEAD_PROUD = 3.0            # bolt head outside the band

# Landing legs
LEG_ROOT = (30.0, 32.0)
LEG_DIR = math.degrees(math.atan2(60.0, 67.0))
LEG_REACH, LEG_DROP = 76.0, 75.0
LEG_FLANGE_T, LEG_FLANGE_W, LEG_T = 4.0, 16.0, 6.0

# Component envelopes (L, W, H). Sources are listed in README.md.
BATTERY = (165.0, 50.0, 38.0)   # upper limit, pack not measured
PIXHAWK = (81.5, 50.0, 15.5)
PI4 = (85.0, 56.0, 22.0)
PI_HOLES = (58.0, 49.0)
X2_TURRET_R = 30.25
X2_HOLES = [(-35.3, 0.0), (36.0, 18.0), (36.0, -18.0)]
X2_SCAN_ABOVE_BASE = 42.0   # scaled from the datasheet drawing, not a stated value
TFMINI_HOLE_SPACING = 36.0
PX4FLOW = (45.5, 35.0)

# Component masses, grams. Assumed values are marked in the README.
MASSES = {
    "motors (4)": 4 * 57, "ESCs (4)": 4 * 35, "props (4)": 4 * 13, "battery": 471,
    "Pixhawk": 38, "Pi 4B + cooling + regulator": 76, "YDLidar X2": 200, "TFmini-S": 5,
    "PX4Flow": 20, "camera": 10, "power module, wiring, connectors": 100,
    "RC receiver + telemetry": 40, "bolts and nuts": 90,
}
THRUST_LOW_G, THRUST_HIGH_G = 642.0, 850.0   # 1045 prop on 3S, per motor
HOVER_G_PER_W = 7.0         # assumed; the 642 g test point gives 6.1 g/W at full power
USABLE_MAH = 4000.0         # 80% of 5000 mAh
AVIONICS_W = 20.0
DENSITY = 1.27              # PETG, g/cm3

OUT = Path(__file__).parent
SEG = 64
M2_5, M3, M4, HEAD_D = 2.8, 3.4, 4.3, 6.5
PROP_R = PROP_IN * 25.4 / 2
GUARD_R = PROP_R + GUARD_CLEAR
GUARD_OUT = GUARD_R + BAND_T + HEAD_PROUD
PROP_Z = ARM_TOP + PROP_Z_ABOVE_PAD
BAND_Z0 = PROP_Z - BAND_H / 2
UPPER_Z = TOP_DECK_Z + DECK_T + POST_H
SCAN_Z = UPPER_Z + UPPER_T + X2_SCAN_ABOVE_BASE


# ------------------------------------------------------------------- HELPERS
def cyl(r, z0, z1, x=0.0, y=0.0):
    return Manifold.cylinder(z1 - z0, r, r, SEG).translate([x, y, z0])


def extrude_shape(shape, z0, z1):
    parts = shape.geoms if hasattr(shape, "geoms") else [shape]
    rings = []
    for part in parts:
        rings.append(list(part.exterior.coords)[:-1])
        rings.extend(list(r.coords)[:-1] for r in part.interiors)
    return Manifold.extrude(CrossSection(rings, fillrule=FillRule.EvenOdd), z1 - z0).translate([0, 0, z0])


def poly(pts, z0, z1):
    return extrude_shape(Polygon(pts), z0, z1)


def union(items):
    return Manifold.batch_boolean(list(items), OpType.Add)


def subtract(base, cutters):
    return base - union(cutters) if cutters else base


def bbox(m):
    b = m.bounding_box()
    return np.array(b[:3]), np.array(b[3:])


def overlap(a, b):
    return (a ^ b).volume()


def blk(x0, x1, y0, y1, z0, z1):
    return Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def teardrop_2d(a, b, d):
    """Hole outline in the (a, b) plane with its point toward +b: prints without support."""
    r = d / 2
    pts = [(a + r * math.cos(math.radians(t)), b + r * math.sin(math.radians(t)))
           for t in range(-225, 46, 15)]
    pts.append((a, b + r * math.sqrt(2)))
    return Polygon(pts)


def x_axis_teardrop(y, z, d, x0, x1):
    """Bore along X whose point faces +Z."""
    prism = extrude_shape(teardrop_2d(y, z, d), x0, x1)
    return prism.transform([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]])


def to_trimesh(m):
    mesh = m.simplify(0.005).to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=False)


# ------------------------------------------------------------------- ARM
TUBE_ANGLE = math.degrees(math.atan2(MOTOR_XY - TUBE_START_Y, MOTOR_XY - HINGE_X))
TUBE_L = math.hypot(MOTOR_XY - TUBE_START_Y, MOTOR_XY - HINGE_X)
CHANNEL_U0, CHANNEL_U1 = 22.0, TUBE_L - 15.0


def tube_t(u):
    return min(max(u / TUBE_L, 0.0), 1.0)


def outer_section(u):
    t = tube_t(u)
    half = (TUBE_W0 + (TUBE_W1 - TUBE_W0) * t) / 2
    zb = ARM_TOP - (TUBE_H0 + (TUBE_H1 - TUBE_H0) * t)
    return [(half, ARM_TOP), (half, zb + KEEL), (half - KEEL, zb), (KEEL - half, zb),
            (-half, zb + KEEL), (-half, ARM_TOP)]


def inner_section(u):
    t = tube_t(u)
    half = (TUBE_W0 + (TUBE_W1 - TUBE_W0) * t) / 2 - WALL
    zb = ARM_TOP - (TUBE_H0 + (TUBE_H1 - TUBE_H0) * t)
    peak = zb + WALL + 1.0
    return [(half, ARM_TOP - WALL), (half, peak + half), (0.0, peak), (-half, peak + half),
            (-half, ARM_TOP - WALL)]


def loft(section, u0, u1):
    return Manifold.hull_points([(u0, y, z) for y, z in section(u0)] + [(u1, y, z) for y, z in section(u1)])


def spoke_hole_points():
    """Guard spoke bolt positions around the motor, in tube-local coordinates."""
    pts = []
    for k in range(4):
        a = math.radians(45 + 90 * k)
        for t in (-SPOKE_HOLE_T, SPOKE_HOLE_T):
            pts.append((SPOKE_HOLE_R * math.cos(a) - t * math.sin(a),
                        SPOKE_HOLE_R * math.sin(a) + t * math.cos(a)))
    return pts


def make_arm_fl():
    """Front-left arm in assembly position, arm deployed. Other arms are copies or mirrors."""
    tube = union([loft(outer_section, 0.0, TUBE_L - 15.0), loft(outer_section, TUBE_L - 15.0, TUBE_L),
                  cyl(MOTOR_R, ARM_TOP - TUBE_H1, ARM_TOP, TUBE_L)])
    lobes = [Point(TUBE_L, 0).buffer(18, 32)]
    for k in range(4):
        a = math.radians(45 + 90 * k)
        cx, cy = TUBE_L + SPOKE_HOLE_R * math.cos(a), SPOKE_HOLE_R * math.sin(a)
        rect = Polygon([(cx - 8 * math.cos(a) + 10 * math.sin(a), cy - 8 * math.sin(a) - 10 * math.cos(a)),
                        (cx + 8 * math.cos(a) + 10 * math.sin(a), cy + 8 * math.sin(a) - 10 * math.cos(a)),
                        (cx + 8 * math.cos(a) - 10 * math.sin(a), cy + 8 * math.sin(a) + 10 * math.cos(a)),
                        (cx - 8 * math.cos(a) - 10 * math.sin(a), cy - 8 * math.sin(a) + 10 * math.cos(a))])
        lobes.append(rect)
        lobes.append(Polygon([(TUBE_L, 0), (rect.exterior.coords[0]), (rect.exterior.coords[3])]))
    pad = extrude_shape(unary_union(lobes), ARM_TOP - PAD_T, ARM_TOP)
    arm_local = tube + pad

    cuts = [loft(inner_section, CHANNEL_U0, CHANNEL_U1),
            cyl(5.0, ARM_TOP - TUBE_H1 - 1, ARM_TOP + 1, TUBE_L),            # shaft clearance
            cyl(4.5, ARM_TOP - 10, ARM_TOP + 1, TUBE_L - 22.0)]              # motor wire exit
    # Side wire exit near the neutral axis; 45 deg diamond prints without support.
    uc, zc, r = 32.0, ARM_TOP - 12.0, 6.0
    diamond = Polygon([(uc + r, zc), (uc, zc + r), (uc - r, zc), (uc, zc - r)])
    cuts.append(extrude_shape(diamond, -40, 40).rotate([90, 0, 0]))
    for k, side in enumerate(MOTOR_HOLES):
        for s in (1, -1):
            x = TUBE_L + (s * side / 2 if k == 0 else 0.0)
            y = 0.0 if k == 0 else s * side / 2
            cuts.append(cyl(M3 / 2, ARM_TOP - PAD_T - 1, ARM_TOP + 1, x, y))
            cuts.append(cyl(HEAD_D / 2, ARM_TOP - 40, ARM_TOP - PAD_T, x, y))
    for x, y in spoke_hole_points():
        cuts.append(cyl(M3 / 2, ARM_TOP - PAD_T - 1, ARM_TOP + 1, TUBE_L + x, y))
    arm_local = subtract(arm_local, cuts)
    placed_tube = arm_local.rotate([0, 0, TUBE_ANGLE]).translate([HINGE_X, TUBE_START_Y, 0])

    # Hinge block: rounded about the hinge so it can only rotate downward past the stop wall.
    y0, y1 = HINGE_Y - BLOCK_BACK, HINGE_Y + BLOCK_OUT
    profile = unary_union([Point(HINGE_Y, HINGE_Z).buffer(BLOCK_BACK, 32),
                           box(y0, HINGE_Z, y1, ARM_TOP), box(HINGE_Y, HINGE_Z - BLOCK_BACK, y1, HINGE_Z)])
    block = extrude_shape(profile, HINGE_X - BLOCK_W / 2, HINGE_X + BLOCK_W / 2).transform(
        [[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]])
    arm = block + placed_tube
    bores = [cyl_x(M4 / 2, HINGE_X - 20, HINGE_X + 20, HINGE_Y, HINGE_Z),
             cyl_x(M4 / 2, HINGE_X - 20, HINGE_X + 20, HINGE_Y + LOCK_DY, HINGE_Z + LOCK_DZ)]
    return subtract(arm, bores)


def cyl_x(r, x0, x1, y, z):
    return Manifold.cylinder(x1 - x0, r, r, SEG).rotate([0, 90, 0]).translate([x0, y, z])


def fold(m, deg):
    """Rotate a front-left part about its hinge line. deg = 0 deployed, 90 folded down."""
    return m.translate([0, -HINGE_Y, -HINGE_Z]).rotate([-deg, 0, 0]).translate([0, HINGE_Y, HINGE_Z])


def to_corner(m, corner):
    """Move a front-left part to any corner. Mirror for FR and RL, turn for RR and RL."""
    sx, sy = corner
    if sx * sy < 0:
        m = m.mirror([0, 1, 0])
    if sx < 0:
        m = m.rotate([0, 0, 180])
    return m


# ------------------------------------------------------------------- GUARD
def make_spoke():
    """Print frame: U = radial, V = height, W = sideways. Web lies on the bed."""
    root0, root1 = 18.0, 32.0
    tip0, tip1 = GUARD_R - 3.0, GUARD_R
    low_tip, high_tip = BAND_Z0, BAND_Z0 + BAND_H
    outer = Polygon([(root0, ARM_TOP), (root1, ARM_TOP), (tip0, low_tip), (tip1, low_tip), (tip1, high_tip),
                     (tip0, high_tip), (tip0, PROP_Z - 6.0), (root0, ARM_TOP + 13.0)])
    window = Polygon([(root1 + 8, ARM_TOP + 5.0), (tip0 - 14, low_tip + 1.0), (root1 + 8, ARM_TOP + 11.0)])
    web = extrude_shape(outer.difference(window), 0, 2.4)
    root = blk(root0, root1, ARM_TOP, ARM_TOP + 3.0, 0, 20.0)
    tip = blk(tip0, tip1, low_tip, high_tip, 0, 20.0)
    spoke = web + root + tip

    cutters = []
    for w in (10.0 - SPOKE_HOLE_T, 10.0 + SPOKE_HOLE_T):
        # Vertical (assembly) bolts through the root flange: bore along V, point toward +W.
        cutters.append(_v_bore(SPOKE_HOLE_R, w, M3, ARM_TOP - 1, ARM_TOP + 5))
    for w in (3.0, 17.0):
        cutters.append(_u_bore(PROP_Z, w, M3, tip0 - 1, tip1 + 1))
    return subtract(spoke, cutters)


def _v_bore(u, w, d, v0, v1):
    """Bore along V at (u, w), point toward +W."""
    prism = extrude_shape(teardrop_2d(u, w, d), 0, v1 - v0)   # outline in (U, W), height along Z
    # Map outline x -> U, outline y -> W, height -> V, keeping a right-handed frame.
    return prism.transform([[1, 0, 0, 0], [0, 0, -1, v1], [0, 1, 0, 0]])


def _u_bore(v, w, d, u0, u1):
    """Bore along U at (v, w), point toward +W."""
    prism = extrude_shape(teardrop_2d(v, w, d), u0, u1)
    return prism.transform([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0]])


def spoke_to_assembly(spoke):
    """Spoke print frame to guard frame: X = radial, Y = 10 - W, Z = V."""
    return spoke.transform([[1, 0, 0, 0], [0, 0, -1, 10.0], [0, 1, 0, 0]])


def make_arc():
    """Quarter of the guard ring, spanning +/-45 deg about +X, printed rim down."""
    r0, r1 = GUARD_R, GUARD_R + BAND_T
    profile = [(r0 - 2.0, 0.0), (r1, 0.0), (r1, BAND_H), (r0 - 2.0, BAND_H), (r0, BAND_H - 2.0), (r0, 2.0)]
    half_gap = math.degrees(0.15 / GUARD_R)
    arc = Manifold.revolve(CrossSection([profile]), 180, 90.0 - 2 * half_gap).rotate([0, 0, -45 + half_gap])
    pads, bores = [], []
    for side in (1, -1):
        end = side * (45 - half_gap)
        offset = 7.0
        ang = end - side * math.degrees(offset / GUARD_R)
        pad = blk(r0 - 0.5, r1 + 0.8, -6.0, 6.0, 0.0, BAND_H).rotate([0, 0, ang])
        pads.append(pad)
        bore = extrude_shape(teardrop_2d(0.0, BAND_H / 2, M3), 0, 12).transform(
            [[0, 0, 1, r0 - 4], [1, 0, 0, 0], [0, 1, 0, 0]]).rotate([0, 0, ang])
        bores.append(bore)
    return subtract(union([arc] + pads), bores)


def guard_in_tube_frame(spoke, arc):
    """All guard pieces around the motor, with the motor axis at the origin."""
    pieces = []
    placed_spoke = spoke_to_assembly(spoke)
    for k in range(4):
        a = 45 + 90 * k
        pieces.append(placed_spoke.rotate([0, 0, a]))
        pieces.append(arc.translate([0, 0, BAND_Z0]).rotate([0, 0, a + 45]))
    return union(pieces)


def guard_fl(spoke, arc):
    return guard_in_tube_frame(spoke, arc).rotate([0, 0, TUBE_ANGLE]).translate([MOTOR_XY, MOTOR_XY, 0])


# ------------------------------------------------------------------- BODY
def leg_hole_points():
    pts = []
    for sx, sy in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        d = math.radians(LEG_DIR)
        ux, uy = sx * math.cos(d), sy * math.sin(d)
        vx, vy = -uy, ux
        for off in (-11.0, 11.0):
            pts.append((sx * LEG_ROOT[0] + off * ux + 3.0 * vx, sy * LEG_ROOT[1] + off * uy + 3.0 * vy))
    return pts


SENSOR_BOLTS = [(sx * 46.0, sy * 18.0) for sx in (1, -1) for sy in (1, -1)]
STRAP_SLOTS = [box(sx * 66.0 - 3, -14, sx * 66.0 + 3, 14) for sx in (1, -1)]
ZIP_SLOTS = [box(sx * 46.0 - 3, sy * 31.0 - 7, sx * 46.0 + 3, sy * 31.0 + 7) for sx in (1, -1) for sy in (1, -1)]


def deck_outline():
    centre = box(-DECK_X, -RAIL_Y - 8, DECK_X, RAIL_Y + 8)
    pads = [box(sx * HINGE_X - 18, -DECK_Y if sy < 0 else 0, sx * HINGE_X + 18, DECK_Y if sy > 0 else 0)
            for sx in (1, -1) for sy in (1, -1)]
    return unary_union([centre] + pads).buffer(4, 16).buffer(-4, 16)


def hex_field(zone, r=9.0, web=3.0):
    flat = 2 * r * math.cos(math.radians(30))
    dx, dy = flat + web, (flat + web) * math.cos(math.radians(30))
    out = []
    for j in range(-9, 10):
        for i in range(-9, 10):
            cx, cy = (i + 0.5 * (j % 2)) * dx, j * dy
            hx = Polygon([(cx + r * math.cos(math.radians(30 + 60 * k)), cy + r * math.sin(math.radians(30 + 60 * k)))
                          for k in range(6)])
            if zone.contains(hx):
                out.append(hx)
    return out


def circles(points, d):
    return [Point(x, y).buffer(d / 2, 24) for x, y in points]


def make_bottom_deck():
    """Bottom deck, side rails, stop walls and hinge ears, printed as one part with z = 0 on the bed."""
    notches = [box(sx * HINGE_X - BLOCK_W / 2 - 0.4, (HINGE_Y - BLOCK_BACK - 1.0) if sy > 0 else -200,
                   sx * HINGE_X + BLOCK_W / 2 + 0.4, 200 if sy > 0 else -(HINGE_Y - BLOCK_BACK - 1.0))
               for sx in (1, -1) for sy in (1, -1)]
    outline = deck_outline().difference(unary_union(notches))
    keep = circles(BOSS_POINTS, 15) + circles(leg_hole_points(), 15) + circles(SENSOR_BOLTS, 13) + \
        [s.buffer(5) for s in STRAP_SLOTS] + [box(-120, RAIL_Y - 6, 120, 200), box(-120, -200, 120, -RAIL_Y + 6)]
    zone = outline.buffer(-4).difference(unary_union(keep))
    holes = hex_field(zone) + STRAP_SLOTS + circles(leg_hole_points(), M3) + circles(SENSOR_BOLTS, M3)
    deck = extrude_shape(outline.difference(unary_union(holes)), 0, DECK_T)

    rails = []
    for sy in (1, -1):
        rails.append(box(-DECK_X + 1, sy * RAIL_Y - RAIL_T / 2, DECK_X - 1, sy * RAIL_Y + RAIL_T / 2))
    rails += circles(BOSS_POINTS, 2 * BOSS_R)
    rail_body = extrude_shape(unary_union(rails), DECK_T - 1, TOP_DECK_Z)

    items = [deck, rail_body]
    cutters = [cyl(M3 / 2, -1, TOP_DECK_Z + 1, x, y) for x, y in BOSS_POINTS]
    zm = (DECK_T + TOP_DECK_Z) / 2
    for xc in (-41.0, 0.0, 41.0):       # 45 deg windows: wire ports that also save weight
        for sy in (1, -1):
            d = Polygon([(xc + 15, zm), (xc, zm + 15), (xc - 15, zm), (xc, zm - 15)])
            cutters.append(_y_prism(d, sy * RAIL_Y - 4, sy * RAIL_Y + 4))
    for sx in (1, -1):
        for sy in (1, -1):
            x0 = sx * HINGE_X
            ear_lo, ear_hi = BLOCK_W / 2 + 0.3, BLOCK_W / 2 + 0.3 + EAR_T
            y_lo, y_hi = HINGE_Y - BLOCK_BACK - 4.0, HINGE_Y + EAR_END
            ear_z = HINGE_Z + LOCK_DZ + 6.0
            for s in (1, -1):
                a, b = sorted((x0 + s * ear_lo, x0 + s * ear_hi))
                ys = sorted((sy * y_lo, sy * y_hi))
                items.append(blk(a, b, ys[0], ys[1], 0.0, ear_z))
            stop = sorted((sy * (HINGE_Y - BLOCK_BACK - 0.2), sy * (HINGE_Y - BLOCK_BACK - 4.0)))
            items.append(blk(x0 - ear_hi, x0 + ear_hi, stop[0], stop[1], 0.0, ear_z))
            for s in (1, -1):
                rib = sorted((sy * (RAIL_Y - 1), sy * (HINGE_Y - BLOCK_BACK - 3.5)))
                a, b = sorted((x0 + s * (ear_hi - 3), x0 + s * ear_hi))
                items.append(blk(a, b, rib[0], rib[1], DECK_T - 1, ear_z))
            for dy, dz in ((0.0, 0.0), (LOCK_DY, LOCK_DZ)):
                cutters.append(x_axis_teardrop(sy * (HINGE_Y + dy), HINGE_Z + dz, M4, x0 - 25, x0 + 25))
    return subtract(union(items), cutters)


def _y_prism(shape, y0, y1):
    """Extrude an (x, z) outline along Y from y0 to y1."""
    return Manifold.extrude(CrossSection([list(shape.exterior.coords)[:-1]]), y1 - y0).rotate([90, 0, 0]).translate([0, y1, 0])


def fc_zip_slots():
    return [box(sx * 34 - 2, sy * 30 - 6, sx * 34 + 2, sy * 30 + 6) for sx in (1, -1) for sy in (1, -1)]


CAMERA_BOLTS = [(81.0, -9.0), (81.0, 9.0)]


def make_top_deck():
    outline = box(-DECK_X, -RAIL_Y - 8, DECK_X, RAIL_Y + 8).buffer(-4).buffer(4, 16)
    wire_slots = [box(sx * 58 - 5, -22, sx * 58 + 5, 22).buffer(1.5) for sx in (1, -1)]
    keep = [box(-44, -38, 44, 38)] + circles(BOSS_POINTS, 15) + circles(POST_POINTS, 15) + \
        circles(CAMERA_BOLTS, 14) + [s.buffer(4) for s in wire_slots]
    zone = outline.buffer(-4).difference(unary_union(keep))
    holes = hex_field(zone) + wire_slots + fc_zip_slots() + circles(BOSS_POINTS, M3) + \
        circles(POST_POINTS, M3) + circles(CAMERA_BOLTS, M3) + [box(-42, -8, -32, 8), box(32, -8, 42, 8)]
    return extrude_shape(outline.difference(unary_union(holes)), 0, DECK_T)


def make_upper_deck():
    """Carries the YDLidar X2 on top and the Pi 4B underneath. Printed posts-up."""
    outline = box(-50, -40, 50, 40).buffer(-4).buffer(4, 16)
    pi = [(sx * PI_HOLES[0] / 2, sy * PI_HOLES[1] / 2) for sx in (1, -1) for sy in (1, -1)]
    keep = circles(POST_POINTS, 15) + circles(pi, 13) + circles(X2_HOLES, 13)
    zone = outline.buffer(-3).difference(unary_union(keep))
    deck = extrude_shape(outline.difference(unary_union(hex_field(zone, 8.0, 3.0))), 0, UPPER_T)
    bodies = [deck, union([cyl(5.0, UPPER_T - 1, UPPER_T + POST_H, x, y) for x, y in POST_POINTS]),
              union([cyl(3.5, UPPER_T - 1, UPPER_T + 6.0, x, y) for x, y in pi])]
    cutters = [cyl(M3 / 2, -1, UPPER_T + POST_H + 1, x, y) for x, y in POST_POINTS] + \
              [cyl(M2_5 / 2, -1, UPPER_T + 7, x, y) for x, y in pi] + \
              [cyl(M2_5 / 2, -1, UPPER_T + 1, x, y) for x, y in X2_HOLES]
    return subtract(union(bodies), cutters)


def make_sensor_mount():
    """Holds the TFmini-S and PX4Flow side by side under the deck. Printed recess up."""
    plate = box(-54, -24, 54, 24).buffer(-3).buffer(3, 16)
    fx, tx = 22.0, -24.0
    window = box(fx - 15, -11, fx + 15, 11)
    zips = [box(fx - 3, sy * 20 - 1.5, fx + 3, sy * 20 + 1.5) for sy in (1, -1)]
    tf = [(tx, sy * TFMINI_HOLE_SPACING / 2) for sy in (1, -1)]
    wire = box(tx - 9, -7, tx + 9, 7)
    shape = plate.difference(unary_union([window, wire] + zips + circles(SENSOR_BOLTS, M3) + circles(tf, 2.4)))
    body = extrude_shape(shape, 0, 5.0)
    recess = blk(fx - PX4FLOW[0] / 2 - 0.5, fx + PX4FLOW[0] / 2 + 0.5,
                 -PX4FLOW[1] / 2 - 0.5, PX4FLOW[1] / 2 + 0.5, 3.0, 6.0)
    return body - recess


def make_leg():
    """Print frame: U = outward, V = down, W = sideways. Web on the bed, flange rising."""
    profile = Polygon([(-15, 0), (15, 0), (30, 52), (LEG_REACH + 6, LEG_DROP - 7), (LEG_REACH + 6, LEG_DROP),
                       (12, LEG_DROP), (-15, 12)])
    windows = [Polygon([(-8, 16), (9, 9), (23, 52), (10, 62)]),
               Polygon([(33, 60), (LEG_REACH - 2, LEG_DROP - 7), (LEG_REACH - 6, LEG_DROP - 5), (20, LEG_DROP - 5)])]
    web = extrude_shape(profile.difference(unary_union(windows)), 0, LEG_T)
    flange = blk(-15, 15, 0, LEG_FLANGE_T, 0, LEG_FLANGE_W)
    leg = web + flange
    cutters = [_v_bore(off, LEG_FLANGE_W - 5.0, M3, -1, LEG_FLANGE_T + 1) for off in (-11.0, 11.0)]
    return subtract(leg, cutters)


def place_leg(leg, sx, sy):
    """U -> horizontal outward, V -> -Z, W -> sideways, then to its corner."""
    m = leg.transform([[1, 0, 0, 0], [0, 0, 1, -LEG_FLANGE_W + 8.0], [0, -1, 0, 0]])
    d = math.degrees(math.atan2(sy * math.sin(math.radians(LEG_DIR)), sx * math.cos(math.radians(LEG_DIR))))
    return m.rotate([0, 0, d]).translate([sx * LEG_ROOT[0], sy * LEG_ROOT[1], 0])


def make_camera_mount():
    """Forward camera bracket for a Raspberry Pi camera board (21 x 12.5 mm holes)."""
    plate = blk(-14, 14, 0, 28, 0, 3)
    flange = blk(-14, 14, 0, 3, 0, 14)
    cutters = [cyl(1.1, -1, 4, sx * 10.5, 14 + sy * 6.25) for sx in (1, -1) for sy in (1, -1)]
    cutters += [_v_bore(off, 9.0, M3, -1, 4) for off in (-9.0, 9.0)]
    cutters.append(blk(-6, 6, 8, 20, -1, 4))
    return subtract(plate + flange, cutters)


# ------------------------------------------------------------------- REPORT
class Report:
    def __init__(self):
        self.failed = 0

    def check(self, label, passed, detail=""):
        self.failed += 0 if passed else 1
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}{'  ' + detail if detail else ''}")


def stl_is_sound(path):
    mesh = trimesh.load_mesh(path, process=True)
    return mesh.is_watertight and mesh.is_winding_consistent, mesh


def best_bed_rotation(m):
    """First turn that fits the bed, trying square and diagonal first."""
    limit = BED - 2 * BED_MARGIN
    best = None
    for deg in [0, 45] + list(range(5, 90, 5)):
        lo, hi = bbox(m.rotate([0, 0, deg]))
        span = max(hi[0] - lo[0], hi[1] - lo[1])
        if span <= limit:
            return deg
        if best is None or span < best[0]:
            best = (span, deg)
    return best[1]


def render(groups, path, elev, azim, title, size=1300):
    from PIL import Image, ImageDraw

    el, az = math.radians(elev), math.radians(azim)
    forward = np.array([math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), math.sin(el)])
    right = np.array([-math.sin(az), math.cos(az), 0.0])
    up = np.cross(forward, right)
    light = forward * 0.75 + right * -0.35 + up * 0.55
    light /= np.linalg.norm(light)
    meshes = [(to_trimesh(m), np.array(c)) for m, c in groups]
    every = np.concatenate([mesh.vertices for mesh, _ in meshes])
    u, v = every @ right, every @ up
    span = max(u.max() - u.min(), v.max() - v.min())
    scale = (size - 120) / span
    cu, cv = (u.max() + u.min()) / 2, (v.max() + v.min()) / 2
    depth = np.full((size, size), -np.inf)
    image = np.full((size, size, 3), 246.0)
    for mesh, color in meshes:
        tris = mesh.triangles
        px = (tris @ right - cu) * scale + size / 2
        py = size / 2 - (tris @ up - cv) * scale
        pz = tris @ forward
        shade = 0.30 + 0.70 * np.abs(mesh.face_normals @ light)
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
            window = depth[y0:y1 + 1, x0:x1 + 1]
            mask = inside & (zz > window)
            window[mask] = zz[mask]
            image[y0:y1 + 1, x0:x1 + 1][mask] = color * s
    out = Image.fromarray(image.astype(np.uint8))
    ImageDraw.Draw(out).text((30, 24), title, fill=(30, 30, 30), font_size=30)
    out.save(path)


CORNERS = [(1, 1), (1, -1), (-1, 1), (-1, -1)]


def main():
    rpt = Report()
    (OUT / "stl").mkdir(exist_ok=True)
    (OUT / "renders").mkdir(exist_ok=True)

    arm_fl = make_arm_fl()
    spoke = make_spoke()
    arc = make_arc()
    bottom = make_bottom_deck()
    top = make_top_deck()
    upper = make_upper_deck()
    sensor = make_sensor_mount()
    leg = make_leg()
    camera = make_camera_mount()

    # Arms print motor face down so the channel's 45 deg peak points up.
    arm_print = arm_fl.rotate([180, 0, 0])
    arm_print_b = arm_fl.mirror([0, 1, 0]).rotate([180, 0, 0])
    # Third value: estimated share of solid plastic after slicing. Thin walled parts
    # stay nearly solid; chunky arms and legs benefit from 30% infill.
    parts = {
        "arm_A_x2": (arm_print, 2, 0.62), "arm_B_x2": (arm_print_b, 2, 0.62),
        "bottom_deck_x1": (bottom, 1, 0.9), "top_deck_x1": (top, 1, 0.95), "upper_deck_x1": (upper, 1, 0.9),
        "sensor_mount_x1": (sensor, 1, 0.9), "leg_x4": (leg, 4, 0.85), "camera_mount_x1": (camera, 1, 1.0),
        "guard_arc_x16": (arc, 16, 1.0), "guard_spoke_x16": (spoke, 16, 1.0),
    }
    print("\nPART CHECKS (fit on Ender 3 V3 SE bed, watertight STL on re-read)")
    solid_g = frame_g = 0.0
    limit = BED - 2 * BED_MARGIN
    for name, (part, count, fill) in parts.items():
        deg = best_bed_rotation(part)
        placed = part.rotate([0, 0, deg])
        lo, hi = bbox(placed)
        placed = placed.translate([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
        lo, hi = bbox(placed)
        size = hi - lo
        path = OUT / "stl" / f"{name}.stl"
        to_trimesh(placed).export(path)
        sound, reread = stl_is_sound(path)
        grams = placed.volume() / 1000 * DENSITY
        solid_g += grams * count
        frame_g += grams * count * fill
        fits = size[0] <= limit and size[1] <= limit and size[2] <= BED_HEIGHT
        same = abs(reread.volume - placed.volume()) / placed.volume() < 0.003
        rpt.check(f"{name}: {size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f} mm, turned {deg} deg, about {grams * fill:.0f} g each",
                  fits and sound and same, "" if sound and same else "STL DEFECT ON RE-READ")

    # ----- assembly
    bot_asm = bottom
    top_asm = top.translate([0, 0, TOP_DECK_Z])
    up_asm = upper.rotate([180, 0, 0]).translate([0, 0, UPPER_Z + UPPER_T])
    sensor_asm = sensor.rotate([180, 0, 0])
    legs = [place_leg(leg, sx, sy) for sx, sy in CORNERS]
    camera_asm = camera.rotate([90, 0, 90]).translate([72.0, 0, TOP_DECK_Z + DECK_T])
    frame_rigid = {"bottom deck": bot_asm, "top deck": top_asm, "upper deck": up_asm, "sensor mount": sensor_asm,
                   "camera mount": camera_asm}
    guard = guard_fl(spoke, arc)
    motor = cyl(MOTOR_R, ARM_TOP, ARM_TOP + MOTOR_H, MOTOR_XY, MOTOR_XY)
    prop_disc = cyl(PROP_R, PROP_Z - 1.5, PROP_Z + 1.5, MOTOR_XY, MOTOR_XY)
    unit = union([arm_fl, guard, motor])

    pose = lambda deg, c: to_corner(fold(unit, deg), c)
    deployed = {c: pose(0, c) for c in CORNERS}
    folded = {c: pose(90, c) for c in CORNERS}

    print("\nCOMPETITION GEOMETRY")
    everything = list(deployed.values()) + list(frame_rigid.values()) + legs
    lows = np.min([bbox(p)[0] for p in everything], axis=0)
    highs = np.max([bbox(p)[1] for p in everything], axis=0)
    size = highs - lows
    head_margin = HEAD_PROUD  # bolt heads are not modelled in the mesh; add them to the envelope
    width = max(size[0], size[1]) + 2 * head_margin
    rpt.check(f"fits the 2 ft launch square (rule 8.23, {LAUNCH_BOX:.0f} mm)", width <= LAUNCH_BOX,
              f"{width:.0f} mm wide, {LAUNCH_BOX - width:.0f} mm spare")
    worst_yaw = 2 * (MOTOR_XY * math.sqrt(2) + GUARD_OUT)
    rpt.check(f"fits a {CORRIDOR:.0f} mm corridor", worst_yaw < CORRIDOR,
              f"square on: {(CORRIDOR - width) / 2:.0f} mm each side; at the worst 45 deg yaw: {(CORRIDOR - worst_yaw) / 2:.0f} mm each side")

    print("\nCLEARANCE CHECKS")
    keys = list(deployed)
    for state, label in ((deployed, "deployed"), (folded, "folded")):
        worst = 0.0
        for i, k in enumerate(keys):
            for other in keys[i + 1:]:
                worst = max(worst, overlap(state[k], state[other]))
            for body in list(frame_rigid.values()) + legs:
                worst = max(worst, overlap(state[k], body))
        rpt.check(f"{label}: arms and guards clear each other, body and legs", worst < 0.5, f"worst overlap {worst:.2f} mm3")

    worst = 0.0
    for deg in np.linspace(5, 85, 9):
        poses = {c: pose(float(deg), c) for c in CORNERS}
        for c in CORNERS:
            for other in CORNERS:
                if c < other:
                    worst = max(worst, overlap(poses[c], poses[other]))
            for body in list(frame_rigid.values()) + legs:
                worst = max(worst, overlap(poses[c], body))
    rpt.check("arms swing down without jamming, even all four together", worst < 0.5, f"worst overlap {worst:.2f} mm3")

    # The hinge cannot rotate upward past the stop wall.
    worst = overlap(to_corner(fold(arm_fl, -6), (1, 1)), bot_asm)
    rpt.check("stop wall takes the thrust load: arm cannot rotate upward", worst > 1.0, f"contact {worst:.0f} mm3 at 6 deg up")

    hits = {}
    for name, body in {**frame_rigid, "arm": arm_fl, "guard": guard}.items():
        hits[name] = overlap(prop_disc, body)
    rpt.check(f"{PROP_IN:.0f} in props clear the guard, arm and body", max(hits.values()) < 0.5,
              f"worst {max(hits, key=hits.get)} {max(hits.values()):.1f} mm3")
    ring_gap = 2 * MOTOR_XY - 2 * (GUARD_R + BAND_T)
    rpt.check("neighbouring guards do not touch", ring_gap > 2.0, f"{ring_gap:.1f} mm between rings")

    battery_box = blk(-BATTERY[0] / 2, BATTERY[0] / 2, -BATTERY[1] / 2, BATTERY[1] / 2, DECK_T + 3.0, DECK_T + 3.0 + BATTERY[2])
    rpt.check("battery bay holds up to 165 x 50 x 38 mm", overlap(battery_box, bot_asm) < 0.5 and overlap(battery_box, top_asm) < 0.5)
    fc_box = blk(-PIXHAWK[0] / 2, PIXHAWK[0] / 2, -PIXHAWK[1] / 2, PIXHAWK[1] / 2, TOP_DECK_Z + DECK_T + 4, TOP_DECK_Z + DECK_T + 4 + PIXHAWK[2])
    pi_box = blk(-PI4[0] / 2, PI4[0] / 2, -PI4[1] / 2, PI4[1] / 2, UPPER_Z - 6 - PI4[2], UPPER_Z - 6)
    rpt.check("Pixhawk clears posts and top deck, with a 4 mm foam gap", overlap(fc_box, up_asm) < 0.5 and overlap(fc_box, top_asm) < 0.5)
    rpt.check("Pi 4B fits under the upper deck", overlap(pi_box, up_asm) < 0.5 and overlap(pi_box, fc_box) < 0.5,
              f"{bbox(pi_box)[0][2] - bbox(fc_box)[1][2]:.0f} mm above the Pixhawk")
    scan = cyl(3000.0, SCAN_Z - 2.0, SCAN_Z + 2.0)
    blocked = max(overlap(scan, p) for p in list(deployed.values()) + [top_asm, bot_asm, camera_asm])
    rpt.check("X2 scan plane is clear of guards, motors and props", blocked < 0.5,
              f"scan at {SCAN_Z:.0f} mm, guard top at {BAND_Z0 + BAND_H:.0f} mm")
    # (name, x, lens start z, lens radius, half angle). TFmini-S: 2 deg FOV from its datasheet.
    # PX4Flow: lens assumed at board centre, 12 deg half angle assumed for its 16 mm lens.
    views = [("TFmini-S", -24.0, -21.0, 6.0, 1.0), ("PX4Flow", 22.0, -25.0, 8.0, 12.0)]
    bad = []
    for label, x, z0, r, half in views:
        reach = 300.0
        cone = Manifold.cylinder(reach, r + reach * math.tan(math.radians(half)), r, SEG).translate([x, 0.0, z0 - reach])
        if max(overlap(cone, l) for l in legs) > 0.5:
            bad.append(label)
    rpt.check("downward sensors have a clear view past the legs", not bad, ",".join(bad))

    hole_ok = True
    for x, y in leg_hole_points():
        hole_ok &= overlap(cyl(0.8, -LEG_FLANGE_T - 1, DECK_T + 1, x, y), union([bot_asm] + legs)) < 1e-6
    rpt.check("leg bolt holes line up with the deck", hole_ok)
    hole_ok = all(overlap(cyl(0.8, -6.0, DECK_T + 1, x, y), union([bot_asm, sensor_asm])) < 1e-6 for x, y in SENSOR_BOLTS)
    rpt.check("sensor mount bolt holes line up with the deck", hole_ok)
    hole_ok = all(overlap(cyl(0.8, TOP_DECK_Z - 1, UPPER_Z + UPPER_T + 1, x, y), union([top_asm, up_asm])) < 1e-6 for x, y in POST_POINTS)
    rpt.check("upper deck post holes line up with the top deck", hole_ok)
    hole_ok = all(overlap(cyl(0.8, -1, TOP_DECK_Z + DECK_T + 1, x, y), union([bot_asm, top_asm])) < 1e-6 for x, y in BOSS_POINTS)
    rpt.check("deck bolt holes line up", hole_ok)
    hole_ok = all(overlap(cyl(0.8, TOP_DECK_Z - 1, TOP_DECK_Z + DECK_T + 4, x, y), union([top_asm, camera_asm])) < 1e-6 for x, y in CAMERA_BOLTS)
    rpt.check("camera mount bolt holes line up with the top deck", hole_ok)
    pin = cyl_x(1.2, HINGE_X - 18, HINGE_X + 18, HINGE_Y, HINGE_Z) + cyl_x(1.2, HINGE_X - 18, HINGE_X + 18, HINGE_Y + LOCK_DY, HINGE_Z + LOCK_DZ)
    rpt.check("hinge and lock bores line up through ears and arm", overlap(pin, bot_asm) < 1e-6 and overlap(pin, arm_fl) < 1e-6)
    spoke_pin = union([cyl(1.0, ARM_TOP - PAD_T - 1, ARM_TOP + 4, x, y) for x, y in spoke_hole_points()])
    spoke_pin = spoke_pin.rotate([0, 0, TUBE_ANGLE]).translate([MOTOR_XY, MOTOR_XY, 0])
    rpt.check("guard spoke bolts line up with the motor pad", overlap(spoke_pin, union([arm_fl, guard])) < 1e-6)
    arc_pin = cyl_x(1.0, GUARD_R - 4, GUARD_R + 3, 0.0, 0.0)
    pins = []
    for k in range(4):
        for t in (-7.0, 7.0):
            pins.append(arc_pin.translate([0, t, PROP_Z]).rotate([0, 0, 45 + 90 * k + TUBE_ANGLE]).translate([MOTOR_XY, MOTOR_XY, 0]))
    arc_overlap = overlap(union(pins), guard)
    rpt.check("guard arc bolts line up with the spokes", arc_overlap < 0.05, f"{arc_overlap:.3f} mm3 in the bore")

    # ----- mass, thrust, endurance
    component_g = sum(MASSES.values())
    auw = frame_g + component_g
    print("\nMASS AND PERFORMANCE (estimates)")
    print(f"  printed frame after slicing, about {frame_g:.0f} g (at most {solid_g:.0f} g if every part were solid)")
    print(f"  components                         {component_g:.0f} g")
    print(f"  all-up weight                      {auw:.0f} g  (limit {MAX_AUW_G:.0f} g)")
    rpt.check("all-up weight under 10 kg", auw < MAX_AUW_G)
    for label, thrust in (("low", THRUST_LOW_G), ("high", THRUST_HIGH_G)):
        print(f"  thrust-to-weight, {label} thrust figure ({thrust:.0f} g per motor): {4 * thrust / auw:.2f}")
    hover_w = auw / HOVER_G_PER_W + AVIONICS_W
    amps = hover_w / 11.1
    print(f"  hover about {hover_w:.0f} W, {amps:.0f} A, {USABLE_MAH / 1000 / amps * 60:.1f} min on {USABLE_MAH:.0f} mAh usable")
    masses = [(frame_g * 0.35, 18), (MASSES['battery'], 25), (MASSES['Pixhawk'], 60), (76, 80), (MASSES['YDLidar X2'], 130),
              (MASSES['motors (4)'] + MASSES['props (4)'] + MASSES['ESCs (4)'] + frame_g * 0.65, 55)]
    cg = sum(m * z for m, z in masses) / sum(m for m, _ in masses)
    stance = min(LEG_ROOT[0] + LEG_REACH * math.cos(math.radians(LEG_DIR)), LEG_ROOT[1] + LEG_REACH * math.sin(math.radians(LEG_DIR)))
    print(f"  CG about {cg:.0f} mm above the deck underside; tips over past {math.degrees(math.atan2(stance, cg + LEG_DROP)):.0f} deg on its legs")

    def envelope(pieces):
        lo = np.min([bbox(p)[0] for p in pieces], axis=0)
        hi = np.max([bbox(p)[1] for p in pieces], axis=0)
        return hi - lo

    fold_size = envelope(list(folded.values()) + list(frame_rigid.values()) + legs)
    print(f"\nSIZE  deployed {size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f} mm (bolt heads add 6 mm); folded {fold_size[0]:.0f} x {fold_size[1]:.0f} x {fold_size[2]:.0f} mm")
    print(f"      motor to motor across the diagonal {2 * MOTOR_XY * math.sqrt(2):.0f} mm")

    # ----- renders (motors grey, arms dark, decks orange, guards yellow)
    dark, orange, yellow, grey = (70, 76, 84), (214, 96, 40), (232, 178, 70), (165, 170, 178)

    def scene(deg):
        arms = union([to_corner(fold(arm_fl, deg), c) for c in CORNERS])
        guards = union([to_corner(fold(guard, deg), c) for c in CORNERS])
        motors = union([to_corner(fold(motor, deg), c) for c in CORNERS])
        lidar = cyl(X2_TURRET_R, UPPER_Z + UPPER_T + 15, UPPER_Z + UPPER_T + 50)
        return [(arms, dark), (guards, yellow), (motors, grey), (union(list(frame_rigid.values())), orange),
                (union(legs), dark), (lidar, grey)]

    print("\nRENDERING")
    render(scene(0), OUT / "renders" / "deployed.png", 25, -125, "Deployed, guards on")
    render(scene(90), OUT / "renders" / "folded.png", 20, -125, "Arms folded down")
    render(scene(45), OUT / "renders" / "half_folded.png", 25, -125, "Mid-fold")
    render([(bottom, orange)], OUT / "renders" / "bottom_deck.png", 45, -125, "Bottom deck with hinge ears")
    render([(arm_fl, dark)], OUT / "renders" / "arm.png", 30, -55, "Arm, front left")
    render([(spoke_to_assembly(spoke), dark), (arc.translate([0, 0, BAND_Z0]), yellow)], OUT / "renders" / "guard_parts.png",
           30, -60, "Guard spoke and one arc")

    print(f"\n{'ALL CHECKS PASSED' if rpt.failed == 0 else str(rpt.failed) + ' CHECK(S) FAILED'}")
    return rpt.failed


if __name__ == "__main__":
    raise SystemExit(main())
