#!/usr/bin/env python3
"""Parametric 600 mm folding quadcopter airframe. Original design.

Run:  python generate_frame.py
Out:  stl/*.stl (print-oriented), renders/*.png, and a check report on stdout.

Assembly coordinates: +X forward, +Y left, +Z up, origin at the top face of the
bottom plate. All lengths are in mm. Change the values in the PARAMETERS block
and re-run; the checks at the end re-validate the result.
"""
import math
from pathlib import Path

import numpy as np
import trimesh
from manifold3d import CrossSection, FillRule, Manifold
from shapely.affinity import translate as sh_translate
from shapely.geometry import LineString, Point, Polygon, box
from shapely.ops import unary_union

# ---------------------------------------------------------------- PARAMETERS
BED = 220.0                 # Ender 3 V3 SE bed (X and Y)
BED_HEIGHT = 250.0
BED_MARGIN = 2.5            # per side, leaves room for a skirt or small brim

DIAGONAL = 600.0            # motor-to-motor
PIVOT = 70.0                # arm pivot at (+/-PIVOT, +/-PIVOT)
ARM_L = DIAGONAL / 2 - PIVOT * math.sqrt(2)  # pivot to motor centre
LOCK = 26.0                 # pivot to lock bolt, along the arm
FOLD_FRONT = 160.0          # folded heading of the front-left arm (deg)
FOLD_REAR = 180.0           # folded heading of the rear-left arm (deg)

GAP = 30.2                  # clear height between the plates
ROOT_H = 30.0               # arm root height (0.2 less than GAP)
PLATE_T = 4.0
EAR_R = 12.0                # plate radius around the lock bolt

ROOT_W, TIP_W = 28.0, 20.0  # arm width at root and motor end
TIP_RISE = 8.0              # underside rises this much toward the motor
KEEL = 6.0                  # 45 deg underside chamfer
WALL = 2.4                  # arm wall thickness (6 lines of a 0.4 mm nozzle)
ROOT_END = 38.0             # solid root ends here, along the arm
CHANNEL_START = 44.0
TIP_SOLID = 30.0            # solid length before the motor centre

PAD_R, PAD_T = 27.0, 6.0    # motor pad radius and thickness
MOTOR_PATTERNS = (25.0, 30.0)  # square bolt patterns; measure your motor
MOTOR_R, MOTOR_H = 29.0, 30.0  # envelope used for the fold check
SHAFT_R = 6.0
PROP_IN = 12.0

M3, M4 = 3.4, 4.3           # clearance hole diameters
HEAD_D = 6.5                # M3 head counterbore

BAY_Y, BAY_WALL_T = 50.0, 3.0
BOLTS_X = (-80.0, -40.0, 0.0, 40.0, 80.0)
BOSS_R = 5.0

HOOP_HOLE_X, HOOP_HOLE_Y = 65.0, 35.0
HOOP_DROP = 130.0           # ground clearance below the bottom plate
HOOP_FOOT_Y = 100.0
HOOP_T, HOOP_STRUT = 12.0, 14.0
FLANGE_T, FLANGE_W = 6.0, 24.0
HOOP_HOLE_W = 17.0          # hole position across the flange

CANOPY_X, CANOPY_Y = 85.0, 46.0
CANOPY_SKIRT, CANOPY_WALL = 4.0, 1.6

# Assumptions for the beam estimate. Replace with measured values.
THRUST_N = 19.6             # 2 kg per motor at full throttle (assumed)
E_MPA = 1800.0              # printed PETG modulus (conservative assumption)
STRENGTH_MPA = 30.0         # printed PETG along the layers (assumption)
TIP_MASS_KG = 0.15          # motor + prop + wiring (assumption)
DENSITY = 1.27              # g/cm3, PETG

OUT = Path(__file__).parent
SEG = 64


# ------------------------------------------------------------------- HELPERS
def cyl(r, z0, z1, x=0.0, y=0.0):
    return Manifold.cylinder(z1 - z0, r, r, SEG).translate([x, y, z0])


def prism(pts, z0, z1):
    """Extrude a 2D polygon (x, y) from z0 to z1."""
    pts = list(pts)
    if Polygon(pts).exterior.is_ccw is False:
        pts.reverse()
    return Manifold.extrude(CrossSection([pts]), z1 - z0).translate([0, 0, z0])


def extrude_shape(shape, z0, z1):
    """Extrude a shapely polygon, holes included."""
    parts = shape.geoms if hasattr(shape, "geoms") else [shape]
    rings = []
    for part in parts:
        rings.append(list(part.exterior.coords)[:-1])
        rings.extend(list(r.coords)[:-1] for r in part.interiors)
    section = CrossSection(rings, fillrule=FillRule.EvenOdd)
    return Manifold.extrude(section, z1 - z0).translate([0, 0, z0])


def union(items):
    return Manifold.batch_boolean(list(items), _op("add"))


def _op(name):
    from manifold3d import OpType
    return {"add": OpType.Add, "sub": OpType.Subtract}[name]


def subtract(base, cutters):
    for cutter in cutters:
        base = base - cutter
    return base


def bbox(m):
    b = m.bounding_box()
    return np.array(b[:3]), np.array(b[3:])


def overlap(a, b):
    return (a ^ b).volume()


def rot_z(m, deg):
    return m.rotate([0, 0, deg])


# ------------------------------------------------------------------- ARM
def arm_t(x):
    return min(max((x - ROOT_END) / (ARM_L - TIP_SOLID - ROOT_END), 0.0), 1.0)


def outer_section(x):
    t = arm_t(x)
    half = (ROOT_W + (TIP_W - ROOT_W) * t) / 2
    zb = TIP_RISE * t
    return [(half, ROOT_H), (half, zb + KEEL), (half - KEEL, zb),
            (KEEL - half, zb), (-half, zb + KEEL), (-half, ROOT_H)]


def inner_section(x):
    t = arm_t(x)
    half = (ROOT_W + (TIP_W - ROOT_W) * t) / 2 - WALL
    peak = TIP_RISE * t + WALL + 2.0
    top = ROOT_H - WALL
    return [(half, top), (half, peak + half), (0.0, peak), (-half, peak + half),
            (-half, top)]


def loft(section, x0, x1):
    points = [(x0, y, z) for y, z in section(x0)]
    points += [(x1, y, z) for y, z in section(x1)]
    return Manifold.hull_points(points)


def make_arm():
    """Arm in local frame: pivot at origin, +X along the arm, flat top at ROOT_H."""
    root_plan = unary_union([Point(0, 0).buffer(ROOT_W / 2, 64),
                             box(0, -ROOT_W / 2, ROOT_END, ROOT_W / 2)])
    root = extrude_shape(root_plan, 0, ROOT_H)
    taper = loft(outer_section, ROOT_END, ARM_L - TIP_SOLID)
    tip = loft(outer_section, ARM_L - TIP_SOLID, ARM_L)
    boss = cyl(12.0, TIP_RISE, ROOT_H, ARM_L)
    pad = cyl(PAD_R, ROOT_H - PAD_T, ROOT_H, ARM_L)
    arm = union([root, taper, tip, boss, pad])

    cuts = [loft(inner_section, CHANNEL_START, ARM_L - TIP_SOLID),
            cyl(M4 / 2, -1, ROOT_H + 1), cyl(M4 / 2, -1, ROOT_H + 1, LOCK),
            cyl(SHAFT_R, -1, ROOT_H + 1, ARM_L),
            cyl(5.0, ROOT_H / 2, ROOT_H + 1, ARM_L - 36.0)]  # motor wire exit

    # Side wire exits sit near the neutral axis, where bending stress is lowest.
    # The 45 deg diamond shape prints without supports.
    xc, zc, r = 55.0, 15.0, 7.0
    diamond = Manifold.extrude(
        CrossSection([[(xc + r, zc), (xc, zc + r), (xc - r, zc), (xc, zc - r)]]),
        60.0).rotate([90, 0, 0]).translate([0, 30, 0])
    cuts.append(diamond)

    # Two motor patterns, turned 45 deg apart so their bolt heads never clash.
    for side, start_deg in zip(MOTOR_PATTERNS, (0.0, 45.0)):
        radius = side * math.sqrt(2) / 2
        for k in range(4):
            a = math.radians(start_deg + 90 * k)
            x, y = ARM_L + radius * math.cos(a), radius * math.sin(a)
            cuts.append(cyl(M3 / 2, ROOT_H - PAD_T - 1, ROOT_H + 1, x, y))
            cuts.append(cyl(HEAD_D / 2, -1, ROOT_H - PAD_T, x, y))
    return subtract(arm, cuts)


def place_arm(arm, sx, sy, heading):
    return rot_z(arm, heading).translate([sx * PIVOT, sy * PIVOT, 0])


# ------------------------------------------------------------------- PLATES
def lock_xy():
    d = PIVOT + LOCK / math.sqrt(2)
    return d


def bolt_points():
    return [(x, sy * BAY_Y) for x in BOLTS_X for sy in (1, -1)]


def arm_bolt_points():
    return [(sx * d, sy * d) for d in (PIVOT, lock_xy()) for sx in (1, -1)
            for sy in (1, -1)]


def hoop_points():
    return [(sx * HOOP_HOLE_X, sy * HOOP_HOLE_Y) for sx in (1, -1) for sy in (1, -1)]


FC_PATTERNS = (30.5, 45.0)


def fc_points():
    return [(sx * p / 2, sy * p / 2) for p in FC_PATTERNS for sx in (1, -1)
            for sy in (1, -1)]


STRAP_SLOTS = [box(sx * 45 - 10, sy * 22 - 2.5, sx * 45 + 10, sy * 22 + 2.5)
               for sx in (1, -1) for sy in (1, -1)]
CABLE_SLOTS = [box(sx * 42 - 7, -16, sx * 42 + 7, 16).buffer(2) for sx in (1, -1)]


def plate_outline():
    d = lock_xy()
    return box(-d, -d, d, d).buffer(EAR_R, 32)


def hex_field(zone, r=10.0, web=3.5):
    """Lightening hexes, kept only where the whole hex fits inside zone."""
    flat = 2 * r * math.cos(math.radians(30))
    dx, dy = flat + web, (flat + web) * math.cos(math.radians(30))
    hexes = []
    for j in range(-8, 9):
        for i in range(-8, 9):
            cx, cy = (i + 0.5 * (j % 2)) * dx, j * dy
            hx = Polygon([(cx + r * math.cos(math.radians(30 + 60 * k)),
                           cy + r * math.sin(math.radians(30 + 60 * k)))
                          for k in range(6)])
            if zone.contains(hx):
                hexes.append(hx)
    return hexes


def base_keepout():
    keep = []
    for x, y in bolt_points():
        keep.append(Point(x, y).buffer(9))
    for sx in (1, -1):
        for sy in (1, -1):
            keep.append(LineString([(sx * PIVOT, sy * PIVOT),
                                    (sx * lock_xy(), sy * lock_xy())]).buffer(19))
    for sy in (1, -1):
        keep.append(box(-120, sy * BAY_Y - 5, 120, sy * BAY_Y + 5))
    return keep


def holes(points, d):
    return [Point(x, y).buffer(d / 2, 32) for x, y in points]


def plate_shape(extra_keepout, cutouts, hole_list):
    outline = plate_outline()
    zone = outline.buffer(-6).difference(unary_union(base_keepout() + extra_keepout))
    remove = hex_field(zone) + cutouts + hole_list
    return outline.difference(unary_union(remove))


def make_top_plate():
    keep = [box(-30, -30, 30, 30)] + [s.buffer(5) for s in CABLE_SLOTS]
    shape = plate_shape(keep, CABLE_SLOTS,
                        holes(bolt_points(), M3) + holes(arm_bolt_points(), M4) +
                        holes(fc_points(), M3))
    return extrude_shape(shape, 0, PLATE_T)


def make_bottom_plate():
    keep = [Point(x, y).buffer(10) for x, y in hoop_points()] + \
           [s.buffer(5) for s in STRAP_SLOTS]
    # The bay bolt holes are drilled once, below, through plate and bosses together.
    # Cutting them here as well leaves two nearly identical surfaces and a bad mesh.
    shape = plate_shape(keep, STRAP_SLOTS,
                        holes(arm_bolt_points(), M4) + holes(hoop_points(), M3))
    plate = extrude_shape(shape, 0, PLATE_T)

    half = lock_xy() + EAR_R - 2
    # Walls and bosses are one 2D shape, sunk 1 mm into the plate. Touching only
    # on a shared flat face leaves sliver triangles that break the STL.
    rails = []
    for sy in (1, -1):
        y = sy * BAY_Y
        rails.append(box(-half, y - BAY_WALL_T / 2, half, y + BAY_WALL_T / 2))
        rails += [Point(x, y).buffer(BOSS_R, 32) for x in BOLTS_X]
    body = plate + extrude_shape(unary_union(rails), PLATE_T - 1, PLATE_T + GAP)

    cuts = [cyl(M3 / 2, -1, PLATE_T + GAP + 1, x, y) for x, y in bolt_points()]
    for xc in (-60.0, -20.0, 20.0, 60.0):   # wire ports through the bay walls
        zc, r = PLATE_T + GAP / 2, 12.0
        port = Manifold.extrude(
            CrossSection([[(xc + r, zc), (xc, zc + r), (xc - r, zc), (xc, zc - r)]]),
            2 * BAY_Y + 20).rotate([90, 0, 0]).translate([0, BAY_Y + 10, 0])
        cuts.append(port)
    return subtract(body, cuts)


# ------------------------------------------------------------------- LANDING GEAR
def make_hoop():
    """Print frame: X = across the drone, Y = down, Z = along the drone.
    The flange rises off the bed, so the hoop prints with no supports."""
    top = LineString([(-55, HOOP_STRUT / 2), (55, HOOP_STRUT / 2)]).buffer(HOOP_STRUT / 2, 16)
    parts = [top]
    for s in (1, -1):
        parts.append(LineString([(s * 52, HOOP_STRUT / 2),
                                 (s * HOOP_FOOT_Y, HOOP_DROP - 9)]).buffer(HOOP_STRUT / 2, 16))
        parts.append(LineString([(s * 30, HOOP_STRUT / 2),
                                 (s * HOOP_FOOT_Y, HOOP_DROP - 9)]).buffer(HOOP_STRUT / 2 - 2, 16))
        parts.append(Point(s * HOOP_FOOT_Y, HOOP_DROP - 9).buffer(9, 32))
    web_shape = unary_union(parts)
    web_shape = web_shape.intersection(box(-200, 0, 200, 400))
    web = extrude_shape(web_shape, 0, HOOP_T)

    flange = Manifold.cube([110, FLANGE_T, FLANGE_W]).translate([-55, 0, 0])
    items = [web, flange]
    for u in (-50.0, -20.0, 20.0, 50.0):  # gussets carry sideways landing loads
        gusset = Manifold.extrude(
            CrossSection([[(FLANGE_T, HOOP_T), (FLANGE_T + 18, HOOP_T),
                           (FLANGE_T, FLANGE_W)]]), 4.0)
        # Map (profile x, profile y, extrude) to (hoop y, hoop z, hoop x).
        gusset = gusset.transform([[0, 0, 1, u - 2], [1, 0, 0, 0], [0, 1, 0, 0]])
        items.append(gusset)
    hoop = union(items)

    r = M3 / 2
    for u in (-HOOP_HOLE_Y, HOOP_HOLE_Y):
        # Teardrop hole: the point faces +Z (up on the printer), no supports.
        tear = [(u + r * math.cos(math.radians(a)), HOOP_HOLE_W + r * math.sin(math.radians(a)))
                for a in range(-225, 46, 15)] + [(u, HOOP_HOLE_W + r * math.sqrt(2))]
        cutter = Manifold.extrude(CrossSection([tear]), FLANGE_T + 2)
        cutter = cutter.transform([[1, 0, 0, 0], [0, 0, 1, -1], [0, 1, 0, 0]])
        hoop = hoop - cutter
    return hoop


def place_hoop(hoop, front):
    """Hoop in assembly position: flange against the underside of the bottom plate."""
    # Print (u, v, w) -> assembly (x = HOLE + 17 - w, y = u, z = -v), with mirror correction.
    s = 1.0 if front else -1.0
    web_x = s * (HOOP_HOLE_X + HOOP_HOLE_W)
    matrix = [[0, 0, -s, web_x],
              [s, 0, 0, 0],
              [0, -1, 0, 0]]
    return hoop.transform(matrix)


# ------------------------------------------------------------------- CANOPY
def canopy_hull(hx, hy, z0, z1, peak):
    ridge = hx - hy
    pts = [(sx * hx, sy * hy, z) for sx in (1, -1) for sy in (1, -1) for z in (z0, z1)]
    pts += [(ridge, 0, peak), (-ridge, 0, peak)]
    return Manifold.hull_points(pts)


def make_canopy():
    """45 deg hip roof: every surface prints open side down without supports."""
    outer = canopy_hull(CANOPY_X, CANOPY_Y, 0, CANOPY_SKIRT, CANOPY_SKIRT + CANOPY_Y)
    ix, iy = CANOPY_X - CANOPY_WALL, CANOPY_Y - CANOPY_WALL
    inner_peak = CANOPY_SKIRT + CANOPY_Y - CANOPY_WALL * math.sqrt(2)
    inner = canopy_hull(ix, iy, -1, inner_peak - iy, inner_peak)
    shell = outer - inner

    lugs = [(sx * 80.0, sy * BAY_Y) for sx in (1, -1) for sy in (1, -1)]
    tabs = []
    for x, y in lugs:
        tab = unary_union([Point(x, y).buffer(BOSS_R + 2, 32),
                           box(x - 7, min(y, y - math.copysign(8, y)), x + 7,
                               max(y, y - math.copysign(8, y)))])
        tabs.append(extrude_shape(tab, 0, 3.0))
    tab_body = union(tabs) - inner
    canopy = shell + tab_body
    return subtract(canopy, [cyl(M3 / 2, -1, 4, x, y) for x, y in lugs])


# ------------------------------------------------------------------- CHECKS
class Report:
    def __init__(self):
        self.failed = 0

    def check(self, label, passed, detail=""):
        self.failed += 0 if passed else 1
        print(f"  [{'PASS' if passed else 'FAIL'}] {label}{'  ' + detail if detail else ''}")


def to_trimesh(m):
    mesh = m.simplify(0.005).to_mesh()
    return trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3],
                           np.asarray(mesh.tri_verts), process=False)


def stl_is_sound(path):
    """Re-read the STL from disk with vertex merging, as a slicer would."""
    mesh = trimesh.load_mesh(path, process=True)
    return mesh.is_watertight and mesh.is_winding_consistent, mesh


def best_bed_rotation(m):
    """Smallest footprint among whole-degree turns, preferring 0 and 45."""
    best = None
    for deg in [0, 45] + list(range(5, 90, 5)):
        lo, hi = bbox(m.rotate([0, 0, deg]))
        span = max(hi[0] - lo[0], hi[1] - lo[1])
        if best is None or span < best[0] - 1e-6:
            best = (span, deg)
    return best[1]


def section_props(points):
    """Area, centroid z and second moment about the centroid for a (y, z) polygon."""
    pts = list(points) + [points[0]]
    a = cz = izz = 0.0
    for (y0, z0), (y1, z1) in zip(pts, pts[1:]):
        c = y0 * z1 - y1 * z0
        a += c
        cz += (z0 + z1) * c
        izz += (z0 ** 2 + z0 * z1 + z1 ** 2) * c
    a /= 2
    cz /= 6 * a
    izz = izz / 12 - a * cz ** 2
    return abs(a), cz, abs(izz)


def beam_estimate():
    """Tip-loaded cantilever from the plate edge out to the motor, by virtual work."""
    x0 = lock_xy()  # placeholder replaced below
    x0 = LOCK + EAR_R
    xs = np.linspace(x0, ARM_L, 400)
    deflection, peak_stress = 0.0, 0.0
    for x in xs:
        _, cz_o, i_o = section_props(outer_section(x))
        a_o = section_props(outer_section(x))[0]
        if CHANNEL_START <= x <= ARM_L - TIP_SOLID:
            a_i, cz_i, i_i = section_props(inner_section(x))
            a = a_o - a_i
            cz = (a_o * cz_o - a_i * cz_i) / a
            inertia = i_o + a_o * (cz_o - cz) ** 2 - i_i - a_i * (cz_i - cz) ** 2
        else:
            cz, inertia = cz_o, i_o
        moment = THRUST_N * (ARM_L - x)
        deflection += moment * (ARM_L - x) / (E_MPA * inertia) * (xs[1] - xs[0])
        c = max(ROOT_H - cz, cz - TIP_RISE * arm_t(x))
        peak_stress = max(peak_stress, moment * c / inertia)
    stiffness = THRUST_N / deflection * 1000  # N/m
    freq = math.sqrt(stiffness / TIP_MASS_KG) / (2 * math.pi)
    return deflection, peak_stress, freq


def min_wall():
    worst = 1e9
    for x in np.linspace(CHANNEL_START, ARM_L - TIP_SOLID, 25):
        worst = min(worst, Polygon(inner_section(x)).exterior.distance(
            Polygon(outer_section(x)).exterior))
    return worst


# ------------------------------------------------------------------- RENDERS
def render(groups, path, elev, azim, title, size=1300):
    """Orthographic software renderer with a real depth buffer."""
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
        for (x, y, z), s in zip(zip(px, py, pz), shade):
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


def render_sections(arm, top, bottom, hoop, path):
    """Flat cut-through views, used to confirm interior geometry."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    def draw(ax, section, title, color="#c9571f"):
        for ring in section.to_polygons():
            pts = np.vstack([ring, ring[0]])
            ax.plot(pts[:, 0], pts[:, 1], color=color, lw=1.1)
        ax.set_aspect("equal")
        ax.set_title(title, fontsize=10)
        ax.grid(alpha=0.25)

    # Rotate so a plane across the arm becomes the slicing plane.
    across = arm.rotate([0, -90, 0])
    fig, axes = plt.subplots(2, 3, figsize=(15, 9))
    draw(axes[0][0], across.slice(60.0), "Arm section, 60 mm from pivot", "#222222")
    draw(axes[0][1], across.slice(130.0), "Arm section, 130 mm from pivot", "#222222")
    side = arm.rotate([90, 0, 0])
    draw(axes[0][2], side.slice(0.0), "Arm lengthwise section, on centre", "#222222")
    draw(axes[1][0], top.slice(PLATE_T / 2), "Top plate")
    draw(axes[1][1], bottom.slice(PLATE_T / 2), "Bottom plate")
    draw(axes[1][2], hoop.slice(HOOP_T / 2), "Landing hoop (flat profile)", "#222222")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)

# ------------------------------------------------------------------- MAIN
def main():
    rpt = Report()
    (OUT / "stl").mkdir(exist_ok=True)
    (OUT / "renders").mkdir(exist_ok=True)

    arm = make_arm()
    top = make_top_plate()
    bottom = make_bottom_plate()
    hoop = make_hoop()
    canopy = make_canopy()

    # Print orientation. The arm goes motor face down, which also points the
    # hollow channel's 45 deg peak upward, so it needs no supports.
    arm_print = arm.rotate([180, 0, 0])
    parts = {
        "arm_x4": (arm_print, 4),
        "top_plate_x1": (top, 1),
        "bottom_plate_x1": (bottom, 1),
        "landing_hoop_x2": (hoop, 2),
        "canopy_x1": (canopy, 1),
    }

    print("\nPART CHECKS (fit, watertight, export)")
    total_solid_g = 0.0
    limit = BED - 2 * BED_MARGIN
    for name, (part, count) in parts.items():
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
        total_solid_g += grams * count
        fits = size[0] <= limit and size[1] <= limit and size[2] <= BED_HEIGHT
        same = abs(reread.volume - placed.volume()) / placed.volume() < 0.002
        rpt.check(f"{name}: {size[0]:.0f} x {size[1]:.0f} x {size[2]:.0f} mm, "
                  f"turned {deg} deg, max {grams:.0f} g each",
                  fits and sound and same,
                  "" if sound and same else "STL DEFECT ON RE-READ")

    # Assembly, with the origin on the bottom plate's top surface (plate z 0..4 moved down).
    bot_asm = bottom.translate([0, 0, -PLATE_T])
    top_asm = top.translate([0, 0, GAP])
    can_asm = canopy.translate([0, 0, GAP + PLATE_T])
    hoop_front = place_hoop(hoop, True).translate([0, 0, -PLATE_T])
    hoop_rear = place_hoop(hoop, False).translate([0, 0, -PLATE_T])
    frame = [bot_asm, top_asm]

    motor_env = cyl(MOTOR_R, ROOT_H, ROOT_H + MOTOR_H, ARM_L)
    arm_motor = arm + motor_env

    corners = [(1, 1), (1, -1), (-1, 1), (-1, -1)]
    deployed_heads = {c: math.degrees(math.atan2(c[1], c[0])) for c in corners}
    folded_heads = {(1, 1): FOLD_FRONT, (1, -1): -FOLD_FRONT,
                    (-1, 1): FOLD_REAR, (-1, -1): -FOLD_REAR}
    deployed = {c: place_arm(arm_motor, c[0], c[1], deployed_heads[c]) for c in corners}
    folded = {c: place_arm(arm_motor, c[0], c[1], folded_heads[c]) for c in corners}

    print("\nASSEMBLY CHECKS")
    for label, state in (("deployed", deployed), ("folded", folded)):
        worst = 0.0
        keys = list(state)
        for i, key in enumerate(keys):
            for other in keys[i + 1:]:
                worst = max(worst, overlap(state[key], state[other]))
            for body in frame:
                worst = max(worst, overlap(state[key], body))
        rpt.check(f"{label}: arms and motors clear each other and the frame",
                  worst < 0.5, f"worst overlap {worst:.2f} mm3")

    # Check the whole swing, so an arm cannot jam part-way through folding.
    worst_swing = 0.0
    for c in corners:
        start, end = deployed_heads[c], folded_heads[c]
        delta = (end - start + 180) % 360 - 180
        if abs(delta) < 1e-6:
            continue
        for step in np.linspace(0, 1, 25):
            pose = place_arm(arm_motor, c[0], c[1], start + delta * step)
            for body in frame:
                worst_swing = max(worst_swing, overlap(pose, body))
    rpt.check("arms swing from deployed to folded without hitting the frame",
              worst_swing < 0.5, f"worst overlap {worst_swing:.2f} mm3")

    rpt.check("canopy clears the top plate (bolted flush)",
              overlap(can_asm, top_asm) < 0.5, f"overlap {overlap(can_asm, top_asm):.2f} mm3")

    hoop_top = max(bbox(hoop_front)[1][2], bbox(hoop_rear)[1][2])
    hoop_low = min(bbox(hoop_front)[0][2], bbox(hoop_rear)[0][2])
    rpt.check("hoops seat under the bottom plate",
              abs(hoop_top + PLATE_T) < 0.01 and overlap(hoop_front, bot_asm) < 0.5,
              f"plate-to-ground {-hoop_low - PLATE_T:.0f} mm")

    bore_ok = True
    for sx in (1, -1):
        s = sx * HOOP_HOLE_X
        for sy in (1, -1):
            probe = cyl(0.8, -PLATE_T - FLANGE_T - 2, 2, s, sy * HOOP_HOLE_Y)
            body = hoop_front if sx > 0 else hoop_rear
            bore_ok &= overlap(probe, body) < 1e-6 and overlap(probe, bot_asm) < 1e-6
    rpt.check("hoop bolt holes line up with the plate holes", bore_ok)

    lug_ok = all(overlap(cyl(0.8, GAP - 1, GAP + PLATE_T + 4, sx * 80.0, sy * BAY_Y),
                         union([top_asm, can_asm])) < 1e-6
                 for sx in (1, -1) for sy in (1, -1))
    rpt.check("canopy bolt holes line up with the top plate", lug_ok)

    prop_r = PROP_IN * 25.4 / 2
    prop_z = ROOT_H + MOTOR_H + 8
    discs = [Manifold.cylinder(4, prop_r, prop_r, 128).translate(
        [c[0] * DIAGONAL / 2 / math.sqrt(2), c[1] * DIAGONAL / 2 / math.sqrt(2), prop_z])
        for c in corners]
    tip_gap = DIAGONAL / math.sqrt(2) - 2 * prop_r
    hits = max(overlap(d, b) for d in discs for b in [can_asm, top_asm])
    rpt.check(f"{PROP_IN} in props clear each other and the canopy", tip_gap > 25 and hits < 0.5,
              f"tip-to-tip {tip_gap:.0f} mm, canopy overlap {hits:.1f} mm3")

    wall = min_wall()
    rpt.check("arm wall stays at least 2 mm thick", wall >= 2.0, f"minimum {wall:.2f} mm")

    defl, stress, freq = beam_estimate()
    margin = STRENGTH_MPA / stress
    rpt.check("arm bending safety factor at least 3 (estimate)", margin >= 3,
              f"{stress:.1f} MPa, factor {margin:.1f}")
    print(f"\nESTIMATES at {THRUST_N} N assumed thrust, E = {E_MPA:.0f} MPa")
    print(f"  arm tip deflection  {defl:.2f} mm")
    print(f"  arm bending mode    {freq:.0f} Hz")
    print(f"  printed frame mass  up to {total_solid_g:.0f} g (solid parts, excludes bolts)")

    # Footprints
    def footprint(pieces):
        lo = np.min([bbox(p)[0] for p in pieces], axis=0)
        hi = np.max([bbox(p)[1] for p in pieces], axis=0)
        return hi - lo

    frame_all = frame + [can_asm, hoop_front, hoop_rear]
    fold_size = footprint(frame_all + list(folded.values()))
    dep_size = footprint(frame_all + list(deployed.values()))
    print(f"  deployed envelope   {dep_size[0]:.0f} x {dep_size[1]:.0f} x {dep_size[2]:.0f} mm (no props)")
    print(f"  folded envelope     {fold_size[0]:.0f} x {fold_size[1]:.0f} x {fold_size[2]:.0f} mm (no props)")

    # Renders use the arm without the motor envelope, plus a separate motor colour.
    arm_col, plate_col, accent, motor_col = (70, 76, 84), (214, 96, 40), (232, 178, 70), (165, 170, 178)

    def scene(heads):
        arms = union([place_arm(arm, c[0], c[1], heads[c]) for c in corners])
        motors = union([place_arm(motor_env, c[0], c[1], heads[c]) for c in corners])
        return [(arms, arm_col), (union(frame), plate_col), (can_asm, accent),
                (union([hoop_front, hoop_rear]), arm_col), (motors, motor_col)]

    print("\nRENDERING")
    render(scene(deployed_heads), OUT / "renders" / "deployed.png", 28, -125,
           "Deployed: 600 mm motor to motor")
    render(scene(folded_heads), OUT / "renders" / "folded.png", 55, -125, "Folded for transport")
    render([(arm.rotate([180, 0, 0]), arm_col)], OUT / "renders" / "arm_as_printed.png", 40, -125,
           "Arm as printed (motor face down)")
    render([(bottom, plate_col)], OUT / "renders" / "bottom_plate.png", 50, -125, "Bottom plate")
    render_sections(arm, top, bottom, hoop, OUT / "renders" / "sections.png")

    print(f"\n{'ALL CHECKS PASSED' if rpt.failed == 0 else str(rpt.failed) + ' CHECK(S) FAILED'}")
    return rpt.failed


if __name__ == "__main__":
    raise SystemExit(main())
