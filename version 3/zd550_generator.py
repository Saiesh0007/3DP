#!/usr/bin/env python3
"""
ZD550-style quadcopter frame: fabrication files from open-source libraries only.

Outputs (in ./output next to this script):
    zd550_top_plate.dxf / .step        core chassis plate (top)
    zd550_bottom_plate.dxf / .step     core chassis plate (bottom, same profile)
    zd550_arm.step                     hollow boom tube, cut to computed length
    zd550_arm_cut_sheet.dxf            tube cross-section and cut length, for the workshop
    zd550_motor_mount.dxf / .step      motor mount endplate

DXF files are 2D profiles in millimetres with true lines, arcs and circles, ready
for laser, waterjet or CNC routing. STEP files are 3D solids for any CAD package.

Install:   pip install cadquery ezdxf
Run:       python zd550_generator.py
"""
import math
from pathlib import Path

import cadquery as cq
import ezdxf
from ezdxf import units

# =============================================================================
# EDITABLE PARAMETERS (all lengths in mm)
# =============================================================================

# --- 1. Core chassis plates (top and bottom share one profile) --------------
PLATE_SIZE = 150.0              # square outline, edge to edge
PLATE_THICKNESS = 1.5
CORNER_CHAMFER = 20.0           # leg length of each 45 deg corner relief cut
INNER_GRID = 30.0               # square bolt grid, hole centre to hole centre
INNER_HOLE_D = 2.0
OUTER_GRID = 47.0               # square flight-controller grid
OUTER_HOLE_D = 3.0
# Holes that bolt each folding-arm hinge to the plates: two per corner, on the
# 45 deg diagonal, measured from the plate centre. Set to () for none.
HINGE_HOLE_RADII = (72.0, 84.0)
HINGE_HOLE_D = 3.0

# --- 2. Boom arms -------------------------------------------------------------
TUBE_OD = 20.0
TUBE_WALL = 1.0
TUBE_ID = TUBE_OD - 2 * TUBE_WALL           # 18.0
WHEELBASE = 550.0                           # diagonal, motor axis to motor axis
HINGE_PIVOT_RADIUS = 95.0       # plate centre to hinge pivot, along the diagonal
HINGE_TUBE_START = 12.0         # pivot to the inner end of the tube, inside the hinge clamp
TUBE_PAST_MOTOR = 12.0          # tube runs this far past the motor axis into the mount clamp

# --- 3. Motor mount endplates -------------------------------------------------
MOUNT_THICKNESS = 1.5
MOUNT_DIAMETER = 50.0
SHAFT_CLEARANCE_D = 8.0         # centre hole for the shaft and circlip
SLOT_R_MIN, SLOT_R_MAX = 8.0, 16.0  # radial slots: fit bolt circles of 16 to 32 mm
SLOT_WIDTH = 3.2                # M3 clearance
# Extra fixed holes, set 45 deg off the slots. Keep to one diameter: two patterns
# on the same 45 deg lines (for example 25 and 30 mm) overlap and merge.
FIXED_PATTERN_DIAMETERS = (25.0,)
FIXED_HOLE_D = 3.2
CLAMP_HOLE_X, CLAMP_HOLE_Y = 15.0, 13.5  # tube clamp strap holes, either side of the tube
CLAMP_HOLE_D = 3.2

OUT = Path(__file__).resolve().parent / "output"

# =============================================================================
# BOOM LENGTH
# =============================================================================
# The wheelbase is measured across the diagonal, from one motor axis to the
# opposite one. Each arm therefore puts its motor at
#       R_motor = WHEELBASE / 2 = 275 mm  from the plate centre.
#
# The arms lie on the plate diagonals, so every length below is measured along
# the same straight line from the plate centre:
#       inner tube end  R_in  = HINGE_PIVOT_RADIUS + HINGE_TUBE_START  = 95 + 12 = 107 mm
#       outer tube end  R_out = R_motor + TUBE_PAST_MOTOR              = 275 + 12 = 287 mm
#
#       TUBE_LENGTH = R_out - R_in = 287 - 107 = 180 mm
#
# The plate does not set this directly. Its half-diagonal is
#       PLATE_SIZE / 2 * sqrt(2) = 106.07 mm,
# and each corner chamfer removes CORNER_CHAMFER / sqrt(2) = 14.14 mm of that,
# leaving the plate corner at 91.92 mm. The hinge pivot (95 mm) therefore sits
# 3 mm beyond the chamfered corner, so the arm can swing clear.
R_MOTOR = WHEELBASE / 2
R_TUBE_IN = HINGE_PIVOT_RADIUS + HINGE_TUBE_START
R_TUBE_OUT = R_MOTOR + TUBE_PAST_MOTOR
TUBE_LENGTH = R_TUBE_OUT - R_TUBE_IN
PLATE_CORNER_RADIUS = PLATE_SIZE / 2 * math.sqrt(2) - CORNER_CHAMFER / math.sqrt(2)


# =============================================================================
# 2D PROFILES (one definition, used for both DXF and STEP)
# =============================================================================
def square_grid(pitch):
    h = pitch / 2
    return [(h, h), (-h, h), (-h, -h), (h, -h)]


def plate_outline():
    """150 mm square with a 45 deg relief cut across each corner."""
    h, c = PLATE_SIZE / 2, CORNER_CHAMFER
    return [(h - c, -h), (h, -h + c), (h, h - c), (h - c, h),
            (-h + c, h), (-h, h - c), (-h, -h + c), (-h + c, -h)]


def plate_holes():
    holes = [(x, y, INNER_HOLE_D) for x, y in square_grid(INNER_GRID)]
    holes += [(x, y, OUTER_HOLE_D) for x, y in square_grid(OUTER_GRID)]
    for r in HINGE_HOLE_RADII:
        d = r / math.sqrt(2)
        holes += [(sx * d, sy * d, HINGE_HOLE_D) for sx in (1, -1) for sy in (1, -1)]
    return holes


def mount_holes():
    holes = [(0.0, 0.0, SHAFT_CLEARANCE_D)]
    for dia in FIXED_PATTERN_DIAMETERS:
        r = dia / 2
        holes += [(r * math.cos(math.radians(a)), r * math.sin(math.radians(a)), FIXED_HOLE_D)
                  for a in (45, 135, 225, 315)]
    holes += [(sx * CLAMP_HOLE_X, sy * CLAMP_HOLE_Y, CLAMP_HOLE_D) for sx in (1, -1) for sy in (1, -1)]
    return holes


def mount_slots():
    """Radial slots on 0/90/180/270 deg: (centre x, centre y, length, width, angle)."""
    mid, length = (SLOT_R_MIN + SLOT_R_MAX) / 2, SLOT_R_MAX - SLOT_R_MIN + SLOT_WIDTH
    return [(mid * math.cos(math.radians(a)), mid * math.sin(math.radians(a)), length, SLOT_WIDTH, a)
            for a in (0, 90, 180, 270)]


# =============================================================================
# DXF OUTPUT (ezdxf)
# =============================================================================
def new_dxf():
    doc = ezdxf.new("R2010", setup=True)
    doc.units = units.MM
    doc.header["$MEASUREMENT"] = 1          # metric
    doc.layers.add("CUT_OUTER", color=1)    # red: outside profile
    doc.layers.add("CUT_INNER", color=5)    # blue: holes and slots
    doc.layers.add("NOTES", color=7)
    return doc


def add_slot(msp, cx, cy, length, width, angle):
    """Stadium slot as two lines and two arcs, so it stays editable as true geometry."""
    r, half = width / 2, (length - width) / 2
    ux, uy = math.cos(math.radians(angle)), math.sin(math.radians(angle))
    vx, vy = -uy, ux
    a = (cx + ux * half, cy + uy * half)
    b = (cx - ux * half, cy - uy * half)
    attr = {"layer": "CUT_INNER"}
    msp.add_line((a[0] + vx * r, a[1] + vy * r), (b[0] + vx * r, b[1] + vy * r), dxfattribs=attr)
    msp.add_line((a[0] - vx * r, a[1] - vy * r), (b[0] - vx * r, b[1] - vy * r), dxfattribs=attr)
    msp.add_arc(a, r, angle - 90, angle + 90, dxfattribs=attr)
    msp.add_arc(b, r, angle + 90, angle + 270, dxfattribs=attr)


def write_plate_dxf(path, label):
    doc = new_dxf()
    msp = doc.modelspace()
    msp.add_lwpolyline(plate_outline(), close=True, dxfattribs={"layer": "CUT_OUTER"})
    for x, y, d in plate_holes():
        msp.add_circle((x, y), d / 2, dxfattribs={"layer": "CUT_INNER"})
    msp.add_text(f"{label}  {PLATE_THICKNESS} mm  qty 1", height=3,
                 dxfattribs={"layer": "NOTES"}).set_placement((-PLATE_SIZE / 2, -PLATE_SIZE / 2 - 8))
    doc.saveas(path)


def write_mount_dxf(path):
    doc = new_dxf()
    msp = doc.modelspace()
    msp.add_circle((0, 0), MOUNT_DIAMETER / 2, dxfattribs={"layer": "CUT_OUTER"})
    for x, y, d in mount_holes():
        msp.add_circle((x, y), d / 2, dxfattribs={"layer": "CUT_INNER"})
    for slot in mount_slots():
        add_slot(msp, *slot)
    msp.add_text(f"MOTOR MOUNT  {MOUNT_THICKNESS} mm  qty 4", height=3,
                 dxfattribs={"layer": "NOTES"}).set_placement((-MOUNT_DIAMETER / 2, -MOUNT_DIAMETER / 2 - 8))
    doc.saveas(path)


def write_arm_cut_sheet(path):
    """Tube section and length: the tube is bought, not cut from sheet."""
    doc = new_dxf()
    msp = doc.modelspace()
    msp.add_circle((0, 0), TUBE_OD / 2, dxfattribs={"layer": "CUT_OUTER"})
    msp.add_circle((0, 0), TUBE_ID / 2, dxfattribs={"layer": "CUT_INNER"})
    x0 = 20.0
    msp.add_lwpolyline([(x0, -TUBE_OD / 2), (x0 + TUBE_LENGTH, -TUBE_OD / 2),
                        (x0 + TUBE_LENGTH, TUBE_OD / 2), (x0, TUBE_OD / 2)], close=True,
                       dxfattribs={"layer": "CUT_OUTER"})
    dim = msp.add_linear_dim(base=(x0, TUBE_OD / 2 + 10), p1=(x0, TUBE_OD / 2), p2=(x0 + TUBE_LENGTH, TUBE_OD / 2),
                             dimstyle="EZDXF", override={"dimtxt": 5, "dimasz": 3, "dimexe": 2, "dimexo": 1,
                                       "dimlfac": 1.0, "dimdec": 1})  # EZDXF style scales by 100 otherwise
    dim.render()
    msp.add_text(f"BOOM TUBE  OD {TUBE_OD} x ID {TUBE_ID}  cut {TUBE_LENGTH:.1f} mm  qty 4", height=3,
                 dxfattribs={"layer": "NOTES"}).set_placement((-TUBE_OD / 2, -TUBE_OD / 2 - 10))
    doc.saveas(path)


# =============================================================================
# STEP OUTPUT (CadQuery)
# =============================================================================
def plate_solid():
    body = cq.Workplane("XY").polyline(plate_outline()).close().extrude(PLATE_THICKNESS)
    for d in sorted({d for *_, d in plate_holes()}):
        pts = [(x, y) for x, y, dd in plate_holes() if dd == d]
        body = body.faces(">Z").workplane().pushPoints(pts).hole(d)
    return body


def mount_solid():
    body = cq.Workplane("XY").circle(MOUNT_DIAMETER / 2).extrude(MOUNT_THICKNESS)
    for d in sorted({d for *_, d in mount_holes()}):
        pts = [(x, y) for x, y, dd in mount_holes() if dd == d]
        body = body.faces(">Z").workplane().pushPoints(pts).hole(d)
    for cx, cy, length, width, angle in mount_slots():
        cutter = (cq.Workplane("XY").center(cx, cy).slot2D(length, width, angle)
                  .extrude(MOUNT_THICKNESS * 3).translate((0, 0, -MOUNT_THICKNESS)))
        body = body.cut(cutter)
    return body


def arm_solid():
    """Tube along +X, starting at x = 0."""
    return cq.Workplane("YZ").circle(TUBE_OD / 2).circle(TUBE_ID / 2).extrude(TUBE_LENGTH)


# =============================================================================
# MAIN
# =============================================================================
def main():
    OUT.mkdir(exist_ok=True)
    print(f"Boom tube length: {TUBE_LENGTH:.1f} mm  "
          f"(motor at {R_MOTOR:.1f} mm, tube {R_TUBE_IN:.1f} to {R_TUBE_OUT:.1f} mm from centre)")
    print(f"Chamfered plate corner at {PLATE_CORNER_RADIUS:.2f} mm, hinge pivot at {HINGE_PIVOT_RADIUS:.2f} mm")

    for name in ("top", "bottom"):
        write_plate_dxf(OUT / f"zd550_{name}_plate.dxf", f"{name.upper()} PLATE")
        cq.exporters.export(plate_solid(), str(OUT / f"zd550_{name}_plate.step"))
    write_mount_dxf(OUT / "zd550_motor_mount.dxf")
    cq.exporters.export(mount_solid(), str(OUT / "zd550_motor_mount.step"))
    cq.exporters.export(arm_solid(), str(OUT / "zd550_arm.step"))
    write_arm_cut_sheet(OUT / "zd550_arm_cut_sheet.dxf")

    for f in sorted(OUT.iterdir()):
        print(f"  wrote {f.name}")


if __name__ == "__main__":
    main()
