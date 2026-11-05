#!/usr/bin/env python3
"""Export real CAD solids (STEP) for SolidWorks from each version's generate_frame.py.

The design scripts stay the single source of truth. This file supplies a small
stand-in for the mesh library they import, backed by the OpenCascade CAD kernel,
so their geometry comes out with true planes, cylinders and arcs instead of triangles.

Setup (Python 3.12):  pip install cadquery-ocp shapely numpy trimesh scipy
Run:                  python export_step.py
Out:                  version N/step/*.step, one per printed part, plus assembly_deployed.step
"""
import importlib.util
import math
import sys
import types
from pathlib import Path

import numpy as np
from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.BRepAlgoAPI import BRepAlgoAPI_Common, BRepAlgoAPI_Cut, BRepAlgoAPI_Fuse
from OCP.BRepBndLib import BRepBndLib
from OCP.BRepBuilderAPI import (BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeFace, BRepBuilderAPI_MakePolygon,
                                BRepBuilderAPI_MakeSolid, BRepBuilderAPI_MakeWire, BRepBuilderAPI_Sewing,
                                BRepBuilderAPI_Transform)
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.BRepGProp import BRepGProp
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.BRepPrimAPI import (BRepPrimAPI_MakeBox, BRepPrimAPI_MakeCone, BRepPrimAPI_MakeCylinder,
                             BRepPrimAPI_MakePrism, BRepPrimAPI_MakeRevol)
from OCP.Bnd import Bnd_Box
from OCP.GC import GC_MakeArcOfCircle
from OCP.GProp import GProp_GProps
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane
from OCP.Geom import Geom_Circle
from OCP.IFSelect import IFSelect_RetDone
from OCP.Interface import Interface_Static
from OCP.STEPControl import STEPControl_AsIs, STEPControl_Reader, STEPControl_Writer
from OCP.ShapeFix import ShapeFix_Face, ShapeFix_Solid
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.StlAPI import StlAPI_Writer
from OCP.TopAbs import TopAbs_FACE, TopAbs_SOLID
from OCP.TopExp import TopExp_Explorer
from OCP.TopoDS import TopoDS, TopoDS_Compound
from OCP.gp import gp_Ax1, gp_Ax2, gp_Circ, gp_Dir, gp_Pnt, gp_Trsf, gp_Vec
from scipy.spatial import ConvexHull
from shapely.geometry import Point, Polygon

ROOT = Path(__file__).resolve().parent
FUZZ = 1e-5
ARC_TOL = 0.03        # polygon points may lie this far from a fitted circle
ARC_MAX_TURN = 26.0   # steeper corners than this per vertex are real corners, not arcs
ARC_MIN_SEGMENTS = 3


# ------------------------------------------------------------- polyline to arcs
def _circle3(a, b, c):
    ax, ay, bx, by, cx, cy = *a, *b, *c
    d = 2 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by))
    if abs(d) < 1e-9:
        return None
    s_a, s_b, s_c = ax * ax + ay * ay, bx * bx + by * by, cx * cx + cy * cy
    ux = (s_a * (by - cy) + s_b * (cy - ay) + s_c * (ay - by)) / d
    uy = (s_a * (cx - bx) + s_b * (ax - cx) + s_c * (bx - ax)) / d
    return ux, uy, math.hypot(ax - ux, ay - uy)


def _turn(p0, p1, p2):
    a1 = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
    a2 = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
    return math.degrees((a2 - a1 + math.pi) % (2 * math.pi) - math.pi)


def _clean(ring):
    pts = []
    for p in ring:
        p = (float(p[0]), float(p[1]))
        if not pts or math.dist(p, pts[-1]) > 1e-7:
            pts.append(p)
    if math.dist(pts[0], pts[-1]) <= 1e-7:
        pts.pop()
    return pts


def _is_arc(pts):
    """True if consecutive points follow one circle with a steady, shallow turn."""
    if len(pts) < ARC_MIN_SEGMENTS + 1:
        return None
    circle = _circle3(pts[0], pts[len(pts) // 2], pts[-1])
    if circle is None:
        return None
    ux, uy, r = circle
    if any(abs(math.hypot(x - ux, y - uy) - r) > ARC_TOL for x, y in pts):
        return None
    # Arcs have evenly spaced points. A long segment means a straight edge, not an arc;
    # only the end segments may be shorter, where an arc was cut by another shape.
    lengths = sorted(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
    if lengths[-1] > 1.25 * lengths[len(lengths) // 2]:
        return None
    turns = [_turn(pts[i], pts[i + 1], pts[i + 2]) for i in range(len(pts) - 2)]
    if not turns or any(abs(t) > ARC_MAX_TURN or abs(t) < 0.3 for t in turns) or any(t * turns[0] < 0 for t in turns):
        return None
    return circle


def _ring_to_wire(ring, plane):
    """Closed 2D ring to a wire, using true arcs where the points trace a circle.

    plane maps (a, b) to a 3D gp_Pnt. Only z = 0 rings use full circle edges.
    """
    pts = _clean(ring)
    n = len(pts)
    closed = pts + pts[:2]
    if _is_arc(closed) and n >= 8:
        ux, uy, r = _is_arc(closed)
        centre, axis = plane((ux, uy)), gp_Dir(0, 0, 1)
        edge = BRepBuilderAPI_MakeEdge(gp_Circ(gp_Ax2(centre, axis), r)).Edge()
        return BRepBuilderAPI_MakeWire(edge).Wire()

    # Start on a real corner, so no arc is split across the ring's seam.
    start = 0
    for i in range(n):
        if abs(_turn(pts[i - 1], pts[i], pts[(i + 1) % n])) > ARC_MAX_TURN:
            start = i
            break
    else:
        for i in range(n):
            if abs(math.dist(pts[i - 1], pts[i]) - math.dist(pts[i], pts[(i + 1) % n])) > 1e-3:
                start = i
                break
    pts = pts[start:] + pts[:start]
    pts.append(pts[0])

    wire = BRepBuilderAPI_MakeWire()
    i = 0
    last = len(pts) - 1
    while i < last:
        j = i + 1
        best = None
        while j + 1 <= last:
            span = pts[i:j + 2]
            if len(span) > ARC_MIN_SEGMENTS and not _is_arc(span):
                break
            if _is_arc(span):
                best = j + 1
            j += 1
        if best is not None:
            mid = (i + best) // 2
            curve = GC_MakeArcOfCircle(plane(pts[i]), plane(pts[mid]), plane(pts[best])).Value()
            wire.Add(BRepBuilderAPI_MakeEdge(curve).Edge())
            i = best
        else:
            wire.Add(BRepBuilderAPI_MakeEdge(plane(pts[i]), plane(pts[i + 1])).Edge())
            i += 1
    return wire.Wire()


def _face_from_rings(rings, plane=lambda p: gp_Pnt(p[0], p[1], 0)):
    """Faces from even-odd rings: depth 0 rings are outlines, depth 1 rings are holes."""
    polys = [Polygon(_clean(r)) for r in rings]
    parents = [[j for j, q in enumerate(polys) if j != i and q.area > polys[i].area and q.contains(polys[i])]
               for i in range(len(polys))]
    faces = []
    for i, ring in enumerate(rings):
        if len(parents[i]) % 2:
            continue
        maker = BRepBuilderAPI_MakeFace(_ring_to_wire(ring, plane), True)
        for k, other in enumerate(rings):
            if i in parents[k] and len(parents[k]) == len(parents[i]) + 1:
                maker.Add(TopoDS.Wire(_ring_to_wire(other, plane).Reversed()))
        fix = ShapeFix_Face(maker.Face())
        fix.FixOrientation()
        fix.Perform()
        faces.append(TopoDS.Face(fix.Face()))
    return faces


# ------------------------------------------------------------- Manifold stand-in
def _compound(shapes):
    builder, comp = BRep_Builder(), TopoDS_Compound()
    builder.MakeCompound(comp)
    for s in shapes:
        builder.Add(comp, s)
    return comp


def _unify(shape):
    u = ShapeUpgrade_UnifySameDomain(shape, True, True, True)
    u.AllowInternalEdges(False)
    u.Build()
    return u.Shape()


def _bool(op, a, b):
    from OCP.collections import List_TopoDS_Shape
    args, tools = List_TopoDS_Shape(), List_TopoDS_Shape()
    args.Append(a)
    tools.Append(b)
    algo = op()
    algo.SetArguments(args)
    algo.SetTools(tools)
    algo.SetFuzzyValue(FUZZ)
    algo.SetRunParallel(True)
    algo.Build()
    if not algo.IsDone():
        raise RuntimeError("boolean failed")
    return _unify(algo.Shape())


class CrossSection:
    def __init__(self, rings, fillrule=None):
        rings = [[tuple(map(float, p)) for p in r] for r in rings]
        # Boolean leftovers can be degenerate slivers with no printable size.
        self.rings = [r for r in rings if len(_clean(r)) >= 3 and Polygon(_clean(r)).area > 0.01]


class Manifold:
    def __init__(self, shape):
        self.shape = shape

    # --- constructors
    @staticmethod
    def cube(size, center=False):
        m = Manifold(BRepPrimAPI_MakeBox(float(size[0]), float(size[1]), float(size[2])).Shape())
        return m.translate([-size[0] / 2, -size[1] / 2, -size[2] / 2]) if center else m

    @staticmethod
    def cylinder(height, r_low, r_high=-1.0, segments=0, center=False):
        r_high = r_low if r_high < 0 else r_high
        ax = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))
        if abs(r_low - r_high) < 1e-9:
            shape = BRepPrimAPI_MakeCylinder(ax, float(r_low), float(height)).Shape()
        else:
            shape = BRepPrimAPI_MakeCone(ax, float(r_low), float(r_high), float(height)).Shape()
        m = Manifold(shape)
        return m.translate([0, 0, -height / 2]) if center else m

    @staticmethod
    def extrude(section, height):
        faces = _face_from_rings(section.rings)
        solids = [BRepPrimAPI_MakePrism(f, gp_Vec(0, 0, float(height))).Shape() for f in faces]
        shape = solids[0]
        for s in solids[1:]:
            shape = _bool(BRepAlgoAPI_Fuse, shape, s)
        return Manifold(shape)

    @staticmethod
    def revolve(section, segments=0, degrees=360.0):
        face = _face_from_rings(section.rings, plane=lambda p: gp_Pnt(p[0], 0, p[1]))[0]
        axis = gp_Ax1(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1))
        return Manifold(BRepPrimAPI_MakeRevol(face, axis, math.radians(degrees)).Shape())

    @staticmethod
    def hull_points(points):
        pts = np.asarray(points, float)
        hull = ConvexHull(pts)
        centre = pts[hull.vertices].mean(axis=0)
        sew = BRepBuilderAPI_Sewing(1e-4)
        for tri, eq in zip(hull.simplices, hull.equations):
            a, b, c = pts[tri]
            if np.dot(np.cross(b - a, c - a), eq[:3]) < 0:
                b, c = c, b
            poly = BRepBuilderAPI_MakePolygon(gp_Pnt(*a), gp_Pnt(*b), gp_Pnt(*c), True)
            sew.Add(BRepBuilderAPI_MakeFace(poly.Wire(), True).Face())
        sew.Perform()
        solid = BRepBuilderAPI_MakeSolid(TopoDS.Shell(sew.SewedShape())).Solid()
        fix = ShapeFix_Solid(solid)
        fix.Perform()
        return Manifold(_unify(fix.Solid()))

    @staticmethod
    def batch_boolean(items, op):
        items = list(items)
        shape = items[0].shape
        for it in items[1:]:
            shape = _bool(BRepAlgoAPI_Fuse, shape, it.shape)
        return Manifold(shape)

    # --- booleans
    def __add__(self, other):
        return Manifold(_bool(BRepAlgoAPI_Fuse, self.shape, other.shape))

    def __sub__(self, other):
        return Manifold(_bool(BRepAlgoAPI_Cut, self.shape, other.shape))

    def __xor__(self, other):
        return Manifold(_bool(BRepAlgoAPI_Common, self.shape, other.shape))

    # --- transforms
    def _apply(self, trsf):
        return Manifold(BRepBuilderAPI_Transform(self.shape, trsf, True).Shape())

    def translate(self, v):
        t = gp_Trsf()
        t.SetTranslation(gp_Vec(float(v[0]), float(v[1]), float(v[2])))
        return self._apply(t)

    def rotate(self, angles):
        m = self
        for deg, axis in zip(angles, (gp_Dir(1, 0, 0), gp_Dir(0, 1, 0), gp_Dir(0, 0, 1))):
            if abs(deg) > 1e-12:
                t = gp_Trsf()
                t.SetRotation(gp_Ax1(gp_Pnt(0, 0, 0), axis), math.radians(float(deg)))
                m = m._apply(t)
        return m

    def mirror(self, normal):
        t = gp_Trsf()
        t.SetMirror(gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(float(normal[0]), float(normal[1]), float(normal[2]))))
        return self._apply(t)

    def transform(self, m):
        t = gp_Trsf()
        t.SetValues(*[float(x) for row in m for x in row])
        return self._apply(t)

    def simplify(self, *_):
        return self

    # --- queries
    def volume(self):
        props = GProp_GProps()
        BRepGProp.VolumeProperties_s(self.shape, props)
        return props.Mass()

    def bounding_box(self):
        box = Bnd_Box()
        BRepBndLib.AddOptimal_s(self.shape, box, True, False)
        lo, hi = box.CornerMin(), box.CornerMax()
        return (lo.X(), lo.Y(), lo.Z(), hi.X(), hi.Y(), hi.Z())


def install_shim():
    shim = types.ModuleType("manifold3d")
    shim.Manifold, shim.CrossSection = Manifold, CrossSection
    shim.FillRule = types.SimpleNamespace(EvenOdd=1)
    shim.OpType = types.SimpleNamespace(Add=0, Subtract=1)
    sys.modules["manifold3d"] = shim


def load(version_dir, name):
    spec = importlib.util.spec_from_file_location(name, version_dir / "generate_frame.py")
    module = importlib.util.module_from_spec(spec)
    sys.dont_write_bytecode = True
    spec.loader.exec_module(module)
    return module


# ------------------------------------------------------------- parts per version
def v1_parts(g):
    arm, top, bottom, hoop, canopy = g.make_arm(), g.make_top_plate(), g.make_bottom_plate(), g.make_hoop(), g.make_canopy()
    parts = {"arm_x4": arm.rotate([180, 0, 0]), "top_plate_x1": top, "bottom_plate_x1": bottom,
             "landing_hoop_x2": hoop, "canopy_x1": canopy}
    assembly = [bottom.translate([0, 0, -g.PLATE_T]), top.translate([0, 0, g.GAP]),
                canopy.translate([0, 0, g.GAP + g.PLATE_T]),
                g.place_hoop(hoop, True).translate([0, 0, -g.PLATE_T]), g.place_hoop(hoop, False).translate([0, 0, -g.PLATE_T])]
    for c in ((1, 1), (1, -1), (-1, 1), (-1, -1)):
        assembly.append(g.place_arm(arm, c[0], c[1], math.degrees(math.atan2(c[1], c[0]))))
    return parts, assembly


def v2_parts(g):
    arm, spoke, arc = g.make_arm_fl(), g.make_spoke(), g.make_arc()
    bottom, top, upper = g.make_bottom_deck(), g.make_top_deck(), g.make_upper_deck()
    sensor, leg, camera = g.make_sensor_mount(), g.make_leg(), g.make_camera_mount()
    parts = {"arm_A_x2": arm.rotate([180, 0, 0]), "arm_B_x2": arm.mirror([0, 1, 0]).rotate([180, 0, 0]),
             "bottom_deck_x1": bottom, "top_deck_x1": top, "upper_deck_x1": upper, "sensor_mount_x1": sensor,
             "leg_x4": leg, "camera_mount_x1": camera, "guard_arc_x16": arc, "guard_spoke_x16": spoke}
    assembly = [bottom, top.translate([0, 0, g.TOP_DECK_Z]),
                upper.rotate([180, 0, 0]).translate([0, 0, g.UPPER_Z + g.UPPER_T]), sensor.rotate([180, 0, 0]),
                camera.rotate([90, 0, 90]).translate([72.0, 0, g.TOP_DECK_Z + g.DECK_T])]
    placed_spoke = g.spoke_to_assembly(spoke)
    guard_pieces = []
    for k in range(4):
        a = 45 + 90 * k
        guard_pieces.append(placed_spoke.rotate([0, 0, a]))
        guard_pieces.append(arc.translate([0, 0, g.BAND_Z0]).rotate([0, 0, a + 45]))
    guard_pieces = [p.rotate([0, 0, g.TUBE_ANGLE]).translate([g.MOTOR_XY, g.MOTOR_XY, 0]) for p in guard_pieces]
    for c in g.CORNERS:
        assembly.append(g.to_corner(arm, c))
        assembly += [g.to_corner(p, c) for p in guard_pieces]
        assembly.append(g.place_leg(leg, *c))
    return parts, assembly


def write_step(shape, path):
    Interface_Static.SetCVal_s("write.step.schema", "AP214IS")
    Interface_Static.SetCVal_s("write.step.unit", "MM")
    writer = STEPControl_Writer()
    writer.Transfer(shape, STEPControl_AsIs)
    if writer.Write(str(path)) != IFSelect_RetDone:
        raise RuntimeError(f"could not write {path}")


def read_step(path):
    reader = STEPControl_Reader()
    if reader.ReadFile(str(path)) != IFSelect_RetDone:
        raise RuntimeError(f"could not read {path}")
    reader.TransferRoots()
    return reader.OneShape()


def describe(shape):
    solids = planes = cyls = other = 0
    ex = TopExp_Explorer(shape, TopAbs_SOLID)
    while ex.More():
        solids += 1
        ex.Next()
    ex = TopExp_Explorer(shape, TopAbs_FACE)
    while ex.More():
        kind = BRepAdaptor_Surface(TopoDS.Face(ex.Current())).GetType()
        planes += kind == GeomAbs_Plane
        cyls += kind == GeomAbs_Cylinder
        other += kind not in (GeomAbs_Plane, GeomAbs_Cylinder)
        ex.Next()
    return solids, planes, cyls, other


def main():
    import trimesh  # must load before the stand-in module is installed
    install_shim()
    work = Path(sys.argv[1]) if len(sys.argv) > 1 else None
    failed = 0
    for folder, name, builder in (("version 1", "frame_v1", v1_parts), ("version 2", "frame_v2", v2_parts)):
        vdir = ROOT / folder
        print(f"\n{folder}")
        (vdir / "step").mkdir(exist_ok=True)
        g = load(vdir, name)
        parts, assembly = builder(g)
        for pname, part in parts.items():
            deg = g.best_bed_rotation(part)
            placed = part.rotate([0, 0, deg])
            lo, hi = g.bbox(placed)
            placed = placed.translate([-(lo[0] + hi[0]) / 2, -(lo[1] + hi[1]) / 2, -lo[2]])
            path = vdir / "step" / f"{pname}.step"
            write_step(placed.shape, path)
            back = read_step(path)
            solids, planes, cyls, other = describe(back)
            valid = BRepCheck_Analyzer(back).IsValid()
            ref = trimesh.load_mesh(vdir / "stl" / f"{pname}.stl")
            vol = Manifold(back).volume()
            dv = abs(vol - ref.volume) / ref.volume * 100
            size = np.array(Manifold(back).bounding_box()[3:]) - np.array(Manifold(back).bounding_box()[:3])
            ds = float(np.abs(size - ref.extents).max())
            ok = valid and solids == 1 and dv < 1.5 and ds < 0.3
            failed += not ok
            print(f"  [{'PASS' if ok else 'FAIL'}] {pname}: {solids} solid, valid={valid}, faces {planes} plane / {cyls} cylinder / {other} other, "
                  f"volume off STL {dv:.2f}%, size off STL {ds:.2f} mm, turned {deg}")
            if work:
                BRepMesh_IncrementalMesh(back, 0.02, False, 0.2, True)
                StlAPI_Writer().Write(back, str(work / f"{folder.replace(' ', '')}_{pname}.stl"))
        asm_path = vdir / "step" / "assembly_deployed.step"
        write_step(_compound([p.shape for p in assembly]), asm_path)
        s, *_ = describe(read_step(asm_path))
        print(f"  assembly_deployed: {s} solids")
    print(f"\n{'ALL PARTS PASSED' if not failed else str(failed) + ' PART(S) FAILED'}")
    return failed


if __name__ == "__main__":
    raise SystemExit(main())
