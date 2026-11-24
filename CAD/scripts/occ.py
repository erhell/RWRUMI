"""Small OpenCascade helper layer for the v9 build. Local part frames: x = lateral, y = E (finger direction), z = U (dorsal)."""
import numpy as np, math
from OCP.gp import gp_Pnt, gp_Dir, gp_Vec, gp_Ax1, gp_Ax2, gp_Ax3, gp_Trsf, gp_Circ, gp_Pln, gp_GTrsf, gp_Mat
from OCP.BRepPrimAPI import BRepPrimAPI_MakeCylinder, BRepPrimAPI_MakeBox, BRepPrimAPI_MakePrism, BRepPrimAPI_MakeSphere, BRepPrimAPI_MakeRevol
from OCP.BRepAlgoAPI import BRepAlgoAPI_Fuse, BRepAlgoAPI_Cut, BRepAlgoAPI_Common
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeFace, BRepBuilderAPI_Transform, BRepBuilderAPI_MakePolygon, BRepBuilderAPI_Copy
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakeOffset
from OCP.GeomAbs import GeomAbs_Arc, GeomAbs_Intersection
from OCP.ShapeUpgrade import ShapeUpgrade_UnifySameDomain
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TopoDS import TopoDS, TopoDS_Compound, TopoDS_Shape
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_WIRE, TopAbs_SOLID, TopAbs_EDGE, TopAbs_SHELL
from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp
from OCP.BRepTools import BRepTools
from OCP.GC import GC_MakeArcOfCircle
from OCP.GeomAPI import GeomAPI_Interpolate, GeomAPI_PointsToBSpline
from OCP.collections import HArray1_gp_Pnt, Array1_gp_Pnt
from OCP.BRepFilletAPI import BRepFilletAPI_MakeFillet, BRepFilletAPI_MakeChamfer
from OCP.BRepExtrema import BRepExtrema_DistShapeShape
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.ShapeFix import ShapeFix_Shape

def P(x, y, z): return gp_Pnt(float(x), float(y), float(z))
def D(x, y, z): return gp_Dir(float(x), float(y), float(z))
XDIR = D(1, 0, 0)
def cyl_x(y, z, r, x0, x1):
    """cylinder with axis along local x through (y, z), from x0 to x1"""
    return BRepPrimAPI_MakeCylinder(gp_Ax2(P(x0, y, z), XDIR), float(r), float(x1 - x0)).Shape()
def cyl(p, d, r, h, back=0.0):
    p = np.asarray(p, float); d = np.asarray(d, float); d = d / np.linalg.norm(d); p0 = p - d * back
    return BRepPrimAPI_MakeCylinder(gp_Ax2(P(*p0), D(*d)), float(r), float(h + back)).Shape()
def box(x0, y0, z0, x1, y1, z1):
    return BRepPrimAPI_MakeBox(P(min(x0, x1), min(y0, y1), min(z0, z1)), P(max(x0, x1), max(y0, y1), max(z0, z1))).Shape()
def pol(r, a): return (r * math.cos(math.radians(a)), r * math.sin(math.radians(a)))
def prism_yz(pts, x0, x1):
    """prism of a polygon given in (y, z), extruded along x from x0 to x1"""
    mk = BRepBuilderAPI_MakePolygon()
    for (y, z) in pts: mk.Add(P(x0, y, z))
    mk.Close(); f = BRepBuilderAPI_MakeFace(mk.Wire()).Face()
    return BRepPrimAPI_MakePrism(f, gp_Vec(float(x1 - x0), 0, 0)).Shape()
def wedge(a0, a1, R, x0, x1, n=None):
    """angular sector (polygonal outer boundary beyond R) between angles a0 < a1 (deg) about the x axis"""
    n = n or max(2, int(math.ceil((a1 - a0) / 30.0)) + 1); Rb = R / math.cos(math.radians((a1 - a0) / (n - 1) / 2)) * 1.05
    pts = [(0.0, 0.0)] + [pol(Rb, a) for a in np.linspace(a0, a1, n)]
    return prism_yz(pts, x0, x1)
def fuse(*s):
    r = s[0]
    for t in s[1:]: r = BRepAlgoAPI_Fuse(r, t).Shape()
    return r
def cut(a, *s):
    for t in s: a = BRepAlgoAPI_Cut(a, t).Shape()
    return a
def common(a, b): return BRepAlgoAPI_Common(a, b).Shape()
def unify(s):
    u = ShapeUpgrade_UnifySameDomain(s, True, True, True); u.Build(); return u.Shape()
def annulus_sector(r0, r1, a0, a1, x0, x1):
    ring = cut(cyl_x(0, 0, r1, x0, x1), cyl_x(0, 0, r0, x0 - 1, x1 + 1))
    return common(ring, wedge(a0, a1, r1, x0 - 0.5, x1 + 0.5))
def arc_slot(rc, w, a0, a1, x0, x1):
    """arc band of width w centred on radius rc from a0 to a1 with round ends"""
    s = annulus_sector(rc - w / 2, rc + w / 2, a0, a1, x0, x1)
    for a in (a0, a1): y, z = pol(rc, a); s = fuse(s, cyl_x(y, z, w / 2, x0, x1))
    return s
def stadium(p0, p1, r, x0, x1):
    p0 = np.asarray(p0, float); p1 = np.asarray(p1, float); d = p1 - p0; L = np.linalg.norm(d); d = d / L; n = np.array([-d[1], d[0]]) * r
    return fuse(cyl_x(p0[0], p0[1], r, x0, x1), cyl_x(p1[0], p1[1], r, x0, x1), prism_yz([p0 + n, p1 + n, p1 - n, p0 - n], x0, x1))
def trsf_from_frame(O, X, E, U):
    """local (x, y, z) -> world O + x X + y E + z U"""
    t = gp_Trsf(); t.SetValues(X[0], E[0], U[0], O[0], X[1], E[1], U[1], O[1], X[2], E[2], U[2], O[2]); return t
def move(s, t, copy=True): return BRepBuilderAPI_Transform(s, t, copy).Shape()
def rot_x(s, ang_deg, y=0.0, z=0.0):
    t = gp_Trsf(); t.SetRotation(gp_Ax1(P(0, y, z), XDIR), math.radians(ang_deg)); return move(s, t)
def rot_axis(s, p, d, ang_deg):
    t = gp_Trsf(); t.SetRotation(gp_Ax1(P(*p), D(*d)), math.radians(ang_deg)); return move(s, t)
def translate(s, v):
    t = gp_Trsf(); t.SetTranslation(gp_Vec(float(v[0]), float(v[1]), float(v[2]))); return move(s, t)
def volume(s):
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(s, g); return g.Mass()
def centroid(s):
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(s, g); c = g.CentreOfMass(); return np.array([c.X(), c.Y(), c.Z()])
def valid(s): return BRepCheck_Analyzer(s).IsValid()
def nsub(s, kind=TopAbs_SOLID):
    e = TopExp_Explorer(s, kind); n = 0
    while e.More(): n += 1; e.Next()
    return n
def solids(s):
    e = TopExp_Explorer(s, TopAbs_SOLID); out = []
    while e.More(): out.append(TopoDS.Solid(e.Current())); e.Next()
    return out
def biggest(s):
    ss = solids(s); return max(ss, key=volume) if ss else s
def bbox(s):
    b = Bnd_Box(); BRepBndLib.AddOptimal_s(s, b, False, False); return np.array([b.CornerMin().X(), b.CornerMin().Y(), b.CornerMin().Z()]), np.array([b.CornerMax().X(), b.CornerMax().Y(), b.CornerMax().Z()])
def mindist(a, b):
    d = BRepExtrema_DistShapeShape(a, b); return d.Value() if d.IsDone() else float('nan')
def interference(a, b):
    c = common(a, b); return volume(c) if nsub(c) else 0.0
def compound(shapes):
    c = TopoDS_Compound(); b = BRep_Builder(); b.MakeCompound(c)
    for s in shapes: b.Add(c, s)
    return c
def save(s, fn): BRepTools.Write_s(s, fn)
def load(fn):
    s = TopoDS_Shape(); BRepTools.Read_s(s, fn, BRep_Builder()); return s
def report(name, s):
    print(f'{name:28s} solids {nsub(s)} faces {nsub(s, TopAbs_FACE):4d} vol {volume(s):9.1f} valid {valid(s)}')

def hull2(c1, r1, c2, r2, x0, x1):
    """convex hull of two discs (in y,z) extruded along x: two cylinders + tangent trapezoid"""
    c1 = np.asarray(c1, float); c2 = np.asarray(c2, float); d = c2 - c1; L = np.linalg.norm(d); d = d / L
    n = np.array([-d[1], d[0]]); s = (r1 - r2) / L; c = math.sqrt(max(0.0, 1 - s * s))
    pts = []
    for sg in (1, -1):
        nn = d * s + n * c * sg
        pts.append((c1 + nn * r1, c2 + nn * r2))
    poly = [pts[0][0], pts[0][1], pts[1][1], pts[1][0]]
    return fuse(cyl_x(c1[0], c1[1], r1, x0, x1), cyl_x(c2[0], c2[1], r2, x0, x1), prism_yz(poly, x0, x1))
def hull3(c, r, x0, x1):
    c = [np.asarray(q, float) for q in c]
    return fuse(hull2(c[0], r[0], c[1], r[1], x0, x1), hull2(c[1], r[1], c[2], r[2], x0, x1), hull2(c[2], r[2], c[0], r[0], x0, x1), prism_yz(c, x0, x1))
def bspline_edge(pts3):
    arr = Array1_gp_Pnt(1, len(pts3))
    for i, p in enumerate(pts3): arr.SetValue(i + 1, P(*p))
    return BRepBuilderAPI_MakeEdge(GeomAPI_PointsToBSpline(arr, 3, 8).Curve()).Edge()
def line_edge(a, b): return BRepBuilderAPI_MakeEdge(P(*a), P(*b)).Edge()
def arc_edge_x(a, m, b):
    """circular arc through three points"""
    return BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(P(*a), P(*m), P(*b)).Value()).Edge()
def face_from_edges(edges):
    w = BRepBuilderAPI_MakeWire()
    for e in edges: w.Add(e)
    return BRepBuilderAPI_MakeFace(w.Wire()).Face()
def prism(face, v): return BRepPrimAPI_MakePrism(face, gp_Vec(float(v[0]), float(v[1]), float(v[2]))).Shape()

def gear_tooth(z, m, x0, x1, centre=(0.0, 0.0), ang_deg=0.0, backlash=0.1, alpha=20.0, n=7, root_extra=0.25):
    """one involute tooth (prism along x) of an external gear centred at `centre` (y,z); tooth axis at polar angle ang_deg"""
    r = z * m / 2; al = math.radians(alpha); rb = r * math.cos(al); ra = r + m; rf = r - 1.25 * m
    s = math.pi * m / 2 - backlash; inv = lambda a: math.tan(a) - a
    def psi(rho): return s / (2 * r) + inv(al) - inv(math.acos(min(1.0, rb / rho)))
    rhos = np.linspace(max(rb, rf), ra, n); a0 = math.radians(ang_deg); cy, cz = centre
    def pt(rho, ang): return (x0, cy + rho * math.cos(a0 + ang), cz + rho * math.sin(a0 + ang))
    left = [pt(rho, psi(rho)) for rho in rhos]; right = [pt(rho, -psi(rho)) for rho in rhos][::-1]
    rin = rf - root_extra; pb = psi(max(rb, rf))
    edges = [line_edge(pt(rin, pb), left[0])] if True else []
    edges.append(bspline_edge(left))
    edges.append(arc_edge_x(left[-1], pt(ra, 0.0), right[0]))
    edges.append(bspline_edge(right))
    edges.append(line_edge(right[-1], pt(rin, -pb)))
    edges.append(line_edge(pt(rin, -pb), pt(rin, pb)))
    return prism(face_from_edges(edges), (x1 - x0, 0, 0))

from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Plane
from OCP.BRepTools import BRepTools_WireExplorer
from OCP.ShapeAnalysis import ShapeAnalysis
def faces_of(s):
    e = TopExp_Explorer(s, TopAbs_FACE); out = []
    while e.More(): out.append(TopoDS.Face(e.Current())); e.Next()
    return out
def face_area(f):
    g = GProp_GProps(); BRepGProp.SurfaceProperties_s(f, g); c = g.CentreOfMass(); return g.Mass(), np.array([c.X(), c.Y(), c.Z()])
def planar_face_at_x(s, x, tol=1e-4):
    best = None
    for f in faces_of(s):
        a = BRepAdaptor_Surface(f)
        if a.GetType() != GeomAbs_Plane: continue
        n = a.Plane().Axis().Direction(); ar, c = face_area(f)
        if abs(abs(n.X()) - 1) < 1e-6 and abs(c[0] - x) < tol and (best is None or ar > best[0]): best = (ar, f)
    return best[1]
def offset_wire(w, d):
    mk = BRepOffsetAPI_MakeOffset(w, GeomAbs_Arc); mk.Perform(float(d)); r = mk.Shape()
    e = TopExp_Explorer(r, TopAbs_WIRE); ws = []
    while e.More(): ws.append(TopoDS.Wire(e.Current())); e.Next()
    return ws
def smooth_prism(solid, x0, x1, r_close=2.5, r_open=0.0):
    """re-extrude the outline of a prism (along x) after rounding its concave corners (closing) and optionally convex ones (opening)"""
    f = planar_face_at_x(solid, x0); w = ShapeAnalysis.OuterWire_s(f)
    seq = [(r_close, -r_close)] + ([(-r_open, r_open)] if r_open > 0 else [])
    for (a, b) in seq:
        for dd in (a, b):
            ws = offset_wire(w, dd)
            if len(ws) != 1: raise RuntimeError(f'offset gave {len(ws)} wires')
            w = ws[0]
    face = BRepBuilderAPI_MakeFace(w).Face()
    p = prism(face, (x1 - x0, 0, 0))
    if volume(p) < 0: p = p.Reversed()
    return p
