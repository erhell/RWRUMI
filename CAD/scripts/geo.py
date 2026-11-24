"""Geometry helpers: load cached BREPs, tessellate, analytic face listing."""
import json, os, numpy as np, pickle
from OCP.TopoDS import TopoDS_Shape, TopoDS
from OCP.BRep import BRep_Builder, BRep_Tool
from OCP.BRepTools import BRepTools
from OCP.BRepMesh import BRepMesh_IncrementalMesh
from OCP.TopExp import TopExp_Explorer
from OCP.TopAbs import TopAbs_FACE, TopAbs_REVERSED
from OCP.TopLoc import TopLoc_Location
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Cylinder, GeomAbs_Plane, GeomAbs_Cone, GeomAbs_Sphere, GeomAbs_Torus
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

C = '/home/claude/cache'
def meta(tag): return json.load(open(f'{C}/{tag}/meta.json'))
def brep(tag, i):
    s = TopoDS_Shape(); BRepTools.Read_s(s, f'{C}/{tag}/{i:03d}.brep', BRep_Builder()); return s
def faces(shape):
    e = TopExp_Explorer(shape, TopAbs_FACE); out = []
    while e.More(): out.append(TopoDS.Face(e.Current())); e.Next()
    return out
def mesh(shape, lin=0.08, ang=0.35):
    BRepMesh_IncrementalMesh(shape, lin, False, ang, True)
    V = []; F = []; off = 0
    for f in faces(shape):
        loc = TopLoc_Location(); tri = BRep_Tool.Triangulation_s(f, loc)
        if tri is None: continue
        T = loc.Transformation(); n = tri.NbNodes()
        pts = np.array([[*(lambda p: (p.X(), p.Y(), p.Z()))(tri.Node(k).Transformed(T))] for k in range(1, n + 1)])
        tr = np.array([[tri.Triangle(k).Value(1), tri.Triangle(k).Value(2), tri.Triangle(k).Value(3)] for k in range(1, tri.NbTriangles() + 1)]) - 1
        if f.Orientation() == TopAbs_REVERSED: tr = tr[:, ::-1]
        V.append(pts); F.append(tr + off); off += n
    return np.vstack(V), np.vstack(F)
def mesh_cached(tag, i, lin=0.08):
    fn = f'{C}/{tag}/{i:03d}_m{lin}.npz'
    if os.path.exists(fn):
        d = np.load(fn); return d['V'], d['F']
    V, F = mesh(brep(tag, i), lin); np.savez(fn, V=V, F=F); return V, F
def props(shape):
    g = GProp_GProps(); BRepGProp.VolumeProperties_s(shape, g); c = g.CentreOfMass()
    return g.Mass(), np.array([c.X(), c.Y(), c.Z()])
def farea(f):
    g = GProp_GProps(); BRepGProp.SurfaceProperties_s(f, g); c = g.CentreOfMass(); return g.Mass(), np.array([c.X(), c.Y(), c.Z()])
def analytic(shape):
    """list of dicts for cylinder/plane/cone faces"""
    out = []
    for k, f in enumerate(faces(shape)):
        a = BRepAdaptor_Surface(f); t = a.GetType(); ar, cen = farea(f)
        d = dict(k=k, area=ar, cen=cen)
        if t == GeomAbs_Cylinder:
            cy = a.Cylinder(); ax = cy.Axis(); p = ax.Location(); v = ax.Direction()
            d.update(type='cyl', r=cy.Radius(), p=np.array([p.X(), p.Y(), p.Z()]), v=np.array([v.X(), v.Y(), v.Z()]),
                     u=(a.FirstUParameter(), a.LastUParameter()), vv=(a.FirstVParameter(), a.LastVParameter()))
        elif t == GeomAbs_Plane:
            pl = a.Plane(); p = pl.Location(); v = pl.Axis().Direction()
            d.update(type='plane', p=np.array([p.X(), p.Y(), p.Z()]), v=np.array([v.X(), v.Y(), v.Z()]))
        elif t == GeomAbs_Cone: d.update(type='cone')
        elif t == GeomAbs_Sphere: d.update(type='sph')
        elif t == GeomAbs_Torus: d.update(type='tor')
        else: d.update(type='other')
        out.append(d)
    return out
