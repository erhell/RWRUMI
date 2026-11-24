"""Glove v9 palm frame: v7 lower palm kept; new side wall extension, strut, flat bridge plate with the four AA seats, PCB deck."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, math, json, time
import occ, geo, finger9 as F
from OCP.gp import gp_Pnt, gp_Dir, gp_Vec, gp_Ax2, gp_Circ, gp_Pln
from OCP.BRepBuilderAPI import BRepBuilderAPI_MakeEdge, BRepBuilderAPI_MakeWire, BRepBuilderAPI_MakeFace
from OCP.BRepOffsetAPI import BRepOffsetAPI_MakePipe
from OCP.GC import GC_MakeArcOfCircle
from OCP.BRepAdaptor import BRepAdaptor_Surface
from OCP.GeomAbs import GeomAbs_Plane
Z_WALL, Z_STRUT = 40.0, 44.0
FING = ('index', 'middle', 'ring', 'pinky')
def seat_world(f, U=F.BRIDGE_U):
    d = F.FR[f]; O = np.array(d['O']); X, E, Uv = (np.array(d[k]) for k in 'XEU'); p = F.aa_pt(f, U); return O + X * p[0] + E * p[1] + Uv * p[2]
def axis_world(f): return np.array(F.FR[f]['aa_v'])
def plate_frame():
    n = np.mean([axis_world(f) for f in FING], axis=0); n /= np.linalg.norm(n)
    ex = np.array([1.0, 0, 0]) - n * n[0]; ex /= np.linalg.norm(ex); ey = np.cross(n, ex)
    # underside height: clear of every housing block (top 1 mm below its seat, radius 10.5, tilted axis) by 0.6 mm
    h = max(seat_world(f) @ n - 1.0 * (axis_world(f) @ n) + 10.5 * math.sin(math.acos(min(1.0, axis_world(f) @ n))) + 0.6 for f in FING)
    c = np.mean([seat_world(f) for f in FING], axis=0); O = c + n * (h - c @ n)
    return O, ex, ey, n, h
def face_at_z(shape, z, side):
    """largest planar face lying in the plane z = const on the ulnar (side>0) or radial side"""
    best = None
    for f in occ.faces_of(shape):
        a = BRepAdaptor_Surface(f)
        if a.GetType() != GeomAbs_Plane: continue
        ar, c = occ.face_area(f)
        if abs(c[2] - z) < 1e-4 and abs(abs(a.Plane().Axis().Direction().Z()) - 1) < 1e-6 and c[0] * side > 0 and (best is None or ar > best[0]): best = (ar, f, c)
    return best
def lower_palm():
    p = geo.brep('v7', 0)
    p = occ.cut(p, occ.box(20, 60, Z_WALL, 120, 220, 220), occ.box(-120, 60, Z_STRUT, 20, 220, 220))
    return p
def bend_up(face, c0, t0, R, z_top):
    """sweep `face` along an arc that starts at c0 with tangent t0 and ends vertical (radius R), then extrude vertically past z_top"""
    t0 = np.asarray(t0, float); t0 /= np.linalg.norm(t0); c0 = np.asarray(c0, float)
    h = np.array([t0[0], t0[1], 0.0]); nh = np.linalg.norm(h); h /= nh; a = math.atan2(nh, t0[2]); z = np.array([0, 0, 1.0])
    pt = lambda th: c0 + R * ((math.cos(th) - math.cos(a)) * h + (math.sin(a) - math.sin(th)) * z)
    p0, pm, p1 = pt(a), pt(a / 2), pt(0.0)
    arc = BRepBuilderAPI_MakeEdge(GC_MakeArcOfCircle(occ.P(*p0), occ.P(*pm), occ.P(*p1)).Value()).Edge()
    mk = BRepOffsetAPI_MakePipe(BRepBuilderAPI_MakeWire(arc).Wire(), face); mk.Build(); pipe = mk.Shape(); end = mk.LastShape()
    up = occ.prism(end, (0, 0, z_top - p1[2] + 14.0))
    return occ.fuse(pipe, up), p1, math.degrees(a)
def section_shift(z0, z1, side):
    """translation of the wall/strut cross-section between two heights (centroid of the v7 section)"""
    import trimesh
    V, Fc = geo.mesh_cached('v7', 0); m = trimesh.Trimesh(V, Fc, process=False); out = []
    for z in (z0, z1):
        s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1]); pl, T = s.to_2D()
        best = None
        for poly in pl.polygons_full:
            c = np.array(poly.centroid.coords[0]); cw = (T @ np.array([c[0], c[1], 0, 1]))[:3]
            if cw[0] * side > 0 and (best is None or poly.area > best[0]): best = (poly.area, cw)
        out.append(best[1])
    d = out[1] - out[0]; return d / d[2], out
def to_plate(O, ex, ey, n, h, x, y):
    z = (h - n[0] * x - n[1] * y) / n[2]; P = np.array([x, y, z]) - O; return np.array([P @ ex, P @ ey])
def plate_outline(O, ex, ey, n, h, c0, c1):
    """plate body between local heights c0 and c1; helper coords (x=c, y=a, z=b)"""
    L = lambda x, y: to_plate(O, ex, ey, n, h, x, y)
    seats = [L(*seat_world(f)[:2]) for f in FING]
    poly = [L(-43.5, 97.5), L(43.5, 97.5), L(52.5, 116), L(71.5, 138), L(71.5, 161.5)] + seats[::-1] + [L(-61.5, 163.5), L(-61.5, 145.5), L(-51, 136), L(-43.5, 127)]
    s = occ.prism_yz(poly, c0, c1)
    for i, q in enumerate(seats):
        s = occ.fuse(s, occ.cyl_x(q[0], q[1], 11.5, c0, c1))
        if i: s = occ.fuse(s, occ.hull2(seats[i - 1], 11.5, q, 11.5, c0, c1))
    s = occ.unify(s)
    return occ.smooth_prism(s, c0, c1, 12.0, 6.0)
PCB = dict(cx=0.0, cy=125.0, w=82.0, d=50.0, t=1.5)
def build():
    O, ex, ey, n, h = plate_frame(); TP = occ.trsf_from_frame(O, n, ex, ey)      # helper x -> n, y -> ex, z -> ey
    low = lower_palm()
    ar, fw, cw = face_at_z(low, Z_WALL, 1); dw, _ = section_shift(30.0, 40.0, 1)
    ar2, fs, cs = face_at_z(low, Z_STRUT, -1); ds, _ = section_shift(42.0, 44.0, -1)
    ztop = (h + 7.0) / n[2] + 6
    wall, pw, aw = bend_up(fw, cw, dw, 40.0, ztop); strut, pst, ast = bend_up(fs, cs, ds, 35.0, ztop)
    print('wall bend %.1f deg, ends vertical at %s; strut bend %.1f deg, ends at %s' % (aw, pw.round(1), ast, pst.round(1)))
    plate = occ.move(plate_outline(O, ex, ey, n, h, 0.0, 7.0), TP)
    body = occ.fuse(low, wall, strut, plate)
    # trim everything above the plate top
    slab = occ.move(occ.box(7.0, -200, -200, 80.0, 200, 200), TP); body = occ.cut(body, slab)
    # PCB pocket, window and clamp screw pilots
    pc = to_plate(O, ex, ey, n, h, PCB['cx'], PCB['cy']); w, d = PCB['w'] + 0.6, PCB['d'] + 0.6
    pocket = occ.move(occ.box(7.0 - PCB['t'], pc[0] - w / 2, pc[1] - d / 2, 9.0, pc[0] + w / 2, pc[1] + d / 2), TP)
    win = occ.move(occ.box(-2.0, pc[0] - w / 2 + 4.5, pc[1] - d / 2 + 4.5, 9.0, pc[0] + w / 2 - 4.5, pc[1] + d / 2 - 4.5), TP)
    body = occ.cut(body, pocket, win)
    for sx in (-22.0, 22.0):
        for sy in (-1, 1):
            q = (pc[0] + sx, pc[1] + sy * (d / 2 + 2.3)); body = occ.cut(body, occ.move(occ.cyl_x(q[0], q[1], 1.25, 0.8, 9.0), TP))
    # AA seats
    for f in FING:
        d = F.FR[f]; T = F.world(f); ax = F.aa_dir(f); ax = ax / np.linalg.norm(ax); Pl = lambda U: F.aa_pt(f, U)
        ln = lambda U0, U1: float(np.linalg.norm(Pl(U1) - Pl(U0)))
        boss = occ.cyl(Pl(F.BRIDGE_U), ax, 8.0, ln(F.BRIDGE_U, 81.0))
        body = occ.fuse(body, occ.move(boss, T))
    # sensor trays on the boss tops: floor 81.0-83.2, walls to 85.0 on the front and both sides, open to the rear (terminals)
    for f in FING:
        T = F.world(f); ax = F.aa_dir(f); ax = ax / np.linalg.norm(ax); A0 = F.aa_pt(f, 81.0); ey_l = np.cross(ax, [1.0, 0, 0])   # ~ +E
        TL = occ.trsf_from_frame(A0, [1.0, 0, 0], ey_l, ax)                                                             # tray frame: x lateral, y forward, z along the axis
        tray = occ.fuse(occ.box(-7.27, -7.6, 0.0, 7.27, 7.3, 2.2), occ.box(-7.27, 5.69, 2.2, 7.27, 7.3, 4.0), occ.box(-7.27, -6.6, 2.2, -5.66, 7.3, 4.0), occ.box(5.66, -6.6, 2.2, 7.27, 7.3, 4.0))
        body = occ.fuse(body, occ.move(occ.move(tray, TL), T))
    for f in FING:
        T = F.world(f); ax = F.aa_dir(f); ax = ax / np.linalg.norm(ax); Pl = lambda U: F.aa_pt(f, U)
        ln = lambda U0, U1: float(np.linalg.norm(Pl(U1) - Pl(U0)))
        cuts = [occ.cyl(Pl(F.U_B1[0] - 6.0), ax, 4.05, ln(F.U_B1[0] - 6.0, F.U_B1[1] + 0.1)),      # lower bearing seat, open downwards
                occ.cyl(Pl(F.U_B1[1]), ax, 2.6, ln(F.U_B1[1], F.U_B2[0])),                         # bore for pin + sleeve
                occ.cyl(Pl(F.U_B2[0] - 0.1), ax, 4.05, ln(F.U_B2[0] - 0.1, 90.0))]                 # upper bearing seat, open upwards
        # free sweep of the housing block under the plate (radius 11.2 about the axis, up to 0.4 mm above the block)
        cuts.append(occ.cut(occ.cyl(Pl(45.0), ax, 11.2, ln(45.0, F.ZB + 0.4)), occ.cyl(Pl(44.0), ax, 2.61, 30.0)))
        for c in cuts: body = occ.cut(body, occ.move(c, T))
    return body, (O, ex, ey, n, h)
if __name__ == '__main__':
    t = time.time()
    body, fr = build(); occ.report('Palm v9 (no sensor seats yet)', body); print('solids', occ.nsub(body))
    body = occ.biggest(body); occ.save(body, '/home/claude/cache/v9/Palm_stage1.brep')
    import camera9
    final = body
    for hcyl in camera9.wall_holes(): final = occ.cut(final, hcyl)
    occ.report('Palm (with camera screw holes)', final); occ.save(final, '/home/claude/cache/v9/Palm.brep'); json.dump(dict(O=fr[0].tolist(), ex=fr[1].tolist(), ey=fr[2].tolist(), n=fr[3].tolist(), h=float(fr[4])), open('/home/claude/cache/v9/plate_frame.json', 'w')); print('time', round(time.time() - t, 1))
