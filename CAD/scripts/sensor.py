"""3382H sensor: canonical solid (axis = +z, body underside at z = 0, body centred on the outline) taken from the v7 model, and placement helper."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, math, json, occ, geo, finger9 as F
AA_SENS = {'index': 12, 'middle': 22, 'ring': 50, 'pinky': 30}
def analyse(i, p, v):
    """returns (s_bottom, e1, e2, half1, half2, centre_offset) of the body rectangle in the plane normal to v"""
    sh = geo.brep('v7', i); best = None
    for f in occ.faces_of(sh):
        ar, c = occ.face_area(f)
        a = occ.BRepAdaptor_Surface(f)
        if a.GetType() != occ.GeomAbs_Plane: continue
        nrm = a.Plane().Axis().Direction(); nv = np.array([nrm.X(), nrm.Y(), nrm.Z()])
        if abs(abs(nv @ v) - 1) < 1e-4 and (best is None or ar > best[0]): best = (ar, f, c)
    ar, fb, c = best; s_bot = (c - p) @ v
    V, _ = geo.mesh(fb, 0.05); a1 = np.cross(v, [1.0, 0, 0]); a1 /= np.linalg.norm(a1); a2 = np.cross(v, a1)
    Q = np.c_[(V - p) @ a1, (V - p) @ a2]; bestr = None
    for th in np.radians(np.arange(0, 180, 0.25)):
        R = np.array([[math.cos(th), math.sin(th)], [-math.sin(th), math.cos(th)]]); W = Q @ R.T; lo, hi = W.min(0), W.max(0); area = np.prod(hi - lo)
        if bestr is None or area < bestr[0]: bestr = (area, th, lo, hi)
    _, th, lo, hi = bestr; e1 = a1 * math.cos(th) + a2 * math.sin(th); e2 = -a1 * math.sin(th) + a2 * math.cos(th)
    return s_bot, e1, e2, (hi - lo) / 2, (lo + hi) / 2, sh
if __name__ == '__main__':
    out = {}
    for f, i in AA_SENS.items():
        d = F.FR[f]; p = np.array(d['aa_p']); v = np.array(d['aa_v']); X, E, U = (np.array(d[k]) for k in 'XEU')
        s_bot, e1, e2, half, cen, sh = analyse(i, p, v)
        if half[0] < half[1]: e1, e2, half, cen = e2, -e1, half[::-1], np.array([cen[1], -cen[0]])
        V, _ = geo.mesh(sh, 0.1); s = (V - p) @ v
        print(f"{f:7s} body underside s={s_bot:.2f}, body {2*half[0]:.2f} x {2*half[1]:.2f}, centre offset from axis ({cen[0]:.2f},{cen[1]:.2f}); long side direction in finger frame (X,E,U) = ({e1@X:.3f},{e1@E:.3f},{e1@U:.3f}); s range {s.min():.2f}..{s.max():.2f}")
        out[f] = dict(s_bot=float(s_bot), e1=e1.tolist(), e2=e2.tolist(), half=half.tolist(), cen=cen.tolist(), id=i)
    json.dump(out, open('/home/claude/cache/v7/aa_sensors.json', 'w'), indent=1)
