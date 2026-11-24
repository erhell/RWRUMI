"""Thumb coupling link R-T-AP -> T-DP (IP ~ 0.8 x MCP), on the side away from the sensors."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, math, json, occ, geo, trimesh
from scipy.optimize import brentq
V9 = '/home/claude/cache/v9'
T = json.load(open('/home/claude/cache/v7/thumb_axes.json'))
a = np.array(T['T4'][1]); a /= np.linalg.norm(a); M = np.array(T['T3'][0]); G = np.array(T['T4'][0])
G = G - a * ((G - M) @ a)                                  # IP axis point in the plane of M
d = G - M; L0 = float(np.linalg.norm(d)); d /= L0; w = np.cross(-a, d)   # w = dorsal (side of the user's thumb)
A2 = np.array([3.0, 5.0]); B2 = np.array([-2.0, -7.0])     # link pivots: on AP relative to M, on T-DP relative to G (d, w)
def P3(o, uv, lat): return o + d * uv[0] + w * uv[1] + a * lat
def lat_extent(i, centre, rad=4.5):
    V, _ = geo.mesh_cached('v7', i); r = V - centre; lat = r @ a; rr = np.linalg.norm(r - np.outer(lat, a), axis=1); m = rr < rad
    return (lat[m].min(), lat[m].max()) if m.any() else None
if __name__ == '__main__':
    Aw = P3(M, A2, 0); Bw = P3(G, B2, 0)
    print('MCP-IP', round(L0, 2), 'axis a', a.round(3))
    for nm, i in (('AP', 56), ('PP', 57), ('DP', 58), ('enc T3', 61), ('enc T4', 62), ('bearing T3', 68), ('bearing T4', 69)):
        print(f'  {nm:10s} lateral extent (along a, rel. to M) near A: {lat_extent(i, Aw)}  near B: {lat_extent(i, Bw)}  overall: {lat_extent(i, M, 1e9)}')
    # along the link path: which bodies does it pass, and how far do they reach on the +a side
    for t in np.linspace(0, 1, 6):
        c = Aw + (Bw - Aw) * t; print('  t=%.1f' % t, {nm: (None if lat_extent(i, c, 3.0) is None else round(lat_extent(i, c, 3.0)[1], 2)) for nm, i in (('AP', 56), ('PP', 57), ('DP', 58))})

# ---------------------------------------------------------------- v9 link on the far side (27 mm from the sensors), push type
A2 = np.array([1.0, -4.0]); B2 = np.array([0.0, 5.5])
LAT_LINK = (27.6, 32.6); LAT_ARM = (33.0, 38.0); LAT_TAB = (21.8, 26.8)
TF = occ.trsf_from_frame(M, a, d, w)                      # local x = lateral (along the joint axes), y = along T-PP, z = dorsal
LINK_L = float(np.linalg.norm(np.array([L0, 0.0]) + B2 - A2))
def Rm(t): c, s = math.cos(t), math.sin(t); return np.array([[c, -s], [s, c]])
def ip_of(th):
    """IP flexion (rad) for MCP flexion th (rad)"""
    if abs(th) < 1e-9: return 0.0
    Gq = Rm(-th) @ np.array([L0, 0.0]); f = lambda x: np.linalg.norm(Gq + Rm(-(th + x)) @ B2 - A2) - LINK_L
    xs = np.radians(np.linspace(-30, 110, 281)); fs = [f(x) for x in xs]
    roots = [brentq(f, xs[i], xs[i + 1]) for i in range(len(xs) - 1) if fs[i] * fs[i + 1] <= 0]
    return min(roots, key=lambda t: abs(t - 0.8 * th))
def build():
    ap = geo.brep('v7', 56); dp = geo.brep('v7', 58)
    Bp = np.array([L0, 0.0]) + B2
    arm = occ.hull2((-24.0, -31.0), 6.0, tuple(A2), 3.4, *LAT_ARM)
    arm = occ.cut(arm, occ.cyl_x(A2[0], A2[1], 1.0, LAT_ARM[0] - 1, LAT_ARM[1] + 1))
    tab = occ.hull2(tuple(Bp), 3.0, (L0 + 11.5, 4.4), 2.6, *LAT_TAB)
    tab = occ.cut(tab, occ.cyl_x(Bp[0], Bp[1], 1.0, LAT_TAB[0] + 0.0, LAT_TAB[1] + 1))
    link = occ.cut(occ.hull2(tuple(A2), 2.9, tuple(Bp), 2.9, *LAT_LINK), occ.cyl_x(A2[0], A2[1], 1.1, LAT_LINK[0] - 1, LAT_LINK[1] + 1), occ.cyl_x(Bp[0], Bp[1], 1.1, LAT_LINK[0] - 1, LAT_LINK[1] + 1))
    pinA = occ.cyl_x(A2[0], A2[1], 1.0, LAT_LINK[0] + 0.4, LAT_LINK[0] + 10.4); pinB = occ.cyl_x(Bp[0], Bp[1], 1.0, LAT_LINK[1] - 10.0, LAT_LINK[1])
    armw = occ.move(arm, TF); tabw = occ.move(tab, TF)
    print('outrigger overlaps AP by %.1f mm3; tab overlaps T-DP by %.1f mm3' % (occ.interference(armw, ap), occ.interference(tabw, dp)))
    ap2 = occ.fuse(ap, armw); dp2 = occ.fuse(dp, tabw)
    return {'Thumb_AP': ap2, 'Thumb_DP': dp2, 'Thumb_Link': occ.unify(occ.move(link, TF)), 'Thumb_Pin2x10_LinkA': occ.move(pinA, TF), 'Thumb_Pin2x10_LinkB': occ.move(pinB, TF)}
def pose(shape, part, th):
    """move a thumb body for MCP flexion th (rad); IP follows the link. part in 'PP','DP','link'"""
    ti = ip_of(th); c, s = math.cos(-th), math.sin(-th)
    if part == 'PP': return occ.rot_axis(shape, M, a, math.degrees(th))
    if part == 'DP':
        s1 = occ.rot_axis(shape, M, a, math.degrees(th)); Gw = M + (d * math.cos(th) - w * math.sin(th)) * L0
        return occ.rot_axis(s1, Gw, a, math.degrees(ti))
    if part == 'link':
        Gq = Rm(-th) @ np.array([L0, 0.0]); Bq = Gq + Rm(-(th + ti)) @ B2; B0 = np.array([L0, 0.0]) + B2
        ang = math.atan2(*(Bq - A2)[::-1]) - math.atan2(*(B0 - A2)[::-1]); Aw = M + d * A2[0] + w * A2[1]
        return occ.rot_axis(shape, Aw, a, -math.degrees(ang))
if __name__ == '__main__' and len(sys.argv) > 1 and sys.argv[1] == 'build':
    parts = build()
    for k, v in parts.items(): occ.report(k, v); occ.save(v, f'{V9}/{k}.brep')
    print('link length', round(LINK_L, 2), '| coupling: MCP -> IP', [(t, round(math.degrees(ip_of(math.radians(t))), 1)) for t in (10, 20, 30, 40, 50, 60, 70)])
    # which way is flexion? rotating T-PP about `a` by +th must move G towards -w
    pp = geo.brep('v7', 57); enc4 = geo.brep('v7', 62); b4 = geo.brep('v7', 69); enc3 = geo.brep('v7', 61)
    print('sweep:')
    for t in (0, 15, 30, 45, 60, 70):
        th = math.radians(t); PPq = pose(pp, 'PP', th); DPq = pose(parts['Thumb_DP'], 'DP', th); Lq = pose(parts['Thumb_Link'], 'link', th); pb = pose(parts['Thumb_Pin2x10_LinkB'], 'DP', th)
        print('  MCP %2d IP %4.1f: link x PP %.2f, link x DP %.2f, link x AP %.2f, DP x PP %.2f, DP x AP %.2f, PP x AP %.2f | link-PP dist %.2f | pin B inside link eye: dist to link %.2f' % (
            t, math.degrees(ip_of(th)), occ.interference(Lq, PPq), occ.interference(Lq, DPq), occ.interference(Lq, parts['Thumb_AP']), occ.interference(DPq, PPq), occ.interference(DPq, parts['Thumb_AP']), occ.interference(PPq, parts['Thumb_AP']), occ.mindist(Lq, PPq), occ.mindist(pb, Lq)))
