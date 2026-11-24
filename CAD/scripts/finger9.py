"""Glove v9 knuckle joint, parametric build in the finger frame (x lateral/ulnar+, y = finger direction at file pose, z = dorsal)."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, math, json
import occ
from occ import pol
FR = json.load(open('/home/claude/cache/v7/frames.json'))
RS, SW, WALL = 47.0, 8.2, 5.0
RI, RO = RS - SW / 2 - WALL, RS + SW / 2 + WALL            # 37.9 / 56.1
PH = 29.0; QMIN, QMAX = -5.0, 68.0; ROM = QMAX - QMIN
DELTA = math.degrees(9.6 / RS)
L_ANG = PH + QMAX + DELTA                                   # 108.70  bearing pair on the arc
C0, C1 = PH + DELTA, PH + DELTA + ROM                       # 40.70 .. 113.70  cheek slots
GAP, TC, POCKET = 0.35, 4.15, 3.6
MOD, ZS, ZG = 0.9, 128, 32; RACK_R, GEAR_R = MOD * ZS / 2, MOD * ZG / 2   # 57.6 / 14.4
PHA = 34.0; GC = np.array(pol(RACK_R + GEAR_R, PHA))        # gear axle
TIP_S = RACK_R + MOD                                        # 58.5 rack tip radius
P1 = np.array(pol(62.7, 53.0))                              # spacer post between the cheeks
ZB = 61.5                                                   # top of the housing block
BRIDGE_U = 62.5                                             # underside of the bridge at the AA axis
U_B1, U_B2 = (62.5, 65.5), (78.0, 81.0)                     # AA bearings (along the axis)
BEARING = (4.0, 8.0, 3.0)
def aa(f):
    d = FR[f]; return d['aa_X'], d['aa_E0'], d['aa_dE'], d['aa_dU']
def aa_pt(f, U):
    x, e0, dE, dU = aa(f); return np.array([x, e0 + dE / dU * U, U])
def aa_dir(f):
    x, e0, dE, dU = aa(f); return np.array([0.0, dE, dU])
def layers(f):
    T = FR[f]['T']; return dict(T=T, xL=(-T / 2, T / 2), xR=(-T / 2 - GAP - TC, -T / 2 - GAP), xU=(T / 2 + GAP, T / 2 + GAP + TC))
def rack_angles():
    out = []
    for k in range(-40, 60):
        a = PHA + (k + 0.5) * 360.0 / ZS
        if 26.4 <= a <= 104.4: out.append(a)
    return out
def bearing(p, d):
    return occ.cut(occ.cyl(p, d, BEARING[1] / 2, BEARING[2]), occ.cyl(p, d, BEARING[0] / 2, BEARING[2] + 2, back=1))
def tube(p, d, ro, ri, h): return occ.cut(occ.cyl(p, d, ro, h), occ.cyl(p, d, ri, h + 2, back=1))
# ------------------------------------------------------------------ arc on the proximal piece
def link_arc(f):
    L = layers(f); x0, x1 = L['xL']
    body = occ.arc_slot(RS, SW + 2 * WALL, PH + QMIN, 104.0, x0, x1)
    ang = rack_angles(); root = occ.annulus_sector(RO - 0.3, RACK_R - 1.25 * MOD, ang[0] - 1.6, ang[-1] + 1.6, x0, x1)
    body = occ.fuse(body, root)
    for a in ang: body = occ.fuse(body, occ.gear_tooth(ZS, MOD, x0, x1, ang_deg=a))
    slot = occ.arc_slot(RS, SW, PH + QMIN, PH + QMAX, x0 - 1, x1 + 1)
    y, z = pol(RS, L_ANG); pin = occ.cyl_x(y, z, 1.0, x0 - 1, x1 + 1)
    return occ.unify(occ.cut(body, slot, pin))
def old_bow_cutter(f):
    """removes the v7 bow (r > 39.5) across the web/bow thickness; the PIP ear (within 8.7 mm of the PIP axis) is kept"""
    L = layers(f); x0, x1 = L['xL']
    c = occ.annulus_sector(39.5, 75.0, -25.0, 150.0, x0 - 1.5, x1 + 1.5)
    return occ.cut(c, occ.cyl_x(FR[f]['pip_r'], 0.0, 8.7, x0 - 3, x1 + 3))
# ------------------------------------------------------------------ housing
def block_profile(f, x0, x1, rin, y_lo=None, y_hi=None):
    ea = aa_pt(f, 57.0)[1]; y0 = ea - 10.3 if y_lo is None else y_lo; y1 = ea + 8.5 if y_hi is None else y_hi
    top = occ.hull2((y0 + 3, ZB - 3), 3.0, (y1 - 3, ZB - 3), 3.0, x0, x1)
    s = occ.fuse(top, occ.box(x0, y0, 30.0, x1, y1, ZB - 3))
    return occ.cut(s, occ.cyl_x(0, 0, rin, x0 - 1, x1 + 1))
def planform(f, x0, x1, gear_r):
    s = occ.arc_slot(RS, SW + 2 * WALL, C0, 110.0, x0, x1)
    s = occ.fuse(s, occ.hull2(pol(RS, 110.0), SW / 2 + WALL, pol(RS, C1), SW / 2 + 4.0, x0, x1))
    s = occ.fuse(s, occ.hull2(pol(RS, C0), SW / 2 + WALL, pol(RS, PH), 5.2, x0, x1))
    s = occ.fuse(s, occ.hull3([GC, pol(RS, PH), P1], [gear_r, 5.2, 3.6], x0, x1))
    s = occ.fuse(s, occ.hull2(P1, 3.6, pol(RO - 4.2, 66.0), 4.2, x0, x1))
    s = occ.fuse(s, block_profile(f, x0, x1, RO - 3.0))
    return s
def screws(f):
    ea = aa_pt(f, 57.0)[1]
    return [np.array([ea - 7.7, 58.6]), np.array([ea - 7.7, 54.3]), P1.copy()]
def cheek_radial(f):
    L = layers(f); x0, x1 = L['xR']; xa = aa(f)[0]
    s = occ.smooth_prism(occ.unify(planform(f, x0, x1, 6.0)), x0, x1, 3.0)
    # lug on the outer face around the AA pin
    ea = aa_pt(f, 57.0)[1]; xl = xa - 4.5
    lug = occ.fuse(occ.hull2((ea - 2.5, ZB - 2), 2.0, (ea + 2.5, ZB - 2), 2.0, xl, x0), occ.box(xl, ea - 4.5, 30, x0, ea + 4.5, ZB - 2))
    lug = occ.cut(lug, occ.cyl_x(0, 0, RS + SW / 2, xl - 1, x0 + 1))
    s = occ.fuse(s, lug)
    # AA pin bore (2.95) from the top down to just above the slot
    d = aa_dir(f); d = d / np.linalg.norm(d); ptop = aa_pt(f, ZB + 1.0); pbot = aa_pt(f, 49.0)
    s = occ.cut(s, occ.cyl(pbot, d, 1.475, np.linalg.norm(ptop - pbot)))
    # set screw M2x3 against the pin
    pm = aa_pt(f, 57.5); s = occ.cut(s, occ.cyl([xl - 0.5, pm[1], pm[2]], [1, 0, 0], 0.85, 4.0))
    # bearing pocket, H pin hole, gear shaft seat
    s = occ.cut(s, occ.arc_slot(RS, SW, C0, C1, x1 - POCKET, x1 + 0.5))
    y, z = pol(RS, PH); s = occ.cut(s, occ.cyl_x(y, z, 1.0, x1 - 2.45, x1 + 0.5))
    s = occ.cut(s, occ.cyl_x(GC[0], GC[1], 2.12, x1 - 3.2, x1 + 0.5))
    for (y, z) in screws(f):
        s = occ.cut(s, occ.cyl_x(y, z, 1.1, x0 - 1, x1 + 1), occ.cyl_x(y, z, 2.1, x0 - 1, x0 + 1.3))
    return occ.unify(s)
def roof(f):
    L = layers(f); x0, x1 = L['xR'][1], L['xU'][0]; ea = aa_pt(f, 57.0)[1]
    ysplit = (TIP_S + 0.7) * math.cos(math.radians(104.4 + 1.6 - QMIN + 1.0))   # behind the swept rack
    r1 = block_profile(f, x0, x1, TIP_S + 0.7, y_hi=ea + 4.0)
    r2 = block_profile(f, x0, x1, RO + 0.6, y_hi=ysplit)
    return occ.fuse(r1, r2)
def cheek_ulnar(f):
    L = layers(f); x0, x1 = L['xU']; xr1 = L['xR'][1]
    s = occ.smooth_prism(occ.unify(planform(f, x0, x1, 10.0)), x0, x1, 3.0)
    s = occ.fuse(s, roof(f), occ.cyl_x(P1[0], P1[1], 3.5, xr1, x0))
    s = occ.cut(s, occ.arc_slot(RS, SW, C0, C1, x0 - 0.5, x0 + POCKET))
    y, z = pol(RS, PH); s = occ.cut(s, occ.cyl_x(y, z, 1.0, x0 - 0.5, x0 + 2.45))
    s = occ.cut(s, occ.cyl_x(GC[0], GC[1], 2.12, x0 - 0.5, x1 + 0.5))
    # sensor recess on the outer face: body 12.04 x 11.02, long side along the tangent, terminals towards +t (up and back)
    t = np.array([-math.sin(math.radians(PHA)), math.cos(math.radians(PHA))]); n = np.array([math.cos(math.radians(PHA)), math.sin(math.radians(PHA))])
    c = [GC + t * a + n * b for (a, b) in ((-5.71, -5.67), (7.7, -5.67), (7.7, 5.67), (-5.71, 5.67))]
    s = occ.cut(s, occ.prism_yz(c, x1 - 1.6, x1 + 0.5))
    for (y, z) in screws(f): s = occ.cut(s, occ.cyl_x(y, z, 0.8, xr1 - 0.5, xr1 + 4.3))
    return occ.unify(s)
def gear(f):
    L = layers(f); T = L['T']; x0, x1 = -T / 2 + 0.3, T / 2 - 0.3
    g = occ.cyl_x(GC[0], GC[1], GEAR_R - 1.25 * MOD + 0.02, x0, x1)
    for j in range(ZG): g = occ.fuse(g, occ.gear_tooth(ZG, MOD, x0, x1, centre=tuple(GC), ang_deg=PHA + 180.0 + j * 360.0 / ZG))
    xr1 = L['xR'][1]; xu1 = L['xU'][1]
    shaft = occ.cyl_x(GC[0], GC[1], 2.0, xr1 - 3.0, xu1 - 1.6)                  # journal in both cheeks
    dsh = occ.cyl_x(GC[0], GC[1], 1.97, xu1 - 1.6, xu1 + 1.2)                   # D end inside the sensor rotor
    n = np.array([math.cos(math.radians(PHA)), math.sin(math.radians(PHA))]); t = np.array([-n[1], n[0]])
    flat = occ.prism_yz([GC - t * 0.99 + n * 3, GC - t * 3 + n * 3, GC - t * 3 - n * 3, GC - t * 0.99 - n * 3], xu1 - 1.6, xu1 + 1.3)   # flat faces -t, like the rotor
    return occ.unify(occ.fuse(g, shaft, occ.cut(dsh, flat)))
def small_parts(f):
    """bearings, pins, sleeves of the arc guide (at q = 0)"""
    L = layers(f); T = L['T']; out = {}
    hy, hz = pol(RS, PH); ly, lz = pol(RS, L_ANG); X = np.array([1.0, 0, 0])
    out['MCP_Bearing_4x8x3_H'] = bearing([-1.5, hy, hz], X)
    out['MCP_Pin2x10_H'] = occ.cyl([-5.0, hy, hz], X, 1.0, 10.0)
    xu0 = L['xU'][0]; xr1 = L['xR'][1]
    out['MCP_SleeveH'] = occ.fuse(tube([-1.5, hy, hz], X, 1.975, 1.03, 3.0), tube([1.5, hy, hz], X, 2.6, 1.03, xu0 - 1.5 - 0.05))
    out['MCP_RingH'] = tube([xr1 + 0.05, hy, hz], X, 2.6, 1.03, -1.5 - xr1 - 0.05)
    out['MCP_Pin2x10_L'] = occ.cyl([-5.0, ly, lz], X, 1.0, 10.0)
    out['MCP_Bearing_4x8x3_L1'] = bearing([xr1 - 3.0, ly, lz], X); out['MCP_Bearing_4x8x3_L2'] = bearing([xu0, ly, lz], X)
    out['MCP_SleeveL_1'] = occ.fuse(tube([xr1 - 3.0, ly, lz], X, 1.975, 1.03, 3.0), tube([xr1, ly, lz], X, 2.6, 1.03, GAP - 0.03))
    out['MCP_SleeveL_2'] = occ.fuse(tube([xu0, ly, lz], X, 1.975, 1.03, 3.0), tube([T / 2 + 0.03, ly, lz], X, 2.6, 1.03, GAP - 0.03))
    return out
def aa_parts(f):
    d = aa_dir(f); d = d / np.linalg.norm(d); out = {}
    P = lambda U: aa_pt(f, U)
    ln = lambda U0, U1: float(np.linalg.norm(P(U1) - P(U0)))
    out['AA_Pin3x33'] = occ.cyl(P(49.2), d, 1.5, ln(49.2, 82.6))
    out['AA_Bearing_4x8x3_1'] = occ.cut(occ.cyl(P(U_B1[0]), d, 4.0, ln(*U_B1)), occ.cyl(P(U_B1[0] - 1), d, 2.0, ln(*U_B1) + 2))
    out['AA_Bearing_4x8x3_2'] = occ.cut(occ.cyl(P(U_B2[0]), d, 4.0, ln(*U_B2)), occ.cyl(P(U_B2[0] - 1), d, 2.0, ln(*U_B2) + 2))
    low = occ.fuse(occ.cyl(P(ZB + 0.05), d, 2.6, ln(ZB + 0.05, U_B1[0])), occ.cyl(P(U_B1[0] - 0.2), d, 1.975, ln(U_B1[0] - 0.2, U_B1[1])))
    out['AA_SleeveLower'] = occ.unify(occ.cut(low, occ.cyl(P(ZB - 1), d, 1.53, ln(ZB - 1, U_B1[1] + 1))))
    tip = occ.fuse(occ.cyl(P(U_B2[0]), d, 1.975, ln(U_B2[0], U_B2[1] + 0.2)), occ.cyl(P(U_B2[1]), d, 3.0, ln(U_B2[1], 83.2)))
    tip = occ.cut(tip, occ.cyl(P(U_B2[0] - 1), d, 1.53, ln(U_B2[0] - 1, 82.8)))
    ds = occ.cyl(P(83.0), d, 1.97, ln(83.0, 87.4))
    n = np.array([0.0, d[2], -d[1]]); c = P(83.25) + n * 1.0
    flat = occ.move(occ.box(-3, 0, 0, 3, 3, 4.6), occ.trsf_from_frame(c, [1, 0, 0], n, d))
    out['AA_DTip'] = occ.unify(occ.fuse(tip, occ.cut(ds, flat)))
    return out
def world(f): d = FR[f]; return occ.trsf_from_frame(d['O'], d['X'], d['E'], d['U'])
def to_world(f, p):
    d = FR[f]; return np.array(d['O']) + np.array(d['X']) * p[0] + np.array(d['E']) * p[1] + np.array(d['U']) * p[2]
def sensors(f):
    """3382H bodies (world): AA sensor moved up along its axis, MCP sensor on the ulnar cheek. Uses the v7 sensor solid of this finger."""
    import geo
    S = json.load(open('/home/claude/cache/v7/aa_sensors.json'))[f]; d = FR[f]; p = np.array(d['aa_p']); v = np.array(d['aa_v'])
    src = geo.brep('v7', S['id']); e1 = np.array(S['e1']); e2 = np.array(S['e2'])
    s_new = float((to_world(f, aa_pt(f, 83.2)) - p) @ v)
    out = {'3382H_AA': occ.translate(src, v * (s_new - S['s_bot']))}
    L = layers(f); t = np.array([0.0, -math.sin(math.radians(PHA)), math.cos(math.radians(PHA))]); X = np.array([1.0, 0, 0])
    Pd = to_world(f, np.array([L['xU'][1] - 1.6, GC[0], GC[1]])); R = np.array([d['X'], d['E'], d['U']]).T      # local -> world rotation
    b3 = R @ X; b1 = R @ (-t); b2 = np.cross(b3, b1); Ps = p + v * S['s_bot']
    M = np.outer(b1, e1) + np.outer(b2, e2) + np.outer(b3, v); tr = Pd - M @ Ps
    g = occ.gp_Trsf(); g.SetValues(M[0, 0], M[0, 1], M[0, 2], tr[0], M[1, 0], M[1, 1], M[1, 2], tr[1], M[2, 0], M[2, 1], M[2, 2], tr[2])
    out['3382H_MCP'] = occ.move(src, g)
    return out
if __name__ == '__main__':
    import time
    for f in ('middle',):
        t = time.time(); L = layers(f); print(f, L, 'AA', aa(f))
        parts = dict(arc=link_arc(f), cheekR=cheek_radial(f), cheekU=cheek_ulnar(f), gear=gear(f)); parts.update(small_parts(f)); parts.update(aa_parts(f))
        for k, v in parts.items(): occ.report(k, v); occ.save(v, f'/home/claude/cache/t_{k}.brep')
        print('time', round(time.time() - t, 1))
