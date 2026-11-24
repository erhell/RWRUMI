"""Glove (v7 = v9 axes) against ORCA v2 (Base_Right.step): palm-fixed axes by rigid fit, moving axes by pose-independent chain numbers."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, json, math
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation as R
O = json.load(open('/home/claude/cache/orca/axes.json')); O['middle'], O['ring'] = O['ring'], O['middle']     # 'M (1)' is the middle finger, 'M' the ring finger
FR = json.load(open('/home/claude/cache/v7/frames.json')); AX = json.load(open('/home/claude/cache/v7/axes.json')); TA = json.load(open('/home/claude/cache/v7/thumb_axes.json'))
FING = ('index', 'middle', 'ring', 'pinky'); A = lambda x: np.array(x, float)
G = {f: dict(aa=(A(FR[f]['aa_p']), A(FR[f]['aa_v'])), mcp=(A(FR[f]['O']), A(FR[f]['X'])), pip=(A(AX[f]['pip_c']), A(FR[f]['X']))) for f in FING}
G['thumb'] = dict(cmc=(A(TA['T1q'][0]), A(TA['T1q'][1])), abd=(A(TA['T2a'][0]), A(TA['T2a'][1])), mcp=(A(TA['T3'][0]), A(TA['T3'][1])), ip=(A(TA['T4'][0]), A(TA['T4'][1])))
Oa = {f: {j: (A(d['p']), A(d['v'])) for j, d in O[f].items()} for f in O}
def ang(u, v): return math.degrees(math.acos(min(1.0, abs(float(np.dot(u, v)) / np.linalg.norm(u) / np.linalg.norm(v)))))
def cn(l1, l2):
    """common normal of two lines: distance, feet"""
    p1, v1 = l1; p2, v2 = l2; v1 = v1 / np.linalg.norm(v1); v2 = v2 / np.linalg.norm(v2); n = np.cross(v1, v2); nn = np.linalg.norm(n)
    if nn < 1e-6: d = (p2 - p1) - v1 * np.dot(p2 - p1, v1); return np.linalg.norm(d), p1, p1 + d
    Am = np.array([v1, -v2, n / nn]).T; s, t, dd = np.linalg.solve(Am, p2 - p1); return abs(dd), p1 + s * v1, p2 + t * v2
def ptline(p, l): d = (p - l[0]) - l[1] / np.linalg.norm(l[1]) * np.dot(p - l[0], l[1] / np.linalg.norm(l[1])); return np.linalg.norm(d)
print('=== chain numbers that do not depend on the pose (glove | ORCA)')
for f in FING:
    for nm, S in (('glove', G[f]), ('ORCA ', Oa[f])):
        d1, fa, fm = cn(S['aa'], S['mcp']); d2, _, _ = cn(S['mcp'], S['pip'])
        print(f'  {f:6s} {nm}: AA-MCP angle {ang(S["aa"][1], S["mcp"][1]):8.4f} deg, AA-MCP distance {d1:7.4f} mm, MCP-PIP angle {ang(S["mcp"][1], S["pip"][1]):7.4f} deg, MCP-PIP distance {d2:8.4f} mm')
for nm, S in (('glove', G['thumb']), ('ORCA ', Oa['thumb'])):
    a1, _, f2a = cn(S['cmc'], S['abd']); a2, f2b, f3 = cn(S['abd'], S['mcp']); a3, _, _ = cn(S['mcp'], S['ip'])
    v2 = S['abd'][1] / np.linalg.norm(S['abd'][1]); off = abs(np.dot(f2b - f2a, v2))
    print(f'  thumb  {nm}: CMC-ABD {ang(S["cmc"][1], S["abd"][1]):8.4f} deg / {a1:7.4f} mm, ABD-MCP {ang(S["abd"][1], S["mcp"][1]):8.4f} deg / {a2:7.4f} mm, offset along ABD {off:6.4f} mm, MCP-IP {ang(S["mcp"][1], S["ip"][1]):6.4f} deg / {a3:8.4f} mm')
# palm-fixed features: 4 AA lines + CMC line, plus the feet of the AA-MCP / CMC-ABD common normals on them
fixed = [(f, 'aa', 'mcp') for f in FING] + [('thumb', 'cmc', 'abd')]
def feats(S):
    out = []
    for f, a, b in fixed: d, fa, fb = cn(S[f][a], S[f][b]); out.append((fa, S[f][a][1] / np.linalg.norm(S[f][a][1])))
    return out
Fg = feats(G); Fo = feats(Oa)
def res(x, use_feet=True):
    Rm = R.from_rotvec(x[:3]).as_matrix(); t = x[3:]; r = []
    for (pg, vg), (po, vo) in zip(Fg, Fo):
        p = Rm @ po + t; v = Rm @ vo
        if np.dot(v, vg) < 0: v = -v
        r += list((v - vg) * 50.0)
        d = (p - pg); r += list(d if use_feet else d - vg * np.dot(d, vg))
    return np.array(r)
for use_feet in (False, True):
    s = least_squares(res, np.zeros(6), args=(use_feet,), xtol=1e-14, ftol=1e-14, gtol=1e-14); Rm = R.from_rotvec(s.x[:3]).as_matrix(); t = s.x[3:]
    print(f'=== rigid fit ORCA -> glove on the palm-fixed axes ({"lines and feet" if use_feet else "lines only"}): rotation {np.degrees(np.linalg.norm(s.x[:3])):.5f} deg, translation {t.round(4)}')
    for (f, a, b), (pg, vg), (po, vo) in zip(fixed, Fg, Fo):
        p = Rm @ po + t; v = Rm @ vo; d = p - pg; along = np.dot(d, vg); perp = np.linalg.norm(d - vg * along)
        print(f'    {f:6s} {a:3s}: direction {ang(v, vg):.5f} deg, line offset {perp:.4f} mm, foot of the next axis along it {along:+.4f} mm')
json.dump(dict(rotvec=s.x[:3].tolist(), t=s.x[3:].tolist()), open('/home/claude/cache/orca/to_glove.json', 'w'))
