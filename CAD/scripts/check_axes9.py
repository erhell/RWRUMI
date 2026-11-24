"""New functional surfaces of v9 against the v7 (= ORCA) axes."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, json, math, geo, occ
from OCP.TopoDS import TopoDS_Shape
FR = json.load(open('/home/claude/cache/v7/frames.json')); A = lambda x: np.array(x, float)
def cyls(name): return [a for a in geo.analytic(occ.load(f'/home/claude/cache/v9/{name}.brep')) if a.get('type') == 'cyl']
def rel(c, p, v):
    v = v / np.linalg.norm(v); ang = math.degrees(math.acos(min(1, abs(np.dot(c['v'], v))))); d = c['p'] - p; d = d - v * np.dot(d, v); return ang, np.linalg.norm(d)
palm = cyls('Palm')
for f in ('index', 'middle', 'ring', 'pinky'):
    d = FR[f]; N = f.capitalize(); p, v = A(d['aa_p']), A(d['aa_v']); O, X = A(d['O']), A(d['X'])
    row = []
    sel = [c for c in palm if rel(c, p, v)[0] < 0.5 and rel(c, p, v)[1] < 0.5]
    for r0 in (4.05, 2.6): 
        cc = [c for c in sel if abs(c['r'] - r0) < 0.02]; row.append(f'palm Ø{2*r0:.1f} x{len(cc)}: max {max(rel(c, p, v)[0] for c in cc):.4f} deg / {max(rel(c, p, v)[1] for c in cc):.4f} mm')
    ch = cyls(f'{N}_MCP_CheekRadial'); cc = [c for c in ch if abs(c['r'] - 1.475) < 0.02 and rel(c, p, v)[1] < 0.5]; row.append(f'cheek pin bore Ø2.95 x{len(cc)}: {max(rel(c, p, v)[0] for c in cc):.4f} deg / {max(rel(c, p, v)[1] for c in cc):.4f} mm')
    pin = cyls(f'{N}_AA_Pin3x33'); row.append(f'pin: {rel(pin[0], p, v)[0]:.4f} deg / {rel(pin[0], p, v)[1]:.4f} mm')
    print(f'{f:6s} AA : ' + '; '.join(row))
    row = []
    pr = cyls(f'{N}_Proximal'); cc = [c for c in pr if c['r'] > 37 and rel(c, O, X)[0] < 0.01 and rel(c, O, X)[1] < 0.01]; row.append('arc faces about the MCP axis r = ' + ', '.join(sorted(set(f"{c['r']:.2f}" for c in cc))))
    bad = [c for c in pr if c['r'] > 37 and c['r'] < 60 and rel(c, O, X)[0] < 1 and 0.01 <= rel(c, O, X)[1] < 3]; row.append(f'{len(bad)} off-centre')
    for part in ('MCP_CheekRadial', 'MCP_CheekUlnar'):
        cc = [c for c in cyls(f'{N}_{part}') if c['r'] > 37 and rel(c, O, X)[0] < 0.01]; row.append(f'{part[4:]} slot r = ' + ', '.join(sorted(set(f"{c['r']:.2f}" for c in cc))) + f' (max offset {max(rel(c, O, X)[1] for c in cc):.4f} mm)')
    print(f'{f:6s} MCP: ' + '; '.join(row))
import thumb9 as TH
for n in ('Thumb_Link', 'Thumb_AP', 'Thumb_DP', 'Thumb_Pin2x10_LinkA', 'Thumb_Pin2x10_LinkB'):
    cc = [c for c in cyls(n) if abs(c['r'] - 1.0) < 0.15]; print(f'{n}: {len(cc)} pin-size cylinders, angle to the thumb MCP axis max {max(rel(c, TH.M, TH.a)[0] for c in cc):.4f} deg' if cc else f'{n}: none')
