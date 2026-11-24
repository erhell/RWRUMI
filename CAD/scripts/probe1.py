import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, json
np.set_printoptions(precision=3, suppress=True)
ax = json.load(open('/home/claude/cache/v7/axes.json'))
F = {'index': dict(prox=31, enc=32, dist=38, hub=11, aasens=12), 'middle': dict(prox=33, enc=34, dist=39, hub=14, aasens=22), 'ring': dict(prox=51, enc=52, dist=53, hub=42, aasens=50), 'pinky': dict(prox=35, enc=36, dist=37, hub=24, aasens=30)}
fr = {}
for f, d in F.items():
    a = ax[f]; c0 = np.array(a['mcp_c']); m = np.array(a['mcp_v']); v = np.array(a['aa_v']); pip = np.array(a['pip_c']); aap = np.array(a['aa_p'])
    e = pip - c0; e -= m * (e @ m); e /= np.linalg.norm(e); u = np.cross(m, e)
    V, _ = geo.mesh_cached('v7', d['prox'], 0.08); rel = V - c0; X = rel @ m; E = rel @ e; U = rel @ u; r = np.hypot(E, U); ang = np.degrees(np.arctan2(U, E))
    # bow flange region just inside the old slot: r 41.2..42.8, angle 40..100
    sel = (r > 41.2) & (r < 42.8) & (ang > 40) & (ang < 100); xb = (X[sel].min(), X[sel].max())
    selw = (r > 20) & (r < 34) & (ang > 15) & (ang < 45); xw = (X[selw].min(), X[selw].max()) if selw.any() else None
    selo = (r > 39.6) & (ang > 14); xo = (X[selo].min(), X[selo].max()); ao = (ang[r > 44].min(), ang[r > 44].max()); rmax = r.max()
    xmid = (xb[0] + xb[1]) / 2; O = c0 + m * xmid
    aax = (aap - O) @ m; aaE = (aap - O) @ e; aaU = (aap - O) @ u; dv = (v @ e, v @ u)
    print(f'{f:7s} bow plate X [{xb[0]:.3f},{xb[1]:.3f}] mid {xmid:.3f} | web X {np.round(xw,3) if xw else None} | old bow region X [{xo[0]:.2f},{xo[1]:.2f}] ang(r>44) [{ao[0]:.1f},{ao[1]:.1f}] rmax {rmax:.2f}')
    print(f'        O {O}  AA: X {aax:.3f} E {aaE:.3f} U {aaU:.3f} dir(E,U) ({dv[0]:.4f},{dv[1]:.4f}) tilt {np.degrees(np.arctan2(dv[0], dv[1])):.2f} deg | PIP r {np.linalg.norm(pip - c0):.3f}')
    fr[f] = dict(O=O.tolist(), m=m.tolist(), e=e.tolist(), u=u.tolist(), aa_p=aap.tolist(), aa_v=v.tolist(), aa_X=float(aax), aa_E=float(aaE), aa_U=float(aaU), aa_dE=float(dv[0]), aa_dU=float(dv[1]), pip_r=float(np.linalg.norm(pip - c0)), ids=d)
json.dump(fr, open('/home/claude/cache/v7/frames.json', 'w'), indent=1)
# 3382H geometry (body 22 = AA sensor of the middle finger)
A = geo.analytic(geo.brep('v7', 22)); cy = sorted([q for q in A if q['type'] == 'cyl'], key=lambda q: -q['area'])
print('3382H cylinders:'); [print('   r %.3f area %.1f axis %s len %.2f' % (q['r'], q['area'], q['v'], abs(q['vv'][1] - q['vv'][0]))) for q in cy[:8]]
V, _ = geo.mesh_cached('v7', 22); a = ax['middle']; v = np.array(a['aa_v']); p = np.array(a['aa_p']); s = (V - p) @ v; print('   along AA axis: s %.2f..%.2f' % (s.min(), s.max()))
pl = sorted([q for q in A if q['type'] == 'plane'], key=lambda q: -q['area'])[:6]
for q in pl: print('   plane area %.1f n %s s=%.2f' % (q['area'], q['v'], (q['cen'] - p) @ v))
