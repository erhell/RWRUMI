"""Finger frames for v9: origin O on the MCP axis at the web mid-plane; X = MCP axis (ulnar +), E = finger direction in the v7 file pose, U = X x E (dorsal)."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, json
ax = json.load(open('/home/claude/cache/v7/axes.json'))
WEB = {'index': (-1.615, 3.281), 'middle': (-4.279, 0.506), 'ring': (-4.279, 0.506), 'pinky': (-1.5, 3.4)}
IDS = {'index': dict(prox=31, enc=32, dist=38, guide=list(range(1, 13))), 'middle': dict(prox=33, enc=34, dist=39, guide=list(range(13, 23))), 'ring': dict(prox=51, enc=52, dist=53, guide=list(range(41, 51))), 'pinky': dict(prox=35, enc=36, dist=37, guide=list(range(23, 31)))}
def build():
    fr = {}
    for f in ('index', 'middle', 'ring', 'pinky'):
        a = ax[f]; c0 = np.array(a['mcp_c']); m = np.array(a['mcp_v']); v = np.array(a['aa_v']); pip = np.array(a['pip_c']); aap = np.array(a['aa_p'])
        e = pip - c0; e -= m * (e @ m); e /= np.linalg.norm(e); u = np.cross(m, e)
        xr, xu = WEB[f]; T = xu - xr; O = c0 + m * (xr + xu) / 2
        d = aap - O; aaX, aaE, aaU = d @ m, d @ e, d @ u; dE, dU = v @ e, v @ u
        # AA axis point at U = 0
        t = -aaU / dU; E0 = aaE + dE * t
        fr[f] = dict(O=O.tolist(), X=m.tolist(), E=e.tolist(), U=u.tolist(), T=T, aa_X=aaX, aa_E0=E0, aa_dE=dE, aa_dU=dU, aa_p=aap.tolist(), aa_v=v.tolist(), pip_r=float(np.linalg.norm(pip - c0)), ids=IDS[f])
    return fr
if __name__ == '__main__':
    fr = build(); json.dump(fr, open('/home/claude/cache/v7/frames.json', 'w'), indent=1)
    for f, d in fr.items():
        print(f"{f:7s} T {d['T']:.3f}  AA: X {d['aa_X']:.3f}  E(U=0) {d['aa_E0']:.3f}  dir(E,U) ({d['aa_dE']:.4f}, {d['aa_dU']:.4f}) lean {np.degrees(np.arctan2(d['aa_dE'], d['aa_dU'])):.2f} deg  X-component of AA dir {np.dot(d['aa_v'], d['X']):.2e}  PIP r {d['pip_r']:.2f}")
    A = geo.analytic(geo.brep('v7', 22)); cy = sorted([q for q in A if q['type'] == 'cyl'], key=lambda q: -q['area'])
    a = fr['middle']; v = np.array(a['aa_v']); p = np.array(a['aa_p'])
    print('3382H (AA sensor, middle):')
    for q in cy[:8]: print('   cyl r %.3f area %.1f axis.v %.3f offset from AA axis %.3f s %.2f' % (q['r'], q['area'], abs(q['v'] @ v), np.linalg.norm(np.cross(q['p'] - p, v)), (q['cen'] - p) @ v))
    V, _ = geo.mesh_cached('v7', 22); s = (V - p) @ v; print('   along AA: s %.2f..%.2f' % (s.min(), s.max()))
    for q in sorted([q for q in A if q['type'] == 'plane'], key=lambda q: -q['area'])[:8]: print('   plane area %.1f n.v %.2f s=%.2f' % (q['area'], q['v'] @ v, (q['cen'] - p) @ v))
    Vh, _ = geo.mesh_cached('v7', 14); s = (Vh - p) @ v; print('hub s range', s.min(), s.max())
