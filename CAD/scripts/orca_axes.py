"""Joint axes of the ORCA v2 right hand from the bearings in Base_Right.step."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, json, geo
m = geo.meta('orca')
def bearing_axis(i):
    best = None
    for j in (i, i + 1, i + 2):
        for a in geo.analytic(geo.brep('orca', j)):
            if a.get('type') == 'cyl' and (best is None or a['r'] > best['r']): best = a
    V, F = geo.mesh(geo.brep('orca', i)); c = (V.min(0) + V.max(0)) / 2
    p = best['p'] + best['v'] * np.dot(c - best['p'], best['v']); return p, best['v'] / np.linalg.norm(best['v']), best['r']
J = {'index': dict(aa=(796, 799), mcp=(779, 782), pip=(787, 790)), 'middle': dict(aa=(824, 827), mcp=(807, 810), pip=(815, 818)), 'pinky': dict(aa=(852, 855), mcp=(835, 838), pip=(843, 846)),
     'thumb': dict(cmc=(1282, 1285), abd=(1273, 1276), mcp=(1256, 1259), ip=(1263, 1268))}
# ring = second instance of the middle finger: find the bodies of 'M (1)'
ring = [d['i'] for d in m if d['path'][1].startswith('M (1)')]; off = ring[0] - 803
J['ring'] = dict(aa=(824 + off, 827 + off), mcp=(807 + off, 810 + off), pip=(815 + off, 818 + off))
out = {}
for f, d in J.items():
    out[f] = {}
    for j, (a, b) in d.items():
        pa, va, ra = bearing_axis(a); pb, vb, rb = bearing_axis(b)
        v = pb - pa; L = np.linalg.norm(v); v = v / L
        ang = np.degrees(np.arccos(min(1, abs(np.dot(v, va))))); angb = np.degrees(np.arccos(min(1, abs(np.dot(va, vb)))))
        mid = (pa + pb) / 2; out[f][j] = dict(p=mid.tolist(), v=va.tolist(), pa=pa.tolist(), pb=pb.tolist())
        print(f'{f:6s} {j:4s} bearings {a},{b}: r {ra:.2f}/{rb:.2f}, centres {L:6.2f} mm apart, line vs bearing axis {ang:.3f} deg, bearing vs bearing {angb:.3f} deg, p {mid.round(2)}, v {va.round(4)}')
json.dump(out, open('/home/claude/cache/orca/axes.json', 'w'), indent=1)
