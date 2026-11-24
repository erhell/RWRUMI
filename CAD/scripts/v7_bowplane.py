import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, json
np.set_printoptions(precision=2, suppress=True)
ax = json.load(open('/home/claude/cache/v7/axes.json'))
F = {'index': dict(guide=list(range(1, 13)), prox=31, enc=32), 'middle': dict(guide=list(range(13, 23)), prox=33, enc=34), 'ring': dict(guide=list(range(41, 51)), prox=51, enc=52), 'pinky': dict(guide=list(range(23, 31)), prox=35, enc=36)}
for f, d in F.items():
    a = ax[f]; c = np.array(a['mcp_c']); m = np.array(a['mcp_v']); v = np.array(a['aa_v']); fw = np.cross(v, m)
    A = geo.analytic(geo.brep('v7', d['prox'])); big = max([q for q in A if q['type'] == 'cyl'], key=lambda q: q['area'] if 40 < q['r'] < 52 else 0)
    s0 = (big['p'] - c) @ m * 0  # ref
    lo = (big['p'] + big['v'] * big['vv'][0] - c) @ m; hi = (big['p'] + big['v'] * big['vv'][1] - c) @ m
    aa_lat = (np.array(a['aa_p']) - c) @ m
    def ext(i):
        V, _ = geo.mesh_cached('v7', i); s = (V - c) @ m; return s.min(), s.max()
    print(f, 'bow lateral [%.2f, %.2f]' % (min(lo, hi), max(lo, hi)), 'AA lateral %.2f' % aa_lat, '| prox', np.round(ext(d['prox']), 2), '| PIP enc', np.round(ext(d['enc']), 2))
    for i in d['guide']:
        V, _ = geo.mesh_cached('v7', i); s = (V - c) @ m; rel = V - c; y = rel @ fw; z = rel @ v
        print('    guide body', i, 'lat [%.2f, %.2f]' % (s.min(), s.max()), 'Y [%.1f, %.1f] Z [%.1f, %.1f]' % (y.min(), y.max(), z.min(), z.max()))
    ax[f]['bow_lat'] = [float(min(lo, hi)), float(max(lo, hi))]; ax[f]['aa_lat'] = float(aa_lat)
json.dump(ax, open('/home/claude/cache/v7/axes.json', 'w'), indent=1)
