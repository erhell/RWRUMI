import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, trimesh, json, finger9 as F
V, Fc = geo.mesh_cached('v7', 0); m = trimesh.Trimesh(V, Fc, process=False)
print('palm bbox', V.min(0).round(1), V.max(0).round(1))
for f in ('index', 'middle', 'ring', 'pinky'):
    d = F.FR[f]; O = np.array(d['O']); X, E, U = (np.array(d[k]) for k in 'XEU'); v = np.array(d['aa_v']); p = np.array(d['aa_p'])
    # hits along AA axis
    loc, _, _ = m.ray.intersects_location([p - v * 40], [v], multiple_hits=True)
    s = np.sort((loc - p) @ v); u_of = lambda P: (P - O) @ U
    print(f, 'AA axis hits s:', np.round(s, 2), '| as U:', np.round([u_of(p + v * q) for q in s], 2))
    # band underside around the axis: sample rays parallel to v within radius 10
    und = []
    for ang in np.linspace(0, 2 * np.pi, 16, endpoint=False):
        for rr in (4, 8, 11):
            o = p + (X * np.cos(ang) + np.cross(v, X) * np.sin(ang)) * rr + v * 20
            l, _, _ = m.ray.intersects_location([o], [v], multiple_hits=True)
            if len(l): und.append(u_of(l[np.argmin((l - o) @ v)]))
    und = np.array(und); print('    band underside (U) within r 11 of the axis: min %.2f median %.2f max %.2f' % (und.min(), np.median(und), und.max()))
