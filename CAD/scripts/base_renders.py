import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, rend, json
M = geo.meta('v7')
pal = {'Carpal': (0.80, 0.81, 0.83), 'Guide': (0.96, 0.72, 0.38), 'Proximal': (0.55, 0.74, 0.93), 'Distal': (0.55, 0.82, 0.62), 'T-': (0.82, 0.62, 0.84), 'R-T': (0.75, 0.52, 0.76), 'Encoder': (0.15, 0.15, 0.15), 'Bearing': (0.85, 0.2, 0.2)}
def items(filt=None):
    out = []
    for d in M:
        nm = d['path'][1]
        if filt and not filt(nm): continue
        V, F = geo.mesh_cached('v7', d['i'])
        if len(F) < 4: continue
        out.append((V, F, next((c for k, c in pal.items() if k in nm), (0.6, 0.6, 0.6))))
    return out
info = {}
# side view from the ulnar side (+x): right = +y, up = +z
f = np.array([0.0, 130.0, 20.0]); ps = 105.0; size = (1900, 1100)
rend.render(items(), '/home/claude/out/base_side.png', pos=f + np.array([600, 0, 0]), focal=f, up=(0, 0, 1), size=size, edges=True, scale=ps)
info['side'] = dict(focal=f.tolist(), ps=ps, size=size)
# thumb view along the IP axis
T = json.load(open('/home/claude/cache/v7/thumb_axes.json'))
a = np.array(T['T4'][1]); a /= np.linalg.norm(a); Mc = np.array(T['T3'][0]); Ic = np.array(T['T4'][0]); d = Ic - Mc; d -= a * (d @ a); d /= np.linalg.norm(d)
for sgn, tag in ((1, 'a'), (-1, 'b')):
    view = a * sgn; up = np.cross(d, view) * 1.0   # right = d  => up = view x right ... choose so that right=d
    up = np.cross(view, d)
    f2 = (Mc + Ic) / 2 + d * 8; ps2 = 52.0; size2 = (1700, 1100)
    rend.render(items(lambda n: n.startswith('T-') or n.startswith('R-T') or 'Encoder_3382H_T' in n or 'Bearing' in n or 'Carpal' in n), f'/home/claude/out/base_thumb_{tag}.png', pos=f2 + view * 500, focal=f2, up=up, size=size2, edges=True, scale=ps2)
    info['thumb_' + tag] = dict(focal=f2.tolist(), ps=ps2, size=size2, view=view.tolist(), up=up.tolist(), right=np.cross(up, view).tolist(), d=d.tolist(), M=Mc.tolist(), I=Ic.tolist())
json.dump(info, open('/home/claude/cache/base_info.json', 'w'), indent=1)
