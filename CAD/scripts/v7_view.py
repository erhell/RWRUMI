import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, rend, pickle
M = geo.meta('v7')
pal = {'Carpal': (0.75, 0.75, 0.78), 'Guide': (0.95, 0.6, 0.2), 'Proximal': (0.3, 0.6, 0.9), 'Distal': (0.4, 0.8, 0.5), 'T-': (0.8, 0.5, 0.8), 'R-T': (0.7, 0.4, 0.7), 'Encoder': (0.1, 0.1, 0.1), 'Bearing': (0.9, 0.1, 0.1)}
items = []
for d in M:
    V, F = geo.mesh_cached('v7', d['i']); nm = d['path'][1]
    col = next((c for k, c in pal.items() if k in nm), (0.6, 0.6, 0.6))
    if len(F) < 3: continue
    items.append((V, F, col))
c = np.array([0, 170, 40.])
for name, dirv, up in [('iso1', (-1, -1, 0.7), (0, 0, 1)), ('side_ulnar', (1, 0, 0), (0, 0, 1)), ('top', (0, 0, 1), (0, 1, 0)), ('bottom', (0, 0, -1), (0, 1, 0)), ('front', (0, 1, 0), (0, 0, 1)), ('back', (0, -1, 0.0), (0, 0, 1)), ('iso2', (1, -1, -0.6), (0, 0, 1))]:
    rend.render(items, f'/home/claude/out/v7_{name}.png', pos=c + 600 * np.array(dirv), focal=c, up=up, size=(1300, 950), edges=True)
print('ok')
