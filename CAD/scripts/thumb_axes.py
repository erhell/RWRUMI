import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, json
np.set_printoptions(precision=2, suppress=True)
names = {63: 'T1d', 64: 'T1d2', 65: 'T1q', 66: 'T2a', 67: 'T2b', 68: 'T3', 69: 'T4'}
ax = {}
for i, n in names.items():
    A = [a for a in geo.analytic(geo.brep('v7', i)) if a['type'] == 'cyl']
    o = max(A, key=lambda a: a['r']); V, _ = geo.mesh_cached('v7', i); c = V.mean(0)
    v = o['v']; p = o['p'] + v * ((c - o['p']) @ v)
    ax[n] = (p, v); print(n, 'centre', p, 'axis', v, 'r', o['r'])
cmc_v = ax['T1q'][0] - ax['T1d'][0]; print('CMC axis T1d->T1q', cmc_v / np.linalg.norm(cmc_v), 'len', np.linalg.norm(cmc_v))
print('ABD T2a->T2b', ax['T2b'][0] - ax['T2a'][0])
M = ax['T3'][0]; I = ax['T4'][0]; print('MCP->IP', I - M, np.linalg.norm(I - M))
for i, n in [(57, 'T-PP'), (58, 'T-DP'), (56, 'R-T-AP'), (55, 'T-TP')]:
    V, _ = geo.mesh_cached('v7', i); print(n, 'lo', V.min(0), 'hi', V.max(0))
# T-DP tip: farthest point from IP axis centre
V, _ = geo.mesh_cached('v7', 58); d = np.linalg.norm(V - I, axis=1); tip = V[d.argmax()]; print('T-DP tip', tip, 'dist from IP', d.max())
V, _ = geo.mesh_cached('v7', 57); d = np.linalg.norm(V - M, axis=1); print('T-PP far from MCP', V[d.argmax()], d.max())
json.dump({k: [list(map(float, p)), list(map(float, v))] for k, (p, v) in ax.items()}, open('/home/claude/cache/v7/thumb_axes.json', 'w'))
