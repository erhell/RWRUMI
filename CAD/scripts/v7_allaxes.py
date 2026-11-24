import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, json
np.set_printoptions(precision=3, suppress=True)
F = {'index': dict(guide=[1, 11], prox=31, dist=38), 'middle': dict(guide=[13, 14], prox=33, dist=39), 'ring': dict(guide=[41, 42], prox=51, dist=53), 'pinky': dict(guide=[23, 24], prox=35, dist=37)}
out = {}
for f, d in F.items():
    A = geo.analytic(geo.brep('v7', d['prox'])); cy = [a for a in A if a['type'] == 'cyl']
    big = max(cy, key=lambda a: a['area'] if 40 < a['r'] < 52 else 0)   # bow rail face -> MCP axis
    mcp_p, mcp_v = big['p'], big['v']
    pip = max([a for a in cy if abs(a['r'] - 4.05) < 0.02 and abs(np.dot(a['v'], mcp_v)) > 0.99], key=lambda a: a['area'])
    H = geo.analytic(geo.brep('v7', d['guide'][1])); hc = [a for a in H if a['type'] == 'cyl']
    aa = max([a for a in hc if abs(a['r'] - 10.0) < 0.06], key=lambda a: a['area'])
    v = aa['v'] * np.sign(aa['v'][2]); m = mcp_v * np.sign(mcp_v[0])
    # web/bow plane: mid of bow along mcp axis
    V, _ = geo.mesh_cached('v7', d['prox']); rel = V - mcp_p; r = np.linalg.norm(rel - np.outer(rel @ m, m), axis=1); s = (rel @ m)[(r > 44) & (r < 47)]
    # point on MCP axis at bow mid-plane
    c = mcp_p + m * s.mean() if len(s) else mcp_p
    # AA axis point closest to MCP axis level
    t = ((c - aa['p']) @ v); aap = aa['p'] + v * t
    fwd = np.cross(v, m); fwd /= np.linalg.norm(fwd)
    pipc = pip['p'] + m * ((c - pip['p']) @ m)
    out[f] = dict(mcp_c=c.tolist(), mcp_v=m.tolist(), aa_p=aap.tolist(), aa_v=v.tolist(), pip_c=pipc.tolist(), bow_s=[float(s.min()), float(s.max())] if len(s) else None)
    print(f, 'MCP c', c, 'dir', m, '| AA p', aap, 'v', v, '| PIP', pipc)
    print('   bow width along axis', s.min() if len(s) else None, s.max() if len(s) else None, '| AA-to-MCP: behind', (c - aap) @ fwd, 'lateral (along mcp axis)', (aap - c) @ m, '| MCP-PIP', np.linalg.norm(pipc - c), '| angle AA vs MCP', np.degrees(np.arccos(abs(v @ m))))
    hub = geo.mesh_cached('v7', d['guide'][1])[0]; print('   hub z', hub[:, 2].min(), hub[:, 2].max())
json.dump(out, open('/home/claude/cache/v7/axes.json', 'w'), indent=1)
