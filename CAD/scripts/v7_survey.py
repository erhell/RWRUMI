import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, pickle
M = geo.meta('v7'); S = {}
for d in M:
    sh = geo.brep('v7', d['i']); V, F = geo.mesh_cached('v7', d['i'])
    vol, c = geo.props(sh)
    S[d['i']] = dict(name=d['path'][1].split(':')[0], vol=vol, c=c, lo=V.min(0), hi=V.max(0))
    print(f"{d['i']:2d} {S[d['i']]['name']:28s} vol {vol:9.1f} c {np.round(c,1)} lo {np.round(V.min(0),1)} hi {np.round(V.max(0),1)} nf {d['nface']}")
pickle.dump(S, open('/home/claude/cache/v7/survey.pkl', 'wb'))
