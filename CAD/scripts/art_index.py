import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, json
np.set_printoptions(precision=2, suppress=True)
M = geo.meta('art')
idx = [d for d in M if 'Index_Finger_Ass' in d['path'][1] and 'encoder' not in d['path'][-1].lower() and 'FFC' not in d['path'][-1]]
for d in idx:
    V, F = geo.mesh_cached('art', d['i'], 0.05)
    print(d['i'], d['path'][-1], np.round(V.min(0), 1), np.round(V.max(0), 1), d['nface'])
