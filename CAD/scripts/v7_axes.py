import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, pickle
np.set_printoptions(precision=3, suppress=True)
def cyls(i, rmin=0, rmax=1e9, amin=1.0):
    out = [d for d in geo.analytic(geo.brep('v7', i)) if d['type'] == 'cyl' and rmin <= d['r'] <= rmax and d['area'] >= amin]
    return sorted(out, key=lambda d: -d['area'])
for i in [33, 13, 14, 15, 16, 17, 18, 39]:
    print('--- body', i)
    for d in cyls(i)[:14]:
        print(f"  r {d['r']:7.3f} area {d['area']:8.1f} p {d['p']} v {d['v']} cen {d['cen']}")
