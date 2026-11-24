import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo
np.set_printoptions(precision=2, suppress=True)
for i in [516, 518, 515, 521, 519, 520, 581, 583]:
    A = geo.analytic(geo.brep('art', i))
    cy = sorted([d for d in A if d['type'] == 'cyl'], key=lambda d: -d['area'])
    print('--- ', i, 'ncyl', len(cy), 'nface', len(A))
    for d in cy[:16]:
        print(f"  r {d['r']:7.3f} area {d['area']:7.1f} p {d['p']} v {d['v']} cen {d['cen']} u {np.degrees(d['u'][1]-d['u'][0]):.0f}deg")
