import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, occ, geo, rend, glob, os
cols = dict(arc=(0.45, 0.68, 0.92), cheekR=(0.96, 0.7, 0.33), cheekU=(0.98, 0.8, 0.45), gear=(0.72, 0.79, 0.3))
def col(k):
    if k in cols: return cols[k]
    if 'Bearing' in k: return (0.85, 0.2, 0.2)
    if 'Pin' in k: return (0.3, 0.3, 0.33)
    return (0.6, 0.85, 0.6)
def items(skip=()):
    out = []
    for fn in sorted(glob.glob('/home/claude/cache/t_*.brep')):
        k = os.path.basename(fn)[2:-5]
        if k in skip: continue
        V, F = geo.mesh(occ.load(fn), 0.05); out.append((V, F, col(k)))
    return out
c = np.array([0, 15, 45.0])
views = [('loc_radial', (-1, 0, 0), ()), ('loc_ulnar', (1, 0, 0), ()), ('loc_iso', (-0.7, 0.6, 0.6), ()), ('loc_iso_noR', (-0.8, 0.3, 0.5), ('cheekR',)), ('loc_iso_noU', (0.8, 0.3, 0.5), ('cheekU',)), ('loc_top', (0, 0, 1), ()), ('loc_back', (0, -1, 0.15), ())]
for name, dv, skip in views:
    up = (0, 0, 1) if abs(dv[2]) < 0.9 else (0, 1, 0)
    rend.render(items(skip), f'/home/claude/out/{name}.png', pos=c + 400 * np.array(dv, float), focal=c, up=up, size=(1200, 900), edges=True, scale=52)
