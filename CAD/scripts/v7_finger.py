import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, rend
cols = {33: (0.3, 0.6, 0.9), 13: (0.95, 0.6, 0.2), 14: (0.9, 0.4, 0.1), 15: (0.95, 0.8, 0.3), 16: (0.6, 0.3, 0.1), 17: (0.5, 0.5, 0.1), 18: (0.7, 0.7, 0.2), 19: (0.2,0.2,0.2), 20:(0.2,0.2,0.2), 21: (0.1, 0.1, 0.1), 22: (0.1, 0.1, 0.1), 34: (0.1, 0.1, 0.1), 39: (0.4, 0.8, 0.5), 40:(0,0,0)}
def view(ids, name, dirv, up=(0, 0, 1), c=(-10, 205, 50), sc=60, extra=()):
    items = [(*geo.mesh_cached('v7', i), cols.get(i, (0.7, 0.7, 0.7))) for i in ids] + list(extra)
    c = np.array(c, float)
    rend.render(items, f'/home/claude/out/{name}.png', pos=c + 500 * np.array(dirv, float), focal=c, up=up, size=(1300, 1000), edges=True, scale=sc)
mid = [33, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 34, 39]
view(mid + [0], 'mid_side_radial', (-1, 0, 0))
view([33, 39], 'mid_prox_iso', (-1, -0.8, 0.6))
view([33, 39], 'mid_prox_front', (0.15, 1, 0.2))
view([13, 14, 15, 22], 'mid_guide_iso', (-1, -0.8, 0.6))
view(mid, 'mid_iso_ulnar', (1, -0.8, 0.6))
view(mid, 'mid_front', (0, 1, 0), sc=45)
view(mid, 'mid_top', (0, 0, 1), up=(0, 1, 0))
