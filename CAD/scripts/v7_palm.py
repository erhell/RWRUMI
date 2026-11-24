import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, rend
V, F = geo.mesh_cached('v7', 0)
c = np.array([0, 135, 35.])
for name, dirv, up in [('palm_iso_ulnar_below', (1, -0.6, -0.8), (0, 0, 1)), ('palm_below', (0, 0, -1), (0, 1, 0)), ('palm_ulnar', (1, 0, 0), (0, 0, 1)), ('palm_back', (0, -1, 0), (0, 0, 1)), ('palm_iso_top', (0.7, -0.8, 0.7), (0, 0, 1)), ('palm_radial', (-1, 0, 0), (0, 0, 1))]:
    rend.render([(V, F, (0.78, 0.78, 0.8))], f'/home/claude/out/{name}.png', pos=c + 500 * np.array(dirv, float), focal=c, up=up, size=(1200, 900), edges=True, scale=75)
