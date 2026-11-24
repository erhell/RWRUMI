import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, rend
from PIL import Image, ImageDraw
M = geo.meta('v7')
pal = {'Carpal': (0.78, 0.78, 0.8), 'Guide': (0.95, 0.6, 0.2), 'Proximal': (0.3, 0.6, 0.9), 'Distal': (0.4, 0.8, 0.5), 'T-': (0.8, 0.5, 0.8), 'R-T': (0.7, 0.4, 0.7), 'Encoder': (0.1, 0.1, 0.1), 'Bearing': (0.9, 0.1, 0.1)}
items = []
for d in M:
    V, F = geo.mesh_cached('v7', d['i'])
    if len(F) < 4: continue
    items.append((V, F, next((c for k, c in pal.items() if k in d['path'][1]), (0.6, 0.6, 0.6))))
# a table plane 150 mm below palm and a few objects for context
def box(c, s):
    c = np.array(c, float); s = np.array(s, float) / 2
    V = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)]) * s + c
    F = np.array([[0,1,3],[0,3,2],[4,6,7],[4,7,5],[0,4,5],[0,5,1],[2,3,7],[2,7,6],[0,2,6],[0,6,4],[1,5,7],[1,7,3]])
    return V, F
items.append((*box((0, 230, -122), (500, 500, 2)), (0.93, 0.9, 0.82)))
items.append((*box((10, 250, -91), (50, 50, 60)), (0.85, 0.35, 0.3)))
items.append((*box((-70, 300, -101), (40, 40, 40)), (0.3, 0.5, 0.8)))
out = []
for xc, tilt, yl in [(15, 0, 69.5), (15, 5, 69.5), (15, 0, 27.5)]:
    pos = np.array([xc, yl, -32.0])
    t = np.radians(tilt); dirv = np.array([0, np.cos(t), np.sin(t)])
    fn = f'/home/claude/out/camview_y{yl:.0f}_t{tilt}.png'
    rend.render(items, fn, pos=pos, focal=pos + 200 * dirv, up=(0, 0, 1), size=(1200, 900), parallel=False, view_angle=55.0)
    im = Image.open(fn).convert('RGB'); dr = ImageDraw.Draw(im)
    for k in (1, 2): dr.line([(0, 300 * k), (1200, 300 * k)], fill=(220, 0, 0), width=2)
    im.save(fn); out.append(im)
W = Image.new('RGB', (3600, 900), 'white')
for k, im in enumerate(out): W.paste(im, (k * 1200, 0))
W.resize((2400, 600)).save('/home/claude/out/camview_compare.png')
