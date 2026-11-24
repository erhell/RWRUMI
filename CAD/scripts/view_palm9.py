import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, occ, geo, rend
from PIL import Image
fn = sys.argv[1] if len(sys.argv) > 1 else '/home/claude/cache/v9/Palm_stage1.brep'
V, F = geo.mesh(occ.load(fn), 0.08)
c = np.array([3, 140, 50.0]); ims = []
for name, dv, up in [('iso_top_ulnar', (0.8, -0.7, 0.7), (0, 0, 1)), ('iso_top_radial', (-0.8, -0.7, 0.7), (0, 0, 1)), ('back', (0, -1, 0.05), (0, 0, 1)), ('below', (0.3, -0.5, -1), (0, 0, 1)), ('top', (0, 0, 1), (0, 1, 0)), ('ulnar', (1, 0, 0), (0, 0, 1))]:
    f = f'/home/claude/out/palm9_{name}.png'
    rend.render([(V, F, (0.8, 0.81, 0.84))], f, pos=c + 500 * np.array(dv, float), focal=c, up=up, size=(1100, 850), edges=True, scale=92)
    ims.append(Image.open(f))
W = Image.new('RGB', (3300, 1700), 'white')
for k, im in enumerate(ims): W.paste(im, ((k % 3) * 1100, (k // 3) * 850))
W.resize((2000, 1030)).save('/home/claude/out/palm9_views.png')
