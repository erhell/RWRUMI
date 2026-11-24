import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, rend, thumb9 as T
from PIL import Image
a, d, w, M, G = T.a, T.d, T.w, T.M, T.G
cols = {56: (0.75, 0.52, 0.76), 57: (0.86, 0.72, 0.88), 58: (0.7, 0.85, 0.95), 61: (0.1, 0.1, 0.1), 62: (0.1, 0.1, 0.1), 68: (0.85, 0.2, 0.2), 69: (0.85, 0.2, 0.2), 55: (0.9, 0.85, 0.7)}
items = [(*geo.mesh_cached('v7', i), c) for i, c in cols.items()]
c = M + d * 16 + a * 12
fns = []
for name, dv, up in [('far_side', a, w), ('sensor_side', -a, w), ('dorsal', w, d), ('palmar', -w, d), ('tip_on', d, w), ('iso', a * 0.7 + w * 0.6 - d * 0.4, w)]:
    fn = f'/home/claude/out/thumbp_{name}.png'; rend.render(items, fn, pos=c + np.array(dv) * 400, focal=c, up=up, size=(1000, 800), edges=True, scale=42); fns.append(fn)
ims = [Image.open(f) for f in fns]; W = Image.new('RGB', (3000, 1600), 'white')
for k, im in enumerate(ims): W.paste(im, ((k % 3) * 1000, (k // 3) * 800))
W.resize((2000, 1067)).save('/home/claude/out/thumb_probe.png')
for nm, i in (('AP', 56), ('PP', 57), ('DP', 58)):
    V, _ = geo.mesh_cached('v7', i); r = V - M; lat = r @ a; u = r @ d; v = r @ w
    print(nm)
    for lo in range(-6, 44, 6):
        m = (lat >= lo) & (lat < lo + 6)
        if m.any(): print('   lateral %3d..%3d: d [%.1f, %.1f]  w [%.1f, %.1f]' % (lo, lo + 6, u[m].min(), u[m].max(), v[m].min(), v[m].max()))
