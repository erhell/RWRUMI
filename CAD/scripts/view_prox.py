import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, occ, geo, rend, finger9 as F
from PIL import Image
ims = []
for f, N in (('index', 'Index'), ('middle', 'Middle'), ('ring', 'Ring'), ('pinky', 'Pinky')):
    d = F.FR[f]; O = np.array(d['O']); X = np.array(d['X']); E = np.array(d['E']); U = np.array(d['U'])
    V, Fc = geo.mesh(occ.load(f'/home/claude/cache/v9/{N}_Proximal.brep'), 0.05)
    Vd, Fd = geo.mesh_cached('v7', d['ids']['dist']); Ve, Fe = geo.mesh_cached('v7', d['ids']['enc'])
    c = O + E * 20 + U * 22
    for sgn in (-1, 1):
        fn = f'/home/claude/out/prox_{f}_{sgn}.png'
        rend.render([(V, Fc, (0.45, 0.68, 0.92)), (Vd, Fd, (0.5, 0.8, 0.55)), (Ve, Fe, (0.15, 0.15, 0.15))], fn, pos=c + X * 400 * sgn, focal=c, up=U, size=(900, 700), edges=True, scale=48)
        ims.append(Image.open(fn))
W = Image.new('RGB', (3600, 1400), 'white')
for k, im in enumerate(ims): W.paste(im, ((k // 2) * 900, (k % 2) * 700))
W.resize((2000, 778)).save('/home/claude/out/prox_all.png')
