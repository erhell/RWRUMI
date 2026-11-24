import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, trimesh, kin9, json
M = kin9.Model(); M.pose({})
allm = trimesh.util.concatenate([M.mesh[n].copy().apply_transform(M.T[M.group[n]]) for n in M.names if not n.startswith('Thumb')])
palm = M.mesh['Palm']
def cast(mesh, o, d):
    loc, ir, it = mesh.ray.intersects_location([o], [d]); 
    if len(loc) == 0: return None
    t = (loc - o) @ np.array(d); t = t[t > 1e-6]; return t.min() if len(t) else None
ymid = M.fr['middle']['O'][1]
print('hand space (all glove parts except the thumb), z measured in mm; middle knuckle axis at y = %.1f' % ymid)
for back in (10, 25, 40, 55, 70):
    y = ymid - back; rows = []
    for x in (-30, -10, 10, 30):
        dn = cast(allm, np.array([x, y, 42.0]), [0, 0, -1]); up = cast(allm, np.array([x, y, 42.0]), [0, 0, 1])
        rows.append(f'x {x:+d}: floor {42 - dn:5.1f} ceiling {42 + up:5.1f} height {dn + up:5.1f}' if dn and up else f'x {x:+d}: floor {"open" if not dn else round(42 - dn, 1)} ceiling {"open" if not up else round(42 + up, 1)}')
    l = cast(allm, np.array([0.0, y, 40.0]), [-1, 0, 0]); r = cast(allm, np.array([0.0, y, 40.0]), [1, 0, 0])
    print(f'  {back:2d} mm behind the knuckles: ' + ' | '.join(rows) + f' | width at z 40: thumb side {"open" if not l else round(-l, 1)}, little-finger side {"wall at x " + str(round(r, 1)) if r else "open"}')
# forearm capsule vs camera parts
J = json.load(open('/home/claude/cache/v9/hand.json')); 
from trimesh.collision import CollisionManager
cm = CollisionManager()
for n in M.names:
    if n.startswith('Camera'): cm.add_object(n, M.mesh[n])
def caps(a, b, r):
    a = np.array(a, float); b = np.array(b, float); v = b - a; L = np.linalg.norm(v); c = trimesh.creation.capsule(height=L, radius=r, count=[24, 24]); T = trimesh.geometry.align_vectors([0, 0, 1], v / L); T[:3, 3] = (a + b) / 2; c.apply_transform(T); return c
# wrist/forearm: 60 x 42 mm oval approximated by two capsules r 21, centre height = plate top (~24) + 21; from the palm rear edge y 119.5 back to y 0
for dz, lab in ((0, 'forearm straight'), (-25, 'wrist bent 20 deg towards the palm side')):
    d1, n1 = cm.min_distance_single(caps([-9, 119, 45], [-9, 30, 45 + dz * 89 / 70], 21), return_name=True); d2, n2 = cm.min_distance_single(caps([9, 119, 45], [9, 30, 45 + dz * 89 / 70], 21), return_name=True)
    print(f'  {lab}: forearm to camera mount {min(d1, d2):.1f} mm ({n1 if d1 < d2 else n2})')
bb = M.mesh['Camera_Arm'].bounds; print('camera arm box', bb.round(1).tolist()); print('palm box', palm.bounds.round(1).tolist())
