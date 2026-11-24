"""Free side-to-side (AA) range of each finger with the other fingers neutral: v7 baseline versus v9."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, trimesh, json, math, collections
from trimesh.collision import CollisionManager
import geo, kin9, finger9 as F
FING = kin9.FING
def v7_model():
    meta = geo.meta('v7'); AX = json.load(open('/home/claude/cache/v7/axes.json')); parts = {}
    key = {'Index_Guide_1': ('index', 'aa'), 'MiddleRing_Guide_1': ('middle', 'aa'), 'MiddleRing_Guide_2': ('ring', 'aa'), 'Pinky_Guide_1': ('pinky', 'aa'), 'Index_Proximal_1': ('index', 'mcp'), 'MiddleRing_Proximal_1_1': ('middle', 'mcp'), 'MiddleRing_Proximal_1_2': ('ring', 'mcp'), 'Pinky_Proximal_1': ('pinky', 'mcp'),
           'Index_Distal_1': ('index', 'pip'), 'MiddleRing_Distal_4_1': ('middle', 'pip'), 'MiddleRing_Distal_4_2': ('ring', 'pip'), 'Pinky_Distal_1': ('pinky', 'pip')}
    for d in meta:
        nm = d['path'][1].split(':')[0]; V, Fc = geo.mesh_cached('v7', d['i'])
        if len(Fc) < 4: continue
        g = key.get(nm, ('fixed', '')); parts[f"{d['i']}_{nm}"] = (trimesh.Trimesh(V, Fc, process=False), g)
    return parts
def sweep(parts_groups, pose_T, label):
    """parts_groups: name -> (mesh, (finger, sub)); pose_T(q) -> dict[(finger, sub)] -> 4x4"""
    cm = CollisionManager()
    for n, (m, g) in parts_groups.items(): cm.add_object(n, m)
    res = {}
    for f in FING:
        lim = {}
        for sgn in (1, -1):
            free = 0.0
            for a in np.arange(0, 30.1, 1.0):
                ok = True; who = None
                for mcp in (-5, 0, 30, 68) if label == 'v9' else (0, 30, 68):
                    T = pose_T({f'{f}_aa': sgn * a, f'{f}_mcp': mcp})
                    for n, (m, g) in parts_groups.items(): cm.set_transform(n, T.get(g, np.eye(4)))
                    hit, names = cm.in_collision_internal(return_names=True)
                    for (x, y) in names:
                        gx, gy = parts_groups[x][1], parts_groups[y][1]
                        if gx[0] == gy[0]: continue                                        # same finger / both fixed
                        if f not in (gx[0], gy[0]): continue
                        if {gx[0], gy[0]} == {f, 'fixed'} and label == 'v7' and gx[1] in ('aa', '') and gy[1] in ('aa', ''): continue   # hub on its post
                        other = gy[0] if gx[0] == f else gx[0]; ok = False; who = (other, x if gx[0] != f else y, mcp); break
                    if not ok: break
                if not ok: break
                free = a
            lim[sgn] = (free, who)
        res[f] = lim
        print(f'{label} {f:7s}: free towards thumb {lim[1][0]:4.0f} deg (stops on {lim[1][1]}), towards little finger {lim[-1][0]:4.0f} deg (stops on {lim[-1][1]})')
    return res
# ---- v7
AX = json.load(open('/home/claude/cache/v7/axes.json'))
def v7_T(q):
    T = {}
    for f in FING:
        d = F.FR[f]; O = np.array(d['O']); X = np.array(d['X']); al = math.radians(q.get(f'{f}_aa', 0)); m = math.radians(q.get(f'{f}_mcp', 0))
        Taa = kin9.rot(d['aa_p'], d['aa_v'], al); Tm = Taa @ kin9.rot(O, X, -m); T[(f, 'aa')] = Taa; T[(f, 'mcp')] = Tm; T[(f, 'pip')] = Tm
    return T
print('rotation sign: positive = towards the thumb side')
r7 = sweep(v7_model(), v7_T, 'v7')
# ---- v9
M = kin9.Model()
pg = {}
for n in M.names:
    g = M.group[n]; f = g.split('_')[0]
    pg[n] = (M.mesh[n], (f if f in FING else ('thumb' if f == 'thumb' else 'fixed'), g))
def v9_T(q):
    T = M.transforms(q); return {(g.split('_')[0] if g.split('_')[0] in FING else ('thumb' if g.startswith('thumb') else 'fixed'), g): T[g] for g in T}
# hardware sitting in the palm (AA bearings, sensors) must not count as a collision with its own housing parts
def filt(names): return names
r9 = sweep({n: v for n, v in pg.items() if not (kin9.is_hw(n) and v[1][0] == 'fixed')}, v9_T, 'v9')
