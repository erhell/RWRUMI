import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, math, kin9, json, trimesh, geo
from trimesh.collision import CollisionManager
import finger9 as F
M = kin9.Model(); FING = kin9.FING
def where(q, a, b, fin):
    M.pose(q); hit, names, data = M.cm.in_collision_internal(return_names=True, return_data=True)
    pts = [c.point for c in data if {a, b} == set(c.names)]
    if not pts: print('  no contact', a, b, q); return
    pts = np.array(pts); d = M.fr[fin]; Tm = M.T[f'{fin}_mcp']; loc = (np.linalg.inv(Tm) @ np.c_[pts, np.ones(len(pts))].T).T[:, :3] - d['O']
    x = loc @ d['X']; e = loc @ d['E']; u = loc @ d['U']; r = np.hypot(e, u); ang = np.degrees(np.arctan2(u, e))
    print(f'  {a} x {b} at {q}: {len(pts)} contact pts; in the {fin} proximal frame: lateral x {x.min():.2f}..{x.max():.2f}, radius {r.min():.1f}..{r.max():.1f}, angle {ang.min():.0f}..{ang.max():.0f} deg')
    # depth via min signed approach: move the pair apart along X until free
    ma = CollisionManager(); ma.add_object('a', M.mesh[a], M.T[M.group[a]])
    for s in np.arange(0.05, 3.01, 0.05):
        for sg in (1, -1):
            T = np.eye(4); T[:3, 3] = sg * s * d['X']; 
            if not ma.in_collision_single(M.mesh[b], T @ M.T[M.group[b]]): print(f'     free after moving {b} by {sg*s:+.2f} mm along the axis'); return
all68 = {f'{f}_mcp': 68 for f in FING}
where(all68, 'Index_3382H_PIP', 'Middle_Proximal', 'middle')
where({'ring_mcp': -5}, 'Middle_3382H_PIP', 'Ring_Distal', 'ring')
print('layers middle (xL, xR ...):', F.layers('middle'))
# the same poses in v7
meta = geo.meta('v7'); cm = CollisionManager(); grp = {}
key = {31: ('index', 'mcp'), 32: ('index', 'mcp'), 38: ('index', 'pip'), 33: ('middle', 'mcp'), 34: ('middle', 'mcp'), 39: ('middle', 'pip'), 51: ('ring', 'mcp'), 52: ('ring', 'mcp'), 53: ('ring', 'pip'), 35: ('pinky', 'mcp'), 36: ('pinky', 'mcp'), 37: ('pinky', 'pip')}
for i, g in key.items():
    V, Fc = geo.mesh_cached('v7', i); cm.add_object(str(i), trimesh.Trimesh(V, Fc, process=False)); grp[str(i)] = g
def v7pose(q):
    for n, (f, s) in grp.items():
        d = M.fr[f]; Tm = kin9.rot(d['aa_p'], d['aa_v'], math.radians(q.get(f'{f}_aa', 0))) @ kin9.rot(d['O'], d['X'], -math.radians(q.get(f'{f}_mcp', 0)))
        if s == 'pip': Tm = Tm @ kin9.rot(M.pip[f], d['X'], -math.radians(q.get(f'{f}_pip', 0)))
        cm.set_transform(n, Tm)
    hit, names = cm.in_collision_internal(return_names=True); return sorted(tuple(sorted(x)) for x in names if grp[x[0]][0] != grp[x[1]][0])
print('v7, all four at 68:', v7pose(all68)); print('v7, all four at 60:', v7pose({f'{f}_mcp': 60 for f in FING})); print('v7, ring alone at -5:', v7pose({'ring_mcp': -5}))
for m in (50, 55, 60, 64, 68):
    M.pose({f'{f}_mcp': m for f in FING}); print('v9 all at', m, M.collisions())
