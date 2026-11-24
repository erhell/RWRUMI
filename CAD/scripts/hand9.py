"""Coarse 'normal hand' check: knuckle spheres + finger capsules + back-of-hand capsules in the glove, clearances over the bending range."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, math, trimesh, kin9, geo, finger9 as F, occ, json
from trimesh.collision import CollisionManager
M = kin9.Model(); FING = kin9.FING; CAP = kin9.CAP
R_F = dict(index=9.5, middle=9.5, ring=9.0, pinky=8.0); R_K = dict(index=14.0, middle=14.0, ring=13.5, pinky=12.0)     # finger radius, half hand thickness at the knuckle
def seg_capsule(a, b, r):
    v = b - a; L = np.linalg.norm(v); c = trimesh.creation.capsule(height=L, radius=r, count=[24, 24]); T = trimesh.geometry.align_vectors([0, 0, 1], v / L); T[:3, 3] = (a + b) / 2; c.apply_transform(T); return c
res = {}; hand = {}
for f in FING:
    d = M.fr[f]; O, X, E, U = d['O'], d['X'], d['E'], d['U']; T = d['T']
    V, _ = geo.mesh_cached('v7', {'index': 31, 'middle': 33, 'ring': 51, 'pinky': 35}[f]); L = (V - O); x = L @ X; e = L @ E; u = L @ U
    sel = (e > 12) & (e < 26) & (x < -T / 2 - 3) & (u < 8); ut = u[sel].max(); xmin = x[(e > 12) & (e < 26) & (u < 8)].min()
    xc = -T / 2 - R_F[f] - 0.5; J = O + X * xc + U * (ut + R_K[f] - 0.5)                    # human knuckle joint centre
    hand[f] = dict(ut=ut, xc=xc, J=J)
    print(f'{f:6s}: finger tray top {ut:+.1f} mm from the knuckle axis, tray reaches {xmin:.1f} mm to the thumb side of the web; human joint centre {ut + R_K[f] - 0.5:+.1f} mm above the glove axis')
def hand_meshes(f, q):
    d = M.fr[f]; O, X, E, U = d['O'], d['X'], d['E'], d['U']; h = hand[f]; J = h['J']; c = math.cos(math.radians(q)); s = math.sin(math.radians(q))
    ed = E * c - U * s; a0 = J - U * (R_K[f] - R_F[f]) * 1.0                              # finger axis starts lower than the joint (palmar skin continuous)
    a0 = J + (ed * 0 - (U * c + E * s) * (R_K[f] - R_F[f]))
    fin = seg_capsule(a0 + ed * 8, a0 + ed * (F.FR[f]['pip_r'] if False else 40.0), R_F[f])
    kn = trimesh.creation.icosphere(subdivisions=3, radius=R_K[f]); kn.apply_translation(J)
    back = seg_capsule(J, J - E * 70 + U * 0.0, R_K[f])
    return dict(finger=fin, knuckle=kn, back=back)
def run(f, q, aa=0.0):
    M.pose({f'{f}_mcp': q, f'{f}_aa': aa}); hm = hand_meshes(f, q); out = {}
    Taa = M.T[f'{f}_aa']
    for k in hm: hm[k].apply_transform(Taa)
    N = CAP[f]
    sets = {'housing (cheeks, gear, sensor, pin block)': [n for n in M.names if n.startswith(N + '_') and M.group[n] in (f'{f}_aa', f'{f}_gear') ],
            'bridge / palm frame': ['Palm'], 'electronics board': ['PCB_82x50_dummy']}
    for lab, names in sets.items():
        cm = CollisionManager()
        for n in names: cm.add_object(n, M.mesh[n], M.T[M.group[n]])
        for k, m in hm.items():
            if lab != 'housing (cheeks, gear, sensor, pin block)' and k == 'back' : pass
            dd = cm.min_distance_single(m); out[(lab, k)] = dd
    # arc (moving with the finger piece): take the proximal mesh above U = 30 only
    return out
import collections
best = collections.defaultdict(lambda: (1e9, None))
for f in FING:
    for q in (-5, 0, 15, 30, 45, 60, 68):
        o = run(f, q)
        for k, v in o.items():
            if k[0] == 'bridge / palm frame' and k[1] in ('back', 'knuckle', 'finger'): 
                pass
            if v < best[(f,) + k][0]: best[(f,) + k] = (v, q)
print('smallest clearance over the bending range -5..68 deg (mm, at angle):')
for f in FING:
    print(f'  {f:6s}: ' + '; '.join(f'{k[1]} to {k[0].split(" (")[0]} {best[(f,) + k][0]:.1f} (at {best[(f,) + k][1]})' for k in [('housing (cheeks, gear, sensor, pin block)', 'knuckle'), ('housing (cheeks, gear, sensor, pin block)', 'finger'), ('housing (cheeks, gear, sensor, pin block)', 'back')]))
json.dump({f: dict(ut=float(h['ut']), xc=float(h['xc']), J=h['J'].tolist()) for f, h in hand.items()}, open('/home/claude/cache/v9/hand.json', 'w'))
print('finger-to-housing clearance by bending angle, nearest part:')
for f in FING:
    row = []
    for q in (-5, 0, 10, 20, 40, 68):
        M.pose({f'{f}_mcp': q}); hm = hand_meshes(f, q); N = CAP[f]; cm = CollisionManager()
        for n in M.names:
            if n.startswith(N + '_') and M.group[n] in (f'{f}_aa', f'{f}_gear'): cm.add_object(n, M.mesh[n], M.T[M.group[n]])
        dd, nm = cm.min_distance_single(hm['finger'], return_name=True); row.append(f'{q}: {dd:.1f} ({nm.split("_", 1)[1]})')
    print(f'  {f:6s} ' + ', '.join(row))
# where is the lowest front point of the housing in the finger frame
for f in ('middle',):
    d = M.fr[f]; N = CAP[f]
    for part in ('MCP_CheekRadial', 'MCP_CheekUlnar', 'MCP_Bearing_4x8x3_H', 'MCP_Gear', '3382H_MCP'):
        V = M.mesh[f'{N}_{part}'].vertices - d['O']; e = V @ d['E']; u = V @ d['U']; x = V @ d['X']; k = np.argmin(u + 0 * e); s = e > 25
        print(f'   {part}: E {e.min():.1f}..{e.max():.1f}, U {u.min():.1f}..{u.max():.1f}, X {x.min():.1f}..{x.max():.1f}; lowest point in front of E 25: U {u[s].min() if s.any() else float("nan"):.1f}')
