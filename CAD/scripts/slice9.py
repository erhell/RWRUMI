import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, kin9, trimesh, matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt, asm
M = kin9.Model(); M.pose({})
d = M.fr['middle']; O = d['O']; E = d['E']
fig, axs = plt.subplots(1, 3, figsize=(27, 9))
for ax, (ttl, off) in zip(axs, (('20 mm behind the knuckle axis', -20.0), ('at the knuckle axis', 0.0), ('18 mm in front (mid finger segment)', 18.0))):
    org = O + E * off
    for n in M.names:
        m = M.mesh[n].copy(); m.apply_transform(M.T[M.group[n]])
        try: s = m.section(plane_origin=org, plane_normal=E)
        except Exception: s = None
        if s is None: continue
        col = 'tab:blue' if n == 'Palm' else ('tab:red' if 'Cheek' in n or 'Gear' in n else ('tab:green' if 'Proximal' in n else ('k' if 'Camera' in n else 'gray')))
        for ent in s.entities:
            P = s.vertices[ent.points]; ax.plot(P[:, 0], P[:, 2], color=col, lw=0.8)
    for f in kin9.FING:
        o = M.fr[f]['O']; ax.plot(o[0], o[2], 'm+', ms=12)
    ax.set_aspect('equal'); ax.set_title(ttl + '  (x right = little finger, z up; + = knuckle axis height)'); ax.grid(alpha=.3); ax.set_xlim(-75, 85); ax.set_ylim(-25, 125)
plt.tight_layout(); plt.savefig('/home/claude/out/slice9.png', dpi=70)
for f in kin9.FING: print(f, 'O', M.fr[f]['O'].round(1), 'X', M.fr[f]['X'].round(3), 'E', M.fr[f]['E'].round(3))
