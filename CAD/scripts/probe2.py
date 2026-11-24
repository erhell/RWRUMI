import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, json, trimesh, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
ax = json.load(open('/home/claude/cache/v7/axes.json'))
F = {'index': (31, -1.615, 3.281, 32, 38), 'middle': (33, -4.279, 0.506, 34, 39), 'ring': (51, -4.279, 0.506, 52, 53), 'pinky': (35, -1.5, 3.4, 36, 37)}
fig, axs = plt.subplots(2, 4, figsize=(28, 14))
for k, (f, (i, xr, xu, ienc, idist)) in enumerate(F.items()):
    a = ax[f]; c0 = np.array(a['mcp_c']); m = np.array(a['mcp_v']); pip = np.array(a['pip_c']); e = pip - c0; e -= m * (e @ m); e /= np.linalg.norm(e); u = np.cross(m, e)
    for row, (xs, lab) in enumerate([(xr + 1.5, 'bow slab mid'), (xu - 0.6, 'web only (ulnar part)')]):
        A = axs[row, k]
        for j, col in ((i, '#1f5fa8'), (ienc, '#111'), (idist, '#3a9a55')):
            V, Fc = geo.mesh_cached('v7', j); msh = trimesh.Trimesh(V, Fc, process=False)
            s = msh.section(plane_origin=c0 + m * xs, plane_normal=m)
            if s is None: continue
            for ent in s.entities:
                P = s.vertices[ent.points] - c0; A.plot(P @ e, P @ u, color=col, lw=1)
        t = np.linspace(0, 2 * np.pi, 200)
        for r in (37.9, 39.5, 56.1): A.plot(r * np.cos(t), r * np.sin(t), color='#e80', lw=0.5, ls=':')
        for ang in (14, 24, 97, 115): A.plot([0, 65 * np.cos(np.radians(ang))], [0, 65 * np.sin(np.radians(ang))], color='#aaa', lw=0.5)
        A.set_aspect('equal'); A.set_xlim(-35, 70); A.set_ylim(-20, 62); A.grid(alpha=0.3); A.set_title(f'{f}: {lab} (lateral {xs:.2f})')
plt.tight_layout(); plt.savefig('/home/claude/out/probe_prox_sections.png', dpi=60)
