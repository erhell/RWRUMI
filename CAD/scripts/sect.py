import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, trimesh, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
M = geo.meta('v7')
pal = {'Carpal': '#888', 'Guide': '#e8891c', 'Proximal': '#2a7fd6', 'Distal': '#3aa655', 'T-': '#b05ab0', 'R-T': '#8a3a8a', 'Encoder': '#111', 'Bearing': '#d22'}
def section(origin, normal, ax, a0, a1):
    for d in M:
        V, F = geo.mesh_cached('v7', d['i'])
        if len(F) < 4: continue
        m = trimesh.Trimesh(V, F, process=False)
        try: s = m.section(plane_origin=origin, plane_normal=normal)
        except Exception as e: s = None
        if s is None: continue
        col = next((c for k, c in pal.items() if k in d['path'][1]), '#666')
        for ent in s.entities:
            P = s.vertices[ent.points]
            ax.plot(P[:, a0], P[:, a1], color=col, lw=0.8)
fig, axs = plt.subplots(2, 2, figsize=(22, 16))
for ax, (y, t) in zip(axs.ravel(), [(175, 'y=175 (AA axes)'), (192, 'y=192 (MCP axis middle)'), (212, 'y=212 (mid proximal)'), (150, 'y=150 (palm)')]):
    section([0, y, 0], [0, 1, 0], ax, 0, 2); ax.set_aspect('equal'); ax.set_title(t); ax.grid(alpha=0.3); ax.set_xlim(-80, 75); ax.set_ylim(-15, 95); ax.invert_xaxis()
    ax.set_xticks(range(-80, 76, 10)); ax.set_yticks(range(-10, 96, 10))
plt.tight_layout(); plt.savefig('/home/claude/out/v7_sections_xz.png', dpi=70)
fig, axs = plt.subplots(2, 2, figsize=(24, 16))
for ax, (x, t) in zip(axs.ravel(), [(-9.7, 'x=-9.7 (middle bow plane)'), (-15.35, 'x=-15.35 (middle AA axis)'), (-18, 'x=-18 (middle pad radial part)'), (30, 'x=30 (pinky/ulnar)')]):
    section([x, 0, 0], [1, 0, 0], ax, 1, 2); ax.set_aspect('equal'); ax.set_title(t); ax.grid(alpha=0.3); ax.set_xlim(70, 285); ax.set_ylim(-15, 100)
    ax.set_xticks(range(70, 286, 10)); ax.set_yticks(range(-10, 101, 10))
plt.tight_layout(); plt.savefig('/home/claude/out/v7_sections_yz.png', dpi=70)
