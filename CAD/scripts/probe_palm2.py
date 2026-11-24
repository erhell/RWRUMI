import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, trimesh, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
V, Fc = geo.mesh_cached('v7', 0); m = trimesh.Trimesh(V, Fc, process=False)
fig, axs = plt.subplots(2, 4, figsize=(30, 15))
for ax, z in zip(axs[0], (30, 38, 44, 50)):
    s = m.section(plane_origin=[0, 0, z], plane_normal=[0, 0, 1])
    for e in s.entities: P = s.vertices[e.points]; ax.plot(P[:, 0], P[:, 1], lw=1)
    ax.set_aspect('equal'); ax.set_title(f'z = {z}'); ax.grid(alpha=0.3); ax.set_xlim(-80, 75); ax.set_ylim(70, 195); ax.invert_xaxis()
for ax, y in zip(axs[1], (125, 138, 150, 160)):
    s = m.section(plane_origin=[0, y, 0], plane_normal=[0, 1, 0])
    for e in s.entities: P = s.vertices[e.points]; ax.plot(P[:, 0], P[:, 2], lw=1)
    ax.set_aspect('equal'); ax.set_title(f'y = {y}'); ax.grid(alpha=0.3); ax.set_xlim(-80, 75); ax.set_ylim(-15, 95); ax.invert_xaxis()
    ax.set_yticks(range(-10, 96, 5))
plt.tight_layout(); plt.savefig('/home/claude/out/probe_palm_sections.png', dpi=55)
