import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, geo, trimesh, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Wedge, Circle, FancyBboxPatch, Polygon
M = geo.meta('v7')
MCP = np.array([191.966, 26.345])
def sec(ids, x):
    out = []
    for i in ids:
        V, F = geo.mesh_cached('v7', i); m = trimesh.Trimesh(V, F, process=False)
        s = m.section(plane_origin=[x, 0, 0], plane_normal=[1, 0, 0])
        if s is None: continue
        for ent in s.entities: out.append(s.vertices[ent.points][:, 1:] - MCP)
    return out
palm = sec([0], -9.7); prox = sec([33], -15.0); dist = sec([39], -15.0); web = sec([33], -9.7)
def band(ax, r0, r1, a0, a1, **k): ax.add_patch(Wedge((0, 0), r1, a0, a1, width=r1 - r0, **k))
opts = [('A  smallest possible', 41, 60, 32), ('B  about the size of v7', 47, 73, 35), ('C  larger, most hand room', 52, 80, 38)]
fig, axs = plt.subplots(1, 3, figsize=(24, 8.2))
for ax, (name, rs, rom, zt) in zip(axs, opts):
    for P in palm:
        lower = P[:, 1].max() < 10
        Q = P.copy()
        if lower:
            ax.fill(Q[:, 0], Q[:, 1], color='#c9ccd1', ec='#666', lw=0.8, zorder=2)
        else:
            ax.plot(P[:, 0], P[:, 1], color='#999', lw=1.0, ls=':', zorder=2)
    for P in prox + dist: ax.fill(P[:, 0], P[:, 1], color='#9cc7ee', ec='#2a6db0', lw=0.8, zorder=2)
    # hand: 28 mm thick over plate, finger 19 mm
    hand = np.array([[-95, -1.5], [-12, -2.5], [8, -5.5], [78, -5.5], [86, 2], [84, 11], [45, 13.5], [12, 16], [-2, 26.5], [-12, 27], [-40, 26], [-95, 27]])
    ax.add_patch(Polygon(hand, closed=True, fc='#f3d9c4', ec='#b98b6a', lw=1, alpha=0.75, zorder=1))
    ax.add_patch(Circle((-5, 13), 2.2, fc='none', ec='#8a5a3a', lw=1.2, zorder=5)); ax.plot(0, 0, '+', color='k', ms=14, mew=1.6, zorder=6)
    # ART reference arc about the human joint
    ax.add_patch(Wedge((-5, 13), 26, 34, 141, width=5, fc='none', ec='#7a3fb5', lw=1.2, ls='--', zorder=4))
    ri, ro = rs - 9.1, rs + 9.1
    zb = ro + 1.5
    br = np.array([[-50, 47.5], [-40, 47.5], [-33, zb], [-8, zb], [-8, zb + 9], [-36, zb + 9], [-44, 56.5], [-50, 56.5]])
    ax.add_patch(Polygon(br, closed=True, fc='#c9ccd1', ec='#666', lw=0.8, zorder=2))
    ax.add_patch(Polygon(np.array([[-95, 47.5], [-50, 47.5], [-50, 56.5], [-95, 56.5]]), closed=True, fc='#c9ccd1', ec='#666', lw=0.8, zorder=2))
    band(ax, 41, 50.75, -12, 117, fc='none', ec='#3b7', lw=1.3, ls=(0, (4, 3)), zorder=4)
    phiH = 29; delta = np.degrees(9.6 / rs); T = phiH + delta + rom; qmin = -5 if rom < 70 else -8
    e = np.degrees(8.1 / rs)
    band(ax, ri, ro, phiH - np.degrees(4.5 / rs) , T + e, fc='#f2a23a', ec='#a85d00', lw=1.0, alpha=0.55, zorder=3)
    band(ax, rs - 4.1, rs + 4.1, phiH + delta - np.degrees(4.1/rs), T + np.degrees(4.1/rs), fc='white', ec='#a85d00', lw=0.8, zorder=3)
    for a in (phiH, T):
        c = rs * np.array([np.cos(np.radians(a)), np.sin(np.radians(a))]); ax.add_patch(Circle(c, 4, fc='#d22', ec='#700', lw=0.8, zorder=6)); ax.add_patch(Circle(c, 2, fc='white', ec='#700', lw=0.6, zorder=7))
    zt_real = rs * np.sin(np.radians(T)) - 8.1
    ax.annotate('', xy=(rs*np.cos(np.radians(T)) - 2, zt_real), xytext=(rs*np.cos(np.radians(T)) - 2, -1.5), arrowprops=dict(arrowstyle='<->', color='#c00', lw=1.4), zorder=8)
    ax.text(rs*np.cos(np.radians(T)) - 4, 8, f'{zt_real:.0f} mm', color='#c00', ha='right', fontsize=14, weight='bold', bbox=dict(fc='white', ec='none', alpha=0.8, pad=1.5), zorder=9)
    ax.annotate('', xy=(0, ri), xytext=(0, 0), arrowprops=dict(arrowstyle='<->', color='#06c', lw=1.4), zorder=8)
    ax.text(3, 17.5, f'inner radius\n{ri:.0f} mm', color='#06c', fontsize=13, weight='bold', bbox=dict(fc='white', ec='none', alpha=0.8, pad=1.5), zorder=9)
    ax.text(-97, 121, f'{name}', fontsize=19, weight='bold', va='top')
    ax.text(-97, 111, f'arc {ri:.0f}–{ro:.0f} mm from the ORCA knuckle axis\nfinger bend range about {qmin}° to {rom+qmin}°  ({rom}° total)\nlowest point behind the knuckle: {zt_real:.0f} mm above the palm plate', fontsize=14, va='top', linespacing=1.35)
    ax.set_aspect('equal'); ax.set_xlim(-100, 92); ax.set_ylim(-26, 122); ax.axis('off')
axs[0].annotate('ART glove arc (r ≈ 24 mm),\ncentred on the human joint', xy=(-22, 31), xytext=(-97, 33), color='#7a3fb5', fontsize=11, va='center', arrowprops=dict(arrowstyle='-', color='#7a3fb5', lw=0.8))
axs[0].annotate('human knuckle joint', xy=(-5, 13), xytext=(-70, 12), fontsize=11, va='center', color='#8a5a3a', arrowprops=dict(arrowstyle='-', color='#8a5a3a', lw=0.8))
axs[0].annotate('ORCA knuckle axis\n(centre of the bow)', xy=(0, 0), xytext=(-52, -17), fontsize=11, va='center', arrowprops=dict(arrowstyle='-', color='k', lw=0.8))
axs[0].text(-97, 60, 'upper plate, +20 mm', fontsize=11, color='#444'); axs[0].annotate('upper plate today', xy=(-45, 29.5), xytext=(-97, 42), fontsize=10.5, color='#777', arrowprops=dict(arrowstyle='-', color='#999', lw=0.8))
axs[0].annotate('v7 bow', xy=(41*np.cos(np.radians(20)), 41*np.sin(np.radians(20))), xytext=(52, 2), fontsize=11, color='#2a8a55', arrowprops=dict(arrowstyle='-', color='#3b7', lw=0.8))
axs[0].annotate('raised bridge for the\nside-to-side joint', xy=(-12, 58), xytext=(10, 74), fontsize=11, color='#444', arrowprops=dict(arrowstyle='-', color='#666', lw=0.8))
axs[0].text(30, -17, 'red: ball bearings 4×8×3', fontsize=11, color='#a00')
fig.suptitle('Middle finger, side view, to scale. Skin-coloured: average hand (28 mm thick) lying on the lower plate. Orange: new arc at full extension; it sits beside the finger, between two fingers, as in v7.', fontsize=14.5, y=0.035)
plt.tight_layout(rect=(0, 0.05, 1, 1)); plt.savefig('/home/claude/out/Glove_v9_bow_radius_options.png', dpi=85, bbox_inches='tight', pad_inches=0.25)
for (name, rs, rom, zt) in opts:
    d = np.degrees(9.6 / rs); T = 29 + d + rom; print(name, rs, 'T', round(T, 1), 'Ztail', round(rs*np.sin(np.radians(T)) - 8.1, 1))
