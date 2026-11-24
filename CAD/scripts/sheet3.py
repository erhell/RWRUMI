import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, json, geo, rend, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt, matplotlib.image as mpimg
from matplotlib.patches import Polygon as MP, Circle, FancyBboxPatch, Rectangle
from matplotlib.transforms import Affine2D
from scipy.optimize import brentq
BI = json.load(open('/home/claude/cache/base_info.json'))['thumb_b']
view = np.array(BI['view']); up = np.array(BI['up']); d = np.array(BI['d']); Mc = np.array(BI['M']); Ic = np.array(BI['I'])
L0 = float((Ic - Mc) @ d); ps = 46.0; size = (1500, 1150)
Mm = geo.meta('v7')
def rodr(P, c, ang):
    n = view; Q = P - c; ca, sa = np.cos(ang), np.sin(ang)
    return c + Q * ca + np.cross(n, Q) * sa + np.outer(Q @ n, n) * (1 - ca)
def render(thM, thI, fn):
    items = []
    for m in Mm:
        nm = m['path'][1]
        if not (nm.startswith('T-') or nm.startswith('R-T') or 'Encoder_3382H_T' in nm or 'Bearing' in nm): continue
        V, F = geo.mesh_cached('v7', m['i']); i = m['i']
        col = (0.86, 0.72, 0.88) if nm.startswith('T-') or nm.startswith('R-T') else ((0.15, 0.15, 0.15) if 'Enc' in nm else (0.8, 0.2, 0.2))
        if i in (57, 62, 69): V = rodr(V, Mc, -thM)
        if i == 58:
            V = rodr(V, Mc, -thM); G = rodr(Ic[None], Mc, -thM)[0]; V = rodr(V, G, -thI); col = (0.78, 0.58, 0.82)
        if i not in (56, 57, 58, 61, 62, 68, 69): continue
        items.append((V, F, col))
    f = Mc + d * 14 - up * 10
    rend.render(items, fn, pos=f + view * 500, focal=f, up=up, size=size, edges=True, scale=ps)
    uf, wf = (f - Mc) @ d, (f - Mc) @ up
    return (uf - ps * size[0] / size[1], uf + ps * size[0] / size[1], wf - ps, wf + ps)
A = np.array([3.0, 5.0]); b0 = np.array([-2.0, -7.0]); Lb = np.linalg.norm(np.array([L0, 0]) + b0 - A)
def R(t): c, s = np.cos(t), np.sin(t); return np.array([[c, -s], [s, c]])
def ip_of(thM):
    G = R(-thM) @ np.array([L0, 0]); f = lambda x: np.linalg.norm(G + R(-(thM + x)) @ b0 - A) - Lb
    return brentq(f, np.radians(-5), np.radians(90)) if thM > 1e-6 else 0.0
thM = np.radians(50); thI = ip_of(thM); print('MCP 50 -> IP', np.degrees(thI), 'bar length', Lb)
e0 = render(0, 0, '/home/claude/out/thumb_straight.png'); e1 = render(thM, thI, '/home/claude/out/thumb_bent.png')
C = dict(hand='#f3d9c4', hand_e='#b98b6a', part='#f08a24', part_e='#8a4300')
def thumb(ax, th):
    t = Affine2D().rotate(-th) + ax.transData
    ax.add_patch(FancyBboxPatch((-44, 8.0), 45, 21, boxstyle='round,pad=0,rounding_size=6', fc=C['hand'], ec=C['hand_e'], lw=1, alpha=0.8, zorder=3))
    ax.add_patch(FancyBboxPatch((0.5, 7.9), L0 - 0.5, 18.5, boxstyle='round,pad=0,rounding_size=8', fc=C['hand'], ec=C['hand_e'], lw=1, alpha=0.85, zorder=4, transform=t))
    ax.add_patch(Circle((0, 18), 2.2, fc='none', ec='#8a5a3a', lw=1.2, zorder=5))
def bar(ax, th, ti):
    G = R(-th) @ np.array([L0, 0]); B = G + R(-(th + ti)) @ b0; v = (B - A) / np.linalg.norm(B - A); n = np.array([-v[1], v[0]]) * 2.6
    ax.add_patch(MP([A + n - v * 3, B + n + v * 3, B - n + v * 3, A - n - v * 3], closed=True, fc=C['part'], ec=C['part_e'], lw=1.2, alpha=0.92, zorder=6, joinstyle='round'))
    for p in (A, B): ax.add_patch(Circle(p, 1.1, fc='#333', ec='k', lw=0.5, zorder=7))
    return G, B
def note(ax, txt, xy, xt, ha='left', va='center', col='#222', fs=11.5):
    ax.annotate(txt, xy=xy, xytext=xt, fontsize=fs, color=col, ha=ha, va=va, zorder=20, arrowprops=dict(arrowstyle='-', color=col, lw=0.8, shrinkA=2, shrinkB=1), bbox=dict(fc='white', ec='none', alpha=0.85, pad=1.2))
fig = plt.figure(figsize=(26, 10.8)); gs = fig.add_gridspec(1, 3, wspace=0.02, left=0.01, right=0.99, top=0.9, bottom=0.15)
lim = dict(x=(-47, 73), y=(-50, 40))
ax = fig.add_subplot(gs[0, 0]); ax.imshow(mpimg.imread('/home/claude/out/thumb_straight.png'), extent=e0, zorder=0); ax.set_aspect('equal'); ax.axis('off')
thumb(ax, 0); G, B = bar(ax, 0, 0); ax.set_xlim(*lim['x']); ax.set_ylim(*lim['y'])
ax.set_title('Option 1 (my suggestion): coupling link, thumb straight', fontsize=15, weight='bold', loc='left')
note(ax, "your thumb's last segment lies on the\nsecond-to-last glove piece (T-PP)", (18, 20), (-46, 37), va='top')
note(ax, 'your thumb tip ends here', (L0, 16), (40, 34))
note(ax, 'last glove piece (T-DP),\n34 mm beyond your thumb tip', (52, 2), (42, 20))
note(ax, 'Link 30 mm long, 5 × 5 mm, on two 2×10 pins: from the\nbase piece (above its joint) to the last piece (below its\njoint). Drawn in front here; it sits on the far side,\naway from the sensors', tuple((A + B) / 2), (-2, -34), va='center', col=C['part_e'])
note(ax, 'joint moved by the last joint\nof your thumb (sensor unchanged)', (-3, -3), (-46, -16), va='center')
note(ax, 'last glove joint\n(sensor unchanged)', (L0 + 3, -3), (44, -18), va='center')
ax = fig.add_subplot(gs[0, 1]); ax.imshow(mpimg.imread('/home/claude/out/thumb_bent.png'), extent=e1, zorder=0); ax.set_aspect('equal'); ax.axis('off')
thumb(ax, thM); G, B = bar(ax, thM, thI); ax.set_xlim(*lim['x']); ax.set_ylim(*lim['y'])
ax.set_title('Option 1: thumb bent', fontsize=15, weight='bold', loc='left')
note(ax, f'you bend your last thumb joint by {np.degrees(thM):.0f}°:\nT-PP follows, and the link bends the\nlast piece by another {np.degrees(thI):.0f}° (ratio 0.8)', tuple(G + R(-(thM + thI)) @ np.array([16, 0])), (24, 22), va='center')
ax = fig.add_subplot(gs[0, 2]); ax.imshow(mpimg.imread('/home/claude/out/thumb_straight.png'), extent=e0, zorder=0); ax.set_aspect('equal'); ax.axis('off')
thumb(ax, 0); ax.set_xlim(*lim['x']); ax.set_ylim(*lim['y'])
sad = np.array([[6, 6.9], [27, 6.9], [29, 9], [29, 15.5], [36, 15.5], [39, 8.2], [58, 8.2], [58, 12.2], [41.5, 12.2], [38.5, 19.5], [29, 19.5], [29, 21], [26, 21], [26, 9.6], [9, 9.6], [9, 21], [6, 21]])
ax.add_patch(MP(sad, closed=True, fc=C['part'], ec=C['part_e'], lw=1.2, alpha=0.9, zorder=6, joinstyle='round'))
for u in (46, 54): ax.add_patch(Circle((u, 10.2), 1.5, fc='#eee', ec='k', lw=0.8, zorder=7))
ax.plot([9, 26], [27.2, 27.2], color='#555', lw=4, alpha=0.7, zorder=7, solid_capstyle='round')
ax.set_title('Option 2: bracket from your thumb tip to the last piece', fontsize=15, weight='bold', loc='left')
note(ax, 'cradle with strap for your last thumb segment,\nscrewed to the last glove piece (2 × M3×8)', (33, 17.5), (-10, 37), va='top', col=C['part_e'])
note(ax, 'Your last thumb segment and T-PP swing about\n(almost) the same axis and are equally long.\nThe bracket therefore just holds the last piece\nin line with T-PP: the last ORCA thumb joint\nwould stay straight all the time.', (16, -2), (-10, -30), va='center')
fig.text(0.012, 0.07, "Why a choice is needed: your thumb tip ends at the last glove joint, so none of your thumb joints is left over to move the last glove piece on its own.\nOption 1 lets it curl along with the piece before it (fixed ratio, here 0.8; any ratio from about 0.5 to 1 is possible). Option 2 keeps it straight. In both, the three other thumb joints and all axes stay as in v7;\nonly the base piece (R-T-AP) and the last piece (T-DP) get a small boss or screw holes on the side away from the sensors.", fontsize=12.5, va='center', linespacing=1.5)
fig.suptitle('Glove v9 concept, sheet 3: thumb, last joint (view along the joint axes, to scale)', fontsize=17, y=0.975)
plt.savefig('/home/claude/out/Glove_v9_concept_3_thumb.png', dpi=72)
