import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, pickle, json, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt, matplotlib.image as mpimg
from matplotlib.patches import Polygon as MP, Circle, Rectangle, Ellipse, FancyBboxPatch
S = pickle.load(open('/home/claude/cache/v7/palm_xz.pkl', 'rb')); AX = json.load(open('/home/claude/cache/v7/axes.json')); BI = json.load(open('/home/claude/cache/base_info.json'))
RAISE = 35.0
C = dict(palm='#c9ccd1', palm_e='#555', hand='#f3d9c4', hand_e='#b98b6a', house='#f5b556', house_e='#a85d00', brg='#d62828', cam='#3a3f47', arm='#f08a24')
fig = plt.figure(figsize=(26, 18.5))
gs = fig.add_gridspec(2, 2, height_ratios=[1, 1.02], width_ratios=[1, 1.05], hspace=0.06, wspace=0.03, left=0.01, right=0.99, top=0.95, bottom=0.035)
# ---------- (a) frame, seen from the wrist
ax = fig.add_subplot(gs[0, 0]); ax.set_aspect('equal'); ax.axis('off')
for P in S[150]:
    if len(P) < 5: continue
    ax.plot(P[:, 0], P[:, 1], color='#999', lw=1.0, ls=':')
    Q = P.copy(); Q[:, 1] = np.where(Q[:, 1] > 40, Q[:, 1] + RAISE, Q[:, 1]); ax.fill(Q[:, 0], Q[:, 1], fc=C['palm'], ec=C['palm_e'], lw=1.0, zorder=2)
ax.add_patch(Ellipse((6, 37.5), 88, 28, angle=-4, fc=C['hand'], ec=C['hand_e'], lw=1, alpha=0.85, zorder=1))
ax.text(6, 37.5, 'hand, 28 mm thick', ha='center', va='center', fontsize=11.5, color='#8a5a3a')
bm = {'index': -0.11, 'middle': -2.78, 'ring': -2.78, 'pinky': 0.0}
for f in ('index', 'middle', 'ring', 'pinky'):
    a = AX[f]; c = np.array(a['mcp_c']); m = np.array(a['mcp_v']); v = np.array(a['aa_v']); p = np.array(a['aa_p'])
    X = lambda q: (q[0], q[2])
    s0, s1 = bm[f] - 7.35, bm[f] + 7.05
    ax.add_patch(MP([X(c + m * s0 + v * 35), X(c + m * s1 + v * 35), X(c + m * s1 + v * 61.5), X(c + m * s0 + v * 61.5)], closed=True, fc=C['house'], ec=C['house_e'], lw=1, alpha=0.9, zorder=4))
    ax.add_patch(MP([X(p + m * -7 + v * 70.5), X(p + m * 7 + v * 70.5), X(p + m * 7 + v * 82), X(p + m * -7 + v * 82)], closed=True, fc=C['palm'], ec=C['palm_e'], lw=1, zorder=5))
    for h in (63.5, 78.0): ax.add_patch(MP([X(p + m * -4 + v * h), X(p + m * 4 + v * h), X(p + m * 4 + v * (h + 3)), X(p + m * -4 + v * (h + 3))], closed=True, fc=C['brg'], ec='#700', lw=0.6, zorder=6))
    ax.add_patch(MP([X(p + m * -5.5 + v * 82), X(p + m * 5.5 + v * 82), X(p + m * 5.5 + v * 84.6), X(p + m * -5.5 + v * 84.6)], closed=True, fc='#222', ec='k', lw=0.5, zorder=6))
    ax.plot(*zip(X(p + v * -6), X(p + v * 92)), color='k', lw=0.7, ls='-.', zorder=7)
    ax.plot(*zip(X(p + v * 48.7), X(p + v * 86.5)), color='#444', lw=2.6, zorder=6.5, solid_capstyle='butt')
    ax.text(*X(p + v * 96), f[0].upper() + f[1:], ha='center', fontsize=11.5)
def dim(ax, x, z0, z1, t, side=1, col='#c00'):
    ax.annotate('', xy=(x, z1), xytext=(x, z0), arrowprops=dict(arrowstyle='<->', color=col, lw=1.4), zorder=12)
    ax.text(x + 1.5 * side, (z0 + z1) / 2, t, color=col, fontsize=12.5, weight='bold', ha='left' if side > 0 else 'right', va='center', zorder=13, bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.8))
dim(ax, -4, 26.0, 61.3, '35 mm', 1); dim(ax, 46, 20.5, 84.5, '62 mm', 1)
def note(ax, txt, xy, xt, ha='left', va='center', col='#222', fs=11.5):
    ax.annotate(txt, xy=xy, xytext=xt, fontsize=fs, color=col, ha=ha, va=va, zorder=20, arrowprops=dict(arrowstyle='-', color=col, lw=0.8, shrinkA=2, shrinkB=1), bbox=dict(fc='white', ec='none', alpha=0.85, pad=1.2))
note(ax, 'Bridge with the four\nside-to-side joints\n(steel pin, 2 bearings,\nsensor on top)', (38, 88), (70, 112))
note(ax, 'Ulnar side wall,\n35 mm taller.\nThe camera arm\nscrews onto its\nouter face', (66.5, 50), (74, 52), va='center')
note(ax, 'Strut on the thumb side,\n35 mm longer; the thumb\nseat below stays in place', (-55, 62), (-106, 84), va='center')
note(ax, 'Arc housings hang\nunder the bridge', (-36, 70), (-106, 60), va='center', col=C['house_e'])
note(ax, 'Lower plate and ORCA palm\nsurface: unchanged', (20, 17), (30, 2), va='center')
note(ax, 'upper plate of v7', (45, 51), (74, 30), col='#777', fs=10.5)
ax.set_xlim(-108, 112); ax.set_ylim(-8, 146)
ax.text(-106, 145, 'Palm frame, seen from the wrist', fontsize=17, weight='bold', va='top')
ax.text(-106, 137, 'section 3 cm behind the knuckles, to scale; dotted = v7', fontsize=11.5, va='top')
# ---------- (b) side view with camera
ax = fig.add_subplot(gs[0, 1]); ax.axis('off')
im = mpimg.imread('/home/claude/out/base_side.png'); b = BI['side']; ps = b['ps']; W, Hh = b['size']; fy, fz = b['focal'][1], b['focal'][2]
ext = (fy - ps * W / Hh, fy + ps * W / Hh, fz - ps, fz + ps)
ax.imshow(im, extent=ext, zorder=0, alpha=0.75); ax.set_aspect('equal')
YE, ZU = 119.5, 18.0; yl, zl = YE - 50, ZU - 50
ax.add_patch(FancyBboxPatch((yl - 23.1, zl - 14.75), 23.1, 29.5, boxstyle='round,pad=0,rounding_size=2.5', fc=C['cam'], ec='k', lw=1, zorder=6))
ax.add_patch(Rectangle((yl - 23.1 - 6.5, zl - 12), 6.5, 24, fc='#70757d', ec='k', lw=0.6, zorder=5, hatch='----'))
arm = np.array([[150, 44], [150, 14], [124, 14], [96, 2], [72, -8], [58, -13], [45, -13], [45, -52], [41, -52], [41, -9], [56, -8], [70, 0], [92, 12], [118, 26], [124, 44]])
ax.add_patch(MP(arm, closed=True, fc=C['arm'], ec='#8a4300', lw=1.2, alpha=0.8, zorder=4, joinstyle='round'))
ax.add_patch(Rectangle((yl - 23.1 - 6.5 - 0.5, zl - 19.5), 32, 4.5, fc=C['arm'], ec='#8a4300', lw=1, alpha=0.9, zorder=7))
for (y, z) in [(130, 20), (144, 20), (130, 38), (144, 38)]: ax.add_patch(Circle((y, z), 1.7, fc='#eee', ec='k', lw=0.8, zorder=8))
half = np.radians(27.5); third = np.arctan(np.tan(half) / 3); Lr = 235
for ang, st, lw in [(half, '-', 1.4), (-half, '-', 1.4), (third, '--', 1.1), (-third, '--', 1.1)]:
    ax.plot([yl, yl + Lr * np.cos(ang)], [zl, zl + Lr * np.sin(ang)], color='#0a7', lw=lw, ls=st, zorder=9)
ax.fill([yl, yl + Lr * np.cos(half), yl + Lr * np.cos(third)], [zl, zl + Lr * np.sin(half), zl + Lr * np.sin(third)], color='#0a7', alpha=0.13, zorder=3)
ax.text(yl + 150, zl + 150 * np.tan((half + third) / 2) + 1, 'upper third of the image', color='#075', fontsize=11.5, rotation=np.degrees((half + third) / 2), rotation_mode='anchor', va='center', zorder=10)
ax.plot([YE, YE], [ZU, -66], color='#c00', lw=0.9, ls=':', zorder=9); ax.plot([yl, YE + 8], [ZU, ZU], color='#c00', lw=0.9, ls=':', zorder=9)
ax.annotate('', xy=(yl, -62), xytext=(YE, -62), arrowprops=dict(arrowstyle='<->', color='#c00', lw=1.4), zorder=12); ax.text((yl + YE) / 2, -66.5, '50 mm', color='#c00', fontsize=12.5, weight='bold', ha='center', va='top')
ax.annotate('', xy=(yl - 2, zl), xytext=(yl - 2, ZU), arrowprops=dict(arrowstyle='<->', color='#c00', lw=1.4), zorder=12); ax.text(yl - 4.5, ZU - 14, '50 mm', color='#c00', fontsize=12.5, weight='bold', ha='right', va='center', bbox=dict(fc='white', ec='none', alpha=0.85, pad=0.8), zorder=13)
note(ax, 'OAK-D Pro, looking straight forward (0° tilt).\nHeld by a clamp around the housing;\nthe cooling fins stay free', (yl - 14, zl - 8), (-48, -66), va='center')
note(ax, 'Arm: closed box section 20 × 14 mm, screwed to the\nouter face of the ulnar side wall with 4 × M3 and nuts.\nNothing touches the palm underside', (96, 6), (-48, 70), va='center')
note(ax, 'wrist-side edge of\nthe palm plate', (YE, -40), (128, -50), va='center', col='#c00', fs=10.5)
ax.set_xlim(-50, 300); ax.set_ylim(-82, 124)
ax.text(-48, 123, 'Camera mount, seen from the little-finger side', fontsize=17, weight='bold', va='top')
ax.text(-48, 112, 'to scale; glove shown as v7', fontsize=11.5, va='top')
# ---------- (c) simulated images
for k, (fn, t) in enumerate([('camview_y70_t0.png', 'Simulated camera image: lens 50 mm behind the edge of the palm plate (as drawn)'), ('camview_y28_t0.png', 'Alternative: lens 92 mm behind the plate edge (50 mm behind the thumb joint block)')]):
    ax = fig.add_subplot(gs[1, k]); ax.axis('off'); ax.imshow(mpimg.imread('/home/claude/out/' + fn)); ax.set_title(t, fontsize=14.5, weight='bold', loc='left', pad=8)
    ax.text(12, 290, 'upper third', color='#c00', fontsize=11, va='bottom')
fig.text(0.012, 0.008, 'Both images: 69° × 55° colour camera of the OAK-D Pro, 0° tilt, 50 mm below the palm underside. Red lines divide the image into thirds. Test objects on a table 14 cm below the palm.', fontsize=11.5)
fig.suptitle('Glove v9 concept, sheet 2: palm frame and camera mount', fontsize=17, y=0.985)
plt.savefig('/home/claude/out/Glove_v9_concept_2_frame_camera.png', dpi=72)
