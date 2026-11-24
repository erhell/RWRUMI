import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, pickle, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as MP, Circle, Rectangle, FancyArrowPatch
from shapely.geometry import Polygon, Point, box
from shapely.ops import unary_union
from shapely import affinity
import kn
D = pickle.load(open('/home/claude/cache/v7/mid_sections.pkl', 'rb'))
C = dict(palm='#c9ccd1', palm_e='#666', hand='#f3d9c4', hand_e='#b98b6a', link='#7fb2e8', link_e='#1f5fa8', house='#f5b556', house_e='#a85d00', brg='#d62828', steel='#444', gear='#b7c94a', gear_e='#6b7a12', pot='#222', glove='#b9d6f2')
def fill(ax, g, **k):
    gs = [g] if g.geom_type == 'Polygon' else list(g.geoms)
    for p in gs:
        if p.is_empty: continue
        ax.fill(*p.exterior.xy, **k)
        for r in p.interiors: ax.fill(*r.xy, fc='white', ec=k.get('ec', 'k'), lw=k.get('lw', 1), zorder=k.get('zorder', 1))
def P(a): return Polygon(a).buffer(0)
L = kn.link_parts(); H = kn.housing_parts(); B = kn.bridge_parts()
# v7 proximal web without the old bow, PIP region kept
web_v7 = unary_union([P(p) for p in D['prox_web'][:1]]).difference(kn.sector(39.0, 70, 13, 130))
pad = unary_union([P(p) for p in D['prox_mid']]); dist = unary_union([P(p) for p in D['dist_mid']])
plate = [P(p) for p in D['palm_web'][1:]]; deck_v7 = P(D['palm_web'][0])
hand_palm = Polygon([(-105, 0.5), (-14, 0.5), (-6, -4), (6, -5.4), (10, 14), (0, 26.5), (-10, 27.5), (-45, 27), (-105, 28)])
hand_fing = Polygon([(2, -5.4), (40, -5.4), (78, -5.0), (86, 2), (84, 9.5), (44, 12.5), (8, 14.5), (-4, 10)])
def draw(ax, q, title):
    for p in plate: fill(ax, p, fc=C['palm'], ec=C['palm_e'], lw=0.8, zorder=2)
    ax.plot(*deck_v7.exterior.xy, color='#999', lw=1.0, ls=':', zorder=2)
    fill(ax, hand_palm, fc=C['hand'], ec=C['hand_e'], lw=0.8, alpha=0.8, zorder=1)
    fill(ax, affinity.rotate(hand_fing, -q, origin=(-3, 8)), fc=C['hand'], ec=C['hand_e'], lw=0.8, alpha=0.8, zorder=1)
    for g in (pad, dist): fill(ax, kn.rot(g, q), fc=C['glove'], ec=C['link_e'], lw=0.7, zorder=3)
    # bridge + AA joint
    fill(ax, B['band'], fc=C['palm'], ec=C['palm_e'], lw=0.9, zorder=4); fill(ax, B['boss'], fc=C['palm'], ec=C['palm_e'], lw=0.9, zorder=4)
    fill(ax, B['sensor'], fc=C['pot'], ec='k', lw=0.5, zorder=6); fill(ax, B['pin'], fc=C['steel'], ec='k', lw=0.4, zorder=7)
    for b in (B['b1'], B['b2']): fill(ax, b, fc=C['brg'], ec='#700', lw=0.6, zorder=8)
    # link (web + arc)
    lk = unary_union([L['body'], web_v7]).difference(L['slot'])
    fill(ax, kn.rot(L['rack'], q), fc='#4d86c9', ec=C['link_e'], lw=0.5, zorder=5, hatch='||||')
    fill(ax, kn.rot(lk, q), fc=C['link'], ec=C['link_e'], lw=1.0, zorder=5)
    lp = kn.rot(Point(L['Lpin']), q)
    # housing
    hb = H['body'].difference(H['slot'])
    fill(ax, hb, fc=C['house'], ec=C['house_e'], lw=1.0, alpha=0.62, zorder=6)
    ax.plot(*H['slot'].exterior.xy, color=C['house_e'], lw=0.9, ls='--', zorder=6)
    ax.add_patch(Circle(kn.GEAR_C, kn.GEAR_RP + 1, fc=C['gear'], ec=C['gear_e'], lw=0.9, alpha=0.9, zorder=5.5, hatch='....'))
    ax.add_patch(Rectangle(kn.GEAR_C - np.array([5.5, 6.35]), 11, 12.7, fc=C['pot'], ec='k', lw=0.5, zorder=7, alpha=0.9)); ax.add_patch(Circle(kn.GEAR_C, 2, fc='#bbb', ec='k', lw=0.5, zorder=8))
    for c in (H['H'], (lp.x, lp.y)):
        ax.add_patch(Circle(c, 4.0, fc=C['brg'], ec='#700', lw=0.7, zorder=9)); ax.add_patch(Circle(c, 2.0, fc='#eee', ec='#700', lw=0.5, zorder=9)); ax.add_patch(Circle(c, 1.0, fc=C['steel'], ec='k', lw=0.3, zorder=9))
    ax.plot(0, 0, '+', color='k', ms=13, mew=1.5, zorder=10)
    a0, a1 = kn.aa_at(-8), kn.aa_at(98); ax.plot([a0[0], a1[0]], [a0[1], a1[1]], color='k', lw=0.8, ls='-.', zorder=10)
    ax.set_aspect('equal'); ax.set_xlim(-72, 96); ax.set_ylim(-62, 100); ax.axis('off')
    ax.text(-70, 99, title, fontsize=17, weight='bold', va='top')
    return lp
def note(ax, txt, xy, xt, ha='left', va='center', col='#222', fs=11.5):
    ax.annotate(txt, xy=xy, xytext=xt, fontsize=fs, color=col, ha=ha, va=va, zorder=20, arrowprops=dict(arrowstyle='-', color=col, lw=0.8, shrinkA=2, shrinkB=1), bbox=dict(fc='white', ec='none', alpha=0.85, pad=1.2))
fig = plt.figure(figsize=(26, 19))
gs = fig.add_gridspec(2, 2, height_ratios=[12.4, 5.6], width_ratios=[1, 1], hspace=0.02, wspace=0.02, left=0.01, right=0.99, top=0.955, bottom=0.01)
ax1 = fig.add_subplot(gs[0, 0]); ax2 = fig.add_subplot(gs[0, 1]); ax3 = fig.add_subplot(gs[1, 0]); ax4 = fig.add_subplot(gs[1, 1])
lp = draw(ax1, kn.QMIN, f'Finger fully stretched ({kn.QMIN:.0f}°)')
H0 = H['H']
note(ax1, 'Bridge = new upper plate. Underside 62 mm\nabove the knuckle axis (v7: 27 mm)', (-40, 67), (-70, 86), va='top')
note(ax1, 'Side-to-side joint: steel pin Ø3 turns in\ntwo 4×8×3 bearings, 14.5 mm apart', tuple(kn.aa_at(72.5) + np.array([1.5, 0])), (6, 90))
note(ax1, 'Angle sensor 3382H on top, as in v7', tuple(kn.aa_at(83.3) + np.array([5, 0])), (6, 80))
note(ax1, 'Pin clamped in the block of the housing\n(12.8 mm, M3 pinch screw)', tuple(kn.aa_at(58)), (-70, 54), va='center')
note(ax1, 'Housing: two cheeks with an arc slot (8.2 mm wide,\nwalls 5 mm). It turns only side to side', kn.pol(53.5, 72), (22, 68))
note(ax1, "Bearing pair on the finger's arc,\nrolls in the two cheek slots", (lp.x - 3, lp.y), (-70, 41), va='center')
note(ax1, 'Bearing fixed in the housing,\nrolls in the slot of the arc', tuple(H0 + np.array([3, -2])), (54, 4), va='center')
note(ax1, 'Arc = part of the finger piece (blue):\n5 mm plate with slot and gear teeth', kn.pol(57.2, 62), (22, 57), va='center')
note(ax1, 'Gear Ø31 and 3382H sensor, 4 : 1\n(73° of the finger = 292° at the sensor)', tuple(kn.GEAR_C + np.array([9, 9])), (52, 78))
note(ax1, 'ORCA knuckle axis = centre of the arc', (0, 0), (-8, -22), va='top')
note(ax1, 'upper plate of v7', (-40, 27.5), (-70, 17), col='#777', fs=10.5)
note(ax1, 'average hand, 28 mm thick', (-55, 10), (-70, -14), col='#8a5a3a', fs=10.5)
hb = H['body']; zt = hb.intersection(box(-80, -50, 0, 200)).bounds[1]; yt = -19.2
for (x, z0, z1, t, dx) in [(yt, 0.5, zt, f'{zt:.0f} mm', -2.5), (0, 0, kn.RI, f'{kn.RI:.0f} mm', 2.2)]:
    ax1.annotate('', xy=(x, z1), xytext=(x, z0), arrowprops=dict(arrowstyle='<->', color='#c00', lw=1.5), zorder=15)
    ax1.text(x + dx, 9, t, color='#c00', fontsize=13, weight='bold', ha='right' if dx < 0 else 'left', zorder=16, bbox=dict(fc='white', ec='none', alpha=0.8, pad=0.8))
lp2 = draw(ax2, kn.QMAX, f'Finger fully bent ({kn.QMAX:.0f}°)')
note(ax2, 'The arc has rolled forward out of the housing.\nThe two bearing stations are now 10 mm apart\n(stretched: 63 mm). ART works the same way.', ((H0[0] + lp2.x) / 2 + 3, (H0[1] + lp2.y) / 2 + 3), (40, 70), va='center')
note(ax2, 'Nothing is left behind the knuckle:\n62 mm free under the bridge', (-22, 45), (-70, 48), va='center')
note(ax2, 'Sensor of the middle joint, unchanged', tuple(np.array(kn.rot(Point(39.5, 0), kn.QMAX).coords[0])), (30, -52))
# ---- cross sections (lateral x horizontal)
def xsec(ax):
    ax.set_aspect('equal'); ax.axis('off')
    x0 = -16.7; cr = (x0, x0 + 4.15); lkx = (-12.2, -7.2); cu = (-6.85, -2.7)
    # view A-A through the link's bearing pair (rear), looking forward. vertical = radius
    for (a, b) in (cr, cu):
        ax.add_patch(Rectangle((a, kn.RI), b - a, kn.RO - kn.RI, fc=C['house'], ec=C['house_e'], lw=1, alpha=0.75))
    ax.add_patch(Rectangle((cr[0], kn.RO), cu[1] - cr[0], 5.4, fc=C['house'], ec=C['house_e'], lw=1, alpha=0.75))
    ax.add_patch(Rectangle((cr[1] - 3.6, kn.RS - 4.1), 3.6, 8.2, fc='white', ec=C['house_e'], lw=0.8)); ax.add_patch(Rectangle((cu[0], kn.RS - 4.1), 3.6, 8.2, fc='white', ec=C['house_e'], lw=0.8))
    ax.add_patch(Rectangle((lkx[0], kn.RS - 5.5), 5, 11.0, fc=C['link'], ec=C['link_e'], lw=1))
    ax.add_patch(Rectangle((lkx[0] - 3.35, kn.RS - 4), 3, 8, fc=C['brg'], ec='#700', lw=0.7)); ax.add_patch(Rectangle((lkx[1] + 0.35, kn.RS - 4), 3, 8, fc=C['brg'], ec='#700', lw=0.7))
    ax.add_patch(Rectangle((lkx[0] - 2.5, kn.RS - 1), 10, 2, fc=C['steel'], ec='k', lw=0.4))
    ax.add_patch(Rectangle((-15.35 - 1.5, kn.RS + 4.6), 3, 22, fc=C['steel'], ec='k', lw=0.4))
    ax.plot([-15.35, -15.35], [kn.RS - 12, kn.RS + 30], 'k-.', lw=0.7)
    ax.text(-19, kn.RS + 22, 'side-to-side axis,\nsteel pin Ø3', fontsize=11, ha='right', va='center')
    for (a, b, t) in [(cr[0], cr[1], '4.2'), (lkx[0], lkx[1], '5.0'), (cu[0], cu[1], '4.2')]:
        ax.annotate('', xy=(a, kn.RI - 2.5), xytext=(b, kn.RI - 2.5), arrowprops=dict(arrowstyle='<->', color='#333', lw=1)); ax.text((a + b) / 2, kn.RI - 5.2, t, ha='center', fontsize=11)
    ax.annotate('', xy=(cr[0], kn.RI - 8), xytext=(cu[1], kn.RI - 8), arrowprops=dict(arrowstyle='<->', color='#333', lw=1)); ax.text((cr[0] + cu[1]) / 2, kn.RI - 11, '14.0 mm wide (v7: 14.2)', ha='center', fontsize=11)
    ax.text(cr[0] - 1.5, kn.RS - 6, 'cheek', ha='right', va='center', fontsize=11, color=C['house_e']); ax.text(cu[1] + 1.5, kn.RS - 6, 'cheek', ha='left', va='center', fontsize=11, color=C['house_e'])
    ax.annotate('arc of the finger piece', xy=(-9.7, kn.RS + 3), xytext=(-40, kn.RS + 8), fontsize=11, color=C['link_e'], ha='right', arrowprops=dict(arrowstyle='-', color=C['link_e'], lw=0.8))
    ax.text(8, kn.RS + 4, 'Bearings 4×8×3 on a 2×10 steel pin,\nwith printed sleeves Ø2 → Ø4 (1 mm wall).\nThe cheek slots are pockets,\nclosed on the outside.\nThe front bearing sits the same way\nbetween the cheeks, in the slot of the arc.', ha='left', va='center', fontsize=11.5, linespacing=1.35)
    ax.text(-73, kn.RS + 33, 'Section through the rear bearing pair', fontsize=15, weight='bold', va='top')
    ax.text(-73, kn.RS + 28, 'seen from the fingertip, to scale', fontsize=11.5, va='top')
    ax.set_xlim(-75, 55); ax.set_ylim(kn.RI - 26, kn.RS + 34)
xsec(ax3)
ax4.axis('off')
txt = [('Numbers', None),
       ('Arc', f'{kn.RI:.0f}–{kn.RO:.0f} mm from the ORCA knuckle axis\n(v7: bow 41–51, guide 36.5–54)'),
       ('Bend range', f'{kn.QMIN:.0f}° to {kn.QMAX:.0f}°  ({kn.ROM:.0f}° total, hard stops at the slot ends)'),
       ('Free height', f'{zt:.0f} mm under the housing ends, {kn.RI:.0f} mm over the knuckles,\n62 mm under the bridge. v7: 27 mm under the upper plate'),
       ('Upper plate', 'about 35 mm higher than in v7, not 20: the hugging arc\nand its pivot block have to fit under it'),
       ('Axes', 'all joint axes stay exactly where they are in v7'),
       ('Per finger', '5 bearings 4×8×3 (3 in the arc, 2 at the pivot),\n1 pin Ø3 × 38 (cut from 3×50), 2 pins 2×10,\n3 screws M3×8 with nuts'),
       ('Printed parts', 'finger piece with arc (1), two cheeks, gear,\n4 small sleeves. All print flat, slots upright')]
y = 0.97
for k, v in txt:
    if v is None: ax4.text(0.02, y, k, fontsize=15, weight='bold', va='top', transform=ax4.transAxes); y -= 0.085; continue
    ax4.text(0.02, y, k, fontsize=11.5, weight='bold', va='top', transform=ax4.transAxes); ax4.text(0.17, y, v, fontsize=11.5, va='top', transform=ax4.transAxes, linespacing=1.35)
    y -= 0.075 + 0.058 * v.count('\n')
fig.suptitle('Glove v9 concept, sheet 1: knuckle joint in ART style (middle finger, side view, to scale; the other fingers are built the same way)', fontsize=17, y=0.985)
plt.savefig('/home/claude/out/Glove_v9_concept_1_knuckle.png', dpi=72)
print('ok', zt)
