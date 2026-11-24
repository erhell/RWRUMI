"""Concept geometry of the new knuckle (MCP) joint, middle finger, in the (Y,Z) plane about the ORCA MCP axis [mm]."""
import numpy as np
from shapely.geometry import Polygon, Point, LineString, box
from shapely.ops import unary_union
from shapely import affinity
RS, SW, WALL = 47.0, 8.2, 5.0
RI, RO = RS - SW / 2 - WALL, RS + SW / 2 + WALL          # 37.9, 56.1
PH = 29.0; QMIN, QMAX = -5.0, 68.0; ROM = QMAX - QMIN
DELTA = np.degrees(9.6 / RS)                             # slot end -> neighbouring axle
RACK_R = 57.6; GEAR_RP = 14.4; PHA = 34.0                # rack pitch radius, gear pitch radius, gear axle angle
AA_P0 = np.array([-19.21, 1.242]); AA_D = np.array([0.0651, 0.9979])   # AA axis in this plane (from v7)
BAND_Z = 62.5; BAND_T = 9.0
def pol(r, a): return np.array([r * np.cos(np.radians(a)), r * np.sin(np.radians(a))])
def arc(r, a0, a1, n=120): return LineString([pol(r, a) for a in np.linspace(a0, a1, n)])
def sector(r0, r1, a0, a1, n=120):
    return Polygon([pol(r1, a) for a in np.linspace(a0, a1, n)] + [pol(r0, a) for a in np.linspace(a1, a0, n)])
def rot(g, q): return affinity.rotate(g, -q, origin=(0, 0))      # flexion q>0 turns the finger down
def aa_at(z): t = (z - AA_P0[1]) / AA_D[1]; return AA_P0 + AA_D * t
# ---- link (moves with the finger); finger frame = v7 file pose (q = 0)
L_ANG = PH + QMAX + DELTA                                  # 108.7
def link_parts():
    body = unary_union([arc(RS, PH + QMIN, PH + QMAX).buffer(SW / 2 + WALL), arc(RS, PH + QMAX, L_ANG).buffer(5.5)]).buffer(3).buffer(-3)
    slot = arc(RS, PH + QMIN, PH + QMAX).buffer(SW / 2)
    rack = sector(RO - 0.2, RACK_R + 1.0, PHA + QMIN - 3, PHA + QMAX + 3)
    return dict(body=body, slot=slot, rack=rack, Lpin=pol(RS, L_ANG))
# ---- housing (turns only side to side, about the AA axis)
GEAR_C = pol(RACK_R + GEAR_RP, PHA)
def housing_parts():
    c0, c1 = PH + DELTA, PH + DELTA + ROM                  # travel of the link's bearing pair: 40.7 .. 113.7
    cheek = unary_union([arc(RS, c0, c1 - 3.7).buffer(SW / 2 + WALL), Point(pol(RS, c1)).buffer(SW / 2 + 4.0), Point(pol(RS, PH)).buffer(5.2)])
    nose = unary_union([Point(GEAR_C).buffer(6.0), Point(pol(RO - 4.5, PH + 3)).buffer(4.5), Point(pol(RO - 4.5, PHA + 22)).buffer(4.5)]).convex_hull
    a = aa_at(55.0)
    block = Polygon([(a[0] - 9.5, 46), (a[0] - 9.5, 58.5), (a[0] - 6.5, 61.5), (a[0] + 6.0, 61.5), (a[0] + 8.5, 58.5), (a[0] + 8.5, 48)]).difference(Point(0, 0).buffer(RO - 2))
    body = unary_union([cheek, nose, block]).buffer(2).buffer(-2)
    slot = arc(RS, c0, c1).buffer(SW / 2)
    return dict(body=body, slot=slot, block=block, H=pol(RS, PH), c0=c0, c1=c1)
def bridge_parts():
    a_lo, a_hi = aa_at(BAND_Z), aa_at(BAND_Z + BAND_T)
    band = Polygon([(-54, BAND_Z), (a_lo[0] + 9, BAND_Z), (a_lo[0] + 9, BAND_Z + BAND_T), (-54, BAND_Z + BAND_T)]).buffer(-2).buffer(2)
    n = np.array([AA_D[1], -AA_D[0]])
    def rect(z0, z1, w): p0, p1 = aa_at(z0), aa_at(z1); return Polygon([p0 - n * w / 2, p0 + n * w / 2, p1 + n * w / 2, p1 - n * w / 2])
    return dict(band=band, boss=rect(BAND_Z + BAND_T - 0.5, 82.0, 14.0), b1=rect(63.5, 66.5, 8.0), b2=rect(78.0, 81.0, 8.0), pin=rect(48.7, 86.5, 3.0), sensor=rect(82.0, 84.6, 11.1), dtip=rect(84.6, 87.5, 4.0))
if __name__ == '__main__':
    L = link_parts(); H = housing_parts()
    print('radii', RI, RS, RO, 'delta', round(DELTA, 2), 'link slot', PH + QMIN, PH + QMAX, 'L pin at', round(L_ANG, 1), 'cheek slot', round(H['c0'], 1), round(H['c1'], 1))
    hb = H['body']; rear = hb.intersection(box(-80, -50, 0, 200)); print('housing lowest point behind knuckle axis Z=%.1f, over the axis Z=%.1f, rear-most Y=%.1f, top Z=%.1f' % (rear.bounds[1], RI, hb.bounds[0], hb.bounds[3]))
    for q in (QMIN, 0, 30, QMAX):
        lb = rot(unary_union([L['body'], L['rack']]), q); lp = rot(Point(L['Lpin']), q)
        print('q=%5.1f link: Ymin %.1f Zmin(behind axis) %.1f Zmax %.1f  L at world %.1f deg, baseline H-L %.1f mm' % (q, lb.bounds[0], lb.intersection(box(-80, -50, 0, 200)).bounds[1] if not lb.intersection(box(-80, -50, 0, 200)).is_empty else float('nan'), lb.bounds[3], np.degrees(np.arctan2(lp.y, lp.x)), np.hypot(lp.x - H['H'][0], lp.y - H['H'][1])))
    print('gear centre', GEAR_C.round(1), 'radius from axis', round(np.hypot(*GEAR_C), 1), 'ratio', round(RACK_R / GEAR_RP, 2), 'sensor turn over ROM', round(ROM * RACK_R / GEAR_RP, 0))
    print('AA axis at Z=55:', aa_at(55).round(2), ' pin clamp length', 61.5 - 48.7)
