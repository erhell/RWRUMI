"""Camera mount for the OAK-D Pro: pad on the vertical part of the ulnar side wall, flat arm, top bar with fixed jaw, removable jaw."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, math, occ, json
V9 = '/home/claude/cache/v9'
# wall outer face (vertical part): through (64.3, 136) and (68.1, 156) in plan
P0 = np.array([64.3, 136.0, 0.0]); tdir = np.array([68.1 - 64.3, 156.0 - 136.0, 0.0]); tdir /= np.linalg.norm(tdir)
NW = np.array([tdir[1], -tdir[0], 0.0])            # outward normal of the wall face
SD = -tdir                                          # rearward along the wall face
TA = occ.trsf_from_frame(P0, NW, SD, [0, 0, 1.0])   # arm frame: x = out of the wall, y = rearward along the face, z = up
PAD_T, ARM_T = 4.6, 9.0
PAD_S, PAD_Z = (-21.0, 1.0), (49.0, 79.0)
SCREWS = [(-17.0, 55.0), (-4.0, 55.0), (-17.0, 73.0), (-4.0, 73.0)]
CAM = dict(x=(-33.5, 63.5), y=(46.4, 69.5), z=(-46.75, -17.25))      # lens plane y = 69.5, centre z = -32, 0 deg tilt
BAR = dict(x=(-33.5, 68.6), y=(47.9, 68.0), z=(-17.1, -10.6))
def w2a(p): p = np.asarray(p, float) - P0; return np.array([p @ NW, p @ SD, p[2]])
def screw_axes():
    """world (point on the pad outer face, direction into the wall) of the four pad screws"""
    out = []
    for (s, z) in SCREWS: out.append((P0 + NW * PAD_T + SD * s + np.array([0, 0, z]), -NW))
    return out
def arm(palm):
    pad = occ.move(occ.box(-7.0, PAD_S[0], PAD_Z[0], PAD_T, PAD_S[1], PAD_Z[1]), TA)
    pieces = occ.solids(occ.cut(pad, palm)); pad = max(pieces, key=lambda s: occ.centroid(s) @ NW)       # the piece outside the wall
    end = w2a([53.0, (BAR['y'][0] + BAR['y'][1]) / 2, -7.0])
    beam = occ.hull2((-9.0, 64.0), 13.0, (end[1], end[2]), 9.5, 0.0, ARM_T)
    beam = occ.fuse(beam, occ.box(0.0, PAD_S[0], PAD_Z[0], PAD_T, PAD_S[1], PAD_Z[1]))
    beam = occ.smooth_prism(occ.unify(occ.fuse(beam, occ.box(0.0, PAD_S[0], PAD_Z[0], ARM_T, PAD_S[1], PAD_Z[1]))), 0.0, ARM_T, 6.0, 3.0)
    beam = occ.cut(occ.move(beam, TA), palm)
    b = BAR; c = CAM
    bar = occ.box(b['x'][0], b['y'][0], b['z'][0], b['x'][1], b['y'][1], b['z'][1])
    jaw = occ.fuse(occ.box(c['x'][1] + 0.15, c['y'][0] - 2.1, c['z'][0] - 2.85, b['x'][1], c['y'][1] + 2.1, b['z'][1]),              # end plate
                   occ.box(c['x'][1] - 6.0, c['y'][0] - 2.1, c['z'][0] - 2.85, b['x'][1], c['y'][1] + 2.1, c['z'][0] - 0.25),          # lip under the camera
                   occ.box(c['x'][1] - 1.9, c['y'][1] + 0.15, c['z'][0] - 2.85, b['x'][1], c['y'][1] + 2.1, b['z'][1]),                 # front return
                   occ.box(c['x'][1] - 1.9, c['y'][0] - 2.1, c['z'][0] - 2.85, b['x'][1], c['y'][0] - 0.15, b['z'][1]))                 # rear return
    s = occ.fuse(occ.biggest(beam), pad, bar, jaw)
    # pad screws: clearance 3.3, counterbore 6.2 leaving 1.6 mm under the head
    for (p, d) in screw_axes(): s = occ.cut(s, occ.cyl(p - d * 6.0, d, 1.65, 20.0), occ.cyl(p - d * 6.0, d, 3.1, 6.0 + PAD_T - 1.6))
    # nut slots and threads for the removable jaw (two M3 along x at the radial bar end)
    for y in (52.5, 63.5):
        zc = (b['z'][0] + b['z'][1]) / 2
        s = occ.cut(s, occ.cyl([b['x'][0] - 1, y, zc], [1, 0, 0], 1.65, 9.0), occ.box(b['x'][0] + 2.6, y - 2.85, zc - 3.3, b['x'][0] + 5.3, y + 2.85, b['z'][1] + 1))
    return occ.unify(s)
def jaw():
    b = BAR; c = CAM
    j = occ.fuse(occ.box(b['x'][0] - 5.1, c['y'][0] - 2.1, c['z'][0] - 2.85, c['x'][0] - 0.15, c['y'][1] + 2.1, b['z'][1]),
                 occ.box(b['x'][0] - 5.1, c['y'][0] - 2.1, c['z'][0] - 2.85, c['x'][0] + 6.0, c['y'][1] + 2.1, c['z'][0] - 0.25),
                 occ.box(b['x'][0] - 5.1, c['y'][1] + 0.15, c['z'][0] - 2.85, c['x'][0] + 1.9, c['y'][1] + 2.1, b['z'][1]),
                 occ.box(b['x'][0] - 5.1, c['y'][0] - 2.1, c['z'][0] - 2.85, c['x'][0] + 1.9, c['y'][0] - 0.15, b['z'][1]))
    zc = (b['z'][0] + b['z'][1]) / 2
    for y in (52.5, 63.5): j = occ.cut(j, occ.cyl([b['x'][0] - 6, y, zc], [1, 0, 0], 1.65, 8.0), occ.cyl([b['x'][0] - 6, y, zc], [1, 0, 0], 3.0, 0.9 + 1.5))
    return occ.unify(j)
def camera_dummy():
    c = CAM; return occ.box(c['x'][0], c['y'][0], c['z'][0], c['x'][1], c['y'][1], c['z'][1])
def wall_holes():
    """clearance holes through the palm's ulnar wall for the four pad screws"""
    return [occ.cyl(p - d * 2.0, d, 1.65, 16.0) for (p, d) in screw_axes()]
if __name__ == '__main__':
    palm = occ.load(f'{V9}/Palm_stage1.brep')
    a = arm(palm); occ.report('Camera_Arm', a); occ.save(a, f'{V9}/Camera_Arm.brep')
    j = jaw(); occ.report('Camera_Jaw', j); occ.save(j, f'{V9}/Camera_Jaw.brep')
    cd = camera_dummy(); occ.save(cd, f'{V9}/Camera_OAK-D_Pro_dummy.brep')
    print('arm x palm interference', round(occ.interference(a, palm), 3), '| arm x camera', round(occ.interference(a, cd), 3), '| jaw x camera', round(occ.interference(j, cd), 3), '| jaw x arm', round(occ.interference(j, a), 3))
    lo, hi = occ.bbox(a); print('arm bbox', lo.round(1), hi.round(1))
