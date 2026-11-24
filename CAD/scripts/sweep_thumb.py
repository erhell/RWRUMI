import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, math, kin9, collections
M = kin9.Model()
cam = lambda n: n.startswith('Camera'); thumb = lambda n: n.startswith('Thumb')
print('neutral: thumb to camera mount distance %.1f mm (%s)' % M.dist(thumb, cam))
grid_c = range(-60, 61, 10); grid_b = range(-40, 41, 10)
hits = collections.defaultdict(list); mind = (1e9, None)
linkhits = collections.defaultdict(list)
for c in grid_c:
    for b in grid_b:
        for m in (0, 25, 50):
            M.pose({'thumb_cmc': c, 'thumb_abd': b, 'thumb_mcp': m})
            hit, names = M.cm.in_collision_internal(return_names=True)
            for (x, y) in names:
                if (cam(x) and thumb(y)) or (cam(y) and thumb(x)): hits[(c, b)].append((x, y, m))
                for nm in ('Thumb_Link', 'Thumb_Pin2x10_LinkA', 'Thumb_Pin2x10_LinkB'):
                    if nm in (x, y) and not M.allowed(x, y): linkhits[tuple(sorted((x, y)))].append((c, b, m))
print('thumb base angle (CMC) x spread angle (ABD) grid, MCP 0/25/50.  X = thumb touches the camera mount, . = free')
print('      ABD ' + ' '.join(f'{b:4d}' for b in grid_b))
for c in grid_c: print(f'CMC {c:4d}  ' + ' '.join('   X' if (c, b) in hits else '   .' for b in grid_b))
parts = collections.Counter(tuple(sorted((x, y))) for v in hits.values() for (x, y, m) in v)
print('pairs:', dict(parts))
print('link contacts with non-neighbour parts:', {k: (len(v), v[:3]) for k, v in linkhits.items()})
# which rotation direction brings the thumb towards the palm? report the tip position
tipn = 'Thumb_DP'
for c in (-40, 0, 40):
    for b in (-30, 0, 30):
        M.pose({'thumb_cmc': c, 'thumb_abd': b}); T = M.T['thumb_dp']; p = (T @ np.array([-43.84, 166.78, -68.19, 1.0]))[:3]; print(f'  CMC {c:4d} ABD {b:4d}: thumb tip at {p.round(0)}')
