"""Realistic hand patterns: fist (all fingers together), spread fan, all fingers sideways, single finger bent while the others stay straight."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, kin9, collections
M = kin9.Model(); FING = kin9.FING
def run(label, poses):
    bad = collections.defaultdict(list)
    for tag, q in poses:
        M.pose(q)
        for pr in M.collisions(): bad[pr].append(tag)
    print(f'{label}: {len(poses)} poses ->', 'no collisions' if not bad else '')
    for pr, L in sorted(bad.items()): print(f'    {pr[0]} x {pr[1]}: {len(L)} poses, e.g. {L[:4]}')
# 1 fist: all MCP equal, PIP equal
run('all four fingers bent together', [((m, p), {**{f'{f}_mcp': m for f in FING}, **{f'{f}_pip': p for f in FING}}) for m in (-5, 0, 10, 20, 30, 40, 50, 60, 68) for p in (0, 30, 60, 90)])
# 2 spread fan: index +a, middle +a/3, ring -a/3, pinky -a  (positive = towards the thumb)
for a in (5, 10, 15, 20, 25):
    run(f'spread fan, outer fingers {a} deg', [((m, p), {'index_aa': a, 'middle_aa': a / 3, 'ring_aa': -a / 3, 'pinky_aa': -a, **{f'{f}_mcp': m for f in FING}, **{f'{f}_pip': p for f in FING}}) for m in (-5, 0, 20, 40, 68) for p in (0, 45, 90)])
# 3 all sideways together
for a in (-15, -10, -5, 5, 10, 15):
    run(f'all four fingers sideways {a:+d} deg', [((m,), {**{f'{f}_aa': a for f in FING}, **{f'{f}_mcp': m for f in FING}}) for m in (-5, 0, 20, 40, 68)])
# 4 one finger bent, others straight, and one finger straight, others bent
for f in FING:
    run(f'only {f} bent', [((m, p), {f'{f}_mcp': m, f'{f}_pip': p}) for m in (-5, 10, 20, 30, 40, 50, 60, 68) for p in (0, 45, 90)])
for f in FING:
    run(f'all bent except {f}', [((m, p), {**{f'{g}_mcp': m for g in FING if g != f}, **{f'{g}_pip': p for g in FING if g != f}}) for m in (10, 20, 30, 40, 50, 60, 68) for p in (0, 45, 90)])
# 5 neighbour differential at small angles
for f in FING:
    run(f'{f} alone at -5 (others 0)', [((0,), {f'{f}_mcp': -5})])
