import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, itertools, collections, time, kin9
M = kin9.Model(); t = time.time()
print('--- one finger at a time (others neutral)')
for f in kin9.FING:
    bad = collections.defaultdict(list); n = 0
    for aa in range(-25, 26, 5):
        for mcp in (-5, 0, 15, 30, 45, 60, 68):
            for pip in (0, 45, 90):
                M.pose({f'{f}_aa': aa, f'{f}_mcp': mcp, f'{f}_pip': pip}); n += 1
                for pr in M.collisions(): bad[pr].append((aa, mcp, pip))
    print(f, n, 'poses;', 'no collisions' if not bad else '')
    for pr, L in sorted(bad.items()):
        aas = sorted(set(x[0] for x in L)); mc = sorted(set(x[1] for x in L)); pp = sorted(set(x[2] for x in L))
        print(f'   {pr[0]} x {pr[1]}: {len(L)} poses; AA {aas}; MCP {mc}; PIP {pp}')
print('time', round(time.time() - t, 1))
