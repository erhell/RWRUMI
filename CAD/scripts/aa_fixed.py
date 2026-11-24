import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, kin9
M = kin9.Model(); FING = kin9.FING
fixed = lambda n: M.group[n] == 'fixed' or n.startswith('Thumb')
print('side-to-side angle at which a finger assembly first touches a fixed part (palm frame, board, camera mount) or the thumb at rest; positive = towards the thumb')
for f in FING:
    out = []
    for sgn in (1, -1):
        hit = None
        for a in range(0, 41):
            for mcp in (-5, 30, 68):
                M.pose({f'{f}_aa': sgn * a, f'{f}_mcp': mcp}); h, names = M.cm.in_collision_internal(return_names=True)
                for x, y in names:
                    if M.allowed(x, y): continue
                    gx, gy = M.group[x], M.group[y]
                    if (gx.startswith(f) and fixed(y)) or (gy.startswith(f) and fixed(x)): hit = (a, x, y, mcp); break
                if hit: break
            if hit: break
        out.append(f'{"+" if sgn > 0 else "-"}: ' + (f'{hit[0]} deg ({hit[1]} x {hit[2]}, MCP {hit[3]})' if hit else 'free to 40 deg'))
    print(f'  {f:6s} ' + ' | '.join(out))
