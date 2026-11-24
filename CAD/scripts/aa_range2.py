import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, math, kin9
from trimesh.collision import CollisionManager
M = kin9.Model(); FING = kin9.FING
NEW = ('MCP_Cheek', 'MCP_Gear', '3382H_MCP', 'MCP_Bearing', 'MCP_Pin', 'MCP_Sleeve', 'MCP_RingH', 'AA_', 'Camera', 'PCB', 'Palm', 'Thumb_Link', 'Thumb_AP', 'Thumb_DP')
def fing(n): g = M.group[n].split('_')[0]; return g if g in FING else ('thumb' if g == 'thumb' else 'fixed')
print('free side-to-side angle of one finger, others neutral (MCP 0/30/68). positive = towards the thumb')
for f in FING:
    out = []
    for sgn in (1, -1):
        first_any = None; first_new = None
        for a in np.arange(0, 30.1, 1.0):
            for mcp in (0, 30, 68):
                M.pose({f'{f}_aa': sgn * a, f'{f}_mcp': mcp})
                hit, names = M.cm.in_collision_internal(return_names=True)
                for (x, y) in names:
                    fx, fy = fing(x), fing(y)
                    if fx == fy or f not in (fx, fy): continue
                    if (kin9.is_hw(x) and fx == 'fixed') or (kin9.is_hw(y) and fy == 'fixed'): continue
                    if first_any is None: first_any = (a, x, y, mcp)
                    isnew = any(k in x for k in NEW) or any(k in y for k in NEW)
                    if isnew and first_new is None: first_new = (a, x, y, mcp)
            if first_any and first_new: break
        out.append((first_any, first_new))
    for (fa, fn), lab in zip(out, ('towards thumb', 'towards little finger')):
        print(f'  {f:7s} {lab:22s}: first contact at {fa[0] if fa else ">30":>4} deg ({fa[1]+" x "+fa[2] if fa else ""}); first contact involving a new part at {fn[0] if fn else ">30":>4} deg ({fn[1]+" x "+fn[2] if fn else ""})')
