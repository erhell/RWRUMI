import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, kin9, asm, rend
M = kin9.Model()
def items(q, skip=lambda n: False):
    M.pose(q); out = []
    for n in M.names:
        if skip(n): continue
        T = M.T[M.group[n]]; V = M.mesh[n].vertices @ T[:3, :3].T + T[:3, 3]; out.append((V, M.mesh[n].faces, asm.color(n)))
    return out
neutral = items({}); fist = items({**{f'{f}_mcp': 60 for f in kin9.FING}, **{f'{f}_pip': 60 for f in kin9.FING}, 'thumb_mcp': 40})
c = np.array([5.0, 150.0, 30.0]); S = (1500, 1150)
views = [('a_front_top', neutral, (-0.55, 0.65, 0.52), (0, 0, 1), 125), ('b_rear_side', neutral, (0.75, -0.55, 0.38), (0, 0, 1), 125), ('c_side', neutral, (1, 0, 0.0), (0, 0, 1), 125),
         ('d_top', neutral, (0, 0.001, 1), (0, 1, 0), 125), ('e_below', neutral, (-0.5, 0.5, -0.7), (0, 0, 1), 125), ('f_bent', fist, (0.8, 0.45, 0.4), (0, 0, 1), 125)]
fns = []
for name, it, dv, up, sc in views:
    fn = f'/home/claude/out/v9_{name}.png'; rend.render(it, fn, pos=c + 600 * np.array(dv, float), focal=c, up=up, size=S, edges=True, scale=sc); fns.append(fn)
asm.montage(fns, '/home/claude/out/v9_overview.png', cols=3, w=2400)
# close-up of the middle knuckle joint, ulnar cheek and sensor removed
d = M.fr['middle']; cc = d['O'] + d['U'] * 45 + d['E'] * 10
close = items({'middle_mcp': 30}, skip=lambda n: not (n.startswith('Middle_') or n == 'Palm') or n in ('Middle_MCP_CheekUlnar', 'Middle_3382H_MCP'))
rend.render(close, '/home/claude/out/v9_joint.png', pos=cc + 400 * np.array([1.0, 0.15, 0.1]), focal=cc, up=(0, 0, 1), size=(1500, 1150), edges=True, scale=62)
print('ok')
