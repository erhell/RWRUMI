import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, json, math, geo
from scipy.spatial.transform import Rotation as R
exec(open('/home/claude/scripts/cmp_orca.py').read().split("print('=== chain numbers")[0])
tg = json.load(open('/home/claude/cache/orca/to_glove.json')); Rm = R.from_rotvec(tg['rotvec']).as_matrix(); t = A(tg['t'])
mp = lambda l: (Rm @ l[0] + t, Rm @ l[1])
print('--- ORCA AA bearings taken one by one (they are 0.10 mm out of line with each other in the ORCA file)')
for f in FING:
    d = O[f]['aa']; va = A(d['v'])
    for k in ('pa', 'pb'):
        l = (A(d[k]), va); dist, _, _ = cn(l, Oa[f]['mcp']); lg = mp(l); print(f'   {f:6s} {k}: AA-MCP distance {dist:.4f} mm; offset of the glove AA axis from this bearing axis {ptline(lg[0], G[f]["aa"]):.4f} mm')
print('--- joint angles of the glove file pose relative to the ORCA file pose (zero offsets)')
def signed(u, v, ax): u = u - ax * np.dot(u, ax); v = v - ax * np.dot(v, ax); return math.degrees(math.atan2(np.dot(np.cross(u, v), ax), np.dot(u, v)))
for f in FING:
    aa_g = G[f]['aa'][1] / np.linalg.norm(G[f]['aa'][1]); mo = mp(Oa[f]['mcp']); po = mp(Oa[f]['pip']); vo = mo[1] if np.dot(mo[1], G[f]['mcp'][1]) > 0 else -mo[1]
    daa = signed(vo, G[f]['mcp'][1], aa_g)
    # rotate the ORCA finger by daa about the glove AA axis, then compare the MCP->PIP direction about the MCP axis
    Raa = R.from_rotvec(aa_g * math.radians(daa)).as_matrix(); pa = G[f]['aa'][0]
    mo_p = Raa @ (mo[0] - pa) + pa; po_p = Raa @ (po[0] - pa) + pa; X = G[f]['mcp'][1] / np.linalg.norm(G[f]['mcp'][1])
    eo = po_p - mo_p; eo -= X * np.dot(eo, X); eg = G[f]['pip'][0] - G[f]['mcp'][0]; eg -= X * np.dot(eg, X)
    dm = signed(eo, eg, X); off = ptline(mo_p, G[f]['mcp'])
    lat = np.dot((G[f]['pip'][0] - G[f]['mcp'][0]), X) - np.dot(po_p - mo_p, X)
    print(f'   {f:6s}: AA {daa:+.4f} deg, MCP {dm:+.4f} deg (glove MCP axis is {off:.4f} mm from the ORCA one after the AA turn)')
S = Oa['thumb']; Gt = G['thumb']; c_g = Gt['cmc'][1] / np.linalg.norm(Gt['cmc'][1]); ab_o = mp(S['abd']); vo = ab_o[1] if np.dot(ab_o[1], Gt['abd'][1]) > 0 else -ab_o[1]
dc = signed(vo, Gt['abd'][1], c_g); print(f'   thumb : CMC {dc:+.4f} deg', end='')
Rc = R.from_rotvec(c_g * math.radians(dc)).as_matrix(); pc = Gt['cmc'][0]; T1 = lambda p: Rc @ (p - pc) + pc
m_o = mp(S['mcp']); m_o = (T1(m_o[0]), Rc @ m_o[1]); b_g = Gt['abd'][1] / np.linalg.norm(Gt['abd'][1]); vo = m_o[1] if np.dot(m_o[1], Gt['mcp'][1]) > 0 else -m_o[1]
db = signed(vo, Gt['mcp'][1], b_g); print(f', ABD {db:+.4f} deg', end='')
Rb = R.from_rotvec(b_g * math.radians(db)).as_matrix(); pb = Gt['abd'][0]; T2 = lambda p: Rb @ (p - pb) + pb
i_o = mp(S['ip']); mo_p = T2(T1(mp(S['mcp'])[0])); io_p = T2(T1(i_o[0])); a = Gt['mcp'][1] / np.linalg.norm(Gt['mcp'][1])
eo = io_p - mo_p; eo -= a * np.dot(eo, a); eg = Gt['ip'][0] - Gt['mcp'][0]; eg -= a * np.dot(eg, a); print(f', MCP {signed(eo, eg, a):+.4f} deg')
