"""Kinematic model of glove v9 on meshes, with fast collision / clearance queries (FCL)."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, json, math, itertools, trimesh, fcl
from trimesh.collision import CollisionManager
import asm, geo, finger9 as F, thumb9 as TH
FING = ('index', 'middle', 'ring', 'pinky'); CAP = {'index': 'Index', 'middle': 'Middle', 'ring': 'Ring', 'pinky': 'Pinky'}
TA = json.load(open('/home/claude/cache/v7/thumb_axes.json')); AX7 = json.load(open('/home/claude/cache/v7/axes.json'))
def rot(p, d, ang):
    p = np.asarray(p, float); d = np.asarray(d, float); d = d / np.linalg.norm(d); T = trimesh.transformations.rotation_matrix(ang, d, p); return T
HARD = ('Bearing', 'Pin', 'Sleeve', 'RingH', 'DTip', '3382H')
def is_hw(n): t = n.split('_', 1)[1] if '_' in n else n; return any(h in t for h in HARD)
def group_of(n):
    for f in FING:
        N = CAP[f]
        if n.startswith(N + '_'):
            t = n[len(N) + 1:]
            if t in ('Proximal', 'MCP_Pin2x10_L', 'MCP_Bearing_4x8x3_L1', 'MCP_Bearing_4x8x3_L2', 'MCP_SleeveL_1', 'MCP_SleeveL_2', '3382H_PIP'): return f'{f}_mcp'
            if t == 'Distal': return f'{f}_pip'
            if t == 'MCP_Gear': return f'{f}_gear'
            if t in ('AA_Bearing_4x8x3_1', 'AA_Bearing_4x8x3_2', '3382H_AA'): return 'fixed'
            return f'{f}_aa'
    if n in ('Thumb_TP', 'Thumb_3382H_T2', 'Thumb_Bearing_4x8x3_T2a', 'Thumb_Bearing_4x8x3_T2b'): return 'thumb_tp'
    if n in ('Thumb_AP', 'Thumb_3382H_T3', 'Thumb_Bearing_4x8x3_T3', 'Thumb_Pin2x10_LinkA'): return 'thumb_ap'
    if n in ('Thumb_PP', 'Thumb_3382H_T4', 'Thumb_Bearing_4x8x3_T4'): return 'thumb_pp'
    if n in ('Thumb_DP', 'Thumb_Pin2x10_LinkB'): return 'thumb_dp'
    if n == 'Thumb_Link': return 'thumb_link'
    return 'fixed'
class Model:
    def __init__(self, lin=0.08, skip=lambda n: False):
        self.mesh = {}; self.group = {}
        for n, fn in asm.new_parts().items():
            if skip(n): continue
            V, Fc = asm.mesh_of(fn, lin); self.mesh[n] = trimesh.Trimesh(V, Fc, process=False)
        for i, n in asm.KEEP_V7.items():
            if skip(n): continue
            V, Fc = geo.mesh_cached('v7', i); self.mesh[n] = trimesh.Trimesh(V, Fc, process=False)
        for n in self.mesh: self.group[n] = group_of(n)
        self.cm = CollisionManager()
        for n, m in self.mesh.items(): self.cm.add_object(n, m)
        self.names = list(self.mesh)
        self.fr = {f: {k: np.array(v) if isinstance(v, list) else v for k, v in F.FR[f].items() if k != 'ids'} for f in FING}
        self.pip = {f: np.array(AX7[f]['pip_c']) for f in FING}
        self.t1 = (np.array(TA['T1d'][0]), np.array(TA['T1q'][0]) - np.array(TA['T1d'][0])); self.t2 = (np.array(TA['T2a'][0]), np.array(TA['T2b'][0]) - np.array(TA['T2a'][0]))
        self.pose({})
    def transforms(self, q):
        T = {'fixed': np.eye(4)}
        for f in FING:
            d = self.fr[f]; al = math.radians(q.get(f'{f}_aa', 0.0)); m = math.radians(q.get(f'{f}_mcp', 0.0)); p = math.radians(q.get(f'{f}_pip', 0.0))
            Taa = rot(d['aa_p'], d['aa_v'], al); Tm = Taa @ rot(d['O'], d['X'], -m); Tp = Tm @ rot(self.pip[f], d['X'], -p)
            gc = d['O'] + d['E'] * F.GC[0] + d['U'] * F.GC[1]; Tg = Taa @ rot(gc, d['X'], m * F.ZS / F.ZG)
            T[f'{f}_aa'] = Taa; T[f'{f}_mcp'] = Tm; T[f'{f}_pip'] = Tp; T[f'{f}_gear'] = Tg
        c = math.radians(q.get('thumb_cmc', 0.0)); b = math.radians(q.get('thumb_abd', 0.0)); th = math.radians(q.get('thumb_mcp', 0.0))
        Ttp = rot(*self.t1, c); Tap = Ttp @ rot(*self.t2, b); Tpp = Tap @ rot(TH.M, TH.a, th); ti = TH.ip_of(th); Gw = TH.G
        Tdp = Tpp @ rot(Gw, TH.a, ti)
        Gq = TH.Rm(-th) @ np.array([TH.L0, 0.0]); Bq = Gq + TH.Rm(-(th + ti)) @ TH.B2; B0 = np.array([TH.L0, 0.0]) + TH.B2
        ang = math.atan2(*(Bq - TH.A2)[::-1]) - math.atan2(*(B0 - TH.A2)[::-1]); Aw = TH.M + TH.d * TH.A2[0] + TH.w * TH.A2[1]
        T['thumb_tp'] = Ttp; T['thumb_ap'] = Tap; T['thumb_pp'] = Tpp; T['thumb_dp'] = Tdp; T['thumb_link'] = Tap @ rot(Aw, TH.a, -ang)
        return T
    def pose(self, q):
        self.q = dict(q); self.T = self.transforms(q)
        for n in self.names: self.cm.set_transform(n, self.T[self.group[n]])
    def allowed(self, a, b):
        ga, gb = self.group[a], self.group[b]
        if ga == gb: return True
        fa = ga.split('_')[0]; fb = gb.split('_')[0]
        hw = is_hw(a) or is_hw(b)
        if hw and (fa == fb or 'fixed' in (ga, gb)): return True          # hardware sits in / on its own neighbours
        pair = {a.split('_', 1)[-1] if fa in FING or True else a, b}
        ta = a.split('_', 1)[1] if '_' in a else a; tb = b.split('_', 1)[1] if '_' in b else b
        if fa == fb and fa in FING:
            s = {ta, tb}
            if s == {'Proximal', 'Distal'} or s == {'MCP_Gear', 'Proximal'} or 'MCP_Gear' in s and (s & {'MCP_CheekRadial', 'MCP_CheekUlnar'}): return True
        if fa == fb == 'thumb':
            order = ['thumb_tp', 'thumb_ap', 'thumb_pp', 'thumb_dp']
            if 'thumb_link' in (ga, gb) and (set((ga, gb)) & {'thumb_ap', 'thumb_dp'}): return True
            if ga in order and gb in order and abs(order.index(ga) - order.index(gb)) == 1: return True
        if {a, b} == {'Palm', 'Thumb_TP'} or {a, b} <= {'Palm', 'Camera_Arm', 'Camera_Jaw', 'Camera_OAK-D_Pro_dummy', 'PCB_82x50_dummy'}: return True
        return False
    def collisions(self):
        hit, names = self.cm.in_collision_internal(return_names=True)
        return sorted(p for p in (tuple(sorted(x)) for x in names) if not self.allowed(*p)) if hit else []
    def dist(self, sel_a, sel_b):
        """min distance between two sets of parts (name predicates)"""
        A = [n for n in self.names if sel_a(n)]; B = [n for n in self.names if sel_b(n)]; best = (1e9, None, None)
        ma = CollisionManager(); mb = CollisionManager()
        for n in A: ma.add_object(n, self.mesh[n], self.T[self.group[n]])
        for n in B: mb.add_object(n, self.mesh[n], self.T[self.group[n]])
        d, nm = ma.min_distance_other(mb, return_names=True); return d, nm
if __name__ == '__main__':
    import time; t = time.time(); M = Model(); print(len(M.names), 'parts, model built in', round(time.time() - t, 1), 's')
    t = time.time(); c = M.collisions(); print('neutral pose collisions:', c, round(time.time() - t, 2), 's')
