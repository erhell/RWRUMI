import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, occ, finger9 as F, time
f = sys.argv[1] if len(sys.argv) > 1 else 'middle'
t = time.time()
arc = F.link_arc(f); cr = F.cheek_radial(f); cu = F.cheek_ulnar(f); g = F.gear(f); sp = F.small_parts(f); ap = F.aa_parts(f)
fixed = {'cheekR': cr, 'cheekU': cu, 'H bearing': sp['MCP_Bearing_4x8x3_H'], 'H pin': sp['MCP_Pin2x10_H'], 'H sleeve': sp['MCP_SleeveH'], 'H ring': sp['MCP_RingH'], 'AA pin': ap['AA_Pin3x33']}
moving = {'arc': arc, 'L pin': sp['MCP_Pin2x10_L'], 'L bearing 1': sp['MCP_Bearing_4x8x3_L1'], 'L bearing 2': sp['MCP_Bearing_4x8x3_L2'], 'L sleeve 1': sp['MCP_SleeveL_1'], 'L sleeve 2': sp['MCP_SleeveL_2']}
print('static pairs at q=0 (interference volume mm3 / min distance mm):')
for a, b in [('cheekR', 'cheekU'), ('cheekR', 'AA pin'), ('cheekR', 'H pin'), ('cheekU', 'H pin'), ('H sleeve', 'H bearing'), ('H sleeve', 'H pin'), ('H ring', 'cheekR')]:
    print(f'   {a:10s} x {b:10s} {occ.interference(fixed[a], fixed[b]):8.3f}  {occ.mindist(fixed[a], fixed[b]):.3f}')
print('   arc x L pin', round(occ.interference(arc, moving['L pin']), 3), '| gear x cheekR', round(occ.interference(g, cr), 3), '| gear x cheekU', round(occ.interference(g, cu), 3))
print('sweep (interference volumes; clearance = min distance):')
for q in (-5, 0, 10, 20, 30, 40, 50, 60, 68):
    mv = {k: occ.rot_x(v, -q) for k, v in moving.items()}; gq = occ.rot_x(g, q * F.ZS / F.ZG, F.GC[0], F.GC[1])
    worst = []; 
    for km, vm in mv.items():
        for kf, vf in fixed.items():
            iv = occ.interference(vm, vf)
            if iv > 1e-3: worst.append((km, kf, round(iv, 2)))
    ig = occ.interference(mv['arc'], gq)
    d_arc_cr = occ.mindist(mv['arc'], cr); d_arc_cu = occ.mindist(mv['arc'], cu); d_b1 = occ.mindist(mv['L bearing 1'], cr); d_h = occ.mindist(mv['arc'], fixed['H bearing'])
    print(f'  q={q:4d}: collisions {worst if worst else "none"} | gear x rack overlap {ig:.3f} | arc-cheekR {d_arc_cr:.2f} arc-cheekU {d_arc_cu:.2f} L-bearing in pocket {d_b1:.2f} H-bearing in slot {d_h:.2f}')
print('time', round(time.time() - t, 1))
