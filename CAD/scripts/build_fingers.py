import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, json, os, time, occ, geo, finger9 as F
OUT = '/home/claude/cache/v9'; os.makedirs(OUT, exist_ok=True)
NAME = {'index': 'Index', 'middle': 'Middle', 'ring': 'Ring', 'pinky': 'Pinky'}
def build(f, manifest):
    t = time.time(); T = F.world(f); ids = F.FR[f]['ids']; N = NAME[f]
    prox = geo.brep('v7', ids['prox'])
    cutter = occ.move(F.old_bow_cutter(f), T); arc = occ.move(F.link_arc(f), T)
    base = occ.cut(prox, cutter); nb = occ.nsub(base)
    fz = occ.fuse(occ.biggest(base), arc); new = occ.unify(fz)
    if not occ.valid(new): new = fz; print('   (kept without face merging)')
    print(f'{f}: v7 proximal vol {occ.volume(prox):.1f}; after bow removal {occ.volume(base):.1f} ({nb} solids); with new arc {occ.volume(new):.1f}, solids {occ.nsub(new)}, valid {occ.valid(new)}')
    parts = {f'{N}_Proximal': new, f'{N}_MCP_CheekRadial': occ.move(F.cheek_radial(f), T), f'{N}_MCP_CheekUlnar': occ.move(F.cheek_ulnar(f), T), f'{N}_MCP_Gear': occ.move(F.gear(f), T)}
    for k, v in {**F.small_parts(f), **F.aa_parts(f)}.items(): parts[f'{N}_{k}'] = occ.move(v, T)
    for k, v in F.sensors(f).items(): parts[f'{N}_{k}'] = v
    for k, v in parts.items():
        occ.save(v, f'{OUT}/{k}.brep'); manifest[k] = dict(finger=f, valid=bool(occ.valid(v)), vol=occ.volume(v), solids=occ.nsub(v))
        if not occ.valid(v) or occ.nsub(v) != 1: print('   !!', k, 'valid', occ.valid(v), 'solids', occ.nsub(v))
    print(f'   {len(parts)} parts, {time.time() - t:.1f}s')
if __name__ == '__main__':
    man = {}
    for f in (sys.argv[1:] or ['index', 'middle', 'ring', 'pinky']): build(f, man)
    old = json.load(open(f'{OUT}/manifest.json')) if os.path.exists(f'{OUT}/manifest.json') else {}
    old.update(man); json.dump(old, open(f'{OUT}/manifest.json', 'w'), indent=1)
