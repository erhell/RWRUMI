"""Assembly helpers for v9: part list (new BREPs + unchanged v7 bodies), cached meshes, rendering."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, os, glob, json, occ, geo, rend
V9 = '/home/claude/cache/v9'
KEEP_V7 = {37: 'Pinky_Distal', 38: 'Index_Distal', 39: 'Middle_Distal', 53: 'Ring_Distal', 32: 'Index_3382H_PIP', 34: 'Middle_3382H_PIP', 52: 'Ring_3382H_PIP', 36: 'Pinky_3382H_PIP',
           55: 'Thumb_TP', 57: 'Thumb_PP', 59: 'Thumb_3382H_T1', 60: 'Thumb_3382H_T2', 61: 'Thumb_3382H_T3', 62: 'Thumb_3382H_T4',
           63: 'Thumb_Bearing_4x8x3_T1d', 64: 'Thumb_Bearing_4x8x3_T1d2', 65: 'Thumb_Bearing_4x8x3_T1q', 66: 'Thumb_Bearing_4x8x3_T2a', 67: 'Thumb_Bearing_4x8x3_T2b', 68: 'Thumb_Bearing_4x8x3_T3', 69: 'Thumb_Bearing_4x8x3_T4'}
def color(n):
    t = n.split('_', 1)[1] if '_' in n else n
    if '3382H' in t: return (0.13, 0.13, 0.14)
    if 'Bearing' in t: return (0.85, 0.2, 0.2)
    if 'Pin' in t: return (0.35, 0.36, 0.4)
    if 'Sleeve' in t or 'RingH' in t or 'DTip' in t: return (0.55, 0.85, 0.6)
    if 'Cheek' in t: return (0.97, 0.72, 0.35)
    if 'Gear' in t: return (0.72, 0.8, 0.3)
    if t == 'Proximal': return (0.5, 0.72, 0.95)
    if t == 'Distal': return (0.55, 0.82, 0.62)
    if n.startswith('Thumb_Link'): return (0.95, 0.55, 0.15)
    if n.startswith('Thumb'): return (0.82, 0.64, 0.86)
    if n.startswith('Palm'): return (0.8, 0.81, 0.84)
    if n.startswith('Camera_OAK'): return (0.25, 0.27, 0.3)
    if n.startswith('Camera'): return (0.95, 0.55, 0.15)
    if n.startswith('PCB'): return (0.1, 0.5, 0.25)
    return (0.6, 0.6, 0.6)
def new_parts(exclude=('Palm_stage1',)):
    out = {}
    for fn in sorted(glob.glob(f'{V9}/*.brep')):
        n = os.path.basename(fn)[:-5]
        if n in exclude or n.startswith('_'): continue
        out[n] = fn
    return out
def mesh_of(fn, lin=0.08):
    c = fn[:-5] + f'_m{lin}.npz'
    if os.path.exists(c) and os.path.getmtime(c) > os.path.getmtime(fn):
        d = np.load(c); return d['V'], d['F']
    V, F = geo.mesh(occ.load(fn), lin); np.savez(c, V=V, F=F); return V, F
def all_items(skip=lambda n: False, palm='Palm'):
    items = []; names = []
    for n, fn in new_parts().items():
        if skip(n): continue
        V, F = mesh_of(fn); items.append((V, F, color(n))); names.append(n)
    for i, n in KEEP_V7.items():
        if skip(n): continue
        V, F = geo.mesh_cached('v7', i); items.append((V, F, color(n))); names.append(n)
    return items, names
def shots(tag, views, items, c=(0, 150, 45), scale=110, size=(1300, 1000)):
    c = np.array(c, float); fns = []
    for name, dv, up in views:
        fn = f'/home/claude/out/{tag}_{name}.png'
        rend.render(items, fn, pos=c + 600 * np.array(dv, float), focal=c, up=up, size=size, edges=True, scale=scale); fns.append(fn)
    return fns
def montage(fns, out, cols=3, w=2000):
    from PIL import Image
    ims = [Image.open(f) for f in fns]; W, H = ims[0].size; rows = (len(ims) + cols - 1) // cols
    M = Image.new('RGB', (W * cols, H * rows), 'white')
    for k, im in enumerate(ims): M.paste(im, ((k % cols) * W, (k // cols) * H))
    M.resize((w, int(w * H * rows / (W * cols)))).save(out)
if __name__ == '__main__':
    import shutil
    if os.path.exists(f'{V9}/Palm_stage1.brep') and not os.path.exists(f'{V9}/Palm.brep'): shutil.copy(f'{V9}/Palm_stage1.brep', f'{V9}/Palm.brep')
    items, names = all_items(); print(len(names), 'parts')
    views = [('iso_ru', (-0.8, -0.8, 0.6), (0, 0, 1)), ('iso_uu', (0.8, -0.8, 0.6), (0, 0, 1)), ('front', (0, 1, 0.15), (0, 0, 1)), ('ulnar', (1, 0, 0), (0, 0, 1)), ('top', (0, 0, 1), (0, 1, 0)), ('back', (0, -1, 0.1), (0, 0, 1))]
    montage(shots('asm', views, items), '/home/claude/out/asm_views.png')
