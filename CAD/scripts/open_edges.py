import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, trimesh, asm, geo, json
def open_edges(V, F):
    m = trimesh.Trimesh(V, F, process=True); m.merge_vertices(digits_vertex=4)
    e = np.sort(m.edges, axis=1); u, c = np.unique(e, axis=0, return_counts=True); b = u[c == 1]
    return m, b
FR = json.load(open('/home/claude/cache/v7/frames.json'))
for n, v7i, f in (('Middle_Proximal', 33, 'middle'), ('Ring_Proximal', 51, 'ring'), ('Index_Proximal', 31, 'index'), ('Pinky_Proximal', 35, 'pinky'), ('Palm', 0, None), ('Index_Distal', 38, None), ('Middle_Distal', 39, None), ('Ring_Distal', 53, None), ('Pinky_Distal', 37, None)):
    out = []
    for lab, (V, F) in (('v9', asm.mesh_of(asm.new_parts()[n]) if n in asm.new_parts() else geo.mesh_cached('v7', v7i)), ('v7', geo.mesh_cached('v7', v7i))):
        m, b = open_edges(V, F); P = m.vertices[b].reshape(-1, 3) if len(b) else np.zeros((0, 3))
        s = f'{lab}: {len(b)} open edges'
        if len(b):
            L = np.linalg.norm(m.vertices[b[:, 0]] - m.vertices[b[:, 1]], axis=1).sum()
            if f: d = FR[f]; q = P - np.array(d['O']); u = q @ np.array(d['U']); e = q @ np.array(d['E']); r = np.hypot(u, e); s += f' (total length {L:.2f} mm, radius from the knuckle axis {r.min():.1f}..{r.max():.1f})'
            else: s += f' (total length {L:.2f} mm, z {P[:, 2].min():.1f}..{P[:, 2].max():.1f}, y {P[:, 1].min():.1f}..{P[:, 1].max():.1f})'
        out.append(s)
    print(f'{n:16s} ' + ' | '.join(out))
