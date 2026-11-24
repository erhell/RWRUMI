"""Check every part and write the named STEP assembly Glove_v9.step."""
import sys; sys.path.insert(0, '/home/claude/scripts')
import numpy as np, json, trimesh, asm, geo, occ
from OCP.BRepCheck import BRepCheck_Analyzer
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFApp import XCAFApp_Application
from OCP.XCAFDoc import XCAFDoc_DocumentTool, XCAFDoc_ColorSurf, XCAFDoc_ColorGen
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.Quantity import Quantity_Color, Quantity_TOC_RGB
from OCP.STEPCAFControl import STEPCAFControl_Writer
from OCP.STEPControl import STEPControl_AsIs
from OCP.Interface import Interface_Static
from OCP.IFSelect import IFSelect_RetDone
OUT = sys.argv[1] if len(sys.argv) > 1 else '/mnt/user-data/outputs/Glove_v9.step'
parts = {n: occ.load(fn) for n, fn in asm.new_parts().items()}
for i, n in asm.KEEP_V7.items(): parts[n] = geo.brep('v7', i)
def group(n):
    for g in ('Index', 'Middle', 'Ring', 'Pinky', 'Thumb', 'Camera'):
        if n.startswith(g + '_'): return g
    return 'Palm' if n.startswith('Palm') else 'Electronics'
PRINT = lambda n: not any(k in (n.split('_', 1)[1] if '_' in n else n) for k in ('3382H', 'Bearing', 'Pin', 'dummy'))
rows = []; bad = []
for n, s in sorted(parts.items()):
    ok = BRepCheck_Analyzer(s).IsValid(); vol = occ.volume(s)
    V, F = (asm.mesh_of(asm.new_parts()[n]) if n in asm.new_parts() else geo.mesh_cached('v7', [i for i, m in asm.KEEP_V7.items() if m == n][0]))
    m = trimesh.Trimesh(V, F, process=True); m.merge_vertices(digits_vertex=5); wt = m.is_watertight
    rows.append((n, ok, vol, wt, PRINT(n)))
    if PRINT(n) and (not ok or not wt): bad.append((n, ok, wt))
print(len(rows), 'parts;', sum(r[4] for r in rows), 'printed,', sum(not r[4] for r in rows), 'bought / dummy')
print('printed parts not valid or mesh not closed:', bad if bad else 'none')
tot = {}
for n, ok, vol, wt, p in rows:
    if p: tot[group(n)] = tot.get(group(n), 0) + vol
print('printed volume by group (cm3):', {k: round(v / 1000, 1) for k, v in tot.items()}, 'total', round(sum(tot.values()) / 1000, 1))
json.dump([dict(name=n, valid=bool(ok), volume=round(vol, 1), watertight=bool(wt), printed=bool(p), group=group(n)) for n, ok, vol, wt, p in rows], open('/home/claude/cache/v9/partlist.json', 'w'), indent=0)
# ---- STEP
app = XCAFApp_Application.GetApplication_s(); doc = TDocStd_Document(TCollection_ExtendedString('XmlXCAF')); app.InitDocument(doc)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main()); ct = XCAFDoc_DocumentTool.ColorTool_s(doc.Main())
def setname(lab, s): TDataStd_Name.Set_s(lab, TCollection_ExtendedString(s))
root = st.NewShape(); setname(root, 'Glove_v9'); subs = {}
for g in ('Palm', 'Index', 'Middle', 'Ring', 'Pinky', 'Thumb', 'Camera', 'Electronics'):
    subs[g] = st.NewShape(); setname(subs[g], g)
for n, s in sorted(parts.items()):
    lab = st.AddShape(s, False); setname(lab, n); c = asm.color(n); col = Quantity_Color(c[0], c[1], c[2], Quantity_TOC_RGB); ct.SetColor(lab, col, XCAFDoc_ColorGen)
    comp = st.AddComponent(subs[group(n)], lab, TopLoc_Location()); setname(comp, n)
for g, l in subs.items(): comp = st.AddComponent(root, l, TopLoc_Location()); setname(comp, g)
st.UpdateAssemblies()
Interface_Static.SetCVal_s('write.step.schema', 'AP214IS'); Interface_Static.SetCVal_s('write.step.product.name', 'Glove_v9')
w = STEPCAFControl_Writer(); w.SetNameMode(True); w.SetColorMode(True)
assert w.Transfer(doc, STEPControl_AsIs); assert w.Write(OUT) == IFSelect_RetDone
import os; print('written', OUT, round(os.path.getsize(OUT) / 1e6, 1), 'MB')
