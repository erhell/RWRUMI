"""Load a STEP with names (XCAF), flatten to leaf shapes with world transforms, cache as BREP + json."""
import sys, os, json, time, faulthandler; faulthandler.enable()
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.TDF import TDF_Label
from OCP.collections import Sequence_TDF_Label as TDF_LabelSequence
from OCP.TDataStd import TDataStd_Name
from OCP.TopLoc import TopLoc_Location
from OCP.BRepTools import BRepTools
from OCP.Bnd import Bnd_Box
from OCP.BRepBndLib import BRepBndLib
from OCP.TopAbs import TopAbs_SOLID, TopAbs_FACE
from OCP.TopExp import TopExp_Explorer
from OCP.IFSelect import IFSelect_RetDone

def name_of(lab):
    if lab.IsNull() or not lab.IsAttribute(TDataStd_Name.GetID_s()):
        return ''
    n = TDataStd_Name()
    if lab.FindAttribute(TDataStd_Name.GetID_s(), n):
        return n.Get().ToExtString()
    return ''

def count(shape, kind):
    e = TopExp_Explorer(shape, kind); c = 0
    while e.More(): c += 1; e.Next()
    return c

def load(path, tag):
    t = time.time()
    from OCP.XCAFApp import XCAFApp_Application
    app = XCAFApp_Application.GetApplication_s()
    doc = TDocStd_Document(TCollection_ExtendedString('XmlXCAF'))
    app.InitDocument(doc)
    r = STEPCAFControl_Reader(); r.SetNameMode(True)
    assert r.ReadFile(path) == IFSelect_RetDone
    r.Transfer(doc)
    st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
    roots = TDF_LabelSequence(); st.GetFreeShapes(roots)
    out = []
    def walk(lab, loc, pathn):
        nm = name_of(lab)
        if st.IsReference_s(lab):
            ref = TDF_Label(); st.GetReferredShape_s(lab, ref)
            l2 = loc.Multiplied(st.GetLocation_s(lab))
            rn = name_of(ref)
            walk(ref, l2, pathn + [nm or rn])
            return
        if st.IsAssembly_s(lab):
            ch = TDF_LabelSequence(); st.GetComponents_s(lab, ch)
            for i in range(1, ch.Length() + 1):
                walk(ch.Value(i), loc, pathn + ([nm] if nm and (not pathn or pathn[-1] != nm) else []))
            return
        sh = st.GetShape_s(lab).Moved(loc)
        # sub-shape labels (bodies) with names
        subs = TDF_LabelSequence(); st.GetSubShapes_s(lab, subs)
        named = []
        for i in range(1, subs.Length() + 1):
            sl = subs.Value(i); ss = st.GetShape_s(sl)
            if ss.ShapeType() == TopAbs_SOLID:
                named.append((name_of(sl), ss.Moved(loc)))
        if named and count(sh, TopAbs_SOLID) == len(named):
            for n2, s2 in named:
                out.append((pathn + [nm, n2], s2))
        else:
            out.append((pathn + [nm], sh))
    for i in range(1, roots.Length() + 1):
        walk(roots.Value(i), TopLoc_Location(), [])
    os.makedirs(f'/home/claude/cache/{tag}', exist_ok=True)
    meta = []
    for i, (pn, sh) in enumerate(out):
        b = Bnd_Box(); BRepBndLib.Add_s(sh, b)
        bb = (b.CornerMin().X(), b.CornerMin().Y(), b.CornerMin().Z(), b.CornerMax().X(), b.CornerMax().Y(), b.CornerMax().Z()) if not b.IsVoid() else (0,)*6
        fn = f'/home/claude/cache/{tag}/{i:03d}.brep'
        BRepTools.Write_s(sh, fn)
        meta.append(dict(i=i, path=[p for p in pn if p], bbox=[round(v, 3) for v in bb], nsolid=count(sh, TopAbs_SOLID), nface=count(sh, TopAbs_FACE), file=fn))
    json.dump(meta, open(f'/home/claude/cache/{tag}/meta.json', 'w'), indent=0)
    print(tag, 'leafs', len(meta), 'time', round(time.time() - t, 1))
    return meta

if __name__ == '__main__':
    m = load(sys.argv[1], sys.argv[2])
    for d in m:
        print(d['i'], ' / '.join(d['path'])[:90], d['nsolid'], d['nface'], d['bbox'])
