import sys, time, faulthandler; faulthandler.enable()
from OCP.STEPCAFControl import STEPCAFControl_Reader
from OCP.TDocStd import TDocStd_Document
from OCP.TCollection import TCollection_ExtendedString
from OCP.XCAFDoc import XCAFDoc_DocumentTool
from OCP.collections import Sequence_TDF_Label
from OCP.XCAFApp import XCAFApp_Application
t=time.time()
app = XCAFApp_Application.GetApplication_s()
doc = TDocStd_Document(TCollection_ExtendedString('XmlXCAF'))
app.InitDocument(doc)
print('doc ok', flush=True)
r = STEPCAFControl_Reader(); r.SetNameMode(True)
print('read', r.ReadFile(sys.argv[1]), time.time()-t, flush=True)
print('transfer', r.Transfer(doc), time.time()-t, flush=True)
st = XCAFDoc_DocumentTool.ShapeTool_s(doc.Main())
roots = Sequence_TDF_Label(); st.GetFreeShapes(roots)
print('roots', roots.Length(), flush=True)
