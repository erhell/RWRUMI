"""Offscreen VTK renderer for lists of (V,F,color[,opacity])."""
import numpy as np, vtk
from vtk.util import numpy_support as ns
def polydata(V, F):
    pd = vtk.vtkPolyData(); pts = vtk.vtkPoints(); pts.SetData(ns.numpy_to_vtk(np.ascontiguousarray(V, dtype=np.float64), deep=True)); pd.SetPoints(pts)
    cells = np.hstack([np.full((len(F), 1), 3, dtype=np.int64), F.astype(np.int64)]).ravel()
    ca = vtk.vtkCellArray(); ca.SetCells(len(F), ns.numpy_to_vtkIdTypeArray(cells, deep=True)); pd.SetPolys(ca)
    return pd
def render(items, fn, pos=None, focal=None, up=(0, 0, 1), size=(1400, 1000), parallel=True, scale=None, bg=(1, 1, 1), edges=False, view_angle=None, lines=None):
    ren = vtk.vtkRenderer(); ren.SetBackground(*bg)
    rw = vtk.vtkRenderWindow(); rw.SetOffScreenRendering(1); rw.AddRenderer(ren); rw.SetSize(*size); rw.SetMultiSamples(0)
    for it in items:
        V, F, col = it[:3]; op = it[3] if len(it) > 3 else 1.0
        n = vtk.vtkPolyDataNormals(); n.SetInputData(polydata(V, F)); n.SetFeatureAngle(40); n.SplittingOn(); n.ConsistencyOff(); n.Update()
        m = vtk.vtkPolyDataMapper(); m.SetInputConnection(n.GetOutputPort())
        a = vtk.vtkActor(); a.SetMapper(m); p = a.GetProperty(); p.SetColor(*col); p.SetOpacity(op); p.SetAmbient(0.25); p.SetDiffuse(0.75); p.SetSpecular(0.1)
        if edges:
            fe = vtk.vtkFeatureEdges(); fe.SetInputData(polydata(V, F)); fe.BoundaryEdgesOff(); fe.FeatureEdgesOn(); fe.SetFeatureAngle(35); fe.ColoringOff(); fe.Update()
            me = vtk.vtkPolyDataMapper(); me.SetInputConnection(fe.GetOutputPort()); ae = vtk.vtkActor(); ae.SetMapper(me); ae.GetProperty().SetColor(0.1, 0.1, 0.1); ae.GetProperty().SetLineWidth(1); ren.AddActor(ae)
        ren.AddActor(a)
    for L in (lines or []):
        p0, p1, col, w = L
        ls = vtk.vtkLineSource(); ls.SetPoint1(*p0); ls.SetPoint2(*p1); ls.Update()
        m = vtk.vtkPolyDataMapper(); m.SetInputConnection(ls.GetOutputPort()); a = vtk.vtkActor(); a.SetMapper(m); a.GetProperty().SetColor(*col); a.GetProperty().SetLineWidth(w); ren.AddActor(a)
    cam = ren.GetActiveCamera()
    if parallel: cam.ParallelProjectionOn()
    ren.ResetCamera()
    if pos is not None:
        cam.SetPosition(*pos); cam.SetFocalPoint(*focal); cam.SetViewUp(*up)
        ren.ResetCameraClippingRange()
        if parallel and scale is None: ren.ResetCamera(); 
    if scale is not None: cam.SetParallelScale(scale)
    if view_angle: cam.SetViewAngle(view_angle)
    ren.ResetCameraClippingRange()
    rw.Render()
    w2 = vtk.vtkWindowToImageFilter(); w2.SetInput(rw); w2.Update()
    wr = vtk.vtkPNGWriter(); wr.SetFileName(fn); wr.SetInputConnection(w2.GetOutputPort()); wr.Write()
