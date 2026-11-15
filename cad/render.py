"""Shaded pictures of the CAD model for docs/media/ (`make render`). Needs a display or macOS."""

from __future__ import annotations

from pathlib import Path

import cadquery as cq
import vtkmodules.vtkRenderingOpenGL2  # noqa: F401  (registers the renderer)
from vtkmodules.vtkCommonCore import vtkPoints
from vtkmodules.vtkCommonDataModel import vtkCellArray, vtkPolyData
from vtkmodules.vtkFiltersCore import vtkPolyDataNormals
from vtkmodules.vtkIOImage import vtkPNGWriter
from vtkmodules.vtkRenderingCore import (
    vtkActor,
    vtkPolyDataMapper,
    vtkRenderer,
    vtkRenderWindow,
    vtkWindowToImageFilter,
)

from cad import checks, verify

MEDIA = Path(__file__).resolve().parent.parent / "docs" / "media"
COLOURS = {
    "chassis": (0.86, 0.86, 0.84), "lid": (0.78, 0.78, 0.76), "board_carrier": (0.3, 0.5, 0.3),
    "skid": (0.95, 0.55, 0.15), "wheel_hub": (0.95, 0.55, 0.15), "tyre": (0.12, 0.12, 0.12),
    "motor": (0.65, 0.67, 0.7), "ring_lower": (0.16, 0.45, 0.7), "ring_upper": (0.3, 0.62, 0.85),
    "bearing_retainer": (0.95, 0.55, 0.15), "hub_washers": (0.7, 0.72, 0.75),
}  # fmt: skip


def _actor(shape: cq.Shape, colour: tuple[float, float, float], opacity: float) -> vtkActor:
    v, t = checks.mesh(shape)
    points = vtkPoints()
    for p in v:
        points.InsertNextPoint(*p)
    cells = vtkCellArray()
    for tri in t:
        cells.InsertNextCell(3, [int(i) for i in tri])
    data = vtkPolyData()
    data.SetPoints(points)
    data.SetPolys(cells)
    normals = vtkPolyDataNormals()
    normals.SetInputData(data)
    normals.SetFeatureAngle(40)
    normals.SplittingOn()
    mapper = vtkPolyDataMapper()
    mapper.SetInputConnection(normals.GetOutputPort())
    actor = vtkActor()
    actor.SetMapper(mapper)
    prop = actor.GetProperty()
    prop.SetColor(*colour)
    prop.SetOpacity(opacity)
    prop.SetSpecular(0.15)
    return actor


def picture(
    solids: list[tuple[str, cq.Shape]],
    path: Path,
    azimuth: float = -35.0,
    elevation: float = 28.0,
    size: tuple[int, int] = (1400, 900),
    see_through: tuple[str, ...] = (),
) -> Path:
    renderer = vtkRenderer()
    renderer.SetBackground(1, 1, 1)
    for name, shape in solids:
        kind = name.removesuffix("_b")
        renderer.AddActor(_actor(shape, COLOURS[kind], 0.35 if kind in see_through else 1.0))
    window = vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetMultiSamples(8)
    window.SetSize(*size)
    window.AddRenderer(renderer)
    camera = renderer.GetActiveCamera()
    camera.SetViewUp(0, 0, 1)
    camera.SetPosition(0, -1, 0)
    camera.SetFocalPoint(0, 0, 0)
    camera.Azimuth(azimuth)
    camera.Elevation(elevation)
    camera.ParallelProjectionOn()
    renderer.ResetCamera()
    camera.Zoom(1.5)
    window.Render()
    grab = vtkWindowToImageFilter()
    grab.SetInput(window)
    grab.Update()
    writer = vtkPNGWriter()
    writer.SetFileName(str(path))
    writer.SetInputConnection(grab.GetOutputPort())
    writer.Write()
    return path


def main() -> None:
    m = verify.Model()
    MEDIA.mkdir(parents=True, exist_ok=True)
    one = m.robot["base"] + m.robot["ring"]
    print(picture(one, MEDIA / "cad_robot.png"))
    a, b = verify.linked_pair(m, 40.0)
    print(picture(a + b, MEDIA / "cad_linked.png", azimuth=-20.0, elevation=35.0,
                  see_through=("ring_upper",)))  # fmt: skip


if __name__ == "__main__":
    main()
