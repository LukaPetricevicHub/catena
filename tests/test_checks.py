"""The CAD checks themselves, tried on small shapes whose answer is known (spec section 11)."""

from __future__ import annotations

import math

import cadquery as cq
import numpy as np
import pytest

from cad import checks
from cad.checks import Listed


def box(x: float, y: float, z: float, at=(0.0, 0.0, 0.0)):
    return cq.Workplane("XY").box(x, y, z, centered=False).translate(at)


def cup(wall: float):
    """A 20 mm cube with a pocket from the top that leaves side walls and a floor of `wall`."""
    pocket = box(20 - 2 * wall, 20 - 2 * wall, 20, at=(wall, wall, wall))
    return box(20, 20, 20).cut(pocket).val()


def mushroom():
    """A 4 mm stem under a 20 mm cap: the cap hangs over flat."""
    return box(4, 4, 10, at=(8, 8, 0)).union(box(20, 20, 3, at=(0, 0, 10))).val()


def test_mesh_normals_point_out_of_the_solid():
    v, t = checks.mesh(box(10, 20, 30).val())
    a, b, c = v[t[:, 0]], v[t[:, 1]], v[t[:, 2]]
    volume = (a * np.cross(b, c)).sum() / 6.0
    assert volume == pytest.approx(6000.0, rel=1e-6)


def test_a_part_larger_than_the_bed_is_caught():
    assert checks.bed_overflow_mm(box(170, 100, 5).val(), (180.0, 180.0)) <= 0
    assert checks.bed_overflow_mm(box(100, 185, 5).val(), (180.0, 180.0)) == pytest.approx(5.0)


def test_a_wall_thinner_than_the_limit_is_caught_and_a_thicker_one_is_not():
    thin = checks.thin_walls(cup(0.8), 1.2)
    assert thin and thin[0].value == pytest.approx(0.8, abs=0.05)
    assert checks.thin_walls(cup(1.2), 1.2) == []
    assert checks.thin_walls(cup(1.6), 1.2) == []


def test_two_holes_too_close_together_are_a_thin_wall():
    def plate(gap: float):
        holes = cq.Workplane("XY").pushPoints([(8, 10), (8 + 3.2 + gap, 10)]).circle(1.6)
        return box(30, 20, 4).cut(holes.extrude(4)).val()

    close = checks.thin_walls(plate(0.7), 1.2)
    assert close and close[0].value == pytest.approx(0.7, abs=0.06)
    assert checks.thin_walls(plate(1.5), 1.2) == []


def test_a_sharp_edge_is_not_a_thin_wall():
    wedge = cq.Workplane("XZ").polyline([(0, 0), (20, 0), (0, 10)]).close().extrude(20).val()
    assert checks.thin_walls(wedge, 1.2) == []


def test_a_flat_overhang_is_caught_unless_it_is_listed():
    found = checks.unlisted_overhang(mushroom(), 50.0)
    assert found is not None
    assert found.value == pytest.approx(20 * 20 - 4 * 4, rel=0.01)
    support = Listed("support", (0, 0, 9.9), (20, 20, 10.1), "under the cap")
    assert checks.unlisted_overhang(mushroom(), 50.0, (support,)) is None


def test_a_listed_box_does_not_excuse_an_overhang_outside_it():
    half = Listed("support", (0, 0, 9.9), (10, 20, 10.1), "under half the cap")
    found = checks.unlisted_overhang(mushroom(), 50.0, (half,))
    assert found is not None and found.value > 150


def test_overhang_limit_is_measured_from_the_vertical():
    def cone(angle_from_vertical_deg: float):
        top = 5 + 10 * math.tan(math.radians(angle_from_vertical_deg))
        return cq.Solid.makeCone(5, top, 10)

    assert checks.unlisted_overhang(cone(45), 50.0) is None
    assert checks.unlisted_overhang(cone(55), 50.0) is not None
    assert checks.unlisted_overhang(box(10, 10, 10).val(), 50.0) is None


def test_overlap_is_the_shared_volume_and_touching_parts_share_none():
    a = box(10, 10, 10).val()
    assert checks.overlap_mm3(a, box(10, 10, 10, at=(8, 0, 0)).val()) == pytest.approx(200.0)
    assert checks.overlap_mm3(a, box(10, 10, 10, at=(10, 0, 0)).val()) == pytest.approx(0.0)
    assert checks.overlap_mm3(a, box(10, 10, 10, at=(30, 0, 0)).val()) == 0.0


def test_placing_turns_about_the_vertical_axis_then_moves():
    bb = checks.placed(box(10, 2, 2).val(), 90.0, (80.0, 0.0, 0.0)).BoundingBox()
    assert (bb.xmin, bb.xmax) == pytest.approx((78.0, 80.0), abs=1e-6)
    assert (bb.ymin, bb.ymax) == pytest.approx((0.0, 10.0), abs=1e-6)
