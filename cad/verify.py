"""The checks `make cad` runs on the robot's parts (docs/SPEC.md section 11)."""

from __future__ import annotations

import math
from collections.abc import Callable
from dataclasses import dataclass

import cadquery as cq
import numpy as np

from cad import checks, coupons, parts
from cad.parts import Dims, Part

VOLUME_TOLERANCE_MM3 = 0.05
YAW_STEP_DEG = 10
# Pairs that are meant to press into each other.
PRESS_FITS = {frozenset(("wheel_hub", "motor")), frozenset(("wheel_hub", "tyre"))}


@dataclass(frozen=True)
class Result:
    name: str
    ok: bool
    detail: str


class Model:
    """Parts built once and shared by all checks."""

    def __init__(self, d: Dims | None = None) -> None:
        self.d = d or Dims()
        self.printed = parts.printed_parts(self.d)
        self.coupons = coupons.coupons(self.d)
        self.robot = parts.robot(self.d)
        self.limits = self.d.c["print"]

    def printable(self) -> list[Part]:
        return self.printed + self.coupons

    def part(self, name: str) -> Part:
        return next(p for p in self.printable() if p.name == name)

    def solid(self, name: str) -> cq.Shape:
        return next(s for group in self.robot.values() for n, s in group if n == name)

    def coupon_plate(self) -> cq.Shape:
        extra = (self.part("bearing_retainer").print_shape, self.part("wheel_hub").print_shape)
        return coupons.plate(self.d, extra)


def _kind(name: str) -> str:
    return name.removesuffix("_b")


def _worst_overlap(a: list[tuple[str, cq.Shape]], b: list[tuple[str, cq.Shape]]):
    worst = (0.0, "", "")
    for na, sa in a:
        for nb, sb in b:
            if frozenset((_kind(na), _kind(nb))) in PRESS_FITS:
                continue
            v = checks.overlap_mm3(sa, sb)
            if v > worst[0]:
                worst = (v, na, nb)
    return worst


def every_part_fits_the_bed(m: Model) -> Result:
    bed = tuple(m.limits["bed"])
    over = [(checks.bed_overflow_mm(p.print_shape, bed), p.name) for p in m.printable()]
    over.append((checks.bed_overflow_mm(m.coupon_plate(), bed), "coupon plate"))
    worst = max(over)
    return Result("every part and the coupon plate fit the bed", worst[0] <= 0,
                  f"{len(over)} items on {bed[0]:.0f} x {bed[1]:.0f} mm; "
                  f"tightest is {worst[1]} with {-worst[0]:.1f} mm to spare")  # fmt: skip


def no_wall_is_thinner_than_the_minimum(m: Model) -> Result:
    bad = []
    for p in m.printable():
        found = checks.thin_walls(p.print_shape, m.limits["min_wall"])
        if found:
            bad.append(f"{p.name}: {found[0]} mm ({len(found)} places)")
    return Result(f"no wall is thinner than {m.limits['min_wall']} mm", not bad,
                  "; ".join(bad) or f"{len(m.printable())} parts probed")  # fmt: skip


def no_overhang_without_a_listed_support(m: Model) -> Result:
    bad = []
    listed = 0
    for p in m.printable():
        listed += len(p.listed)
        found = checks.unlisted_overhang(p.print_shape, m.limits["max_overhang"], p.listed)
        if found:
            bad.append(f"{p.name}: {found} mm^2")
        for box in p.listed:
            if box.kind == "bridge" and not 0 < box.span_mm <= m.limits["max_bridge"]:
                bad.append(f"{p.name}: bridge of {box.span_mm} mm ({box.why})")
    return Result(f"no overhang above {m.limits['max_overhang']:.0f} deg without a listed support",
                  not bad, "; ".join(bad) or f"{listed} listed supports and bridges")  # fmt: skip


def parts_of_one_robot_do_not_overlap(m: Model) -> Result:
    solids = m.robot["base"] + m.robot["ring"]
    worst = (0.0, "", "")
    for i, a in enumerate(solids):
        found = _worst_overlap([a], solids[i + 1 :])
        worst = max(worst, found)
    return Result("the parts of one robot do not run into each other",
                  worst[0] < VOLUME_TOLERANCE_MM3,
                  f"{len(solids)} solids; largest shared volume {worst[0]:.3f} mm^3 "
                  f"{worst[1]} {worst[2]}".rstrip())  # fmt: skip


def base_turns_a_full_circle_inside_its_ring(m: Model) -> Result:
    worst = (0.0, "", "", 0)
    for yaw in range(0, 360, YAW_STEP_DEG):
        ring = [(n, checks.placed(s, yaw)) for n, s in m.robot["ring"]]
        found = _worst_overlap(ring, m.robot["base"])
        worst = max(worst, (*found, yaw))
    radius = max(float(np.hypot(*checks.mesh(s)[0][:, :2].T).max()) for _, s in m.robot["base"])
    ok = worst[0] < VOLUME_TOLERANCE_MM3 and radius <= m.d.base_r + 1e-3
    return Result("a base turns a full circle inside its ring", ok,
                  f"largest shared volume {worst[0]:.3f} mm^3 over {360 // YAW_STEP_DEG} headings; "
                  f"base stays within radius {radius:.2f} of {m.d.base_r:.2f} mm")  # fmt: skip


def linked_pair(m: Model, yaw_deg: float, distance: float | None = None):
    """Robot A at the origin with its tongue in robot B, whose ring is turned by yaw_deg."""
    at = (m.d.pitch if distance is None else distance, 0.0, 0.0)
    a = m.robot["base"] + m.robot["ring"]
    b = [(n, checks.placed(s, 0.0, at)) for n, s in m.robot["base"]]
    b += [(n, checks.placed(s, yaw_deg, at)) for n, s in m.robot["ring"]]
    return a, b


def linked_robots_do_not_intersect(m: Model) -> Result:
    limit = int(m.d.yaw_deg)
    worst = (0.0, "", "", 0)
    yaws = range(-limit, limit + 1, YAW_STEP_DEG)
    for yaw in yaws:
        worst = max(worst, (*_worst_overlap(*linked_pair(m, yaw)), yaw))
    detail = f"largest shared volume {worst[0]:.3f} mm^3 over {len(yaws)} yaw angles"
    if worst[0] >= VOLUME_TOLERANCE_MM3:
        detail += f": {worst[1]} with {worst[2]} at {worst[3]} deg"
    return Result(f"two linked robots do not intersect from -{limit} to +{limit} deg",
                  worst[0] < VOLUME_TOLERANCE_MM3, detail)  # fmt: skip


def linked_centre_distance_is_the_pitch(m: Model) -> Result:
    d = m.d
    tongue = [("ring_lower", m.solid("ring_lower"))]
    hub = m.solid("hub_washers")

    def shared(distance: float) -> float:
        return _worst_overlap(tongue, [("hub_washers", checks.placed(hub, 0, (distance, 0, 0)))])[0]

    tip = float(checks.extent(m.solid("ring_lower"))[1][0])
    engaged = tip - (d.pitch - d.plate_r)
    want = d.cfg["geometry"]["tongue"]["engagement"]
    ok = (
        abs(tip + d.hub_r - d.pitch) < 1e-3
        and shared(d.pitch) < VOLUME_TOLERANCE_MM3
        and shared(d.pitch - 0.2) > VOLUME_TOLERANCE_MM3
        and abs(engaged - want) < 1e-3
    )
    return Result("the linked centre distance equals the pitch", ok,
                  f"tip at {tip:.2f} + hub radius {d.hub_r:.2f} = {tip + d.hub_r:.2f} mm; the tip "
                  f"touches the hub at the pitch and cuts into it 0.2 mm closer; "
                  f"{engaged:.1f} mm of tongue between the neighbour's plates")  # fmt: skip


def heights_match_the_spec(m: Model) -> Result:
    d = m.d
    g = d.cfg["geometry"]

    def low(name: str) -> float:
        return float(checks.extent(m.solid(name))[0][2])

    def high(name: str, axis: int = 2) -> float:
        return float(checks.extent(m.solid(name))[1][axis])

    measured = {
        "ring top": (high("ring_upper"), g["overall_height"]),
        "tyre on the table": (low("tyre"), 0.0),
        "floor": (low("chassis"), g["base"]["ground_clearance"]),
        "skid": (low("skid"), d.c["skid"]["clearance"]),
        "base top": (low("lid") + d.c["shell"]["lid"], d.z_base_top),
        "ring above the base": (low("ring_lower"), d.z_ring),
        "plate radius": (high("ring_upper", 0), g["ring"]["plate_diameter"] / 2),
    }
    tol = checks.MESH_TOLERANCE_MM  # round parts are measured on their mesh
    bad = [f"{k}: {a:.2f} instead of {b:.2f}" for k, (a, b) in measured.items() if abs(a - b) > tol]
    nose = d.c["nose"]["height"] + d.z_floor - d.c["skid"]["clearance"]
    droop = d.cfg["bridge"]["expected_tip_droop"]
    if nose < droop:
        bad.append(f"nose rises {nose:.1f} mm, less than the {droop:.1f} mm droop")
    if d.motor_face_y > d.y_well:
        bad.append(f"motors reach y = {d.motor_face_y:.1f}, past the wheel well at {d.y_well:.1f}")
    return Result("heights, clearances and the motor bay match the config", not bad,
                  "; ".join(bad) or f"overall {g['overall_height']:.0f} mm, floor "
                  f"{g['base']['ground_clearance']:.0f} mm, nose covers {nose:.1f} mm of "
                  f"{droop:.1f} mm droop")  # fmt: skip


CHECKS: tuple[Callable[[Model], Result], ...] = (
    every_part_fits_the_bed,
    no_wall_is_thinner_than_the_minimum,
    no_overhang_without_a_listed_support,
    linked_robots_do_not_intersect,
    base_turns_a_full_circle_inside_its_ring,
    linked_centre_distance_is_the_pitch,
    parts_of_one_robot_do_not_overlap,
    heights_match_the_spec,
)


def mass_report(m: Model) -> list[tuple[str, float, float]]:
    """(group, grams of printed parts if printed solid, grams of the bought parts modelled)."""
    density = m.limits["density"]
    out = []
    for group in ("base", "ring"):
        printed = sum(
            p.print_shape.Volume() / 1000.0 * density[p.material] * p.count
            for p in m.printed if p.group == group
        )  # fmt: skip
        out.append((group, printed, 0.0))
    w = m.d.c["hub_washer"]
    steel = math.pi * (m.d.hub_r**2 - (w["bore"] / 2) ** 2) * w["thickness"] * w["count"] * 7.85e-3
    out[1] = ("ring", out[1][1], steel)
    return out
