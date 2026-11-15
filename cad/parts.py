"""The parts of one robot in the robot frame (docs/SPEC.md sections 3 and 11).

x forward, y left, z up from the table. The ring is drawn with its tongue along +x.
Every number comes from config/robot.yaml; `Dims` derives the levels the parts share.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import cadquery as cq

from cad.checks import Listed, extent
from sim.config import load_config

Z = cq.Vector(0, 0, 1)


class Dims:
    """Config values and the levels derived from them, all in mm."""

    def __init__(self, cfg: dict[str, Any] | None = None) -> None:
        cfg = cfg or load_config()
        g, c = cfg["geometry"], cfg["cad"]
        ring, tongue, base = g["ring"], g["tongue"], g["base"]
        self.cfg, self.c = cfg, c
        self.pitch: float = g["pitch"]
        self.wall: float = c["shell"]["wall"]
        self.floor: float = c["shell"]["floor"]
        self.overhang_deg: float = c["print"]["max_overhang"]

        self.base_r = base["envelope_diameter"] / 2
        self.z_floor = base["ground_clearance"]
        self.z_base_top = self.z_floor + base["height"]
        self.z_lid = self.z_base_top - c["shell"]["lid"]

        self.plate_r = ring["plate_diameter"] / 2
        self.plate_t = ring["plate_thickness"]
        self.groove = ring["groove_height"]
        self.hub_r = ring["hub_diameter"] / 2
        self.yaw_deg = ring["max_joint_yaw"]
        self.z_top = g["overall_height"]
        self.z_ring = self.z_top - (2 * self.plate_t + self.groove)
        self.z_groove = self.z_ring + self.plate_t
        self.z_groove_top = self.z_groove + self.groove

        self.tongue_t = tongue["thickness"]
        self.tongue_w = tongue["width"]
        self.reach = tongue["reach"]
        self.tip_r = tongue["tip_radius"]
        self.z_tongue = self.z_groove + (self.groove - self.tongue_t) / 2

        self.wheel_r = base["wheel_diameter"] / 2
        self.wheel_w = base["wheel_width"]
        self.track = base["track"]
        self.axle_z = self.wheel_r
        m = c["motor"]
        self.motor_w, self.motor_h = m["section"]
        self.motor_face_y = m["terminals"] + m["length"]
        self.y_well = self.track / 2 - self.wheel_w / 2 - c["wheel"]["side_clearance"]
        self.y_wall = self.y_well - self.wall
        self.z_seat = self.axle_z - self.motor_h / 2
        self.z_carrier = self.axle_z + self.motor_h / 2
        self.well_r = self.wheel_r + c["wheel"]["well_clearance"]
        # Above this half-width the wheel arch would hang over at more than the limit.
        self.well_open_x = self.well_r * math.cos(math.radians(self.overhang_deg - 2.0))

        # Sloped nose: a plane rising from the floor line x = nose_x to the wall.
        self.nose_tan = math.tan(math.radians(90.0 - self.overhang_deg + 3.0))
        self.nose_x = self.base_r - c["nose"]["height"] / self.nose_tan

        b = c["bearing"]
        self.z_flange_top = self.z_lid - b["axial_play"]
        self.z_flange = self.z_flange_top - b["flange_thickness"]
        self.z_spigot_top = self.z_groove_top - 0.2


# ---------------------------------------------------------------- small solid helpers


def W(shape: cq.Shape) -> cq.Workplane:
    return cq.Workplane("XY").newObject([shape])


def cyl(r: float, z0: float, z1: float, x: float = 0.0, y: float = 0.0) -> cq.Workplane:
    return W(cq.Solid.makeCylinder(r, z1 - z0, cq.Vector(x, y, z0), Z))


def rod(r: float, p0: tuple[float, float, float], p1: tuple[float, float, float]) -> cq.Workplane:
    a, b = cq.Vector(*p0), cq.Vector(*p1)
    return W(cq.Solid.makeCylinder(r, (b - a).Length, a, b - a))


def box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> cq.Workplane:
    return W(cq.Solid.makeBox(x1 - x0, y1 - y0, z1 - z0, cq.Vector(x0, y0, z0)))


def prism(points_xy: list[tuple[float, float]], z0: float, z1: float) -> cq.Workplane:
    wire = cq.Wire.makePolygon([cq.Vector(x, y, z0) for x, y in points_xy], close=True)
    return W(cq.Solid.extrudeLinear(wire, [], cq.Vector(0, 0, z1 - z0)))


def xz_prism(points_xz: list[tuple[float, float]], y0: float, y1: float) -> cq.Workplane:
    wire = cq.Wire.makePolygon([cq.Vector(x, y0, z) for x, z in points_xz], close=True)
    return W(cq.Solid.extrudeLinear(wire, [], cq.Vector(0, y1 - y0, 0)))


def revolve(points_rz: list[tuple[float, float]], degrees: float = 360.0) -> cq.Workplane:
    wire = cq.Wire.makePolygon([cq.Vector(r, 0, z) for r, z in points_rz], close=True)
    return W(cq.Solid.revolve(wire, [], degrees, cq.Vector(0, 0, 0), Z))


def countersink(x: float, y: float, z_top: float, screw: dict[str, float]) -> cq.Workplane:
    """Clearance hole with a 90 degree countersink, drilled down from z_top."""
    depth = (screw["head"] - screw["clear"]) / 2
    low = cq.Vector(x, y, z_top - depth)
    cone = cq.Solid.makeCone(screw["clear"] / 2, screw["head"] / 2 + 0.01, depth + 0.01, low, Z)
    return cyl(screw["clear"] / 2, z_top - 50, z_top + 1, x, y).union(W(cone))


def turned(wp: cq.Workplane, degrees: float) -> cq.Workplane:
    return wp.rotate((0, 0, 0), (0, 0, 1), degrees)


def d_profile(diameter: float, flat: float, z0: float, z1: float) -> cq.Workplane:
    """A round bar with one flat, `flat` from the axis toward +x."""
    r = diameter / 2
    return cyl(r, z0, z1).intersect(box(-r - 1, flat, -r - 1, r + 1, z0, z1))


# ---------------------------------------------------------------- base


def chassis(d: Dims) -> cq.Shape:
    c = d.c
    R, zf, zl = d.base_r, d.z_floor, d.z_lid
    ri, top = R - d.wall, zf + d.floor
    t = d.nose_tan
    off = d.wall * math.sqrt(1 + t * t)  # the nose wall, measured vertically
    far = R + 1

    def both(wp: cq.Workplane) -> cq.Workplane:
        return wp.union(turned(wp, 180))

    rise = (far - d.nose_x) * t
    nose_cut = both(
        xz_prism([(d.nose_x, zf), (d.nose_x, zf - 1), (far, zf - 1), (far, zf + rise)], -far, far)
    )
    xa = d.nose_x + (d.floor - off) / t
    nose_wall = both(
        xz_prism([(xa, top), (far, top), (far, zf + rise + off)], -far, far)
    )
    cavity = cyl(ri, top, zl + 1).intersect(box(-far, far, -d.y_wall, d.y_wall, 0, zl + 2))
    body = cyl(R, zf, zl).cut(cavity.cut(nose_wall))

    # Motor cradle: a seat under both motors and a rail on either side.
    half = d.motor_w / 2 + c["fit"]["side"]
    body = body.union(box(-half - d.wall, half + d.wall, -d.y_wall, d.y_wall, top, d.z_seat))
    for s in (-1, 1):
        x0, x1 = sorted((s * half, s * (half + d.wall)))
        body = body.union(box(x0, x1, -d.y_wall, d.y_wall, top, d.z_carrier))

    # Lid screw pillars, front and rear, tied to the wall.
    pil = c["pillar"]
    pr = pil["diameter"] / 2
    pillar = cyl(pr, top, zl, pil["x"], 0).union(box(pil["x"], ri + 0.5 * d.wall, -pr, pr, top, zl))
    body = body.union(both(pillar))
    body = body.cut(both(cyl(c["screw"]["m2"]["pilot"] / 2, zl - 10, zl + 1, pil["x"], 0)))
    body = body.cut(nose_cut)

    # Wheel wells, the opening the gearbox sits in, the window of the wheel tick sensor.
    sens_tall, sens_wide = c["reflective_sensor"]["body"]
    scl = c["reflective_sensor"]["clearance"]
    tx = c["tick_sensor_x"]
    well = rod(d.well_r, (0, d.y_well, d.axle_z), (0, far, d.axle_z)).union(
        box(-d.well_open_x, d.well_open_x, d.y_well, far, d.axle_z, zl + 1)
    )
    well = well.union(box(-half, half, d.y_wall - 0.1, d.y_well + 0.1, d.z_seat, zl + 1))
    well = well.union(
        box(tx - sens_wide / 2 - scl, tx + sens_wide / 2 + scl, d.y_wall - 0.1, d.y_well + 0.1,
            d.axle_z - sens_tall / 2 - scl, d.axle_z + sens_tall / 2 + scl)
    )
    body = body.cut(well).cut(well.mirror("XZ"))

    # Infrared pairs every 60 degrees; each hole is parallel to its pair's direction.
    ir = c["ir"]
    for k in range(6):
        a = math.radians(60 * k)
        ux, uy = math.cos(a), math.sin(a)
        lx, ly = -uy, ux
        if k % 3 == 0:
            shift, spacing = 0.0, ir["axis_pair_spacing"]
        else:
            shift = ir["wheel_side_shift"] * (-1 if k % 3 == 1 else 1)
            spacing = ir["pair_spacing"]
        for s in (shift - spacing / 2, shift + spacing / 2):
            p0 = ((ri - 9) * ux + s * lx, (ri - 9) * uy + s * ly, ir["height"])
            p1 = (far * ux + s * lx, far * uy + s * ly, ir["height"])
            body = body.cut(rod(ir["hole"] / 2, p0, p1))

    # Edge sensor windows through the floor, long side along the wall.
    ex, ey = c["edge_window_xy"]
    window = box(-sens_wide / 2 - scl, sens_wide / 2 + scl, -sens_tall / 2 - scl,
                 sens_tall / 2 + scl, zf - 1, top + 1)
    window = turned(window, math.degrees(math.atan2(ey, ex))).translate((ex, ey, 0))
    body = body.cut(window).cut(window.mirror("XZ"))

    # Slots for the tabs of the two skids.
    sk = c["skid"]
    side = c["fit"]["side"]
    slot = box(sk["tab"][0] - side, sk["tab"][1] + side, -sk["width"] / 2 - side,
               sk["width"] / 2 + side, zf - 1, top + 1)
    return body.cut(both(slot)).val()


def lid(d: Dims) -> cq.Shape:
    c = d.c
    b = c["bearing"]
    far = d.base_r + 1
    body = cyl(d.base_r, d.z_lid, d.z_base_top).union(
        cyl(b["land_diameter"] / 2, d.z_base_top, d.z_ring)
    )
    bore_r = (b["journal_diameter"] + c["fit"]["turning"]) / 2
    body = body.cut(cyl(bore_r, d.z_lid - 1, d.z_ring + 1))
    slot = box(-d.well_open_x, d.well_open_x, d.y_well, far, d.z_lid - 1, d.z_base_top + 1)
    body = body.cut(slot).cut(slot.mirror("XZ"))
    for s in (-1, 1):
        body = body.cut(countersink(s * c["pillar"]["x"], 0, d.z_base_top, c["screw"]["m2"]))
    det = c["detent"]
    hole_r = (det["magnet"][0] + c["fit"]["side"]) / 2
    return body.cut(cyl(hole_r, d.z_lid - 1, d.z_base_top + 1, det["radius"], 0)).val()


def carrier(d: Dims) -> cq.Shape:
    c = d.c
    car, pil = c["carrier"], c["pillar"]
    z0, z1 = d.z_carrier, d.z_carrier + car["thickness"]
    edge = d.y_wall - c["fit"]["static"]
    far = d.base_r + 1
    plate = cyl(d.base_r - d.wall - 0.5, z0, z1).intersect(box(-far, far, -edge, edge, z0, z1))
    gap = pil["diameter"] / 2 + 0.5
    sx, sy = car["slot"]
    for s in (-1, 1):
        plate = plate.cut(cyl(gap, z0 - 1, z1 + 1, s * pil["x"], 0))
        x0, x1 = sorted((s * pil["x"], s * far))
        plate = plate.cut(box(x0, x1, -gap, gap, z0 - 1, z1 + 1))
        x0, x1 = sorted((s * (car["slot_x"] - sx / 2), s * (car["slot_x"] + sx / 2)))
        plate = plate.cut(box(x0, x1, -sy / 2, sy / 2, z0 - 1, z1 + 1))
        plate = plate.union(cyl(car["post_diameter"] / 2, z1, d.z_lid, 0, s * car["post_y"]))
    return plate.val()


def skid_profile(d: Dims) -> list[tuple[float, float]]:
    sk = d.c["skid"]
    zf, low = d.z_floor, sk["clearance"]
    t0, t1 = sk["tab"]
    return [
        (sk["back"], zf), (sk["foot"][0], low), (sk["foot"][1], low), (d.nose_x - 0.4, zf),
        (t1, zf), (t1, zf + d.floor), (t0, zf + d.floor), (t0, zf),
    ]  # fmt: skip


def skid(d: Dims, rear: bool = False) -> cq.Shape:
    w = d.c["skid"]["width"]
    part = xz_prism(skid_profile(d), -w / 2, w / 2)
    return (turned(part, 180) if rear else part).val()


def _wheel_levels(d: Dims) -> tuple[float, float, float, float]:
    wh = d.c["wheel"]
    flange_r = d.wheel_r - wh["tyre_thickness"]
    seat_r = flange_r - wh["groove_depth"]
    z_floor0 = (d.wheel_w - wh["groove_floor"]) / 2
    z_mouth0 = z_floor0 - wh["groove_depth"]  # 45 degree groove walls
    return flange_r, seat_r, z_floor0, z_mouth0


def wheel_hub_print(d: Dims, fit: float | None = None) -> cq.Shape:
    """The hub as printed: axle upright, the striped inner face on top."""
    c = d.c
    wh, shaft = c["wheel"], c["motor"]["shaft"]
    w = d.wheel_w
    flange_r, seat_r, zf0, zm0 = _wheel_levels(d)
    hub = revolve([
        (0, 0), (flange_r, 0), (flange_r, zm0), (seat_r, zf0), (seat_r, w - zf0),
        (flange_r, w - zm0), (flange_r, w), (0, w),
    ])  # fmt: skip
    fit = 0.0 if fit is None else fit
    hub = hub.cut(
        d_profile(shaft["diameter"] + fit, shaft["flat"] + fit - shaft["diameter"] / 2, -1, w + 1)
    )
    r0, r1 = wh["stripe_radii"]
    stripes = d.cfg["drive"]["wheel_ticks_per_rev"]
    step = 360.0 / stripes
    top = w - wh["stripe_depth"]
    stripe = revolve([(r0, top), (r1, top), (r1, w + 1), (r0, w + 1)], step / 2)
    for k in range(stripes):
        hub = hub.cut(turned(stripe, k * step))
    return hub.val()


def tyre_print(d: Dims) -> cq.Shape:
    """Printed tyre with the section of the hub groove; a 24 x 3 O-ring takes the same seat."""
    w = d.wheel_w
    flange_r, seat_r, zf0, zm0 = _wheel_levels(d)
    return revolve([
        (seat_r, zf0), (flange_r, zm0), (d.wheel_r, zm0), (d.wheel_r, w - zm0),
        (flange_r, w - zm0), (seat_r, w - zf0),
    ]).val()  # fmt: skip


def on_axle(d: Dims, printed: cq.Shape, left: bool) -> cq.Shape:
    """A wheel part from its print position onto the left or right axle, stripes inward."""
    out = printed.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 90)
    out = out.translate(cq.Vector(0, d.track / 2 + d.wheel_w / 2, d.axle_z))
    return out if left else out.rotate(cq.Vector(0, 0, 0), Z, 180)


def motor(d: Dims, left: bool) -> cq.Shape:
    """Bought part: N20 gearmotor as a block with its shaft."""
    m = d.c["motor"]
    body = box(-d.motor_w / 2, d.motor_w / 2, m["terminals"], d.motor_face_y,
               d.axle_z - d.motor_h / 2, d.axle_z + d.motor_h / 2)
    shaft = rod(m["shaft"]["diameter"] / 2, (0, d.motor_face_y, d.axle_z),
                (0, d.motor_face_y + m["shaft"]["length"], d.axle_z))
    out = body.union(shaft).val()
    return out if left else out.rotate(cq.Vector(0, 0, 0), Z, 180)


# ---------------------------------------------------------------- ring


def sector_wedge(d: Dims) -> list[tuple[float, float]]:
    """Outline of the solid front sector: what a neighbour's tongue at the yaw limit leaves."""
    a = math.radians(180.0 - d.yaw_deg)
    ca, sa = math.cos(a), math.sin(a)
    apex = (d.tongue_w / 2 + d.c["ring"]["sector_clearance"]) / sa
    reach = 3 * d.plate_r
    return [
        (apex, 0), (apex + reach * ca, -reach * sa), (apex + reach * ca + reach, -reach * sa),
        (apex + reach * ca + reach, reach * sa), (apex + reach * ca, reach * sa),
    ]  # fmt: skip


def tongue_outline(d: Dims, root_x: float, z0: float, z1: float) -> cq.Workplane:
    xc = d.reach - d.tip_r
    half = d.tongue_w / 2
    return box(root_x, xc, -half, half, z0, z1).union(cyl(d.tip_r, z0, z1, xc, 0))


def tip_magnet_pocket(d: Dims, z_top: float, wall: float) -> cq.Workplane:
    magnet = d.cfg["coupling"]["tip_magnet"]
    dia, thick = magnet["diameter"], magnet["thickness"]
    r = (dia + d.c["fit"]["side"]) / 2
    return cyl(r, z_top - thick - d.c["fit"]["side"], z_top + 1, d.reach - wall - r, 0)


def ring_lower(d: Dims) -> cq.Shape:
    c = d.c
    rg, det, b = c["ring"], c["detent"], c["bearing"]
    pt, g = d.plate_t, d.groove
    z_t = pt + (g - d.tongue_t) / 2
    core_r = (c["hub_washer"]["bore"] - rg["core_clearance"]) / 2
    hub_gap = d.hub_r + rg["hub_clearance"]

    body = cyl(d.plate_r, 0, pt).union(cyl(core_r, pt, pt + g))
    sector = prism(sector_wedge(d), pt, pt + g).intersect(cyl(d.plate_r, pt, pt + g))
    body = body.union(sector.cut(cyl(hub_gap, pt - 1, pt + g + 1)))
    body = body.union(tongue_outline(d, hub_gap + 2.0, z_t, z_t + d.tongue_t))
    body = body.cut(tip_magnet_pocket(d, z_t + d.tongue_t, c["tongue"]["magnet_wall"]))

    # Detent magnet, let in from below so that it sits `gap` above the magnet in the lid.
    depth = det["magnet"][1] + c["fit"]["side"] + det["gap"] - (d.z_ring - d.z_base_top)
    body = body.cut(cyl((det["magnet"][0] + c["fit"]["side"]) / 2, -1, depth, det["radius"], 0))
    body = body.cut(
        d_profile(b["spigot_diameter"] + c["fit"]["static"],
                  b["spigot_flat"] + c["fit"]["static"] / 2, -1, pt + g + 1)
    )
    sx, sy = rg["screw_xy"]
    for s in (-1, 1):
        body = body.cut(cyl(c["screw"]["m2"]["pilot"] / 2, pt + g - 6, pt + g + 1, sx, s * sy))
    return body.translate((0, 0, d.z_ring)).val()


def ring_upper(d: Dims) -> cq.Shape:
    c = d.c
    body = cyl(d.plate_r, d.z_groove_top, d.z_top)
    body = body.cut(countersink(0, 0, d.z_top, c["screw"]["m3"]))
    sx, sy = c["ring"]["screw_xy"]
    for s in (-1, 1):
        body = body.cut(countersink(sx, s * sy, d.z_top, c["screw"]["m2"]))
    return body.val()


def retainer(d: Dims) -> cq.Shape:
    """Bearing retainer: a pin screwed to the ring from below the lid; holds the angle magnet."""
    c = d.c
    b = c["bearing"]
    pin = cyl(b["flange_diameter"] / 2, d.z_flange, d.z_flange_top)
    pin = pin.union(cyl(b["journal_diameter"] / 2, d.z_flange_top, d.z_ring))
    pin = pin.union(
        d_profile(b["spigot_diameter"], b["spigot_flat"], d.z_ring, d.z_spigot_top)
    )
    pin = pin.cut(cyl(c["screw"]["m3"]["pilot"] / 2, d.z_spigot_top - 9, d.z_spigot_top + 1))
    mag_d, mag_t = b["angle_magnet"]
    side = c["fit"]["side"]
    return pin.cut(cyl((mag_d + side) / 2, d.z_flange - 1, d.z_flange + mag_t + side)).val()


def washers(d: Dims, radius_change: float = 0.0) -> cq.Shape:
    """Bought part: the steel hub, DIN 125 washers stacked in the groove."""
    w = d.c["hub_washer"]
    height = w["thickness"] * w["count"]
    return cyl(d.hub_r + radius_change, d.z_groove, d.z_groove + height).cut(
        cyl(w["bore"] / 2, d.z_groove - 1, d.z_groove + height + 1)
    ).val()


# ---------------------------------------------------------------- the part list


@dataclass(frozen=True)
class Part:
    name: str
    group: str  # "base" turns under the "ring"; "bought" parts are not printed
    count: int
    material: str
    orientation: str
    shape: cq.Shape  # robot frame, one instance
    print_shape: cq.Shape | None = None  # z from the bed; None for bought parts
    listed: tuple[Listed, ...] = ()
    others: tuple[cq.Shape, ...] = ()  # further instances in the robot frame


def on_bed(shape: cq.Shape) -> cq.Shape:
    return shape.translate(cq.Vector(0, 0, -float(extent(shape)[0][2])))


def printed_parts(d: Dims) -> list[Part]:
    c = d.c
    span = 2 * d.base_r
    ir = c["ir"]
    crown = ir["hole"] / 2
    crown_z = ir["height"] - d.z_floor
    sens_tall, sens_wide = c["reflective_sensor"]["body"]
    scl = c["reflective_sensor"]["clearance"]
    tick_z = d.axle_z + sens_tall / 2 + scl - d.z_floor
    tx = c["tick_sensor_x"]
    chassis_listed = (
        Listed("bridge", (-span, -span, crown_z + crown * 0.7), (span, span, crown_z + crown + 0.1),
               "crowns of the twelve infrared holes", ir["hole"]),
    ) + tuple(
        Listed("bridge", (tx - sens_wide / 2 - scl, min(s * d.y_wall, s * d.y_well), tick_z - 0.1),
               (tx + sens_wide / 2 + scl, max(s * d.y_wall, s * d.y_well), tick_z + 0.1),
               "top of a wheel tick sensor window", sens_wide + 2 * scl)
        for s in (-1, 1)
    )  # fmt: skip

    det = c["detent"]
    det_r = (det["magnet"][0] + c["fit"]["side"]) / 2
    z_t = d.plate_t + (d.groove - d.tongue_t) / 2
    rim_x = math.sqrt(d.plate_r**2 - (d.tongue_w / 2) ** 2)
    lower_listed = (
        Listed("support", (rim_x - 0.5, -d.tongue_w / 2, z_t - 0.1),
               (d.reach, d.tongue_w / 2, z_t + 0.1),
               f"under the tongue outside the plate, {z_t:.1f} mm high"),
        Listed("bridge", (det["radius"] - det_r, -det_r, 0.1), (det["radius"] + det_r, det_r, 10),
               "ceiling of the detent magnet pocket", 2 * det_r),
    )  # fmt: skip
    mag_r = (c["bearing"]["angle_magnet"][0] + c["fit"]["side"]) / 2
    pin_listed = (
        Listed("bridge", (-mag_r, -mag_r, 0.1), (mag_r, mag_r, 10),
               "ceiling of the angle magnet pocket", 2 * mag_r),
    )  # fmt: skip

    hub, tyre = wheel_hub_print(d), tyre_print(d)
    flat = "as it sits in the robot"
    front = skid(d)
    lower, pin = ring_lower(d), retainer(d)
    ch = chassis(d)
    return [
        Part("chassis", "base", 1, "PLA", f"{flat}, floor on the bed", ch, on_bed(ch),
             chassis_listed),
        Part("lid", "base", 1, "PLA", f"{flat}, underside on the bed", lid(d), on_bed(lid(d))),
        Part("board_carrier", "base", 1, "PLA", f"{flat}, plate on the bed", carrier(d),
             on_bed(carrier(d))),
        Part("skid", "base", 2, "PETG", "on its side, so the ramp is drawn in the layer", front,
             on_bed(front.rotate(cq.Vector(0, 0, 0), cq.Vector(1, 0, 0), 90)),
             others=(skid(d, rear=True),)),
        Part("wheel_hub", "base", 2, "PETG", "axle upright, striped face up",
             on_axle(d, hub, True), on_bed(hub), others=(on_axle(d, hub, False),)),
        Part("tyre", "base", 2, "TPU", "axle upright", on_axle(d, tyre, True), on_bed(tyre),
             others=(on_axle(d, tyre, False),)),
        Part("ring_lower", "ring", 1, "PETG", f"{flat}, plate on the bed, tongue flat", lower,
             on_bed(lower), lower_listed),
        Part("ring_upper", "ring", 1, "PETG", f"{flat}, underside on the bed", ring_upper(d),
             on_bed(ring_upper(d))),
        Part("bearing_retainer", "ring", 1, "PETG", "flange on the bed, pin upright", pin,
             on_bed(pin), pin_listed),
    ]  # fmt: skip


def bought_parts(d: Dims) -> list[Part]:
    return [
        Part("motor", "base", 2, "bought", "", motor(d, True), others=(motor(d, False),)),
        Part("hub_washers", "ring", 1, "bought", "", washers(d)),
    ]


def robot(d: Dims) -> dict[str, list[tuple[str, cq.Shape]]]:
    """Every solid of one robot by group: {"base": [(name, shape), ...], "ring": [...]}."""
    out: dict[str, list[tuple[str, cq.Shape]]] = {"base": [], "ring": []}
    for part in printed_parts(d) + bought_parts(d):
        for i, shape in enumerate((part.shape, *part.others)):
            out[part.group].append((f"{part.name}{'_b' if i else ''}", shape))
    return out
