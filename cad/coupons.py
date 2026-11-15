"""Coupons of docs/SPEC.md section 11: small prints that answer one question each."""

from __future__ import annotations

import cadquery as cq

from cad import parts
from cad.parts import Dims, Part, box, countersink, cyl, d_profile, on_bed

GAP_MM = 4.0  # between coupons on the plate


def _marks(body: cq.Workplane, count: int, x0: float, y: float, z1: float) -> cq.Workplane:
    """`count` notches in the face y, starting at x0: they tell the variants apart."""
    for k in range(count):
        body = body.cut(box(x0 + 3.0 * k, x0 + 3.0 * k + 1.5, y - 1.0, y + 1.0, -1, z1 + 1))
    return body


def groove_lower(d: Dims, height: float, mark: int) -> cq.Shape:
    """Lower plate with two rails of the groove height; the tongue stub slides between them."""
    rail, slack = 6.0, 2.0
    inner = d.tongue_w + slack
    depth = d.cfg["geometry"]["tongue"]["engagement"] + 5.0
    width = inner + 2 * rail
    body = box(0, width, 0, depth, 0, d.plate_t)
    for x0 in (0.0, rail + inner):
        body = body.union(box(x0, x0 + rail, 0, depth, d.plate_t, d.plate_t + height))
        body = body.cut(cyl(d.c["screw"]["m2"]["pilot"] / 2, d.plate_t + height - 6,
                            d.plate_t + height + 1, x0 + rail / 2, depth - 6))
    return _marks(body, mark, rail + 4.0, 0.0, d.plate_t).val()


def groove_upper(d: Dims) -> cq.Shape:
    rail, slack = 6.0, 2.0
    inner = d.tongue_w + slack
    depth = d.cfg["geometry"]["tongue"]["engagement"] + 5.0
    body = box(0, inner + 2 * rail, 0, depth, 0, d.plate_t)
    for x0 in (0.0, rail + inner):
        body = body.cut(countersink(x0 + rail / 2, depth - 6, d.plate_t, d.c["screw"]["m2"]))
    return body.val()


def tongue_stub(d: Dims, wall: float, mark: int) -> cq.Shape:
    """The tongue's last 45 mm with the tip magnet pocket behind a wall of the given thickness."""
    root = d.reach - 45.0
    body = parts.tongue_outline(d, root, 0, d.tongue_t)
    body = body.cut(parts.tip_magnet_pocket(d, d.tongue_t, wall))
    return _marks(body, mark, root + 3.0, d.tongue_w / 2, d.tongue_t).val()


def washer_post(d: Dims) -> cq.Shape:
    """Holds the two hub washers while a tongue stub is pulled off them."""
    w = d.c["hub_washer"]
    half = d.hub_r + 3.0
    core_r = (w["bore"] - d.c["ring"]["core_clearance"]) / 2
    return box(-half, half, -half, half, 0, d.plate_t).union(
        cyl(core_r, d.plate_t, d.plate_t + d.groove)
    ).val()


def bearing_lid(d: Dims) -> cq.Shape:
    """The middle of the lid: bore and thrust land."""
    b = d.c["bearing"]
    r = b["land_diameter"] / 2 + 6.0
    body = cyl(r, d.z_lid, d.z_base_top).union(cyl(b["land_diameter"] / 2, d.z_base_top, d.z_ring))
    return body.cut(
        cyl((b["journal_diameter"] + d.c["fit"]["turning"]) / 2, d.z_lid - 1, d.z_ring + 1)
    ).val()


def bearing_ring(d: Dims) -> cq.Shape:
    """The middle of the lower ring: plate, hub core and the flatted bore for the pin."""
    b, c = d.c["bearing"], d.c
    r = b["land_diameter"] / 2 + 6.0
    core_r = (c["hub_washer"]["bore"] - c["ring"]["core_clearance"]) / 2
    body = cyl(r, 0, d.plate_t).union(cyl(core_r, d.plate_t, d.plate_t + d.groove))
    return body.cut(
        d_profile(b["spigot_diameter"] + c["fit"]["static"],
                  b["spigot_flat"] + c["fit"]["static"] / 2, -1, d.plate_t + d.groove + 1)
    ).val()


def bearing_cap(d: Dims) -> cq.Shape:
    """Stands in for the upper plate: the M3 screw clamps it onto the hub core."""
    r = d.c["hub_washer"]["bore"] / 2 + 2.0
    return cyl(r, 0, d.plate_t).cut(countersink(0, 0, d.plate_t, d.c["screw"]["m3"])).val()


def shaft_bar(d: Dims) -> cq.Shape:
    """Three D-bores for the motor shaft, tighter to looser; one notch marks the tightest."""
    shaft = d.c["motor"]["shaft"]
    fits = d.c["coupon"]["shaft_fits"]
    step, width = 12.0, 12.0
    body = box(0, step * len(fits), 0, width, 0, d.wheel_w)
    for k, fit in enumerate(fits):
        bore = d_profile(shaft["diameter"] + fit, shaft["flat"] + fit - shaft["diameter"] / 2,
                         -1, d.wheel_w + 1)
        body = body.cut(bore.translate((step * (k + 0.5), width / 2, 0)))
        body = _marks(body, k + 1, step * k + 1.5, 0.0, d.wheel_w)
    return body.val()


def coupons(d: Dims) -> list[Part]:
    cp = d.c["coupon"]
    out: list[Part] = []

    def add(name: str, shape: cq.Shape, how: str, count: int = 1) -> None:
        out.append(Part(name, "coupon", count, "PETG", how, on_bed(shape), on_bed(shape)))

    for k, h in enumerate(cp["groove_heights"]):
        add(f"coupon_groove_{h:.1f}", groove_lower(d, h, k + 1), "plate on the bed")
    add("coupon_groove_top", groove_upper(d), "flat", len(cp["groove_heights"]))
    for k, w in enumerate(cp["magnet_walls"]):
        add(f"coupon_tongue_wall_{w:.1f}", tongue_stub(d, w, k + 1), "flat, pocket up")
    add("coupon_washer_post", washer_post(d), "plate on the bed")
    add("coupon_bearing_lid", bearing_lid(d), "underside on the bed")
    add("coupon_bearing_ring", bearing_ring(d), "plate on the bed")
    add("coupon_bearing_cap", bearing_cap(d), "flat")
    add("coupon_shaft_bar", shaft_bar(d), "flat")
    return out


def plate(d: Dims, extra: tuple[cq.Shape, ...] = ()) -> cq.Shape:
    """Every coupon, plus the given production parts, laid out in rows on one bed."""
    pieces = [c.print_shape for c in coupons(d) for _ in range(c.count)] + list(extra)
    pieces.sort(key=lambda s: -s.BoundingBox().ylen)
    bed_x = d.c["print"]["bed"][0]
    x = y = row = 0.0
    placed: list[cq.Shape] = []
    for piece in pieces:
        bb = piece.BoundingBox()
        if x + bb.xlen > bed_x:
            x, y, row = 0.0, y + row + GAP_MM, 0.0
        placed.append(piece.translate(cq.Vector(x - bb.xmin, y - bb.ymin, -bb.zmin)))
        x += bb.xlen + GAP_MM
        row = max(row, bb.ylen)
    return cq.Compound.makeCompound(placed)
