"""The robot's parts pass the checks of spec section 11, and the checks notice a bad part."""

from __future__ import annotations

from pathlib import Path

import pytest

from cad import build, checks, parts, verify
from sim.config import load_config

REPO = Path(__file__).resolve().parent.parent


@pytest.fixture(scope="module")
def model() -> verify.Model:
    return verify.Model()


def changed(**overrides: object) -> parts.Dims:
    return parts.Dims(load_config({k.replace("__", "."): v for k, v in overrides.items()}))


@pytest.mark.parametrize("check", verify.CHECKS, ids=lambda c: c.__name__)
def test_make_cad_check_passes(model, check):
    result = check(model)
    assert result.ok, f"{result.name}: {result.detail}"


def test_the_part_list_is_the_one_in_the_spec(model):
    names = {p.name for p in model.printed}
    assert names == {
        "chassis", "lid", "board_carrier", "skid", "wheel_hub", "tyre",
        "ring_lower", "ring_upper", "bearing_retainer",
    }  # fmt: skip


def test_coupons_cover_groove_magnet_bearing_and_shaft(model):
    names = [c.name for c in model.coupons]
    cfg = model.d.c["coupon"]
    assert sum(n.startswith("coupon_groove_5") for n in names) == len(cfg["groove_heights"]) == 3
    assert sum(n.startswith("coupon_tongue_wall") for n in names) == len(cfg["magnet_walls"]) == 3
    assert {"coupon_bearing_lid", "coupon_bearing_ring", "coupon_shaft_bar"} <= set(names)
    # The three grooves differ by exactly the heights in the config.
    lows = [float(checks.extent(model.part(f"coupon_groove_{h:.1f}").print_shape)[1][2])
            for h in cfg["groove_heights"]]  # fmt: skip
    assert lows == pytest.approx([model.d.plate_t + h for h in cfg["groove_heights"]], abs=1e-6)


def test_a_chassis_wall_of_one_millimetre_is_caught():
    thin = parts.chassis(changed(cad__shell__wall=1.0))
    found = checks.thin_walls(parts.on_bed(thin), 1.2)
    assert found and found[0].value == pytest.approx(1.0, abs=0.06)


def test_infrared_holes_on_their_60_degree_lines_would_break_into_the_wheel_wells():
    plain = parts.chassis(changed(cad__ir__wheel_side_shift=0.0))
    assert checks.thin_walls(parts.on_bed(plain), 1.2)


def test_a_joint_bent_past_its_limit_is_caught(model):
    volume, a, b = verify._worst_overlap(*verify.linked_pair(model, model.d.yaw_deg + 10))
    assert volume > 1.0 and {a, b} == {"ring_lower"}


def test_robots_closer_than_the_pitch_are_caught(model):
    volume, *_ = verify._worst_overlap(*verify.linked_pair(model, 0.0, model.d.pitch - 1.0))
    assert volume > 1.0


def test_the_tongue_overhang_is_only_excused_by_its_listed_support(model):
    lower = model.part("ring_lower")
    bridges = tuple(b for b in lower.listed if b.kind == "bridge")
    found = checks.unlisted_overhang(lower.print_shape, 50.0, bridges)
    outside = (model.d.reach - model.d.plate_r) * model.d.tongue_w
    assert found is not None and found.value == pytest.approx(outside, rel=0.15)


def test_a_base_wider_than_its_envelope_is_caught(model):
    wide = verify.Model.__new__(verify.Model)
    wide.__dict__.update(model.__dict__)
    wide.robot = {"base": [*model.robot["base"], ("skid", checks.placed(
        model.solid("skid"), 0, (10.0, 0, 0)))], "ring": model.robot["ring"]}  # fmt: skip
    assert not verify.base_turns_a_full_circle_inside_its_ring(wide).ok


def test_exports_are_step_stl_and_3mf_for_every_part_and_the_coupon_plate(model, tmp_path):
    written = build.export(model, tmp_path)
    names = {p.name for p in model.printable()} | {"coupon_plate"}
    assert {p.name for p in written} == {f"{n}.{ext}" for n in names for ext in build.FORMATS}
    assert all(p.stat().st_size > 1000 for p in written)


def test_hardware_doc_lists_every_printed_part_with_orientation_and_material(model):
    text = (REPO / "docs" / "HARDWARE.md").read_text(encoding="utf-8")
    for p in model.printed:
        row = next((ln for ln in text.splitlines() if ln.startswith(f"| `{p.name}`")), None)
        assert row is not None, f"{p.name} is missing from the printed-parts table"
        assert p.material in row and p.orientation in row and f"| {p.count} |" in row


def test_pictures_of_one_robot_and_of_two_linked_robots_are_committed():
    for name in ("cad_robot.png", "cad_linked.png"):
        assert (REPO / "docs" / "media" / name).stat().st_size > 10_000
