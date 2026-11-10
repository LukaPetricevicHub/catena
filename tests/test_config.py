"""config/robot.yaml is the single source of numbers: header and loader must agree."""

import math
import struct

import pytest

from sim import corelib
from sim.config import CORE_FIELDS, core_constants, core_params, load_config, render_header


def as_float32(value: float) -> float:
    return struct.unpack("f", struct.pack("f", value))[0]


def test_generated_header_and_python_loader_read_the_same_pitch():
    cfg = load_config()
    compiled = corelib.default_params(corelib.load())
    assert compiled.pitch_mm == as_float32(cfg["geometry"]["pitch"])


def test_every_compiled_parameter_matches_the_loader():
    expected = core_params(load_config())
    compiled = corelib.default_params(corelib.load())
    for field in CORE_FIELDS:
        assert getattr(compiled, field.name) == as_float32(expected[field.name]), field.name


def test_angles_reach_code_in_radians():
    cfg = load_config()
    yaw_deg = cfg["geometry"]["ring"]["max_joint_yaw"]
    assert core_params(cfg)["max_joint_yaw_rad"] == pytest.approx(math.radians(yaw_deg))


def test_overrides_change_one_value_and_reject_unknown_keys():
    assert core_params(load_config({"geometry.pitch": 82.0}))["pitch_mm"] == 82.0
    assert core_params(load_config())["pitch_mm"] != 82.0
    with pytest.raises(KeyError):
        load_config({"geometry.no_such_key": 1.0})
    with pytest.raises(KeyError):
        load_config({"no_such_section.pitch": 1.0})


def test_header_declares_every_field_and_constant():
    cfg = load_config()
    header = render_header(cfg)
    for field in CORE_FIELDS:
        assert f"float {field.name};" in header
    for name, value in core_constants(cfg).items():
        assert f"#define {name} {value}\n" in header


def test_config_is_self_consistent():
    cfg = load_config()
    geo, coupling, mass = cfg["geometry"], cfg["coupling"], cfg["mass"]
    # Linked centres are one pitch apart: tongue tip on the neighbour's hub.
    assert geo["tongue"]["reach"] + geo["ring"]["hub_diameter"] / 2 == pytest.approx(geo["pitch"])
    # Linked plates and bases must not touch.
    assert geo["ring"]["plate_diameter"] < geo["pitch"]
    assert geo["base"]["envelope_diameter"] < geo["ring"]["plate_diameter"]
    # Decision D1: a robot unlinks by driving away.
    low, high = coupling["hold_force_range_n"]
    assert low <= coupling["hold_force_n"] <= high < cfg["drive"]["traction_force_n"]
    assert mass["ring_assembly"] + mass["base_assembly"] == pytest.approx(mass["target_total"])
    # A straight chain of all robots fits on the table.
    chain_length = (cfg["swarm"]["design_count"] - 1) * geo["pitch"] + geo["ring"]["plate_diameter"]
    assert chain_length < max(cfg["table"]["size"])
