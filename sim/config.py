"""Loader for config/robot.yaml, the single source of numbers.

It also generates the C header that core/ compiles against:

    python -m sim.config --header build/generated/catena/params_gen.h

The YAML is in mm, g, deg, s and N. Everything handed to code is converted here, once:
field names carry the unit and angles are radians.
"""

from __future__ import annotations

import argparse
import copy
import math
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

REPO_ROOT = Path(__file__).resolve().parent.parent
CONFIG_PATH = REPO_ROOT / "config" / "robot.yaml"

RAD_PER_DEG = math.pi / 180.0


def load_config(
    overrides: dict[str, Any] | None = None, path: Path = CONFIG_PATH
) -> dict[str, Any]:
    """Return the YAML tree. Overrides use dotted keys, e.g. {"sensing.ir_range": 180.0}."""
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    for key, value in (overrides or {}).items():
        *parents, leaf = key.split(".")
        node = cfg
        for part in parents:
            node = node.get(part) if isinstance(node, dict) else None
        if not isinstance(node, dict) or leaf not in node:
            raise KeyError(f"unknown config key: {key}")
        node[leaf] = copy.deepcopy(value)
    return cfg


def lookup(cfg: dict[str, Any], dotted: str) -> Any:
    """Value at a dotted key, e.g. lookup(cfg, "geometry.pitch")."""
    node: Any = cfg
    for part in dotted.split("."):
        node = node[part]
    return node


@dataclass(frozen=True)
class CoreField:
    """One float member of catena_params_t: C name with unit, YAML key, config-to-code scale."""

    name: str
    source: str
    scale: float = 1.0


# Order here is the order of the C struct and of the ctypes mirror in sim/corelib.py.
CORE_FIELDS: tuple[CoreField, ...] = (
    CoreField("pitch_mm", "geometry.pitch"),
    CoreField("max_joint_yaw_rad", "geometry.ring.max_joint_yaw", RAD_PER_DEG),
    CoreField("track_mm", "geometry.base.track"),
    CoreField("max_speed_mm_s", "drive.max_speed"),
    CoreField("max_turn_rate_rad_s", "drive.max_turn_rate", RAD_PER_DEG),
    CoreField("traction_force_n", "drive.traction_force_n"),
    CoreField("hold_force_n", "coupling.hold_force_n"),
    CoreField("ir_range_mm", "sensing.ir_range"),
    CoreField("edge_margin_mm", "table.edge_margin"),
)

# Compile-time integer constants (array sizes in core).
CORE_CONSTANTS: tuple[tuple[str, str], ...] = (
    ("CATENA_IR_COUNT", "sensing.ir_pairs"),
    ("CATENA_DESIGN_COUNT", "swarm.design_count"),
)


def core_params(cfg: dict[str, Any]) -> dict[str, float]:
    """Values of catena_params_t in code units, in declaration order."""
    return {f.name: float(lookup(cfg, f.source)) * f.scale for f in CORE_FIELDS}


def core_constants(cfg: dict[str, Any]) -> dict[str, int]:
    return {name: int(lookup(cfg, source)) for name, source in CORE_CONSTANTS}


def _c_float(value: float) -> str:
    """Literal that reads back as exactly the float32 nearest to value."""
    as_f32 = struct.unpack("f", struct.pack("f", value))[0]
    text = f"{as_f32:.9g}"
    if "." not in text and "e" not in text:
        text += ".0"
    return text + "f"


def render_header(cfg: dict[str, Any]) -> str:
    values = core_params(cfg)
    lines = [
        "/* Generated from config/robot.yaml by sim/config.py. Do not edit. */",
        "#ifndef CATENA_PARAMS_GEN_H",
        "#define CATENA_PARAMS_GEN_H",
        "",
    ]
    lines += [f"#define {name} {value}" for name, value in core_constants(cfg).items()]
    lines += ["", "typedef struct catena_params_t {"]
    lines += [f"  float {f.name};" for f in CORE_FIELDS]
    lines += ["} catena_params_t;", "", "#define CATENA_PARAMS_DEFAULT \\", "  { \\"]
    lines += [f"    {_c_float(values[f.name])}, /* {f.name} */ \\" for f in CORE_FIELDS]
    lines += ["  }", "", "#endif", ""]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser(description="Generate the core parameter header.")
    parser.add_argument("--header", type=Path, required=True, help="output path")
    parser.add_argument("--config", type=Path, default=CONFIG_PATH)
    args = parser.parse_args()
    args.header.parent.mkdir(parents=True, exist_ok=True)
    args.header.write_text(render_header(load_config(path=args.config)), encoding="utf-8")


if __name__ == "__main__":
    main()
