"""Run one scenario with one seed and print pass/fail plus metrics as JSON.

    python -m sim.runner wander --seed 3
    python -m sim.runner wander --seed 3 --viewer
    python -m sim.runner wander --seed 3 --set behaviour.wander.speed=80 --duration 30
"""

from __future__ import annotations

import argparse
import importlib
import json
import os
import sys
from concurrent.futures import ProcessPoolExecutor
from types import ModuleType
from typing import Any

import yaml

from sim.config import load_config
from sim.swarm import Swarm


def load_scenario(name: str) -> ModuleType:
    return importlib.import_module(f"sim.scenarios.{name}")


def build(
    name: str, seed: int, overrides: dict[str, Any] | None = None
) -> tuple[ModuleType, Swarm]:
    scenario = load_scenario(name)
    return scenario, scenario.build(load_config(overrides), seed)


def result(name: str, seed: int, scenario: ModuleType, swarm: Swarm) -> dict[str, Any]:
    passed, metrics = scenario.evaluate(swarm)
    return {
        "scenario": name,
        "seed": seed,
        "passed": bool(passed),
        "sim_time_s": round(swarm.world.time_s, 4),
        "metrics": metrics,
        "final_positions": swarm.final_positions(),
    }


def run(
    name: str,
    seed: int,
    overrides: dict[str, Any] | None = None,
    duration_s: float | None = None,
) -> dict[str, Any]:
    scenario, swarm = build(name, seed, overrides)
    swarm.run(duration_s if duration_s is not None else scenario.DURATION_S)
    return result(name, seed, scenario, swarm)


def _run_job(job: tuple[str, int, dict[str, Any] | None, float | None]) -> dict[str, Any]:
    return run(*job)


def run_seeds(
    name: str,
    seeds: list[int],
    overrides: dict[str, Any] | None = None,
    duration_s: float | None = None,
) -> list[dict[str, Any]]:
    """Run one scenario over many seeds in parallel processes; results in seed order."""
    jobs = [(name, seed, overrides, duration_s) for seed in seeds]
    workers = max(1, min(len(jobs), (os.cpu_count() or 2) - 1))
    with ProcessPoolExecutor(max_workers=workers) as pool:
        return list(pool.map(_run_job, jobs))


def parse_overrides(items: list[str]) -> dict[str, Any]:
    overrides = {}
    for item in items:
        key, _, value = item.partition("=")
        overrides[key] = yaml.safe_load(value)
    return overrides


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("scenario")
    parser.add_argument("--seed", type=int, default=1)
    parser.add_argument("--set", action="append", default=[], metavar="KEY=VALUE",
                        help="override a config value, e.g. radio.drop_rate=0.2")
    parser.add_argument("--duration", type=float, help="simulated seconds (default: scenario)")
    parser.add_argument("--viewer", action="store_true", help="open the viewer")
    parser.add_argument("--speed", type=float, default=1.0, help="viewer: sim seconds per second")
    parser.add_argument("--exit-after", type=float, help="viewer: close after this sim time")
    parser.add_argument("--screenshot", help="viewer: save a PNG when it closes")
    args = parser.parse_args(argv)

    overrides = parse_overrides(args.set)
    scenario, swarm = build(args.scenario, args.seed, overrides)
    duration = args.duration if args.duration is not None else scenario.DURATION_S
    if args.viewer:
        from sim.viewer import Viewer

        Viewer(swarm, title=f"Catena: {args.scenario}, seed {args.seed}").run(
            duration, speed=args.speed, exit_after=args.exit_after, screenshot=args.screenshot)
    else:
        swarm.run(duration)
    out = result(args.scenario, args.seed, scenario, swarm)
    out.pop("final_positions")
    print(json.dumps(out, indent=2))
    return 0 if out["passed"] else 1


if __name__ == "__main__":
    sys.exit(main())
