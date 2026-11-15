"""M4 acceptance: a chain holding a shape heals after a robot is taken out (spec 7.8).
All simulated."""

import os

import pytest

from sim.runner import run_seeds

# M4 asks for 50 seeds; `make test` runs the first 12, `make test-full` (CATENA_FULL=1) all.
SEEDS = list(range(50 if os.environ.get("CATENA_FULL") else 12))
REQUIRED_FRACTION = 0.8


@pytest.fixture(scope="module")
def results():
    return run_seeds("heal", SEEDS)


def summary(results) -> str:
    passed = sum(r["passed"] for r in results)
    lines = [f"{passed} of {len(results)} seeds healed"]
    lines += [f"seed {r['seed']}: {r['metrics']}" for r in results if not r["passed"]]
    return "\n".join(lines)


def test_the_chain_relinks_recounts_and_is_back_in_shape_within_60_s_in_four_of_five_runs(results):
    passed = [r for r in results if r["passed"]]
    assert len(passed) >= REQUIRED_FRACTION * len(results), summary(results)


def test_robots_are_removed_while_holding_a_shape_and_while_moving(results):
    states = {r["metrics"]["state_at_removal"] for r in results}
    assert "SHAPE" in states and "MOVE" in states, states


def test_no_robot_falls_while_the_chain_heals(results):
    assert sum(r["metrics"]["falls"] for r in results) == 0, summary(results)
