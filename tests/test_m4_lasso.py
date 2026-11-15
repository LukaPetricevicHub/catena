"""M4 acceptance: a wandering chain finds a ball, closes a ring round it and carries it
400 mm (spec 7.6). All simulated."""

import os

import pytest

from sim.runner import run_seeds

# M4 asks for 50 seeds; `make test` runs the first 10, `make test-full` (CATENA_FULL=1) all.
SEEDS = list(range(50 if os.environ.get("CATENA_FULL") else 10))
REQUIRED_FRACTION = 0.8


@pytest.fixture(scope="module")
def results():
    return run_seeds("lasso", SEEDS)


def summary(results) -> str:
    passed = sum(r["passed"] for r in results)
    lines = [f"{passed} of {len(results)} seeds caught and carried the ball"]
    lines += [f"seed {r['seed']}: {r['metrics']}" for r in results if not r["passed"]]
    return "\n".join(lines)


def test_the_ball_is_ringed_and_carried_400_mm_in_four_of_five_runs(results):
    passed = [r for r in results if r["passed"]]
    assert len(passed) >= REQUIRED_FRACTION * len(results), summary(results)
    for r in passed:
        m = r["metrics"]
        assert m["closed"] and m["ball_inside"] and m["arrival_error_mm"] < 40.0


def test_no_robot_and_no_ball_leaves_the_table(results):
    assert sum(r["metrics"]["falls"] for r in results) == 0, summary(results)
    assert not any(r["metrics"]["ball_fell"] for r in results), summary(results)
