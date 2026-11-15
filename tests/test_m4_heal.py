"""M4 acceptance: a chain holding a shape heals after a robot is taken out (spec 7.8).
All simulated."""

import os

import pytest

from sim.runner import run_seeds

# M4 asks for 50 seeds; `make test` runs the first 12, `make test-full` (CATENA_FULL=1) all.
SEEDS = list(range(50 if os.environ.get("CATENA_FULL") else 12))
# The acceptance line is 80% of 50 seeds and `make test-full` holds it. The short set of
# `make test` only has to show that nothing broke: the same seeds play out differently on
# another machine (seen on CI: 7 of 10 on Ubuntu where macOS had 9), and ten runs at a
# true rate near 90% miss 8 of 10 too often to gate on.
REQUIRED_FRACTION = 0.8 if os.environ.get("CATENA_FULL") else 0.6


@pytest.fixture(scope="module")
def results():
    return run_seeds("heal", SEEDS)


@pytest.fixture(scope="module")
def ball_results():
    return run_seeds("heal:ball", SEEDS)


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


def test_a_ring_holding_a_ball_heals_and_keeps_the_ball_in_four_of_five_runs(ball_results):
    passed = [r for r in ball_results if r["passed"]]
    assert len(passed) >= REQUIRED_FRACTION * len(ball_results), summary(ball_results)
    for r in passed:
        assert r["metrics"]["ball_inside"] and r["metrics"]["closed"]


def test_no_robot_falls_while_a_ring_with_a_ball_heals(ball_results):
    assert sum(r["metrics"]["falls"] for r in ball_results) == 0, summary(ball_results)
