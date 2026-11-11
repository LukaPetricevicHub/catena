"""M1 acceptance: wandering robots stay on the table, and a seed fixes the whole run."""

import os

import pytest

from sim.config import load_config
from sim.runner import run, run_seeds

DURATION_S = 120.0
# M1 asks for 10 of 10 seeds; `make test-full` (CATENA_FULL=1) runs 50.
SEEDS = list(range(50 if os.environ.get("CATENA_FULL") else 10))
# Seed 15 once fell: a robot parallel to an edge reversed along it (fixed in core wander).
REGRESSION_SEEDS = [15]


@pytest.fixture(scope="module")
def results():
    # The last job repeats the first seed in a fresh process, for the determinism test.
    return run_seeds("wander", SEEDS + REGRESSION_SEEDS + SEEDS[:1])


def test_twelve_wandering_robots_stay_on_the_table_for_120_s(results):
    for r in results:
        assert r["sim_time_s"] == pytest.approx(DURATION_S)
        assert r["metrics"]["falls"] == 0, f"seed {r['seed']}: {r['metrics']}"
        assert r["passed"]


def test_wandering_robots_keep_moving(results):
    # Standing still would pass trivially: every robot must cover at least a third of
    # what cruising at wander speed would.
    cruise_mm = load_config()["behaviour"]["wander"]["speed"] * DURATION_S
    for r in results:
        assert r["metrics"]["min_path_mm"] > cruise_mm / 3, f"seed {r['seed']}"


def test_same_seed_gives_identical_final_positions(results):
    first, repeat = results[0], results[-1]
    assert first["seed"] == repeat["seed"]
    assert first["final_positions"] == repeat["final_positions"]
    assert first["metrics"] == repeat["metrics"]


def test_same_seed_is_identical_within_one_process():
    a = run("wander", 4, duration_s=20.0)
    b = run("wander", 4, duration_s=20.0)
    assert a["final_positions"] == b["final_positions"]


def test_different_seeds_give_different_runs(results):
    assert results[0]["final_positions"] != results[1]["final_positions"]
