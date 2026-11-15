"""A linked chain takes the shape of a stroke (spec 7.4). All simulated."""

import pytest

from sim.runner import run_seeds
from sim.scenarios import shape

SEEDS = list(range(5))  # 5 seeds per shape


@pytest.fixture(scope="module", params=["line", "ring", "u", "s"])
def runs(request):
    return request.param, run_seeds(f"shape:{request.param}", SEEDS)


def summary(results) -> str:
    return "; ".join(f"seed {r['seed']}: {r['metrics']}" for r in results if not r["passed"])


def test_mean_vertex_error_is_under_12_mm_after_60_s(runs):
    name, results = runs
    for r in results:
        assert r["metrics"]["error_mm"] < shape.ERROR_LIMIT_MM, f"{name}: {summary(results)}"


def test_no_robot_falls_and_no_link_breaks_while_the_shape_forms(runs):
    name, results = runs
    for r in results:
        assert r["metrics"]["falls"] == 0, f"{name}: {summary(results)}"
        assert r["metrics"]["links_broken"] == 0, f"{name}: {summary(results)}"


def test_the_stroke_reaches_every_robot_within_2_s(runs):
    name, results = runs
    for r in results:
        heard = r["metrics"]["heard_by_all_s"]
        assert heard is not None and heard <= 2.0, f"{name}: {summary(results)}"


def test_the_ring_closes_itself_and_open_shapes_stay_open(runs):
    # The scenario only hands over the stroke; nothing tells the head to dock onto its tail.
    name, results = runs
    for r in results:
        assert r["metrics"]["closed"] == (name == "ring"), f"{name}: {summary(results)}"
