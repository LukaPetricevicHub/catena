"""A ring moves as one body (spec 7.5). All simulated."""

import pytest

from sim.runner import run_seeds
from sim.scenarios import twist

SEEDS = list(range(5))


@pytest.fixture(scope="module")
def results():
    return run_seeds("twist", SEEDS)


def summary(results) -> str:
    return "; ".join(f"seed {r['seed']}: {r['metrics']}" for r in results)


def test_a_ring_told_to_move_400_mm_along_shape_x_arrives_within_40_mm(results):
    for r in results:
        arrived = r["metrics"]["arrival_error_mm"]
        assert arrived is not None and arrived < twist.ARRIVE_LIMIT_MM, summary(results)


def test_the_ring_keeps_its_shape_within_15_mm_on_the_way(results):
    for r in results:
        assert r["metrics"]["worst_shape_error_mm"] < twist.SHAPE_LIMIT_MM, summary(results)


def test_nothing_falls_or_comes_apart_while_the_ring_moves(results):
    for r in results:
        assert r["metrics"]["falls"] == 0 and r["metrics"]["links_broken"] == 0, summary(results)
