"""Sweep (spec 7.7): a U-shaped scoop pushes eight blocks toward a table corner and parks
against both edges of it. All simulated."""

import os

import pytest

from sim.runner import run_seeds
from sim.scenarios import sweep

# The acceptance line is 80% of 20 seeds; `make test-full` (CATENA_FULL=1) holds it. `make
# test` runs the first 6 and asks less of them: the same seeds play out differently on
# another machine, and six runs cannot hold a rate.
SEEDS = list(range(20 if os.environ.get("CATENA_FULL") else 6))
REQUIRED_FRACTION = 0.8 if os.environ.get("CATENA_FULL") else 0.6


@pytest.fixture(scope="module")
def results():
    return run_seeds("sweep", SEEDS)


def summary(results) -> str:
    passed = sum(r["passed"] for r in results)
    lines = [f"{passed} of {len(results)} seeds parked the scoop with its blocks"]
    lines += [f"seed {r['seed']}: {r['metrics']}" for r in results if not r["passed"]]
    return "\n".join(lines)


def test_the_scoop_parks_against_both_edges_with_the_blocks_in_four_of_five_runs(results):
    passed = [r for r in results if r["passed"]]
    assert len(passed) >= REQUIRED_FRACTION * len(results), summary(results)
    for r in passed:
        m = r["metrics"]
        assert m["parked"] and m["links_broken"] == 0
        assert max(m["edge_gap_mm"]) <= sweep.AGAINST_MM
        assert m["blocks_inside"] >= sweep.BLOCKS_INSIDE_FRACTION * sweep.BLOCKS


def test_no_robot_leaves_the_table_while_sweeping(results):
    assert sum(r["metrics"]["falls"] for r in results) == 0, summary(results)


def test_a_block_is_pushed_by_a_robot_and_stays_where_it_is_left():
    from sim.config import load_config
    from sim.swarm import Swarm

    swarm = Swarm(load_config(), 1)
    robot = swarm.world.add_robot(0.0, 0.0, 0.0)
    block = swarm.world.add_block(60.0, 0.0)
    swarm.world.set_wheels(robot, 30.0, 30.0)
    for _ in range(round(3.0 * swarm.physics_hz)):
        swarm.world.step()
    pushed_to = block.position.x
    # 90 mm of travel; the block's centre is a base radius and half a block ahead: 136 mm.
    assert 130.0 < pushed_to < 142.0
    swarm.world.set_wheels(robot, -30.0, -30.0)
    for _ in range(round(1.0 * swarm.physics_hz)):
        swarm.world.step()
    assert abs(block.position.x - pushed_to) < 2.0  # a wooden cube does not roll on
