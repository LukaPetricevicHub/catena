"""M2: the link handshake, unrequested links and FOLLOW, with core in the loop."""

import math

from sim import corelib
from sim.config import load_config
from sim.scenarios.chain import physical_chains
from sim.swarm import Swarm


def line_of(count: int, seed: int = 1, linked: bool = False) -> Swarm:
    """Robots in a row along x, facing +x, robot 0 in front. `linked` joins them
    physically; the cores are not told."""
    cfg = load_config()
    swarm = Swarm(cfg, seed)
    spacing = cfg["geometry"]["pitch"] if linked else 200.0
    for i in range(count):
        swarm.add_robot(300.0 - i * spacing, 0.0, 0.0)
    if linked:
        for i in range(1, count):
            swarm.world.link(swarm.world.robots[i], swarm.world.robots[i - 1])
        for r in swarm.world.robots:  # held in the hand until the cores have found the links
            swarm.world.set_grounded(r, False)
        swarm.run(1.0)
        for r in swarm.world.robots:
            swarm.world.set_grounded(r, True)
    return swarm


def run_until(swarm: Swarm, done, limit_s: float) -> bool:
    while swarm.world.time_s < limit_s:
        swarm.advance()
        if done():
            return True
    return False


def test_the_lower_ranked_robot_docks_onto_the_other_and_both_agree():
    swarm = line_of(2)
    r0, r1 = swarm.world.robots

    def agreed() -> bool:
        d0, d1 = swarm.debug.get(0), swarm.debug.get(1)
        return bool(d0 and d1 and d0.behind == 1 and d1.ahead == 0)

    assert run_until(swarm, agreed, 60.0)
    # Rank: equal size, so robot 1 (higher id) docked onto robot 0, never the reverse.
    assert r1.ahead is not None and r1.ahead.hub is r0
    assert swarm.debug[0].index == 0 and swarm.debug[1].index == 1
    assert swarm.debug[0].chain_size == swarm.debug[1].chain_size == 2
    assert swarm.debug[0].chain_id == swarm.debug[1].chain_id == 0
    assert swarm.debug[1].state == corelib.STATE_FOLLOW


def test_robots_linked_without_a_handshake_adopt_the_link():
    # M1 saw wandering robots dock by accident: core must take such a link as it finds it.
    swarm = line_of(3, linked=True)
    assert [d.index for d in (swarm.debug[i] for i in range(3))] == [0, 1, 2]
    assert all(swarm.debug[i].chain_size == 3 for i in range(3))
    assert swarm.debug[1].ahead == 0 and swarm.debug[1].behind == 2


def test_a_chain_follows_its_wandering_head_without_breaking():
    swarm = line_of(5, seed=3, linked=True)
    start = [r.position for r in swarm.world.robots]
    swarm.run(90.0)
    assert physical_chains(swarm) == [[0, 1, 2, 3, 4]]
    assert not [e for e in swarm.world.events if e.kind in ("unlink", "fall")]
    moved = [math.dist(r.position, p) for r, p in zip(swarm.world.robots, start, strict=True)]
    assert swarm.path_mm[0] > 1000.0  # the head really wandered
    assert min(swarm.path_mm) > 500.0 and max(moved) > 100.0
    assert all(swarm.debug[i].state == corelib.STATE_FOLLOW for i in range(1, 5))


def test_when_a_robot_is_lifted_out_the_two_halves_recount():
    swarm = line_of(5, linked=True)
    swarm.run(3.0)
    assert all(swarm.debug[i].chain_size == 5 for i in range(5))
    world = swarm.world
    world.unlink(world.robots[2].ahead)
    for r in world.robots[2:]:  # carry the rear half away, still linked to each other
        r.base.position = r.ring.position = (r.position.x, r.position.y + 300.0)
    swarm.run(world.time_s + 1.5)
    assert [swarm.debug[i].chain_size for i in range(5)] == [2, 2, 3, 3, 3]
    assert [swarm.debug[i].index for i in range(5)] == [0, 1, 0, 1, 2]
    assert swarm.debug[2].ahead == corelib.NONE and swarm.debug[1].behind == corelib.NONE
