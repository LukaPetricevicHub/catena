"""M1 acceptance: two robots pushed together link; the joint breaks above the hold force and
not below it. A test script drives the wheels directly; core is not involved."""

import math

import pytest

from sim.config import load_config
from sim.world import NEWTON, World

PUSH_SPEED_MM_S = 20.0
START_GAP_MM = 30.0  # tongue tip to hub surface at the start
SEATED_MM = 0.5  # tip this close to the hub counts as landed
PULL_TIME_S = 3.0


def pair(approach_deg: float):
    """A hub robot at the origin facing +x, and a tongue robot facing it from
    `approach_deg` off the hub robot's rear."""
    cfg = load_config()
    world = World(cfg)
    hub = world.add_robot(0.0, 0.0, 0.0)
    side = math.pi + math.radians(approach_deg)
    reach = cfg["geometry"]["tongue"]["reach"] + cfg["geometry"]["ring"]["hub_diameter"] / 2
    d = reach + START_GAP_MM
    tongue = world.add_robot(d * math.cos(side), d * math.sin(side), side + math.pi)
    return world, tongue, hub


def seated(world, tongue) -> bool:
    return bool(tongue.ahead) and world.link_gap(tongue.ahead) < SEATED_MM


def push_together(world, tongue, timeout_s=4.0):
    """Drive the tongue robot at the hub until the tip sits on it, as docking will."""
    world.set_wheels(tongue, PUSH_SPEED_MM_S, PUSH_SPEED_MM_S)
    while world.time_s < timeout_s and not seated(world, tongue):
        world.step()
    world.set_wheels(tongue, 0.0, 0.0)
    for _ in range(round(1.0 / world.dt)):
        world.step()


def linked_pair():
    world, tongue, hub = pair(0.0)
    push_together(world, tongue)
    assert tongue.ahead is not None and tongue.ahead.hub is hub
    return world, tongue, hub


def pull(world, tongue, hub, force_n: float, seconds: float) -> None:
    """Lift the hub robot off the table and pull it straight away from the tongue robot,
    until the time is up or the link lets go."""
    world.set_grounded(hub, False)
    direction = (hub.position - tongue.position).normalized()
    for _ in range(round(seconds / world.dt)):
        if not hub.behind:
            break
        hub.base.apply_force_at_world_point(direction * force_n * NEWTON, hub.position)
        world.step()


def test_two_robots_pushed_together_link_at_one_pitch():
    world, tongue, hub = linked_pair()
    assert (hub.position - tongue.position).length == pytest.approx(world.pitch, abs=SEATED_MM)
    assert [e.kind for e in world.events] == ["link"]
    assert world.hall(hub) > 0.9
    assert world.hall(tongue) == 0.0


@pytest.mark.parametrize("approach_deg", [-90.0, -45.0, 45.0, 90.0])
def test_a_tongue_links_from_anywhere_inside_the_acceptance_angle(approach_deg):
    world, tongue, hub = pair(approach_deg)
    push_together(world, tongue)
    assert tongue.ahead is not None and hub.behind is tongue.ahead


@pytest.mark.parametrize("approach_deg", [120.0, 180.0, -135.0])
def test_a_tongue_does_not_link_from_outside_the_acceptance_angle(approach_deg):
    world, tongue, hub = pair(approach_deg)
    push_together(world, tongue)
    assert tongue.ahead is None and hub.behind is None


@pytest.mark.parametrize("fraction", [0.5, 0.9])
def test_the_joint_holds_below_the_hold_force(fraction):
    world, tongue, hub = linked_pair()
    pull(world, tongue, hub, fraction * load_config()["coupling"]["hold_force_n"], PULL_TIME_S)
    assert tongue.ahead is not None
    assert (hub.position - tongue.position).length < world.pitch + world.capture_range


@pytest.mark.parametrize("fraction", [1.1, 1.5])
def test_the_joint_breaks_above_the_hold_force(fraction):
    world, tongue, hub = linked_pair()
    pull(world, tongue, hub, fraction * load_config()["coupling"]["hold_force_n"], PULL_TIME_S)
    assert tongue.ahead is None and hub.behind is None
    assert [e.kind for e in world.events] == ["link", "unlink"]


def test_a_robot_unlinks_by_driving_away():
    # Decision D1: traction beats the magnet.
    world, tongue, hub = linked_pair()
    world.set_wheels(tongue, -PUSH_SPEED_MM_S, -PUSH_SPEED_MM_S)
    for _ in range(round(2.0 / world.dt)):
        world.step()
    assert tongue.ahead is None
