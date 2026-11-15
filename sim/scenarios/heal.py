"""M4: a robot is taken out of a chain that holds a shape; the rest must heal (spec 7.8).

    python -m sim.runner heal --seed 4 --viewer

    python -m sim.runner heal:ball --seed 4 --viewer    a ring holding a ball

The chain takes one of the four shapes (by seed). In some seeds it is also moving when it
happens. At a random time a random robot is removed. In the `ball` variant the chain is a
ring that has closed round a ball, and the ball must still be inside it at the end.
Passes if, 60 s later, the remaining robots are one chain again, agree on index and count,
and are within the limit of the shape drawn for their new number; nothing fell and no
other link was lost for good."""

from __future__ import annotations

import math
import random
from typing import Any

import numpy as np

from sim import corelib, shapes
from sim.scenarios import shape as shape_scenario
from sim.swarm import Swarm

HEAL_LIMIT_S = 60.0
ERROR_LIMIT_MM = 15.0
REMOVE_WINDOW_S = (8.0, 50.0)  # after the stroke: while the shape forms or is held
BALL_REMOVE_WINDOW_S = (35.0, 50.0)  # the ring round the ball is closed by then
MOVE_SPEED_MM_S = 12.0
MOVE_S = 10.0
DURATION_S = shape_scenario.HOLD_LIMIT_S + shape_scenario.LEAD_IN_S + REMOVE_WINDOW_S[1] \
    + HEAL_LIMIT_S + 1.0


class Monitor:
    def __init__(self, swarm: Swarm, stroke: shapes.Stroke, rng: random.Random,
                 ball: bool = False) -> None:
        self.stroke = stroke
        count = len(swarm.world.robots)
        self.start_s = swarm.world.time_s + shape_scenario.LEAD_IN_S
        window = BALL_REMOVE_WINDOW_S if ball else REMOVE_WINDOW_S
        self.remove_s = self.start_s + rng.uniform(*window)
        self.victim = rng.randrange(count)
        self.moving = not ball and rng.random() < 0.3  # removed during MOVE instead of SHAPE
        self.injected = rng.randrange(count)
        self.sent = self.twisting = self.removed = False
        self.state_at_removal: int | None = None
        self.healed_s: float | None = None  # one chain again and the cores agree
        self.polygon = shapes.target_polygon(stroke, count - 1, swarm.cfg["geometry"]["pitch"])

    def chain(self, swarm: Swarm) -> list[int] | None:
        """Robot ids head to tail if the survivors are one chain and their cores agree."""
        alive = swarm.alive()
        debug = [swarm.debug.get(r.id) for r in alive]
        if any(d is None or d.chain_size != len(alive) for d in debug):
            return None
        order = sorted(alive, key=lambda r: swarm.debug[r.id].index)
        if [swarm.debug[r.id].index for r in order] != list(range(len(alive))):
            return None
        for a, b in zip(order, order[1:], strict=False):  # and the links are really there
            if b.ahead is None or b.ahead.hub is not a:
                return None
        return [r.id for r in order]

    def error_mm(self, swarm: Swarm) -> float | None:
        chain = self.chain(swarm)
        if chain is None:
            return None
        here = np.array([[swarm.world.robots[i].position.x, swarm.world.robots[i].position.y]
                         for i in chain])
        return shapes.fit(here, self.polygon).mean_error_mm

    def __call__(self, swarm: Swarm) -> None:
        now = swarm.world.time_s
        if not self.sent and now >= self.start_s:
            swarm.set_shape(self.stroke, self.injected)
            self.sent = True
        if self.moving:
            on = self.remove_s - 0.4 * MOVE_S <= now < self.remove_s + 0.6 * MOVE_S
            if on != self.twisting:
                self.twisting = on
                swarm.set_twist(MOVE_SPEED_MM_S if on else 0.0, 0.0, 0.0, self.injector(swarm))
        if not self.removed and now >= self.remove_s:
            self.removed = True
            self.state_at_removal = swarm.debug[self.victim].state
            swarm.world.remove(swarm.world.robots[self.victim])
        if self.removed:
            if self.healed_s is None and self.chain(swarm) is not None:
                self.healed_s = now - self.remove_s
            if now >= self.remove_s + HEAL_LIMIT_S:
                swarm.stop = True

    def injector(self, swarm: Swarm) -> int:
        """The phone stays with its robot unless that robot is the one removed."""
        alive = [r.id for r in swarm.alive()]
        return self.injected if self.injected in alive else alive[0]


def ball_inside(swarm: Swarm, chain: list[int] | None) -> bool:
    """The ball lies inside the polygon of the robots, taken in chain order."""
    balls = [b for b in swarm.world.balls if not b.fallen]
    if not balls or chain is None:
        return False
    bx, by = balls[0].position
    pts = [swarm.world.robots[i].position for i in chain]
    inside = False
    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1], strict=False):  # ray casting
        if (y1 > by) != (y2 > by) and bx < x1 + (by - y1) * (x2 - x1) / (y2 - y1):
            inside = not inside
    return inside


def build(cfg: dict[str, Any], seed: int, variant: str = "") -> Swarm:
    rng = random.Random(seed * 7919 + 31)
    if variant == "ball":
        # An open arc a little wider than the ring, with the ball at its centre.
        swarm = Swarm(cfg, seed)
        direction = math.radians(rng.uniform(-180.0, 180.0))
        directions = []
        for _ in range(cfg["swarm"]["design_count"] - 1):
            directions.append(direction)
            direction += math.radians(rng.uniform(22.0, 28.0))
        centre = (rng.uniform(-150.0, 150.0), rng.uniform(-60.0, 60.0))
        shape_scenario.linked_chain(swarm, centre, directions)
        swarm.world.add_ball(*centre)
        name = "ring"
    else:
        name = ("line", "ring", "u", "s")[seed % 4]
        swarm = shape_scenario.build(cfg, seed, name)
        swarm.on_tick.remove(swarm.monitor)  # same start as the shape scenario, another script
    swarm.monitor = Monitor(swarm, shapes.named(name), rng, ball=variant == "ball")
    swarm.on_tick.append(swarm.monitor)
    return swarm


def evaluate(swarm: Swarm) -> tuple[bool, dict[str, Any]]:
    mon: Monitor = swarm.monitor
    error = mon.error_mm(swarm)
    ring = any(d.closure != corelib.NONE for d in swarm.debug.values()
               if d is not None) and all(r.ahead is not None for r in swarm.alive())
    unlinks = sum(1 for e in swarm.world.events if e.kind == "unlink" and e.time_s > 0.01)
    metrics = {
        "shape": "ring" if swarm.world.balls else ("line", "ring", "u", "s")[swarm.seed % 4],
        "removed": mon.victim,
        "removed_at_s": round(mon.remove_s - mon.start_s, 1),
        "state_at_removal": corelib.STATE_NAMES.get(mon.state_at_removal or 0, "?"),
        "healed_s": None if mon.healed_s is None else round(mon.healed_s, 2),
        "error_mm": None if error is None else round(error, 2),
        "closed": ring,
        "ball_inside": ball_inside(swarm, mon.chain(swarm)) if swarm.world.balls else None,
        "falls": swarm.falls(),
        "unlinks": unlinks,
        "min_edge_mm": round(swarm.min_edge_mm, 1),
    }
    passed = (error is not None and error < ERROR_LIMIT_MM and metrics["falls"] == 0
              and ring == mon.stroke.closed and metrics["ball_inside"] is not False)
    return passed, metrics
