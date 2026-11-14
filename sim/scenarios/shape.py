"""M3: a linked chain is handed a stroke and must take its shape (spec 7.4).

    python -m sim.runner shape:ring --seed 2 --viewer      (line, ring, u or s)

The chain starts linked, in a gentle random curve. One robot is handed the stroke; the
others get it from their neighbours. Passes if the mean distance of the robots from their
vertices is under the limit 60 s later, nothing fell or came apart, and a closed shape has
closed itself."""

from __future__ import annotations

import math
import random
from typing import Any

import numpy as np

from sim import corelib, shapes
from sim.swarm import Swarm

HOLD_LIMIT_S = 5.0  # the chain is held in the hand at most this long
# On the table this long before the stroke is handed over. A robot needs about 2 s of flash
# bearings of the robot behind it before it can hold a shape (decision D34).
LEAD_IN_S = 3.0
SETTLE_S = 60.0
ERROR_LIMIT_MM = 12.0
DURATION_S = HOLD_LIMIT_S + LEAD_IN_S + SETTLE_S  # upper bound; the monitor stops the run
SAMPLE_S = 0.25
START_EDGE_MM = 150.0  # every robot starts at least this far from the table edge


def linked_chain(swarm: Swarm, start: tuple[float, float], directions: list[float]) -> None:
    """Robots joined into one chain, robot 0 the head. directions[i] is where robot i + 1's
    tongue points, i.e. the direction from robot i + 1 to robot i. The chain is then moved
    so its centroid lies at `start`, and held in the hand until the cores know their links."""
    pitch = swarm.cfg["geometry"]["pitch"]
    pts = [(0.0, 0.0)]
    for d in directions:
        x, y = pts[-1]
        pts.append((x - pitch * math.cos(d), y - pitch * math.sin(d)))
    cx = sum(p[0] for p in pts) / len(pts) - start[0]
    cy = sum(p[1] for p in pts) / len(pts) - start[1]
    for i, (x, y) in enumerate(pts):
        swarm.add_robot(x - cx, y - cy, directions[max(i - 1, 0)])
    robots = swarm.world.robots
    for i in range(1, len(robots)):
        swarm.world.link(robots[i], robots[i - 1])
    for r in robots:
        swarm.world.set_grounded(r, False)
    while swarm.world.time_s < HOLD_LIMIT_S and not all(
            swarm.debug.get(r.id) and swarm.debug[r.id].chain_size == len(robots) for r in robots):
        swarm.advance()
    for r in robots:
        swarm.world.set_grounded(r, True)


def positions(swarm: Swarm) -> np.ndarray:
    return np.array([[r.position.x, r.position.y] for r in swarm.world.robots])


class Monitor:
    """Hands the stroke over LEAD_IN_S after the chain is put down, then samples the shape
    error and stops the run SETTLE_S later."""

    def __init__(self, swarm: Swarm, stroke: shapes.Stroke, robot_id: int) -> None:
        self.stroke = stroke
        self.robot_id = robot_id
        self.start_s = swarm.world.time_s + LEAD_IN_S
        self.sent = False
        self.heard_by_all_s: float | None = None
        self.polygon = shapes.target_polygon(stroke, len(swarm.world.robots),
                                             swarm.cfg["geometry"]["pitch"])
        self.samples: list[tuple[float, float]] = []  # (time, mean vertex error)
        self.closed_s: float | None = None
        self._next = self.start_s

    def fit(self, swarm: Swarm) -> shapes.Fit:
        return shapes.fit(positions(swarm), self.polygon)

    def __call__(self, swarm: Swarm) -> None:
        now = swarm.world.time_s
        if not self.sent and now >= self.start_s:
            swarm.set_shape(self.stroke, self.robot_id)
            self.sent = True
        if not self.sent:
            return
        if self.heard_by_all_s is None and all(
                d.state in (corelib.STATE_SHAPE, corelib.STATE_MOVE)
                for d in swarm.debug.values()):
            self.heard_by_all_s = now - self.start_s
        if self.closed_s is None and swarm.world.robots[0].ahead is not None:
            self.closed_s = now - self.start_s
        if now >= self._next:
            self._next += SAMPLE_S
            self.samples.append((now - self.start_s, self.fit(swarm).mean_error_mm))
        if now >= self.start_s + SETTLE_S:
            swarm.stop = True


def build(cfg: dict[str, Any], seed: int, shape: str = "line") -> Swarm:
    swarm = Swarm(cfg, seed)
    rng = random.Random(seed * 7919 + 17)
    count = cfg["swarm"]["design_count"]
    pitch = cfg["geometry"]["pitch"]
    half_w, half_h = (v / 2 - START_EDGE_MM for v in cfg["table"]["size"])
    while True:
        # A straight chain of twelve only fits along the table, so the curve starts that way.
        direction = math.radians(rng.uniform(-25.0, 25.0)) + rng.choice([0.0, math.pi])
        directions, pts = [], [(0.0, 0.0)]
        for _ in range(count - 1):
            directions.append(direction)
            pts.append((pts[-1][0] - pitch * math.cos(direction),
                        pts[-1][1] - pitch * math.sin(direction)))
            direction += math.radians(rng.uniform(-18.0, 18.0))
        centre = (rng.uniform(-60.0, 60.0), rng.uniform(-40.0, 40.0))
        mean = (sum(p[0] for p in pts) / count, sum(p[1] for p in pts) / count)
        if all(abs(x - mean[0] + centre[0]) < half_w and abs(y - mean[1] + centre[1]) < half_h
               for x, y in pts):
            break
    linked_chain(swarm, centre, directions)
    swarm.monitor = Monitor(swarm, shapes.named(shape), rng.randrange(count))
    swarm.on_tick.append(swarm.monitor)
    return swarm


def evaluate(swarm: Swarm) -> tuple[bool, dict[str, Any]]:
    mon: Monitor = swarm.monitor
    final = mon.fit(swarm)
    over = [t for t, e in mon.samples if e >= ERROR_LIMIT_MM]
    settled_s = over[-1] + SAMPLE_S if over else 0.0
    ring = swarm.world.robots[0].ahead is not None
    metrics = {
        "error_mm": round(final.mean_error_mm, 2),
        "max_vertex_error_mm": round(final.max_error_mm, 2),
        "under_limit_since_s": (round(settled_s, 2) if final.mean_error_mm < ERROR_LIMIT_MM
                                else None),
        "error_at_s": {round(t): round(e, 1) for t, e in mon.samples
                       if abs(t / 10.0 - round(t / 10.0)) < 1e-6},
        "heard_by_all_s": None if mon.heard_by_all_s is None else round(mon.heard_by_all_s, 2),
        "closed": ring,
        "closed_s": None if mon.closed_s is None else round(mon.closed_s, 2),
        "falls": swarm.falls(),
        "links_broken": sum(1 for e in swarm.world.events if e.kind == "unlink"),
        "min_edge_mm": round(swarm.min_edge_mm, 1),
    }
    passed = (final.mean_error_mm < ERROR_LIMIT_MM and metrics["falls"] == 0
              and metrics["links_broken"] == 0 and ring == mon.stroke.closed)
    return passed, metrics
