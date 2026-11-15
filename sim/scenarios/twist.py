"""A ring is told to move 400 mm along its own x axis (spec 7.5).

    python -m sim.runner twist --seed 2 --viewer

The chain starts as an open arc on the left of the table, takes the ring stroke and closes
itself. The script then does what a person at the phone would: it turns the ring with the
rotate slider until shape x lies along the table (400 mm does not fit across it), and then
hands one robot a twist along shape x for as long as 400 mm takes. Passes
if the ring's centre ends within 40 mm of where that should bring it and the shape error
stays under 15 mm on the way."""

from __future__ import annotations

import math
import random
from typing import Any

from sim import shapes
from sim.scenarios.shape import LEAD_IN_S, linked_chain, positions
from sim.swarm import Swarm

DISTANCE_MM = 400.0
SPEED_MM_S = 40.0
ARRIVE_LIMIT_MM = 40.0
SHAPE_LIMIT_MM = 15.0
READY_ERROR_MM = 6.0  # the move starts once the closed ring has been this good ...
READY_S = 3.0         # ... for this long
FORM_LIMIT_S = 120.0  # ... or the run fails
AIM_LIMIT_RAD = math.radians(3.0)  # shape x is turned to within this of table x
AIM_RATE_RAD_S = math.radians(10.0)
REST_S = 5.0          # time to come to rest after the twist ends
DURATION_S = 5.0 + LEAD_IN_S + FORM_LIMIT_S + DISTANCE_MM / SPEED_MM_S + REST_S


class Monitor:
    def __init__(self, swarm: Swarm, robot_id: int) -> None:
        self.stroke = shapes.named("ring")
        self.robot_id = robot_id
        self.polygon = shapes.target_polygon(self.stroke, len(swarm.world.robots),
                                             swarm.cfg["geometry"]["pitch"])
        self.start_s = swarm.world.time_s + LEAD_IN_S
        self.sent = False
        self.good_since: float | None = None
        self.turning = 0.0  # rotate rate being commanded
        self.move_s: float | None = None  # when the twist was handed over
        self.stop_s: float | None = None
        self.goal: tuple[float, float] | None = None
        self.worst_mm = 0.0

    def __call__(self, swarm: Swarm) -> None:
        now = swarm.world.time_s
        if not self.sent:
            if now >= self.start_s:
                swarm.set_shape(self.stroke, self.robot_id)
                self.sent = True
            return
        fit = shapes.fit(positions(swarm), self.polygon)
        if self.move_s is None:
            closed = swarm.world.robots[0].ahead is not None
            aimed = abs(fit.rotation_rad) < AIM_LIMIT_RAD
            want = 0.0 if aimed or not closed else -math.copysign(AIM_RATE_RAD_S, fit.rotation_rad)
            if want != self.turning:
                self.turning = want
                swarm.set_twist(0.0, 0.0, want, self.robot_id)
            good = closed and aimed and fit.mean_error_mm < READY_ERROR_MM
            self.good_since = (self.good_since if self.good_since is not None else now) if good \
                else None
            if good and now - self.good_since >= READY_S:
                self.move_s = now
                self.goal = (fit.centroid[0] + DISTANCE_MM * math.cos(fit.rotation_rad),
                             fit.centroid[1] + DISTANCE_MM * math.sin(fit.rotation_rad))
                swarm.set_twist(SPEED_MM_S, 0.0, 0.0, self.robot_id)
            elif now - self.start_s > FORM_LIMIT_S:
                swarm.stop = True
            return
        self.worst_mm = max(self.worst_mm, fit.mean_error_mm)
        if self.stop_s is None and now - self.move_s >= DISTANCE_MM / SPEED_MM_S:
            self.stop_s = now
            swarm.set_twist(0.0, 0.0, 0.0, self.robot_id)
        if self.stop_s is not None and now - self.stop_s >= REST_S:
            swarm.stop = True


def build(cfg: dict[str, Any], seed: int) -> Swarm:
    swarm = Swarm(cfg, seed)
    rng = random.Random(seed * 7919 + 23)
    count = cfg["swarm"]["design_count"]
    # An arc a little wider than the ring, robot 0 where the stroke starts.
    direction = math.radians(-75.0 + rng.uniform(-5.0, 5.0))
    directions = []
    for _ in range(count - 1):
        directions.append(direction)
        direction += math.radians(rng.uniform(22.0, 28.0))
    linked_chain(swarm, (-200.0 + rng.uniform(-30.0, 30.0), rng.uniform(-30.0, 30.0)), directions)
    swarm.monitor = Monitor(swarm, rng.randrange(count))
    swarm.on_tick.append(swarm.monitor)
    return swarm


def evaluate(swarm: Swarm) -> tuple[bool, dict[str, Any]]:
    mon: Monitor = swarm.monitor
    fit = shapes.fit(positions(swarm), mon.polygon)
    arrived = None
    if mon.goal is not None and mon.stop_s is not None:
        arrived = math.hypot(fit.centroid[0] - mon.goal[0], fit.centroid[1] - mon.goal[1])
    metrics = {
        "formed_s": None if mon.move_s is None else round(mon.move_s - mon.start_s, 2),
        "arrival_error_mm": None if arrived is None else round(arrived, 1),
        "worst_shape_error_mm": round(mon.worst_mm, 2),
        "final_shape_error_mm": round(fit.mean_error_mm, 2),
        "falls": swarm.falls(),
        "links_broken": sum(1 for e in swarm.world.events if e.kind == "unlink"),
        "min_edge_mm": round(swarm.min_edge_mm, 1),
    }
    passed = (arrived is not None and arrived < ARRIVE_LIMIT_MM
              and mon.worst_mm < SHAPE_LIMIT_MM and metrics["falls"] == 0
              and metrics["links_broken"] == 0)
    return passed, metrics
