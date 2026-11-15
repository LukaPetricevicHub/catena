"""M4: a wandering chain finds a ball, closes a ring round it and carries it 400 mm
(spec 7.6).

    python -m sim.runner lasso --seed 3 --viewer

A linked chain of twelve wanders; a ball lies somewhere on the table, clear of the edges.
Nothing tells the chain where the ball is or to close: the head senses it and does the
rest. Once the ring is closed round the ball, the script does what a person at the phone
would: it turns the ring until shape x points where there is room, and sends a twist along
shape x for as long as 400 mm takes. Passes if the ring closed round the ball in time, its
centre ends within 40 mm of where the twist should bring it, and the ball is still inside."""

from __future__ import annotations

import math
import random
from typing import Any

import numpy as np

from sim import shapes
from sim.scenarios.heal import ball_inside
from sim.scenarios.shape import HOLD_LIMIT_S, linked_chain, positions
from sim.swarm import Swarm

CATCH_LIMIT_S = 360.0   # from the start to a closed ring round the ball
DISTANCE_MM = 400.0
SPEED_MM_S = 30.0
ARRIVE_LIMIT_MM = 40.0
READY_ERROR_MM = 8.0    # the carry starts once the ring has been this good ...
READY_S = 3.0           # ... for this long
AIM_LIMIT_RAD = math.radians(3.0)
AIM_RATE_RAD_S = math.radians(10.0)
AIM_LIMIT_S = 60.0
REST_S = 5.0
ROOM_SLACK_MM = 30.0    # kept free between the carried ring and the edge margin
BALL_EDGE_MM = 250.0    # the ball lies at least this far from the edges: a ring fits round it
BALL_CLEAR_MM = 220.0   # ... and this far from every robot at the start
DURATION_S = HOLD_LIMIT_S + CATCH_LIMIT_S + AIM_LIMIT_S + DISTANCE_MM / SPEED_MM_S + REST_S + 10.0


class Monitor:
    def __init__(self, swarm: Swarm, rng: random.Random) -> None:
        count = len(swarm.world.robots)
        self.ids = list(range(count))
        self.polygon = shapes.target_polygon(shapes.named("ring"), count,
                                             swarm.cfg["geometry"]["pitch"])
        self.robot_id = rng.randrange(count)
        self.start_s = swarm.world.time_s
        self.ball_start = tuple(swarm.world.balls[0].position)
        self.good_since: float | None = None
        self.caught_s: float | None = None   # ring closed round the ball and in shape
        self.heading: float | None = None    # where the carry goes, world frame
        self.turning = 0.0
        self.move_s: float | None = None
        self.stop_s: float | None = None
        self.goal: tuple[float, float] | None = None
        self.worst_mm = 0.0

    def room(self, swarm: Swarm, centre: tuple[float, float], angle: float) -> float:
        """How far the ring's centre can go along `angle` before the ring nears an edge."""
        cfg = swarm.cfg
        keep = (float(np.hypot(*self.polygon.T).max()) + cfg["geometry"]["base"][
            "envelope_diameter"] / 2 + cfg["table"]["edge_margin"] + ROOM_SLACK_MM)
        reach = math.inf
        for half, at, step in ((swarm.world.half_w, centre[0], math.cos(angle)),
                               (swarm.world.half_h, centre[1], math.sin(angle))):
            if abs(step) > 1e-9:
                reach = min(reach, ((half - keep) * math.copysign(1.0, step) - at) / step)
        return reach

    def __call__(self, swarm: Swarm) -> None:
        now = swarm.world.time_s
        ball = swarm.world.balls[0]
        if ball.fallen:
            swarm.stop = True
            return
        closed = swarm.world.robots[0].ahead is not None
        if self.move_s is None:
            if not closed:
                self.good_since = None
                if now - self.start_s > CATCH_LIMIT_S:
                    swarm.stop = True
                return
            fit = shapes.fit(positions(swarm), self.polygon)
            holds = ball_inside(swarm, self.ids) and fit.mean_error_mm < READY_ERROR_MM
            if self.caught_s is None and holds:
                self.caught_s = now - self.start_s
                angles = [math.radians(a) for a in range(0, 360, 45)]
                self.heading = max(angles, key=lambda a: self.room(swarm, fit.centroid, a))
            if self.caught_s is None:
                if now - self.start_s > CATCH_LIMIT_S:
                    swarm.stop = True
                return
            off = math.remainder(fit.rotation_rad - self.heading, 2 * math.pi)
            aimed = abs(off) < AIM_LIMIT_RAD
            want = 0.0 if aimed else -math.copysign(AIM_RATE_RAD_S, off)
            if want != self.turning:
                self.turning = want
                swarm.set_twist(0.0, 0.0, want, self.robot_id)
            good = aimed and fit.mean_error_mm < READY_ERROR_MM
            self.good_since = (self.good_since if self.good_since is not None else now) if good \
                else None
            if good and now - self.good_since >= READY_S:
                self.move_s = now
                self.goal = (fit.centroid[0] + DISTANCE_MM * math.cos(fit.rotation_rad),
                             fit.centroid[1] + DISTANCE_MM * math.sin(fit.rotation_rad))
                swarm.set_twist(SPEED_MM_S, 0.0, 0.0, self.robot_id)
            elif now - self.start_s > self.caught_s + AIM_LIMIT_S:
                swarm.stop = True
            return
        self.worst_mm = max(self.worst_mm, shapes.fit(positions(swarm), self.polygon).mean_error_mm)
        if self.stop_s is None and now - self.move_s >= DISTANCE_MM / SPEED_MM_S:
            self.stop_s = now
            swarm.set_twist(0.0, 0.0, 0.0, self.robot_id)
        if self.stop_s is not None and now - self.stop_s >= REST_S:
            swarm.stop = True


def build(cfg: dict[str, Any], seed: int) -> Swarm:
    swarm = Swarm(cfg, seed)
    rng = random.Random(seed * 7919 + 37)
    count = cfg["swarm"]["design_count"]
    half_w, half_h = (v / 2 for v in cfg["table"]["size"])
    direction = math.radians(rng.uniform(-15.0, 15.0)) + rng.choice([0.0, math.pi])
    directions = []
    for _ in range(count - 1):
        directions.append(direction)
        direction += math.radians(rng.uniform(-6.0, 6.0))
    linked_chain(swarm, (rng.uniform(-120.0, 120.0), rng.uniform(-250.0, 250.0)), directions)
    while True:
        bx = rng.uniform(-(half_w - BALL_EDGE_MM), half_w - BALL_EDGE_MM)
        by = rng.uniform(-(half_h - BALL_EDGE_MM), half_h - BALL_EDGE_MM)
        if all(math.hypot(r.position.x - bx, r.position.y - by) >= BALL_CLEAR_MM
               for r in swarm.world.robots):
            break
    swarm.world.add_ball(bx, by)
    swarm.monitor = Monitor(swarm, rng)
    swarm.on_tick.append(swarm.monitor)
    return swarm


def evaluate(swarm: Swarm) -> tuple[bool, dict[str, Any]]:
    mon: Monitor = swarm.monitor
    ball = swarm.world.balls[0]
    fit = shapes.fit(positions(swarm), mon.polygon)
    arrived = None
    if mon.goal is not None and mon.stop_s is not None:
        arrived = math.hypot(fit.centroid[0] - mon.goal[0], fit.centroid[1] - mon.goal[1])
    inside = not ball.fallen and ball_inside(swarm, mon.ids)
    closed = swarm.world.robots[0].ahead is not None
    metrics = {
        "caught_s": None if mon.caught_s is None else round(mon.caught_s, 1),
        "arrival_error_mm": None if arrived is None else round(arrived, 1),
        "ball_inside": inside,
        "ball_carried_mm": round(math.hypot(ball.position.x - mon.ball_start[0],
                                            ball.position.y - mon.ball_start[1]), 1),
        "closed": closed,
        "worst_shape_error_mm": round(mon.worst_mm, 2),
        "ball_fell": ball.fallen,
        "falls": swarm.falls(),
        "unlinks": sum(1 for e in swarm.world.events if e.kind == "unlink" and e.time_s > 0.01),
        "min_edge_mm": round(swarm.min_edge_mm, 1),
    }
    passed = (mon.caught_s is not None and closed and inside and arrived is not None
              and arrived < ARRIVE_LIMIT_MM and metrics["falls"] == 0)
    return passed, metrics
