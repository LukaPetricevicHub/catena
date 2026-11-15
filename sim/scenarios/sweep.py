"""A U-shaped scoop sweeps eight blocks toward a table corner and parks there (spec 7.7).

    python -m sim.runner sweep --seed 3 --viewer

The chain starts linked, roughly in a U, some way from a corner picked by the seed; eight
blocks lie between it and the corner. The script does what a person at the phone would: it
hands over the U stroke, turns the scoop with the rotate slider until its opening faces
the corner, and sends one sweep along the opening. From there the chain is on its own: it
turns along the first edge its robots meet and stops at the second. Passes if every robot
reports the sweep as parked, a robot stands against each of the corner's two edges, at
least 80% of the blocks lie inside the scoop, no robot fell and no link broke."""

from __future__ import annotations

import math
import random
from typing import Any

import numpy as np

from sim import corelib, shapes
from sim.scenarios.shape import LEAD_IN_S, linked_chain, positions
from sim.swarm import Swarm

BLOCKS = 8
BLOCKS_INSIDE_FRACTION = 0.8
AGAINST_MM = 40.0        # "against an edge": the nearest base rim is no further from it
SPEED_MM_S = 30.0
READY_ERROR_MM = 8.0     # the sweep starts once the U has been this good ...
READY_S = 3.0            # ... for this long
FORM_LIMIT_S = 120.0     # ... or the run fails
AIM_LIMIT_RAD = math.radians(3.0)
AIM_RATE_RAD_S = math.radians(6.0)
SWEEP_LIMIT_S = 120.0    # from the sweep command to a parked scoop
REST_S = 8.0             # time to come to rest once parked
START_MM = (680.0, 760.0)         # scoop centre from the corner
START_DEG = (33.0, 45.0)          # ... at this angle from the long edge
MISAIM_DEG = 15.0                 # the scoop starts turned off the corner by up to this
BLOCK_LANE_MM = 55.0              # blocks lie this far either side of the line to the corner
BLOCK_AHEAD_MM = 20.0             # ... from this far in front of the scoop's mouth
BLOCK_STOP_MM = 300.0             # ... to this far short of the corner
BLOCK_SPACING_MM = 32.0
DURATION_S = 5.0 + LEAD_IN_S + FORM_LIMIT_S + SWEEP_LIMIT_S + REST_S


def inside(point: tuple[float, float], pts: list[tuple[float, float]]) -> bool:
    """Ray casting: the point lies inside the polygon (here the scoop, closed across its mouth)."""
    x, y = point
    hit = False
    for (x1, y1), (x2, y2) in zip(pts, pts[1:] + pts[:1], strict=False):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


class Monitor:
    def __init__(self, swarm: Swarm, corner: tuple[float, float], robot_id: int) -> None:
        self.stroke = shapes.named("u")
        self.corner = corner
        self.robot_id = robot_id
        self.polygon = shapes.target_polygon(self.stroke, len(swarm.world.robots),
                                             swarm.cfg["geometry"]["pitch"])
        self.start_s = swarm.world.time_s + LEAD_IN_S
        self.sent = False
        self.good_since: float | None = None
        self.turning = 0.0
        self.sweep_s: float | None = None   # when the sweep was handed over
        self.first_edge_s: float | None = None
        self.parked_s: float | None = None

    def __call__(self, swarm: Swarm) -> None:
        now = swarm.world.time_s
        if not self.sent:
            if now >= self.start_s:
                swarm.set_shape(self.stroke, self.robot_id)
                self.sent = True
            return
        if self.sweep_s is None:
            fit = shapes.fit(positions(swarm), self.polygon)
            want_dir = math.atan2(self.corner[1] - fit.centroid[1],
                                  self.corner[0] - fit.centroid[0])
            # The U's opening is shape +y.
            off = math.remainder(fit.rotation_rad + math.pi / 2 - want_dir, 2 * math.pi)
            shaped = fit.mean_error_mm < READY_ERROR_MM
            aimed = abs(off) < AIM_LIMIT_RAD
            want = 0.0 if aimed or not shaped else -math.copysign(AIM_RATE_RAD_S, off)
            if want != self.turning:
                self.turning = want
                swarm.set_twist(0.0, 0.0, want, self.robot_id)
            good = shaped and aimed
            self.good_since = (self.good_since if self.good_since is not None else now) if good \
                else None
            if good and now - self.good_since >= READY_S:
                self.sweep_s = now
                swarm.set_sweep(0.0, SPEED_MM_S, self.robot_id)
            elif now - self.start_s > FORM_LIMIT_S:
                swarm.stop = True
            return
        stages = [d.sweep for d in swarm.debug.values()]
        if self.first_edge_s is None and any(s >= corelib.SWEEP_SLIDE_HEAD for s in stages):
            self.first_edge_s = now
        if self.parked_s is None and all(s == corelib.SWEEP_PARKED for s in stages):
            self.parked_s = now
        if self.parked_s is not None and now - self.parked_s >= REST_S:
            swarm.stop = True
        if self.parked_s is None and now - self.sweep_s > SWEEP_LIMIT_S:
            swarm.stop = True


def build(cfg: dict[str, Any], seed: int) -> Swarm:
    swarm = Swarm(cfg, seed)
    rng = random.Random(seed * 7919 + 41)
    count = cfg["swarm"]["design_count"]
    half_w, half_h = (v / 2 for v in cfg["table"]["size"])
    sx, sy = rng.choice([(-1, -1), (-1, 1), (1, -1), (1, 1)])
    corner = (sx * half_w, sy * half_h)
    reach = rng.uniform(*START_MM)
    beta = math.radians(rng.uniform(*START_DEG))
    centre = (corner[0] - sx * reach * math.cos(beta), corner[1] - sy * reach * math.sin(beta))
    aim = math.atan2(corner[1] - centre[1], corner[0] - centre[0])

    # The chain is put down roughly as the U it will hold, its opening roughly at the corner.
    polygon = shapes.target_polygon(shapes.named("u"), count, cfg["geometry"]["pitch"])
    turn = aim - math.pi / 2 + math.radians(rng.uniform(-MISAIM_DEG, MISAIM_DEG))
    c, s = math.cos(turn), math.sin(turn)
    laid = polygon @ np.array([[c, s], [-s, c]])
    directions = [math.atan2(laid[i][1] - laid[i + 1][1], laid[i][0] - laid[i + 1][0])
                  + math.radians(rng.uniform(-6.0, 6.0)) for i in range(count - 1)]
    linked_chain(swarm, centre, directions)

    mouth = float(polygon[:, 1].max()) + cfg["geometry"]["base"]["envelope_diameter"] / 2
    placed: list[tuple[float, float]] = []
    while len(placed) < BLOCKS:
        along = rng.uniform(mouth + BLOCK_AHEAD_MM, reach - BLOCK_STOP_MM)
        across = rng.uniform(-BLOCK_LANE_MM, BLOCK_LANE_MM)
        x = centre[0] + along * math.cos(aim) - across * math.sin(aim)
        y = centre[1] + along * math.sin(aim) + across * math.cos(aim)
        if all(math.hypot(x - px, y - py) >= BLOCK_SPACING_MM for px, py in placed):
            placed.append((x, y))
            swarm.world.add_block(x, y, rng.uniform(0.0, math.pi / 2))
    swarm.monitor = Monitor(swarm, corner, rng.randrange(count))
    swarm.on_tick.append(swarm.monitor)
    return swarm


def evaluate(swarm: Swarm) -> tuple[bool, dict[str, Any]]:
    mon: Monitor = swarm.monitor
    world = swarm.world
    robots = swarm.alive()
    base_r = swarm.cfg["geometry"]["base"]["envelope_diameter"] / 2
    # The two edges of the corner: the gap between each and the nearest base rim.
    gaps = [min(abs(mon.corner[axis] - r.position[axis]) for r in robots) - base_r
            for axis in (0, 1)] if robots else [math.inf, math.inf]
    order = sorted(world.robots, key=lambda r: swarm.debug[r.id].index)
    scoop = [(r.position.x, r.position.y) for r in order]
    blocks_inside = sum(1 for b in world.blocks
                        if not b.fallen and inside((b.position.x, b.position.y), scoop))
    fit = shapes.fit(positions(swarm), mon.polygon)
    metrics = {
        "formed_s": None if mon.sweep_s is None else round(mon.sweep_s - mon.start_s, 1),
        "first_edge_s": None if mon.first_edge_s is None or mon.sweep_s is None
        else round(mon.first_edge_s - mon.sweep_s, 1),
        "parked_s": None if mon.parked_s is None or mon.sweep_s is None
        else round(mon.parked_s - mon.sweep_s, 1),
        "parked": mon.parked_s is not None,
        "edge_gap_mm": [round(g, 1) for g in gaps],
        "blocks_inside": blocks_inside,
        "blocks_fell": sum(1 for b in world.blocks if b.fallen),
        "shape_error_mm": round(fit.mean_error_mm, 1),
        "falls": swarm.falls(),
        "links_broken": sum(1 for e in world.events if e.kind == "unlink"),
        "min_edge_mm": round(swarm.min_edge_mm, 1),
    }
    passed = (metrics["parked"] and max(gaps) <= AGAINST_MM
              and blocks_inside >= BLOCKS_INSIDE_FRACTION * BLOCKS
              and metrics["falls"] == 0 and metrics["links_broken"] == 0)
    return passed, metrics
