"""Sensor and radio models (spec sections 5 and 8). They turn the world's truth into what
one robot's hardware would report, with the noise from config/robot.yaml."""

from __future__ import annotations

import math
import random
from dataclasses import dataclass
from typing import Any

from sim.world import Robot, World, wrap_pi


@dataclass
class FlashSample:
    """What a listener's six receivers saw of one flash, plus the truth for tests."""

    ir: list[float] | None  # None: no clean sample (out of range or spoiled by a collision)
    true_bearing: float
    true_range: float


class FlashModel:
    """Six cosine-lobe receivers and the inverse-square law.

    Noise is drawn on bearing and range with the configured sigmas and then rendered into
    the six readings, so core's estimator recovers exactly the configured noise.
    """

    def __init__(self, cfg: dict[str, Any], rng: random.Random) -> None:
        sensing = cfg["sensing"]
        self.rng = rng
        self.count = sensing["ir_pairs"]
        self.max_range = sensing["ir_range"]
        self.reference = sensing["ir_reference_range"]
        self.bearing_sigma = math.radians(sensing["ir_bearing_sigma_deg"])
        self.range_sigma = sensing["ir_range_sigma_frac"]
        self.angles = [2.0 * math.pi * k / self.count for k in range(self.count)]

    def sample(
        self, listener: tuple[float, float, float], sender: tuple[float, float], clean: bool
    ) -> FlashSample:
        x, y, heading = listener
        dx, dy = sender[0] - x, sender[1] - y
        true_range = math.hypot(dx, dy)
        true_bearing = wrap_pi(math.atan2(dy, dx) - heading)
        if not clean or true_range > self.max_range:
            return FlashSample(None, true_bearing, true_range)
        bearing = true_bearing + self.rng.gauss(0.0, self.bearing_sigma)
        rng = max(1.0, true_range * (1.0 + self.rng.gauss(0.0, self.range_sigma)))
        scale = (self.reference / rng) ** 2
        ir = [max(0.0, math.cos(bearing - a)) * scale for a in self.angles]
        return FlashSample(ir, true_bearing, true_range)


class BodySensors:
    """Edge sensors, gyro, ring angle sensor, hall sensor and wheel ticks of one robot."""

    def __init__(self, cfg: dict[str, Any], rng: random.Random) -> None:
        sensing = cfg["sensing"]
        self.rng = rng
        self.edge_x, self.edge_y = sensing["edge_sensor_xy"]
        drift = math.radians(sensing["gyro_drift_deg_per_min"]) / 60.0
        self.gyro_bias = rng.uniform(-drift, drift)
        self.ring_sigma = math.radians(sensing["ring_angle_sigma_deg"])
        wheel = cfg["geometry"]["base"]["wheel_diameter"]
        self.mm_per_tick = math.pi * wheel / cfg["drive"]["wheel_ticks_per_rev"]

    def table(self, world: World, robot: Robot) -> tuple[bool, bool]:
        c, s = math.cos(robot.heading), math.sin(robot.heading)
        x, y = robot.position
        seen = []
        for side in (+1.0, -1.0):
            ex, ey = self.edge_x, side * self.edge_y
            seen.append(world.on_table(x + c * ex - s * ey, y + s * ex + c * ey))
        return seen[0], seen[1]

    def gyro(self, robot: Robot) -> float:
        return robot.base.angular_velocity + self.gyro_bias

    def ring_angle(self, robot: Robot) -> float:
        return wrap_pi(robot.ring_angle + self.rng.gauss(0.0, self.ring_sigma))

    def ticks(self, robot: Robot) -> tuple[int, int]:
        return (int(robot.wheel_travel[0] / self.mm_per_tick),
                int(robot.wheel_travel[1] / self.mm_per_tick))

