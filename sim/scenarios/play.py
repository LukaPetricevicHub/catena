"""A linked chain waiting for the phone: nothing is handed to it and nothing is judged.

    make sim S=play ARGS=--serve        then open the page it prints on a phone
"""

from __future__ import annotations

import math
import random
from typing import Any

from sim.scenarios.shape import linked_chain
from sim.swarm import Swarm

DURATION_S = 3600.0


def build(cfg: dict[str, Any], seed: int) -> Swarm:
    swarm = Swarm(cfg, seed)
    rng = random.Random(seed * 7919 + 29)
    direction = math.radians(rng.uniform(-10.0, 10.0))
    directions = []
    for _ in range(cfg["swarm"]["design_count"] - 1):
        directions.append(direction)
        direction += math.radians(rng.uniform(-8.0, 8.0))
    linked_chain(swarm, (0.0, 0.0), directions)
    return swarm


def evaluate(swarm: Swarm) -> tuple[bool, dict[str, Any]]:
    return swarm.falls() == 0, {"falls": swarm.falls()}
