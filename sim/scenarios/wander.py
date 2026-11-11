"""M1: twelve robots wander on the table. Passes if none falls off."""

from __future__ import annotations

from typing import Any

from sim.swarm import Swarm

DURATION_S = 120.0


def build(cfg: dict[str, Any], seed: int) -> Swarm:
    swarm = Swarm(cfg, seed)
    swarm.scatter(cfg["swarm"]["design_count"])
    return swarm


def evaluate(swarm: Swarm) -> tuple[bool, dict[str, Any]]:
    stats = swarm.stats
    metrics = {
        "falls": swarm.falls(),
        "min_edge_distance_mm": round(swarm.min_edge_mm, 2),
        "min_path_mm": round(min(swarm.path_mm), 1),
        "mean_path_mm": round(sum(swarm.path_mm) / len(swarm.path_mm), 1),
        "links_formed": sum(1 for e in swarm.world.events if e.kind == "link"),
        "flashes": stats["flashes"],
        "flash_spoiled_frac": round(stats["spoiled"] / max(1, stats["flashes"]), 3),
        "frames_dropped": stats["dropped"],
        "rx_overflow": stats["overflow"],
    }
    return metrics["falls"] == 0, metrics
