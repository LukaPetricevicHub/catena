"""M2: twelve robots start at random and must form one chain. Passes if one chain of all
robots exists within the time limit, its indices run 0..n-1 from head to tail and every
robot reports the right count within 5 s of the last link."""

from __future__ import annotations

from typing import Any

from sim import corelib
from sim.swarm import Swarm

FORM_LIMIT_S = 180.0
COUNT_LIMIT_S = 5.0
CHASE_LIMIT_S = 60.0
STABLE_S = 6.0  # the run stops once the finished chain has held this long
DURATION_S = FORM_LIMIT_S + STABLE_S


def physical_chains(swarm: Swarm) -> list[list[int]]:
    """Robot ids of every chain on the table, head first, from the world's links."""
    chains = []
    for r in swarm.alive():
        if r.ahead is None:
            chain, node = [], r
            while node is not None and node.id not in chain:
                chain.append(node.id)
                node = node.behind.tongue if node.behind else None
            chains.append(chain)
    return chains


class Monitor:
    """Watches a run through the world's truth and each core's debug output."""

    def __init__(self, swarm: Swarm) -> None:
        self.count = len(swarm.world.robots)
        self.complete_since: float | None = None  # the single chain exists since this time
        self.settled_s: float | None = None  # cores agree on index and count since this time
        self.chase_since: dict[tuple[int, int], float] = {}
        self.longest_chase_s = 0.0
        self.max_chains_seen = 0

    def __call__(self, swarm: Swarm) -> None:
        now = swarm.world.time_s
        chains = physical_chains(swarm)
        self.max_chains_seen = max(self.max_chains_seen, len(chains))
        if len(chains) == 1 and len(chains[0]) == self.count:
            if self.complete_since is None:
                self.complete_since = now
                self.settled_s = None
            if self.settled_s is None and self.cores_agree(swarm, chains[0]):
                self.settled_s = now
            if now - self.complete_since >= STABLE_S:
                swarm.stop = True
        else:
            self.complete_since = None
            self.settled_s = None
        self.watch_chases(swarm, chains, now)

    def cores_agree(self, swarm: Swarm, chain: list[int]) -> bool:
        return all(swarm.debug[rid].index == i and swarm.debug[rid].chain_size == len(chain)
                   for i, rid in enumerate(chain))

    def watch_chases(self, swarm: Swarm, chains: list[list[int]], now: float) -> None:
        """Two chains chase each other while each head is after a robot of the other."""
        chain_of = {rid: i for i, chain in enumerate(chains) for rid in chain}
        after = set()
        for i, chain in enumerate(chains):
            dbg = swarm.debug.get(chain[0])
            if dbg and dbg.state in (corelib.STATE_SEEK, corelib.STATE_DOCK):
                other = chain_of.get(dbg.target)
                if other is not None and other != i:
                    after.add((chain[0], chains[other][0]))
        mutual = {(a, b) for a, b in after if a < b and (b, a) in after}
        self.chase_since = {pair: self.chase_since.get(pair, now) for pair in mutual}
        for since in self.chase_since.values():
            self.longest_chase_s = max(self.longest_chase_s, now - since)

    def chase_at_end_s(self, now: float) -> float:
        return max((now - since for since in self.chase_since.values()), default=0.0)


def build(cfg: dict[str, Any], seed: int) -> Swarm:
    swarm = Swarm(cfg, seed)
    swarm.scatter(cfg["swarm"]["design_count"])
    swarm.monitor = Monitor(swarm)
    swarm.on_tick.append(swarm.monitor)
    return swarm


def evaluate(swarm: Swarm) -> tuple[bool, dict[str, Any]]:
    mon: Monitor = swarm.monitor
    now = swarm.world.time_s
    chains = physical_chains(swarm)
    links = [e for e in swarm.world.events if e.kind == "link"]
    formed_s = links[-1].time_s if mon.complete_since is not None and links else None
    count_delay = (max(0.0, mon.settled_s - formed_s)
                   if formed_s is not None and mon.settled_s is not None else None)
    one_chain = formed_s is not None and formed_s <= FORM_LIMIT_S
    chain = chains[0] if chains else []
    stats = swarm.stats
    metrics = {
        "one_chain": one_chain,
        "formed_s": None if formed_s is None else round(formed_s, 2),
        "count_delay_s": None if count_delay is None else round(count_delay, 2),
        "index_ok": one_chain and mon.cores_agree(swarm, chain),
        "chains_at_end": len(chains),
        "largest_chain": max((len(c) for c in chains), default=0),
        "chase_at_end_s": round(mon.chase_at_end_s(now), 2),
        "longest_chase_s": round(mon.longest_chase_s, 2),
        "falls": swarm.falls(),
        "links_formed": len(links),
        "links_broken": sum(1 for e in swarm.world.events if e.kind == "unlink"),
        "flash_spoiled_frac": round(stats["spoiled"] / max(1, stats["flashes"]), 3),
        "rx_overflow": stats["overflow"],
    }
    passed = (one_chain and metrics["index_ok"] and count_delay is not None
              and count_delay <= COUNT_LIMIT_S and metrics["falls"] == 0)
    return passed, metrics
