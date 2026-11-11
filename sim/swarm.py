"""The simulated swarm: the physics world, one core instance per robot, and the sensor and
radio models between them. Physics runs at sim.physics_hz, every core at core.tick_hz."""

from __future__ import annotations

import math
import random
from collections.abc import Callable
from dataclasses import dataclass
from typing import Any

from sim import corelib
from sim.sensors import BodySensors, FlashModel
from sim.world import Robot, World

Pose = tuple[float, float, float]


@dataclass
class Delivery:
    src: int
    data: bytes
    ir: list[float] | None = None


@dataclass
class Flash:
    sender: int
    time_s: float
    data: bytes
    poses: dict[int, Pose]  # every robot's pose when the flash was requested


@dataclass
class RxRecord:
    """One delivered FLASH sample with the truth behind it (only kept if rx_log is set)."""

    listener: int
    sender: int
    ir: list[float] | None
    true_bearing: float
    true_range: float


class Swarm:
    def __init__(self, cfg: dict[str, Any], seed: int) -> None:
        self.cfg = cfg
        self.seed = seed
        master = random.Random(seed)
        self.rng = random.Random(master.getrandbits(64))  # entropy fed to cores
        self.radio_rng = random.Random(master.getrandbits(64))
        self.sensor_rng = random.Random(master.getrandbits(64))
        self.world = World(cfg)
        self.params = corelib.params_from_config(cfg)
        self.flash_model = FlashModel(cfg, self.sensor_rng)
        self.drop_rate = cfg["radio"]["drop_rate"]
        self.collision_window = cfg["radio"]["flash_collision_window"]
        self.physics_hz = cfg["sim"]["physics_hz"]
        self.tick_hz = cfg["core"]["tick_hz"]
        self.cores: list[corelib.Core] = []
        self.body: list[BodySensors] = []
        self.inbox: dict[int, list[Delivery]] = {}
        self.debug: dict[int, tuple[int, int, int, int, int]] = {}
        self.path_mm: list[float] = []
        self.ticks = 0
        self.min_edge_mm = math.inf
        self.rx_log: list[RxRecord] | None = None
        self.on_tick: list[Callable[[Swarm], None]] = []
        self.stats = {"flashes": 0, "spoiled": 0, "dropped": 0, "delivered": 0, "overflow": 0}
        self._pending: list[Flash] = []
        self._previous_times: list[float] = []
        self._last_tick_step = 0
        self._last_pos: dict[int, tuple[float, float]] = {}

    # -- set-up ---------------------------------------------------------------------------

    def add_robot(self, x: float, y: float, heading: float) -> Robot:
        robot = self.world.add_robot(x, y, heading)
        self.cores.append(corelib.Core(robot.id, self.params))
        self.body.append(BodySensors(self.cfg, self.sensor_rng))
        self.path_mm.append(0.0)
        self._last_pos[robot.id] = (x, y)
        return robot

    def scatter(self, count: int) -> None:
        """Random, non-overlapping starts on the table."""
        spacing = self.cfg["sim"]["start_spacing"]
        inset = self.cfg["table"]["edge_margin"] + self.cfg["geometry"]["base"][
            "envelope_diameter"] / 2
        hw, hh = self.world.half_w - inset, self.world.half_h - inset
        placed: list[tuple[float, float]] = []
        for _ in range(100000):
            if len(placed) == count:
                break
            x, y = self.rng.uniform(-hw, hw), self.rng.uniform(-hh, hh)
            if all(math.hypot(x - px, y - py) >= spacing for px, py in placed):
                placed.append((x, y))
        if len(placed) < count:
            raise RuntimeError(f"could not place {count} robots")
        for x, y in placed:
            self.add_robot(x, y, self.rng.uniform(-math.pi, math.pi))

    # -- running --------------------------------------------------------------------------

    def alive(self) -> list[Robot]:
        return [r for r in self.world.robots if not r.fallen]

    def advance(self) -> None:
        """One physics step, preceded by a core tick when one is due."""
        if self.world.steps * self.tick_hz >= self.ticks * self.physics_hz:
            self.tick()
        self.world.step()

    def run(self, duration_s: float) -> None:
        end_step = round(duration_s * self.physics_hz)
        while self.world.steps < end_step:
            self.advance()

    def tick(self) -> None:
        world = self.world
        steps = world.steps - self._last_tick_step
        dt_us = round(steps * 1e6 / self.physics_hz) if self.ticks else round(1e6 / self.tick_hz)
        self._last_tick_step = world.steps
        now = world.time_s
        robots = self.alive()
        poses = {r.id: (r.position.x, r.position.y, r.heading) for r in robots}
        new: list[Flash] = []
        for r in robots:
            core = self.cores[r.id]
            body = self.body[r.id]
            inp = core.inputs
            inp.dt_us = dt_us
            inp.entropy = self.rng.getrandbits(32)
            inp.gyro_z_rad_s = body.gyro(r)
            inp.ring_angle_rad = body.ring_angle(r)
            inp.hall = world.hall(r)
            inp.ticks_left, inp.ticks_right = body.ticks(r)
            inp.table_left, inp.table_right = body.table(world, r)
            mail = self.inbox.pop(r.id, [])
            count = min(len(mail), corelib.RX_MAX)
            self.stats["overflow"] += len(mail) - count
            for slot, d in zip(inp.rx, mail[:count], strict=False):
                slot.src = d.src
                slot.len = len(d.data)
                slot.data[: len(d.data)] = d.data
                slot.has_ir = d.ir is not None
                if d.ir is not None:
                    slot.ir[:] = d.ir
            inp.rx_count = count
            out = core.step()
            world.set_wheels(r, out.wheel_left_mm_s, out.wheel_right_mm_s)
            dbg = out.debug
            self.debug[r.id] = (dbg.state, dbg.detail, dbg.index, dbg.chain_size, dbg.neighbours)
            if out.flash:
                data = bytes(out.flash_data[: out.flash_len])
                new.append(Flash(r.id, now + out.flash_delay_us * 1e-6, data, poses))
            for tx in out.tx[: out.tx_count]:
                self._send(r.id, tx.dst, bytes(tx.data[: tx.len]))
        self._resolve_flashes(new)
        for r in robots:
            x, y = r.position
            lx, ly = self._last_pos[r.id]
            self.path_mm[r.id] += math.hypot(x - lx, y - ly)
            self._last_pos[r.id] = (x, y)
            self.min_edge_mm = min(self.min_edge_mm, world.edge_distance(x, y))
        self.ticks += 1
        for script in self.on_tick:
            script(self)

    def _send(self, src: int, dst: int, data: bytes) -> None:
        targets = [r.id for r in self.alive() if r.id != src] if dst == corelib.BROADCAST else [dst]
        for t in targets:
            if self.radio_rng.random() < self.drop_rate:
                self.stats["dropped"] += 1
                continue
            self.inbox.setdefault(t, []).append(Delivery(src, data))

    def _resolve_flashes(self, new: list[Flash]) -> None:
        """Deliver last tick's flashes, now that every flash that could collide is known."""
        times = self._previous_times + [f.time_s for f in self._pending] + [f.time_s for f in new]
        for flash in self._pending:
            near = sum(1 for t in times if abs(t - flash.time_s) < self.collision_window)
            clean = near == 1  # only itself
            self.stats["flashes"] += 1
            self.stats["spoiled"] += not clean
            sender = flash.poses[flash.sender]
            for listener in sorted(flash.poses):
                if listener == flash.sender or self.world.robots[listener].fallen:
                    continue
                if self.radio_rng.random() < self.drop_rate:
                    self.stats["dropped"] += 1
                    continue
                sample = self.flash_model.sample(flash.poses[listener], sender[:2], clean)
                self.inbox.setdefault(listener, []).append(Delivery(flash.sender, flash.data,
                                                                    sample.ir))
                self.stats["delivered"] += 1
                if self.rx_log is not None:
                    self.rx_log.append(RxRecord(listener, flash.sender, sample.ir,
                                                sample.true_bearing, sample.true_range))
        self._previous_times = [f.time_s for f in self._pending]
        self._pending = new

    # -- results --------------------------------------------------------------------------

    def final_positions(self) -> list[tuple[float, float, float]]:
        return [(r.position.x, r.position.y, r.heading) for r in self.world.robots]

    def falls(self) -> int:
        return sum(1 for e in self.world.events if e.kind == "fall")
