"""The simulator's end of the phone page (spec 10): a websocket that takes strokes and
twists as JSON and hands them to one robot, plus a small web server for `ui/`.

Messages from the page:
    {"type": "shape", "points": [[x, y], ...], "closed": true}   any units, y up
    {"type": "twist", "vx": 0.5, "vy": 0, "wz": -1}             joystick and slider, -1..1
    {"type": "release"}                                          back to FOLLOW
Message to the page, a few times a second:
    {"type": "status", "robots": 12, "linked": 12, "count": 12, "shape_error_mm": 3.1,
     "version": 4, "moving": false}
"""

from __future__ import annotations

import asyncio
import functools
import http.server
import json
import math
import queue
import threading
from typing import Any

import numpy as np
from websockets.asyncio.server import ServerConnection, serve

from sim import corelib, shapes
from sim.config import REPO_ROOT
from sim.swarm import Swarm

STATUS_PERIOD_S = 0.2


class _QuietHandler(http.server.SimpleHTTPRequestHandler):
    def log_message(self, format: str, *args: Any) -> None:  # noqa: A002
        pass


class Bridge:
    def __init__(self, swarm: Swarm, host: str = "0.0.0.0", ws_port: int = 8765,
                 http_port: int | None = 8000) -> None:
        self.swarm = swarm
        self.host = host
        self.ws_port = ws_port
        self.http_port = http_port
        self.commands: queue.Queue[dict[str, Any]] = queue.Queue()
        self.applied = 0  # commands handed to a robot so far
        self._clients: set[ServerConnection] = set()
        self._loop: asyncio.AbstractEventLoop | None = None
        self._ready = threading.Event()
        self._next_status = 0.0
        self._httpd: http.server.ThreadingHTTPServer | None = None
        shape_cfg = swarm.cfg["behaviour"]["shape"]
        self._max_speed = shape_cfg["twist_max_speed"]
        self._max_rate = math.radians(shape_cfg["twist_max_rate"])

    # -- servers (their own threads) ------------------------------------------------------

    def start(self) -> None:
        threading.Thread(target=lambda: asyncio.run(self._serve()), daemon=True).start()
        if not self._ready.wait(5.0):
            raise RuntimeError("websocket server did not start")
        if self.http_port is not None:
            handler = functools.partial(_QuietHandler, directory=str(REPO_ROOT / "ui"))
            self._httpd = http.server.ThreadingHTTPServer((self.host, self.http_port), handler)
            threading.Thread(target=self._httpd.serve_forever, daemon=True).start()
        self.swarm.on_tick.append(self.on_tick)

    async def _serve(self) -> None:
        self._loop = asyncio.get_running_loop()
        async with serve(self._client, self.host, self.ws_port) as server:
            self.ws_port = server.sockets[0].getsockname()[1]
            self._ready.set()
            await asyncio.get_running_loop().create_future()  # run until the process ends

    async def _client(self, ws: ServerConnection) -> None:
        self._clients.add(ws)
        try:
            async for raw in ws:
                try:
                    message = json.loads(raw)
                except (TypeError, ValueError):
                    continue
                if isinstance(message, dict):
                    self.commands.put(message)
        finally:
            self._clients.discard(ws)

    async def _broadcast(self, text: str) -> None:
        for ws in list(self._clients):
            try:
                await ws.send(text)
            except Exception:  # noqa: BLE001  a client that went away is not our problem
                self._clients.discard(ws)

    # -- simulator side (the thread that steps the swarm) ---------------------------------

    def joined_robot(self) -> int | None:
        """The robot the phone has joined: the lowest id still on the table."""
        alive = self.swarm.alive()
        return alive[0].id if alive else None

    def apply(self, message: dict[str, Any]) -> None:
        robot = self.joined_robot()
        if robot is None:
            return
        kind = message.get("type")
        try:
            if kind == "shape":
                path = [(float(x), float(y)) for x, y in message["points"]]
                closed = message.get("closed")
                self.swarm.set_shape(shapes.stroke_from_path(path, closed), robot)
            elif kind == "release":
                self.swarm.set_shape(shapes.named("line"), robot, active=False)
            elif kind == "twist":
                vx, vy, wz = (max(-1.0, min(1.0, float(message.get(k, 0.0))))
                              for k in ("vx", "vy", "wz"))
                self.swarm.set_twist(vx * self._max_speed, vy * self._max_speed,
                                     wz * self._max_rate, robot)
            else:
                return
        except (KeyError, TypeError, ValueError):
            return  # a malformed message changes nothing
        self.applied += 1

    def status(self) -> dict[str, Any]:
        swarm = self.swarm
        alive = swarm.alive()
        debug = [swarm.debug[r.id] for r in alive if r.id in swarm.debug]
        chain = sorted((r for r in alive), key=lambda r: swarm.debug[r.id].index
                       if r.id in swarm.debug else 0)
        linked = max((d.chain_size for d in debug), default=0)
        error = None
        if swarm.stroke is not None and len(chain) >= 2 and linked == len(chain):
            polygon = shapes.target_polygon(swarm.stroke, len(chain),
                                            swarm.cfg["geometry"]["pitch"])
            here = np.array([[r.position.x, r.position.y] for r in chain])
            error = round(shapes.fit(here, polygon).mean_error_mm, 1)
        return {
            "type": "status",
            "robots": len(alive),
            "linked": linked,
            "count": max((d.chain_size for d in debug), default=0),
            "shape_error_mm": error,
            "version": max((d.shape_version for d in debug), default=0),
            "moving": any(d.state == corelib.STATE_MOVE for d in debug),
        }

    def on_tick(self, swarm: Swarm) -> None:
        while True:
            try:
                self.apply(self.commands.get_nowait())
            except queue.Empty:
                break
        now = swarm.world.time_s
        if now >= self._next_status and self._loop is not None and self._clients:
            self._next_status = now + STATUS_PERIOD_S
            asyncio.run_coroutine_threadsafe(self._broadcast(json.dumps(self.status())),
                                             self._loop)
