"""What the phone page sends changes the shape in the running simulator
within 2 s (spec 10). The page's own JavaScript is not exercised here: the test speaks
its protocol over a real websocket."""

import json
import math
import time

import pytest
from websockets.sync.client import connect

from sim import corelib
from sim.bridge import Bridge
from sim.config import load_config
from sim.scenarios import play


@pytest.fixture()
def live():
    swarm = play.build(load_config(), seed=3)
    swarm.run(swarm.world.time_s + 3.0)  # a chain that has been linked for a few seconds
    bridge = Bridge(swarm, host="127.0.0.1", ws_port=0, http_port=None)
    bridge.start()
    with connect(f"ws://127.0.0.1:{bridge.ws_port}") as ws:
        yield swarm, bridge, ws


def send_and_wait(swarm, bridge, ws, message, done, limit_s: float) -> float:
    """Send one message; return the simulated seconds until `done()` holds."""
    before = bridge.applied
    ws.send(json.dumps(message))
    deadline = time.monotonic() + 5.0
    while bridge.applied == before:  # the socket thread hands it to the simulator's thread
        assert time.monotonic() < deadline, "the message never reached the simulator"
        swarm.advance()
        time.sleep(0.001)
    start = swarm.world.time_s
    while swarm.world.time_s - start < limit_s:
        if done():
            return swarm.world.time_s - start
        swarm.advance()
    pytest.fail(f"not done within {limit_s} s: {message['type']}")


def drawn_u() -> list[list[float]]:
    """A hand-drawn U as the page would send it: uneven points in pad pixels turned y up."""
    path = [[40.0, 200.0 - 3.1 * k] for k in range(40)]
    path += [[120.0 - 80.0 * math.cos(a / 30 * math.pi), 76.0 - 60.0 * math.sin(a / 30 * math.pi)]
             for a in range(1, 30)]
    path += [[200.0, 76.0 + 4.2 * k] for k in range(30)]
    return path


def test_a_drawn_stroke_reaches_every_robot_within_2_s(live):
    swarm, bridge, ws = live
    assert all(d.state == corelib.STATE_FOLLOW or d.index == 0 for d in swarm.debug.values())

    def all_hold_it() -> bool:
        return all(d.shape_version == 1 and d.state == corelib.STATE_SHAPE
                   for d in swarm.debug.values())

    took = send_and_wait(swarm, bridge, ws, {"type": "shape", "points": drawn_u()},
                         all_hold_it, 2.0)
    assert took <= 2.0
    assert swarm.stroke is not None and not swarm.stroke.closed


def test_a_second_stroke_replaces_the_first_within_2_s(live):
    swarm, bridge, ws = live
    ring = [[math.cos(a / 40 * 2 * math.pi), math.sin(a / 40 * 2 * math.pi)] for a in range(40)]
    send_and_wait(swarm, bridge, ws, {"type": "shape", "points": drawn_u()},
                  lambda: all(d.shape_version == 1 for d in swarm.debug.values()), 2.0)
    took = send_and_wait(swarm, bridge, ws, {"type": "shape", "points": ring},
                         lambda: all(d.shape_version == 2 for d in swarm.debug.values()), 2.0)
    assert took <= 2.0
    assert swarm.stroke.closed  # ends within 10% of the length: a closed shape


def test_the_joystick_sets_the_chain_moving_and_the_page_gets_a_status_line(live):
    swarm, bridge, ws = live
    send_and_wait(swarm, bridge, ws, {"type": "shape", "points": [[0, 0], [1, 0]]},
                  lambda: all(d.state == corelib.STATE_SHAPE for d in swarm.debug.values()), 2.0)
    took = send_and_wait(swarm, bridge, ws, {"type": "twist", "vx": 0.5, "vy": 0.0, "wz": 0.0},
                         lambda: all(d.state == corelib.STATE_MOVE for d in swarm.debug.values()),
                         2.0)
    assert took <= 2.0
    status = json.loads(ws.recv(timeout=5))
    for _ in range(50):  # skip status lines sent before the twist arrived
        if status.get("moving"):
            break
        swarm.advance()
        status = json.loads(ws.recv(timeout=5))
    assert status["type"] == "status" and status["robots"] == 12 and status["count"] == 12
    assert status["moving"] and status["version"] == 1


def test_a_malformed_message_changes_nothing(live):
    swarm, bridge, ws = live
    ws.send("not json")
    ws.send(json.dumps({"type": "shape", "points": "nonsense"}))
    ws.send(json.dumps({"type": "shape", "points": [[1, 1], [1, 1]]}))  # no length
    for _ in range(240):
        swarm.advance()
    assert all(d.shape_version == 0 for d in swarm.debug.values())
