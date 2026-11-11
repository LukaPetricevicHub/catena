"""Top-down 2D physics of the robots on a table (pymunk). This module knows the truth;
core never sees it. Units: mm, g, s, so a force of 1 N is 1e6 and 1 N m is 1e9.

A robot is two bodies: the base (wheels, sensors) and the ring (plates, hub, tongue) on a
pivot at the base centre. A link is three constraints between the tongue's ring and the
hub's ring: a groove that keeps the hub on the tongue's axis and carries compression, a
magnet that pulls the hub to one pitch with at most the hold force, and the yaw limit.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

import pymunk

NEWTON = 1e6  # g mm / s^2
NEWTON_METRE = 1e9  # g mm^2 / s^2

CAT_BASE, CAT_PLATE, CAT_HUB, CAT_TONGUE, CAT_CAPTURE, CAT_OBJECT = 1, 2, 4, 8, 16, 32
TYPE_HUB, TYPE_CAPTURE = 1, 2


def wrap_pi(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass
class Link:
    tongue: Robot  # the robot whose tongue sits on the hub (its `ahead` is `hub`)
    hub: Robot
    constraints: tuple[pymunk.Constraint, ...]
    created_s: float


@dataclass
class Event:
    time_s: float
    kind: str  # "link", "unlink", "fall"
    a: int
    b: int = -1


class Robot:
    def __init__(self, world: World, rid: int, x: float, y: float, heading: float) -> None:
        g = world.geo
        self.id = rid
        self.world = world
        self.fallen = False
        self.grounded = True
        self.cmd = (0.0, 0.0)  # wheel speeds, mm/s
        self.wheel_travel = [0.0, 0.0]  # mm turned by each wheel
        self.ahead: Link | None = None  # my tongue on another robot's hub
        self.behind: Link | None = None  # another robot's tongue on my hub

        mass = world.cfg["mass"]
        base_r = g["base"]["envelope_diameter"] / 2
        ring_r = g["ring"]["plate_diameter"] / 2
        self.base = pymunk.Body(mass["base_assembly"], pymunk.moment_for_circle(
            mass["base_assembly"], 0, base_r))
        self.ring = pymunk.Body(mass["ring_assembly"], pymunk.moment_for_circle(
            mass["ring_assembly"], 0, ring_r))
        for body in (self.base, self.ring):
            body.position = (x, y)
            body.angle = heading

        friction = world.cfg["sim"]["contact_friction"]
        base_shape = pymunk.Circle(self.base, base_r)
        base_shape.filter = pymunk.ShapeFilter(categories=CAT_BASE, mask=CAT_BASE | CAT_OBJECT)
        plate = pymunk.Circle(self.ring, ring_r)
        plate.filter = pymunk.ShapeFilter(categories=CAT_PLATE, mask=CAT_PLATE)
        for shape in (base_shape, plate):
            shape.friction = friction
        hub = pymunk.Circle(self.ring, g["ring"]["hub_diameter"] / 2)
        hub.filter = pymunk.ShapeFilter(categories=CAT_HUB, mask=CAT_TONGUE | CAT_CAPTURE)
        hub.collision_type = TYPE_HUB
        tip_r = g["tongue"]["tip_radius"]
        tongue = pymunk.Segment(self.ring, (ring_r, 0), (g["tongue"]["reach"] - tip_r, 0), tip_r)
        tongue.filter = pymunk.ShapeFilter(categories=CAT_TONGUE, mask=CAT_HUB)
        capture = pymunk.Circle(self.ring, world.cfg["coupling"]["capture_range"],
                                (g["tongue"]["reach"], 0))
        capture.sensor = True
        capture.filter = pymunk.ShapeFilter(categories=CAT_CAPTURE, mask=CAT_HUB)
        capture.collision_type = TYPE_CAPTURE
        self.shapes = (base_shape, plate, hub, tongue, capture)

        bearing = pymunk.PivotJoint(self.base, self.ring, (0, 0), (0, 0))
        bearing.collide_bodies = False

        # Each wheel: a velocity constraint to a ground patch moving at the commanded wheel
        # speed, limited by half the traction in any direction (friction circle).
        self.patches: list[pymunk.Body] = []
        self.wheel_joints: list[pymunk.PivotJoint] = []
        half_track = g["base"]["track"] / 2
        for side in (+1.0, -1.0):  # left, right
            patch = pymunk.Body(body_type=pymunk.Body.KINEMATIC)
            patch.position = (x, y)
            joint = pymunk.PivotJoint(patch, self.base, (0, 0), (0, side * half_track))
            joint.max_bias = 0.0
            joint.max_force = world.traction / 2
            self.patches.append(patch)
            self.wheel_joints.append(joint)
        self._space_items: list[Any] = [self.base, self.ring, *self.shapes, bearing,
                                        *self.patches, *self.wheel_joints]
        world.space.add(*self._space_items)

    @property
    def position(self) -> pymunk.Vec2d:
        return self.base.position

    @property
    def heading(self) -> float:
        return self.base.angle

    @property
    def ring_angle(self) -> float:
        """Tongue direction relative to the base, counter-clockwise."""
        return wrap_pi(self.ring.angle - self.base.angle)

    @property
    def tongue_tip(self) -> pymunk.Vec2d:
        return self.ring.local_to_world((self.world.geo["tongue"]["reach"], 0))


class World:
    def __init__(self, cfg: dict[str, Any]) -> None:
        self.cfg = cfg
        self.geo = cfg["geometry"]
        sim = cfg["sim"]
        self.dt = 1.0 / sim["physics_hz"]
        self.time_s = 0.0
        self.steps = 0
        self.space = pymunk.Space()
        self.space.iterations = sim["solver_iterations"]
        self.traction = cfg["drive"]["traction_force_n"] * NEWTON
        self.max_speed = cfg["drive"]["max_speed"]
        self.pitch = self.geo["pitch"]
        self.capture_range = cfg["coupling"]["capture_range"]
        self.hold_force = cfg["coupling"]["hold_force_n"] * NEWTON
        self.max_yaw = math.radians(self.geo["ring"]["max_joint_yaw"])
        self.acceptance = math.radians(self.geo["ring"]["acceptance_half_angle"])
        self.half_w, self.half_h = (s / 2 for s in cfg["table"]["size"])
        self.robots: list[Robot] = []
        self.links: list[Link] = []
        self.events: list[Event] = []
        self._robot_of: dict[pymunk.Body, Robot] = {}
        self._near: dict[tuple[int, int], int] = {}  # (tongue id, hub id) -> overlap count

        ring = cfg["mass"]["ring_assembly"]
        ring_inertia = pymunk.moment_for_circle(ring, 0, self.geo["ring"]["plate_diameter"] / 2)
        self.detent = cfg["coupling"]["centering_detent_mnm"] * 1e-3 * NEWTON_METRE
        self.ring_damping = 2.0 * sim["ring_damping_ratio"] * math.sqrt(self.detent * ring_inertia)
        self.space.on_collision(TYPE_CAPTURE, TYPE_HUB, begin=self._near_begin,
                                separate=self._near_end)

    # -- set-up and commands --------------------------------------------------------------

    def add_robot(self, x: float, y: float, heading: float) -> Robot:
        robot = Robot(self, len(self.robots), x, y, heading)
        self.robots.append(robot)
        self._robot_of[robot.ring] = robot
        return robot

    def set_wheels(self, robot: Robot, left: float, right: float) -> None:
        limit = self.max_speed
        robot.cmd = (max(-limit, min(limit, left)), max(-limit, min(limit, right)))

    def set_grounded(self, robot: Robot, grounded: bool) -> None:
        """Lift a robot's wheels off the table (as if held by a hand), or put them back."""
        robot.grounded = grounded
        for joint in robot.wheel_joints:
            joint.max_force = self.traction / 2 if grounded else 0.0

    def on_table(self, x: float, y: float) -> bool:
        return abs(x) <= self.half_w and abs(y) <= self.half_h

    def edge_distance(self, x: float, y: float) -> float:
        return min(self.half_w - abs(x), self.half_h - abs(y))

    # -- stepping -------------------------------------------------------------------------

    def step(self) -> None:
        dt = self.dt
        for r in self.robots:
            if r.fallen:
                continue
            c, s = math.cos(r.base.angle), math.sin(r.base.angle)
            for k, patch in enumerate(r.patches):
                patch.velocity = (r.cmd[k] * c, r.cmd[k] * s)
                r.wheel_travel[k] += r.cmd[k] * dt
            twist = wrap_pi(r.ring.angle - r.base.angle)
            spin = r.ring.angular_velocity - r.base.angular_velocity
            torque = -self.detent * math.sin(twist) - self.ring_damping * spin
            r.ring.torque = torque
            r.base.torque = -torque
        self.space.step(dt)
        self.steps += 1
        self.time_s = self.steps * dt
        self._update_links()
        for r in self.robots:
            if not r.fallen and not self.on_table(*r.position):
                self._fall(r)

    def _near_begin(self, arbiter: pymunk.Arbiter, space: pymunk.Space, data: Any) -> None:
        key = self._pair(arbiter)
        if key:
            self._near[key] = self._near.get(key, 0) + 1

    def _near_end(self, arbiter: pymunk.Arbiter, space: pymunk.Space, data: Any) -> None:
        key = self._pair(arbiter)
        if key and key in self._near:
            self._near[key] -= 1
            if self._near[key] <= 0:
                del self._near[key]

    def _pair(self, arbiter: pymunk.Arbiter) -> tuple[int, int] | None:
        capture, hub = arbiter.shapes
        a, b = self._robot_of.get(capture.body), self._robot_of.get(hub.body)
        return (a.id, b.id) if a and b else None

    def link_gap(self, link: Link) -> float:
        """How far the tip is from sitting on the hub, mm."""
        return (link.hub.ring.position - link.tongue.ring.position).length - self.pitch

    def hall(self, robot: Robot) -> float:
        """Analogue hall reading under the hub: 1 with a tip on it, falling to 0 at the edge
        of capture range."""
        if not robot.behind:
            return 0.0
        return max(0.0, min(1.0, 1.0 - self.link_gap(robot.behind) / self.capture_range))

    def _update_links(self) -> None:
        for link in list(self.links):
            if self.link_gap(link) > self.capture_range:
                self.unlink(link)
        for tongue_id, hub_id in sorted(self._near):
            a, b = self.robots[tongue_id], self.robots[hub_id]
            if a.ahead or b.behind or a.fallen or b.fallen:
                continue
            if b.ahead and b.ahead.hub is a:  # two robots cannot hold each other's hubs
                continue
            gap = (a.tongue_tip - b.ring.position).length - self.geo["ring"]["hub_diameter"] / 2
            if gap > self.capture_range:
                continue
            tip_dir = (a.tongue_tip - b.ring.position).angle
            if abs(wrap_pi(tip_dir - (b.ring.angle + math.pi))) > self.acceptance:
                continue
            self.link(a, b)

    def link(self, a: Robot, b: Robot) -> Link:
        """Put a's tongue on b's hub."""
        p = self.pitch
        seat = self.cfg["sim"]["link_seat_speed"]
        groove = pymunk.GrooveJoint(a.ring, b.ring, (p, 0), (11 * p, 0), (0, 0))
        magnet = pymunk.SlideJoint(a.ring, b.ring, (0, 0), (0, 0), 0, p)
        magnet.max_force = self.hold_force
        rel = b.ring.angle - a.ring.angle
        centre = rel - wrap_pi(rel)
        yaw = pymunk.RotaryLimitJoint(a.ring, b.ring, centre - self.max_yaw, centre + self.max_yaw)
        for c in (groove, magnet, yaw):
            c.collide_bodies = False
            c.max_bias = seat
        self.space.add(groove, magnet, yaw)
        link = Link(a, b, (groove, magnet, yaw), self.time_s)
        a.ahead = link
        b.behind = link
        self.links.append(link)
        self.events.append(Event(self.time_s, "link", a.id, b.id))
        return link

    def unlink(self, link: Link) -> None:
        self.space.remove(*link.constraints)
        self.links.remove(link)
        link.tongue.ahead = None
        link.hub.behind = None
        self.events.append(Event(self.time_s, "unlink", link.tongue.id, link.hub.id))

    def _fall(self, robot: Robot) -> None:
        for link in (robot.ahead, robot.behind):
            if link:
                self.unlink(link)
        robot.fallen = True
        self.space.remove(*robot._space_items)
        self._near = {k: v for k, v in self._near.items() if robot.id not in k}
        self.events.append(Event(self.time_s, "fall", robot.id))
