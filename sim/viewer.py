"""pygame viewer: table, robots, tongues, links, each robot's state.

Keys: space pause, + and - change speed, s saves a screenshot, Esc or closing the window quits.
"""

from __future__ import annotations

import math
from pathlib import Path

import pygame

from sim import corelib
from sim.swarm import Swarm

MARGIN_MM = 60.0
MAX_WINDOW_PX = (1280, 860)
FPS = 60
DETAIL_COLOURS = {  # wander sub-state
    0: (46, 139, 87),  # cruise
    1: (218, 165, 32),  # avoid
    2: (200, 60, 45),  # backoff
    3: (65, 105, 225),  # turn
}
TABLE = (236, 230, 214)
FLOOR = (52, 58, 56)
BASE = (88, 98, 95)
RING = (20, 135, 111)
HUB = (170, 180, 181)
LINK = (230, 120, 40)
TEXT = (24, 33, 31)


class Viewer:
    def __init__(self, swarm: Swarm, title: str = "Catena") -> None:
        self.swarm = swarm
        world = swarm.world
        self.width_mm = 2 * (world.half_w + MARGIN_MM)
        self.height_mm = 2 * (world.half_h + MARGIN_MM)
        self.scale = min(MAX_WINDOW_PX[0] / self.width_mm, MAX_WINDOW_PX[1] / self.height_mm)
        pygame.init()
        size = (round(self.width_mm * self.scale), round(self.height_mm * self.scale) + 28)
        self.screen = pygame.display.set_mode(size)
        pygame.display.set_caption(title)
        self.font = pygame.font.Font(None, 18)
        self.hud_font = pygame.font.Font(None, 22)
        geo = swarm.cfg["geometry"]
        self.base_r = geo["base"]["envelope_diameter"] / 2
        self.ring_r = geo["ring"]["plate_diameter"] / 2
        self.hub_r = geo["ring"]["hub_diameter"] / 2
        self.tongue_w = geo["tongue"]["width"]

    def px(self, x: float, y: float) -> tuple[int, int]:
        w = self.swarm.world
        return (round((x + w.half_w + MARGIN_MM) * self.scale),
                round((w.half_h + MARGIN_MM - y) * self.scale))

    def run(self, duration_s: float, speed: float = 1.0, exit_after: float | None = None,
            screenshot: str | None = None) -> None:
        clock = pygame.time.Clock()
        paused = False
        running = True
        swarm = self.swarm
        end_steps = round(duration_s * swarm.physics_hz)
        stop_steps = round(exit_after * swarm.physics_hz) if exit_after is not None else None
        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False
                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        running = False
                    elif event.key == pygame.K_SPACE:
                        paused = not paused
                    elif event.key in (pygame.K_PLUS, pygame.K_EQUALS, pygame.K_KP_PLUS):
                        speed = min(32.0, speed * 2)
                    elif event.key in (pygame.K_MINUS, pygame.K_KP_MINUS):
                        speed = max(0.125, speed / 2)
                    elif event.key == pygame.K_s:
                        self.save(f"catena-{swarm.world.steps}.png")
            if not paused:
                target = min(end_steps, swarm.world.steps + round(speed * swarm.physics_hz / FPS))
                if stop_steps is not None:
                    target = min(target, stop_steps)
                while swarm.world.steps < target:
                    swarm.advance()
            self.draw(speed, paused)
            pygame.display.flip()
            if stop_steps is not None and swarm.world.steps >= stop_steps:
                running = False
            clock.tick(FPS)
        if screenshot:
            self.save(screenshot)
        pygame.quit()

    def save(self, path: str) -> None:
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        pygame.image.save(self.screen, path)

    def draw(self, speed: float, paused: bool) -> None:
        swarm, world, s = self.swarm, self.swarm.world, self.scale
        self.screen.fill(FLOOR)
        x0, y0 = self.px(-world.half_w, world.half_h)
        table = pygame.Rect(x0, y0, round(2 * world.half_w * s), round(2 * world.half_h * s))
        pygame.draw.rect(self.screen, TABLE, table)
        for link in world.links:
            pygame.draw.line(self.screen, LINK, self.px(*link.tongue.position),
                             self.px(*link.hub.position), max(2, round(6 * s)))
        for r in world.robots:
            if not r.fallen:
                self.draw_robot(r)
        alive = len(swarm.alive())
        falls = len(world.robots) - alive
        hud = (f"t {world.time_s:6.1f} s   speed {speed:g}x{'  PAUSED' if paused else ''}   "
               f"robots {alive}   falls {falls}   links {len(world.links)}   "
               f"flashes spoiled {swarm.stats['spoiled']}/{swarm.stats['flashes']}")
        bar_y = self.screen.get_height() - 24
        self.screen.blit(self.hud_font.render(hud, True, (235, 240, 238)), (8, bar_y))

    def draw_robot(self, r) -> None:
        s = self.scale
        cx, cy = self.px(*r.position)
        state, detail, index, _, neighbours = self.swarm.debug.get(r.id, (0, 0, 0, 0, 0))
        colour = DETAIL_COLOURS.get(detail, BASE) if state == 1 else BASE
        pygame.draw.circle(self.screen, BASE, (cx, cy), round(self.base_r * s))
        rim = max(1, round(2 * s))
        pygame.draw.circle(self.screen, RING, (cx, cy), round(self.ring_r * s), rim)
        tongue = max(2, round(self.tongue_w * s * 0.6))
        pygame.draw.line(self.screen, RING, (cx, cy), self.px(*r.tongue_tip), tongue)
        pygame.draw.circle(self.screen, HUB, (cx, cy), round(self.hub_r * s))
        hx = r.position.x + math.cos(r.heading) * self.base_r * 0.8
        hy = r.position.y + math.sin(r.heading) * self.base_r * 0.8
        pygame.draw.line(self.screen, colour, (cx, cy), self.px(hx, hy), max(2, round(4 * s)))
        pygame.draw.circle(self.screen, colour, (cx, cy), max(3, round(7 * s)))
        name = corelib.WANDER_DETAIL.get(detail, "") if state == 1 else ""
        label = self.font.render(f"{r.id} {name} n{neighbours}", True, TEXT)
        self.screen.blit(label, (cx - label.get_width() // 2, cy + round(self.ring_r * s) + 2))
