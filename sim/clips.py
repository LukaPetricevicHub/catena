"""Render the README clips from scenarios (`make clips`): docs/media/lasso.gif, heal.gif,
sweep.gif.

Each clip is the simulator's own viewer, drawn off screen while the scenario runs."""

from __future__ import annotations

import os

os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

import sys  # noqa: E402
from collections.abc import Callable  # noqa: E402
from pathlib import Path  # noqa: E402

import imageio.v2 as imageio  # noqa: E402
import numpy as np  # noqa: E402
import pygame  # noqa: E402

from sim import corelib  # noqa: E402
from sim.config import REPO_ROOT  # noqa: E402
from sim.runner import build  # noqa: E402
from sim.swarm import Swarm  # noqa: E402
from sim.viewer import Viewer  # noqa: E402

MEDIA = REPO_ROOT / "docs" / "media"
FRAME_EVERY_S = 1.0   # simulated time between frames
PLAYBACK_FPS = 10     # so a clip plays at ten times real time
SHRINK = 2            # every second pixel


def render(scenario: str, seed: int, path: Path, starts: Callable[[Swarm], bool],
           lead_s: float, limit_s: float) -> dict:
    """Run a scenario to its end; keep frames from `lead_s` before `starts` first holds.
    The run is done twice: once to learn when that is, once to draw."""
    module, swarm = build(scenario, seed)
    while not swarm.stop and swarm.world.time_s < module.DURATION_S and not starts(swarm):
        swarm.advance()
    first = max(0.0, swarm.world.time_s - lead_s)

    module, swarm = build(scenario, seed)
    viewer = Viewer(swarm, title=scenario)
    frames, next_frame = [], first
    while not swarm.stop and swarm.world.time_s < module.DURATION_S:
        swarm.advance()
        now = swarm.world.time_s
        if now >= next_frame and now - first <= limit_s:
            next_frame += FRAME_EVERY_S
            viewer.draw(PLAYBACK_FPS * FRAME_EVERY_S, False)
            shot = pygame.surfarray.array3d(viewer.screen).transpose(1, 0, 2)
            frames.append(np.ascontiguousarray(shot[::SHRINK, ::SHRINK]))
    pygame.quit()
    path.parent.mkdir(parents=True, exist_ok=True)
    imageio.mimsave(path, frames, duration=1.0 / PLAYBACK_FPS, loop=0)
    passed, metrics = module.evaluate(swarm)
    print(f"{path.relative_to(REPO_ROOT)}: {len(frames)} frames, {path.stat().st_size // 1024} kB, "
          f"scenario {scenario} seed {seed} {'passed' if passed else 'FAILED'}")
    return {"passed": passed, "metrics": metrics}


def main(argv: list[str] | None = None) -> int:
    """`python -m sim.clips` renders every clip; names (lasso, heal, sweep) pick some."""
    clips = {
        "lasso": lambda: render("lasso", 0, MEDIA / "lasso.gif", lead_s=6.0, limit_s=150.0,
                                starts=lambda s: s.debug.get(0) is not None
                                and s.debug[0].state == corelib.STATE_LASSO),
        "heal": lambda: render("heal:ball", 0, MEDIA / "heal.gif", lead_s=5.0, limit_s=45.0,
                               starts=lambda s: s.monitor.removed),
        "sweep": lambda: render("sweep", 0, MEDIA / "sweep.gif", lead_s=4.0, limit_s=40.0,
                                starts=lambda s: s.monitor.sweep_s is not None),
    }
    names = (sys.argv[1:] if argv is None else argv) or list(clips)
    return 0 if all([clips[name]()["passed"] for name in names]) else 1


if __name__ == "__main__":
    sys.exit(main())
