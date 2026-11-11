"""M1 acceptance: `make sim S=wander` opens the viewer. Runs headless (SDL dummy driver),
closes itself after two simulated seconds and must leave a screenshot of a drawn table."""

import os
import shutil
import subprocess

import pygame
import pytest

from sim.config import REPO_ROOT


@pytest.mark.skipif(shutil.which("make") is None, reason="needs make")
def test_make_sim_wander_opens_the_viewer(tmp_path):
    shot = tmp_path / "wander.png"
    env = dict(os.environ, SDL_VIDEODRIVER="dummy", SDL_AUDIODRIVER="dummy")
    args = f"--exit-after 2 --speed 4 --screenshot {shot}"
    done = subprocess.run(["make", "--no-print-directory", "sim", "S=wander", f"ARGS={args}"],
                          cwd=REPO_ROOT, env=env, capture_output=True, text=True, timeout=300)
    assert done.returncode == 0, done.stdout + done.stderr
    assert '"sim_time_s": 2.0' in done.stdout
    image = pygame.image.load(str(shot))
    w, h = image.get_size()
    assert w > 600 and h > 400
    colours = {image.get_at((x, y))[:3] for x in range(0, w, 7) for y in range(0, h, 7)}
    assert len(colours) > 5  # table, floor and robots, not a blank window
