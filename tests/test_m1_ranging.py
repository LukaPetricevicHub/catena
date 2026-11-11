"""M1 acceptance: flash ranging and the radio model reproduce the configured noise."""

import math
import statistics

import pytest

from sim import corelib
from sim.config import load_config
from sim.scenarios import wander
from sim.world import wrap_pi

SAMPLES = 1000


@pytest.fixture(scope="module")
def swarm():
    s = wander.build(load_config(), seed=11)
    s.rx_log = []
    s.run(20.0)
    return s


def test_flash_ranging_errors_match_the_configured_noise_within_20_percent(swarm):
    cfg = load_config()
    params = corelib.params_from_config(cfg)
    clean = [r for r in swarm.rx_log if r.ir is not None]
    assert len(clean) >= SAMPLES
    bearing_err, range_err = [], []
    for rec in clean[:SAMPLES]:
        bearing, rng = corelib.flash_estimate(params, rec.ir)  # core's estimator
        bearing_err.append(wrap_pi(bearing - rec.true_bearing))
        range_err.append(rng / rec.true_range - 1.0)
    bearing_sigma = math.radians(cfg["sensing"]["ir_bearing_sigma_deg"])
    range_sigma = cfg["sensing"]["ir_range_sigma_frac"]
    assert statistics.pstdev(bearing_err) == pytest.approx(bearing_sigma, rel=0.2)
    assert statistics.pstdev(range_err) == pytest.approx(range_sigma, rel=0.2)
    assert abs(statistics.fmean(bearing_err)) < 0.2 * bearing_sigma
    assert abs(statistics.fmean(range_err)) < 0.2 * range_sigma


def test_no_ir_sample_beyond_ir_range(swarm):
    max_range = load_config()["sensing"]["ir_range"]
    for rec in swarm.rx_log:
        if rec.true_range > max_range:
            assert rec.ir is None


def test_radio_drops_the_configured_fraction_of_frames(swarm):
    drop = load_config()["radio"]["drop_rate"]
    stats = swarm.stats
    measured = stats["dropped"] / (stats["dropped"] + stats["delivered"])
    assert measured == pytest.approx(drop, rel=0.2)


def test_flashes_closer_than_the_collision_window_spoil_the_sample(swarm):
    stats = swarm.stats
    assert 0 < stats["spoiled"] < stats["flashes"]
    max_range = load_config()["sensing"]["ir_range"]
    in_range_but_spoiled = [r for r in swarm.rx_log if r.ir is None and r.true_range <= max_range]
    assert in_range_but_spoiled
