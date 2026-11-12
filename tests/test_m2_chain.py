"""M2 acceptance: from random starts twelve robots form one chain, number themselves and
count themselves. All of it simulated."""

import os

import pytest

from sim.runner import run_seeds
from sim.scenarios import chain

# M2 asks for 50 seeds; `make test` runs the first 10, `make test-full` (CATENA_FULL=1) all.
SEEDS = list(range(50 if os.environ.get("CATENA_FULL") else 10))
REQUIRED_FRACTION = 0.9

@pytest.fixture(scope="module")
def results():
    return run_seeds("chain", SEEDS)


def summary(results) -> str:
    return "\n".join(f"seed {r['seed']}: {r['metrics']}" for r in results if not r["passed"])


def test_one_chain_of_twelve_forms_within_180_s_in_nine_of_ten_runs(results):
    formed = [r for r in results if r["metrics"]["one_chain"]]
    for r in formed:
        assert r["metrics"]["formed_s"] <= chain.FORM_LIMIT_S
        assert r["metrics"]["largest_chain"] == 12
    assert len(formed) >= REQUIRED_FRACTION * len(results), summary(results)


def test_index_runs_0_to_11_and_every_robot_counts_12_within_5_s_of_the_last_link(results):
    formed = [r for r in results if r["metrics"]["one_chain"]]
    assert formed, "no run formed a chain"
    for r in formed:
        m = r["metrics"]
        assert m["index_ok"], f"seed {r['seed']}: {m}"
        assert m["count_delay_s"] is not None, f"seed {r['seed']}: {m}"
        assert m["count_delay_s"] <= chain.COUNT_LIMIT_S, f"seed {r['seed']}: {m}"


def test_no_run_ends_with_two_chains_chasing_each_other(results):
    for r in results:
        assert r["metrics"]["chase_at_end_s"] <= chain.CHASE_LIMIT_S, f"seed {r['seed']}"


def test_no_robot_falls_off_the_table_while_chains_form(results):
    for r in results:
        assert r["metrics"]["falls"] == 0, f"seed {r['seed']}: {r['metrics']}"


def test_enough_runs_pass_every_check_at_once(results):
    passed = [r for r in results if r["passed"]]
    assert len(passed) >= REQUIRED_FRACTION * len(results), summary(results)
