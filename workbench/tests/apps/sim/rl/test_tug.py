"""Unit tests for the scripted tug retention test.

The tug decides whether an episode counts as a grasp, so what is pinned here is
its *verdict*: which envs it takes over, which direction it pulls, and that an
object which leaves the arm fails even if it comes back.
"""

from __future__ import annotations

import math

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import pytest
import torch

from elesim_sim.rl.configs.loader import load_config
from elesim_sim.rl.envs.lift_test import LiftObservation, TugPhase, TugTest


@pytest.fixture()
def cfg():
    return load_config()


@pytest.fixture()
def device():
    return torch.device("cpu")


def _obs(n: int, *, object_pos: torch.Tensor) -> LiftObservation:
    return LiftObservation(
        object_pos=object_pos,
        object_quat=torch.tensor([[1.0, 0.0, 0.0, 0.0]] * n),
        anchor_pos=torch.zeros(n, 3),
        anchor_quat=torch.tensor([[1.0, 0.0, 0.0, 0.0]] * n),
        # The tug does not read these; the lift's retention check does.
        clearance=torch.full((n,), 1.0),
        object_contacts=torch.full((n,), 4),
    )


def test_tug_only_arms_envs_past_the_trigger(cfg, device):
    tug = TugTest(cfg.success, n_envs=3, device=device)
    phi = torch.tensor([0.0, cfg.success.tug.trigger_rad - 0.01,
                        cfg.success.tug.trigger_rad + 0.01])
    newly = tug.arm(
        phi,
        _obs(3, object_pos=torch.zeros(3, 3)),
        gap_bearing_rad=torch.zeros(3),
        weight_n=torch.full((3,), 19.6),
    )
    assert newly.tolist() == [False, False, True]
    assert tug.follows_policy.tolist() == [True, True, False]


def test_tug_pulls_along_the_gap_bearing(cfg, device):
    """The pull leaves by the opening, not into the arm.

    Pulling in an arbitrary direction would mostly press the object into the
    links, which any pose survives; the wrap is only tested by a force pointed
    where it left a way out.
    """
    tug = TugTest(cfg.success, n_envs=2, device=device)
    bearing = torch.tensor([0.0, math.pi / 2])
    tug.arm(
        torch.full((2,), cfg.success.tug.trigger_rad + 0.1),
        _obs(2, object_pos=torch.zeros(2, 3)),
        gap_bearing_rad=bearing,
        weight_n=torch.full((2,), 10.0),
    )
    force = tug.external_force()
    scale = float(cfg.success.tug.force_scale)
    assert force[0].tolist() == pytest.approx([10.0 * scale, 0.0, 0.0], abs=1e-5)
    assert force[1].tolist() == pytest.approx([0.0, 10.0 * scale, 0.0], abs=1e-5)


def test_tug_applies_no_force_before_it_arms(cfg, device):
    tug = TugTest(cfg.success, n_envs=2, device=device)
    assert bool((tug.external_force() == 0).all())


def test_tug_fails_an_object_that_slips_out_and_returns(cfg, device):
    """A violation is terminal.

    Checking retention only at the end would pass a grasp the object left and
    fell back into, which is not a grasp.
    """
    tug = TugTest(cfg.success, n_envs=1, device=device)
    held = torch.tensor([[0.0, 0.0, 0.5]])
    tug.arm(
        torch.tensor([cfg.success.tug.trigger_rad + 0.1]),
        _obs(1, object_pos=held),
        gap_bearing_rad=torch.zeros(1),
        weight_n=torch.tensor([19.6]),
    )
    escaped = held + torch.tensor(
        [[float(cfg.success.tug.max_rel_translation_m) * 2.0, 0.0, 0.0]]
    )
    tug.advance(_obs(1, object_pos=escaped))
    assert tug.phase.tolist() == [int(TugPhase.FAILED)]
    for _ in range(int(cfg.success.tug.hold_substeps) + 5):
        tug.advance(_obs(1, object_pos=held))
    assert not bool(tug.passed[0])
    assert bool(tug.finished[0])


def test_tug_passes_an_object_that_stays_for_the_whole_pull(cfg, device):
    tug = TugTest(cfg.success, n_envs=1, device=device)
    held = torch.tensor([[0.0, 0.0, 0.5]])
    tug.arm(
        torch.tensor([cfg.success.tug.trigger_rad + 0.1]),
        _obs(1, object_pos=held),
        gap_bearing_rad=torch.zeros(1),
        weight_n=torch.tensor([19.6]),
    )
    # Drifting, but inside tolerance the whole way.
    drift = float(cfg.success.tug.max_rel_translation_m) * 0.5
    for _ in range(int(cfg.success.tug.hold_substeps)):
        tug.advance(_obs(1, object_pos=held + torch.tensor([[drift, 0.0, 0.0]])))
    assert bool(tug.passed[0])
    assert bool((tug.external_force() == 0).all())  # force stops once finished


def test_tug_resets_per_env(cfg, device):
    tug = TugTest(cfg.success, n_envs=2, device=device)
    tug.arm(
        torch.full((2,), cfg.success.tug.trigger_rad + 0.1),
        _obs(2, object_pos=torch.zeros(2, 3)),
        gap_bearing_rad=torch.zeros(2),
        weight_n=torch.full((2,), 19.6),
    )
    tug.reset(torch.tensor([0]))
    assert tug.follows_policy.tolist() == [True, False]
    assert bool((tug.external_force()[0] == 0).all())
