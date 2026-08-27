"""Unit tests for the coupled bend limit on the waypoint mapper.

The cap exists to make one specific pose family unreachable -- both segments
curling the same way far enough that the backbone folds into its own housing --
without touching the S shapes, whose curls cancel and which the wrap needs.
So what is pinned here is which commands survive it and which are truncated.
"""

from __future__ import annotations

import math

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import pytest
import torch

from elesim_sim.rl.arm_kinematics import ArmWaypointMapper
from elesim_sim.rl.configs.loader import load_config


@pytest.fixture()
def cfg():
    return load_config()


def _mapper(cfg, n_envs=1):
    rate = cfg.macro_step.rate_limit
    return ArmWaypointMapper(
        cfg.arm,
        n_envs=n_envs,
        device=torch.device("cpu"),
        rate_limit=(rate.linear_m, rate.roll_rad, rate.theta_rad, rate.theta_rad),
    )


def test_opposite_signs_reach_full_magnitude(cfg):
    """The S shape is untouched: its curls cancel, so it cannot fold.

    Capping the sum of magnitudes rather than the signed sum would forbid this,
    and the arm needs it -- Home itself is an S.
    """
    m = _mapper(cfg)
    m.reset(home=torch.tensor([0.0, 0.0, 0.0, 0.0]))
    limit = float(cfg.arm.limits.bend_per_node_rad)
    for _ in range(20):
        m.apply_action(torch.tensor([[0.0, 0.0, 1.0, -1.0]]))
    assert m.waypoint[0, 2].item() == pytest.approx(limit, abs=1e-4)
    assert m.waypoint[0, 3].item() == pytest.approx(-limit, abs=1e-4)


def _curl(cfg, m):
    w = float(cfg.arm.limits.theta1_curl_weight)
    return (w * m.waypoint[0, 2] + m.waypoint[0, 3]).item()


def test_same_sign_curl_stops_at_the_limit(cfg):
    m = _mapper(cfg)
    m.reset(home=torch.tensor([0.0, 0.0, 0.0, 0.0]))
    cap = float(cfg.arm.limits.curl_limit_per_node_rad)
    for _ in range(20):
        m.apply_action(torch.tensor([[0.0, 0.0, 1.0, 1.0]]))
    assert _curl(cfg, m) == pytest.approx(cap, abs=1e-4)
    # And in the other direction.
    for _ in range(40):
        m.apply_action(torch.tensor([[0.0, 0.0, -1.0, -1.0]]))
    assert _curl(cfg, m) == pytest.approx(-cap, abs=1e-4)


def test_the_three_wrapping_poses_are_reachable(cfg):
    """(15, 36), (18, 36) and (21, 30) deg per node all wrap the object.

    A cap that excluded any of them would make the task harder than the arm
    is, which is the whole reason the limit is 63 rather than the 60 the
    free-space sweep gives.
    """
    for t1, t2 in ((15, 36), (18, 36), (21, 30)):
        m = _mapper(cfg)
        m.reset(home=torch.tensor([0.0, 0.0, 0.0, 0.0]))
        target = torch.tensor([[0.0, 0.0, math.radians(t1), math.radians(t2)]])
        rate = m.rate_limit.unsqueeze(0)
        for _ in range(20):
            m.apply_action(((target - m.waypoint) / rate).clamp(-1.0, 1.0))
        assert m.waypoint[0, 2].item() == pytest.approx(math.radians(t1), abs=1e-4), (t1, t2)
        assert m.waypoint[0, 3].item() == pytest.approx(math.radians(t2), abs=1e-4), (t1, t2)


def test_the_deep_folds_are_unreachable(cfg):
    """Cells the free-space sweep found folding, beyond the cap."""
    cap = float(cfg.arm.limits.curl_limit_per_node_rad)
    for t1, t2 in ((24, 36), (30, 24), (36, 36), (-36, -24), (-24, -36)):
        m = _mapper(cfg)
        m.reset(home=torch.tensor([0.0, 0.0, 0.0, 0.0]))
        target = torch.tensor([[0.0, 0.0, math.radians(t1), math.radians(t2)]])
        rate = m.rate_limit.unsqueeze(0)
        for _ in range(20):
            m.apply_action(((target - m.waypoint) / rate).clamp(-1.0, 1.0))
        assert abs(_curl(cfg, m)) <= cap + 1e-4, (t1, t2)
        reached_t2 = m.waypoint[0, 3].item()
        assert abs(reached_t2 - math.radians(t2)) > 1e-3 or abs(
            m.waypoint[0, 2].item() - math.radians(t1)
        ) > 1e-3, (t1, t2)


def test_the_wrap_pose_is_still_reachable(cfg):
    """18 + 36 deg per node sits exactly on the cap.

    The scripted wrap uses it, so a cap that excluded it would make the task
    unsolvable rather than safer.
    """
    m = _mapper(cfg)
    m.reset(home=torch.tensor([0.0, 0.0, 0.0, 0.0]))
    target = torch.tensor([[0.0, 0.0, math.radians(18), math.radians(36)]])
    rate = m.rate_limit.unsqueeze(0)
    for _ in range(20):
        m.apply_action(((target - m.waypoint) / rate).clamp(-1.0, 1.0))
    assert m.waypoint[0, 2].item() == pytest.approx(math.radians(18), abs=1e-4)
    assert m.waypoint[0, 3].item() == pytest.approx(math.radians(36), abs=1e-4)


def test_the_step_is_truncated_not_rescaled(cfg):
    """A DoF the policy did not move stays where it was.

    Rescaling the pair to fit would drag theta2 back when only theta1 was
    commanded, which reads as the arm undoing a command it was given.
    """
    m = _mapper(cfg)
    cap = float(cfg.arm.limits.curl_limit_per_node_rad)
    start_t2 = math.radians(36)
    m.reset(home=torch.tensor([0.0, 0.0, 0.0, start_t2]))
    # theta1 alone, hard, until it can go no further.  One rate step is
    # 14.3 deg, so this takes more than one.
    for _ in range(10):
        m.apply_action(torch.tensor([[0.0, 0.0, 1.0, 0.0]]))
    assert m.waypoint[0, 3].item() == pytest.approx(start_t2, abs=1e-6)
    assert _curl(cfg, m) == pytest.approx(cap, abs=1e-4)


def test_reset_projects_an_infeasible_home(cfg):
    """A Home outside the cap is scaled in rather than accepted."""
    m = _mapper(cfg)
    cap = float(cfg.arm.limits.curl_limit_per_node_rad)
    lim = float(cfg.arm.limits.bend_per_node_rad)
    m.reset(home=torch.tensor([0.0, 0.0, lim, lim]))
    assert _curl(cfg, m) == pytest.approx(cap, abs=1e-4)
    # Proportional, so the shape of the coil survives.
    assert m.waypoint[0, 2].item() == pytest.approx(m.waypoint[0, 3].item(), abs=1e-6)


def test_configured_home_is_feasible(cfg):
    """Home is an S shape, so the cap must leave it alone."""
    m = _mapper(cfg)
    home = torch.tensor(list(cfg.arm.home_waypoint))
    m.reset(home=home)
    assert torch.allclose(m.waypoint[0], home.to(torch.float32), atol=1e-6)


def test_disabling_the_cap_restores_the_old_reach(cfg):
    import dataclasses

    limits = dataclasses.replace(cfg.arm.limits, curl_limit_per_node_rad=None)
    arm = dataclasses.replace(cfg.arm, limits=limits)
    rate = cfg.macro_step.rate_limit
    m = ArmWaypointMapper(
        arm,
        n_envs=1,
        device=torch.device("cpu"),
        rate_limit=(rate.linear_m, rate.roll_rad, rate.theta_rad, rate.theta_rad),
    )
    m.reset(home=torch.tensor([0.0, 0.0, 0.0, 0.0]))
    lim = float(cfg.arm.limits.bend_per_node_rad)
    for _ in range(20):
        m.apply_action(torch.tensor([[0.0, 0.0, 1.0, 1.0]]))
    assert m.waypoint[0, 2].item() == pytest.approx(lim, abs=1e-4)
    assert m.waypoint[0, 3].item() == pytest.approx(lim, abs=1e-4)
