"""The reset walks a polyline from Home, not a straight line to the near pose.

A straight line between two waypoints is not a path.  The direct Home-to-wrap
line runs through the pole: with no policy acting at all, the reset alone left
the object 11 deg tilted at t = 0.8, 17 deg at 0.7 and 23 deg at 0.6 against a
60 deg topple limit, and the run that used it sat at t = 0.70 for 210 iterations
with 62% of episodes ending as topples.  Three via points bring the worst tilt
over t = 0.1 to 0.9 down to 4.8 deg.

These tests are about the interpolation arithmetic, which is what makes the via
points reachable at all; the poses themselves are measured, not asserted.
"""

from __future__ import annotations

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import dataclasses

import pytest
import torch

from elesim_sim.rl.configs.loader import StartPoseConfig, load_config


def _interp(home, pts, t):
    """The polyline walk `_apply_start_pose` does, on plain tensors."""
    home = torch.tensor([home], dtype=torch.float32)
    pts = torch.cat((home, torch.tensor(pts, dtype=torch.float32)), dim=0)
    t = torch.tensor([[float(t)]])
    legs = pts.shape[0] - 1
    u = (t * legs).clamp(0.0, float(legs))
    i = u.floor().clamp(max=float(legs - 1)).long()
    frac = u - i.to(u.dtype)
    a, b = pts[i.squeeze(-1)], pts[(i + 1).squeeze(-1)]
    return (a + frac * (b - a))[0]


HOME = (0.0, 0.0, 0.0, 0.0)
PTS = [(1.0, 0.0, 0.0, 0.0), (1.0, 1.0, 0.0, 0.0), (1.0, 1.0, 1.0, 0.0)]


def test_t_zero_is_home_and_t_one_is_the_near_pose():
    assert torch.allclose(_interp(HOME, PTS, 0.0), torch.tensor(HOME))
    assert torch.allclose(_interp(HOME, PTS, 1.0), torch.tensor(PTS[-1]))


def test_each_leg_takes_an_equal_share_of_t():
    for k, pt in enumerate(PTS):
        assert torch.allclose(
            _interp(HOME, PTS, (k + 1) / len(PTS)), torch.tensor(pt), atol=1e-6
        )


def test_it_interpolates_within_a_leg():
    # Half way along the second leg: roll half raised, everything else at the
    # first via point.
    got = _interp(HOME, PTS, 0.5)
    assert got.tolist() == pytest.approx([1.0, 0.5, 0.0, 0.0])


def test_a_single_point_is_the_old_straight_line():
    """Configs from before via points still behave exactly as they did."""
    one = [(1.0, 2.0, 3.0, 4.0)]
    for t in (0.0, 0.25, 0.5, 0.75, 1.0):
        assert _interp(HOME, one, t).tolist() == pytest.approx(
            [t * 1.0, t * 2.0, t * 3.0, t * 4.0]
        )


def test_waypoints_puts_the_near_pose_last():
    cfg = StartPoseConfig()
    pts = cfg.waypoints()
    assert pts[-1] == tuple(float(v) for v in cfg.near_waypoint)
    assert len(pts) == len(cfg.path) + 1


def test_an_empty_path_leaves_just_the_near_pose():
    cfg = dataclasses.replace(StartPoseConfig(), path=())
    assert cfg.waypoints() == (tuple(float(v) for v in cfg.near_waypoint),)


def test_the_shipped_path_ends_where_the_wrap_is():
    cfg = load_config().start_pose
    assert cfg.waypoints()[-1] == tuple(float(v) for v in cfg.near_waypoint)
    # Every via point is inside the joint limits the mapper clamps to.
    limits = load_config().arm.limits
    for lin, roll, t1, t2 in cfg.waypoints():
        assert limits.linear_m[0] <= lin <= limits.linear_m[1]
        assert limits.roll_rad[0] <= roll <= limits.roll_rad[1]
        assert abs(t1) <= limits.bend_per_node_rad
        assert abs(t2) <= limits.bend_per_node_rad
