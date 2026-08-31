"""Unit tests for the wrap-grasp action.

The runner is deliberately free of pilot imports -- three callables are its
whole interface to the arm -- so the loop can be exercised without a robot or a
simulator, which is the only way these get run at all.
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Sequence

import pytest

from elesim_pilot.pick.wrap import (
    WrapActions, WrapGraspConfig, WrapGraspRunner,
)

DEPLOY = Path(__file__).resolve().parents[2] / "deploy"
pytestmark = pytest.mark.skipif(
    not (DEPLOY / "policy.pt").is_file(),
    reason="deploy/policy.pt 가 없습니다 (elesim_sim.rl.export 로 생성)",
)


def _cfg(**kw) -> WrapGraspConfig:
    return WrapGraspConfig(
        policy_path=str(DEPLOY / "policy.pt"),
        manifest_path=str(DEPLOY / "interface.json"),
        **kw,
    )


def test_missing_files_say_where_they_were_looked_for(tmp_path):
    """`torch.jit.load` on a missing path raises without naming the search, and
    the pilot's working directory is not obvious from inside a container.
    """
    import pytest as _pytest
    from elesim_pilot.pick.wrap import WrapGraspRunner

    cfg = WrapGraspConfig(
        policy_path="nope/policy.pt", manifest_path="nope/interface.json",
        search_roots=(str(tmp_path),),
    )
    with _pytest.raises(FileNotFoundError, match="찾은 곳"):
        WrapGraspRunner(cfg, read_joints=lambda: (0, 0, 0, 0),
                        command_waypoint=lambda w, t: True)


def test_bare_filenames_under_a_search_root_are_found(tmp_path):
    """`cp deploy/* roles/pilot/config/policy/` is the documented move."""
    import shutil
    from elesim_pilot.pick.wrap import WrapGraspRunner

    for name in ("policy.pt", "interface.json"):
        shutil.copy2(DEPLOY / name, tmp_path / name)
    cfg = WrapGraspConfig(
        policy_path="config/policy/policy.pt",
        manifest_path="config/policy/interface.json",
        search_roots=(str(tmp_path),),
    )
    runner = WrapGraspRunner(cfg, read_joints=lambda: (0, 0, 0, 0),
                            command_waypoint=lambda w, t: True)
    assert runner.policy.iface.obs_dim == 16


class _Arm:
    """An arm that reaches whatever it is told, and remembers the order."""

    def __init__(self, *, reach=True):
        self.q = [-0.1656, 0.0, -0.5934, 0.5934]
        self.commanded: list[tuple] = []
        self.rolls: list[float] = []
        self.reach = reach

    def read(self) -> Sequence[float]:
        return tuple(self.q)

    def command(self, waypoint, timeout_s) -> bool:
        self.commanded.append(tuple(waypoint))
        if self.reach:
            self.q = list(waypoint)
        return self.reach

    def command_roll(self, roll: float) -> None:
        self.rolls.append(float(roll))
        self.q[1] = float(roll)


def test_it_steps_the_policy_and_commands_every_waypoint():
    arm = _Arm()
    out = WrapGraspRunner(
        _cfg(max_steps=5), read_joints=arm.read, command_waypoint=arm.command,
        command_roll=arm.command_roll, sleep=lambda _s: None,
    ).run()
    assert out.steps == len(arm.commanded) > 0
    assert all(len(w) == 4 for w in arm.commanded)


def test_a_waypoint_the_arm_cannot_reach_stops_the_attempt():
    arm = _Arm(reach=False)
    out = WrapGraspRunner(
        _cfg(), read_joints=arm.read, command_waypoint=arm.command,
    ).run()
    assert out.steps == 1 and "도달" in out.reason


def test_cancelling_stops_before_the_next_command():
    arm = _Arm()
    flag = {"stop": False}

    def cancelled() -> bool:
        was = flag["stop"]
        flag["stop"] = True         # cancel after the first check
        return was

    out = WrapGraspRunner(
        _cfg(), read_joints=arm.read, command_waypoint=arm.command,
        cancelled=cancelled,
    ).run()
    assert out.reason == "cancelled"
    assert len(arm.commanded) <= 1


def test_the_lift_ramps_the_roll_and_leaves_the_rest_alone():
    arm = _Arm()
    runner = WrapGraspRunner(
        _cfg(), read_joints=arm.read, command_waypoint=arm.command,
        command_roll=arm.command_roll, sleep=lambda _s: None,
    )
    arm.q[1] = -1.5673                     # wrapped at roll -90
    assert runner._run_lift((0.0, -1.5673, 0.0, 0.0))
    assert arm.rolls[0] < arm.rolls[len(arm.rolls) // 2] <= 0.0
    assert arm.rolls[-1] == pytest.approx(0.0, abs=1e-9)


def test_a_lift_request_without_a_roll_path_is_reported_not_crashed():
    arm = _Arm()
    runner = WrapGraspRunner(
        _cfg(), read_joints=arm.read, command_waypoint=arm.command,
    )
    assert runner._run_lift((0.0, -1.5673, 0.0, 0.0)) is False


def test_the_load_proxy_is_zero_by_default():
    """Motor current in mA is not joint torque, and the normaliser is frozen."""
    arm = _Arm()
    runner = WrapGraspRunner(
        _cfg(), read_joints=arm.read, command_waypoint=arm.command,
        read_load=lambda: (999.0, 999.0, 999.0, 999.0),
    )
    assert tuple(runner._load_proxy()) == (0.0, 0.0, 0.0, 0.0)


def test_currents_map_onto_the_four_channels():
    """The sim reports one bend load into two channels, so the segments average."""
    got = WrapGraspRunner.load_proxy_from_currents(
        {"linear": 100, "roll": 200, "seg1": 300, "seg2": 500}
    )
    assert got == (100.0, 200.0, 400.0, 400.0)
    # Names are matched loosely, the way the UI panel does it.
    assert WrapGraspRunner.load_proxy_from_currents({"S1": 10, "s2": 30})[2] == 20.0
    # ...and a missing joint reads zero rather than raising.
    assert WrapGraspRunner.load_proxy_from_currents({})[0] == 0.0


def test_a_bad_joint_reading_is_refused():
    arm = _Arm()
    arm.q = [0.0, 0.0, 0.0]                # three, not four
    out = WrapGraspRunner(
        _cfg(), read_joints=arm.read, command_waypoint=arm.command,
    ).run()
    assert "4개" in out.reason


def test_the_mixin_defaults_to_the_nominal_condition():
    cfg = WrapActions().wrap_grasp_config()
    assert len(cfg.object_geometry) == 7
    assert cfg.zero_load_proxy is True
