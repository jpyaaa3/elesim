from __future__ import annotations

import json
import math
from pathlib import Path

import pytest

from elesim_pilot.pick.wrap_policy import DeployedPolicy, Interface, LiftScript


def _manifest(tmp_path: Path) -> Path:
    path = tmp_path / "interface.json"
    path.write_text(json.dumps({
        "observation": {"dim": 16},
        "action": {
            "dim": 5,
            "channels": [
                {"scale_m": 0.04},
                {"scale_rad": 0.3},
                {"scale_rad": 0.25},
                {"scale_rad": 0.25},
                {"threshold": 0.0},
            ],
        },
        "waypoint": {
            "home": [-0.1656, 0.0, -0.2, 0.2],
            "limits": {
                "linear_m": [-0.23, 0.0],
                "roll_rad": [-1.5708, 1.5708],
                "theta_rad": [-0.6283, 0.6283],
            },
            "coupled_curl_cap": {"theta1_weight": 1.5, "cap_rad": 1.0647},
        },
        "timing": {
            "macro_step_s": 0.4,
            "substeps": 40,
            "move_fraction": 0.6,
            "max_steps": 28,
        },
        "lift_script": {
            "roll_target_rad": 0.0,
            "roll_rate_rad_per_substep": 0.015,
            "settle_substeps": 80,
            "hold_substeps": 100,
        },
    }), encoding="utf-8")
    return path


def test_manifest_is_the_only_sim_to_pilot_runtime_contract(tmp_path: Path) -> None:
    interface = Interface.from_manifest(_manifest(tmp_path))
    assert interface.obs_dim == 16
    assert interface.rate_limit == (0.04, 0.3, 0.25, 0.25)
    assert interface.lower == (-0.23, -1.5708, -0.6283, -0.6283)
    assert interface.upper == (0.0, 1.5708, 0.6283, 0.6283)


def test_pilot_executes_exported_action_without_importing_sim(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    torch = pytest.importorskip("torch")

    class Policy:
        def eval(self):
            return self

        def __call__(self, observation):
            assert tuple(observation.shape) == (1, 16)
            return torch.tensor([[1.0, -1.0, 0.5, -0.5, 1.0]])

    monkeypatch.setattr(torch.jit, "load", lambda *_args, **_kwargs: Policy())
    deployed = DeployedPolicy(tmp_path / "policy.pt", _manifest(tmp_path))
    waypoint, lift = deployed.act(
        joint_estimate=(0.0,) * 4,
        object_geometry=(0.0,) * 7,
        load_proxy=(0.0,) * 4,
    )
    assert waypoint == pytest.approx((-0.1256, -0.3, -0.075, 0.075))
    assert lift is True


def test_lift_trajectory_preserves_exported_rate(tmp_path: Path) -> None:
    lift = LiftScript(Interface.from_manifest(_manifest(tmp_path)))
    lift.start(-math.pi / 2)
    steps = 0
    while lift.phase == "rolling":
        lift.advance()
        steps += 1
    assert steps * 0.01 == pytest.approx(1.05, abs=0.02)
