from __future__ import annotations

import json
import math
from types import SimpleNamespace
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


def test_manifest_accepts_only_the_two_defined_legacy_layouts(tmp_path: Path) -> None:
    path = _manifest(tmp_path)
    raw = json.loads(path.read_text())
    raw["observation"] = {"dim": 13}
    path.write_text(json.dumps(raw))
    with pytest.raises(ValueError, match="12- or 16"):
        Interface.from_manifest(path)


def test_manifest_mapping_is_checked_against_actual_configured_bounds(tmp_path: Path) -> None:
    interface = Interface.from_manifest(_manifest(tmp_path))
    # A calibrated instance can legitimately expose the older 230 mm travel.
    mapping = SimpleNamespace(
        linear_q_min_m=-0.23, linear_q_max_m=0.0,
        roll_q_min_rad=-1.5708, roll_q_max_rad=1.5708,
        seg1_q_min_rad=-0.6283, seg1_q_max_rad=0.6283,
        seg2_q_min_rad=-0.6283, seg2_q_max_rad=0.6283,
    )
    interface.validate_mapping(mapping)
    mapping.linear_q_min_m = -0.16
    with pytest.raises(ValueError, match="channel 0"):
        interface.validate_mapping(mapping)


def test_observation_rejects_non_finite_inputs_without_torch(tmp_path: Path) -> None:
    interface = Interface.from_manifest(_manifest(tmp_path))
    deployed = object.__new__(DeployedPolicy)
    deployed.iface = interface
    deployed.step_index = 0
    deployed._torch = SimpleNamespace(tensor=lambda *args, **kwargs: args[0])
    with pytest.raises(ValueError, match="non-finite"):
        deployed.observation(
            joint_estimate=(0.0, 0.0, 0.0, 0.0),
            object_geometry=(0.0, 0.0, 0.0, 0.0, float("nan"), 0.0, 0.0),
            load_proxy=(0.0,) * 4,
        )


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
