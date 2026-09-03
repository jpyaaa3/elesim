"""Unit tests for the deployment side.

None of these build a scene: Genesis is a global singleton, and once one exists
in the process every other test in the run fails.  What is checked here is the
part that has to agree with the environment by convention rather than by
construction -- the manifest, the observation width, and the lift script's
timing -- plus that the torch-free path returns what the weights say.
"""

from __future__ import annotations

import elesim_sim.rl  # noqa: F401  # numpy-before-torch ordering
import json
import math
from pathlib import Path

import numpy as np
import pytest

from elesim_sim.rl.deploy import Interface, LiftScript, numpy_policy


def _manifest(tmp_path: Path, **over) -> Path:
    m = {
        "observation": {"dim": 16, "channels": [{"name": f"c{i}"} for i in range(16)]},
        "action": {
            "dim": 5,
            "channels": [
                {"name": "delta/linear", "scale_m": 0.04},
                {"name": "delta/roll", "scale_rad": 0.3},
                {"name": "delta/theta1", "scale_rad": 0.25},
                {"name": "delta/theta2", "scale_rad": 0.25},
                {"name": "lift_request", "threshold": 0.0},
            ],
        },
        "waypoint": {"home": [-0.1656, 0.0, -0.5934, 0.5934]},
        "timing": {"macro_step_s": 0.4, "substeps": 40, "substep_s": 0.01,
                   "move_fraction": 0.6, "max_steps": 28},
        "lift_script": {"roll_target_rad": 0.0,
                        "roll_rate_rad_per_substep": 0.015,
                        "settle_substeps": 80, "hold_substeps": 100},
    }
    for k, v in over.items():
        m[k] = v
    p = tmp_path / "interface.json"
    p.write_text(json.dumps(m), encoding="utf-8")
    return p


def test_the_manifest_carries_the_numbers_the_runner_needs(tmp_path):
    iface = Interface.from_manifest(_manifest(tmp_path))
    assert iface.obs_dim == 16 and iface.action_dim == 5
    assert iface.rate_limit == (0.04, 0.3, 0.25, 0.25)
    assert iface.macro_step_s == pytest.approx(0.4)
    assert iface.lift_settle_substeps == 80


def test_a_manifest_without_a_home_waypoint_is_refused(tmp_path):
    """The runner cannot invent a reset pose, and guessing one is worse."""
    p = _manifest(tmp_path, waypoint={"home": None})
    with pytest.raises(ValueError, match="home"):
        Interface.from_manifest(p)


def test_the_lift_rolls_to_the_target_and_stops_there(tmp_path):
    iface = Interface.from_manifest(_manifest(tmp_path))
    lift = LiftScript(iface)
    lift.start(-math.pi / 2)          # wrapped at roll -90
    seen = [lift.advance() for _ in range(200)]
    # It ramps towards zero from below and does not overshoot.
    assert seen[0] < seen[10] < seen[50] <= 0.0
    assert lift.roll_command == pytest.approx(0.0, abs=1e-9)
    assert min(seen) >= -math.pi / 2 - 1e-9


def test_the_lift_takes_the_measured_time_for_ninety_degrees(tmp_path):
    """0.31 s retains nothing and 1.05 s retains 72%, so the rate is the point."""
    iface = Interface.from_manifest(_manifest(tmp_path))
    lift = LiftScript(iface)
    lift.start(-math.pi / 2)
    steps = 0
    while lift.phase == "rolling":
        lift.advance()
        steps += 1
    assert steps * 0.01 == pytest.approx(1.05, abs=0.02)


def test_the_lift_runs_settle_then_hold_then_finishes(tmp_path):
    iface = Interface.from_manifest(_manifest(tmp_path))
    lift = LiftScript(iface)
    lift.start(-math.pi / 2)
    phases = []
    for _ in range(400):
        lift.advance()
        if not phases or phases[-1] != lift.phase:
            phases.append(lift.phase)
    assert phases == ["rolling", "settling", "holding", "done"]
    assert lift.finished


def test_a_lift_from_the_target_needs_no_rotation(tmp_path):
    iface = Interface.from_manifest(_manifest(tmp_path))
    lift = LiftScript(iface)
    lift.start(0.0)
    lift.advance()
    assert lift.phase in ("settling", "holding", "done")
    assert lift.roll_command == pytest.approx(0.0)


def test_the_numpy_path_reproduces_the_weights(tmp_path):
    """A hand-built two-layer network, so the arithmetic is checkable by hand."""
    p = tmp_path / "policy.npz"
    np.savez(
        p,
        norm_mean=np.zeros(2, dtype=np.float32),
        norm_std=np.ones(2, dtype=np.float32),
        norm_eps=np.asarray(0.0),
        w0=np.array([[1.0, 0.0], [0.0, -1.0]], dtype=np.float32),
        b0=np.zeros(2, dtype=np.float32),
        w1=np.array([[1.0, 1.0]], dtype=np.float32),
        b1=np.zeros(1, dtype=np.float32),
        n_layers=np.asarray(2),
    )
    run = numpy_policy(p)
    # Hidden layer is ELU: [2, -3] -> [2, 3] -> elu -> [2, 3] -> sum = 5.
    assert run([2.0, -3.0])[0, 0] == pytest.approx(5.0)
    # And a negative pre-activation is squashed, not clipped.
    out = run([-1.0, 0.0])[0, 0]
    assert out == pytest.approx(math.expm1(-1.0), abs=1e-6)


def test_the_numpy_path_applies_the_normaliser(tmp_path):
    p = tmp_path / "policy.npz"
    np.savez(
        p,
        norm_mean=np.array([10.0], dtype=np.float32),
        norm_std=np.array([2.0], dtype=np.float32),
        norm_eps=np.asarray(0.0),
        w0=np.array([[1.0]], dtype=np.float32),
        b0=np.zeros(1, dtype=np.float32),
        n_layers=np.asarray(1),
    )
    # (14 - 10) / 2 = 2, and a single layer has no activation after it.
    assert numpy_policy(p)([14.0])[0, 0] == pytest.approx(2.0)


def test_progress_can_be_supplied_instead_of_counted(tmp_path):
    """Training delayed the whole observation by 0 to 2 macro steps.

    A caller replaying a sim rollout has to pass the progress value that came
    with the rest of the vector; computing it from a local step count made an
    equivalence check drift while the actions themselves matched exactly.
    """
    import torch
    from elesim_sim.rl.deploy import DeployedPolicy

    class _Stub(DeployedPolicy):
        def __init__(self, iface):          # no weights, no mapper
            self.iface = iface
            self.step_index = 3

    stub = _Stub(Interface.from_manifest(_manifest(tmp_path)))
    counted = stub.observation(
        joint_estimate=[0] * 4, object_geometry=[0] * 7, load_proxy=[0] * 4,
    )
    assert float(counted[0, 15]) == pytest.approx(3 / 28)
    given = stub.observation(
        joint_estimate=[0] * 4, object_geometry=[0] * 7, load_proxy=[0] * 4,
        progress=0.5,
    )
    assert float(given[0, 15]) == pytest.approx(0.5)


def test_a_wrong_observation_width_is_refused(tmp_path):
    from elesim_sim.rl.deploy import DeployedPolicy

    class _Stub(DeployedPolicy):
        def __init__(self, iface):
            self.iface = iface
            self.step_index = 0

    stub = _Stub(Interface.from_manifest(_manifest(tmp_path)))
    with pytest.raises(ValueError, match="16"):
        stub.observation(joint_estimate=[0] * 4, object_geometry=[0] * 6,
                         load_proxy=[0] * 4)


def test_the_arm_config_comes_out_of_the_manifest(tmp_path):
    """The robot should need the exported files, not a copy of the config.

    A config file that has to travel alongside is one that can be the wrong one.
    """
    from elesim_sim.rl.deploy import arm_config_from_manifest

    m = json.loads(_manifest(tmp_path).read_text(encoding="utf-8"))
    m["waypoint"].update({
        "limits": {"linear_m": [-0.23, 0.0], "roll_rad": [-1.5708, 1.5708],
                   "theta_rad": [-0.6283, 0.6283]},
        "coupled_curl_cap": {"theta1_weight": 1.5, "cap_rad": 1.0647},
        "arm": {"n_seg": 5, "bend_joints": [f"j{i}" for i in range(10)],
                "linear_joint": "j_plate_housing", "roll_joint": "j_housing_wedge"},
        "sign_conventions": {"linear_axis_sign": 1.0, "roll_axis_sign": 1.0,
                             "bend_axis_sign": 1.0},
    })
    p = tmp_path / "m2.json"
    p.write_text(json.dumps(m), encoding="utf-8")

    arm = arm_config_from_manifest(p)
    assert arm.n_seg == 5 and len(arm.bend_joints) == 10
    assert arm.limits.roll_rad == (-1.5708, 1.5708)
    assert arm.limits.curl_limit_per_node_rad == pytest.approx(1.0647)
    assert arm.limits.theta1_curl_weight == pytest.approx(1.5)
    # The sign the RL side uses, not runtime.JointLayout's -1.
    assert arm.bend_axis_sign == 1.0
