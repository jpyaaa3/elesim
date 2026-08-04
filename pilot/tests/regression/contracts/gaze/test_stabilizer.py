from __future__ import annotations

import numpy as np

from elesim_pilot.gaze.stabilizer import GazeStabilizer, GazeStabilizerConfig
from elesim_pilot.vision.visual_servoing.uv_jacobian import default_uv_jacobian, solve_uv_control_delta


def test_positive_u_err_reduces_error_direction() -> None:
    j = default_uv_jacobian(center_u_gain=1.0, center_v_gain=1.0)
    s = np.array([0.2, 0.0], dtype=float)
    du = solve_uv_control_delta(uv_error=s, jacobian=j, damping=0.03, gain=1.0)
    assert float(np.dot(j @ du, s)) < 0.0


def test_gaze_stabilizer_additive_ff() -> None:
    cfg = GazeStabilizerConfig(enable_feedback=False, enable_base_ff=True, base_ff_gain_pitch=0.1)
    stab = GazeStabilizer(cfg)
    du = stab.compute_display_u_delta(
        uv_error=np.zeros(2),
        jacobian=np.eye(2, 3),
        base_ang_vel_body=np.array([0.0, 1.0, 0.0]),
    )
    assert du[1] < 0.0
