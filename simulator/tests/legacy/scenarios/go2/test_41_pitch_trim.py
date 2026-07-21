from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from elesim_simulator.robot.go2.mpc.payload_model import backward_pitch_trim_rad, payload_pitch_trim_rad


@dataclass
class _TrimCfg:
    pitch_trim_gain_x_forward: float = 0.0
    pitch_trim_gain_x_backward: float = -0.15
    pitch_trim_gain_z: float = 0.9
    pitch_trim_z_ref_m: float = 0.12
    pitch_trim_max_rad: float = 0.05


def test_backward_z_only_trim_negative() -> None:
    trim = backward_pitch_trim_rad(np.array([0.0, 0.0, 0.25]), gain_z=0.9, z_ref_m=0.12, max_trim_rad=0.05)
    assert trim < 0.0


def test_payload_trim_xz_forward_vs_backward() -> None:
    cfg = _TrimCfg(pitch_trim_gain_x_forward=0.5, pitch_trim_max_rad=0.2)
    com = np.array([0.10, 0.0, 0.20])
    fwd = payload_pitch_trim_rad(com, vx=0.35, config=cfg)
    bwd = payload_pitch_trim_rad(com, vx=-0.35, config=cfg)
    assert fwd != bwd
