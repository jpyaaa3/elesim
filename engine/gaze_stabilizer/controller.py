from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from engine.gaze_stabilizer.config import GazeStabilizerConfig
from engine.visual_servoing.uv_jacobian import solve_uv_control_delta


__all__ = ["GazeStabilizerConfig", "GazeStabilizer"]

class GazeStabilizer:
    """Display-space UV gaze stabilizer with optional additive base feedforward."""

    def __init__(self, config: GazeStabilizerConfig) -> None:
        self._config = config

    @property
    def config(self) -> GazeStabilizerConfig:
        return self._config

    def compute_display_u_delta(
        self,
        *,
        uv_error: np.ndarray,
        jacobian: np.ndarray,
        base_ang_vel_body: np.ndarray | None = None,
    ) -> np.ndarray:
        """Return du = [d_roll, d_s1, d_s2] in display control space."""
        cfg = self._config
        du = np.zeros(3, dtype=float)
        if cfg.enable_feedback:
            du += solve_uv_control_delta(
                uv_error=np.asarray(uv_error, dtype=float).reshape(2),
                jacobian=np.asarray(jacobian, dtype=float).reshape(2, 3),
                damping=float(cfg.jacobian_damping),
                gain=float(cfg.uv_gain),
                max_abs_delta=(cfg.max_du_roll, cfg.max_du_s1, cfg.max_du_s2),
            )
        if cfg.enable_base_ff and base_ang_vel_body is not None:
            ang = np.asarray(base_ang_vel_body, dtype=float).reshape(3)
            du_ff = np.array(
                [
                    -float(cfg.base_ff_gain_roll) * float(ang[0]),
                    -float(cfg.base_ff_gain_pitch) * float(ang[1]),
                    -float(cfg.base_ff_gain_yaw) * float(ang[2]),
                ],
                dtype=float,
            )
            du += du_ff
        limits = np.array([cfg.max_du_roll, cfg.max_du_s1, cfg.max_du_s2], dtype=float)
        return np.clip(du, -limits, limits)
