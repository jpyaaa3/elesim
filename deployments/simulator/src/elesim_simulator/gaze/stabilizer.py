from __future__ import annotations

import math
from dataclasses import dataclass, fields, replace
from typing import Any

import numpy as np

from elesim_simulator.vision.visual_servoing.uv_jacobian import solve_uv_control_delta


@dataclass(frozen=True)
class GazeStabilizerConfig:
    enable_feedback: bool = True
    enable_base_ff: bool = False
    uv_gain: float = 1.0
    base_ff_gain_pitch: float = 0.0
    base_ff_gain_roll: float = 0.0
    base_ff_gain_yaw: float = 0.0
    max_du_roll: float = 1.0
    max_du_s1: float = 1.0
    max_du_s2: float = 1.0
    jacobian_damping: float = 0.03
    hz: float = 20.0
    center_tol: float = 0.06
    center_u_gain: float = 18.0
    center_v_gain: float = 18.0
    center_roll_max: float = 8.0
    center_seg_max: float = 8.0
    step_scale: float = 1.0
    enable_roll: bool = False
    center_u_kd: float = 0.0
    center_v_kd: float = 4.0
    center_d_seg_max: float = 4.0
    d_filter_alpha: float = 0.35
    max_seg_du_per_tick: float = 1.5
    cmd_settle_s: float = 0.10
    center_u_seg_s2_scale: float = 0.55
    center_u_seg_s1_scale: float = 0.35
    fine_err_max: float = 0.11
    fine_settle_scale: float = 0.35
    fov_margin: float = 0.08
    clamp_go2_vel_on_large_error: bool = False
    preview_enable: bool = False
    preview_tau_s: float = 0.08
    preview_b_pitch: float = 0.05
    preview_q_u: float = 1.0
    preview_q_v: float = 1.0
    preview_r_roll: float = 0.01
    preview_r_s1: float = 0.01
    preview_r_s2: float = 0.01
    preview_max_du_roll: float = 1.0
    preview_max_du_seg: float = 1.5
    preview_lowpass_alpha: float = 0.35
    walking_gaze_mode: str = "uv_ff"
    command_ref_enable: bool = False
    command_ref_max_lead: float = 40.0


__all__ = [
    "GazeStabilizerConfig",
    "GazeStabilizer",
    "gaze_config_to_dict",
    "patch_gaze_config",
    "resolve_walking_gaze_mode",
]


def gaze_config_to_dict(cfg: GazeStabilizerConfig) -> dict[str, Any]:
    return {field.name: getattr(cfg, field.name) for field in fields(GazeStabilizerConfig)}


def _coerce_gaze_config_value(name: str, raw: Any, current: Any) -> Any:
    if isinstance(current, bool):
        if isinstance(raw, str):
            key = raw.strip().lower()
            if key in ("1", "true", "yes", "on"):
                return True
            if key in ("0", "false", "no", "off"):
                return False
            raise ValueError(f"invalid boolean for {name}: {raw!r}")
        return bool(raw)
    if isinstance(current, (float, int)) and not isinstance(current, bool):
        value = float(raw)
        if not math.isfinite(value):
            raise ValueError(f"invalid finite float for {name}: {raw!r}")
        return value
    return str(raw).strip()


def patch_gaze_config(cfg: GazeStabilizerConfig, patch: dict[str, Any]) -> GazeStabilizerConfig:
    if not isinstance(patch, dict):
        raise ValueError("gaze config patch must be a dict")
    allowed = {field.name for field in fields(GazeStabilizerConfig)}
    updates: dict[str, Any] = {}
    for key, raw in dict(patch).items():
        name = str(key).strip()
        if name not in allowed:
            continue
        updates[name] = _coerce_gaze_config_value(name, raw, getattr(cfg, name))
    if not updates:
        return cfg
    next_cfg = replace(cfg, **updates)
    resolve_walking_gaze_mode(next_cfg)
    return next_cfg


def resolve_walking_gaze_mode(cfg: GazeStabilizerConfig, override: str | None = None) -> str:
    mode = str(override or cfg.walking_gaze_mode or "uv_ff").strip().lower()
    if mode not in ("uv", "uv_ff", "pitch_preview"):
        raise ValueError(f"invalid walking gaze mode {mode!r} (uv|uv_ff|pitch_preview)")
    if mode == "pitch_preview" and not bool(cfg.preview_enable):
        raise ValueError("walking gaze_mode=pitch_preview requires gaze_preview_enable=true")
    return mode


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
