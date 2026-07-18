"""Image-UV Jacobian helpers for camera selfish visual servoing."""

from __future__ import annotations

from typing import Sequence

import numpy as np


UV_CONTROL_AXIS_NAMES = ("roll", "s1", "s2")


def damped_pseudoinverse(jacobian: np.ndarray, damping: float) -> np.ndarray:
    """Return J+ = J^T (J J^T + lambda^2 I)^-1 for a 2xN image Jacobian."""
    j = np.asarray(jacobian, dtype=float)
    if j.ndim != 2 or j.shape[0] != 2:
        raise ValueError(f"jacobian must have shape (2, N), got {j.shape}")
    lam = float(max(damping, 1e-9))
    jj_t = j @ j.T
    inv = np.linalg.inv(jj_t + (lam * lam) * np.eye(2, dtype=float))
    return j.T @ inv


def default_uv_jacobian(
    *,
    center_u_gain: float,
    center_v_gain: float,
    seg1_coupling: float = 0.5,
    seg2_coupling: float = 0.5,
) -> np.ndarray:
    """
    Initial d(image_uv_error)/d(display_control_u) for roll/s1/s2.

    seg1 (inner) typically has larger tip leverage than seg2; use a lower
    seg1_coupling so the pseudoinverse commands less inner-joint motion.
    """
    u_gain = float(max(abs(float(center_u_gain)), 1e-6))
    v_gain = float(max(abs(float(center_v_gain)), 1e-6))
    s1 = float(max(abs(float(seg1_coupling)), 1e-6))
    s2 = float(max(abs(float(seg2_coupling)), 1e-6))
    j = np.zeros((2, 3), dtype=float)
    # Positive roll display-u moves normalized image u left on this arm.
    j[0, 0] = -1.0 / u_gain
    # s1/s2 oppose for v (see _apply_pick_center_step: s1=+gain*v_delta, s2=-gain*v_delta).
    j[1, 1] = -s1 / v_gain
    j[1, 2] = +s2 / v_gain
    return j


def broyden_update_uv_jacobian(
    jacobian: np.ndarray,
    *,
    control_delta: Sequence[float],
    uv_delta: Sequence[float],
    alpha: float = 0.35,
    min_control_norm: float = 0.25,
    max_uv_delta_norm: float = 1.25,
    max_column_norm: float = 0.50,
) -> np.ndarray:
    """Rank-one online update from observed duv ~= J * dcontrol."""
    j = np.asarray(jacobian, dtype=float).reshape(2, 3)
    du = np.asarray(control_delta, dtype=float).reshape(3)
    dy = np.asarray(uv_delta, dtype=float).reshape(2)
    du_norm_sq = float(du @ du)
    if du_norm_sq < float(min_control_norm) ** 2:
        return j.copy()
    if float(np.linalg.norm(dy)) > float(max_uv_delta_norm):
        return j.copy()
    residual = dy - (j @ du)
    updated = j + float(np.clip(alpha, 0.0, 1.0)) * np.outer(residual, du) / du_norm_sq
    for col_idx in range(updated.shape[1]):
        norm = float(np.linalg.norm(updated[:, col_idx]))
        if norm > float(max_column_norm):
            updated[:, col_idx] *= float(max_column_norm) / norm
    return updated


def solve_uv_control_delta(
    *,
    uv_error: Sequence[float],
    jacobian: np.ndarray,
    damping: float = 0.03,
    gain: float = 1.0,
    max_abs_delta: Sequence[float] = (6.0, 6.0, 6.0),
) -> np.ndarray:
    """Solve dcontrol = -gain * J+ * uv_error for roll/s1/s2 display-u."""
    err = np.asarray(uv_error, dtype=float).reshape(2)
    j = np.asarray(jacobian, dtype=float).reshape(2, 3)
    pinv = damped_pseudoinverse(j, float(damping))
    delta = -float(gain) * (pinv @ err)
    limits = np.asarray(max_abs_delta, dtype=float).reshape(3)
    limits = np.maximum(np.abs(limits), 1e-6)
    return np.clip(delta, -limits, limits)
