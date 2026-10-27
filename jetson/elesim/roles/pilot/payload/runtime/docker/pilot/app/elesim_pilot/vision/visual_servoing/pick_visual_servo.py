"""Camera-frame visual servo helpers for Look-then-Advance pick control."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np

# LOOK_ALIGN Jacobian axes: roll, theta1, theta2 (linear fixed).
LOOK_JACOBIAN_AXIS_NAMES = ("roll", "theta1", "theta2")


@dataclass(frozen=True)
class LookAlignLimits:
    xy_threshold_m: float = 0.010
    xy_deadband_m: float = 0.008


@dataclass(frozen=True)
class LookGains:
    theta1_per_error_x: float = 1.0
    theta2_per_error_y: float = 1.0
    roll_per_error_x: float = 1.0
    max_step_rad: float = 0.02
    max_step_roll_rad: float = 0.02


@dataclass(frozen=True)
class JacobianLookGains:
    gain: float = 0.6
    damping: float = 0.02
    max_step_roll_rad: float = 0.02
    max_step_theta_rad: float = 0.02
    column_norm_min: float = 1e-5


@dataclass(frozen=True)
class Q4Delta:
    linear_m: float = 0.0
    roll_rad: float = 0.0
    theta1_rad: float = 0.0
    theta2_rad: float = 0.0


def camera_xy_error(
    mu_camera: Sequence[float],
    desired_xy: Sequence[float],
) -> tuple[float, float, float]:
    mu = np.asarray(mu_camera, dtype=float).reshape(3)
    desired = np.asarray(desired_xy, dtype=float).reshape(2)
    ex = float(mu[0] - desired[0])
    ey = float(mu[1] - desired[1])
    return ex, ey, float(np.hypot(ex, ey))


def error_vector_2d(ex: float, ey: float) -> np.ndarray:
    return np.array([float(ex), float(ey)], dtype=float)


def look_align_ok(ex: float, ey: float, limits: LookAlignLimits) -> bool:
    th = float(limits.xy_threshold_m)
    return abs(float(ex)) <= th and abs(float(ey)) <= th


def advance_allowed(ex: float, ey: float, limits: LookAlignLimits) -> bool:
    return look_align_ok(ex, ey, limits)


def should_send_look_command(
    ex: float,
    ey: float,
    limits: LookAlignLimits,
) -> bool:
    db = float(limits.xy_deadband_m)
    return abs(float(ex)) > db or abs(float(ey)) > db


def damped_pseudoinverse(J: np.ndarray, damping: float) -> np.ndarray:
    """J⁺ = Jᵀ (J Jᵀ + λ²I)⁻¹ for J shape (m, n), m <= n typical."""
    j = np.asarray(J, dtype=float)
    if j.ndim != 2:
        raise ValueError("J must be 2D")
    lam = float(max(damping, 1e-9))
    m = int(j.shape[0])
    jj_t = j @ j.T
    inv = np.linalg.inv(jj_t + (lam * lam) * np.eye(m, dtype=float))
    return j.T @ inv


def estimate_jacobian_column(
    e0: Sequence[float],
    e1: Sequence[float],
    eps: float,
) -> np.ndarray:
    e0_v = np.asarray(e0, dtype=float).reshape(2)
    e1_v = np.asarray(e1, dtype=float).reshape(2)
    denom = float(eps)
    if abs(denom) < 1e-12:
        return np.zeros(2, dtype=float)
    return (e1_v - e0_v) / denom


def stack_jacobian(columns: Sequence[np.ndarray]) -> np.ndarray:
    cols = [np.asarray(c, dtype=float).reshape(2) for c in columns]
    if not cols:
        raise ValueError("columns must be non-empty")
    return np.stack(cols, axis=1)


def jacobian_column_usable(column: np.ndarray, *, norm_min: float) -> bool:
    col = np.asarray(column, dtype=float).reshape(2)
    return float(np.linalg.norm(col)) >= float(norm_min)


def compute_look_delta_q(
    ex: float,
    ey: float,
    gains: LookGains,
    *,
    limits: LookAlignLimits,
    use_roll: bool = False,
) -> Q4Delta:
    """Map camera x/y error to roll/theta steps (no linear motion)."""
    if not should_send_look_command(ex, ey, limits):
        return Q4Delta()
    max_step = float(max(gains.max_step_rad, 1e-6))
    max_roll = float(max(gains.max_step_roll_rad, 1e-6))
    d_roll = 0.0
    d_theta1 = 0.0
    if use_roll:
        d_roll = float(np.clip(-float(gains.roll_per_error_x) * float(ex), -max_roll, max_roll))
    else:
        d_theta1 = float(np.clip(-float(gains.theta1_per_error_x) * float(ex), -max_step, max_step))
    d_theta2 = float(np.clip(-float(gains.theta2_per_error_y) * float(ey), -max_step, max_step))
    return Q4Delta(roll_rad=d_roll, theta1_rad=d_theta1, theta2_rad=d_theta2)


def compute_jacobian_look_delta_q(
    e: Sequence[float],
    J: np.ndarray,
    gains: JacobianLookGains,
    *,
    limits: LookAlignLimits,
    include_roll: bool = True,
) -> Q4Delta:
    """Δq = -gain * J⁺ e; only roll/theta1/theta2 (linear fixed)."""
    e_v = np.asarray(e, dtype=float).reshape(2)
    if not should_send_look_command(float(e_v[0]), float(e_v[1]), limits):
        return Q4Delta()
    j = np.asarray(J, dtype=float)
    j_pinv = damped_pseudoinverse(j, float(gains.damping))
    dq = -float(gains.gain) * (j_pinv @ e_v)
    max_roll = float(max(gains.max_step_roll_rad, 1e-6))
    max_theta = float(max(gains.max_step_theta_rad, 1e-6))
    if include_roll:
        if j.shape != (2, 3) or dq.shape[0] != 3:
            raise ValueError(f"J must be shape (2, 3) with roll, got {j.shape}")
        return Q4Delta(
            roll_rad=float(np.clip(dq[0], -max_roll, max_roll)),
            theta1_rad=float(np.clip(dq[1], -max_theta, max_theta)),
            theta2_rad=float(np.clip(dq[2], -max_theta, max_theta)),
        )
    if j.shape != (2, 2) or dq.shape[0] != 2:
        raise ValueError(f"J must be shape (2, 2) without roll, got {j.shape}")
    return Q4Delta(
        theta1_rad=float(np.clip(dq[0], -max_theta, max_theta)),
        theta2_rad=float(np.clip(dq[1], -max_theta, max_theta)),
    )


def apply_q_delta(
    q_linear: float,
    q_roll: float,
    q_theta1: float,
    q_theta2: float,
    delta: Q4Delta,
) -> tuple[float, float, float, float]:
    return (
        float(q_linear + delta.linear_m),
        float(q_roll + delta.roll_rad),
        float(q_theta1 + delta.theta1_rad),
        float(q_theta2 + delta.theta2_rad),
    )


def q4_tuple_to_delta(
    q_from: Sequence[float],
    q_to: Sequence[float],
) -> Q4Delta:
    a = np.asarray(q_from, dtype=float).reshape(4)
    b = np.asarray(q_to, dtype=float).reshape(4)
    return Q4Delta(
        linear_m=float(b[0] - a[0]),
        roll_rad=float(b[1] - a[1]),
        theta1_rad=float(b[2] - a[2]),
        theta2_rad=float(b[3] - a[3]),
    )


def apply_q_delta_to_tuple(
    q: Sequence[float],
    delta: Q4Delta,
) -> tuple[float, float, float, float]:
    return apply_q_delta(
        float(q[0]),
        float(q[1]),
        float(q[2]),
        float(q[3]),
        delta,
    )


def compute_advance_delta_q(step_m: float) -> Q4Delta:
    return Q4Delta(linear_m=float(max(0.0, step_m)))


def compute_backoff_delta_q(backoff_m: float) -> Q4Delta:
    return Q4Delta(linear_m=-float(max(0.0, backoff_m)))
