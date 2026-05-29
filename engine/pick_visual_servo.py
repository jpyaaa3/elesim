"""Camera-frame visual servo helpers for Look-then-Advance pick control."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class LookAlignLimits:
    xy_threshold_m: float = 0.010
    xy_deadband_m: float = 0.008


@dataclass(frozen=True)
class LookGains:
    theta1_per_error_x: float = 1.0
    theta2_per_error_y: float = 1.0
    max_step_rad: float = 0.02


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


def compute_look_delta_q(
    ex: float,
    ey: float,
    gains: LookGains,
    *,
    limits: LookAlignLimits,
) -> Q4Delta:
    """Map camera x/y error to theta1/theta2 steps only (no linear motion)."""
    if not should_send_look_command(ex, ey, limits):
        return Q4Delta()
    max_step = float(max(gains.max_step_rad, 1e-6))
    d_theta1 = float(np.clip(-float(gains.theta1_per_error_x) * float(ex), -max_step, max_step))
    d_theta2 = float(np.clip(-float(gains.theta2_per_error_y) * float(ey), -max_step, max_step))
    return Q4Delta(theta1_rad=d_theta1, theta2_rad=d_theta2)


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


def compute_advance_delta_q(step_m: float) -> Q4Delta:
    return Q4Delta(linear_m=float(max(0.0, step_m)))


def compute_backoff_delta_q(backoff_m: float) -> Q4Delta:
    return Q4Delta(linear_m=-float(max(0.0, backoff_m)))
