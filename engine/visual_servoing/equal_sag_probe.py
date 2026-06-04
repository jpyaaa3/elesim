"""Estimate coarse equal-angle sag offsets from visual-servo drift."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

from engine.iklib.kinematics import _forward_grasp_world


@dataclass(frozen=True)
class EqualSagEstimate:
    accepted: bool
    seg1_equal_offset_deg: float
    seg2_equal_offset_deg: float
    drift_world: tuple[float, float, float]
    reconstructed_drift_world: tuple[float, float, float]
    residual_m: float
    condition: float
    reason: str


def apply_equal_sag_offsets(
    sag_model: dict[str, Any] | None,
    *,
    seg1_equal_offset_deg: float,
    seg2_equal_offset_deg: float,
) -> dict[str, Any]:
    model = dict(sag_model or {})
    model["seg1_equal_offset_deg"] = float(seg1_equal_offset_deg)
    model["seg2_equal_offset_deg"] = float(seg2_equal_offset_deg)
    return model


def solve_equal_sag_offsets(
    *,
    drift_world: Sequence[float],
    sensitivity_m_per_deg: np.ndarray,
    max_abs_offset_deg: float = 12.0,
    min_drift_m: float = 0.002,
    max_residual_m: float = 0.10,
    condition_max: float = 1.0e4,
) -> EqualSagEstimate:
    drift = np.asarray(drift_world, dtype=float).reshape(3)
    drift_norm = float(np.linalg.norm(drift))
    zero = (0.0, 0.0, 0.0)
    if drift_norm < float(min_drift_m):
        return EqualSagEstimate(False, 0.0, 0.0, tuple(float(v) for v in drift), zero, drift_norm, float("inf"), "drift_too_small")

    j = np.asarray(sensitivity_m_per_deg, dtype=float)
    if j.shape != (3, 2):
        raise ValueError(f"sensitivity_m_per_deg must have shape (3, 2), got {j.shape}")
    try:
        singular = np.linalg.svd(j, compute_uv=False)
    except np.linalg.LinAlgError:
        return EqualSagEstimate(False, 0.0, 0.0, tuple(float(v) for v in drift), zero, drift_norm, float("inf"), "svd_failed")
    if singular.size < 2 or float(singular[-1]) <= 1e-9:
        return EqualSagEstimate(False, 0.0, 0.0, tuple(float(v) for v in drift), zero, drift_norm, float("inf"), "singular_sensitivity")
    condition = float(singular[0] / singular[-1])
    if condition > float(condition_max):
        return EqualSagEstimate(False, 0.0, 0.0, tuple(float(v) for v in drift), zero, drift_norm, condition, "ill_conditioned")

    offsets, *_ = np.linalg.lstsq(j, drift, rcond=None)
    seg1 = float(offsets[0])
    seg2 = float(offsets[1])
    reconstructed = j @ offsets
    residual = float(np.linalg.norm(drift - reconstructed))
    if max(abs(seg1), abs(seg2)) > float(max_abs_offset_deg):
        return EqualSagEstimate(
            False,
            seg1,
            seg2,
            tuple(float(v) for v in drift),
            tuple(float(v) for v in reconstructed),
            residual,
            condition,
            "offset_too_large",
        )
    if residual > float(max_residual_m):
        return EqualSagEstimate(
            False,
            seg1,
            seg2,
            tuple(float(v) for v in drift),
            tuple(float(v) for v in reconstructed),
            residual,
            condition,
            "residual_too_large",
        )
    return EqualSagEstimate(
        True,
        seg1,
        seg2,
        tuple(float(v) for v in drift),
        tuple(float(v) for v in reconstructed),
        residual,
        condition,
        "accepted",
    )


def _grasp_with_equal_sag(
    context: dict[str, Any],
    q4: Sequence[float],
    sag_model: dict[str, Any] | None,
    *,
    seg1_equal_offset_deg: float,
    seg2_equal_offset_deg: float,
) -> np.ndarray:
    ctx = dict(context)
    ctx["sag_model"] = apply_equal_sag_offsets(
        sag_model,
        seg1_equal_offset_deg=float(seg1_equal_offset_deg),
        seg2_equal_offset_deg=float(seg2_equal_offset_deg),
    )
    return np.asarray(_forward_grasp_world(ctx, q4), dtype=float).reshape(3)


def estimate_equal_sag_from_ready_pose_drift(
    *,
    context: dict[str, Any],
    q4: Sequence[float],
    ready_pose_drift_world: Sequence[float],
    sag_model: dict[str, Any] | None = None,
    fd_eps_deg: float = 0.25,
    max_abs_offset_deg: float = 24.0,
    min_drift_m: float = 0.002,
    max_residual_m: float = 0.040,
    condition_max: float = 1.0e4,
) -> EqualSagEstimate:
    eps = float(max(abs(float(fd_eps_deg)), 1e-4))
    j = np.zeros((3, 2), dtype=float)
    for col, (s1, s2) in enumerate(((eps, 0.0), (0.0, eps))):
        p_plus = _grasp_with_equal_sag(
            context,
            q4,
            sag_model,
            seg1_equal_offset_deg=s1,
            seg2_equal_offset_deg=s2,
        )
        p_minus = _grasp_with_equal_sag(
            context,
            q4,
            sag_model,
            seg1_equal_offset_deg=-s1,
            seg2_equal_offset_deg=-s2,
        )
        j[:, col] = (p_plus - p_minus) / (2.0 * eps)
    return solve_equal_sag_offsets(
        drift_world=ready_pose_drift_world,
        sensitivity_m_per_deg=j,
        max_abs_offset_deg=max_abs_offset_deg,
        min_drift_m=min_drift_m,
        max_residual_m=max_residual_m,
        condition_max=condition_max,
    )


__all__ = [
    "EqualSagEstimate",
    "apply_equal_sag_offsets",
    "estimate_equal_sag_from_ready_pose_drift",
    "solve_equal_sag_offsets",
]
