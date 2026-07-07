"""Axis-frame decomposition and gating for sag drift inputs (no FK/scipy deps)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence

import numpy as np


@dataclass(frozen=True)
class SagDriftComponents:
    drift_world: tuple[float, float, float]
    axial_m: float
    lateral_m: float
    dir_error_deg: float
    sag_input_world: tuple[float, float, float]
    usable: bool
    reason: str


def _unit_vec3(vec: Sequence[float]) -> np.ndarray:
    v = np.asarray(vec, dtype=float).reshape(3)
    norm = float(np.linalg.norm(v))
    if norm <= 1e-9:
        raise ValueError("direction must be nonzero")
    return v / norm


def _direction_angle_deg(a: Sequence[float], b: Sequence[float]) -> float:
    a_u = _unit_vec3(a)
    b_u = _unit_vec3(b)
    dot = float(np.clip(float(np.dot(a_u, b_u)), -1.0, 1.0))
    return float(math.degrees(math.acos(dot)))


def prepare_sag_drift_input(
    *,
    drift_world: Sequence[float],
    axis_world: Sequence[float],
    reference_dir: Sequence[float],
    max_dir_error_deg: float,
    max_lateral_m: float,
    min_axial_m: float = 0.002,
    axial_only: bool = True,
) -> SagDriftComponents:
    """Decompose drift along FK axis; gate sag input when dir/lateral mismatch is large."""
    drift = np.asarray(drift_world, dtype=float).reshape(3)
    axis = _unit_vec3(axis_world)
    drift_tuple = (float(drift[0]), float(drift[1]), float(drift[2]))
    axial_scalar = float(np.dot(drift, axis))
    axial_vec = axis * axial_scalar
    lateral_vec = drift - axial_vec
    lateral_m = float(np.linalg.norm(lateral_vec))
    dir_error_deg = _direction_angle_deg(axis, reference_dir)
    sag_vec = axial_vec if bool(axial_only) else drift
    sag_tuple = (float(sag_vec[0]), float(sag_vec[1]), float(sag_vec[2]))

    if float(dir_error_deg) > float(max(max_dir_error_deg, 0.0)):
        return SagDriftComponents(
            drift_world=drift_tuple,
            axial_m=axial_scalar,
            lateral_m=lateral_m,
            dir_error_deg=dir_error_deg,
            sag_input_world=sag_tuple,
            usable=False,
            reason="dir_error_too_large",
        )
    # axial_only: lateral is discarded from sag input — do not reject Aim recenter drift.
    if (not bool(axial_only)) and lateral_m > float(max(max_lateral_m, 0.0)):
        return SagDriftComponents(
            drift_world=drift_tuple,
            axial_m=axial_scalar,
            lateral_m=lateral_m,
            dir_error_deg=dir_error_deg,
            sag_input_world=sag_tuple,
            usable=False,
            reason="lateral_drift_too_large",
        )
    magnitude = abs(axial_scalar) if bool(axial_only) else float(np.linalg.norm(drift))
    if magnitude < float(max(min_axial_m, 0.0)):
        return SagDriftComponents(
            drift_world=drift_tuple,
            axial_m=axial_scalar,
            lateral_m=lateral_m,
            dir_error_deg=dir_error_deg,
            sag_input_world=sag_tuple,
            usable=False,
            reason="drift_too_small",
        )
    return SagDriftComponents(
        drift_world=drift_tuple,
        axial_m=axial_scalar,
        lateral_m=lateral_m,
        dir_error_deg=dir_error_deg,
        sag_input_world=sag_tuple,
        usable=True,
        reason="accepted",
    )


__all__ = ["SagDriftComponents", "prepare_sag_drift_input"]
