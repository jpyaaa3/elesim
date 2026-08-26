"""Pure, validated helpers for Go2 MPC contact constraints and cadence."""

from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


def _finite_positive(value: float, name: str, *, allow_zero: bool = False) -> float:
    try:
        result = float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{name} must be a finite number") from exc
    if not math.isfinite(result) or (result < 0.0 if allow_zero else result <= 0.0):
        bound = "non-negative" if allow_zero else "positive"
        raise ValueError(f"{name} must be finite and {bound}")
    return result


def optimization_mu(physical_mu: float) -> float:
    """Return the conservative coefficient for a four-face friction pyramid."""

    return _finite_positive(physical_mu, "physical_mu", allow_zero=True) / math.sqrt(2.0)


def project_grf_to_coulomb_cone(
    grf: np.ndarray,
    *,
    physical_mu: float,
    fz_max: float,
) -> np.ndarray:
    """Bound one GRF by ``0 <= fz <= fz_max`` and a circular friction cone.

    The normal component is clamped first; the tangential component is then
    radially clipped to ``physical_mu * fz``.  This deterministic projection
    preserves the requested normal force whenever it is feasible and returns
    a new array.
    """

    mu = _finite_positive(physical_mu, "physical_mu", allow_zero=True)
    z_limit = _finite_positive(fz_max, "fz_max", allow_zero=True)
    try:
        force = np.asarray(grf, dtype=float)
    except (TypeError, ValueError) as exc:
        raise ValueError("grf must be a finite three-vector") from exc
    if force.shape != (3,) or not np.all(np.isfinite(force)):
        raise ValueError("grf must be a finite three-vector")

    result = force.copy()
    result[2] = np.clip(result[2], 0.0, z_limit)
    tangent_limit = mu * result[2]
    tangent_norm = float(np.linalg.norm(result[:2]))
    if tangent_norm > tangent_limit:
        result[:2] *= tangent_limit / tangent_norm
    return result


def project_grf(grf: np.ndarray, *, physical_mu: float, fz_max: float) -> np.ndarray:
    """Short alias for :func:`project_grf_to_coulomb_cone`."""

    return project_grf_to_coulomb_cone(grf, physical_mu=physical_mu, fz_max=fz_max)


def normal_force_variable_indices(horizon: int) -> np.ndarray:
    """Return flattened QP indices for every leg's normal force."""

    steps = int(horizon)
    if steps < 1:
        raise ValueError(f"horizon must be >= 1, got {horizon!r}")
    state_variables = 12 * steps
    return np.asarray(
        [
            state_variables + 12 * step + 3 * leg + 2
            for step in range(steps)
            for leg in range(4)
        ],
        dtype=int,
    )


@dataclass(frozen=True, slots=True)
class MpcSchedule:
    """Immutable MPC cadence derived from the effective control frequency."""

    effective_control_hz: float
    nominal_mpc_dt_s: float
    solve_stride: int
    mpc_dt_s: float

    @classmethod
    def from_cadence(
        cls,
        effective_control_hz: float,
        nominal_mpc_dt_s: float,
    ) -> "MpcSchedule":
        control_hz = _finite_positive(effective_control_hz, "effective_control_hz")
        nominal_dt = _finite_positive(nominal_mpc_dt_s, "nominal_mpc_dt_s")
        stride = max(1, int(round(control_hz * nominal_dt)))
        return cls(
            effective_control_hz=control_hz,
            nominal_mpc_dt_s=nominal_dt,
            solve_stride=stride,
            mpc_dt_s=float(stride / control_hz),
        )

__all__ = [
    "MpcSchedule",
    "normal_force_variable_indices",
    "optimization_mu",
    "project_grf",
    "project_grf_to_coulomb_cone",
]
