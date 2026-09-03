from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from elesim_pilot.gaze.preview_model import PreviewGazeModel, PreviewGazeState


@dataclass(frozen=True)
class PreviewMpcWeights:
    Q: np.ndarray
    R: np.ndarray
    S: np.ndarray


@dataclass(frozen=True)
class PreviewSolveResult:
    du: np.ndarray
    preview_term_v: float
    solve_time_ms: float
    ok: bool
    reason: str


def solve_preview_du(
    *,
    jacobian: np.ndarray,
    s: np.ndarray,
    preview_term: np.ndarray,
    q_u: float,
    q_v: float,
    r_roll: float,
    r_s1: float,
    r_s2: float,
) -> PreviewSolveResult:
    """One-step preview-regularized least squares: du = -(J'QJ+R)^-1 J'Q (s + preview_term)."""
    t0 = time.perf_counter()
    try:
        j = np.asarray(jacobian, dtype=float).reshape(2, 3)
        err = np.asarray(s, dtype=float).reshape(2) + np.asarray(preview_term, dtype=float).reshape(2)
        q_u_v = float(max(q_u, 1e-9))
        q_v_v = float(max(q_v, 1e-9))
        q = np.diag([q_u_v, q_v_v])
        r = np.diag(
            [
                float(max(r_roll, 1e-9)),
                float(max(r_s1, 1e-9)),
                float(max(r_s2, 1e-9)),
            ]
        )
        h = j.T @ q @ j + r
        g = j.T @ q @ err
        du = -np.linalg.solve(h, g)
        if not np.all(np.isfinite(du)):
            raise FloatingPointError("non-finite du")
        preview_term_v = float(preview_term.reshape(2)[1]) if preview_term.size >= 2 else 0.0
        ms = (time.perf_counter() - t0) * 1000.0
        return PreviewSolveResult(
            du=np.asarray(du, dtype=float).reshape(3),
            preview_term_v=preview_term_v,
            solve_time_ms=float(ms),
            ok=True,
            reason="",
        )
    except (np.linalg.LinAlgError, FloatingPointError, ValueError) as exc:
        ms = (time.perf_counter() - t0) * 1000.0
        return PreviewSolveResult(
            du=np.zeros(3, dtype=float),
            preview_term_v=0.0,
            solve_time_ms=float(ms),
            ok=False,
            reason=f"solve_fail:{exc.__class__.__name__}",
        )


class PreviewMpcController:
    """Preview MPC-lite: single-step solve only (no horizon)."""

    def __init__(self, model: PreviewGazeModel, weights: PreviewMpcWeights) -> None:
        self.model = model
        self.weights = weights

    def solve(
        self,
        state: PreviewGazeState,
        *,
        disturbance_horizon: np.ndarray,
    ) -> np.ndarray:
        d_hat = np.asarray(disturbance_horizon, dtype=float).reshape(-1)
        if d_hat.size < 1:
            d_hat = np.zeros(1, dtype=float)
        preview_term = self.model.b_base @ d_hat.reshape(-1, 1)
        preview_term = np.asarray(preview_term, dtype=float).reshape(2)
        s = np.array([state.u_err, state.v_err], dtype=float)
        q = np.diag(np.diag(self.weights.Q))
        r = np.diag(np.diag(self.weights.R))
        result = solve_preview_du(
            jacobian=self.model.jacobian_uv,
            s=s,
            preview_term=preview_term,
            q_u=float(q[0, 0]),
            q_v=float(q[1, 1]),
            r_roll=float(r[0, 0]),
            r_s1=float(r[1, 1]),
            r_s2=float(r[2, 2]),
        )
        if not result.ok:
            raise RuntimeError(result.reason)
        return result.du
