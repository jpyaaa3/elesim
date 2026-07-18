from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class PreviewGazeState:
    u_err: float
    v_err: float
    d_hat: np.ndarray

    def as_vector(self) -> np.ndarray:
        return np.array([self.u_err, self.v_err, *np.asarray(self.d_hat, dtype=float).reshape(-1)], dtype=float)


@dataclass(frozen=True)
class PreviewGazeModel:
    jacobian_uv: np.ndarray
    b_base: np.ndarray

    def step(
        self,
        state: PreviewGazeState,
        *,
        du: np.ndarray,
        d_disturbance: np.ndarray,
    ) -> PreviewGazeState:
        j = np.asarray(self.jacobian_uv, dtype=float).reshape(2, 3)
        b = np.asarray(self.b_base, dtype=float).reshape(2, -1)
        du_v = np.asarray(du, dtype=float).reshape(3)
        d_v = np.asarray(d_disturbance, dtype=float).reshape(-1)
        if b.shape[1] != d_v.size:
            b = b[:, : d_v.size] if b.shape[1] > d_v.size else np.pad(b, ((0, 0), (0, d_v.size - b.shape[1])))
        uv = np.array([state.u_err, state.v_err], dtype=float) + j @ du_v + b @ d_v
        d_hat = np.asarray(state.d_hat, dtype=float).reshape(-1)
        return PreviewGazeState(
            u_err=float(uv[0]),
            v_err=float(uv[1]),
            d_hat=d_hat,
        )
