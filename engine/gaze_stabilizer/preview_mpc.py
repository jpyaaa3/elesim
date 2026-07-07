from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from engine.gaze_stabilizer.preview_model import PreviewGazeModel, PreviewGazeState


@dataclass(frozen=True)
class PreviewMpcWeights:
    Q: np.ndarray
    R: np.ndarray
    S: np.ndarray


class PreviewMpcController:
    """Preview MPC interface stub — not connected to runtime gaze worker by default."""

    def __init__(self, model: PreviewGazeModel, weights: PreviewMpcWeights) -> None:
        self.model = model
        self.weights = weights

    def solve(
        self,
        state: PreviewGazeState,
        *,
        disturbance_horizon: np.ndarray,
    ) -> np.ndarray:
        raise NotImplementedError("Preview MPC is not connected in v1; use uv or uv_ff gaze modes")
