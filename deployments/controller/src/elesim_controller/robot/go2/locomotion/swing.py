from __future__ import annotations

import numpy as np


class SwingTrajectory:
    """Parabolic swing foot path between touchdown targets."""

    @staticmethod
    def sample(
        progress: float,
        p_start: np.ndarray,
        p_end: np.ndarray,
        swing_height_m: float,
    ) -> np.ndarray:
        s = float(np.clip(progress, 0.0, 1.0))
        p_start = np.asarray(p_start, dtype=float).reshape(3)
        p_end = np.asarray(p_end, dtype=float).reshape(3)
        p = (1.0 - s) * p_start + s * p_end
        p[2] += float(swing_height_m) * 4.0 * s * (1.0 - s)
        return p
