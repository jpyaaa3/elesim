"""Ready-pose target geometry shared by UI actions and host markers."""

from __future__ import annotations

from typing import Sequence

import numpy as np


def compute_ready_pose_target(
    object_world: Sequence[float],
    target_dir_world: Sequence[float],
    *,
    standoff_m: float,
) -> tuple[float, float, float]:
    """Pre-grasp point behind the object relative to the desired approach axis."""
    obj = np.asarray(object_world, dtype=float).reshape(3)
    direction = np.asarray(target_dir_world, dtype=float).reshape(3)
    distance = float(standoff_m)
    if not np.all(np.isfinite(obj)):
        raise ValueError("object position must contain only finite values")
    if not np.all(np.isfinite(direction)):
        raise ValueError("target direction must contain only finite values")
    if not np.isfinite(distance):
        raise ValueError("standoff distance must be finite")
    norm = float(np.linalg.norm(direction))
    if norm <= 1e-9:
        raise ValueError("target direction must be nonzero")
    unit_dir = direction / norm
    target = obj - unit_dir * float(max(distance, 0.0))
    return (float(target[0]), float(target[1]), float(target[2]))
