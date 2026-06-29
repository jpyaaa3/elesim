from __future__ import annotations

import numpy as np


def to_numpy_1d(raw) -> np.ndarray:
    if hasattr(raw, "detach"):
        raw = raw.detach()
    if hasattr(raw, "cpu"):
        raw = raw.cpu()
    if hasattr(raw, "numpy"):
        raw = raw.numpy()
    return np.asarray(raw, dtype=float).reshape(-1)


def quat_wxyz_to_xyzw(quat_wxyz) -> np.ndarray:
    q = np.asarray(quat_wxyz, dtype=float).reshape(4)
    return np.array([float(q[1]), float(q[2]), float(q[3]), float(q[0])], dtype=float)
