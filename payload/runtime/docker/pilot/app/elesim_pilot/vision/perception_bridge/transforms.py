from __future__ import annotations

import numpy as np
from scipy.spatial.transform import Rotation as Rot


def make_transform_from_pose(
    translation_m: np.ndarray | list[float],
    quaternion_xyzw: np.ndarray | list[float],
) -> np.ndarray:
    t = np.asarray(translation_m, dtype=float).reshape(3)
    q = np.asarray(quaternion_xyzw, dtype=float).reshape(4)
    T = np.eye(4, dtype=float)
    T[:3, :3] = Rot.from_quat(q).as_matrix()
    T[:3, 3] = t
    return T


def make_transform_from_world_pose(position: np.ndarray, rotation_matrix: np.ndarray) -> np.ndarray:
    T = np.eye(4, dtype=float)
    T[:3, :3] = np.asarray(rotation_matrix, dtype=float).reshape(3, 3)
    T[:3, 3] = np.asarray(position, dtype=float).reshape(3)
    return T


def transform_point(T: np.ndarray, point: np.ndarray | list[float]) -> np.ndarray:
    p = np.asarray(point, dtype=float).reshape(3)
    hom = np.array([p[0], p[1], p[2], 1.0], dtype=float)
    out = np.asarray(T, dtype=float).reshape(4, 4) @ hom
    return out[:3]
