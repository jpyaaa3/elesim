"""Hand-eye calibration loading for simulator camera attachment."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
from scipy.spatial.transform import Rotation as Rot


def load_hand_eye_transform(path: str | Path) -> tuple[np.ndarray, dict[str, Any]]:
    source = Path(path)
    if not source.is_file():
        raise FileNotFoundError(f"hand-eye config not found: {source}")
    data = json.loads(source.read_text(encoding="utf-8"))
    optical = data.get("optical_frame")
    payload = optical if isinstance(optical, dict) else data
    translation = np.asarray(payload.get("translation_m", [0.0, 0.0, 0.0]), dtype=float).reshape(3)
    quaternion = np.asarray(
        payload.get("quaternion_xyzw", [0.0, 0.0, 0.0, 1.0]),
        dtype=float,
    ).reshape(4)
    transform = np.eye(4, dtype=float)
    transform[:3, :3] = Rot.from_quat(quaternion).as_matrix()
    transform[:3, 3] = translation
    return transform, {
        "parent_frame": str(payload.get("parent_frame", "node9")),
        "child_frame": str(payload.get("child_frame", "camera_color_optical_frame")),
        "path": str(source.resolve()),
    }
