"""Perception output: object position in camera optical frame."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass
class CameraObservation:
    """Object in RealSense color optical frame (x right, y down, z look)."""

    label: str
    confidence: float
    p_camera_object: np.ndarray
    timestamp: float
