from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional

import numpy as np

from elesim_simulator.pick.state import HostState


@dataclass
class CameraObservation:
    label: str
    confidence: float
    p_camera_object: np.ndarray
    timestamp: float


@dataclass(frozen=True)
class VisualObservation:
    label: str
    confidence: float
    center_uv: tuple[float, float]
    scale: float
    timestamp_s: float

    @property
    def age_s(self) -> float:
        return max(0.0, float(time.time() - float(self.timestamp_s)))


def extract_visual_observation(
    host_state: Optional[HostState],
    *,
    target_label: str = "",
    stale_timeout_s: float = 0.75,
    min_confidence: float = 0.0,
    min_scale: float = 0.0,
    max_center_abs: float = 1.05,
) -> Optional[VisualObservation]:
    if host_state is None:
        return None
    center_uv = host_state.perceived_center_uv
    scale = host_state.perceived_scale
    if center_uv is None or scale is None:
        return None
    if float(scale) < float(min_scale):
        return None
    if abs(float(center_uv[0])) > float(max_center_abs) or abs(float(center_uv[1])) > float(max_center_abs):
        return None
    if float(host_state.perceived_timestamp_s) <= 0.0:
        return None
    if (time.time() - float(host_state.perceived_timestamp_s)) > float(max(stale_timeout_s, 0.0)):
        return None
    if float(host_state.perceived_object_confidence) < float(min_confidence):
        return None
    target_key = str(target_label).strip().lower()
    obs_label = str(host_state.perceived_object_label).strip()
    if target_key and obs_label.lower() != target_key:
        return None
    return VisualObservation(
        label=obs_label,
        confidence=float(host_state.perceived_object_confidence),
        center_uv=(float(center_uv[0]), float(center_uv[1])),
        scale=float(scale),
        timestamp_s=float(host_state.perceived_timestamp_s),
    )


def extract_local_perception_observation(
    *,
    running: bool,
    center_uv: Optional[tuple[float, float]],
    image_scale: float,
    last_update_s: float,
    label: str,
    confidence: float,
    target_label: str = "",
    stale_timeout_s: float = 0.75,
    min_confidence: float = 0.0,
) -> Optional[VisualObservation]:
    """Fallback when host relay timestamp lags behind local perception worker."""
    if not bool(running):
        return None
    if center_uv is None:
        return None
    if float(last_update_s) <= 0.0:
        return None
    if (time.time() - float(last_update_s)) > float(max(stale_timeout_s, 0.0)):
        return None
    if float(confidence) < float(min_confidence):
        return None
    target_key = str(target_label).strip().lower()
    obs_label = str(label).strip()
    if target_key and obs_label.lower() != target_key:
        return None
    return VisualObservation(
        label=obs_label,
        confidence=float(confidence),
        center_uv=(float(center_uv[0]), float(center_uv[1])),
        scale=float(max(float(image_scale), 0.0)),
        timestamp_s=float(last_update_s),
    )
