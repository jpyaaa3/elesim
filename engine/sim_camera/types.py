from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class SimCameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass(frozen=True)
class SimCameraFrame:
    color_bgr: np.ndarray
    depth_raw: np.ndarray
    depth_scale: float
    intrinsics: SimCameraIntrinsics
    seq: int = 0
    ts: float = 0.0
    arm_q: Optional[tuple[float, float, float, float]] = None
    camera_world_origin: Optional[tuple[float, float, float]] = None
    camera_world_look: Optional[tuple[float, float, float]] = None
    camera_world_right: Optional[tuple[float, float, float]] = None

    def to_meta_dict(self) -> dict:
        out = {
            "t": "sim_camera_frame",
            "seq": int(self.seq),
            "ts": float(self.ts),
            "width": int(self.intrinsics.width),
            "height": int(self.intrinsics.height),
            "fx": float(self.intrinsics.fx),
            "fy": float(self.intrinsics.fy),
            "cx": float(self.intrinsics.cx),
            "cy": float(self.intrinsics.cy),
            "depth_scale": float(self.depth_scale),
            "arm_q": None if self.arm_q is None else [float(x) for x in self.arm_q],
        }
        if self.camera_world_origin is not None:
            out["camera_world_origin"] = [float(x) for x in self.camera_world_origin]
        if self.camera_world_look is not None:
            out["camera_world_look"] = [float(x) for x in self.camera_world_look]
        if self.camera_world_right is not None:
            out["camera_world_right"] = [float(x) for x in self.camera_world_right]
        return out
