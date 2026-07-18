from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

import numpy as np


@dataclass(frozen=True)
class RgbdIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    width: int
    height: int


@dataclass(frozen=True)
class RgbdFrame:
    color_bgr: np.ndarray
    depth_raw: np.ndarray
    depth_scale: float
    intrinsics: RgbdIntrinsics
    seq: int = 0
    ts: float = 0.0
    arm_q: Optional[tuple[float, float, float, float]] = None
    camera_world_origin: Optional[tuple[float, float, float]] = None
    camera_world_look: Optional[tuple[float, float, float]] = None
    camera_world_right: Optional[tuple[float, float, float]] = None

    def to_meta_dict(self) -> dict[str, object]:
        result: dict[str, object] = {
            "t": "rgbd_frame",
            "seq": int(self.seq),
            "ts": float(self.ts),
            "width": int(self.intrinsics.width),
            "height": int(self.intrinsics.height),
            "fx": float(self.intrinsics.fx),
            "fy": float(self.intrinsics.fy),
            "cx": float(self.intrinsics.cx),
            "cy": float(self.intrinsics.cy),
            "depth_scale": float(self.depth_scale),
            "arm_q": None if self.arm_q is None else [float(value) for value in self.arm_q],
        }
        for key in ("camera_world_origin", "camera_world_look", "camera_world_right"):
            value = getattr(self, key)
            if value is not None:
                result[key] = [float(component) for component in value]
        return result

