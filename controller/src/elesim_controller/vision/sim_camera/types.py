from __future__ import annotations

from elesim_controller.vision.rgbd.types import RgbdFrame, RgbdIntrinsics


SimCameraIntrinsics = RgbdIntrinsics


class SimCameraFrame(RgbdFrame):
    def to_meta_dict(self) -> dict[str, object]:
        result = super().to_meta_dict()
        result["t"] = "sim_camera_frame"
        return result
