"""Profile-to-driver factory for Pilot's physical RGB-D camera."""

from __future__ import annotations

from typing import Any

from .camera_profile import CameraProfileError, camera_profile


def camera_class(profile: str) -> type[Any]:
    selected = camera_profile(profile)
    if selected.driver == "zed":
        from .zed_camera import ZedMiniCamera

        return ZedMiniCamera
    if selected.driver == "realsense":
        from .realsense_camera import RealSenseCamera

        return RealSenseCamera
    raise CameraProfileError(f"no driver factory for camera profile {selected.name!r}")


def camera_factory(profile: str, **kwargs: Any) -> Any:
    return camera_class(profile)(**kwargs)


__all__ = ["camera_class", "camera_factory"]
