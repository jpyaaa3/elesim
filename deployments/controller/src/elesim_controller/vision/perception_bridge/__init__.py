from __future__ import annotations

__all__ = [
    "camera_axes_world",
    "camera_point_to_world",
    "camera_world_transform",
    "load_hand_eye_transform",
    "world_point_to_camera",
]


def __getattr__(name: str):
    if name in set(__all__):
        from . import hand_eye

        return getattr(hand_eye, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
