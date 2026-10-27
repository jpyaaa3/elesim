from __future__ import annotations

__all__ = [
    "PerceptionCapture",
    "PerceptionSnapshot",
    "TrackerPhase",
    "VisualObservation",
    "default_perception_capture_dir",
    "extract_local_perception_observation",
    "extract_visual_observation",
    "load_mock_world_xyz_from_detector_path",
    "save_perception_frame_bundle",
]


def __getattr__(name: str):
    if name in {
        "PerceptionCapture",
        "PerceptionSnapshot",
        "TrackerPhase",
        "default_perception_capture_dir",
        "load_mock_world_xyz_from_detector_path",
        "save_perception_frame_bundle",
    }:
        from . import capture

        return getattr(capture, name)
    if name in {
        "VisualObservation",
        "extract_local_perception_observation",
        "extract_visual_observation",
    }:
        from . import observation

        return getattr(observation, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
