from __future__ import annotations

_EXPORTS = {
    "SimCameraFrame": "types",
    "SimCameraIntrinsics": "types",
    "SimCameraSubscriber": "subscriber",
    "SimCameraVideoRecorder": "recording",
    "capture_sim_camera_snapshot": "recording",
    "save_sim_camera_snapshot": "recording",
}

__all__ = sorted(_EXPORTS)


def __getattr__(name: str):
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    import importlib

    module = importlib.import_module(f"{__name__}.{module_name}")
    return getattr(module, name)
