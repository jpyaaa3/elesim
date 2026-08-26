from __future__ import annotations

_EXPORTS = {
    "Node9EyeInHandCamera": "mount",
    "ObserverCamera": "mount",
    "ObserverViewState": "mount",
    "SimCameraFrame": "types",
    "SimCameraIntrinsics": "types",
    "SimCameraPublisher": "publisher",
    "SimCameraSubscriber": "subscriber",
    "hand_eye_to_genesis_attach_T": "mount",
    "intrinsics_from_fov": "mount",
    "load_hand_eye_offset_T": "mount",
    "CameraRenderSpec": "async_worker",
    "CameraStateSnapshot": "async_worker",
    "CameraRenderWorker": "async_worker",
    "SharedRgbdMailbox": "async_worker",
}

__all__ = sorted(_EXPORTS)


def __getattr__(name: str):
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    import importlib

    module = importlib.import_module(f"{__name__}.{module_name}")
    return getattr(module, name)
