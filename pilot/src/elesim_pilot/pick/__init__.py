from __future__ import annotations

__all__ = [
    "DEFAULT_SAG_MODEL_PATH",
    "ControlClient",
    "ControlService",
    "HostState",
    "PanelState",
    "VisualObservation",
    "extract_visual_observation",
    "load_sag_model_or_empty",
    "resolve_initial_sag_model",
    "resolve_sag_model_path",
]


def __getattr__(name: str):
    if name == "ControlClient":
        from .client import ControlClient

        return ControlClient
    if name in {"HostState", "PanelState"}:
        from . import state

        return getattr(state, name)
    if name in {"VisualObservation", "extract_visual_observation"}:
        from elesim_pilot.vision.perception import observation

        return getattr(observation, name)
    if name in {
        "DEFAULT_SAG_MODEL_PATH",
        "ControlService",
        "load_sag_model_or_empty",
        "resolve_initial_sag_model",
        "resolve_sag_model_path",
    }:
        from . import actions

        return getattr(actions, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
