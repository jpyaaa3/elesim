from __future__ import annotations

__all__ = [
    "ControlClient",
    "ControlService",
    "HostState",
    "PanelState",
    "VisualObservation",
    "extract_visual_observation",
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
        "ControlService",
    }:
        from . import actions

        return getattr(actions, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
