from __future__ import annotations

__all__ = ["ControlPanel"]


def __getattr__(name: str):
    if name != "ControlPanel":
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from .control_panel import ControlPanel

    return ControlPanel
