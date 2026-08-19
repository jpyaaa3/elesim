from __future__ import annotations

__all__ = [
    "ObjectPickPhase",
    "PickConvergence",
    "evaluate_pick_convergence",
    "grid_cell_center_uv",
    "pick_ready_for_extend",
    "pick_uv_deltas",
    "quadrant_fill_target_scale",
]


def __getattr__(name: str):
    if name in set(__all__):
        from . import core

        return getattr(core, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
