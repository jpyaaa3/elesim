from __future__ import annotations

_EXPORTS = {
    "draw_control_4dof_panel": "control_4dof",
    "draw_go2_panel": "go2",
    "draw_hardware_panel": "hardware",
    "draw_ik_panel": "ik",
    "draw_perception_panel": "perception",
    "draw_planned_move_panel": "planned_move",
    "draw_resolution_panel": "live_visual_status",
    "draw_sag_panel": "sag",
    "draw_status_panel": "live_visual_status",
}

__all__ = [
    "draw_control_4dof_panel",
    "draw_go2_panel",
    "draw_hardware_panel",
    "draw_ik_panel",
    "draw_perception_panel",
    "draw_planned_move_panel",
    "draw_sag_panel",
    "draw_resolution_panel",
    "draw_status_panel",
]


def __getattr__(name: str):
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    import importlib

    module = importlib.import_module(f"{__name__}.{module_name}")
    return getattr(module, name)
