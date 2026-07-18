from __future__ import annotations

_EXPORTS = {
    "ALL_LEGS": "types",
    "FootTarget": "types",
    "GaitScheduler": "gait",
    "GO2_LEG_JOINTS": "kinematics",
    "GO2_STAND_Q": "kinematics",
    "Go2Command": "types",
    "Go2KinematicsModel": "kinematics",
    "Go2LocomotionConfig": "config",
    "HIP_OFFSET_BODY": "kinematics",
    "LegId": "types",
    "LegPhase": "types",
    "RaibertFootPlacement": "raibert",
    "RaibertTrotController": "controller",
    "SwingTrajectory": "swing",
    "TROT_PHASE_OFFSET": "kinematics",
}

__all__ = sorted(_EXPORTS)


def __getattr__(name: str):
    module_name = _EXPORTS.get(name)
    if module_name is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    import importlib

    module = importlib.import_module(f"{__name__}.{module_name}")
    return getattr(module, name)
