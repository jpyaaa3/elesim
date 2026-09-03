"""Application configuration schema and format-neutral loader.

This package is the only supported configuration import surface.  Concrete
loader implementation stays in :mod:`elesim_pilot.config.loader`.
"""

from elesim_pilot.config.loader import load_app_config
from elesim_pilot.config.distributed import RuntimeRoleConfig, load_runtime_role_config
from elesim_pilot.config.schema import (
    AppConfigBundle,
    ExperimentConfig,
    HardwareConfig,
    IkConfig,
    PerceptionConfig,
    PickConfig,
    SimConfig,
    SimParam,
    SpawnConfig,
    UrdfExportConfig,
)
from elesim_pilot.robot.arm.joint_defs import JointLimit

__all__ = [
    "AppConfigBundle",
    "ExperimentConfig",
    "HardwareConfig",
    "IkConfig",
    "JointLimit",
    "PerceptionConfig",
    "PickConfig",
    "SimConfig",
    "SimParam",
    "SpawnConfig",
    "UrdfExportConfig",
    "load_app_config",
    "RuntimeRoleConfig",
    "load_runtime_role_config",
]
