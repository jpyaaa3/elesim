"""Elesim installation and network setup tools."""

from .profiles import PROFILES, ROLE_ORDER, Profile, roles_for_profile
from .state import (
    ComputeSettings,
    DdsSettings,
    InstallState,
    NetworkSettings,
    TurnSettings,
)

__version__ = "0.3.0"

__all__ = [
    "ComputeSettings",
    "DdsSettings",
    "InstallState",
    "NetworkSettings",
    "PROFILES",
    "Profile",
    "ROLE_ORDER",
    "TurnSettings",
    "roles_for_profile",
]
