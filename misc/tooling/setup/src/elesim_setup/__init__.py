"""Elesim installation and network setup tools."""

from .profiles import PROFILES, ROLE_ORDER, Profile, roles_for_profile
from .state import ComputeSettings, InstallState, NetworkSettings, SecuritySettings

__version__ = "0.2.0"

__all__ = [
    "ComputeSettings",
    "InstallState",
    "NetworkSettings",
    "PROFILES",
    "Profile",
    "ROLE_ORDER",
    "SecuritySettings",
    "roles_for_profile",
]
