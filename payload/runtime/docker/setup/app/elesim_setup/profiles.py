"""Legacy CLI role presets and canonical role ordering."""

from __future__ import annotations

from typing import Iterable


ROLE_ORDER = ("sim", "pilot", "ui", "robot")


PROFILES: dict[str, tuple[str, ...]] = {
    "local-sim": ("sim", "pilot", "ui"),
    "laptop": ("pilot", "ui"),
    "compute": ("sim",),
    "robot": ("robot",),
    "custom": (),
}


def normalize_roles(values: Iterable[str]) -> tuple[str, ...]:
    requested = {str(value).strip().lower() for value in values if str(value).strip()}
    unknown = sorted(requested - set(ROLE_ORDER))
    if unknown:
        raise ValueError(f"Unknown role: {', '.join(unknown)}")
    if not requested:
        raise ValueError("At least one installation role is required")
    return tuple(role for role in ROLE_ORDER if role in requested)


def roles_for_profile(name: str, custom_roles: Iterable[str] = ()) -> tuple[str, ...]:
    try:
        roles = PROFILES[str(name)]
    except KeyError as exc:
        raise ValueError(f"Unknown installation profile: {name!r}") from exc
    return normalize_roles(custom_roles) if str(name) == "custom" else roles


__all__ = ["PROFILES", "ROLE_ORDER", "normalize_roles", "roles_for_profile"]
