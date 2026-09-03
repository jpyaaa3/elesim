"""Explicit physical-camera profiles used by Pilot perception."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


class CameraProfileError(ValueError):
    """The selected camera profile is unknown or internally inconsistent."""


@dataclass(frozen=True)
class CameraProfile:
    name: str
    driver: str
    calibration_filename: str
    model_bundle: str


CAMERA_PROFILES = {
    "zed_mini": CameraProfile(
        name="zed_mini",
        driver="zed",
        calibration_filename="zed_mini.hand_eye.json",
        model_bundle="zed-mini",
    ),
    "d435": CameraProfile(
        name="d435",
        driver="realsense",
        calibration_filename="d435.hand_eye.json",
        model_bundle="d435",
    ),
}


def camera_profile(raw: str) -> CameraProfile:
    name = str(raw).strip().lower()
    try:
        return CAMERA_PROFILES[name]
    except KeyError as exc:
        allowed = ", ".join(sorted(CAMERA_PROFILES))
        raise CameraProfileError(
            f"unknown camera profile {raw!r}; allowed values: {allowed}"
        ) from exc


def validate_calibration_path(profile: str, path: str | Path) -> Path:
    selected = camera_profile(profile)
    resolved = Path(path)
    # The legacy generic name is accepted as an explicit custom path.  Named
    # profile files must never be paired with the other camera profile.
    if (
        resolved.name in {item.calibration_filename for item in CAMERA_PROFILES.values()}
        and resolved.name != selected.calibration_filename
    ):
        raise CameraProfileError(
            f"camera profile {selected.name!r} requires {selected.calibration_filename}, "
            f"got {resolved.name}"
        )
    return resolved


def validate_bundle_path(profile: str, path: str | Path) -> Path:
    selected = camera_profile(profile)
    resolved = Path(path)
    if (
        resolved.name in {item.model_bundle for item in CAMERA_PROFILES.values()}
        and resolved.name != selected.model_bundle
    ):
        raise CameraProfileError(
            f"camera profile {selected.name!r} requires model bundle "
            f"{selected.model_bundle!r}, got {resolved.name!r}"
        )
    return resolved


__all__ = [
    "CAMERA_PROFILES",
    "CameraProfile",
    "CameraProfileError",
    "camera_profile",
    "validate_bundle_path",
    "validate_calibration_path",
]
