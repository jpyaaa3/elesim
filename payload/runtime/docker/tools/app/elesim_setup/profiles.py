"""Human-facing installation profiles and role ownership metadata."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable


ROLE_ORDER = ("sim", "pilot", "ui", "robot")


@dataclass(frozen=True)
class Profile:
    name: str
    title: str
    roles: tuple[str, ...]
    description: str
    remote: bool


PROFILES: dict[str, Profile] = {
    "local-sim": Profile(
        name="local-sim",
        title="한 PC 시뮬레이션",
        roles=("sim", "pilot", "ui"),
        description="Genesis Sim, Pilot과 UI를 같은 컴퓨터에 설치합니다.",
        remote=False,
    ),
    "laptop": Profile(
        name="laptop",
        title="조작 노트북",
        roles=("pilot", "ui"),
        description="원격 Sim 또는 Robot을 조작하는 Pilot과 UI를 설치합니다.",
        remote=True,
    ),
    "compute": Profile(
        name="compute",
        title="시뮬레이션 서버",
        roles=("sim",),
        description="고성능 컴퓨터에 headless Genesis Sim를 설치합니다.",
        remote=True,
    ),
    "robot": Profile(
        name="robot",
        title="Robot Jetson",
        roles=("robot",),
        description="실제 모터, GO2와 RGBD 카메라를 소유하는 Robot endpoint만 설치합니다.",
        remote=True,
    ),
    "custom": Profile(
        name="custom",
        title="사용자 지정",
        roles=(),
        description="이 컴퓨터에 필요한 역할을 직접 선택합니다.",
        remote=True,
    ),
}


def normalize_roles(values: Iterable[str]) -> tuple[str, ...]:
    requested = {str(value).strip().lower() for value in values if str(value).strip()}
    unknown = sorted(requested - set(ROLE_ORDER))
    if unknown:
        raise ValueError(f"알 수 없는 역할: {', '.join(unknown)}")
    if not requested:
        raise ValueError("설치할 역할이 하나 이상 필요합니다")
    return tuple(role for role in ROLE_ORDER if role in requested)


def roles_for_profile(name: str, custom_roles: Iterable[str] = ()) -> tuple[str, ...]:
    try:
        profile = PROFILES[str(name)]
    except KeyError as exc:
        raise ValueError(f"알 수 없는 설치 프로필: {name!r}") from exc
    return normalize_roles(custom_roles) if profile.name == "custom" else profile.roles


__all__ = ["PROFILES", "ROLE_ORDER", "Profile", "normalize_roles", "roles_for_profile"]
