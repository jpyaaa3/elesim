"""Optional developer tooling attached to the canonical runtime installation.

This module deliberately owns no installer, Compose project, state file,
manager, or uninstall boundary. It only prepares the development image and
returns one profile-scoped service for ``ContainerInstaller``.
"""

from __future__ import annotations

import os
import re
import shutil
from pathlib import Path
from typing import Mapping

from .ownership import DOCKER_BUILD_FINGERPRINT_LABEL, DOCKER_INSTALL_UUID_LABEL
from .state import InstallState


_REQUIRED_PROJECTS = (
    ("payload/runtime/common/protocol", "pyproject.toml"),
    ("payload/runtime/common/elesim_interfaces", "package.xml"),
    ("payload/runtime/common/elesim_interfaces", "CMakeLists.txt"),
    ("payload/runtime/docker/pilot/app", "pyproject.toml"),
    ("payload/runtime/docker/ui/app", "pyproject.toml"),
    ("payload/runtime/docker/sim/app", "pyproject.toml"),
    ("payload/runtime/native/robot/app", "pyproject.toml"),
)
_DEVELOPER_USERNAME = re.compile(r"^[a-z_][a-z0-9_-]{0,31}$")


def resolve_developer_username() -> str:
    """Resolve a deterministic account name without requiring host passwd."""

    for variable in ("ELESIM_HOST_USER", "USER", "LOGNAME"):
        candidate = os.environ.get(variable, "").strip()
        if _DEVELOPER_USERNAME.fullmatch(candidate):
            return candidate
    return "dev"


def validate_developer_workspace(workspace: Path) -> None:
    """Require an existing complete checkout; attachments never clone source."""

    if workspace.is_symlink() or not workspace.is_dir():
        raise ValueError(f"developer workspace는 일반 directory여야 합니다: {workspace}")
    if not (workspace / ".git").is_dir() or not all(
        (workspace / project / marker).is_file()
        for project, marker in _REQUIRED_PROJECTS
    ):
        raise ValueError(
            "developer attachment는 완전한 EleSim Git checkout을 필요로 합니다: "
            f"{workspace}"
        )


def write_developer_context(*, source_root: Path, context: Path) -> None:
    """Copy prepacked developer image inputs into an owned build context."""

    source = source_root / "payload/runtime/docker/development"
    names = ("Dockerfile", "requirements.lock", "entrypoint.sh", "dev-env.sh")
    required = tuple(source / name for name in names) + (
        source_root / "payload/runtime/docker/shared/robotpkg.asc",
    )
    missing = tuple(path for path in required if not path.is_file())
    if missing:
        rendered = "\n".join(f"  - {path}" for path in missing)
        raise FileNotFoundError(f"개발 이미지 입력이 부족합니다:\n{rendered}")
    if os.path.lexists(context):
        if context.is_symlink() or not context.is_dir():
            raise ValueError(f"Developer image context는 directory여야 합니다: {context}")
        shutil.rmtree(context)
    context.mkdir(mode=0o700, parents=True)
    for name in names:
        shutil.copy2(source / name, context / name)
    shutil.copy2(
        source_root / "payload/runtime/docker/shared/robotpkg.asc",
        context / "robotpkg.asc",
    )


def developer_service(
    *,
    state: InstallState,
    context: Path,
    data_root: Path,
    install_uuid: str,
    build_fingerprint: str,
) -> dict[str, object]:
    """Return the optional dev shell service for the one runtime project."""

    workspace = state.developer_attachment.workspace_path
    if workspace is None:
        raise ValueError("developer service에는 활성 attachment가 필요합니다")
    validate_developer_workspace(workspace)
    username = resolve_developer_username()
    home = data_root / "home"
    cache = data_root / "cache"
    home.mkdir(mode=0o700, parents=True, exist_ok=True)
    cache.mkdir(mode=0o700, parents=True, exist_ok=True)
    labels: Mapping[str, str] = {
        DOCKER_INSTALL_UUID_LABEL: install_uuid,
        DOCKER_BUILD_FINGERPRINT_LABEL: build_fingerprint,
    }
    environment: dict[str, object] = {
        "HOME": str(home),
        "USER": username,
        "LOGNAME": username,
        "ELESIM_HOST_USER": username,
        "ELESIM_WORKSPACE": str(workspace),
        "DISPLAY": "${DISPLAY:-:0}",
        "WAYLAND_DISPLAY": "${WAYLAND_DISPLAY:-}",
        "XDG_RUNTIME_DIR": "${XDG_RUNTIME_DIR:-}",
        "PYTHONUNBUFFERED": "1",
        "PYTHONNOUSERSITE": "1",
    }
    service: dict[str, object] = {
        "image": "elesim/dev:local",
        "container_name": "elesim-dev",
        "build": {
            "context": str(context),
            "labels": dict(labels),
            "args": {
                "USERNAME": username,
                "UID": str(os.getuid()),
                "GID": str(os.getgid()),
                "COMPUTE_MODE": state.compute.gpu_mode,
            },
        },
        "profiles": ("developer",),
        "privileged": True,
        "labels": {DOCKER_INSTALL_UUID_LABEL: install_uuid},
        "network_mode": "host",
        "ipc": "host",
        "shm_size": "4gb",
        "working_dir": str(workspace),
        "environment": environment,
        "volumes": [
            f"{workspace}:{workspace}:rw",
            f"{home}:{home}:rw",
            f"{cache}:{cache}:rw",
            "/dev:/dev",
            "/tmp/.X11-unix:/tmp/.X11-unix:rw",
        ],
        "command": ("sleep", "infinity"),
        "restart": "unless-stopped",
    }
    if state.compute.gpu_mode == "cpu":
        environment["CUDA_VISIBLE_DEVICES"] = ""
    elif state.compute.gpu_mode == "specific":
        environment["NVIDIA_DRIVER_CAPABILITIES"] = "compute,utility,graphics"
        service["deploy"] = {
            "resources": {
                "reservations": {
                    "devices": (
                        {
                            "driver": "nvidia",
                            "device_ids": (state.compute.gpu_device,),
                            "capabilities": ("gpu",),
                        },
                    )
                }
            }
        }
    else:
        environment["NVIDIA_DRIVER_CAPABILITIES"] = "compute,utility,graphics"
        environment["CUDA_VISIBLE_DEVICES"] = None
        service["gpus"] = "all"
    if state.developer_attachment.wslg:
        service["volumes"].append("/mnt/wslg:/mnt/wslg:rw")  # type: ignore[union-attr]
        environment["WAYLAND_DISPLAY"] = "${WAYLAND_DISPLAY:-wayland-0}"
        environment["XDG_RUNTIME_DIR"] = "${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
        environment["PULSE_SERVER"] = "${PULSE_SERVER:-unix:/mnt/wslg/PulseServer}"
    return service


__all__ = [
    "developer_service",
    "resolve_developer_username",
    "validate_developer_workspace",
    "write_developer_context",
]
