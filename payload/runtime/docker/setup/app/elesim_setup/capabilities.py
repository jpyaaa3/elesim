"""Read-only host capability detection for the setup wizard."""

from __future__ import annotations

import os
import platform
import re
import subprocess
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable, Mapping, Sequence


CommandOutput = Callable[[tuple[str, ...]], str]
_GPU_LINE = re.compile(
    r"^GPU\s+(?P<index>\d+):\s+(?P<name>.+?)\s+\(UUID:\s*(?P<uuid>[^)]+)\)\s*$"
)


@dataclass(frozen=True)
class GpuDevice:
    index: str
    name: str
    uuid: str


@dataclass(frozen=True)
class HostCapabilities:
    architecture: str
    os_id: str
    os_version: str
    jetson: bool
    robot_installable: bool
    developer_installable: bool
    display_available: bool
    ssh_agent: bool
    gpu_devices: tuple[GpuDevice, ...]
    wsl: bool = False
    wslg_available: bool = False
    docker_backend: str = ""
    docker_context: str = ""
    docker_engine_id: str = ""
    docker_endpoint: str = ""
    docker_host_override: str = ""

    def to_dict(self) -> dict[str, object]:
        raw = asdict(self)
        raw["gpu_devices"] = [asdict(device) for device in self.gpu_devices]
        return raw


def detect_host_capabilities(
    *,
    root: Path = Path("/"),
    environ: Mapping[str, str] | None = None,
    machine: str | None = None,
    command_output: CommandOutput | None = None,
) -> HostCapabilities:
    environment = os.environ if environ is None else environ
    architecture = (machine or platform.machine()).strip().lower()
    os_release = _key_values(root / "etc/os-release")
    model = _read_text(root / "proc/device-tree/model")
    kernel_release = _read_text(root / "proc/sys/kernel/osrelease").lower()
    jetson = (root / "etc/nv_tegra_release").is_file() or "jetson" in model.lower()
    wsl = bool(environment.get("WSL_DISTRO_NAME", "").strip()) or "microsoft" in kernel_release
    wslg_available = wsl and (
        (root / "mnt/wslg").is_dir()
        or bool(environment.get("WSL_INTEROP", "").strip())
        and bool(environment.get("WAYLAND_DISPLAY", "").strip())
    )
    output = _run_output if command_output is None else command_output
    gpu_devices = _parse_gpu_devices(_optional_output(output, ("nvidia-smi", "-L")))
    docker_name = _optional_output(
        output,
        ("docker", "info", "--format", "{{.Name}}"),
    ).strip()
    docker_context = _optional_output(
        output,
        ("docker", "context", "show"),
    ).strip()
    docker_engine_id = _optional_output(
        output,
        ("docker", "info", "--format", "{{.ID}}"),
    ).strip()
    docker_endpoint = (
        _optional_output(
            output,
            (
                "docker",
                "context",
                "inspect",
                docker_context,
                "--format",
                '{{(index .Endpoints "docker").Host}}',
            ),
        ).strip()
        if docker_context
        else ""
    )
    amd64 = architecture in {"amd64", "x86_64"}
    ubuntu = os_release.get("ID", "").lower() == "ubuntu"
    return HostCapabilities(
        architecture=architecture,
        os_id=os_release.get("ID", ""),
        os_version=os_release.get("VERSION_ID", ""),
        jetson=jetson,
        robot_installable=jetson,
        developer_installable=amd64 and ubuntu,
        display_available=bool(
            environment.get("DISPLAY", "").strip()
            or environment.get("WAYLAND_DISPLAY", "").strip()
        ),
        ssh_agent=bool(environment.get("SSH_AUTH_SOCK", "").strip()),
        gpu_devices=gpu_devices,
        wsl=wsl,
        wslg_available=wslg_available,
        docker_backend=_docker_backend(docker_name, docker_context),
        docker_context=docker_context,
        docker_engine_id=docker_engine_id,
        docker_endpoint=docker_endpoint,
        docker_host_override=environment.get("DOCKER_HOST", "").strip(),
    )


def detect_install_host_capabilities(
    environ: Mapping[str, str] | None = None,
) -> HostCapabilities:
    """Use bootstrap-provided host facts when running inside a setup container."""

    environment = os.environ if environ is None else environ
    detected = detect_host_capabilities(environ=environment)
    architecture = environment.get("ELESIM_HOST_ARCH", "").strip() or detected.architecture
    os_id = environment.get("ELESIM_HOST_OS_ID", "").strip() or detected.os_id
    os_version = (
        environment.get("ELESIM_HOST_OS_VERSION", "").strip() or detected.os_version
    )
    jetson_raw = environment.get("ELESIM_HOST_JETSON", "").strip().lower()
    jetson = (
        jetson_raw in {"1", "true", "yes"}
        if jetson_raw
        else detected.jetson
    )
    gpu_raw = environment.get("ELESIM_HOST_GPU_LIST")
    gpu_devices = (
        _parse_gpu_devices(gpu_raw)
        if gpu_raw is not None
        else detected.gpu_devices
    )
    wsl = _environment_flag(environment, "ELESIM_HOST_WSL", detected.wsl)
    wslg_available = _environment_flag(
        environment,
        "ELESIM_HOST_WSLG",
        detected.wslg_available,
    )
    display_available = _environment_flag(
        environment,
        "ELESIM_HOST_DISPLAY",
        detected.display_available,
    )
    docker_backend = (
        environment.get("ELESIM_HOST_DOCKER_BACKEND", "").strip()
        or detected.docker_backend
    )
    docker_context = (
        environment.get("ELESIM_HOST_DOCKER_CONTEXT", "").strip()
        or detected.docker_context
    )
    docker_engine_id = (
        environment.get("ELESIM_HOST_DOCKER_ENGINE_ID", "").strip()
        or detected.docker_engine_id
    )
    docker_endpoint = (
        environment.get("ELESIM_HOST_DOCKER_ENDPOINT", "").strip()
        or detected.docker_endpoint
    )
    docker_host_override = (
        environment.get("ELESIM_HOST_DOCKER_HOST_OVERRIDE", "").strip()
        if "ELESIM_HOST_DOCKER_HOST_OVERRIDE" in environment
        else detected.docker_host_override
    )
    amd64 = architecture.lower() in {"amd64", "x86_64"}
    return HostCapabilities(
        architecture=architecture,
        os_id=os_id,
        os_version=os_version,
        jetson=jetson,
        robot_installable=jetson,
        developer_installable=amd64 and os_id.lower() == "ubuntu",
        display_available=display_available,
        ssh_agent=detected.ssh_agent,
        gpu_devices=gpu_devices,
        wsl=wsl,
        wslg_available=wslg_available,
        docker_backend=docker_backend,
        docker_context=docker_context,
        docker_engine_id=docker_engine_id,
        docker_endpoint=docker_endpoint,
        docker_host_override=docker_host_override,
    )


def _docker_backend(name: str, context: str) -> str:
    if name == "docker-desktop" or context == "desktop-linux":
        return "docker-desktop"
    if name or context:
        return "native"
    return ""


def _environment_flag(
    environment: Mapping[str, str],
    name: str,
    default: bool,
) -> bool:
    raw = environment.get(name)
    if raw is None or not raw.strip():
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


def _key_values(path: Path) -> dict[str, str]:
    values: dict[str, str] = {}
    for line in _read_text(path).splitlines():
        text = line.strip()
        if not text or text.startswith("#") or "=" not in text:
            continue
        name, value = text.split("=", 1)
        values[name.strip()] = value.strip().strip("\"'")
    return values


def _read_text(path: Path) -> str:
    try:
        return path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return ""


def _run_output(command: tuple[str, ...]) -> str:
    completed = subprocess.run(
        command,
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
        timeout=5,
    )
    return completed.stdout if completed.returncode == 0 else ""


def _optional_output(output: CommandOutput, command: Sequence[str]) -> str:
    try:
        return output(tuple(command))
    except (OSError, subprocess.SubprocessError):
        return ""


def _parse_gpu_devices(output: str) -> tuple[GpuDevice, ...]:
    devices: list[GpuDevice] = []
    for line in output.splitlines():
        match = _GPU_LINE.match(line.strip())
        if match is not None:
            devices.append(GpuDevice(**match.groupdict()))
    return tuple(devices)


__all__ = [
    "GpuDevice",
    "HostCapabilities",
    "detect_host_capabilities",
    "detect_install_host_capabilities",
]
