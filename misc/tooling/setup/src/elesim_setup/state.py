"""Persistent, non-secret state shared by the installer and network doctor."""

from __future__ import annotations

import json
import os
import tempfile
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Mapping

from .profiles import normalize_roles


STATE_SCHEMA_VERSION = 2
SUPPORTED_STATE_SCHEMAS = frozenset({1, STATE_SCHEMA_VERSION})
SECURITY_MODES = frozenset({"loopback", "curve", "insecure-lan"})
GPU_MODES = frozenset({"inherit", "specific", "cpu"})
INSTALL_MODES = frozenset({"native", "container"})
DEFAULT_PREFIX = Path("~/.local/share/elesim").expanduser()
DEFAULT_BIN_DIR = Path("~/.local/bin").expanduser()


@dataclass(frozen=True)
class NetworkSettings:
    router_host: str = "127.0.0.1"
    advertise_host: str = "127.0.0.1"
    router_port: int = 5558
    rgbd_port: int = 5568
    turn_urls: tuple[str, ...] = ()
    simulator_id: str = "sim-default"
    controller_id: str = "controller-main"

    def validate(self) -> "NetworkSettings":
        _validate_connect_host(self.router_host, name="Router hostname/IP")
        _validate_connect_host(self.advertise_host, name="advertise hostname/IP")
        for name, value in (("router_port", self.router_port), ("rgbd_port", self.rgbd_port)):
            if isinstance(value, bool) or not 1 <= int(value) <= 65535:
                raise ValueError(f"{name}는 1..65535 범위여야 합니다")
        _validate_identifier(self.simulator_id, name="simulator_id")
        _validate_identifier(self.controller_id, name="controller_id")
        for value in self.turn_urls:
            url = str(value).strip()
            if (
                not url.startswith(("turn:", "turns:"))
                or len(url) > 2048
                or any(character.isspace() for character in url)
            ):
                raise ValueError(f"유효하지 않은 TURN URL: {value!r}")
        return self


@dataclass(frozen=True)
class SecuritySettings:
    mode: str = "loopback"
    credentials_root: str = ""

    def validate(self) -> "SecuritySettings":
        if self.mode not in SECURITY_MODES:
            raise ValueError(f"지원하지 않는 보안 모드: {self.mode!r}")
        if self.mode == "curve" and not self.credentials_root.strip():
            raise ValueError("CURVE 모드에는 credentials root가 필요합니다")
        return self

    @property
    def allow_insecure_remote(self) -> bool:
        return self.mode == "insecure-lan"

    @property
    def root(self) -> Path | None:
        if not self.credentials_root.strip():
            return None
        return Path(self.credentials_root).expanduser().resolve()


@dataclass(frozen=True)
class ComputeSettings:
    gpu_mode: str = "inherit"
    gpu_device: str = ""

    def validate(self) -> "ComputeSettings":
        if self.gpu_mode not in GPU_MODES:
            raise ValueError(f"지원하지 않는 GPU 모드: {self.gpu_mode!r}")
        device = self.gpu_device.strip()
        if self.gpu_mode == "specific":
            if (
                not device
                or len(device) > 128
                or "," in device
                or any(character.isspace() for character in device)
            ):
                raise ValueError(
                    "specific GPU 모드에는 하나의 공백 없는 GPU index 또는 UUID가 필요합니다"
                )
            if device.startswith(("+", "-")) and device[1:].isdigit():
                raise ValueError("GPU index는 0 이상이어야 합니다")
        elif device:
            raise ValueError("gpu_device는 specific GPU 모드에서만 지정할 수 있습니다")
        return self


@dataclass(frozen=True)
class InstallState:
    profile: str
    roles: tuple[str, ...]
    prefix: str
    bin_dir: str
    source_root: str
    network: NetworkSettings = field(default_factory=NetworkSettings)
    security: SecuritySettings = field(default_factory=SecuritySettings)
    compute: ComputeSettings = field(default_factory=ComputeSettings)
    install_mode: str = "native"
    install_go2_mpc: bool = True
    schema_version: int = STATE_SCHEMA_VERSION

    @property
    def prefix_path(self) -> Path:
        return Path(self.prefix).expanduser().resolve()

    @property
    def bin_path(self) -> Path:
        return Path(self.bin_dir).expanduser().resolve()

    @property
    def source_path(self) -> Path:
        return Path(self.source_root).expanduser().resolve()

    @property
    def state_path(self) -> Path:
        return self.prefix_path / "install-state.json"

    def validate(self) -> "InstallState":
        if self.schema_version != STATE_SCHEMA_VERSION:
            raise ValueError(
                f"설치 상태 schema {self.schema_version!r}는 지원되지 않습니다; "
                f"expected {STATE_SCHEMA_VERSION}"
            )
        normalize_roles(self.roles)
        if not self.prefix.strip() or not self.bin_dir.strip() or not self.source_root.strip():
            raise ValueError("prefix, bin_dir와 source_root가 필요합니다")
        self.network.validate()
        self.security.validate()
        self.compute.validate()
        if self.install_mode not in INSTALL_MODES:
            raise ValueError(f"지원하지 않는 설치 방식: {self.install_mode!r}")
        if self.install_mode == "container" and "robot" in self.roles:
            raise ValueError(
                "Robot Jetson은 generic Ubuntu 컨테이너로 설치할 수 없습니다. "
                "JetPack/L4T, ROS2와 unitree_ros2가 준비된 Jetson에서 native 설치를 사용하십시오"
            )
        if self.security.mode == "loopback" and not _host_is_loopback(self.network.router_host):
            raise ValueError("loopback 보안 모드는 loopback Router 주소에서만 사용할 수 있습니다")
        if (
            "router" in self.roles
            and self.network.turn_urls
            and self.security.mode != "curve"
        ):
            raise ValueError("TURN을 발급하는 Router는 CURVE credential root가 필요합니다")
        return self

    def to_dict(self) -> dict[str, Any]:
        raw = asdict(self)
        raw["roles"] = list(self.roles)
        raw["network"]["turn_urls"] = list(self.network.turn_urls)
        return raw

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "InstallState":
        source_schema = int(raw.get("schema_version", 0))
        if source_schema not in SUPPORTED_STATE_SCHEMAS:
            raise ValueError(
                f"설치 상태 schema {source_schema!r}는 지원되지 않습니다; "
                f"expected one of {sorted(SUPPORTED_STATE_SCHEMAS)}"
            )
        network_raw = raw.get("network", {})
        security_raw = raw.get("security", {})
        compute_raw = raw.get("compute", {})
        if not all(
            isinstance(value, Mapping)
            for value in (network_raw, security_raw, compute_raw)
        ):
            raise ValueError("설치 상태의 network/security/compute가 object가 아닙니다")
        network_values = dict(network_raw)
        network_values["turn_urls"] = tuple(network_values.get("turn_urls", ()))
        return cls(
            profile=str(raw.get("profile", "custom")),
            roles=tuple(str(value) for value in raw.get("roles", ())),
            prefix=str(raw.get("prefix", "")),
            bin_dir=str(raw.get("bin_dir", "")),
            source_root=str(raw.get("source_root", "")),
            network=NetworkSettings(**network_values),
            security=SecuritySettings(**dict(security_raw)),
            compute=ComputeSettings(**dict(compute_raw)),
            install_mode=str(raw.get("install_mode", "native")),
            install_go2_mpc=bool(raw.get("install_go2_mpc", True)),
            schema_version=STATE_SCHEMA_VERSION,
        ).validate()

    @classmethod
    def load(cls, path: str | os.PathLike[str] | None = None) -> "InstallState":
        source = default_state_path() if path is None else Path(path).expanduser().resolve()
        raw = json.loads(source.read_text(encoding="utf-8"))
        if not isinstance(raw, Mapping):
            raise ValueError(f"{source}: 설치 상태가 JSON object가 아닙니다")
        return cls.from_dict(raw)

    def save(self, path: str | os.PathLike[str] | None = None) -> Path:
        self.validate()
        destination = self.state_path if path is None else Path(path).expanduser().resolve()
        destination.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(self.to_dict(), ensure_ascii=False, indent=2) + "\n"
        with tempfile.NamedTemporaryFile(
            "w",
            encoding="utf-8",
            dir=destination.parent,
            prefix=f".{destination.name}.",
            delete=False,
        ) as handle:
            handle.write(payload)
            temporary = Path(handle.name)
        temporary.chmod(0o600)
        temporary.replace(destination)
        return destination


def default_state_path() -> Path:
    override = os.environ.get("ELESIM_STATE", "").strip()
    if override:
        return Path(override).expanduser().resolve()
    return DEFAULT_PREFIX / "install-state.json"


def _host_is_loopback(host: str) -> bool:
    import ipaddress

    value = str(host).strip().lower().removeprefix("[").removesuffix("]")
    if value == "localhost":
        return True
    try:
        return ipaddress.ip_address(value).is_loopback
    except ValueError:
        return False


def _validate_connect_host(host: object, *, name: str) -> None:
    import ipaddress

    value = str(host).strip()
    unbracketed = value.removeprefix("[").removesuffix("]")
    if (
        not value
        or "://" in value
        or "/" in value
        or any(character.isspace() for character in value)
    ):
        raise ValueError(f"{name}가 hostname 또는 IP 형식이 아닙니다: {host!r}")
    try:
        address = ipaddress.ip_address(unbracketed)
    except ValueError:
        if ":" in unbracketed:
            raise ValueError(f"{name}에는 port를 포함하지 마십시오: {host!r}")
        return
    if address.is_unspecified:
        raise ValueError(f"{name}에는 bind 전용 주소 {value!r}를 사용할 수 없습니다")


def _validate_identifier(value: object, *, name: str) -> None:
    text = str(value).strip()
    if not text or len(text) > 128 or any(character.isspace() for character in text):
        raise ValueError(f"{name}은 1..128자의 공백 없는 값이어야 합니다")


__all__ = [
    "DEFAULT_BIN_DIR",
    "DEFAULT_PREFIX",
    "ComputeSettings",
    "GPU_MODES",
    "INSTALL_MODES",
    "InstallState",
    "NetworkSettings",
    "SECURITY_MODES",
    "STATE_SCHEMA_VERSION",
    "SecuritySettings",
    "default_state_path",
]
