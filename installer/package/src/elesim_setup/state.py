"""Persistent, non-secret state shared by the installer and network doctor."""

from __future__ import annotations

import json
import os
import re
import tempfile
from dataclasses import asdict, dataclass, field
from pathlib import Path
from typing import Any, Mapping

from .profiles import normalize_roles


# v8 is the first state format whose application roles are named ``pilot`` and
# ``sim``.  Older files are accepted only at this input boundary and are
# normalized to v8 before anything is generated.
STATE_SCHEMA_VERSION = 8
SUPPORTED_STATE_SCHEMAS = frozenset({1, 2, 3, 4, 5, 6, 7, STATE_SCHEMA_VERSION})
GPU_MODES = frozenset({"inherit", "specific", "cpu"})
INSTALL_MODES = frozenset({"native", "container"})
TURN_MODES = frozenset({"none", "managed", "external"})
DDS_DISCOVERY_MODES = frozenset({"multicast", "static"})
DDS_SECURITY_PROFILES = frozenset({"trusted-network", "sros2"})
DDS_SECURITY_PROVISIONING = frozenset({"none", "external", "managed"})
DDS_RMW_IMPLEMENTATIONS = frozenset({"rmw_cyclonedds_cpp"})
DEFAULT_PREFIX = Path("~/.local/share/elesim").expanduser()
DEFAULT_BIN_DIR = Path("~/.local/bin").expanduser()
_ROS_NAME = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
_RMW_NAME = re.compile(r"^rmw_[a-z0-9_]{1,120}$")
_SECURITY_GENERATION = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,95}$")


@dataclass(frozen=True)
class NetworkSettings:
    """Application identities and WebRTC relay endpoints, not DDS locators."""

    turn_urls: tuple[str, ...] = ()
    sim_id: str = "sim-default"
    pilot_id: str = "pilot-main"
    ui_id: str = "ui-main"
    robot_id: str = "robot-go2"

    def validate(self) -> "NetworkSettings":
        _validate_identifier(self.sim_id, name="sim_id")
        _validate_identifier(self.pilot_id, name="pilot_id")
        _validate_identifier(self.ui_id, name="ui_id")
        _validate_identifier(self.robot_id, name="robot_id")
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
class DdsSettings:
    """Shared ROS 2/DDS runtime settings for all hosts in one Elesim system."""

    system_id: str = "elesim"
    domain_id: int = 0
    rmw_implementation: str = "rmw_cyclonedds_cpp"
    discovery_mode: str = "multicast"
    static_peers: tuple[str, ...] = ()
    interface: str = ""
    security_profile: str = "trusted-network"
    security_provisioning: str = "none"
    security_generation: str = ""
    security_bundle: str = ""
    keystore: str = ""
    enclave: str = ""

    def validate(self) -> "DdsSettings":
        if not _ROS_NAME.fullmatch(self.system_id):
            raise ValueError(
                "DDS system_id는 소문자로 시작하는 영문 소문자/숫자/underscore 값이어야 합니다"
            )
        if isinstance(self.domain_id, bool) or not 0 <= int(self.domain_id) <= 232:
            raise ValueError("ROS_DOMAIN_ID는 0..232 범위여야 합니다")
        if (
            not _RMW_NAME.fullmatch(self.rmw_implementation)
            or self.rmw_implementation not in DDS_RMW_IMPLEMENTATIONS
        ):
            supported = ", ".join(sorted(DDS_RMW_IMPLEMENTATIONS))
            raise ValueError(
                f"지원하지 않는 RMW implementation: {self.rmw_implementation!r}; "
                f"supported: {supported}"
            )
        if self.discovery_mode not in DDS_DISCOVERY_MODES:
            raise ValueError(f"지원하지 않는 DDS discovery mode: {self.discovery_mode!r}")
        peers = tuple(str(value).strip() for value in self.static_peers)
        if any(not value or len(value) > 255 or any(ch.isspace() for ch in value) for value in peers):
            raise ValueError("DDS static peer는 공백 없는 hostname/IP여야 합니다")
        if self.discovery_mode == "multicast" and peers:
            raise ValueError("multicast discovery에는 static peer를 함께 지정할 수 없습니다")
        interface = str(self.interface).strip()
        if (
            len(interface) > 128
            or any(character.isspace() for character in interface)
            or "/" in interface
        ):
            raise ValueError("DDS interface는 공백과 경로 구분자가 없는 interface 이름이어야 합니다")
        if self.security_profile not in DDS_SECURITY_PROFILES:
            raise ValueError(
                f"지원하지 않는 DDS security profile: {self.security_profile!r}"
            )
        if self.security_provisioning not in DDS_SECURITY_PROVISIONING:
            raise ValueError(
                "지원하지 않는 DDS security provisioning: "
                f"{self.security_provisioning!r}"
            )
        if not isinstance(self.security_generation, str):
            raise ValueError("DDS security generation은 문자열 식별자여야 합니다")
        generation = self.security_generation.strip()
        if generation and not _SECURITY_GENERATION.fullmatch(generation):
            raise ValueError(
                "DDS security generation은 소문자/숫자로 시작하는 안전한 식별자여야 합니다"
            )
        bundle = str(self.security_bundle).strip()
        keystore = str(self.keystore).strip()
        enclave = str(self.enclave).strip()
        if bool(keystore) != bool(enclave):
            raise ValueError("SROS2 keystore와 enclave는 함께 지정해야 합니다")
        if self.security_profile == "trusted-network" and (
            self.security_provisioning != "none"
            or generation
            or bundle
            or keystore
            or enclave
        ):
            raise ValueError(
                "trusted-network profile에는 SROS2 provisioning/generation/"
                "bundle/keystore/enclave를 지정할 수 없습니다"
            )
        if self.security_profile == "sros2":
            if self.security_provisioning == "none":
                raise ValueError("sros2 profile에는 security provisioning이 필요합니다")
            if self.security_provisioning == "external" and (generation or bundle):
                raise ValueError(
                    "external SROS2 provisioning에는 managed generation/bundle을 "
                    "지정할 수 없습니다"
                )
            if self.security_provisioning == "managed":
                managed_values = (generation, bundle, keystore, enclave)
                if any(managed_values) and not all(managed_values):
                    raise ValueError(
                        "managed SROS2 provisioning은 아직 provision되지 않은 all-empty "
                        "상태이거나 generation/bundle/keystore/enclave가 모두 필요합니다"
                    )
                if bundle and Path(bundle).expanduser().resolve() != Path(
                    keystore
                ).expanduser().resolve():
                    raise ValueError(
                        "managed SROS2 keystore는 role bundle 경로와 같아야 합니다"
                    )
        if enclave and (not enclave.startswith("/") or ".." in Path(enclave).parts):
            raise ValueError("SROS2 enclave는 '..'이 없는 절대 ROS 경로여야 합니다")
        return self

    @property
    def keystore_path(self) -> Path | None:
        value = str(self.keystore).strip()
        return None if not value else Path(value).expanduser().resolve()

    @property
    def security_bundle_path(self) -> Path | None:
        value = str(self.security_bundle).strip()
        return None if not value else Path(value).expanduser().resolve()

    @property
    def migrated_security_needs_configuration(self) -> bool:
        return (
            self.security_profile == "sros2"
            and self.security_provisioning == "external"
            and not self.keystore.strip()
            and not self.enclave.strip()
        )

    @property
    def managed_security_pending(self) -> bool:
        """Whether a managed profile is installable but has no runtime bundle yet."""

        return self.security_profile == "sros2" and (
            self.security_provisioning == "managed"
            and not any(
                str(value).strip()
                for value in (
                    self.security_generation,
                    self.security_bundle,
                    self.keystore,
                    self.enclave,
                )
            )
        )


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
class RuntimeTextLogSettings:
    """Local plain-text snapshots of this install's managed runtime logs."""

    enabled: bool = True

    def validate(self) -> "RuntimeTextLogSettings":
        if not isinstance(self.enabled, bool):
            raise ValueError("runtime text log enabled 값은 boolean이어야 합니다")
        return self


@dataclass(frozen=True)
class TurnSettings:
    """Installer ownership of a TURN relay; URLs remain network endpoints."""

    mode: str = "none"
    realm: str = ""
    public_host: str = ""
    secret_file: str = ""
    credential_file: str = ""

    def validate(self) -> "TurnSettings":
        if self.mode not in TURN_MODES:
            raise ValueError(f"지원하지 않는 TURN 모드: {self.mode!r}")
        realm = self.realm.strip()
        public_host = self.public_host.strip()
        secret_file = self.secret_file.strip()
        credential_file = self.credential_file.strip()
        if self.mode == "managed":
            if not realm:
                raise ValueError("managed TURN에는 realm이 필요합니다")
            _validate_connect_host(public_host, name="TURN public hostname/IP")
            if not secret_file:
                raise ValueError("managed TURN에는 secret file 경로가 필요합니다")
            if credential_file:
                raise ValueError(
                    "managed TURN에는 external credential file을 지정할 수 없습니다"
                )
        elif self.mode == "external":
            if realm or public_host or secret_file:
                raise ValueError(
                    "TURN realm/public_host/secret_file은 managed 모드에서만 "
                    "지정할 수 있습니다"
                )
        elif realm or public_host or secret_file or credential_file:
            raise ValueError(
                "TURN credential 설정은 TURN이 활성화된 경우에만 지정할 수 있습니다"
            )
        return self

    @property
    def managed(self) -> bool:
        return self.mode == "managed"

    @property
    def secret_path(self) -> Path | None:
        value = self.secret_file.strip()
        return None if not value else Path(value).expanduser().resolve()

    @property
    def credential_path(self) -> Path | None:
        value = self.credential_file.strip()
        return None if not value else Path(value).expanduser().resolve()


@dataclass(frozen=True)
class InstallState:
    profile: str
    roles: tuple[str, ...]
    prefix: str
    bin_dir: str
    source_root: str
    network: NetworkSettings = field(default_factory=NetworkSettings)
    dds: DdsSettings = field(default_factory=DdsSettings)
    compute: ComputeSettings = field(default_factory=ComputeSettings)
    turn: TurnSettings = field(default_factory=TurnSettings)
    runtime_text_logs: RuntimeTextLogSettings = field(
        default_factory=RuntimeTextLogSettings
    )
    install_mode: str = "container"
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
        roles = normalize_roles(self.roles)
        if not self.prefix.strip() or not self.bin_dir.strip() or not self.source_root.strip():
            raise ValueError("prefix, bin_dir와 source_root가 필요합니다")
        self.network.validate()
        self.dds.validate()
        self.compute.validate()
        self.turn.validate()
        self.runtime_text_logs.validate()
        if self.install_mode not in INSTALL_MODES:
            raise ValueError(f"지원하지 않는 설치 방식: {self.install_mode!r}")
        if "robot" in roles and roles != ("robot",):
            raise ValueError(
                "Robot native 설치는 다른 역할과 분리한 Robot 단독 "
                "설치여야 합니다"
            )
        if roles == ("robot",) and self.install_mode != "native":
            raise ValueError(
                "Robot Jetson은 generic Ubuntu 컨테이너로 설치할 수 없습니다. "
                "JetPack/L4T, ROS2와 unitree_ros2가 준비된 Jetson에서 native 설치를 사용하십시오"
            )
        if roles != ("robot",) and self.install_mode != "container":
            raise ValueError(
                "Sim, Pilot과 UI는 일반 Docker/Compose 설치만 "
                "지원합니다; native 설치는 Robot Jetson 단독 전용입니다"
            )
        has_turn_urls = bool(self.network.turn_urls)
        if self.turn.mode == "none" and has_turn_urls:
            raise ValueError("TURN URL에는 managed 또는 external TURN 모드가 필요합니다")
        if self.turn.mode != "none" and not has_turn_urls:
            raise ValueError(f"{self.turn.mode} TURN 모드에는 TURN URL이 필요합니다")
        if self.turn.managed and "sim" not in self.roles:
            raise ValueError(
                "managed Coturn은 Sim가 설치되는 호스트에서만 사용할 수 있습니다"
            )
        if self.turn.managed and self.install_mode != "container":
            raise ValueError("managed Coturn lifecycle에는 container 설치가 필요합니다")
        if self.turn.managed and self.dds.security_profile != "sros2":
            raise ValueError(
                "managed TURN credential와 WebRTC signaling에는 sros2 profile이 필요합니다"
            )
        if (
            self.turn.mode == "external"
            and self.turn.credential_path is not None
            and "sim" not in self.roles
        ):
            raise ValueError(
                "external TURN credential file은 Sim 설치 호스트에만 "
                "배포할 수 있습니다"
            )
        return self

    def require_installable_dds(self) -> "InstallState":
        """Validate artifacts that can be generated before managed provisioning."""

        self.validate()
        if self.dds.discovery_mode == "static" and not self.dds.static_peers:
            raise ValueError(
                "static DDS discovery에는 peer가 필요합니다. "
                "이전 ZMQ 상태에서 자동으로 Router 주소를 peer로 재사용하지 않습니다"
            )
        if self.dds.migrated_security_needs_configuration:
            raise ValueError(
                "이전 Curve 상태는 SROS2 key로 자동 변환할 수 없습니다. "
                "SROS2 keystore와 enclave를 명시하십시오"
            )
        if (
            self.turn.mode == "external"
            and "sim" in self.roles
            and self.turn.credential_path is None
        ):
            raise ValueError(
                "Sim의 external TURN에는 username/credential JSON file이 "
                "필요합니다. 이전 상태라면 TURN 자격증명 경로를 다시 지정하십시오"
            )
        return self

    def require_runnable_dds(self) -> "InstallState":
        """Fail closed unless this state has usable DDS runtime credentials."""

        self.require_installable_dds()
        if self.dds.managed_security_pending:
            raise ValueError(
                "managed SROS2 role bundle이 아직 provision되지 않았습니다. "
                "operator laptop에서 elesim-connections를 실행하십시오"
            )
        return self

    def to_dict(self) -> dict[str, Any]:
        raw = asdict(self)
        raw["roles"] = list(self.roles)
        raw["network"]["turn_urls"] = list(self.network.turn_urls)
        raw["dds"]["static_peers"] = list(self.dds.static_peers)
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
        compute_raw = raw.get("compute", {})
        turn_raw = raw.get("turn", {})
        dds_raw = raw.get("dds", {})
        runtime_text_logs_raw = raw.get("runtime_text_logs", {})
        if not all(
            isinstance(value, Mapping)
            for value in (
                network_raw,
                compute_raw,
                turn_raw,
                dds_raw,
                runtime_text_logs_raw,
            )
        ):
            raise ValueError(
                "설치 상태의 network/dds/compute/turn/runtime_text_logs가 "
                "object가 아닙니다"
            )

        network_values = dict(network_raw)
        network_values["turn_urls"] = tuple(network_values.get("turn_urls", ()))
        # The old names are migration input only.  No normal state or emitted
        # config keeps these keys.
        if "sim_id" not in network_values:
            legacy_sim_id = str(network_values.get("simulator_id", "sim-default"))
            network_values["sim_id"] = legacy_sim_id.replace("simulator-", "sim-", 1)
        if "pilot_id" not in network_values:
            legacy_pilot_id = str(network_values.get("controller_id", "pilot-main"))
            network_values["pilot_id"] = legacy_pilot_id.replace("controller-", "pilot-", 1)
        network = NetworkSettings(
            turn_urls=network_values["turn_urls"],
            sim_id=str(network_values.get("sim_id", "sim-default")),
            pilot_id=str(network_values.get("pilot_id", "pilot-main")),
            ui_id=str(network_values.get("ui_id", "ui-main")),
            robot_id=str(network_values.get("robot_id", "robot-go2")),
        )
        if source_schema < 3:
            turn_raw = {
                "mode": "external" if network.turn_urls else "none",
            }
        if source_schema < 4:
            security_raw = raw.get("security", {})
            if not isinstance(security_raw, Mapping):
                raise ValueError("설치 상태의 legacy security가 object가 아닙니다")
            legacy_security = str(security_raw.get("mode", "loopback"))
            dds = DdsSettings(
                # The old Router/advertise addresses are deliberately not peers.
                discovery_mode="multicast",
                static_peers=(),
                security_profile=(
                    "sros2" if legacy_security == "curve" else "trusted-network"
                ),
                security_provisioning=(
                    "external" if legacy_security == "curve" else "none"
                ),
            )
        else:
            dds_values = dict(dds_raw)
            dds_values["static_peers"] = tuple(dds_values.get("static_peers", ()))
            if source_schema < 6:
                dds_values.setdefault(
                    "security_provisioning",
                    "external"
                    if str(dds_values.get("security_profile", "trusted-network"))
                    == "sros2"
                    else "none",
                )
                dds_values.setdefault("security_generation", "")
                dds_values.setdefault("security_bundle", "")
            dds = DdsSettings(**dds_values)

        turn_values = dict(turn_raw)
        if source_schema < 5:
            # v1..v4 external TURN stored only its URL. Keep the state
            # inspectable, but require_runnable_dds() fails closed before a
            # Sim configuration can be regenerated without credentials.
            turn_values.setdefault("credential_file", "")
        if (
            source_schema < 4
            and str(turn_values.get("mode", "none")) == "managed"
            and not str(turn_values.get("secret_file", "")).strip()
        ):
            legacy_security = raw.get("security", {})
            legacy_root = (
                str(legacy_security.get("credentials_root", "")).strip()
                if isinstance(legacy_security, Mapping)
                else ""
            )
            if legacy_root:
                turn_values["secret_file"] = str(
                    Path(legacy_root).expanduser().resolve() / "turn.secret"
                )

        legacy_roles = {"controller": "pilot", "simulator": "sim"}
        roles = tuple(
            legacy_roles.get(role, role)
            for role in (str(value).strip().lower() for value in raw.get("roles", ()))
            if role != "router"
        )
        install_mode = str(
            raw.get(
                "install_mode",
                "native" if normalize_roles(roles) == ("robot",) else "container",
            )
        )
        return cls(
            profile=str(raw.get("profile", "custom")),
            roles=roles,
            prefix=str(raw.get("prefix", "")),
            bin_dir=str(raw.get("bin_dir", "")),
            source_root=str(raw.get("source_root", "")),
            network=network,
            dds=dds,
            compute=ComputeSettings(**dict(compute_raw)),
            turn=TurnSettings(**turn_values),
            # Existing installations did not opt into persistent plain-text
            # archives. Migration therefore preserves their previous behavior.
            runtime_text_logs=(
                RuntimeTextLogSettings(enabled=False)
                if source_schema < 7
                else RuntimeTextLogSettings(**dict(runtime_text_logs_raw))
            ),
            install_mode=install_mode,
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
    "DDS_DISCOVERY_MODES",
    "DDS_RMW_IMPLEMENTATIONS",
    "DDS_SECURITY_PROFILES",
    "DDS_SECURITY_PROVISIONING",
    "DEFAULT_BIN_DIR",
    "DEFAULT_PREFIX",
    "ComputeSettings",
    "DdsSettings",
    "GPU_MODES",
    "INSTALL_MODES",
    "InstallState",
    "NetworkSettings",
    "RuntimeTextLogSettings",
    "STATE_SCHEMA_VERSION",
    "TURN_MODES",
    "TurnSettings",
    "default_state_path",
]
