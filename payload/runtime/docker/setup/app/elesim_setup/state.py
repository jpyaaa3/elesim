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
# ``sim``.  v9 pins the Docker daemon.  v10 makes development tooling an
# optional attachment to the one runtime installation instead of a separate
# installation edition and Compose project. v11 separates the installed role
# inventory from the roles assigned to the current connection topology.
STATE_SCHEMA_VERSION = 11
SUPPORTED_STATE_SCHEMAS = frozenset(
    {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, STATE_SCHEMA_VERSION}
)
GPU_MODES = frozenset({"inherit", "specific", "cpu"})
INSTALL_MODES = frozenset({"native", "container"})
CONTAINER_NETWORK_MODES = frozenset({"direct-host", "tailscale-sidecar"})
TURN_MODES = frozenset({"none", "managed", "external"})
DDS_DISCOVERY_MODES = frozenset({"multicast", "static"})
DDS_SECURITY_PROFILES = frozenset({"trusted-network", "sros2"})
DDS_SECURITY_PROVISIONING = frozenset({"none", "external", "managed"})
DDS_RMW_IMPLEMENTATIONS = frozenset({"rmw_cyclonedds_cpp"})
DEFAULT_PREFIX = Path("~/.local/share/elesim").expanduser()
DEFAULT_BIN_DIR = Path("~/.local/bin").expanduser()
DEFAULT_SOURCE_REPOSITORY = "jpyaaa3/elesim"
DEFAULT_SOURCE_REF = "main"
_ROS_NAME = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
_RMW_NAME = re.compile(r"^rmw_[a-z0-9_]{1,120}$")
_SECURITY_GENERATION = re.compile(r"^[a-z0-9][a-z0-9_.-]{0,95}$")
_TAILSCALE_HOSTNAME = re.compile(r"^[a-z0-9](?:[a-z0-9-]{0,61}[a-z0-9])?$")


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
                raise ValueError(f"Invalid TURN URL: {value!r}")
        return self


@dataclass(frozen=True)
class DdsSettings:
    """Shared ROS 2/DDS runtime settings for all hosts in one EleSim system."""

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
                "DDS system_id must start with a lowercase letter and contain only lowercase letters, digits, or underscores"
            )
        if isinstance(self.domain_id, bool) or not 0 <= int(self.domain_id) <= 232:
            raise ValueError("ROS_DOMAIN_ID must be in the range 0..232")
        if (
            not _RMW_NAME.fullmatch(self.rmw_implementation)
            or self.rmw_implementation not in DDS_RMW_IMPLEMENTATIONS
        ):
            supported = ", ".join(sorted(DDS_RMW_IMPLEMENTATIONS))
            raise ValueError(
                f"Unsupported RMW implementation: {self.rmw_implementation!r}; "
                f"supported: {supported}"
            )
        if self.discovery_mode not in DDS_DISCOVERY_MODES:
            raise ValueError(f"Unsupported DDS discovery mode: {self.discovery_mode!r}")
        peers = tuple(str(value).strip() for value in self.static_peers)
        if any(not value or len(value) > 255 or any(ch.isspace() for ch in value) for value in peers):
            raise ValueError("DDS static peer must be a whitespace-free hostname/IP")
        if self.discovery_mode == "multicast" and peers:
            raise ValueError("static peers cannot be specified with multicast discovery")
        interface = str(self.interface).strip()
        if (
            len(interface) > 128
            or any(character.isspace() for character in interface)
            or "/" in interface
        ):
            raise ValueError("DDS interface must be a whitespace-free interface name without path separators")
        if self.security_profile not in DDS_SECURITY_PROFILES:
            raise ValueError(
                f"Unsupported DDS security profile: {self.security_profile!r}"
            )
        if self.security_provisioning not in DDS_SECURITY_PROVISIONING:
            raise ValueError(
                "Unsupported DDS security provisioning: "
                f"{self.security_provisioning!r}"
            )
        if not isinstance(self.security_generation, str):
            raise ValueError("DDS security generation must be a string identifier")
        generation = self.security_generation.strip()
        if generation and not _SECURITY_GENERATION.fullmatch(generation):
            raise ValueError(
                "DDS security generation must be a safe identifier beginning with a lowercase letter or digit"
            )
        bundle = str(self.security_bundle).strip()
        keystore = str(self.keystore).strip()
        enclave = str(self.enclave).strip()
        if bool(keystore) != bool(enclave):
            raise ValueError("SROS2 keystore and enclave must be specified together")
        if self.security_profile == "trusted-network" and (
            self.security_provisioning != "none"
            or generation
            or bundle
            or keystore
            or enclave
        ):
            raise ValueError(
                "trusted-network profile cannot specify SROS2 provisioning/generation/"
                "bundle/keystore/enclave"
            )
        if self.security_profile == "sros2":
            if self.security_provisioning == "none":
                raise ValueError("sros2 profile requires security provisioning")
            if self.security_provisioning == "external" and (generation or bundle):
                raise ValueError(
                    "external SROS2 provisioning cannot specify a managed generation/bundle"
                )
            if self.security_provisioning == "managed":
                managed_values = (generation, bundle, keystore, enclave)
                if any(managed_values) and not all(managed_values):
                    raise ValueError(
                        "managed SROS2 provisioning must be all-empty before provisioning "
                        "or include all of generation, bundle, keystore, and enclave"
                    )
                if bundle and Path(bundle).expanduser().resolve() != Path(
                    keystore
                ).expanduser().resolve():
                    raise ValueError(
                        "managed SROS2 keystore must match the role bundle path"
                    )
        if enclave and (not enclave.startswith("/") or ".." in Path(enclave).parts):
            raise ValueError("SROS2 enclave must be an absolute ROS path without '..'")
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
            raise ValueError(f"Unsupported GPU mode: {self.gpu_mode!r}")
        device = self.gpu_device.strip()
        if self.gpu_mode == "specific":
            if (
                not device
                or len(device) > 128
                or "," in device
                or any(character.isspace() for character in device)
            ):
                raise ValueError(
                    "specific GPU mode requires one whitespace-free GPU index or UUID"
                )
            if device.startswith(("+", "-")) and device[1:].isdigit():
                raise ValueError("GPU index must be non-negative")
        elif device:
            raise ValueError("gpu_device is only valid in specific GPU mode")
        return self


@dataclass(frozen=True)
class RuntimeTextLogSettings:
    """Local plain-text snapshots of this install's managed runtime logs."""

    enabled: bool = True

    def validate(self) -> "RuntimeTextLogSettings":
        if not isinstance(self.enabled, bool):
            raise ValueError("runtime text log enabled must be a boolean")
        return self


@dataclass(frozen=True)
class TurnSettings:
    """Installer ownership of a TURN relay; URLs remain network endpoints."""

    mode: str = "none"
    realm: str = ""
    public_host: str = ""
    secret_file: str = ""
    credential_file: str = ""
    # Scoped instances leave these unset and receive a deterministic allocation
    # from the instance registry.  Legacy install state keeps the historical
    # Coturn defaults through the effective_* helpers below.
    listen_port: int | None = None
    relay_min_port: int | None = None
    relay_max_port: int | None = None

    def validate(self) -> "TurnSettings":
        if self.mode not in TURN_MODES:
            raise ValueError(f"Unsupported TURN mode: {self.mode!r}")
        realm = self.realm.strip()
        public_host = self.public_host.strip()
        secret_file = self.secret_file.strip()
        credential_file = self.credential_file.strip()
        ports = (self.listen_port, self.relay_min_port, self.relay_max_port)
        for name, port in zip(("listen_port", "relay_min_port", "relay_max_port"), ports):
            if port is not None and (
                isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535
            ):
                raise ValueError(f"{name} must be an integer from 1 through 65535")
        if (self.relay_min_port is None) != (self.relay_max_port is None):
            raise ValueError("relay_min_port and relay_max_port must be set together")
        if (
            self.relay_min_port is not None
            and self.relay_max_port is not None
            and self.relay_min_port > self.relay_max_port
        ):
            raise ValueError("relay_min_port must not exceed relay_max_port")
        if self.mode != "managed" and any(port is not None for port in ports):
            raise ValueError("TURN ports are supported only for managed TURN")
        if self.mode == "managed":
            if not realm:
                raise ValueError("managed TURN requires a realm")
            # A new general install deliberately leaves the endpoint empty.
            # The connection manager fills it from the Sim host's current
            # advertised address after the topology is saved.  Once a URL is
            # present, InstallState.validate requires the public host too.
            if public_host:
                _validate_connect_host(public_host, name="TURN public hostname/IP")
            if not secret_file:
                raise ValueError("managed TURN requires a secret file path")
            if credential_file:
                raise ValueError(
                    "managed TURN cannot specify an external credential file"
                )
        elif self.mode == "external":
            if realm or public_host or secret_file:
                raise ValueError(
                    "TURN realm/public_host/secret_file can only be specified in "
                    "managed mode"
                )
        elif realm or public_host or secret_file or credential_file:
            raise ValueError(
                "TURN credentials can only be specified when TURN is enabled"
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

    @property
    def effective_listen_port(self) -> int:
        """Return the legacy fixed listener when no scoped allocation exists."""

        return 3478 if self.listen_port is None else self.listen_port

    @property
    def effective_relay_min_port(self) -> int:
        return 49160 if self.relay_min_port is None else self.relay_min_port

    @property
    def effective_relay_max_port(self) -> int:
        return 49200 if self.relay_max_port is None else self.relay_max_port


@dataclass(frozen=True)
class ContainerNetworkSettings:
    """Non-secret Docker daemon and runtime-network selection.

    ``docker_context`` and ``docker_engine_id`` are an all-or-nothing daemon
    pin.  The sidecar stores its private machine state only at the exact path
    recorded here; authentication material is deliberately absent from this
    schema.
    """

    mode: str = "direct-host"
    docker_context: str = ""
    docker_engine_id: str = ""
    tailscale_hostname: str = ""
    tailscale_state_dir: str = ""

    def validate(self) -> "ContainerNetworkSettings":
        if self.mode not in CONTAINER_NETWORK_MODES:
            raise ValueError(
                f"Unsupported container network mode: {self.mode!r}"
            )
        context = _bounded_single_line(
            self.docker_context,
            name="Docker context",
            maximum=255,
        )
        engine_id = _bounded_single_line(
            self.docker_engine_id,
            name="Docker engine ID",
            maximum=256,
        )
        if bool(context) != bool(engine_id):
            raise ValueError(
                "Docker context and engine ID must both be specified or both omitted"
            )
        hostname = _bounded_single_line(
            self.tailscale_hostname,
            name="Tailscale hostname",
            maximum=63,
        )
        state_dir = _bounded_single_line(
            self.tailscale_state_dir,
            name="Tailscale state directory",
            maximum=4096,
        )
        if self.mode == "tailscale-sidecar":
            if not context or not engine_id:
                raise ValueError(
                    "Tailscale sidecar requires a pinned Docker context and engine ID"
                )
            if not hostname or not _TAILSCALE_HOSTNAME.fullmatch(hostname):
                raise ValueError(
                    "Tailscale hostname must be a 1..63-character lowercase DNS label"
                )
            if not state_dir or not Path(state_dir).expanduser().is_absolute():
                raise ValueError(
                    "Tailscale state directory must be an absolute path"
                )
        elif hostname or state_dir:
            raise ValueError(
                "direct-host mode cannot specify a Tailscale sidecar hostname or state"
            )
        return self

    @property
    def uses_tailscale_sidecar(self) -> bool:
        return self.mode == "tailscale-sidecar"

    @property
    def tailscale_state_path(self) -> Path | None:
        value = self.tailscale_state_dir.strip()
        if not value:
            return None
        # Preserve the exact lexical child path. Resolving here would follow a
        # malicious ``secrets/tailscale`` symlink and make the escaped target
        # appear equal to an also-resolved expected path.
        return Path(os.path.abspath(Path(value).expanduser()))


@dataclass(frozen=True)
class DeveloperAttachmentSettings:
    """Optional coding-tool attachment; never a runtime application role."""

    enabled: bool = False
    workspace: str = ""
    wslg: bool = False

    def validate(self) -> "DeveloperAttachmentSettings":
        if type(self.enabled) is not bool or type(self.wslg) is not bool:
            raise ValueError("developer attachment enabled/wslg must be boolean")
        value = str(self.workspace).strip()
        if self.enabled:
            if not value:
                raise ValueError("developer attachment requires a Git workspace")
            if not Path(value).expanduser().is_absolute():
                raise ValueError("developer workspace must be an absolute path")
        elif value or self.wslg:
            raise ValueError(
                "disabled developer attachment cannot specify workspace/WSLg"
            )
        return self

    @property
    def workspace_path(self) -> Path | None:
        if not self.enabled:
            return None
        return Path(self.workspace).expanduser().resolve()


@dataclass(frozen=True)
class InstallState:
    profile: str
    roles: tuple[str, ...]
    prefix: str
    bin_dir: str
    source_root: str
    source_repository: str = DEFAULT_SOURCE_REPOSITORY
    source_ref: str = DEFAULT_SOURCE_REF
    network: NetworkSettings = field(default_factory=NetworkSettings)
    dds: DdsSettings = field(default_factory=DdsSettings)
    compute: ComputeSettings = field(default_factory=ComputeSettings)
    turn: TurnSettings = field(default_factory=TurnSettings)
    runtime_text_logs: RuntimeTextLogSettings = field(
        default_factory=RuntimeTextLogSettings
    )
    container_network: ContainerNetworkSettings = field(
        default_factory=ContainerNetworkSettings
    )
    developer_attachment: DeveloperAttachmentSettings = field(
        default_factory=DeveloperAttachmentSettings
    )
    install_mode: str = "container"
    install_go2_mpc: bool = True
    schema_version: int = STATE_SCHEMA_VERSION
    # None preserves the standalone install's all-role launch behavior.
    # A manager assignment selects from installed roles without removing them.
    assigned_roles: tuple[str, ...] | None = None

    @property
    def runtime_roles(self) -> tuple[str, ...]:
        return self.roles if self.assigned_roles is None else self.assigned_roles

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
                f"installation state schema {self.schema_version!r} is unsupported; "
                f"expected {STATE_SCHEMA_VERSION}"
            )
        roles = normalize_roles(self.roles)
        if self.assigned_roles is not None:
            assigned = self.assigned_roles
            if (
                not isinstance(assigned, tuple)
                or not assigned
                or any(not isinstance(role, str) for role in assigned)
                or len(set(assigned)) != len(assigned)
                or not set(assigned).issubset(roles)
            ):
                raise ValueError("assigned_roles must be a nonempty subset of installed roles")
        if not self.prefix.strip() or not self.bin_dir.strip() or not self.source_root.strip():
            raise ValueError("prefix, bin_dir, and source_root are required")
        _validate_source_identity(self.source_repository, name="source_repository")
        _validate_source_identity(self.source_ref, name="source_ref")
        self.network.validate()
        self.dds.validate()
        self.compute.validate()
        self.turn.validate()
        self.runtime_text_logs.validate()
        self.container_network.validate()
        self.developer_attachment.validate()
        if self.install_mode not in INSTALL_MODES:
            raise ValueError(f"Unsupported installation mode: {self.install_mode!r}")
        if "robot" in roles and roles != ("robot",):
            raise ValueError(
                "native Robot installation must be standalone and separate from other roles"
            )
        if roles == ("robot",) and self.install_mode != "native":
            raise ValueError(
                "Robot Jetson cannot use a generic Ubuntu container. "
                "Use native installation on a Jetson with JetPack/L4T, ROS2, "
                "and unitree_ros2 installed"
            )
        if roles != ("robot",) and self.install_mode != "container":
            raise ValueError(
                "Sim, Pilot, and UI support Docker/Compose only; native installation is "
                "for standalone Robot Jetson hosts"
            )
        if self.developer_attachment.enabled and self.install_mode != "container":
            raise ValueError("developer attachment can only be added to a container installation")
        if (
            self.container_network.uses_tailscale_sidecar
            and self.install_mode != "container"
        ):
            raise ValueError("Tailscale sidecar requires a container installation")
        if self.container_network.uses_tailscale_sidecar:
            expected_state = self.prefix_path / "secrets/tailscale"
            if self.container_network.tailscale_state_path != expected_state:
                raise ValueError(
                    "Tailscale sidecar state directory must be under the installation prefix: "
                    f"exact path is required: {expected_state}"
                )
        has_turn_urls = bool(self.network.turn_urls)
        if self.turn.mode == "none" and has_turn_urls:
            raise ValueError("TURN URL requires managed or external TURN mode")
        if self.turn.mode != "none" and not has_turn_urls:
            if not self.managed_turn_pending:
                raise ValueError(f"{self.turn.mode} TURN mode requires a TURN URL")
        if (
            self.turn.mode == "managed"
            and has_turn_urls
            and not self.turn.public_host.strip()
        ):
            raise ValueError("configured managed TURN requires a public host")
        if (
            self.turn.mode == "managed"
            and not has_turn_urls
            and self.turn.public_host.strip()
        ):
            raise ValueError("pending managed TURN cannot specify a public host")
        if self.turn.managed and "sim" not in self.roles:
            raise ValueError(
                "managed Coturn is only available on a host installing Sim"
            )
        if self.turn.managed and self.install_mode != "container":
            raise ValueError("managed Coturn lifecycle requires a container installation")
        if self.turn.managed and self.dds.security_profile != "sros2":
            raise ValueError(
                "managed TURN credentials and WebRTC signaling require the sros2 profile"
            )
        if (
            self.turn.mode == "external"
            and self.turn.credential_path is not None
            and "sim" not in self.roles
        ):
            raise ValueError(
                "an external TURN credential file can only be deployed to the Sim host"
            )
        return self

    @property
    def managed_turn_pending(self) -> bool:
        """Whether the Sim-owned relay awaits a manager-selected endpoint."""

        return (
            self.turn.mode == "managed"
            and "sim" in self.roles
            and not self.network.turn_urls
            and not self.turn.public_host.strip()
        )

    def require_installable_dds(self) -> "InstallState":
        """Validate artifacts that can be generated before managed provisioning."""

        self.validate()
        if self.dds.discovery_mode == "static" and not self.dds.static_peers:
            raise ValueError(
                "static DDS discovery requires at least one peer. "
                "Router addresses from legacy ZMQ state are not reused as peers"
            )
        if self.dds.migrated_security_needs_configuration:
            raise ValueError(
                "legacy Curve state cannot be converted automatically to SROS2 keys. "
                "specify the SROS2 keystore and enclave"
            )
        if (
            self.turn.mode == "external"
            and "sim" in self.roles
            and self.turn.credential_path is None
        ):
            raise ValueError(
                "Sim external TURN requires a username/credential JSON file. "
                "Re-specify the TURN credential path for legacy state"
            )
        return self

    def require_runnable_dds(self) -> "InstallState":
        """Fail closed unless this state has usable DDS runtime credentials."""

        self.require_installable_dds()
        if self.dds.managed_security_pending:
            raise ValueError(
                "managed SROS2 role bundle has not been provisioned. "
                "run elesim-connections on the operator laptop"
            )
        return self

    def to_dict(self) -> dict[str, Any]:
        raw = asdict(self)
        raw["roles"] = list(self.roles)
        raw["assigned_roles"] = (
            None if self.assigned_roles is None else list(self.assigned_roles)
        )
        raw["network"]["turn_urls"] = list(self.network.turn_urls)
        raw["dds"]["static_peers"] = list(self.dds.static_peers)
        return raw

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "InstallState":
        source_schema = int(raw.get("schema_version", 0))
        if source_schema not in SUPPORTED_STATE_SCHEMAS:
            raise ValueError(
                f"installation state schema {source_schema!r} is unsupported; "
                f"expected one of {sorted(SUPPORTED_STATE_SCHEMAS)}"
            )
        network_raw = raw.get("network", {})
        compute_raw = raw.get("compute", {})
        turn_raw = raw.get("turn", {})
        dds_raw = raw.get("dds", {})
        runtime_text_logs_raw = raw.get("runtime_text_logs", {})
        container_network_raw = raw.get("container_network", {})
        developer_attachment_raw = raw.get("developer_attachment", {})
        if not all(
            isinstance(value, Mapping)
            for value in (
                network_raw,
                compute_raw,
                turn_raw,
                dds_raw,
                runtime_text_logs_raw,
                container_network_raw,
                developer_attachment_raw,
            )
        ):
            raise ValueError(
                "installation state network/dds/compute/turn/runtime_text_logs/"
                "container_network/developer_attachment must be objects"
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
                raise ValueError("legacy security in installation state must be an object")
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
        assigned_raw = raw.get("assigned_roles") if source_schema >= 11 else None
        if assigned_raw is not None and not isinstance(assigned_raw, (list, tuple)):
            raise ValueError("assigned_roles must be an array or null")
        return cls(
            profile=str(raw.get("profile", "custom")),
            roles=roles,
            assigned_roles=None if assigned_raw is None else tuple(assigned_raw),
            prefix=str(raw.get("prefix", "")),
            bin_dir=str(raw.get("bin_dir", "")),
            source_root=str(raw.get("source_root", "")),
            source_repository=str(
                raw.get("source_repository", DEFAULT_SOURCE_REPOSITORY)
            ),
            source_ref=str(raw.get("source_ref", DEFAULT_SOURCE_REF)),
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
            container_network=(
                ContainerNetworkSettings()
                if source_schema < 9
                else ContainerNetworkSettings(**dict(container_network_raw))
            ),
            developer_attachment=(
                DeveloperAttachmentSettings()
                if source_schema < 10
                else DeveloperAttachmentSettings(**dict(developer_attachment_raw))
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
            raise ValueError(f"{source}: installation state must be a JSON object")
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
        raise ValueError(f"{name} is not a hostname or IP: {host!r}")
    try:
        address = ipaddress.ip_address(unbracketed)
    except ValueError:
        if ":" in unbracketed:
            raise ValueError(f"{name} must not include a port: {host!r}")
        return
    if address.is_unspecified:
        raise ValueError(f"{name} does not accept bind-only address {value!r}")


def _validate_identifier(value: object, *, name: str) -> None:
    text = str(value).strip()
    if not text or len(text) > 128 or any(character.isspace() for character in text):
        raise ValueError(
            f"{name} must be a whitespace-free value of 1..128 characters"
        )


def _validate_source_identity(value: object, *, name: str) -> None:
    text = _bounded_single_line(value, name=name, maximum=255)
    if not text or any(character.isspace() for character in text):
        raise ValueError(
            f"{name} must be a whitespace-free value of 1..255 characters"
        )


def _bounded_single_line(value: object, *, name: str, maximum: int) -> str:
    text = str(value).strip()
    if len(text) > maximum or "\n" in text or "\r" in text or "\x00" in text:
        raise ValueError(
            f"{name} must be a single-line string of at most {maximum} characters"
        )
    return text


__all__ = [
    "DDS_DISCOVERY_MODES",
    "DDS_RMW_IMPLEMENTATIONS",
    "DDS_SECURITY_PROFILES",
    "DDS_SECURITY_PROVISIONING",
    "DEFAULT_BIN_DIR",
    "DEFAULT_PREFIX",
    "DEFAULT_SOURCE_REF",
    "DEFAULT_SOURCE_REPOSITORY",
    "ComputeSettings",
    "CONTAINER_NETWORK_MODES",
    "ContainerNetworkSettings",
    "DeveloperAttachmentSettings",
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
