"""Persistent, non-secret topology for the Elesim connection manager.

Each host has one advertised IP.  That value is used both as the DDS address
and as the SSH destination; SSH keeps its own port, user, authentication mode,
and host-key fingerprint because those are management settings rather than DDS
settings.
"""

from __future__ import annotations

import ipaddress
import json
import os
import re
import stat
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path, PurePosixPath
from typing import Any, Mapping, Sequence


CONNECTION_SCHEMA_VERSION = 3
LEGACY_CONNECTION_SCHEMA_VERSION = 1
PREVIOUS_CONNECTION_SCHEMA_VERSION = 2
PREFLIGHT_SCHEMA_VERSION = 1
ROLES = ("pilot", "sim", "ui", "robot")
SIMULATION_ROLES = ("pilot", "sim", "ui")
TOPOLOGY_MODES = frozenset({"full", "simulation-only"})
SECURITY_PROFILES = frozenset({"trusted-network", "sros2"})
SSH_AUTH_MODES = frozenset({"openssh", "tailscale"})
INSTALL_MODES = frozenset({"container", "native"})
LIFECYCLES = frozenset({"compose", "systemd"})
DDS_DISCOVERY_MODES = frozenset({"multicast", "static"})
DDS_RMW_IMPLEMENTATIONS = frozenset({"rmw_cyclonedds_cpp"})
MAX_CONNECTION_FILE_BYTES = 1024 * 1024

_SYSTEM_ID = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
_STABLE_ID = re.compile(r"^[a-z][a-z0-9_-]{0,62}$")
_ENDPOINT_ID = re.compile(r"^[a-z][a-z0-9_-]{0,127}$")
_FINGERPRINT = re.compile(r"^SHA256:[A-Za-z0-9+/]{43}$")
_FORBIDDEN_SECRET_KEYS = (
    "password",
    "passphrase",
    "private_key",
    "privatekey",
    "secret",
    "credential",
    "token",
)


@dataclass(frozen=True)
class DdsEndpoint:
    address: str
    interface: str
    # Non-secret provenance hint.  It is advisory only: addresses are always
    # entered from the host's current state and never hard-coded by setup.
    address_source: str = "manual"

    def validate(self) -> "DdsEndpoint":
        _validate_network_host(
            self.address,
            name="DDS address",
            reject_loopback=True,
            reject_multicast=True,
        )
        interface = _plain_text(self.interface, name="DDS interface", maximum=128)
        if any(character.isspace() for character in interface) or "/" in interface:
            raise ValueError("DDS interface must be an interface name, not a path")
        if self.address_source not in {"manual", "tailscale"}:
            raise ValueError("DDS address_source must be manual or tailscale")
        if self.address_source == "tailscale" and interface != "tailscale0":
            raise ValueError("a tailscale DDS address must bind tailscale0")
        return self

    def to_dict(self) -> dict[str, str]:
        self.validate()
        result = {"address": self.address, "interface": self.interface}
        if self.address_source != "manual":
            result["address_source"] = self.address_source
        return result

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "DdsEndpoint":
        if not isinstance(raw, Mapping):
            raise ValueError("dds must be an object")
        values = _strict_object(
            raw,
            required={"address", "interface"},
            optional={"address_source"},
            name="dds",
        )
        return cls(
            address=_required_string(values["address"], name="dds.address"),
            interface=_required_string(values["interface"], name="dds.interface"),
            address_source=_optional_string(
                values.get("address_source", "manual"), name="dds.address_source"
            ) or "manual",
        ).validate()


@dataclass(frozen=True)
class DdsGraphSettings:
    domain_id: int = 0
    rmw_implementation: str = "rmw_cyclonedds_cpp"
    discovery_mode: str = "multicast"

    def validate(self) -> "DdsGraphSettings":
        if isinstance(self.domain_id, bool) or not 0 <= int(self.domain_id) <= 232:
            raise ValueError("DDS domain_id must be in 0..232")
        if self.rmw_implementation not in DDS_RMW_IMPLEMENTATIONS:
            raise ValueError(
                f"unsupported DDS RMW implementation: {self.rmw_implementation!r}"
            )
        if self.discovery_mode not in DDS_DISCOVERY_MODES:
            raise ValueError(
                f"unsupported DDS discovery mode: {self.discovery_mode!r}"
            )
        return self

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return {
            "domain_id": int(self.domain_id),
            "rmw_implementation": self.rmw_implementation,
            "discovery_mode": self.discovery_mode,
        }

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "DdsGraphSettings":
        values = _strict_object(
            raw,
            required={"domain_id", "rmw_implementation", "discovery_mode"},
            name="dds_graph",
        )
        return cls(
            domain_id=_required_integer(values["domain_id"], name="dds_graph.domain_id"),
            rmw_implementation=_required_string(
                values["rmw_implementation"], name="dds_graph.rmw_implementation"
            ),
            discovery_mode=_required_string(
                values["discovery_mode"], name="dds_graph.discovery_mode"
            ),
        ).validate()


@dataclass(frozen=True)
class SshEndpoint:
    host: str
    port: int
    user: str
    identity_file: str
    pinned_fingerprint: str
    # ``openssh`` uses the local agent or an explicitly selected key.  The
    # ``tailscale`` mode speaks Tailscale SSH directly: it has no private-key
    # file and is restricted to Tailscale's port 22 endpoint.
    auth_mode: str = "openssh"

    def validate(self) -> "SshEndpoint":
        _validate_network_host(self.host, name="SSH host")
        if isinstance(self.port, bool) or not 1 <= int(self.port) <= 65535:
            raise ValueError("SSH port must be in 1..65535")
        user = _plain_text(self.user, name="SSH user", maximum=128)
        if any(character.isspace() for character in user):
            raise ValueError("SSH user must not contain whitespace")
        identity = str(self.identity_file)
        if len(identity) > 4096 or "\x00" in identity or "\n" in identity or "\r" in identity:
            raise ValueError("SSH identity_file must be a path, not key contents")
        if (
            not isinstance(self.auth_mode, str)
            or self.auth_mode not in SSH_AUTH_MODES
        ):
            raise ValueError(f"unsupported SSH auth_mode: {self.auth_mode!r}")
        if self.auth_mode == "tailscale":
            if int(self.port) != 22:
                raise ValueError("Tailscale SSH uses port 22")
            if identity.strip():
                raise ValueError("Tailscale SSH must not use a private-key file")
        fingerprint = str(self.pinned_fingerprint).strip()
        if not _FINGERPRINT.fullmatch(fingerprint):
            raise ValueError("SSH pinned_fingerprint must be a SHA256 host-key fingerprint")
        return self

    @property
    def uses_agent(self) -> bool:
        return self.auth_mode == "openssh" and not self.identity_file.strip()

    @property
    def uses_tailscale_ssh(self) -> bool:
        return self.auth_mode == "tailscale"

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return {
            "host": self.host,
            "port": int(self.port),
            "user": self.user,
            "identity_file": self.identity_file,
            "pinned_fingerprint": self.pinned_fingerprint,
            "auth_mode": self.auth_mode,
        }

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "SshEndpoint":
        values = _strict_object(
            raw,
            required={"host", "port", "user", "identity_file", "pinned_fingerprint"},
            optional={"auth_mode"},
            name="ssh",
        )
        return cls(
            host=_required_string(values["host"], name="ssh.host"),
            port=_required_integer(values["port"], name="ssh.port"),
            user=_required_string(values["user"], name="ssh.user"),
            identity_file=_optional_string(
                values["identity_file"], name="ssh.identity_file"
            ),
            pinned_fingerprint=_required_string(
                values["pinned_fingerprint"], name="ssh.pinned_fingerprint"
            ),
            auth_mode=_optional_string(
                values.get("auth_mode", "openssh"), name="ssh.auth_mode"
            ) or "openssh",
        ).validate()


@dataclass(frozen=True)
class PreflightSshEndpoint:
    """Non-secret SSH target used by the Jetson-less endpoint preflight.

    The preflight deliberately carries no identity path or pinned fingerprint:
    it only checks that the management target derived from the advertised DDS
    IP is reachable and, when requested, asks the existing host-key probe to
    reach it.  The
    full :class:`SshEndpoint` remains mandatory for a saved/deployable
    topology, so this type cannot accidentally weaken rollout pinning.
    """

    host: str
    port: int
    user: str
    auth_mode: str = "openssh"

    def validate(self) -> "PreflightSshEndpoint":
        _validate_network_host(self.host, name="preflight SSH host")
        if isinstance(self.port, bool) or not 1 <= int(self.port) <= 65535:
            raise ValueError("preflight SSH port must be in 1..65535")
        user = _plain_text(self.user, name="preflight SSH user", maximum=128)
        if any(character.isspace() for character in user):
            raise ValueError("preflight SSH user must not contain whitespace")
        if self.auth_mode not in SSH_AUTH_MODES:
            raise ValueError(f"unsupported preflight SSH auth_mode: {self.auth_mode!r}")
        if self.auth_mode == "tailscale" and int(self.port) != 22:
            raise ValueError("Tailscale SSH preflight uses port 22")
        return self

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        result = {"host": self.host, "port": int(self.port), "user": self.user}
        if self.auth_mode != "openssh":
            result["auth_mode"] = self.auth_mode
        return result

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "PreflightSshEndpoint":
        values = _strict_object(
            raw,
            required={"host", "port", "user"},
            optional={"auth_mode"},
            name="preflight ssh",
        )
        return cls(
            host=_required_string(values["host"], name="preflight.ssh.host"),
            port=_required_integer(values["port"], name="preflight.ssh.port"),
            user=_required_string(values["user"], name="preflight.ssh.user"),
            auth_mode=_optional_string(
                values.get("auth_mode", "openssh"), name="preflight.ssh.auth_mode"
            )
            or "openssh",
        ).validate()


@dataclass(frozen=True)
class PreflightHost:
    """A role-neutral host endpoint for a two-computer connectivity check."""

    host_id: str
    local: bool
    dds: DdsEndpoint
    ssh: PreflightSshEndpoint | None

    def __post_init__(self) -> None:
        if self.ssh is not None:
            object.__setattr__(self, "ssh", replace(self.ssh, host=self.dds.address))

    def validate(self) -> "PreflightHost":
        if not _STABLE_ID.fullmatch(str(self.host_id)):
            raise ValueError("preflight host_id must be a stable lower-case identifier")
        if not isinstance(self.local, bool):
            raise ValueError("preflight host.local must be boolean")
        self.dds.validate()
        if self.local:
            if self.ssh is not None:
                raise ValueError("the local preflight host must not use SSH")
        elif self.ssh is None:
            raise ValueError("every remote preflight host requires SSH host and port")
        else:
            self.ssh.validate()
            if self.ssh.host != self.dds.address:
                raise ValueError(
                    "preflight SSH IP is derived from the DDS IP; do not configure a separate host"
                )
        return self

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        ssh = None if self.ssh is None else replace(self.ssh, host=self.dds.address)
        return {
            "id": self.host_id,
            "local": self.local,
            "dds": self.dds.to_dict(),
            "ssh": None if ssh is None else ssh.to_dict(),
        }

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "PreflightHost":
        values = _strict_object(
            raw,
            required={"id", "local", "dds", "ssh"},
            # Older ephemeral probe payloads carried a user-editable label.
            # Read and discard it so an existing browser payload remains
            # loadable, but never retain or emit it.
            optional={"display_name"},
            name="preflight host",
        )
        if not isinstance(values["local"], bool):
            raise ValueError("preflight host.local must be boolean")
        if not isinstance(values["dds"], Mapping):
            raise ValueError("preflight host.dds must be an object")
        ssh_raw = values["ssh"]
        if ssh_raw is not None and not isinstance(ssh_raw, Mapping):
            raise ValueError("preflight host.ssh must be an object or null")
        dds = DdsEndpoint.from_dict(values["dds"])
        ssh = (
            None
            if ssh_raw is None
            else PreflightSshEndpoint.from_dict(ssh_raw)
        )
        return cls(
            host_id=_required_string(values["id"], name="preflight host.id"),
            local=values["local"],
            dds=dds,
            ssh=ssh,
        ).validate()


@dataclass(frozen=True)
class TwoHostPreflight:
    """Ephemeral, role-neutral endpoint contract for M2-A.

    This is intentionally not a :class:`ConnectionTopology`: it permits the
    current laptop/computer check while the physical Robot is unavailable, but
    it cannot be saved or passed to a deployment runner.
    """

    hosts: tuple[PreflightHost, ...]
    discovery_mode: str = "static"
    schema_version: int = PREFLIGHT_SCHEMA_VERSION

    def validate(self) -> "TwoHostPreflight":
        if self.schema_version != PREFLIGHT_SCHEMA_VERSION:
            raise ValueError(
                f"unsupported preflight schema {self.schema_version!r}; "
                f"expected {PREFLIGHT_SCHEMA_VERSION}"
            )
        if self.discovery_mode not in DDS_DISCOVERY_MODES:
            raise ValueError(
                f"unsupported preflight discovery mode: {self.discovery_mode!r}"
            )
        if len(self.hosts) != 2:
            raise ValueError("the endpoint preflight requires exactly two hosts")
        for host in self.hosts:
            host.validate()
        ids = [host.host_id for host in self.hosts]
        if len(set(ids)) != len(ids):
            raise ValueError("preflight host IDs must be unique")
        addresses = [host.dds.address.casefold() for host in self.hosts]
        if len(set(addresses)) != len(addresses):
            raise ValueError("preflight DDS addresses must be unique")
        if sum(host.local for host in self.hosts) != 1:
            raise ValueError("the endpoint preflight requires exactly one local host")
        return self

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return {
            "schema_version": self.schema_version,
            "discovery_mode": self.discovery_mode,
            "hosts": [host.to_dict() for host in self.hosts],
        }

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "TwoHostPreflight":
        _reject_secret_fields(raw)
        values = _strict_object(
            raw,
            required={"schema_version", "discovery_mode", "hosts"},
            name="two-host preflight",
        )
        hosts_raw = _object_sequence(values["hosts"], name="preflight hosts")
        return cls(
            schema_version=_required_integer(
                values["schema_version"], name="preflight schema_version"
            ),
            discovery_mode=_required_string(
                values["discovery_mode"], name="preflight discovery_mode"
            ),
            hosts=tuple(PreflightHost.from_dict(item) for item in hosts_raw),
        ).validate()

    def discovery_peers(self, host_id: str) -> tuple[str, ...]:
        self.validate()
        if not any(host.host_id == host_id for host in self.hosts):
            raise KeyError(host_id)
        if self.discovery_mode == "multicast":
            return ()
        return tuple(host.dds.address for host in self.hosts if host.host_id != host_id)


@dataclass(frozen=True)
class RoleAssignment:
    role: str
    endpoint_id: str

    def validate(self) -> "RoleAssignment":
        if self.role not in ROLES:
            raise ValueError(f"unknown role: {self.role!r}")
        if not _ENDPOINT_ID.fullmatch(str(self.endpoint_id)):
            raise ValueError(
                "endpoint_id must start with lower-case a-z and contain only "
                "lower-case letters, digits, '_' or '-'"
            )
        return self

    def to_dict(self) -> dict[str, str]:
        self.validate()
        return {"role": self.role, "endpoint_id": self.endpoint_id}

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "RoleAssignment":
        values = _strict_object(raw, required={"role", "endpoint_id"}, name="role")
        legacy_roles = {"controller": "pilot", "simulator": "sim"}
        role = _required_string(values["role"], name="role.role").lower()
        role = legacy_roles.get(role, role)
        endpoint_id = _required_string(values["endpoint_id"], name="role.endpoint_id")
        endpoint_id = endpoint_id.replace("controller-", "pilot-", 1).replace(
            "simulator-", "sim-", 1
        )
        return cls(
            role=role,
            endpoint_id=endpoint_id,
        ).validate()


@dataclass(frozen=True)
class DeploymentUnit:
    """One independently installed/lifecycle-managed unit on a host.

    A physical computer is not an installation boundary.  In particular, a
    Jetson may carry the mandatory native Robot unit and a separate Compose
    unit for Pilot/UI.  Both units use the host's shared installation and
    command paths; only their lifecycle/install mode differs.
    """

    unit_id: str
    assignments: tuple[RoleAssignment, ...]
    install_mode: str = "container"
    install_root: str = "/opt/elesim"
    bin_dir: str = "/usr/local/bin"
    lifecycle: str = "compose"

    def validate(self, *, jetson: bool) -> "DeploymentUnit":
        if not _STABLE_ID.fullmatch(str(self.unit_id)):
            raise ValueError("unit_id must be a stable lower-case identifier")
        if self.install_mode not in INSTALL_MODES:
            raise ValueError(f"unsupported unit install_mode: {self.install_mode!r}")
        if self.lifecycle not in LIFECYCLES:
            raise ValueError(f"unsupported unit lifecycle: {self.lifecycle!r}")
        _validate_absolute_posix_path(self.install_root, name="unit.install_root")
        _validate_absolute_posix_path(self.bin_dir, name="unit.bin_dir")
        if not self.assignments:
            raise ValueError("every deployment unit must own at least one role")
        roles = [assignment.validate().role for assignment in self.assignments]
        if len(set(roles)) != len(roles):
            raise ValueError("a deployment unit assigns one role more than once")
        if "robot" in roles:
            if roles != ["robot"]:
                raise ValueError("the native Robot unit must contain only Robot")
            if self.install_mode != "native" or not jetson:
                raise ValueError("Robot requires native installation on a Jetson host")
            if self.lifecycle != "systemd":
                raise ValueError("Robot requires the native systemd lifecycle")
        elif self.install_mode != "container" or self.lifecycle != "compose":
            raise ValueError("Pilot, Sim and UI units require container/Compose")
        elif jetson and "sim" in roles:
            raise ValueError(
                "Sim is currently amd64-only and cannot be assigned to a Jetson "
                "until an ARM64 image is validated"
            )
        return self

    @property
    def roles(self) -> tuple[str, ...]:
        return tuple(assignment.role for assignment in self.assignments)

    def to_dict(self) -> dict[str, Any]:
        return {
            "id": self.unit_id,
            "assignments": [assignment.to_dict() for assignment in self.assignments],
            "install_mode": self.install_mode,
            "install_root": self.install_root,
            "bin_dir": self.bin_dir,
            "lifecycle": self.lifecycle,
        }

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "DeploymentUnit":
        values = _strict_object(
            raw,
            required={"id", "assignments", "install_mode", "install_root", "bin_dir", "lifecycle"},
            name="deployment unit",
        )
        assignments_raw = _object_sequence(values["assignments"], name="unit.assignments")
        return cls(
            unit_id=_required_string(values["id"], name="unit.id"),
            assignments=tuple(RoleAssignment.from_dict(item) for item in assignments_raw),
            install_mode=_required_string(values["install_mode"], name="unit.install_mode"),
            install_root=_required_string(values["install_root"], name="unit.install_root"),
            bin_dir=_required_string(values["bin_dir"], name="unit.bin_dir"),
            lifecycle=_required_string(values["lifecycle"], name="unit.lifecycle"),
        )


@dataclass(frozen=True, init=False)
class ManagedHost:
    host_id: str
    local: bool
    dds: DdsEndpoint
    ssh: SshEndpoint | None
    units: tuple[DeploymentUnit, ...]
    jetson: bool = False

    def __init__(
        self,
        host_id: str,
        local: bool,
        dds: DdsEndpoint,
        ssh: SshEndpoint | None,
        assignments: Sequence[RoleAssignment] | None = None,
        install_mode: str = "container",
        jetson: bool = False,
        install_root: str = "/opt/elesim",
        bin_dir: str = "/usr/local/bin",
        lifecycle: str = "compose",
        units: Sequence[DeploymentUnit] | None = None,
    ) -> None:
        if units is None:
            legacy_assignments = tuple(assignments or ())
            unit_id = "robot-native" if install_mode == "native" else "runtime"
            units = (
                DeploymentUnit(
                    unit_id=unit_id,
                    assignments=legacy_assignments,
                    install_mode=install_mode,
                    install_root=install_root,
                    bin_dir=bin_dir,
                    lifecycle=lifecycle,
                ),
            )
        else:
            units = tuple(units)
            if assignments:
                flattened = tuple(
                    assignment
                    for unit in units
                    for assignment in unit.assignments
                )
                if tuple(assignments) != flattened:
                    raise ValueError("assignments and units disagree")
        if ssh is not None:
            ssh = replace(ssh, host=dds.address)
        if units:
            # Installation and command paths are host-level fields in the GUI.
            # Normalize legacy mixed-unit records here so an old Robot-specific
            # path cannot survive as a hidden second configuration.
            path_unit = next(
                (unit for unit in units if unit.install_mode == "container"),
                units[0],
            )
            units = tuple(
                replace(
                    unit,
                    install_root=path_unit.install_root,
                    bin_dir=path_unit.bin_dir,
                )
                for unit in units
            )
        object.__setattr__(self, "host_id", host_id)
        object.__setattr__(self, "local", local)
        object.__setattr__(self, "dds", dds)
        object.__setattr__(self, "ssh", ssh)
        object.__setattr__(self, "units", tuple(units))
        object.__setattr__(self, "jetson", jetson)

    @property
    def assignments(self) -> tuple[RoleAssignment, ...]:
        return tuple(
            assignment
            for unit in self.units
            for assignment in unit.assignments
        )

    @property
    def roles(self) -> tuple[str, ...]:
        return tuple(assignment.role for assignment in self.assignments)

    @property
    def runtime_units(self) -> tuple[DeploymentUnit, ...]:
        return tuple(unit for unit in self.units if unit.install_mode == "container")

    @property
    def robot_units(self) -> tuple[DeploymentUnit, ...]:
        return tuple(unit for unit in self.units if "robot" in unit.roles)

    @property
    def primary_unit(self) -> DeploymentUnit:
        if not self.units:
            raise ValueError("managed host has no deployment units")
        return self.runtime_units[0] if self.runtime_units else self.units[0]

    # These compatibility properties keep older lifecycle/status code and
    # schema-v3 callers readable while all new serialization is unit-based.
    @property
    def install_mode(self) -> str:
        return self.primary_unit.install_mode

    @property
    def install_root(self) -> str:
        return self.primary_unit.install_root

    @property
    def bin_dir(self) -> str:
        return self.primary_unit.bin_dir

    @property
    def lifecycle(self) -> str:
        return self.primary_unit.lifecycle

    def validate(self) -> "ManagedHost":
        if not _STABLE_ID.fullmatch(str(self.host_id)):
            raise ValueError("host_id must be a stable lower-case identifier")
        if not isinstance(self.local, bool):
            raise ValueError("host.local must be boolean")
        self.dds.validate()
        if self.local:
            if self.ssh is not None:
                raise ValueError("the local host must not use its SSH endpoint")
        elif self.ssh is None:
            raise ValueError("every remote host requires an explicit SSH endpoint")
        else:
            self.ssh.validate()
            if self.ssh.host != self.dds.address:
                raise ValueError(
                    "SSH IP is derived from the DDS IP; do not configure a separate host"
                )
        if not isinstance(self.jetson, bool):
            raise ValueError("host.jetson must be boolean")
        if not self.units:
            raise ValueError("every managed host must own at least one deployment unit")
        unit_ids = [unit.unit_id for unit in self.units]
        if len(set(unit_ids)) != len(unit_ids):
            raise ValueError(f"host {self.host_id!r} has duplicate deployment unit IDs")
        roles = [assignment.role for assignment in self.assignments]
        if len(set(roles)) != len(roles):
            raise ValueError(f"host {self.host_id!r} assigns one role more than once")
        if self.jetson and "robot" not in roles:
            raise ValueError(
                "a Jetson host must include the mandatory native Robot unit; "
                "place Pilot/UI in a separate container unit if needed"
            )
        for unit in self.units:
            unit.validate(jetson=self.jetson)
        if len(self.runtime_units) > 1:
            raise ValueError(
                "a host may have only one container/Compose unit because the "
                "runtime project name is fixed to elesim-runtime"
            )
        # Installation and command paths belong to the host, not to a role.
        # A Jetson may therefore expose one shared pair of paths to its native
        # Robot unit and its optional Compose unit.  The unit records retain
        # their lifecycle/install mode, but no longer impose artificial path
        # separation between them.
        return self

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        result: dict[str, Any] = {
            "id": self.host_id,
            "local": self.local,
            "dds": self.dds.to_dict(),
            "ssh": None if self.ssh is None else self.ssh.to_dict(),
            "jetson": self.jetson,
            "units": [unit.to_dict() for unit in self.units],
        }
        # Keep a read-compatible mirror for homogeneous schema-v3 files.  It
        # is intentionally derived, never consumed by new code, and can be
        # removed when the persistent topology schema is next bumped.
        if len(self.units) == 1:
            unit = self.units[0]
            result.update(
                {
                    "assignments": [assignment.to_dict() for assignment in self.assignments],
                    "install_mode": unit.install_mode,
                    "install_root": unit.install_root,
                    "bin_dir": unit.bin_dir,
                    "lifecycle": unit.lifecycle,
                }
            )
        return result

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "ManagedHost":
        values = _strict_object(
            raw,
            required={"id", "local", "dds", "ssh", "jetson"},
            optional={
                # Schema-v1..v3 files may still contain the retired label.
                # It is deliberately ignored during migration and is never
                # represented by ManagedHost or written back out.
                "display_name",
                "units",
                "assignments",
                "install_mode",
                "install_root",
                "bin_dir",
                "lifecycle",
            },
            name="host",
        )
        if not isinstance(values["local"], bool) or not isinstance(values["jetson"], bool):
            raise ValueError("host.local and host.jetson must be boolean")
        ssh_raw = values["ssh"]
        if ssh_raw is not None and not isinstance(ssh_raw, Mapping):
            raise ValueError("host.ssh must be an object or null")
        if not isinstance(values["dds"], Mapping):
            raise ValueError("host.dds must be an object")
        units_raw = values.get("units")
        if units_raw is not None:
            units = tuple(
                DeploymentUnit.from_dict(item)
                for item in _object_sequence(units_raw, name="units")
            )
            # Old callers sometimes edit the mirrored fields in a loaded
            # object.  Honour that explicit legacy edit only when it differs
            # from the canonical unit representation.
            if "assignments" in values:
                legacy_assignments = tuple(
                    RoleAssignment.from_dict(item)
                    for item in _object_sequence(values["assignments"], name="assignments")
                )
                canonical = tuple(
                    assignment for unit in units for assignment in unit.assignments
                )
                legacy_fields = {
                    "install_mode": values.get("install_mode"),
                    "install_root": values.get("install_root"),
                    "bin_dir": values.get("bin_dir"),
                    "lifecycle": values.get("lifecycle"),
                }
                unit = units[0] if len(units) == 1 else None
                differs = legacy_assignments != canonical or (
                    unit is not None
                    and any(legacy_fields[key] != getattr(unit, key) for key in legacy_fields)
                )
                if differs:
                    units = (
                        DeploymentUnit(
                            unit_id=(unit.unit_id if unit is not None else "runtime"),
                            assignments=legacy_assignments,
                            install_mode=_required_string(
                                values["install_mode"], name="install_mode"
                            ),
                            install_root=_required_string(
                                values["install_root"], name="install_root"
                            ),
                            bin_dir=_required_string(
                                values["bin_dir"], name="bin_dir"
                            ),
                            lifecycle=_required_string(
                                values["lifecycle"], name="lifecycle"
                            ),
                        ),
                    )
        else:
            required_legacy = {
                "assignments",
                "install_mode",
                "install_root",
                "bin_dir",
                "lifecycle",
            }
            missing = sorted(key for key in required_legacy if key not in values)
            if missing:
                raise ValueError(
                    "host is missing deployment units and legacy fields: "
                    + ", ".join(missing)
                )
            assignments_raw = _object_sequence(values["assignments"], name="assignments")
            units = (
                DeploymentUnit(
                    unit_id="robot-native" if values["install_mode"] == "native" else "runtime",
                    assignments=tuple(RoleAssignment.from_dict(item) for item in assignments_raw),
                    install_mode=_required_string(values["install_mode"], name="install_mode"),
                    install_root=_required_string(values["install_root"], name="install_root"),
                    bin_dir=_required_string(values["bin_dir"], name="bin_dir"),
                    lifecycle=_required_string(values["lifecycle"], name="lifecycle"),
                ),
            )
        return cls(
            host_id=_required_string(values["id"], name="host.id"),
            local=values["local"],
            dds=DdsEndpoint.from_dict(values["dds"]),
            ssh=None if ssh_raw is None else SshEndpoint.from_dict(ssh_raw),
            jetson=values["jetson"],
            units=units,
        ).validate()


@dataclass(frozen=True)
class ConnectionTopology:
    system_id: str
    security_profile: str
    hosts: tuple[ManagedHost, ...]
    dds_graph: DdsGraphSettings = DdsGraphSettings()
    schema_version: int = CONNECTION_SCHEMA_VERSION
    topology_mode: str = "full"

    def validate(self) -> "ConnectionTopology":
        if self.schema_version != CONNECTION_SCHEMA_VERSION:
            raise ValueError(
                f"unsupported connection schema {self.schema_version!r}; "
                f"expected {CONNECTION_SCHEMA_VERSION}"
            )
        if self.topology_mode not in TOPOLOGY_MODES:
            raise ValueError(f"unsupported topology_mode: {self.topology_mode!r}")
        if not _SYSTEM_ID.fullmatch(str(self.system_id)):
            raise ValueError("system_id must be a ROS-safe lower-case identifier")
        if self.security_profile not in SECURITY_PROFILES:
            raise ValueError(f"unsupported security_profile: {self.security_profile!r}")
        self.dds_graph.validate()
        if self.topology_mode == "full":
            if not 2 <= len(self.hosts) <= len(ROLES):
                raise ValueError("a full managed topology requires 2..4 active hosts")
        elif not 1 <= len(self.hosts) <= len(SIMULATION_ROLES):
            raise ValueError(
                "a simulation-only topology requires 1..3 active hosts"
            )
        for host in self.hosts:
            host.validate()
        host_ids = [host.host_id for host in self.hosts]
        if len(set(host_ids)) != len(host_ids):
            raise ValueError("host IDs must be unique")
        dds_addresses = [host.dds.address.casefold() for host in self.hosts]
        if len(set(dds_addresses)) != len(dds_addresses):
            raise ValueError("active host DDS addresses must be unique")
        routed_addresses = any(
            host.dds.address_source == "tailscale"
            or _is_tailscale_ipv4(host.dds.address)
            or host.dds.interface.casefold() == "tailscale0"
            for host in self.hosts
        )
        if (
            len(self.hosts) > 1
            and self.dds_graph.discovery_mode == "multicast"
            and routed_addresses
        ):
            raise ValueError(
                "multicast DDS discovery cannot cross Tailscale/routed VPN hosts; "
                "select static discovery so every host uses the other hosts' DDS "
                "addresses as direct peers"
            )
        if sum(host.local for host in self.hosts) != 1:
            raise ValueError("exactly one managed host must be local")
        if "robot" in next(host for host in self.hosts if host.local).roles:
            raise ValueError(
                "the operator-owned authority host cannot be the Robot host"
            )

        expected_roles = ROLES if self.topology_mode == "full" else SIMULATION_ROLES
        by_role: dict[str, list[RoleAssignment]] = {
            role: [] for role in expected_roles
        }
        endpoint_ids: list[str] = []
        for host in self.hosts:
            for assignment in host.assignments:
                if assignment.role not in by_role:
                    if self.topology_mode == "simulation-only" and assignment.role == "robot":
                        raise ValueError(
                            "simulation-only topology must not assign the Robot role"
                        )
                    raise ValueError(
                        f"role is not valid for {self.topology_mode}: {assignment.role}"
                    )
                by_role[assignment.role].append(assignment)
                endpoint_ids.append(assignment.endpoint_id)
        invalid = [role for role, values in by_role.items() if len(values) != 1]
        if invalid:
            raise ValueError(
                "every role must be assigned exactly once; invalid roles: "
                + ", ".join(invalid)
            )
        if len(set(endpoint_ids)) != len(endpoint_ids):
            raise ValueError("endpoint IDs must be unique across the topology")
        endpoint_keys = [canonical_endpoint_key(value) for value in endpoint_ids]
        if len(set(endpoint_keys)) != len(endpoint_keys):
            raise ValueError(
                "endpoint IDs must remain unique after '-' to '_' canonicalization"
            )
        return self

    @property
    def local_host(self) -> ManagedHost:
        self.validate()
        return next(host for host in self.hosts if host.local)

    def host(self, host_id: str) -> ManagedHost:
        for host in self.hosts:
            if host.host_id == host_id:
                return host
        raise KeyError(host_id)

    def discovery_peers(self, host_id: str) -> tuple[str, ...]:
        """Return validated static discovery seeds for one active host."""

        self.validate()
        self.host(host_id)
        if self.dds_graph.discovery_mode == "multicast":
            return ()
        return tuple(host.dds.address for host in self.hosts if host.host_id != host_id)

    def to_dict(self) -> dict[str, Any]:
        self.validate()
        return {
            "schema_version": self.schema_version,
            "topology_mode": self.topology_mode,
            "system_id": self.system_id,
            "security_profile": self.security_profile,
            "dds_graph": self.dds_graph.to_dict(),
            "hosts": [host.to_dict() for host in self.hosts],
        }

    @classmethod
    def from_dict(cls, raw: Mapping[str, Any]) -> "ConnectionTopology":
        _reject_secret_fields(raw)
        if not isinstance(raw, Mapping):
            raise ValueError("connection topology must be an object")
        if "schema_version" not in raw:
            _strict_object(
                raw,
                required={
                    "schema_version",
                    "system_id",
                    "security_profile",
                    "dds_graph",
                    "hosts",
                },
                name="connection topology",
            )
        incoming_version = _required_integer(
            raw.get("schema_version"), name="schema_version"
        )
        common_fields = {
            "schema_version",
            "system_id",
            "security_profile",
            "dds_graph",
            "hosts",
        }
        if incoming_version == LEGACY_CONNECTION_SCHEMA_VERSION:
            values = _strict_object(
                raw, required=common_fields, name="connection topology"
            )
            topology_mode = "full"
        elif incoming_version in (PREVIOUS_CONNECTION_SCHEMA_VERSION, CONNECTION_SCHEMA_VERSION):
            values = _strict_object(
                raw,
                required=common_fields | {"topology_mode"},
                name="connection topology",
            )
            topology_mode = _required_string(
                values["topology_mode"], name="topology_mode"
            )
        else:
            raise ValueError(
                f"unsupported connection schema {incoming_version!r}; "
                f"expected {LEGACY_CONNECTION_SCHEMA_VERSION}, "
                f"{PREVIOUS_CONNECTION_SCHEMA_VERSION} or {CONNECTION_SCHEMA_VERSION}"
            )
        hosts_raw = _object_sequence(values["hosts"], name="hosts")
        if not isinstance(values["dds_graph"], Mapping):
            raise ValueError("dds_graph must be an object")
        return cls(
            schema_version=CONNECTION_SCHEMA_VERSION,
            topology_mode=topology_mode,
            system_id=_required_string(values["system_id"], name="system_id"),
            security_profile=_required_string(
                values["security_profile"], name="security_profile"
            ),
            hosts=tuple(ManagedHost.from_dict(item) for item in hosts_raw),
            dds_graph=DdsGraphSettings.from_dict(values["dds_graph"]),
        ).validate()

    @classmethod
    def load(cls, path: str | os.PathLike[str]) -> "ConnectionTopology":
        candidate = Path(path).expanduser()
        if candidate.is_symlink():
            raise ValueError(f"connection topology must not be a symlink: {candidate}")
        source = candidate.resolve()
        if not source.is_file():
            raise ValueError(f"connection topology is not a regular file: {source}")
        if source.stat().st_size > MAX_CONNECTION_FILE_BYTES:
            raise ValueError(f"connection topology is unexpectedly large: {source}")
        try:
            raw = json.loads(source.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"invalid connection topology JSON: {source}") from exc
        if not isinstance(raw, Mapping):
            raise ValueError("connection topology JSON must be an object")
        return cls.from_dict(raw)

    def save(self, path: str | os.PathLike[str]) -> Path:
        self.validate()
        candidate = Path(path).expanduser()
        if candidate.is_symlink():
            raise ValueError(f"refusing to replace symlink: {candidate}")
        destination = candidate.parent.resolve() / candidate.name
        destination.parent.mkdir(parents=True, exist_ok=True)
        payload = json.dumps(self.to_dict(), ensure_ascii=False, indent=2) + "\n"
        temporary: Path | None = None
        try:
            descriptor, name = tempfile.mkstemp(
                dir=destination.parent, prefix=f".{destination.name}.", text=True
            )
            temporary = Path(name)
            os.fchmod(descriptor, 0o600)
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                handle.write(payload)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, destination)
            os.chmod(destination, 0o600)
            directory = os.open(destination.parent, os.O_RDONLY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if temporary is not None and temporary.exists():
                temporary.unlink()
        mode = stat.S_IMODE(destination.stat().st_mode)
        if mode != 0o600:
            raise OSError(f"connection topology mode is {mode:o}, expected 600")
        return destination


def _strict_object(
    raw: Mapping[str, Any], *, required: set[str], name: str,
    optional: set[str] | None = None,
) -> dict[str, Any]:
    if not isinstance(raw, Mapping):
        raise ValueError(f"{name} must be an object")
    keys = {str(key) for key in raw}
    missing = sorted(required - keys)
    allowed = required | set(optional or ())
    unknown = sorted(keys - allowed)
    if missing:
        raise ValueError(f"{name} is missing fields: {', '.join(missing)}")
    if unknown:
        raise ValueError(f"{name} has unknown fields: {', '.join(unknown)}")
    return dict(raw)


def _object_sequence(value: Any, *, name: str) -> tuple[Mapping[str, Any], ...]:
    if not isinstance(value, Sequence) or isinstance(value, (str, bytes, bytearray)):
        raise ValueError(f"{name} must be an array")
    if any(not isinstance(item, Mapping) for item in value):
        raise ValueError(f"every {name} item must be an object")
    return tuple(value)


def _required_string(value: Any, *, name: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value.strip()


def _optional_string(value: Any, *, name: str) -> str:
    if not isinstance(value, str):
        raise ValueError(f"{name} must be a string")
    return value


def _required_integer(value: Any, *, name: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{name} must be an integer")
    return value


def _plain_text(value: Any, *, name: str, maximum: int) -> str:
    text = str(value).strip()
    if not text or len(text) > maximum or any(ord(character) < 32 for character in text):
        raise ValueError(f"{name} must be plain text of at most {maximum} characters")
    return text


def _validate_network_host(
    host: Any,
    *,
    name: str,
    reject_loopback: bool = False,
    reject_multicast: bool = False,
) -> None:
    value = _plain_text(host, name=name, maximum=255)
    unbracketed = value.removeprefix("[").removesuffix("]")
    if "://" in value or "/" in value or any(character.isspace() for character in value):
        raise ValueError(f"{name} must be a hostname or IP without a port")
    try:
        address = ipaddress.ip_address(unbracketed)
    except ValueError:
        if ":" in unbracketed:
            raise ValueError(f"{name} must not include a port")
        return
    if address.is_unspecified:
        raise ValueError(f"{name} must not be an unspecified bind address")
    if reject_loopback and address.is_loopback:
        raise ValueError(f"{name} must not be a loopback address")
    if reject_multicast and address.is_multicast:
        raise ValueError(f"{name} must not be a multicast address")


def _is_tailscale_ipv4(value: str) -> bool:
    """Return whether an address is in Tailscale's CGNAT IPv4 pool."""

    try:
        address = ipaddress.ip_address(str(value).strip())
    except ValueError:
        return False
    return address.version == 4 and address in ipaddress.ip_network("100.64.0.0/10")


def _validate_absolute_posix_path(value: Any, *, name: str) -> None:
    text = _plain_text(value, name=name, maximum=4096)
    path = PurePosixPath(text)
    if not path.is_absolute() or path == PurePosixPath("/") or ".." in path.parts:
        raise ValueError(f"{name} must be a contained absolute POSIX path")


def canonical_endpoint_key(endpoint_id: str) -> str:
    if not _ENDPOINT_ID.fullmatch(str(endpoint_id)):
        raise ValueError(f"invalid endpoint_id: {endpoint_id!r}")
    return str(endpoint_id).replace("-", "_")[:63]


def operator_home_path() -> Path:
    """Return the host HOME mounted into the connection-manager container."""

    configured = os.environ.get("ELESIM_OPERATOR_HOME", "").strip()
    candidate = Path(configured) if configured else Path.home()
    if not candidate.is_absolute():
        raise ValueError("ELESIM_OPERATOR_HOME must be an absolute path")
    return candidate.resolve()


def resolve_ssh_identity_path(identity_file: str) -> Path:
    """Resolve an identity path against the operator HOME, not container HOME."""

    raw = str(identity_file).strip()
    if not raw:
        raise ValueError("SSH identity_file is empty; use SSH agent mode")
    operator_home = operator_home_path()
    if raw == "~":
        candidate = operator_home
    elif raw.startswith("~/"):
        candidate = operator_home / raw[2:]
    else:
        candidate = Path(raw)
        if not candidate.is_absolute():
            raise ValueError("SSH identity_file must be an absolute or '~/...' path")
    normalized = Path(os.path.abspath(candidate))
    if raw == "~" or raw.startswith("~/"):
        if operator_home != normalized and operator_home not in normalized.parents:
            raise ValueError("SSH identity_file '~/...' path escapes operator HOME")
    return normalized


def _reject_secret_fields(value: Any, *, path: str = "root") -> None:
    if isinstance(value, Mapping):
        for key, child in value.items():
            name = str(key).casefold().replace("-", "_")
            if any(marker in name for marker in _FORBIDDEN_SECRET_KEYS):
                raise ValueError(f"secret material is forbidden in connection state: {path}.{key}")
            _reject_secret_fields(child, path=f"{path}.{key}")
    elif isinstance(value, Sequence) and not isinstance(value, (str, bytes, bytearray)):
        for index, child in enumerate(value):
            _reject_secret_fields(child, path=f"{path}[{index}]")


__all__ = [
    "CONNECTION_SCHEMA_VERSION",
    "LEGACY_CONNECTION_SCHEMA_VERSION",
    "PREVIOUS_CONNECTION_SCHEMA_VERSION",
    "PREFLIGHT_SCHEMA_VERSION",
    "INSTALL_MODES",
    "LIFECYCLES",
    "ROLES",
    "SIMULATION_ROLES",
    "SECURITY_PROFILES",
    "SSH_AUTH_MODES",
    "TOPOLOGY_MODES",
    "ConnectionTopology",
    "DdsEndpoint",
    "DdsGraphSettings",
    "DeploymentUnit",
    "ManagedHost",
    "PreflightHost",
    "PreflightSshEndpoint",
    "RoleAssignment",
    "SshEndpoint",
    "TwoHostPreflight",
    "canonical_endpoint_key",
    "operator_home_path",
    "resolve_ssh_identity_path",
]
