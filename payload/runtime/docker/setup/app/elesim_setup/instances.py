"""Small, host-local registry for installed EleSim release instances.

This is deliberately separate from :mod:`elesim_setup.state`: an installation
state describes one prefix, while this registry describes several release
instances which may be selected by a host-side operator.
"""

from __future__ import annotations

import fcntl
import hashlib
import json
import os
import re
import secrets
import stat
import tempfile
import uuid
from contextlib import contextmanager
from dataclasses import dataclass, field, replace as dataclass_replace
from pathlib import Path
from typing import Any, Iterator, Mapping

from .state import (
    ComputeSettings,
    DdsSettings,
    DDS_SECURITY_PROFILES,
    NetworkSettings,
    TurnSettings,
)
from .instance_identity import service_key


# Schema v3 records the graph-wide role endpoint IDs needed by a role on one
# host to address peers assigned to another host.  Schema v2 remains readable:
# local endpoint IDs are recovered from ``endpoints`` and absent remote roles
# retain the historical install defaults until the manager replaces the
# instance with a complete graph-derived record.
SCHEMA = 3
LEGACY_SCHEMA = 2
ROLES = frozenset({"pilot", "sim", "ui"})
_SYSTEM = re.compile(r"^[a-z][a-z0-9_]{0,62}$")
_ENDPOINT = re.compile(r"^[a-z][a-z0-9_-]{0,62}$")
_SHA256 = re.compile(r"^[0-9a-f]{64}$")
# Keep the per-instance listener wholly above the relay allocation window.
# The previous 50000..59999 listener window overlapped the default
# 40000..63999 relay window, making an otherwise deterministic allocation
# ambiguous on a shared host network namespace.
_SCOPED_TURN_LISTEN_BASE = 64000
_SCOPED_TURN_LISTEN_SPAN = 1536
_SCOPED_TURN_RELAY_BASE = 40000
_SCOPED_TURN_RELAY_WIDTH = 40
_LEGACY_TURN_RELAY_MIN = 49160
_LEGACY_TURN_RELAY_MAX = 49200
# Allocate fixed-width relay blocks on either side of the legacy install's
# 49160..49200 block.  The old 600-slot range crossed that block and could
# make a fresh scoped instance steal packets from a still-running legacy
# Coturn on a host-networked Docker engine.
_SCOPED_TURN_RELAY_FIRST_SLOTS = (
    (_LEGACY_TURN_RELAY_MIN - _SCOPED_TURN_RELAY_BASE) // _SCOPED_TURN_RELAY_WIDTH
)
_SCOPED_TURN_RELAY_SECOND_BASE = _LEGACY_TURN_RELAY_MAX + 1
_SCOPED_TURN_RELAY_SECOND_SLOTS = (
    (64000 - _SCOPED_TURN_RELAY_SECOND_BASE) // _SCOPED_TURN_RELAY_WIDTH
)
_SCOPED_TURN_RELAY_SLOTS = (
    _SCOPED_TURN_RELAY_FIRST_SLOTS + _SCOPED_TURN_RELAY_SECOND_SLOTS
)
_GPU_SELECTOR = re.compile(
    r"^(?:[0-9]{1,6}|GPU-[A-Za-z0-9_-]{1,124}|"
    r"MIG-GPU-[A-Za-z0-9_-]{1,116}/[0-9]+/[0-9]+)$"
)


def _string(value: Any, name: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{name} must be a non-empty string")
    return value


@dataclass(frozen=True)
class InstanceEndpoint:
    role: str
    endpoint_id: str

    def __post_init__(self) -> None:
        if not isinstance(self.role, str):
            raise ValueError("role must be a string")
        if self.role == "robot":
            raise ValueError("robot instances are not supported")
        if self.role not in ROLES:
            raise ValueError(f"unsupported instance role: {self.role!r}")
        if not isinstance(self.endpoint_id, str) or not _ENDPOINT.fullmatch(self.endpoint_id):
            raise ValueError("endpoint_id must be a safe identifier")

    def to_dict(self) -> dict[str, str]:
        return {"role": self.role, "endpoint_id": self.endpoint_id}

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "InstanceEndpoint":
        if not isinstance(value, dict) or set(value) != {"role", "endpoint_id"}:
            raise ValueError("endpoint must contain exactly role and endpoint_id")
        return cls(role=_string(value["role"], "role"), endpoint_id=_string(value["endpoint_id"], "endpoint_id"))


@dataclass(frozen=True)
class InstanceState:
    system_id: str
    release_key: str
    endpoints: tuple[InstanceEndpoint, ...]
    domain_id: int
    pilot_id: str = ""
    sim_id: str = ""
    ui_id: str = ""
    rmw_implementation: str = "rmw_cyclonedds_cpp"
    discovery_mode: str = "multicast"
    static_peers: tuple[str, ...] = ()
    interface: str = ""
    security_profile: str = "trusted-network"
    # This is only the active generation identifier.  Private key material and
    # keystore paths remain outside the registry in instance security state.
    security_generation: str = ""
    # TURN belongs to the system instance, never to the shared install.  The
    # optional URL list is kept here as well because different instances may
    # use different external relays.  Managed instances may omit URLs; the
    # preparation layer derives one from public_host and the allocated port.
    turn: TurnSettings = field(default_factory=TurnSettings)
    turn_urls: tuple[str, ...] = ()
    # Compute is part of the immutable instance contract.  It deliberately
    # uses the same three modes as the install-level setting, while allowing
    # two instances in one aggregate to select different devices.  A missing
    # field in older schema-v2 records inherits the historical install policy
    # at preparation time (see ``prepare_instance_services``).
    compute: ComputeSettings = field(default_factory=ComputeSettings)
    # Internal migration marker: schema-v2 records published before compute
    # existed must continue to use the install-wide fixed GPU policy.
    compute_is_explicit: bool = field(default=True, repr=False, compare=False)

    def __post_init__(self) -> None:
        if not isinstance(self.system_id, str) or not _SYSTEM.fullmatch(self.system_id):
            raise ValueError("system_id must be a safe identifier")
        if not isinstance(self.release_key, str) or not _SHA256.fullmatch(self.release_key):
            raise ValueError("release_key must be a lowercase SHA-256 hex digest")
        if isinstance(self.domain_id, bool) or not isinstance(self.domain_id, int) or not 0 <= self.domain_id <= 232:
            raise ValueError("domain_id must be an integer from 0 through 232")
        if not isinstance(self.endpoints, tuple):
            raise ValueError("endpoints must be a tuple")
        if not self.endpoints or any(not isinstance(e, InstanceEndpoint) for e in self.endpoints):
            raise ValueError("endpoints must be non-empty InstanceEndpoint values")
        ids = [endpoint.endpoint_id for endpoint in self.endpoints]
        if len(set(ids)) != len(ids):
            raise ValueError("endpoint_id values must be unique within a system")
        roles = [endpoint.role for endpoint in self.endpoints]
        if len(set(roles)) != len(roles):
            raise ValueError("endpoint roles must be unique within a system")
        local_ids = {endpoint.role: endpoint.endpoint_id for endpoint in self.endpoints}
        defaults = NetworkSettings()
        graph_role_ids: list[str] = []
        for role, field_name in (
            ("pilot", "pilot_id"),
            ("sim", "sim_id"),
            ("ui", "ui_id"),
        ):
            value = getattr(self, field_name)
            if not value:
                value = local_ids.get(role, getattr(defaults, field_name))
                object.__setattr__(self, field_name, value)
            if not isinstance(value, str) or _ENDPOINT.fullmatch(value) is None:
                raise ValueError(f"{field_name} must be a safe endpoint identifier")
            graph_role_ids.append(value)
            if role in local_ids and local_ids[role] != value:
                raise ValueError(
                    f"local {role} endpoint must match graph-wide {field_name}"
                )
        canonical_role_ids = tuple(value.replace("-", "_")[:63] for value in graph_role_ids)
        if len(set(canonical_role_ids)) != len(canonical_role_ids):
            raise ValueError(
                "graph role endpoint IDs must remain unique after '-' to '_' "
                "canonicalization"
            )
        if not isinstance(self.static_peers, tuple):
            raise ValueError("static_peers must be a tuple")
        if self.security_profile not in DDS_SECURITY_PROFILES:
            raise ValueError(f"unsupported instance security profile: {self.security_profile!r}")
        # Reuse the install-state DDS validator for transport fields.  Security
        # provisioning is intentionally omitted here: it is a host-side
        # provisioning mechanism, not per-instance secret state.
        DdsSettings(
            system_id=self.system_id,
            domain_id=self.domain_id,
            rmw_implementation=self.rmw_implementation,
            discovery_mode=self.discovery_mode,
            static_peers=self.static_peers,
            interface=self.interface,
            security_profile="trusted-network",
        ).validate()
        if not isinstance(self.security_generation, str):
            raise ValueError("security_generation must be a string")
        if self.security_generation and not re.fullmatch(
            r"[a-z0-9][a-z0-9_.-]{0,95}", self.security_generation
        ):
            raise ValueError("security_generation must be a safe identifier")
        if not isinstance(self.turn, TurnSettings):
            raise ValueError("turn must be TurnSettings")
        self.turn.validate()
        if not isinstance(self.turn_urls, tuple):
            raise ValueError("turn_urls must be a tuple")
        NetworkSettings(turn_urls=self.turn_urls).validate()
        if not isinstance(self.compute, ComputeSettings):
            raise ValueError("compute must be ComputeSettings")
        # Do not tighten the install-level validator: old fixed-install state
        # may use a legacy selector spelling.  Newly persisted instance
        # policies have the stricter single index/UUID contract.
        if self.compute_is_explicit:
            if not isinstance(self.compute.gpu_mode, str) or self.compute.gpu_mode not in {"inherit", "specific", "cpu"}:
                raise ValueError(f"unsupported instance GPU mode: {self.compute.gpu_mode!r}")
            device = self.compute.gpu_device
            if not isinstance(device, str):
                raise ValueError("instance gpu_device must be a string")
            if device != device.strip():
                raise ValueError("instance gpu_device must not contain surrounding whitespace")
            if self.compute.gpu_mode == "specific" and not _GPU_SELECTOR.fullmatch(device.strip()):
                raise ValueError(
                    "instance specific GPU policy requires one index, GPU UUID, or MIG UUID"
                )
            if self.compute.gpu_mode != "specific" and device:
                raise ValueError("instance gpu_device is valid only in specific mode")
        if type(self.compute_is_explicit) is not bool:
            raise ValueError("compute_is_explicit must be boolean")
        if self.turn.mode == "none" and self.turn_urls:
            raise ValueError("TURN URLs require managed or external TURN")
        if self.turn.mode == "external" and not self.turn.credential_file.strip():
            raise ValueError("external TURN on an instance requires a credential file")
        if self.turn.mode != "none" and "sim" not in {e.role for e in self.endpoints}:
            raise ValueError("TURN is owned by and requires the Sim endpoint")
        if self.turn.mode == "managed" and self.security_profile != "sros2":
            raise ValueError("managed TURN requires the sros2 security profile")

    def validate(self) -> "InstanceState":
        self.__post_init__()
        return self

    def to_dict(self) -> dict[str, Any]:
        payload: dict[str, Any] = {
            "schema_version": SCHEMA,
            "system_id": self.system_id,
            "release_key": self.release_key,
            "endpoints": [endpoint.to_dict() for endpoint in self.endpoints],
            "domain_id": self.domain_id,
            "role_ids": {
                "pilot": self.pilot_id,
                "sim": self.sim_id,
                "ui": self.ui_id,
            },
            "rmw_implementation": self.rmw_implementation,
            "discovery_mode": self.discovery_mode,
            "static_peers": list(self.static_peers),
            "interface": self.interface,
            "security_profile": self.security_profile,
            "security_generation": self.security_generation,
            "turn": {
                "mode": self.turn.mode,
                "realm": self.turn.realm,
                "public_host": self.turn.public_host,
                "secret_file": self.turn.secret_file,
                "credential_file": self.turn.credential_file,
                "listen_port": self.turn.listen_port,
                "relay_min_port": self.turn.relay_min_port,
                "relay_max_port": self.turn.relay_max_port,
            },
            "turn_urls": list(self.turn_urls),
        }
        if self.compute_is_explicit:
            payload["compute"] = {
                "gpu_mode": self.compute.gpu_mode,
                "gpu_device": self.compute.gpu_device,
            }
        return payload

    @classmethod
    def from_dict(cls, value: Mapping[str, Any]) -> "InstanceState":
        if not isinstance(value, dict):
            raise ValueError("instance state must be an object")
        required = {
            "schema_version", "system_id", "release_key", "endpoints", "domain_id",
            "rmw_implementation", "discovery_mode", "static_peers", "interface",
            "security_profile", "security_generation",
        }
        schema = value.get("schema_version")
        optional = {"turn", "turn_urls", "compute", "role_ids"}
        if (
            not set(value).issubset(required | optional)
            or not required.issubset(value)
            or type(schema) is not int
            or schema not in {LEGACY_SCHEMA, SCHEMA}
            or (schema == SCHEMA and "role_ids" not in value)
            or (schema == LEGACY_SCHEMA and "role_ids" in value)
        ):
            raise ValueError("unsupported or malformed instance state schema")
        if not isinstance(value["endpoints"], list):
            raise ValueError("endpoints must be a list")
        if not isinstance(value["static_peers"], list):
            raise ValueError("static_peers must be a list")
        turn_raw = value.get("turn", {})
        if not isinstance(turn_raw, Mapping):
            raise ValueError("turn must be an object")
        turn_urls = value.get("turn_urls", [])
        if not isinstance(turn_urls, list):
            raise ValueError("turn_urls must be a list")
        role_ids = value.get("role_ids", {})
        if not isinstance(role_ids, Mapping) or (
            role_ids and set(role_ids) != {"pilot", "sim", "ui"}
        ):
            raise ValueError("role_ids must contain exactly pilot, sim, and ui")
        compute_present = "compute" in value
        compute_raw = value.get("compute", {})
        if not isinstance(compute_raw, Mapping):
            raise ValueError("compute must be an object")
        # ``compute`` was added after the initial schema-v2 publication.  An
        # omitted field is intentionally the legacy inherit default; callers
        # that load a pre-policy instance can still apply install defaults
        # explicitly before rendering.
        compute_values = {
            "gpu_mode": compute_raw.get("gpu_mode", "inherit"),
            "gpu_device": compute_raw.get("gpu_device", ""),
        }
        if set(compute_raw) - {"gpu_mode", "gpu_device"}:
            raise ValueError("compute contains unsupported fields")
        return cls(
            system_id=_string(value["system_id"], "system_id"),
            release_key=_string(value["release_key"], "release_key"),
            endpoints=tuple(InstanceEndpoint.from_dict(item) for item in value["endpoints"]),
            domain_id=value["domain_id"],
            pilot_id=(
                _string(role_ids["pilot"], "role_ids.pilot")
                if role_ids else ""
            ),
            sim_id=(
                _string(role_ids["sim"], "role_ids.sim")
                if role_ids else ""
            ),
            ui_id=(
                _string(role_ids["ui"], "role_ids.ui")
                if role_ids else ""
            ),
            rmw_implementation=_string(value["rmw_implementation"], "rmw_implementation"),
            discovery_mode=_string(value["discovery_mode"], "discovery_mode"),
            static_peers=tuple(_string(peer, "static_peer") for peer in value["static_peers"]),
            interface=value["interface"],
            security_profile=_string(value["security_profile"], "security_profile"),
            security_generation=value["security_generation"],
            turn=TurnSettings(**dict(turn_raw)),
            turn_urls=tuple(_string(url, "turn_url") for url in turn_urls),
            compute=ComputeSettings(**compute_values),
            compute_is_explicit=compute_present,
        )


def scoped_turn_settings(
    instance: InstanceState,
    install_uuid: str | None = None,
) -> TurnSettings:
    """Resolve omitted managed TURN ports deterministically for one system.

    Legacy install TURN uses its fixed 3478/49160-49200 values.  Instance
    records intentionally omit ports by default, so their allocation cannot
    collide with that legacy service or another system by construction.  The
    installation UUID participates in the default allocation when available;
    otherwise the system ID remains the stable compatibility input.  This is
    important because scoped instances use host networking in direct-host
    mode: two prefixes may contain the same system ID but must not receive
    the same default Coturn ports.  Any explicitly supplied value is
    preserved and checked by the runtime.
    """

    instance.validate()
    turn = instance.turn
    if turn.mode != "managed":
        return turn
    if install_uuid is not None:
        try:
            parsed = uuid.UUID(install_uuid)
        except (TypeError, ValueError, AttributeError) as exc:
            raise ValueError("install_uuid must be a canonical UUID string") from exc
        if str(parsed) != install_uuid:
            raise ValueError("install_uuid must be a canonical lowercase UUID string")
        allocation_key = f"{install_uuid}\0{instance.system_id}"
    else:
        allocation_key = instance.system_id
    digest = hashlib.sha256(allocation_key.encode("utf-8")).digest()
    slot = int.from_bytes(digest[:4], "big") % _SCOPED_TURN_LISTEN_SPAN
    relay_slot = int.from_bytes(digest[4:8], "big") % _SCOPED_TURN_RELAY_SLOTS
    listen = turn.listen_port
    relay_min = turn.relay_min_port
    relay_max = turn.relay_max_port
    if listen is None:
        listen = _SCOPED_TURN_LISTEN_BASE + slot
    if relay_min is None:
        if relay_slot < _SCOPED_TURN_RELAY_FIRST_SLOTS:
            relay_min = _SCOPED_TURN_RELAY_BASE + relay_slot * _SCOPED_TURN_RELAY_WIDTH
        else:
            relay_min = _SCOPED_TURN_RELAY_SECOND_BASE + (
                relay_slot - _SCOPED_TURN_RELAY_FIRST_SLOTS
            ) * _SCOPED_TURN_RELAY_WIDTH
        relay_max = relay_min + _SCOPED_TURN_RELAY_WIDTH - 1
    return TurnSettings(
        mode=turn.mode,
        realm=turn.realm,
        public_host=turn.public_host,
        secret_file=turn.secret_file,
        credential_file=turn.credential_file,
        listen_port=listen,
        relay_min_port=relay_min,
        relay_max_port=relay_max,
    )


def turn_service_key(system_id: str) -> str:
    """Return the stable Compose key for a system's Sim-owned Coturn."""

    return service_key(system_id, "coturn")


def instance_turn_secret_path(prefix: str | os.PathLike[str], system_id: str) -> Path:
    """Return the only valid managed TURN secret location for one instance."""

    if not isinstance(system_id, str) or _SYSTEM.fullmatch(system_id) is None:
        raise ValueError("system_id must be a safe identifier")
    return Path(os.path.abspath(os.path.expanduser(os.fspath(prefix)))) / "instances" / system_id / "secrets" / "turn.secret"


class InstanceRegistry:
    """Concurrent, symlink-safe registry rooted at an installation prefix."""

    def __init__(self, prefix: str | os.PathLike[str]) -> None:
        self.prefix = Path(prefix).expanduser()
        self.instances = self.prefix / "instances"
        # Lock metadata lives below the exact managed instances subtree, so a
        # clean ownership-based uninstall removes it with the registry.
        self.lock_root = self.instances / ".locks"
        self.lock_path = self.lock_root / "install.lock"

    def _validate_lock_root(self) -> None:
        if self.lock_root.is_symlink() or not self.lock_root.is_dir():
            raise ValueError("instance lock metadata must be a real directory")
        for child in self.lock_root.iterdir():
            info = child.lstat()
            if (
                child.is_symlink()
                or not stat.S_ISREG(info.st_mode)
                or info.st_nlink != 1
            ):
                raise ValueError(f"instance lock metadata must be regular files: {child}")

    def _check_path(self, path: Path, *, allow_missing: bool = True) -> None:
        current = Path(os.path.abspath(path))
        while True:
            if current.is_symlink():
                raise ValueError(f"refusing symlink path: {current}")
            if current == current.parent:
                break
            current = current.parent
        if not allow_missing and not path.exists():
            raise FileNotFoundError(path)

    @contextmanager
    def _locked(self) -> Iterator[None]:
        self._check_path(self.prefix)
        self.prefix.mkdir(parents=True, exist_ok=True)
        self._check_path(self.prefix)
        if not self.prefix.is_dir():
            raise ValueError("instance registry prefix must be a directory")
        if self.instances.is_symlink():
            raise ValueError("instance registry instances must not be a symlink")
        if self.instances.exists() and not self.instances.is_dir():
            raise ValueError("instance registry instances must be a directory")
        self.instances.mkdir(parents=True, exist_ok=True)
        if self.lock_root.is_symlink() or (self.lock_root.exists() and not self.lock_root.is_dir()):
            raise ValueError("instance lock metadata must be a directory")
        self.lock_root.mkdir(mode=0o700, parents=True, exist_ok=True)
        self._validate_lock_root()
        if self.lock_path.is_symlink():
            raise ValueError(f"refusing symlink path: {self.lock_path}")
        fd = os.open(
            self.lock_path,
            os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC,
            0o600,
        )
        info = os.fstat(fd)
        if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
            os.close(fd)
            raise ValueError("instance registry lock must be an unlinked regular file")
        handle = os.fdopen(fd, "a+b")
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX)
            try:
                yield
            finally:
                fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        finally:
            handle.close()

    def _state_path(self, system: str) -> Path:
        if not isinstance(system, str) or not _SYSTEM.fullmatch(system):
            raise ValueError("system must be a safe identifier")
        return self.instances / system / "state.json"

    @staticmethod
    def _is_retained_directory(child: Path, state_path: Path) -> bool:
        """Recognize runtime snapshots left after a deliberate removal."""

        if os.path.lexists(state_path):
            return False
        try:
            children = tuple(child.iterdir())
        except OSError:
            return False
        return (
            bool(children)
            and all(not item.is_symlink() and item.is_dir() for item in children)
            and {item.name for item in children}.issubset({"security", "logs", "secrets"})
        )

    def list(self) -> tuple[InstanceState, ...]:
        with self._locked():
            if self.instances.is_symlink():
                raise ValueError(f"refusing symlink path: {self.instances}")
            if not self.instances.exists():
                return ()
            if not self.instances.is_dir():
                raise ValueError("instances is not a directory")
            result = []
            for child in sorted(self.instances.iterdir(), key=lambda p: p.name):
                if child == self.lock_root:
                    self._validate_lock_root()
                    continue
                path = child / "state.json"
                self._check_path(path)
                if not child.is_dir() or child.is_symlink():
                    raise ValueError(f"invalid instance directory: {child}")
                if self._is_retained_directory(child, path):
                    continue
                result.append(self._read(path))
            return tuple(result)

    def _read(self, path: Path) -> InstanceState:
        self._check_path(path, allow_missing=False)
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"invalid instance state file: {path}")
        try:
            fd = os.open(
                path,
                os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK | os.O_CLOEXEC,
            )
        except OSError as exc:
            raise ValueError(f"invalid instance state file: {path}") from exc
        try:
            info = os.fstat(fd)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise ValueError(f"invalid instance state file: {path}")
            with os.fdopen(fd, "r", encoding="utf-8") as stream:
                fd = -1
                state = InstanceState.from_dict(json.load(stream))
            if state.system_id != path.parent.name:
                raise ValueError("instance system_id does not match its directory")
            return state
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise ValueError(f"malformed instance state: {path}") from exc
        finally:
            if fd != -1:
                os.close(fd)

    def load(self, system: str) -> InstanceState:
        with self._locked():
            return self._read(self._state_path(system))

    def save(self, instance: InstanceState, *, replace: bool = False) -> None:
        if instance.turn.mode == "managed":
            # Registry callers are also a supported publication boundary;
            # never persist a caller-selected or install-level secret path.
            instance = dataclass_replace(
                instance,
                turn=dataclass_replace(
                    instance.turn,
                    secret_file=str(instance_turn_secret_path(self.prefix, instance.system_id)),
                ),
            )
        instance.validate()
        with self._locked():
            path = self._state_path(instance.system_id)
            self._check_path(path)
            if path.is_symlink():
                raise ValueError(f"refusing symlink path: {path}")
            exists = path.exists() or path.is_symlink()
            if exists and not replace:
                raise FileExistsError(path)
            if not exists and replace:
                raise FileNotFoundError(path)
            if exists and path.is_file():
                self._read(path)
            if self.instances.exists() and not self.instances.is_dir():
                raise ValueError("instances is not a directory")
            if self.instances.is_dir():
                for child in self.instances.iterdir():
                    if child == self.lock_root:
                        self._validate_lock_root()
                        continue
                    if child.name == instance.system_id:
                        continue
                    candidate = child / "state.json"
                    if child.is_symlink() or not child.is_dir():
                        raise ValueError(f"invalid instance directory: {child}")
                    if self._is_retained_directory(child, candidate):
                        continue
                    if not candidate.exists() and not candidate.is_symlink():
                        raise ValueError(f"missing instance state: {candidate}")
                    prior = self._read(candidate)
                    if prior.domain_id == instance.domain_id:
                        raise ValueError(f"domain_id collision with system {prior.system_id!r}")
            self.instances.mkdir(mode=0o700, exist_ok=True)
            directory = path.parent
            if directory.is_symlink():
                raise ValueError(f"refusing symlink path: {directory}")
            directory_created = not directory.exists()
            directory.mkdir(mode=0o700, exist_ok=True)
            self._check_path(directory)
            if instance.turn.mode == "managed":
                secret = directory / "secrets" / "turn.secret"
                if secret.exists() or secret.is_symlink():
                    if secret.is_symlink() or not secret.is_file():
                        raise ValueError(f"managed TURN secret is not a regular file: {secret}")
                    if stat.S_IMODE(secret.stat().st_mode) != 0o600:
                        raise ValueError(f"managed TURN secret must have mode 0600: {secret}")
                    if not secret.read_bytes().strip():
                        raise ValueError(f"managed TURN secret is empty: {secret}")
                else:
                    secret.parent.mkdir(mode=0o700, parents=True, exist_ok=True)
                    with tempfile.NamedTemporaryFile(
                        "w", encoding="utf-8", dir=secret.parent,
                        prefix=".turn.secret.", delete=False,
                    ) as handle:
                        handle.write(secrets.token_urlsafe(48) + "\n")
                        handle.flush()
                        os.fsync(handle.fileno())
                        temporary_secret = Path(handle.name)
                    temporary_secret.chmod(0o600)
                    os.replace(temporary_secret, secret)
            temporary = None
            try:
                with tempfile.NamedTemporaryFile("w", encoding="utf-8", dir=directory, prefix=".state.", delete=False) as handle:
                    json.dump(instance.to_dict(), handle, sort_keys=True, indent=2)
                    handle.write("\n")
                    handle.flush()
                    os.fsync(handle.fileno())
                    temporary = Path(handle.name)
                temporary.chmod(0o600)
                os.replace(temporary, path)
                fd = os.open(directory, os.O_RDONLY)
                try:
                    os.fsync(fd)
                finally:
                    os.close(fd)
            finally:
                if temporary is not None and temporary.exists():
                    temporary.unlink()
                # A failed first publication must not leave a broken registry
                # entry that prevents unrelated systems from being selected.
                if directory_created and not path.exists():
                    directory.rmdir()

    def select(self, system: str | None = None) -> InstanceState:
        if system is not None:
            return self.load(system)
        found = self.list()
        if not found:
            raise LookupError("no registered instances")
        if len(found) != 1:
            raise LookupError("instance selection is ambiguous")
        return found[0]
