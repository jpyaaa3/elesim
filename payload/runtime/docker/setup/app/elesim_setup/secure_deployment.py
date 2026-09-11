"""Fail-closed SSH/SFTP deployment and all-host security generation rollout.

The module accepts already-created role-scoped security bundles.  It neither
creates an SROS2 authority nor persists private material in connection state.
Remote lifecycle behavior is injected because container hosts and the native
Robot service have deliberately different start/stop implementations.
"""

from __future__ import annotations

import base64
import hashlib
import hmac
import ipaddress
import json
import math
import os
import re
import shlex
import socket
import stat
import subprocess
import tempfile
import threading
import time
import uuid
from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field, replace
from pathlib import Path, PurePosixPath
from typing import Any, Callable, Protocol

from .connection_manager import (
    canonical_endpoint_key,
    ConnectionTopology,
    DeploymentUnit,
    ManagedHost,
    SshEndpoint,
    resolve_ssh_identity_path,
)
from .instance_identity import project_name
from .credentials import (
    _ParamikoProxySocket,
    proxy_failure_detail,
    tailscale_proxy_command,
)


MAX_BUNDLE_FILES = 256
MAX_BUNDLE_FILE_BYTES = 1024 * 1024
MAX_BUNDLE_BYTES = 8 * 1024 * 1024
MAX_REMOTE_OUTPUT_BYTES = 64 * 1024
_HOST_HELPER_RESPONSE_GRACE_S = 2.0
_BUILD_COMMAND_TIMEOUT_S = 30 * 60
# ``elesim-net doctor --timeout N`` is an in-container bounded wait.  The
# enclosing local helper/SSH command must outlive it, otherwise a normal
# Genesis cold start is reported as a host-helper timeout before the doctor
# can return its JSON result.
_DDS_DOCTOR_COMMAND_GRACE_S = 60.0
# Compose lifecycle commands are detached, but Docker may still spend time
# creating networks, extracting image layers, or waiting for a runtime
# backend.  The old 30-second local default made a valid ``up --no-build``
# look like a failed rollout and triggered an unnecessary rollback.
_RUNTIME_LIFECYCLE_TIMEOUT_S = 5 * 60
_NETWORK_LOGIN_TIMEOUT_S = 10 * 60
_RUNTIME_LIFECYCLE_ACTIONS = frozenset({"up", "start", "stop"})
ProgressCallback = Callable[[str, str | None], None]
CommandOutput = Callable[[str, str], None]
_PRIVATE_AUTHORITY_FILES = frozenset(
    {"ca.key.pem", "identity_ca.key.pem", "permissions_ca.key.pem"}
)
_GPU_SELECTOR = re.compile(
    r"^(?:[0-9]{1,6}|GPU-[A-Za-z0-9_-]{1,124}|"
    r"MIG-GPU-[A-Za-z0-9_-]{1,116}/[0-9]+/[0-9]+)$"
)
_VIEWER_USER = re.compile(r"^[A-Za-z_][A-Za-z0-9_.-]{0,31}$")


@dataclass(frozen=True)
class RuntimeLaunchOptions:
    """Ephemeral options for one browser-requested runtime launch.

    These values are intentionally not part of the saved topology.  The
    Compose wrapper receives bounded flags and turns them into environment
    values only for the requested ``up`` operation.
    """

    gpu_inherit: bool
    gpu_device: str
    viewer: bool
    # The first three fields are retained as a compatibility view for older
    # manager clients and persisted job snapshots.  New clients send the
    # role-specific fields below; a unit selects the field matching its
    # runtime role when constructing its launch command.
    pilot_gpu_inherit: bool | None = None
    pilot_gpu_device: str = ""
    sim_gpu_inherit: bool | None = None
    sim_gpu_device: str = ""

    @classmethod
    def from_payload(
        cls, payload: Mapping[str, object] | None
    ) -> "RuntimeLaunchOptions | None":
        if payload is None or not payload:
            return None
        allowed = {
            "gpu_inherit",
            "gpu_device",
            "pilot_gpu_inherit",
            "pilot_gpu_device",
            "sim_gpu_inherit",
            "sim_gpu_device",
            "viewer",
        }
        unknown = set(payload) - allowed
        if unknown:
            raise ValueError(
                "runtime launch options contain unsupported fields: "
                f"{sorted(str(value) for value in unknown)!r}"
            )
        gpu_inherit = payload.get("gpu_inherit", False)
        viewer = payload.get("viewer", False)
        if not isinstance(gpu_inherit, bool) or not isinstance(viewer, bool):
            raise ValueError("gpu_inherit and viewer must be boolean")

        legacy_device = _normalize_gpu_selector(
            payload.get("gpu_device", ""), name="gpu_device"
        )
        legacy = bool(gpu_inherit), legacy_device if gpu_inherit else ""
        role_fields_present = any(
            key in payload
            for key in (
                "pilot_gpu_inherit",
                "pilot_gpu_device",
                "sim_gpu_inherit",
                "sim_gpu_device",
            )
        )

        def role_values(role: str) -> tuple[bool, str]:
            inherit_key = f"{role}_gpu_inherit"
            device_key = f"{role}_gpu_device"
            raw_inherit = payload.get(inherit_key, legacy[0])
            if not isinstance(raw_inherit, bool):
                raise ValueError(f"{inherit_key} must be boolean")
            raw_device = _normalize_gpu_selector(
                payload.get(device_key, legacy[1]), name=device_key
            )
            # An inherited GPU with an empty selector is the explicit
            # ``Free`` choice: leave CUDA_VISIBLE_DEVICES untouched and let
            # the host/runtime assignment decide.  An unchecked role remains
            # distinct and is encoded as an empty override below (CPU-only
            # for this launch).
            return raw_inherit, raw_device if raw_inherit else ""

        pilot_inherit, pilot_device = role_values("pilot")
        sim_inherit, sim_device = role_values("sim")
        return cls(
            legacy[0] if not role_fields_present else pilot_inherit,
            legacy[1] if not role_fields_present else pilot_device,
            viewer,
            pilot_inherit if role_fields_present else None,
            pilot_device if role_fields_present else "",
            sim_inherit if role_fields_present else None,
            sim_device if role_fields_present else "",
        )

    def role_values(self, role: str) -> tuple[bool, str]:
        """Return the requested GPU selector for one runtime role."""

        if role == "pilot" and self.pilot_gpu_inherit is not None:
            return self.pilot_gpu_inherit, self.pilot_gpu_device if self.pilot_gpu_inherit else ""
        if role == "sim" and self.sim_gpu_inherit is not None:
            return self.sim_gpu_inherit, self.sim_gpu_device if self.sim_gpu_inherit else ""
        return self.gpu_inherit, self.gpu_device if self.gpu_inherit else ""

    def launcher_flags(self, roles: Sequence[str] | None = None) -> tuple[str, ...]:
        """Return bounded ``elesim-up`` flags for this one launch."""

        role = ""
        if roles is None:
            gpu_inherit, gpu_device = self.gpu_inherit, self.gpu_device
        else:
            selected = set(str(role) for role in roles)
            role = "sim" if "sim" in selected else "pilot" if "pilot" in selected else ""
            if role:
                gpu_inherit, gpu_device = self.role_values(role)
            else:
                gpu_inherit, gpu_device = False, ""
        if roles is not None and not role:
            flags: tuple[str, ...] = ()
        else:
            # ``Free`` must not export an empty CUDA_VISIBLE_DEVICES value: an
            # empty variable means CPU-only to CUDA.  The unchecked path keeps
            # the explicit empty flag so the launcher can intentionally force
            # CPU for one run.
            flags = (
                ()
                if gpu_inherit and not gpu_device
                else ("--cuda-visible-devices", gpu_device if gpu_inherit else "")
            )
        return (*flags, "--view") if self.viewer else flags


def _normalize_gpu_selector(raw: object, *, name: str) -> str:
    """Validate one bounded CUDA selector (index or NVIDIA GPU UUID)."""

    if isinstance(raw, bool) or not isinstance(raw, (str, int)):
        raise ValueError(f"{name} must be a non-negative GPU index or UUID")
    value = str(raw).strip()
    if not value:
        return ""
    if not _GPU_SELECTOR.fullmatch(value):
        raise ValueError(f"{name} must be a non-negative GPU index or UUID")
    return value


def _command_timeout(argv: Sequence[str], base: float) -> float:
    """Return a bounded timeout appropriate for one managed command."""

    values = tuple(str(value) for value in argv)
    if (
        len(values) >= 4
        and PurePosixPath(values[0]).name == "elesim-net"
        and values[1] == "doctor"
        and values[2] == "--timeout"
    ):
        try:
            doctor_timeout = float(values[3])
        except (TypeError, ValueError):
            doctor_timeout = 0.0
        if math.isfinite(doctor_timeout) and doctor_timeout > 0:
            return max(
                float(base),
                doctor_timeout + float(_DDS_DOCTOR_COMMAND_GRACE_S),
            )
    if "build" in values:
        return max(float(base), float(_BUILD_COMMAND_TIMEOUT_S))
    if (
        values
        and PurePosixPath(values[0]).name == "elesim-tailscale"
        and "login" in values[1:]
    ):
        return max(float(base), float(_NETWORK_LOGIN_TIMEOUT_S))
    command_name = PurePosixPath(values[0]).name if values else ""
    if (
        command_name == "elesim-up"
        or (
            command_name == "elesim-instance"
            and len(values) >= 3
            and values[2] in {"up", "down", "logs", "status"}
        )
        or (
            any(
                value == "compose" or PurePosixPath(value).name == "elesim-compose"
                for value in values
            )
            and any(action in values for action in _RUNTIME_LIFECYCLE_ACTIONS)
        )
    ):
        return max(float(base), float(_RUNTIME_LIFECYCLE_TIMEOUT_S))
    return float(base)


class HostKeyVerificationError(RuntimeError):
    """The live SSH host key did not match the operator-pinned fingerprint."""


class SshAuthenticationError(RuntimeError):
    """The selected SSH authentication method was not accepted by the host."""


class SshConnectionError(RuntimeError):
    """The manager could not reach an SSH endpoint before authentication."""


class RemoteCommandError(RuntimeError):
    def __init__(self, argv: Sequence[str], result: "RemoteCommandResult") -> None:
        super().__init__(
            f"remote command failed ({result.exit_status}): "
            f"{shlex.join([str(value) for value in argv])}: {result.stderr.strip()}"
        )
        self.argv = tuple(str(value) for value in argv)
        self.result = result


class RolloutError(RuntimeError):
    def __init__(
        self,
        phase: str,
        cause: BaseException,
        rollback_errors: Sequence[BaseException] = (),
    ) -> None:
        detail = ""
        if rollback_errors:
            detail = f"; rollback also had {len(rollback_errors)} error(s)"
        super().__init__(f"security rollout failed during {phase}: {cause}{detail}")
        self.phase = phase
        self.cause = cause
        self.rollback_errors = tuple(rollback_errors)


@dataclass(frozen=True)
class SecurityFile:
    relative_path: str
    content: bytes
    mode: int = 0o600

    def validate(self) -> "SecurityFile":
        path = _safe_relative_path(self.relative_path)
        if path.name == "manifest.json":
            raise ValueError("manifest.json is generated by the deployment layer")
        if path.name.casefold() in _PRIVATE_AUTHORITY_FILES:
            raise ValueError(f"authority private key must not enter a host bundle: {path}")
        lowered = tuple(part.casefold() for part in path.parts)
        if "authority" in lowered and "private" in lowered:
            raise ValueError(f"authority private material must not enter a host bundle: {path}")
        if not isinstance(self.content, bytes):
            raise ValueError("security file content must be bytes")
        if len(self.content) > MAX_BUNDLE_FILE_BYTES:
            raise ValueError(f"security file is too large: {path}")
        if self.mode not in {0o600, 0o644}:
            raise ValueError("security file mode must be 0600 or 0644")
        return self


@dataclass(frozen=True)
class SecurityBundle:
    system_id: str
    host_id: str
    generation: str
    files: tuple[SecurityFile, ...]

    def validate(self) -> "SecurityBundle":
        _safe_identifier(self.system_id, name="system_id")
        _safe_identifier(self.host_id, name="host_id")
        _safe_generation(self.generation)
        if not self.files or len(self.files) > MAX_BUNDLE_FILES:
            raise ValueError(f"security bundle must contain 1..{MAX_BUNDLE_FILES} files")
        validated = tuple(file.validate() for file in self.files)
        names = [file.relative_path for file in validated]
        if len(set(names)) != len(names):
            raise ValueError("security bundle paths must be unique")
        if sum(len(file.content) for file in validated) > MAX_BUNDLE_BYTES:
            raise ValueError("security bundle exceeds the total byte limit")
        return self

    def manifest(self) -> dict[str, Any]:
        self.validate()
        return {
            "schema_version": 1,
            "system_id": self.system_id,
            "host_id": self.host_id,
            "generation": self.generation,
            "files": [
                {
                    "path": file.relative_path,
                    "size": len(file.content),
                    "mode": f"{file.mode:04o}",
                    "sha256": hashlib.sha256(file.content).hexdigest(),
                }
                for file in sorted(self.files, key=lambda item: item.relative_path)
            ],
        }

    def manifest_bytes(self) -> bytes:
        return (
            json.dumps(self.manifest(), ensure_ascii=False, sort_keys=True, indent=2) + "\n"
        ).encode("utf-8")

    def for_roles(self, roles: Sequence[str]) -> "SecurityBundle":
        """Return a role-scoped view for one installation unit.

        The Authority export is host-scoped because a computer may own more
        than one unit.  Runtime prefixes are not: a native Robot install must
        never receive the Compose unit's private enclave (and vice versa).
        Public trust material is retained in both views; only enclave and
        stable app-view paths matching ``roles`` are copied. Legacy
        ``roles/`` paths are normalized to ``apps/`` while loading.
        """

        selected = frozenset(str(role) for role in roles)
        if not selected:
            raise ValueError("a unit security bundle requires at least one role")
        files: list[SecurityFile] = []
        for file in self.files:
            parts = PurePosixPath(file.relative_path).parts
            if parts and parts[0] == "roles":
                file = SecurityFile(
                    PurePosixPath("apps", *parts[1:]).as_posix(),
                    file.content,
                    file.mode,
                )
                parts = PurePosixPath(file.relative_path).parts
            if parts[:2] == ("keystore", "public") or parts[:1] == ("public",):
                files.append(file)
                continue
            role = None
            if parts and parts[0] == "apps" and len(parts) >= 2:
                role = parts[1]
            elif parts[:2] == ("keystore", "enclaves"):
                if len(parts) < 5:
                    # governance.p7s/permissions.p7s and other keystore-wide
                    # trust material are common to every unit.  Keep these;
                    # only the nested role enclave is filtered below.
                    files.append(file)
                    continue
                # keystore/enclaves/elesim/<system>/<role>/...
                role = parts[4]
            if role in selected:
                files.append(file)
        return SecurityBundle(
            system_id=self.system_id,
            host_id=self.host_id,
            generation=self.generation,
            files=tuple(files),
        ).validate()

    @classmethod
    def from_directory(
        cls,
        *,
        system_id: str,
        host_id: str,
        generation: str,
        root: Path,
    ) -> "SecurityBundle":
        """Load one already-verified authority export into a bounded bundle."""

        candidate = root.expanduser()
        if candidate.is_symlink():
            raise ValueError(f"security bundle root must not be a symlink: {candidate}")
        source = candidate.resolve()
        if not source.is_dir():
            raise ValueError(f"security bundle root is not a directory: {source}")
        files: list[SecurityFile] = []
        for path in sorted(source.rglob("*")):
            if path.is_symlink():
                raise ValueError(f"security bundle must not contain symlinks: {path}")
            if path.is_dir():
                continue
            if not path.is_file():
                raise ValueError(f"security bundle contains a non-regular file: {path}")
            relative = path.relative_to(source).as_posix()
            parts = PurePosixPath(relative).parts
            if parts and parts[0] == "roles":
                relative = PurePosixPath("apps", *parts[1:]).as_posix()
            # The transport produces a fresh digest manifest after upload.
            # The authority manifest must be verified before this adapter runs.
            if relative == "manifest.json":
                continue
            if path.stat().st_size > MAX_BUNDLE_FILE_BYTES:
                raise ValueError(f"security bundle file is too large: {path}")
            source_mode = stat.S_IMODE(path.stat().st_mode)
            mode = 0o644 if source_mode == 0o644 else 0o600
            files.append(SecurityFile(relative, path.read_bytes(), mode))
        return cls(system_id, host_id, generation, tuple(files)).validate()


@dataclass(frozen=True)
class RemoteCommandResult:
    exit_status: int
    stdout: str = ""
    stderr: str = ""


@dataclass(frozen=True)
class RemoteCapabilities:
    docker: bool
    systemd: bool
    jetson: bool
    security_root_writable: bool
    architecture: str = ""

    def require_for(self, host: ManagedHost) -> None:
        if not self.security_root_writable:
            raise RuntimeError(f"host {host.host_id!r} cannot write its security root")
        if host.runtime_units and not self.docker:
            raise RuntimeError(f"host {host.host_id!r} requires Docker")
        if host.robot_units and (not self.jetson or not self.systemd):
            raise RuntimeError(
                f"Robot host {host.host_id!r} requires Jetson and systemd capabilities"
            )
        if any("sim" in unit.roles for unit in host.runtime_units):
            architecture = self.architecture.casefold()
            if architecture not in {"x86_64", "amd64"}:
                raise RuntimeError(
                    f"Sim on {host.host_id!r} requires an amd64 runtime image; "
                    f"detected architecture {self.architecture or 'unknown'!r}"
                )


@dataclass(frozen=True)
class HostActivationState:
    """Previous security/configuration and exact runtime state for rollback."""

    generation: str | None
    runtime_configuration: Mapping[str, Any]
    running_roles: tuple[str, ...] = ()
    # A mixed Jetson host can have an independently installed Robot and
    # Compose unit.  Keep their previous generations separately; a host-level
    # generation is retained above for the homogeneous schema/API.
    unit_generations: Mapping[str, str | None] = field(default_factory=dict)


class SshSession(Protocol):
    """Command/upload surface; lifecycle ownership stays with the connector."""

    def run(
        self, argv: Sequence[str], *, check: bool = True
    ) -> RemoteCommandResult: ...

    def run_streaming(
        self,
        argv: Sequence[str],
        *,
        output: CommandOutput,
        check: bool = True,
    ) -> RemoteCommandResult: ...

    def upload_bytes(self, path: PurePosixPath, content: bytes, mode: int) -> None: ...


class SshConnector(Protocol):
    def connect(self, endpoint: SshEndpoint) -> SshSession: ...


class RemoteLifecycle(Protocol):
    """Host-specific runtime operations executed over an authenticated session."""

    def preflight(
        self, session: SshSession, host: ManagedHost, security_root: PurePosixPath
    ) -> RemoteCapabilities: ...

    def runtime_network_check(
        self, session: SshSession, host: ManagedHost
    ) -> None: ...

    def runtime_launch_preflight(
        self, session: SshSession, host: ManagedHost
    ) -> None: ...

    def prepare_runtime_network(
        self,
        session: SshSession,
        host: ManagedHost,
        output: CommandOutput,
    ) -> str | None: ...

    def snapshot(
        self, session: SshSession, host: ManagedHost
    ) -> Mapping[str, Any]: ...

    def configure(
        self,
        session: SshSession,
        host: ManagedHost,
        generation: str | None,
        security_root: PurePosixPath,
    ) -> None: ...

    def restore(
        self,
        session: SshSession,
        host: ManagedHost,
        configuration: Mapping[str, Any],
    ) -> None: ...

    def stop(
        self, session: SshSession, host: ManagedHost, roles: Sequence[str]
    ) -> None: ...

    def cleanup_viewer(
        self, session: SshSession, host: ManagedHost, roles: Sequence[str]
    ) -> None: ...

    def start(
        self, session: SshSession, host: ManagedHost, roles: Sequence[str]
    ) -> None: ...

    def build(
        self, session: SshSession, host: ManagedHost, output: CommandOutput
    ) -> None: ...

    def launch(
        self,
        session: SshSession,
        host: ManagedHost,
        runtime_options: RuntimeLaunchOptions | None = None,
        viewer_user: str | None = None,
    ) -> None: ...

    def runtime_doctor(
        self,
        session: SshSession,
        host: ManagedHost,
        expected_peer_ids: Sequence[str],
        timeout_s: float,
    ) -> Mapping[str, Any]: ...

    def status(
        self, session: SshSession, host: ManagedHost
    ) -> Mapping[str, Any]: ...

    def verify(
        self,
        session: SshSession,
        host: ManagedHost,
        generation: str | None,
        running_roles: Sequence[str],
    ) -> None: ...


class HostOperations(Protocol):
    def preflight(self, host: ManagedHost) -> RemoteCapabilities: ...

    def runtime_network_check(self, host: ManagedHost) -> None: ...

    def runtime_launch_preflight(self, host: ManagedHost) -> None: ...

    def prepare_runtime_network(
        self, host: ManagedHost, output: CommandOutput
    ) -> str | None: ...

    def scoped_identity(self, host: ManagedHost, unit: DeploymentUnit) -> Mapping[str, str]: ...

    def scoped_releases(
        self, host: ManagedHost, unit: DeploymentUnit
    ) -> tuple[Mapping[str, Any], ...]: ...

    def scoped_instance_state(
        self, host: ManagedHost, unit: DeploymentUnit, system_id: str
    ) -> Mapping[str, Any] | None: ...

    def scoped_install_state(
        self, host: ManagedHost, unit: DeploymentUnit
    ) -> Mapping[str, Any]: ...

    def register_scoped_instance(
        self, host: ManagedHost, unit: DeploymentUnit, instance: Any,
        release: Any, bundle: SecurityBundle | None = None,
        *, replace_existing: bool = False,
    ) -> None: ...

    def remove_scoped_instance(
        self, host: ManagedHost, unit: DeploymentUnit, system_id: str
    ) -> None: ...

    def capture_state(self, host: ManagedHost) -> HostActivationState: ...

    def stage(self, host: ManagedHost, bundle: SecurityBundle) -> None: ...

    def discard_generation(self, host: ManagedHost, generation: str) -> None: ...

    def stop(self, host: ManagedHost, roles: Sequence[str] | None = None) -> None: ...

    def cleanup_viewer(
        self, host: ManagedHost, roles: Sequence[str] | None = None
    ) -> None: ...

    def activate(self, host: ManagedHost, generation: str) -> None: ...

    def configure_topology(self, host: ManagedHost) -> None: ...

    def start(self, host: ManagedHost, roles: Sequence[str] | None = None) -> None: ...

    def build(self, host: ManagedHost, output: CommandOutput) -> None: ...

    def launch(
        self,
        host: ManagedHost,
        runtime_options: RuntimeLaunchOptions | None = None,
    ) -> None: ...

    def runtime_doctor(
        self,
        host: ManagedHost,
        expected_peer_ids: Sequence[str],
        timeout_s: float = 5 * 60.0,
    ) -> Mapping[str, Any]: ...

    def status(self, host: ManagedHost) -> Mapping[str, Any]: ...

    def verify(
        self,
        host: ManagedHost,
        generation: str,
        running_roles: Sequence[str] = (),
    ) -> None: ...

    def verify_topology(
        self, host: ManagedHost, running_roles: Sequence[str] = ()
    ) -> None: ...

    def rollback(self, host: ManagedHost, previous: HostActivationState) -> None: ...

    def close(self) -> None: ...


class ParamikoConnector:
    """Connect with a pinned host key using OpenSSH or Tailscale SSH."""

    def __init__(
        self,
        *,
        timeout_s: float = 10.0,
        command_timeout_s: float = 120.0,
    ) -> None:
        if timeout_s <= 0 or command_timeout_s <= 0:
            raise ValueError("SSH timeouts must be positive")
        self._timeout_s = float(timeout_s)
        self._command_timeout_s = float(command_timeout_s)

    def connect(self, endpoint: SshEndpoint) -> SshSession:
        import paramiko

        endpoint.validate()
        if endpoint.uses_tailscale_ssh:
            return self._connect_tailscale(endpoint, paramiko)
        client = paramiko.SSHClient()
        client.set_missing_host_key_policy(
            _PinnedHostKeyPolicy(endpoint.pinned_fingerprint)
        )
        arguments: dict[str, Any] = {
            "hostname": endpoint.host,
            "port": int(endpoint.port),
            "username": endpoint.user,
            "password": None,
            "allow_agent": endpoint.uses_agent,
            "look_for_keys": False,
            "timeout": self._timeout_s,
            "banner_timeout": self._timeout_s,
            "auth_timeout": self._timeout_s,
        }
        if not endpoint.uses_agent:
            arguments["key_filename"] = str(
                resolve_ssh_identity_path(endpoint.identity_file)
            )
        try:
            client.connect(**arguments)
            _enable_ssh_keepalive(client)
        except BaseException:
            client.close()
            raise
        return _ParamikoSession(
            client,
            command_timeout_s=self._command_timeout_s,
        )

    def _connect_tailscale(self, endpoint: SshEndpoint, paramiko: object) -> SshSession:
        """Authenticate through the Tailscale SSH server without a local key.

        Tailscale SSH presents a normal SSH transport on TCP/22, but accepts
        Tailscale identity rather than an OpenSSH key.  Paramiko's regular
        ``SSHClient.connect`` path never sends the required ``none`` request,
        so establish the transport explicitly and then hand it to the normal
        session wrapper.  The compatibility password fallback is the method
        documented by Tailscale for clients that cannot use ``auth_none``;
        the value is a disposable placeholder, never a stored credential.
        """

        connection: object | None = None
        proxy_connection: _ParamikoProxySocket | None = None
        transport: object | None = None
        try:
            proxy_command = tailscale_proxy_command(
                endpoint.host,
                int(endpoint.port),
                force=True,
            )
            if proxy_command is None:
                connection = socket.create_connection(
                    (endpoint.host, int(endpoint.port)), timeout=self._timeout_s
                )
            else:
                from paramiko.proxy import ProxyCommand

                proxy_connection = _ParamikoProxySocket(ProxyCommand(proxy_command))
                connection = proxy_connection
            transport = paramiko.Transport(connection)  # type: ignore[attr-defined]
            # Paramiko's Transport defaults auth_timeout to None.  Tailscale
            # ``action=check`` can otherwise leave a headless rollout waiting
            # forever while the user has not approved the interactive check.
            transport.auth_timeout = self._timeout_s  # type: ignore[attr-defined]
            transport.start_client(timeout=self._timeout_s)  # type: ignore[attr-defined]
            key = transport.get_remote_server_key()  # type: ignore[attr-defined]
            _verify_pinned_host_key(endpoint, key)
            authentication_error = getattr(
                paramiko, "AuthenticationException", None
            )
            authentication_errors: tuple[type[BaseException], ...] = (
                (authentication_error,)
                if isinstance(authentication_error, type)
                and issubclass(authentication_error, BaseException)
                else ()
            )
            authenticated = False
            try:
                transport.auth_none(endpoint.user)  # type: ignore[attr-defined]
                authenticated = bool(transport.is_authenticated())  # type: ignore[attr-defined]
            except authentication_errors:
                # Some SSH libraries cannot issue SSH_MSG_USERAUTH_NONE to the
                # Tailscale server.  Tailscale documents ``user+password`` with
                # any password as its compatibility spelling for those clients.
                authenticated = False
            if not authenticated:
                try:
                    transport.auth_password(  # type: ignore[attr-defined]
                        f"{endpoint.user}+password", "tailscale"
                    )
                    authenticated = bool(transport.is_authenticated())  # type: ignore[attr-defined]
                except authentication_errors as exc:
                    raise SshAuthenticationError(
                        f"Tailscale SSH authentication failed for "
                        f"{endpoint.user}@{endpoint.host}. "
                        "If the ACL uses action=check, approve one interactive "
                        "Tailscale SSH re-authentication first and verify that "
                        "the ACL permits this user. Tailscale SSH uses port 22 "
                        "and does not use a private key path."
                    ) from exc
            if not authenticated:
                raise SshAuthenticationError(
                    f"Tailscale SSH did not authenticate {endpoint.user}@{endpoint.host}. "
                    "If the ACL uses action=check, approve one interactive "
                    "Tailscale SSH re-authentication first and verify the ACL user."
                )
            if proxy_connection is not None:
                proxy_connection.mark_established()
            client = paramiko.SSHClient()  # type: ignore[attr-defined]
            # SSHClient owns and closes this transport through its normal close
            # path.  This is the only private Paramiko attribute we rely on;
            # it has been stable since Paramiko's Transport split.
            client._transport = transport  # type: ignore[attr-defined]
            transport.set_keepalive(15)  # type: ignore[attr-defined]
            transport = None
            # The Transport now owns the socket/proxy process.  Clearing both
            # local references prevents the cleanup block from closing the
            # live connection before the returned session opens its first
            # command or SFTP channel.
            connection = None
            return _ParamikoSession(
                client,
                command_timeout_s=self._command_timeout_s,
            )
        except (HostKeyVerificationError, SshAuthenticationError):
            raise
        except (TimeoutError, socket.timeout) as exc:
            raise SshConnectionError(
                f"SSH connection to {endpoint.host}:{endpoint.port} timed out. "
                "Check the remote SSH service, Tailscale SSH/ACL, and the "
                "connection-manager network path."
            ) from exc
        except OSError as exc:
            detail = proxy_failure_detail(connection, exc)
            raise SshConnectionError(
                f"SSH connection to {endpoint.host}:{endpoint.port} failed: {detail}. "
                "Check the remote SSH service, Tailscale SSH/ACL, and the "
                "connection-manager network path."
            ) from exc
        except Exception as exc:
            ssh_error = getattr(paramiko, "SSHException", None)
            if not isinstance(ssh_error, type) or not isinstance(exc, ssh_error):
                raise
            detail = proxy_failure_detail(connection, exc)
            raise SshConnectionError(
                f"SSH connection to {endpoint.host}:{endpoint.port} failed: {detail}. "
                "Check the remote SSH service, Tailscale SSH/ACL, and the "
                "connection-manager network path."
            ) from exc
        finally:
            if transport is not None:
                transport.close()  # type: ignore[attr-defined]
            elif connection is not None:
                # Transport owns the socket after a successful hand-off.  On a
                # failed socket/transport construction, close the raw socket.
                try:
                    connection.close()  # type: ignore[attr-defined]
                except Exception:
                    pass


class _PinnedHostKeyPolicy:
    def __init__(self, expected: str) -> None:
        self._expected = expected

    def missing_host_key(self, _client: object, hostname: str, key: object) -> None:
        _verify_pinned_host_key_values(hostname, key, self._expected)


def _verify_pinned_host_key(endpoint: SshEndpoint, key: object) -> None:
    _verify_pinned_host_key_values(endpoint.host, key, endpoint.pinned_fingerprint)


def _verify_pinned_host_key_values(hostname: str, key: object, expected: str) -> None:
    try:
        key_bytes = key.asbytes()  # type: ignore[attr-defined]
    except (AttributeError, TypeError) as exc:
        raise HostKeyVerificationError(
            f"SSH server {hostname!r} supplied an invalid host key"
        ) from exc
    actual = ssh_sha256_fingerprint(key_bytes)
    if not hmac.compare_digest(actual, expected):
        raise HostKeyVerificationError(
            f"SSH host key mismatch for {hostname!r}: expected "
            f"{expected}, received {actual}"
        )


class _ParamikoSession:
    def __init__(self, client: object, *, command_timeout_s: float) -> None:
        self._client = client
        self._command_timeout_s = float(command_timeout_s)
        self._sftp: object | None = None

    def __enter__(self) -> "_ParamikoSession":
        return self

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        if self._sftp is not None:
            self._sftp.close()  # type: ignore[attr-defined]
        self._client.close()  # type: ignore[attr-defined]

    def run(
        self, argv: Sequence[str], *, check: bool = True
    ) -> RemoteCommandResult:
        return self._run(argv, check=check, output=None)

    def run_streaming(
        self,
        argv: Sequence[str],
        *,
        output: CommandOutput,
        check: bool = True,
    ) -> RemoteCommandResult:
        return self._run(argv, check=check, output=output)

    def _run(
        self,
        argv: Sequence[str],
        *,
        check: bool,
        output: CommandOutput | None,
    ) -> RemoteCommandResult:
        if not argv or any("\x00" in str(value) for value in argv):
            raise ValueError("remote command argv must be non-empty and contain no NUL")
        command = shlex.join([str(value) for value in argv])
        timeout_s = _command_timeout(argv, self._command_timeout_s)
        _stdin, stdout, stderr = self._client.exec_command(  # type: ignore[attr-defined]
            command,
            timeout=timeout_s,
        )
        channel = stdout.channel
        if output is not None and all(
            hasattr(channel, name)
            for name in (
                "recv_ready",
                "recv",
                "recv_stderr_ready",
                "recv_stderr",
                "exit_status_ready",
            )
        ):
            result = _read_paramiko_channel(
                channel,
                output=output,
                timeout_s=timeout_s,
                command=command,
            )
            if check and result.exit_status != 0:
                raise RemoteCommandError(argv, result)
            return result
        streams: dict[str, tuple[bytes, bool]] = {}
        failures: list[Exception] = []

        def drain(name: str, stream: object) -> None:
            try:
                streams[name] = _read_stream_tail(stream, name=name, output=output)
            except Exception as exc:
                failures.append(exc)

        workers = [
            threading.Thread(target=drain, args=("stdout", stdout), daemon=True),
            threading.Thread(target=drain, args=("stderr", stderr), daemon=True),
        ]
        for worker in workers:
            worker.start()
        deadline = time.monotonic() + timeout_s
        for worker in workers:
            worker.join(max(0.0, deadline - time.monotonic()))
        if any(worker.is_alive() for worker in workers):
            self._client.close()  # type: ignore[attr-defined]
            raise TimeoutError(
                f"remote command timed out after {timeout_s:.1f} seconds: {command}"
            )
        if failures:
            raise failures[0]
        stdout_raw, stdout_truncated = streams["stdout"]
        stderr_raw, stderr_truncated = streams["stderr"]
        result = RemoteCommandResult(
            exit_status=int(stdout.channel.recv_exit_status()),
            stdout=_decode_remote_output(stdout_raw, stdout_truncated),
            stderr=_decode_remote_output(stderr_raw, stderr_truncated),
        )
        if check and result.exit_status != 0:
            raise RemoteCommandError(argv, result)
        return result

    def upload_bytes(self, path: PurePosixPath, content: bytes, mode: int) -> None:
        if not path.is_absolute() or ".." in path.parts:
            raise ValueError("remote upload path must be absolute and contained")
        if len(content) > MAX_BUNDLE_FILE_BYTES:
            raise ValueError("remote upload exceeds per-file limit")
        sftp = self._get_sftp()
        _sftp_makedirs(sftp, path.parent)
        with sftp.file(str(path), "wb") as handle:  # type: ignore[attr-defined]
            handle.write(content)
            handle.flush()
        sftp.chmod(str(path), mode)  # type: ignore[attr-defined]

    def _get_sftp(self) -> object:
        if self._sftp is None:
            self._sftp = self._client.open_sftp()  # type: ignore[attr-defined]
            self._sftp.get_channel().settimeout(  # type: ignore[attr-defined]
                self._command_timeout_s
            )
        return self._sftp


def _enable_ssh_keepalive(client: object) -> None:
    getter = getattr(client, "get_transport", None)
    if getter is None:
        # Structurally typed test clients may omit Paramiko's transport API.
        return
    transport = getter()
    if transport is None:
        raise SshConnectionError("SSH connection has no active transport")
    transport.set_keepalive(15)  # type: ignore[attr-defined]


def _read_stream_tail(
    stream: object,
    *,
    name: str = "stdout",
    output: CommandOutput | None = None,
) -> tuple[bytes, bool]:
    retained = bytearray()
    truncated = False
    while True:
        chunk = stream.read(32 * 1024)  # type: ignore[attr-defined]
        if not chunk:
            break
        if output is not None:
            output(name, chunk.decode("utf-8", errors="replace"))
        retained.extend(chunk)
        if len(retained) > MAX_REMOTE_OUTPUT_BYTES:
            del retained[: len(retained) - MAX_REMOTE_OUTPUT_BYTES]
            truncated = True
    return bytes(retained), truncated


def _read_paramiko_channel(
    channel: object,
    *,
    output: CommandOutput,
    timeout_s: float,
    command: str,
) -> RemoteCommandResult:
    """Drain an SSH channel incrementally without ChannelFile read buffering."""

    retained = {"stdout": bytearray(), "stderr": bytearray()}
    truncated = {"stdout": False, "stderr": False}
    deadline = time.monotonic() + timeout_s

    def consume(name: str, chunk: bytes) -> None:
        output(name, chunk.decode("utf-8", errors="replace"))
        tail = retained[name]
        tail.extend(chunk)
        if len(tail) > MAX_REMOTE_OUTPUT_BYTES:
            del tail[: len(tail) - MAX_REMOTE_OUTPUT_BYTES]
            truncated[name] = True

    while True:
        progressed = False
        while channel.recv_ready():  # type: ignore[attr-defined]
            consume("stdout", channel.recv(32 * 1024))  # type: ignore[attr-defined]
            progressed = True
        while channel.recv_stderr_ready():  # type: ignore[attr-defined]
            consume("stderr", channel.recv_stderr(32 * 1024))  # type: ignore[attr-defined]
            progressed = True
        if (
            channel.exit_status_ready()  # type: ignore[attr-defined]
            and not channel.recv_ready()  # type: ignore[attr-defined]
            and not channel.recv_stderr_ready()  # type: ignore[attr-defined]
        ):
            break
        if time.monotonic() >= deadline:
            close = getattr(channel, "close", None)
            if close is not None:
                close()
            raise TimeoutError(
                f"remote command timed out after {timeout_s:.1f} seconds: {command}"
            )
        if not progressed:
            time.sleep(0.05)

    return RemoteCommandResult(
        exit_status=int(channel.recv_exit_status()),  # type: ignore[attr-defined]
        stdout=_decode_remote_output(
            bytes(retained["stdout"]), truncated["stdout"]
        ),
        stderr=_decode_remote_output(
            bytes(retained["stderr"]), truncated["stderr"]
        ),
    )


def _decode_remote_output(content: bytes, truncated: bool) -> str:
    rendered = content.decode("utf-8", errors="replace")
    if not truncated:
        return rendered
    return "[earlier remote output truncated]\n" + rendered


def _managed_turn_from_state(
    state: Mapping[str, Any], host: ManagedHost
) -> dict[str, str]:
    """Read the non-secret managed TURN endpoint from the Sim installation.

    Coturn is a Sim-owned service.  The connection topology therefore does
    not store its URL, realm, public host, or secret path.  A freshly installed
    managed relay has an empty URL/public host; in that one pending state the
    manager derives the endpoint from the Sim host's current DDS address and
    writes the completed value back through ``elesim-net configure``.
    """

    network = state.get("network")
    turn = state.get("turn")
    if not isinstance(network, Mapping) or not isinstance(turn, Mapping):
        raise RuntimeError(
            f"managed Coturn state is missing on {host.host_id}; "
            "install/update the Sim runtime with managed Coturn first"
        )
    mode = str(turn.get("mode", "")).strip()
    raw_urls = network.get("turn_urls", ())
    if isinstance(raw_urls, (str, bytes, bytearray)) or not isinstance(
        raw_urls, Sequence
    ):
        raw_urls = ()
    urls = tuple(
        str(value).strip() for value in raw_urls if str(value).strip()
    )
    realm = str(turn.get("realm", "")).strip()
    public_host = str(turn.get("public_host", "")).strip()
    secret_file = str(turn.get("secret_file", "")).strip()
    secret_path = PurePosixPath(secret_file)
    sim_roots = tuple(
        PurePosixPath(unit.install_root)
        for unit in host.units
        if "sim" in unit.roles
    )
    secret_is_contained = False
    if secret_path.is_absolute() and ".." not in secret_path.parts:
        for root in sim_roots:
            try:
                secret_path.relative_to(root)
            except ValueError:
                continue
            secret_is_contained = True
            break
    if mode == "managed" and not urls and not public_host:
        # Fresh general installs intentionally have no mutable relay endpoint.
        # The topology owns the current Sim address, so derive the endpoint at
        # the manager boundary and persist it through ``elesim-net configure``.
        public_host = host.dds.address
        url_host = f"[{public_host}]" if ":" in public_host else public_host
        urls = (f"turn:{url_host}:3478?transport=udp",)
    if (
        mode != "managed"
        or len(urls) != 1
        or not realm
        or not public_host
        or not secret_file
        or not secret_is_contained
    ):
        raise RuntimeError(
            f"Sim host {host.host_id} has no complete managed Coturn configuration; "
            "install/update the Sim runtime with SROS2-managed Coturn first "
            "and keep its secret file under the Sim installation root"
        )
    return {
        "turn_url": urls[0],
        "turn_realm": realm,
        "turn_public_host": public_host,
        "turn_secret_file": secret_file,
    }


def _scoped_turn_from_instance_state(
    state: Mapping[str, Any], host: ManagedHost, *, system_id: str = ""
) -> dict[str, str] | None:
    """Read TURN exclusively from one registered instance state.

    Scoped lifecycle must never consult the install-wide ``elesim-net show``
    TURN fields: another instance (or the legacy install) may own those.
    """

    turn = state.get("turn")
    if not isinstance(turn, Mapping):
        raise RuntimeError(f"scoped instance TURN state is missing on {host.host_id}")
    mode = str(turn.get("mode", "none")).strip()
    if mode not in {"none", "managed", "external"}:
        raise RuntimeError(f"unsupported scoped instance TURN mode on {host.host_id}")
    raw_urls = state.get("turn_urls", ())
    urls = tuple(str(value).strip() for value in raw_urls) if isinstance(raw_urls, Sequence) and not isinstance(raw_urls, (str, bytes, bytearray)) else ()
    if mode == "none":
        if urls:
            raise RuntimeError("scoped instance TURN URLs are set while TURN is disabled")
        return None
    if len(urls) != 1:
        raise RuntimeError(f"scoped instance {mode} TURN requires exactly one URL")
    if mode == "managed":
        realm = str(turn.get("realm", "")).strip()
        public_host = str(turn.get("public_host", "")).strip()
        secret_file = str(turn.get("secret_file", "")).strip()
        if not realm or not public_host or not secret_file:
            raise RuntimeError("scoped managed TURN state is incomplete")
        if system_id:
            expected_secret = (
                PurePosixPath(host.primary_unit.install_root)
                / "instances" / system_id / "secrets" / "turn.secret"
            )
            if PurePosixPath(secret_file) != expected_secret:
                raise RuntimeError(
                    "scoped managed TURN secret is outside the instance secret namespace"
                )
        return {
            "turn_mode": "managed", "turn_url": urls[0], "turn_realm": realm,
            "turn_public_host": public_host, "turn_secret_file": secret_file,
        }
    credential_file = str(turn.get("credential_file", "")).strip()
    if not credential_file:
        raise RuntimeError("scoped external TURN state has no credential file")
    return {
        "turn_mode": "external", "turn_url": urls[0],
        "turn_credential_file": credential_file,
    }


def _remote_path_contains_symlink(session: SshSession, path: str) -> bool:
    """Check a remote path and every existing ancestor without following it."""

    current = PurePosixPath(path)
    while True:
        result = session.run(("test", "-L", str(current)), check=False)
        if result.exit_status == 0:
            return True
        if result.exit_status != 1:
            raise RuntimeError(
                f"could not inspect managed Coturn secret path component: {current}"
            )
        if current == PurePosixPath("/"):
            return False
        current = current.parent


class SshHostOperations:
    """Deploy one host bundle through a role-specific remote lifecycle."""

    def __init__(
        self,
        connector: SshConnector,
        lifecycle: RemoteLifecycle,
        topology: ConnectionTopology,
        *,
        security_root: str | None = None,
    ) -> None:
        self._connector = connector
        self._lifecycle = lifecycle
        self._topology = topology.validate()
        self._security_root_override = (
            None if security_root is None else _safe_remote_root(security_root)
        )
        self._session: SshSession | None = None

    def preflight(self, host: ManagedHost) -> RemoteCapabilities:
        security_root = self._security_root_for(host)
        with self._connect(host) as session:
            return self._lifecycle.preflight(session, host, security_root)

    def runtime_network_check(self, host: ManagedHost) -> None:
        """Run the cheap direct-interface probe in the runtime namespace."""

        with self._connect(host) as session:
            self._lifecycle.runtime_network_check(session, host)

    def runtime_launch_preflight(self, host: ManagedHost) -> None:
        """Validate the installed files through the normal launch guard."""

        with self._connect(host) as session:
            self._lifecycle.runtime_launch_preflight(session, host)

    def prepare_runtime_network(
        self, host: ManagedHost, output: CommandOutput
    ) -> str | None:
        """Start/enroll installation-owned network infrastructure if needed."""

        with self._connect(host) as session:
            return self._lifecycle.prepare_runtime_network(session, host, output)

    def scoped_identity(self, host: ManagedHost, unit: DeploymentUnit) -> Mapping[str, str]:
        with self._connect(host) as session:
            return self._lifecycle.scoped_identity(session, unit)

    def scoped_releases(
        self, host: ManagedHost, unit: DeploymentUnit
    ) -> tuple[Mapping[str, Any], ...]:
        with self._connect(host) as session:
            return self._lifecycle.scoped_releases(session, unit)

    def scoped_instance_state(
        self, host: ManagedHost, unit: DeploymentUnit, system_id: str
    ) -> Mapping[str, Any] | None:
        with self._connect(host) as session:
            return self._lifecycle.scoped_instance_state(session, unit, system_id)

    def scoped_install_state(
        self, host: ManagedHost, unit: DeploymentUnit
    ) -> Mapping[str, Any]:
        with self._connect(host) as session:
            return self._lifecycle.scoped_install_state(session, unit)

    def register_scoped_instance(
        self, host: ManagedHost, unit: DeploymentUnit, instance: Any,
        release: Any, bundle: SecurityBundle | None = None,
        *, replace_existing: bool = False,
    ) -> None:
        with self._connect(host) as session:
            self._lifecycle.register_scoped_instance(
                session, host, unit, instance, release, bundle,
                replace_existing=replace_existing,
            )

    def remove_scoped_instance(
        self, host: ManagedHost, unit: DeploymentUnit, system_id: str
    ) -> None:
        with self._connect(host) as session:
            self._lifecycle.remove_scoped_instance(session, host, unit, system_id)

    def capture_state(self, host: ManagedHost) -> HostActivationState:
        with self._connect(host) as session:
            unit_generations: dict[str, str | None] = {}
            for unit in host.units:
                security_root = self._security_root_for_unit(host, unit)
                result = session.run(
                    ("readlink", str(security_root / "current")), check=False
                )
                generation: str | None = None
                if result.exit_status == 0 and result.stdout.strip():
                    generation = PurePosixPath(result.stdout.strip()).name
                    _safe_generation(generation)
                unit_generations[unit.unit_id] = generation
            configuration = dict(self._lifecycle.snapshot(session, host))
            status = dict(self._lifecycle.status(session, host))
        generation_values = {value for value in unit_generations.values() if value is not None}
        if len(generation_values) > 1:
            raise RuntimeError(
                f"security generations differ between units on {host.host_id!r}: "
                f"{sorted(generation_values)!r}"
            )
        generation = next(iter(generation_values), None)
        running_raw = status.get("running_roles", ())
        running_roles = tuple(
            role for role in host.roles if role in set(str(value) for value in running_raw)
        )
        return HostActivationState(
            generation,
            configuration,
            running_roles,
            unit_generations=unit_generations,
        )

    def stage(self, host: ManagedHost, bundle: SecurityBundle) -> None:
        _check_bundle_target(host, bundle)
        with self._connect(host) as session:
            for unit in host.units:
                unit_bundle = bundle.for_roles(unit.roles)
                _stage_unit_bundle(
                    session,
                    host,
                    unit,
                    self._security_root_for_unit(host, unit),
                    unit_bundle,
                )

    def discard_generation(self, host: ManagedHost, generation: str) -> None:
        _safe_generation(generation)
        with self._connect(host) as session:
            for unit in host.units:
                security_root = self._security_root_for_unit(host, unit)
                target = security_root / "generations" / generation
                current = session.run(
                    ("readlink", str(security_root / "current")), check=False
                )
                if current.exit_status == 0 and (
                    PurePosixPath(current.stdout.strip()).name == generation
                ):
                    raise RuntimeError(
                        f"refusing to discard active generation on {host.host_id}: "
                        f"{generation}"
                    )
                session.run(("rm", "-rf", "--", str(target)), check=False)

    def stop(self, host: ManagedHost, roles: Sequence[str] | None = None) -> None:
        selected = _selected_roles(host, roles)
        if not selected:
            return
        with self._connect(host) as session:
            self._lifecycle.stop(session, host, selected)

    def cleanup_viewer(
        self, host: ManagedHost, roles: Sequence[str] | None = None
    ) -> None:
        selected = _selected_roles(host, roles)
        if "sim" not in selected:
            return
        with self._connect(host) as session:
            self._lifecycle.cleanup_viewer(session, host, selected)

    def activate(self, host: ManagedHost, generation: str) -> None:
        _safe_generation(generation)
        with self._connect(host) as session:
            installed = self._lifecycle.snapshot(session, host)
            installed_units = installed.get("units")
            for unit in host.units:
                security_root = self._security_root_for_unit(host, unit)
                _activate_unit_generation(
                    session,
                    host,
                    unit,
                    security_root,
                    generation=generation,
                )
                unit_state = (
                    installed_units.get(unit.unit_id)
                    if isinstance(installed_units, Mapping)
                    else installed
                )
                installed_roles = (
                    tuple(str(role) for role in unit_state.get("roles", ()))
                    if isinstance(unit_state, Mapping)
                    else ()
                )
                inactive_roles = tuple(
                    role for role in installed_roles if role not in unit.roles
                )
                if inactive_roles:
                    self._sync_app_views(
                        session,
                        host,
                        security_root,
                        generation=None,
                        roles=inactive_roles,
                    )
            self._lifecycle.configure(
                session,
                host,
                generation,
                self._security_root_for(host),
            )

    def configure_topology(self, host: ManagedHost) -> None:
        if self._topology.security_profile != "trusted-network":
            raise ValueError(
                "bundle-free topology configuration is limited to trusted-network"
            )
        security_root = self._security_root_for(host)
        with self._connect(host) as session:
            self._lifecycle.configure(session, host, None, security_root)

    def start(self, host: ManagedHost, roles: Sequence[str] | None = None) -> None:
        selected = _selected_roles(host, roles)
        if not selected:
            return
        with self._connect(host) as session:
            self._lifecycle.start(session, host, selected)

    def build(self, host: ManagedHost, output: CommandOutput) -> None:
        with self._connect(host) as session:
            self._lifecycle.build(session, host, output)

    def launch(
        self,
        host: ManagedHost,
        runtime_options: RuntimeLaunchOptions | None = None,
    ) -> None:
        viewer_user = (
            host.ssh.user
            if not host.local and host.ssh is not None
            else None
        )
        with self._connect(host) as session:
            if runtime_options is None:
                if viewer_user is None:
                    self._lifecycle.launch(session, host)
                else:
                    self._lifecycle.launch(
                        session, host, viewer_user=viewer_user
                    )
            else:
                if viewer_user is None:
                    self._lifecycle.launch(session, host, runtime_options)
                else:
                    self._lifecycle.launch(
                        session,
                        host,
                        runtime_options,
                        viewer_user=viewer_user,
                    )

    def runtime_doctor(
        self,
        host: ManagedHost,
        expected_peer_ids: Sequence[str],
        timeout_s: float = 5 * 60.0,
    ) -> Mapping[str, Any]:
        if timeout_s <= 0:
            raise ValueError("runtime doctor timeout must be positive")
        with self._connect(host) as session:
            return self._lifecycle.runtime_doctor(
                session,
                host,
                tuple(expected_peer_ids),
                float(timeout_s),
            )

    def status(self, host: ManagedHost) -> Mapping[str, Any]:
        with self._connect(host) as session:
            result = dict(self._lifecycle.status(session, host))
        result.setdefault("host_id", host.host_id)
        result.setdefault("roles", list(host.roles))
        return result

    def verify(
        self,
        host: ManagedHost,
        generation: str,
        running_roles: Sequence[str] = (),
    ) -> None:
        _safe_generation(generation)
        with self._connect(host) as session:
            self._lifecycle.verify(session, host, generation, running_roles)

    def verify_topology(
        self, host: ManagedHost, running_roles: Sequence[str] = ()
    ) -> None:
        with self._connect(host) as session:
            self._lifecycle.verify(session, host, None, running_roles)

    def rollback(self, host: ManagedHost, previous: HostActivationState) -> None:
        unit_generations = dict(previous.unit_generations)
        if not unit_generations:
            unit_generations = {
                unit.unit_id: previous.generation for unit in host.units
            }
        with self._connect(host) as session:
            for unit in host.units:
                generation = unit_generations.get(unit.unit_id)
                security_root = self._security_root_for_unit(host, unit)
                previous_units = previous.runtime_configuration.get("units")
                previous_state = (
                    previous_units[unit.unit_id]
                    if isinstance(previous_units, Mapping)
                    else previous.runtime_configuration
                )
                previous_roles = tuple(
                    previous_state.get("assigned_roles") or previous_state.get("roles", unit.roles)
                )
                if generation is not None:
                    _activate_unit_generation(
                        session,
                        host,
                        unit,
                        security_root,
                        generation=generation,
                        roles=previous_roles,
                    )
                    self._sync_app_views(
                        session, host, security_root, generation=None,
                        roles=tuple(role for role in unit.roles if role not in previous_roles),
                    )
                else:
                    session.run(
                        ("rm", "-f", "--", str(security_root / "current")),
                        check=False,
                    )
                    self._sync_app_views(
                        session,
                        host,
                        security_root,
                        generation=None,
                        roles=unit.roles,
                    )
        with self._connect(host) as session:
            self._lifecycle.restore(session, host, previous.runtime_configuration)

    @staticmethod
    def _sync_app_views(
        session: SshSession,
        host: ManagedHost,
        security_root: PurePosixPath,
        *,
        generation: str | None,
        roles: Sequence[str] | None = None,
    ) -> None:
        """Refresh stable app roots in place while application services stop."""

        selected_roles = tuple(host.roles if roles is None else roles)

        apps_root = security_root / "apps"
        if session.run(("test", "-L", str(apps_root)), check=False).exit_status == 0:
            raise RuntimeError(f"app keystore root is a symlink: {apps_root}")
        session.run(("mkdir", "-p", str(apps_root)))
        session.run(("chmod", "0700", str(apps_root)))

        sources: dict[str, PurePosixPath] = {}
        for role in selected_roles:
            destination = apps_root / role
            if session.run(
                ("test", "-L", str(destination)), check=False
            ).exit_status == 0:
                raise RuntimeError(f"app keystore is a symlink: {destination}")
            session.run(("mkdir", "-p", str(destination)))
            session.run(("chmod", "0700", str(destination)))
            for name in ("public", "enclaves"):
                child = destination / name
                if session.run(
                    ("test", "-L", str(child)), check=False
                ).exit_status == 0:
                    raise RuntimeError(f"app keystore child is a symlink: {child}")
            if generation is None:
                continue
            source = (
                security_root
                / "generations"
                / generation
                / "apps"
                / role
                / "keystore"
            )
            session.run(("test", "-d", str(source / "public")))
            session.run(("test", "-d", str(source / "enclaves")))
            sources[role] = source

        for role in selected_roles:
            destination = apps_root / role
            session.run(
                (
                    "rm",
                    "-rf",
                    "--",
                    str(destination / "public"),
                    str(destination / "enclaves"),
                )
            )
            source = sources.get(role)
            if source is None:
                continue
            session.run(
                ("cp", "-a", str(source / "public"), str(destination / "public"))
            )
            session.run(
                (
                    "cp",
                    "-a",
                    str(source / "enclaves"),
                    str(destination / "enclaves"),
                )
            )

    def _security_root_for(self, host: ManagedHost) -> PurePosixPath:
        if self._topology.host(host.host_id) != host:
            raise ValueError(f"host {host.host_id!r} does not match the managed topology")
        if self._security_root_override is not None:
            return self._security_root_override
        return self._security_root_for_unit(host, host.primary_unit)

    def _security_root_for_unit(
        self, host: ManagedHost, unit: DeploymentUnit
    ) -> PurePosixPath:
        if self._topology.host(host.host_id) != host:
            raise ValueError(f"host {host.host_id!r} does not match the managed topology")
        if getattr(self._lifecycle, "scoped", False) and unit.install_mode == "container":
            return (
                PurePosixPath(unit.install_root)
                / "instances"
                / self._topology.system_id
                / "security"
            )
        if self._security_root_override is not None and len(host.units) == 1:
            return self._security_root_override
        return _safe_remote_root(str(PurePosixPath(unit.install_root) / "security"))

    def _connect(self, host: ManagedHost) -> SshSession:
        if host.local or host.ssh is None:
            raise ValueError(
                f"SshHostOperations cannot operate local host {host.host_id!r}; "
                "inject local HostOperations instead"
        )
        if self._session is None:
            # The management endpoint is independent from the DDS runtime
            # endpoint.  Docker Desktop sidecars have their own tailnet IP.
            self._session = self._connector.connect(host.ssh)
        return _BorrowedSession(self._session)

    def close(self) -> None:
        session, self._session = self._session, None
        if session is not None:
            close_context = getattr(session, "__exit__", None)
            if callable(close_context):
                close_context(None, None, None)
                return
            close_socket = getattr(session, "close", None)
            if callable(close_socket):
                close_socket()


class LocalHostOperations(SshHostOperations):
    """Apply the same transaction locally without shell interpolation or SSH."""

    def __init__(
        self,
        lifecycle: RemoteLifecycle,
        topology: ConnectionTopology,
        *,
        security_root: str | None = None,
        timeout_s: float = 30.0,
    ) -> None:
        super().__init__(
            _UnavailableConnector(),
            lifecycle,
            topology,
            security_root=security_root,
        )
        if timeout_s <= 0:
            raise ValueError("local command timeout must be positive")
        self._local_timeout_s = float(timeout_s)

    def _connect(self, host: ManagedHost) -> SshSession:
        if not host.local:
            raise ValueError(
                f"LocalHostOperations cannot operate remote host {host.host_id!r}"
            )
        return _LocalSession(timeout_s=self._local_timeout_s)


class _BorrowedSession:
    """Expose one job-scoped SSH session through the existing `with` sites."""

    def __init__(self, session: SshSession) -> None:
        self._session = session

    def __enter__(self) -> SshSession:
        return self._session

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        return None


class _UnavailableConnector:
    def connect(self, _endpoint: SshEndpoint) -> SshSession:
        raise AssertionError("LocalHostOperations must not use an SSH connector")


class _LocalSession:
    def __init__(self, *, timeout_s: float) -> None:
        self._timeout_s = timeout_s

    def __enter__(self) -> "_LocalSession":
        return self

    def __exit__(self, _exc_type: object, _exc: object, _traceback: object) -> None:
        return None

    def run(
        self, argv: Sequence[str], *, check: bool = True
    ) -> RemoteCommandResult:
        return self._run(argv, check=check, output=None)

    def run_streaming(
        self,
        argv: Sequence[str],
        *,
        output: CommandOutput,
        check: bool = True,
    ) -> RemoteCommandResult:
        return self._run(argv, check=check, output=output)

    def _run(
        self,
        argv: Sequence[str],
        *,
        check: bool,
        output: CommandOutput | None,
    ) -> RemoteCommandResult:
        values = tuple(str(value) for value in argv)
        if not values or any("\x00" in value for value in values):
            raise ValueError("local command argv must be non-empty and contain no NUL")
        timeout_s = _command_timeout(values, self._timeout_s)
        helper_socket = os.environ.get("ELESIM_HOST_HELPER_SOCKET", "").strip()
        if helper_socket and (
            values[0] == "docker"
            or Path(values[0]).name
            in {
                "elesim-compose",
                "elesim-net",
                "elesim-tailscale",
                "elesim-status",
                "elesim-up",
                "elesim-instance",
                "elesim-instance-register",
                "elesim-viewer-cleanup",
            }
        ):
            result = _run_through_host_helper(
                values,
                socket_path=helper_socket,
                output=output,
                timeout_s=timeout_s,
            )
            if check and result.exit_status != 0:
                raise RemoteCommandError(values, result)
            return result
        if output is not None:
            result = _run_local_streaming(
                values, output=output, timeout_s=timeout_s
            )
            if check and result.exit_status != 0:
                raise RemoteCommandError(values, result)
            return result
        completed = subprocess.run(
            values,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            timeout=timeout_s,
        )
        if (
            len(completed.stdout) > MAX_REMOTE_OUTPUT_BYTES
            or len(completed.stderr) > MAX_REMOTE_OUTPUT_BYTES
        ):
            raise RemoteCommandError(
                values,
                RemoteCommandResult(1, stderr="local command output exceeded limit"),
            )
        result = RemoteCommandResult(
            completed.returncode,
            completed.stdout.decode("utf-8", errors="replace"),
            completed.stderr.decode("utf-8", errors="replace"),
        )
        if check and result.exit_status != 0:
            raise RemoteCommandError(values, result)
        return result

    def upload_bytes(self, path: PurePosixPath, content: bytes, mode: int) -> None:
        if not path.is_absolute() or ".." in path.parts:
            raise ValueError("local deployment path must be absolute and contained")
        if len(content) > MAX_BUNDLE_FILE_BYTES:
            raise ValueError("local upload exceeds per-file limit")
        helper_socket = os.environ.get("ELESIM_HOST_HELPER_SOCKET", "").strip()
        if helper_socket:
            _upload_through_host_helper(
                path, content, mode, socket_path=helper_socket, timeout_s=self._timeout_s
            )
            return
        destination = Path(str(path))
        destination.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
        descriptor, temporary_name = tempfile.mkstemp(
            dir=destination.parent, prefix=f".{destination.name}."
        )
        temporary = Path(temporary_name)
        try:
            os.fchmod(descriptor, mode)
            with os.fdopen(descriptor, "wb") as handle:
                handle.write(content)
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, destination)
            os.chmod(destination, mode)
        finally:
            if temporary.exists():
                temporary.unlink()


def _upload_through_host_helper(
    path: PurePosixPath,
    content: bytes,
    mode: int,
    *,
    socket_path: str,
    timeout_s: float,
) -> None:
    request = json.dumps(
        {
            "operation": "upload",
            "path": str(path),
            "mode": int(mode),
            "data": base64.b64encode(content).decode("ascii"),
        },
        separators=(",", ":"),
    ).encode("utf-8") + b"\n"
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        connection.settimeout(max(float(timeout_s) + 2.0, 2.0))
        connection.connect(socket_path)
        connection.sendall(request)
        line = connection.makefile("rb").readline(MAX_REMOTE_OUTPUT_BYTES + 1)
    try:
        result = json.loads(line.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise RuntimeError("host-helper upload response is malformed") from exc
    if not isinstance(result, Mapping) or result.get("ok") is not True:
        detail = result.get("error", "request refused") if isinstance(result, Mapping) else "invalid response"
        raise RuntimeError(f"host-helper upload failed: {detail}")


def _run_through_host_helper(
    argv: Sequence[str],
    *,
    socket_path: str,
    timeout_s: float,
    output: CommandOutput | None = None,
) -> RemoteCommandResult:
    path = Path(socket_path)
    if not path.is_absolute() or "\x00" in socket_path:
        raise ValueError("host-helper socket path must be absolute")
    request = json.dumps(
        {
            "operation": "run",
            "argv": [str(value) for value in argv],
            "stream": output is not None,
            "timeout_s": float(timeout_s),
        },
        separators=(",", ":"),
    ).encode("utf-8") + b"\n"
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as connection:
        # The helper enforces the command deadline itself.  Leave a small
        # response grace period for it to terminate the child, drain both
        # pipes, and return a structured timeout error.
        connection.settimeout(
            max(float(timeout_s) + _HOST_HELPER_RESPONSE_GRACE_S, 2.0)
        )
        connection.connect(socket_path)
        connection.sendall(request)
        with connection.makefile("rb") as response:
            while True:
                line = response.readline(256 * 1024 + 1)
                if not line or len(line) > 256 * 1024:
                    raise RuntimeError("host-helper returned an invalid response")
                payload = json.loads(line.decode("utf-8"))
                if not isinstance(payload, Mapping):
                    raise RuntimeError("host-helper response is malformed")
                if payload.get("type") != "output":
                    break
                if output is None or payload.get("stream") not in {
                    "stdout",
                    "stderr",
                }:
                    raise RuntimeError("host-helper output frame is malformed")
                try:
                    chunk = base64.b64decode(str(payload["data"]), validate=True)
                except (KeyError, ValueError) as exc:
                    raise RuntimeError("host-helper output frame is malformed") from exc
                output(
                    str(payload["stream"]),
                    chunk.decode("utf-8", errors="replace"),
                )
    if not isinstance(payload, Mapping) or payload.get("ok") is not True:
        detail = (
            payload.get("error", "request refused")
            if isinstance(payload, Mapping)
            else "invalid response"
        )
        raise RuntimeError(f"host-helper failed: {detail}")
    try:
        stdout = base64.b64decode(str(payload.get("stdout", "")), validate=True)
        stderr = base64.b64decode(str(payload.get("stderr", "")), validate=True)
        exit_status = int(payload["returncode"])
    except (KeyError, TypeError, ValueError) as exc:
        raise RuntimeError("host-helper response is malformed") from exc
    return RemoteCommandResult(
        exit_status,
        _decode_remote_output(stdout, bool(payload.get("stdout_truncated"))),
        _decode_remote_output(stderr, bool(payload.get("stderr_truncated"))),
    )


def _run_local_streaming(
    argv: Sequence[str], *, output: CommandOutput, timeout_s: float
) -> RemoteCommandResult:
    process = subprocess.Popen(tuple(argv), stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    streams: dict[str, tuple[bytes, bool]] = {}
    failures: list[Exception] = []

    def drain(name: str, stream: object) -> None:
        retained = bytearray()
        truncated = False
        try:
            while True:
                chunk = os.read(stream.fileno(), 32 * 1024)  # type: ignore[attr-defined]
                if not chunk:
                    break
                output(name, chunk.decode("utf-8", errors="replace"))
                retained.extend(chunk)
                if len(retained) > MAX_REMOTE_OUTPUT_BYTES:
                    del retained[: len(retained) - MAX_REMOTE_OUTPUT_BYTES]
                    truncated = True
        except Exception as exc:
            failures.append(exc)
            process.terminate()
        finally:
            streams[name] = bytes(retained), truncated

    workers = (
        threading.Thread(target=drain, args=("stdout", process.stdout), daemon=True),
        threading.Thread(target=drain, args=("stderr", process.stderr), daemon=True),
    )
    for worker in workers:
        worker.start()
    try:
        returncode = process.wait(timeout=timeout_s)
    except subprocess.TimeoutExpired:
        process.terminate()
        try:
            process.wait(timeout=2)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
        raise
    finally:
        for worker in workers:
            worker.join()
    if failures:
        raise failures[0]
    stdout, out_cut = streams["stdout"]
    stderr, err_cut = streams["stderr"]
    return RemoteCommandResult(
        returncode,
        _decode_remote_output(stdout, out_cut),
        _decode_remote_output(stderr, err_cut),
    )


class InstalledElesimLifecycle:
    """Concrete lifecycle for independently installed units on one host."""

    def __init__(
        self,
        topology: ConnectionTopology,
        *,
        scoped: bool | None = None,
    ) -> None:
        self._topology = topology.validate()
        # ``None`` is the compatibility mode used by direct callers that do
        # not have an install manifest.  The connection manager supplies an
        # explicit value after validating its local ownership manifest; a
        # remote target is then required to expose the exact instance paths.
        self._scoped = scoped

    @property
    def scoped(self) -> bool | None:
        return self._scoped

    def _validate_scoped_target(
        self,
        session: SshSession,
        unit: DeploymentUnit,
        *,
        local: bool = False,
    ) -> None:
        """Establish the per-system lifecycle boundary before any mutation.

        A remote scoped target must be explicitly bound in the topology to the
        install UUID/project returned by its authenticated, host-fingerprint
        pinned ``elesim-net identity`` command.  Legacy local direct callers
        may omit the binding because their prefix is already selected by the
        local ownership boundary; remote targets never receive that fallback.
        """

        # A host may carry the native Robot unit beside a scoped Compose
        # unit.  Robot remains an install-wide/systemd boundary; it has no
        # scoped instance dispatcher or ``elesim-net identity`` command.
        if not self._scoped or unit.install_mode != "container":
            return
        expected_uuid = str(unit.install_uuid).strip()
        expected_project = str(unit.project).strip()
        if not expected_uuid:
            if not local:
                raise RuntimeError(
                    f"scoped deployment unit {unit.unit_id!r} has no enrolled install UUID"
                )
        else:
            expected_project = expected_project or project_name(expected_uuid)
            command = (str(_net_command(unit)), "identity")
            result = session.run(command, check=False)
            if result.exit_status != 0:
                raise RemoteCommandError(command, result)
            try:
                identity = json.loads(result.stdout)
            except (TypeError, json.JSONDecodeError) as exc:
                raise RuntimeError(
                    f"scoped install identity is not valid JSON on {unit.unit_id!r}"
                ) from exc
            if not isinstance(identity, Mapping):
                raise RuntimeError(f"scoped install identity is not an object on {unit.unit_id!r}")
            if set(identity) != {"schema_version", "install_uuid", "project"}:
                raise RuntimeError(f"scoped install identity fields are invalid on {unit.unit_id!r}")
            if identity.get("schema_version") != 1:
                raise RuntimeError(f"unsupported scoped install identity on {unit.unit_id!r}")
            if (
                identity.get("install_uuid") != expected_uuid
                or identity.get("project") != expected_project
            ):
                raise RuntimeError(
                    f"scoped install identity mismatch on {unit.unit_id!r}: "
                    f"expected UUID/project {expected_uuid}/{expected_project}"
                )
        system_id = self._topology.system_id
        _safe_identifier(system_id, name="system_id")
        prefix = PurePosixPath(unit.install_root)
        bin_dir = PurePosixPath(unit.bin_dir)
        instance_root = prefix / "instances" / system_id
        paths = (
            prefix / "instances",
            bin_dir / "elesim-instance",
            instance_root,
            instance_root / "state.json",
            instance_root / "bin",
            *(instance_root / "bin" / action for action in ("up", "down", "status", "logs")),
            prefix / "containers" / "compose.instances.yaml",
        )
        for path in paths:
            linked = session.run(("test", "-L", str(path)), check=False)
            if linked.exit_status == 0:
                raise RuntimeError(
                    f"scoped lifecycle path is a symlink on {unit.unit_id}: {path}"
                )
            exists = session.run(("test", "-e", str(path)), check=False)
            if exists.exit_status != 0:
                raise RuntimeError(
                    f"scoped lifecycle scope is not established on "
                    f"{unit.unit_id}: missing {path}"
                )
        for path in (
            bin_dir / "elesim-instance",
            *(instance_root / "bin" / action for action in ("up", "down", "status", "logs")),
        ):
            executable = session.run(("test", "-x", str(path)), check=False)
            if executable.exit_status != 0:
                raise RuntimeError(
                    f"scoped lifecycle wrapper is not executable on "
                    f"{unit.unit_id}: {path}"
                )

    def _scoped_instance_state(
        self, session: SshSession, unit: DeploymentUnit
    ) -> Mapping[str, Any]:
        """Read the exact registered instance state for a scoped unit."""

        if unit.install_mode != "container":
            raise RuntimeError("native Robot units do not have scoped instance state")

        state_path = (
            PurePosixPath(unit.install_root)
            / "instances"
            / self._topology.system_id
            / "state.json"
        )
        result = session.run(("cat", str(state_path)))
        try:
            raw = json.loads(result.stdout)
        except (TypeError, json.JSONDecodeError) as exc:
            raise RuntimeError(
                f"scoped instance state is not valid JSON on {unit.unit_id!r}"
            ) from exc
        if not isinstance(raw, Mapping):
            raise RuntimeError(f"scoped instance state is not an object on {unit.unit_id!r}")
        if str(raw.get("system_id", "")) != self._topology.system_id:
            raise RuntimeError(f"scoped instance state system ID mismatch on {unit.unit_id!r}")
        return raw

    def _validate_scoped_security(
        self,
        session: SshSession,
        unit: DeploymentUnit,
        instance_state: Mapping[str, Any],
    ) -> None:
        """Validate the registered instance generation, not install security."""

        if self._topology.security_profile != "sros2":
            return
        generation = instance_state.get("security_generation")
        if not isinstance(generation, str) or not re.fullmatch(
            r"[a-z0-9][a-z0-9_.-]{0,95}", generation
        ):
            raise RuntimeError("scoped SROS2 instance has no valid security generation")
        security_root = (
            PurePosixPath(unit.install_root)
            / "instances"
            / self._topology.system_id
            / "security"
        )
        marker = security_root / "provisioning-required"
        marker_link = session.run(("test", "-L", str(marker)), check=False)
        if marker_link.exit_status == 0:
            raise RuntimeError("scoped SROS2 provisioning marker is a symlink")
        if session.run(("test", "-e", str(marker)), check=False).exit_status == 0:
            raise RuntimeError("scoped SROS2 instance still has a provisioning marker")
        generations = security_root / "generations"
        if session.run(("test", "-L", str(generations)), check=False).exit_status == 0:
            raise RuntimeError("scoped SROS2 generations path is a symlink")
        current_path = security_root / "current"
        if session.run(("test", "-L", str(current_path)), check=False).exit_status != 0:
            raise RuntimeError("scoped SROS2 current path is not a symlink")
        current = session.run(("readlink", str(current_path)))
        if PurePosixPath(current.stdout.strip()) != PurePosixPath("generations") / generation:
            raise RuntimeError(
                f"scoped SROS2 current generation mismatch on {unit.unit_id!r}"
            )
        manifest_path = security_root / "current/manifest.json"
        if session.run(("test", "-L", str(manifest_path)), check=False).exit_status == 0:
            raise RuntimeError("scoped SROS2 manifest is a symlink")
        session.run(("test", "-f", str(manifest_path)))

    def scoped_identity(
        self, session: SshSession, unit: DeploymentUnit
    ) -> Mapping[str, str]:
        """Read one container install's immutable enrollment identity."""

        if unit.install_mode != "container":
            raise ValueError("only container units expose scoped identity")
        result = session.run((str(_net_command(unit)), "identity"))
        try:
            value = json.loads(result.stdout)
        except (TypeError, json.JSONDecodeError) as exc:
            raise RuntimeError(
                f"scoped install identity is invalid on {unit.unit_id!r}"
            ) from exc
        if not isinstance(value, Mapping):
            raise RuntimeError(f"scoped install identity is not an object on {unit.unit_id!r}")
        if set(value) != {"schema_version", "install_uuid", "project"}:
            raise RuntimeError(f"scoped install identity fields are invalid on {unit.unit_id!r}")
        if type(value.get("schema_version")) is not int or value["schema_version"] != 1:
            raise RuntimeError(
                f"scoped install identity schema is unsupported on {unit.unit_id!r}"
            )
        install_uuid = value.get("install_uuid")
        project = value.get("project")
        if not isinstance(install_uuid, str) or not isinstance(project, str):
            raise RuntimeError(f"scoped install identity values are invalid on {unit.unit_id!r}")
        if project != project_name(install_uuid):
            raise RuntimeError(
                f"scoped install identity project is not derived from UUID on {unit.unit_id!r}"
            )
        return {"install_uuid": install_uuid, "project": project}

    def scoped_releases(
        self, session: SshSession, unit: DeploymentUnit
    ) -> tuple[Mapping[str, Any], ...]:
        """Read the immutable release registry from one enrolled unit."""

        if unit.install_mode != "container":
            raise ValueError("only container units expose scoped releases")
        result = session.run((str(_net_command(unit)), "releases"))
        try:
            value = json.loads(result.stdout)
        except (TypeError, json.JSONDecodeError) as exc:
            raise RuntimeError(
                f"scoped release registry is invalid on {unit.unit_id!r}"
            ) from exc
        if not isinstance(value, list) or not all(isinstance(item, Mapping) for item in value):
            raise RuntimeError(f"scoped release registry is not a list on {unit.unit_id!r}")
        return tuple(dict(item) for item in value)

    def scoped_instance_state(
        self, session: SshSession, unit: DeploymentUnit, system_id: str
    ) -> Mapping[str, Any] | None:
        """Read one target system state without changing the registry."""

        if unit.install_mode != "container":
            raise ValueError("only container units expose scoped instance state")
        state_path = PurePosixPath(unit.install_root) / "instances" / system_id / "state.json"
        result = session.run(("cat", str(state_path)), check=False)
        if result.exit_status != 0:
            return None
        try:
            value = json.loads(result.stdout)
        except (TypeError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"scoped instance state is invalid on {unit.unit_id!r}") from exc
        if not isinstance(value, Mapping) or value.get("system_id") != system_id:
            raise RuntimeError(f"scoped instance state identity mismatch on {unit.unit_id!r}")
        return dict(value)

    def scoped_install_state(
        self, session: SshSession, unit: DeploymentUnit
    ) -> Mapping[str, Any]:
        """Read the install's non-secret state for per-unit compute policy."""

        if unit.install_mode != "container":
            raise ValueError("only container units expose scoped install state")
        result = session.run((str(_net_command(unit)), "show"))
        try:
            value = json.loads(result.stdout)
        except (TypeError, json.JSONDecodeError) as exc:
            raise RuntimeError(f"scoped install state is invalid on {unit.unit_id!r}") from exc
        if not isinstance(value, Mapping):
            raise RuntimeError(f"scoped install state is not an object on {unit.unit_id!r}")
        return dict(value)

    def register_scoped_instance(
        self,
        session: SshSession,
        host: ManagedHost,
        unit: DeploymentUnit,
        instance: Any,
        release: Any,
        bundle: SecurityBundle | None = None,
        *,
        replace_existing: bool = False,
    ) -> None:
        """Register one unit through its installed host-owned wrapper.

        Security bytes are staged below ``maintenance`` (never under the
        final ``instances/<system>`` tree) and the installed wrapper performs
        the atomic InstanceRuntime transaction.  The wrapper is the only
        mutating command exposed to the manager's local host helper.
        """

        if unit.install_mode != "container":
            raise ValueError("scoped registration is available only for container units")
        identity = self.scoped_identity(session, unit)
        if identity.get("install_uuid") != unit.install_uuid:
            raise RuntimeError(f"scoped install UUID changed on {unit.unit_id!r}")
        system_id = str(instance.system_id)
        generation = str(getattr(instance, "security_generation", ""))
        stage_root: PurePosixPath | None = None
        if bundle is not None:
            if not generation:
                raise ValueError("a security bundle requires an instance generation")
            _safe_generation(generation)
            stage_root = (
                PurePosixPath(unit.install_root)
                / "maintenance"
                / ".connection-scoped"
                / system_id
                / generation
            )
            try:
                for file in bundle.for_roles(unit.roles).files:
                    destination = stage_root / file.relative_path
                    session.upload_bytes(destination, file.content, file.mode)
            except BaseException as upload_error:
                # Uploads are intentionally no-replace, so a broken transfer
                # leaves a partial manager staging tree behind.  Remove only
                # this exact system/generation through the same host-owned
                # wrapper before allowing the caller to retry.  Never issue a
                # generic recursive delete from the manager boundary.
                cleanup_command = (
                    str(PurePosixPath(unit.bin_dir) / "elesim-instance-register"),
                    "cleanup-staging",
                    "--system",
                    system_id,
                    "--security-generation",
                    generation,
                )
                try:
                    session.run(cleanup_command)
                except BaseException as cleanup_error:
                    raise RuntimeError(
                        f"scoped security upload failed on {host.host_id}/{unit.unit_id}; "
                        f"exact staging cleanup also failed: {cleanup_error}"
                    ) from upload_error
                raise
        command = [
            str(PurePosixPath(unit.bin_dir) / "elesim-instance-register"),
            "replace" if replace_existing else "register",
            "--system", system_id,
            "--release", str(instance.release_key),
            "--domain-id", str(instance.domain_id),
            "--rmw", str(instance.rmw_implementation),
            "--discovery-mode", str(instance.discovery_mode),
            "--interface", str(instance.interface),
            "--security-profile", str(instance.security_profile),
        ]
        command.extend(value for peer in instance.static_peers for value in ("--static-peer", str(peer)))
        command.extend(
            value for endpoint in instance.endpoints
            for value in ("--endpoint", f"{endpoint.role}:{endpoint.endpoint_id}")
        )
        command.extend(
            value
            for role, endpoint_id in (
                ("pilot", instance.pilot_id),
                ("sim", instance.sim_id),
                ("ui", instance.ui_id),
            )
            for value in ("--graph-endpoint", f"{role}:{endpoint_id}")
        )
        compute = getattr(instance, "compute", None)
        if compute is not None:
            command.extend(("--gpu-mode", str(compute.gpu_mode)))
            if str(compute.gpu_device):
                command.extend(("--gpu-device", str(compute.gpu_device)))
        turn = getattr(instance, "turn", None)
        turn_mode = str(getattr(turn, "mode", "none"))
        command.extend(("--turn-mode", turn_mode))
        command.extend(
            value
            for url in getattr(instance, "turn_urls", ())
            for value in ("--turn-url", str(url))
        )
        if turn_mode == "managed":
            command.extend(("--turn-realm", str(turn.realm)))
            command.extend(("--turn-public-host", str(turn.public_host)))
            for option, value in (
                ("--turn-listen-port", turn.listen_port),
                ("--turn-relay-min-port", turn.relay_min_port),
                ("--turn-relay-max-port", turn.relay_max_port),
            ):
                if value is not None:
                    command.extend((option, str(value)))
        elif turn_mode == "external":
            command.extend(
                ("--turn-credential-file", str(turn.credential_file))
            )
        if generation:
            command.extend(("--security-generation", str(stage_root or generation)))
        if stage_root is not None:
            command.extend(("--security-bundle-root", str(stage_root)))
        # The installed wrapper consumes and removes the exact validated
        # maintenance staging tree.  Do not issue a generic recursive-delete
        # command from the manager container/SSH client: on the local helper
        # path it cannot address the host mount reliably, and cleanup belongs
        # to the same host-owned validation boundary as registration.
        session.run(tuple(command))

    def remove_scoped_instance(
        self, session: SshSession, host: ManagedHost, unit: DeploymentUnit, system_id: str
    ) -> None:
        if unit.install_mode != "container":
            raise ValueError("native Robot units cannot remove scoped instances")
        self._validate_scoped_target(session, unit, local=host.local)
        session.run(
            (
                str(PurePosixPath(unit.bin_dir) / "elesim-instance-register"),
                "remove",
                "--system",
                system_id,
            )
        )

    def preflight(
        self, session: SshSession, host: ManagedHost, security_root: PurePosixPath
    ) -> RemoteCapabilities:
        state = self.snapshot(session, host)
        unit_states = state.get("units")
        if isinstance(unit_states, Mapping):
            states = {str(key): value for key, value in unit_states.items()}
        else:
            states = {host.primary_unit.unit_id: state}
        for unit in host.units:
            scoped_unit = self._scoped and unit.install_mode == "container"
            self._validate_scoped_target(session, unit, local=host.local)
            scoped_state = self._scoped_instance_state(session, unit) if scoped_unit else None
            unit_state = states.get(unit.unit_id)
            if not isinstance(unit_state, Mapping):
                raise RuntimeError(
                    f"installed state for unit {unit.unit_id!r} is missing on {host.host_id!r}"
                )
            configured_roles = tuple(str(value) for value in unit_state.get("roles", ()))
            if scoped_unit:
                raw_endpoints = scoped_state.get("endpoints", []) if scoped_state else []
                configured_roles = tuple(
                    str(row.get("role"))
                    for row in raw_endpoints
                    if isinstance(row, Mapping) and isinstance(row.get("role"), str)
                )
                if set(configured_roles) != set(unit.roles):
                    raise RuntimeError(
                        f"scoped instance roles do not match {host.host_id}/{unit.unit_id!r}"
                    )
            if not set(unit.roles).issubset(configured_roles):
                raise RuntimeError(
                    f"assigned roles are not installed on {host.host_id}/{unit.unit_id!r}: "
                    f"installed={configured_roles!r}, assigned={unit.roles!r}"
                )
            if (
                not self._scoped
                and unit.install_mode == "container"
                and set(configured_roles) - set(unit.roles)
            ):
                running = session.run(
                    (*_compose_command(unit), "ps", "--status", "running", "--services")
                )
                unassigned = (set(running.stdout.split()) & set(configured_roles)) - set(unit.roles)
                if unassigned:
                    raise RuntimeError(
                        f"stop unassigned roles before changing topology on {host.host_id}: "
                        + ", ".join(sorted(unassigned))
                    )
            for key, expected in {
                "prefix": unit.install_root,
                "bin_dir": unit.bin_dir,
                "install_mode": unit.install_mode,
            }.items():
                if str(unit_state.get(key, "")) != expected:
                    raise RuntimeError(
                        f"{key} mismatch on {host.host_id}/{unit.unit_id}"
                    )
            if scoped_unit:
                self._validate_scoped_security(session, unit, scoped_state or {})
            else:
                self._validate_managed_security_state(
                    session,
                    host,
                    self._unit_security_root(host, unit, security_root),
                    unit_state,
                )
            if "sim" in unit.roles:
                turn_state = (
                        _scoped_turn_from_instance_state(
                            scoped_state or {}, host, system_id=self._topology.system_id
                        )
                    if scoped_unit
                    else (_managed_turn_from_state(unit_state, host)
                          if self._topology.security_profile == "sros2" else None)
                )
                if turn_state is None:
                    continue
                if turn_state.get("turn_mode", "managed") == "external":
                    credential_file = turn_state["turn_credential_file"]
                    if _remote_path_contains_symlink(session, credential_file):
                        raise RuntimeError(
                            f"external TURN credential path is a symlink or has a symlink ancestor on "
                            f"{host.host_id}/{unit.unit_id}: {credential_file}"
                        )
                    if session.run(("test", "-f", credential_file), check=False).exit_status != 0:
                        raise RuntimeError(f"external TURN credential file is missing on {host.host_id}/{unit.unit_id}: {credential_file}")
                    continue
                managed_turn = turn_state
                if _remote_path_contains_symlink(
                    session, managed_turn["turn_secret_file"]
                ):
                    raise RuntimeError(
                        f"managed Coturn secret path is a symlink or has a symlink ancestor on "
                        f"{host.host_id}/{unit.unit_id}: "
                        f"{managed_turn['turn_secret_file']}"
                    )
                if session.run(
                    ("test", "-f", managed_turn["turn_secret_file"]),
                    check=False,
                ).exit_status != 0:
                    raise RuntimeError(
                        f"managed Coturn secret file is missing on {host.host_id}/{unit.unit_id}: "
                        f"{managed_turn['turn_secret_file']}"
                    )
                if not self._scoped and unit.install_mode == "container":
                    services = session.run(
                        (*_compose_command(unit), "config", "--services"),
                        check=False,
                    )
                    if services.exit_status != 0 or "coturn" not in services.stdout.split():
                        raise RuntimeError(
                            f"managed Coturn service is missing on {host.host_id}/{unit.unit_id}; "
                            "run elesim-update after installing Sim with SROS2"
                        )

        # The connection-manager container mounts the operator home and the
        # installation prefix read-only, while exposing only each unit's
        # security directory as a writable bind.  Checking install_root here
        # therefore reports a false negative for a local host even though the
        # exact directory used by staging is writable.  Probe the paths that
        # the transactional rollout actually creates instead.
        security_paths = tuple(
            self._unit_security_root(host, unit, security_root)
            for unit in host.units
        )
        writable = all(
            session.run(("test", "-w", str(path)), check=False).exit_status == 0
            for path in security_paths
        )
        # The generated wrapper pins the installation's Docker context and
        # Engine ID.  Never fall back to whichever global context happens to
        # be selected in the operator's current shell.
        docker = True
        systemd = session.run(
            ("test", "-x", "/usr/bin/systemctl"), check=False
        ).exit_status == 0
        jetson = session.run(
            ("test", "-f", "/etc/nv_tegra_release"), check=False
        ).exit_status == 0
        architecture_result = session.run(("uname", "-m"), check=False)

        for unit in host.runtime_units:
            compose = PurePosixPath(unit.install_root) / "containers/compose.yaml"
            if self._scoped:
                # The installation aggregate is not the runtime graph for a
                # scoped target; its immutable aggregate lives in the exact
                # per-system compose context checked above.
                continue
            if session.run(("test", "-f", str(compose)), check=False).exit_status != 0:
                raise RuntimeError(
                    f"Compose manifest is missing on {host.host_id}/{unit.unit_id}"
                )
            docker = docker and session.run(
                (*_compose_command(unit), "config", "--quiet"), check=False
            ).exit_status == 0
        for unit in host.robot_units:
            service = _robot_service(unit)
            sudo_probe = session.run(
                (
                    "sudo",
                    "-n",
                    "systemctl",
                    "show",
                    service,
                    "--property=LoadState",
                    "--value",
                ),
                check=False,
            )
            systemd = systemd and sudo_probe.exit_status == 0 and (
                sudo_probe.stdout.strip() == "loaded"
            )

        return RemoteCapabilities(
            docker=docker,
            systemd=systemd,
            jetson=jetson,
            security_root_writable=writable,
            architecture=architecture_result.stdout.strip(),
        )

    def runtime_network_check(
        self, session: SshSession, host: ManagedHost
    ) -> None:
        """Validate the pending DDS interface immediately before runtime use.

        This deliberately does not belong to security preflight: SROS2
        authority generation is independent of whether a host's current
        container backend exposes a direct interface.  The check itself is
        still required before a lifecycle start, and receives the topology's
        interface explicitly so an older installed state cannot be mistaken
        for the pending connection-manager configuration.
        """

        peer_args = tuple(
            value
            for peer in self._topology.discovery_peers(host.host_id)
            for value in ("--dds-peer", peer)
        )
        for unit in host.units:
            session.run(
                (
                    str(_net_command(unit)),
                    "namespace-check",
                    "--dds-interface",
                    host.dds.interface,
                    "--dds-address",
                    host.dds.address,
                    *peer_args,
                )
            )

    def runtime_launch_preflight(
        self, session: SshSession, host: ManagedHost
    ) -> None:
        """Run each unit's no-override launch guard before any mutation."""

        state = self.snapshot(session, host)
        unit_states = state.get("units")
        for unit in host.units:
            scoped_unit = self._scoped and unit.install_mode == "container"
            self._validate_scoped_target(session, unit, local=host.local)
            installed = unit_states[unit.unit_id] if isinstance(unit_states, Mapping) else state
            if scoped_unit:
                scoped_state = self._scoped_instance_state(session, unit)
                assigned_roles = tuple(
                    str(row.get("role"))
                    for row in scoped_state.get("endpoints", ())
                    if isinstance(row, Mapping) and isinstance(row.get("role"), str)
                )
            else:
                assigned = installed.get("assigned_roles")
                assigned_roles = installed.get("roles", ()) if assigned is None else assigned
            if set(assigned_roles) != set(unit.roles):
                raise RuntimeError(
                    f"topology assignment is not applied on {host.host_id}/{unit.unit_id}; "
                    "run connection-manager preparation before start"
                )
            if not self._scoped:
                session.run((str(_net_command(unit)), "configuration-check"))

    def prepare_runtime_network(
        self,
        session: SshSession,
        host: ManagedHost,
        output: CommandOutput,
    ) -> str | None:
        """Check a Docker-Desktop sidecar before namespace preflight.

        Native installs are a no-op.  Sidecar image pull and browser/device
        enrollment are explicit installation commands; the connection manager
        only verifies the resulting status document and discovers its runtime
        DDS address.  Keeping enrollment outside this job prevents a hidden
        browser wait when the GUI has no login step.
        """

        installed = self.snapshot(session, host)
        raw_units = installed.get("units")
        unit_states = (
            raw_units
            if isinstance(raw_units, Mapping)
            else {host.primary_unit.unit_id: installed}
        )
        discovered: set[str] = set()
        for unit in host.runtime_units:
            raw_state = unit_states.get(unit.unit_id)
            if not isinstance(raw_state, Mapping):
                raise RuntimeError(
                    f"installed state for {host.host_id}/{unit.unit_id} is missing"
                )
            settings = raw_state.get("container_network", {})
            mode = (
                str(settings.get("mode", "direct-host"))
                if isinstance(settings, Mapping)
                else "direct-host"
            )
            if mode == "direct-host":
                continue
            if mode != "tailscale-sidecar":
                raise RuntimeError(
                    f"unsupported container network mode on "
                    f"{host.host_id}/{unit.unit_id}: {mode!r}"
                )

            status_command = (
                str(_tailscale_command(unit)),
                "status",
                "--json",
            )
            result = session.run(status_command, check=False)
            try:
                backend, ipv4 = _parse_tailscale_status(result.stdout)
            except RuntimeError as exc:
                if result.exit_status == 0:
                    raise RuntimeError(
                        f"EleSim Tailscale sidecar status is invalid on "
                        f"{host.host_id}/{unit.unit_id}"
                    ) from exc
                backend, ipv4 = "", ""
            if (
                result.exit_status != 0
                or backend.casefold() != "running"
                or not ipv4
            ):
                raise RuntimeError(
                    f"EleSim Tailscale sidecar is not ready on "
                    f"{host.host_id}/{unit.unit_id}. Run "
                    f"elesim-tailscale login on that host, then retry "
                    f"elesim-connections (backend={backend or 'unknown'})"
                )
            discovered.add(ipv4)
        if len(discovered) > 1:
            raise RuntimeError(
                f"runtime units on {host.host_id} reported different Tailscale "
                f"addresses: {sorted(discovered)!r}"
            )
        return next(iter(discovered), None)

    @staticmethod
    def _unit_security_root(
        host: ManagedHost,
        unit: DeploymentUnit,
        host_security_root: PurePosixPath,
    ) -> PurePosixPath:
        if len(host.units) == 1:
            return host_security_root
        return PurePosixPath(unit.install_root) / "security"

    def _validate_managed_security_state(
        self,
        session: SshSession,
        host: ManagedHost,
        security_root: PurePosixPath,
        state: Mapping[str, Any],
    ) -> None:
        dds = state.get("dds")
        if not isinstance(dds, Mapping):
            raise RuntimeError(f"DDS state is missing on {host.host_id!r}")
        if (
            str(dds.get("security_profile", "")) != "sros2"
            or str(dds.get("security_provisioning", "")) != "managed"
        ):
            return
        values = tuple(
            str(dds.get(name, "")).strip()
            for name in (
                "security_generation",
                "security_bundle",
                "keystore",
                "enclave",
            )
        )
        marker = security_root / "provisioning-required"
        marker_exists = session.run(("test", "-f", str(marker)), check=False).exit_status == 0
        current = session.run(("readlink", str(security_root / "current")), check=False)
        current_generation = (
            PurePosixPath(current.stdout.strip()).name
            if current.exit_status == 0 and current.stdout.strip()
            else ""
        )
        if not any(values):
            if not marker_exists or current_generation:
                raise RuntimeError(
                    f"managed SROS2 pending state is inconsistent on {host.host_id!r}; "
                    "run connection-manager recovery"
                )
            return
        if not all(values):
            raise RuntimeError(
                f"managed SROS2 fields are partially populated on {host.host_id!r}; "
                "run connection-manager recovery"
            )
        generation = values[0]
        if marker_exists or current_generation != generation:
            raise RuntimeError(
                f"managed SROS2 generation/current marker mismatch on {host.host_id!r}; "
                "run connection-manager recovery"
            )
        session.run(("test", "-f", str(security_root / "current/manifest.json")))

    def snapshot(
        self, session: SshSession, host: ManagedHost
    ) -> Mapping[str, Any]:
        snapshots: dict[str, Mapping[str, Any]] = {}
        for unit in host.units:
            result = session.run((str(_net_command(unit)), "show"))
            try:
                raw = json.loads(result.stdout)
            except json.JSONDecodeError as exc:
                raise RuntimeError(
                    f"elesim-net show returned invalid JSON on "
                    f"{host.host_id}/{unit.unit_id!r}"
                ) from exc
            if not isinstance(raw, Mapping):
                raise RuntimeError(
                    f"installed state is not an object on {host.host_id}/{unit.unit_id!r}"
                )
            snapshots[unit.unit_id] = dict(raw)
        if len(snapshots) == 1:
            return dict(next(iter(snapshots.values())))
        primary = snapshots[host.primary_unit.unit_id]
        result = dict(primary)
        result["units"] = snapshots
        result["roles"] = list(host.roles)
        return result

    def configure(
        self,
        session: SshSession,
        host: ManagedHost,
        generation: str | None,
        security_root: PurePosixPath,
    ) -> None:
        if self._scoped:
            raise RuntimeError(
                "scoped instances do not expose install-wide configuration; "
                "register or replace the exact elesim instance instead"
            )
        installed = self.snapshot(session, host)
        installed_units = installed.get("units")
        installed_states = (
            installed_units
            if isinstance(installed_units, Mapping)
            else {host.primary_unit.unit_id: installed}
        )
        endpoints = {
            assignment.role: assignment.endpoint_id
            for managed_host in self._topology.hosts
            for assignment in managed_host.assignments
        }
        for unit in host.units:
            self._validate_scoped_target(session, unit, local=host.local)
            unit_root = self._unit_security_root(host, unit, security_root)
            values: dict[str, Any] = {
                "system_id": self._topology.system_id,
                "domain_id": self._topology.dds_graph.domain_id,
                "rmw_implementation": self._topology.dds_graph.rmw_implementation,
                "discovery_mode": self._topology.dds_graph.discovery_mode,
                "static_peers": self._topology.discovery_peers(host.host_id),
                "interface": host.dds.interface,
                "security_profile": self._topology.security_profile,
            }
            if self._topology.security_profile == "sros2":
                if generation is None:
                    raise ValueError("managed sros2 configuration requires a generation")
                values.update(
                    {
                        "security_provisioning": "managed",
                        "security_generation": generation,
                        "security_bundle": str(unit_root / "current" / "keystore"),
                        "keystore": str(unit_root / "current" / "keystore"),
                        "enclave": f"/elesim/{self._topology.system_id}",
                    }
                )
            else:
                if generation is not None:
                    raise ValueError("trusted-network configuration has no generation")
                values.update(
                    {
                        "security_provisioning": "none",
                        "security_generation": "",
                        "security_bundle": "",
                        "keystore": "",
                        "enclave": "",
                    }
                )
            if "sim" in unit.roles:
                installed_state = (
                    self._scoped_instance_state(session, unit)
                    if self._scoped
                    else installed_states.get(unit.unit_id)
                )
                if not isinstance(installed_state, Mapping):
                    raise RuntimeError(
                        f"Sim runtime state is missing on {host.host_id}/{unit.unit_id}"
                    )
                turn_state = (
                    _scoped_turn_from_instance_state(
                        installed_state, host, system_id=self._topology.system_id
                    )
                    if self._scoped
                    else (_managed_turn_from_state(installed_state, host)
                          if self._topology.security_profile == "sros2" else None)
                )
                if turn_state is not None:
                    values.update(turn_state)
                    if not self._scoped:
                        values["turn_mode"] = "managed"
                else:
                    values["turn_mode"] = "none"
            # Remote role identities are also runtime inputs (UI -> Pilot/Sim,
            # Pilot -> Sim). Installed-but-unassigned local copies do not own them.
            role_ids = endpoints
            session.run(
                _configuration_command(
                    unit,
                    values,
                    sim_id=role_ids.get("sim", ""),
                    pilot_id=role_ids.get("pilot", ""),
                    ui_id=role_ids.get("ui", ""),
                    robot_id=role_ids.get("robot", ""),
                )
            )

    def restore(
        self,
        session: SshSession,
        host: ManagedHost,
        configuration: Mapping[str, Any],
    ) -> None:
        if self._scoped:
            raise RuntimeError(
                "scoped instances do not expose install-wide configuration rollback"
            )
        dds = configuration.get("dds")
        network = configuration.get("network")
        if not isinstance(dds, Mapping) or not isinstance(network, Mapping):
            raise RuntimeError(f"rollback state is malformed for {host.host_id!r}")
        unit_snapshots = configuration.get("units")
        for unit in host.units:
            self._validate_scoped_target(session, unit, local=host.local)
            snapshot = (
                unit_snapshots.get(unit.unit_id, {})
                if isinstance(unit_snapshots, Mapping)
                else configuration
            )
            if not isinstance(snapshot, Mapping):
                raise RuntimeError(f"rollback state for {host.host_id}/{unit.unit_id} is malformed")
            payload = base64.urlsafe_b64encode(
                json.dumps(
                    dict(snapshot),
                    ensure_ascii=False,
                    sort_keys=True,
                    separators=(",", ":"),
                ).encode("utf-8")
            ).decode("ascii")
            session.run(
                (
                    str(_net_command(unit)),
                    "restore-snapshot",
                    "--payload",
                    payload,
                )
            )

    def stop(
        self, session: SshSession, host: ManagedHost, roles: Sequence[str]
    ) -> None:
        selected = set(_selected_roles(host, roles))
        units = sorted(host.units, key=lambda unit: ("robot" not in unit.roles, unit.unit_id))
        for unit in units:
            self._validate_scoped_target(session, unit, local=host.local)
            unit_roles = tuple(role for role in unit.roles if role in selected)
            if unit_roles:
                if self._scoped and unit.install_mode == "container":
                    session.run(
                        _lifecycle_command(
                            unit,
                            action="stop",
                            roles=unit_roles,
                            system_id=self._topology.system_id,
                            scoped=True,
                        )
                    )
                    continue
                include_coturn = (
                    "sim" in unit_roles
                    and unit.install_mode == "container"
                    and "coturn" in self._compose_services(session, unit)
                )
                session.run(
                    _lifecycle_command(
                        unit,
                        action="stop",
                        roles=unit_roles,
                        include_coturn=include_coturn,
                    )
                )

    def cleanup_viewer(
        self, session: SshSession, host: ManagedHost, roles: Sequence[str]
    ) -> None:
        selected = set(_selected_roles(host, roles))
        if "sim" not in selected:
            return
        if self._scoped:
            # Instance wrappers own their exact service set and do not expose
            # the legacy X11 cleanup channel.
            return
        for unit in host.units:
            if (
                "sim" in unit.roles
                and unit.install_mode == "container"
                and unit.lifecycle == "compose"
            ):
                session.run(
                    (str(PurePosixPath(unit.bin_dir) / "elesim-viewer-cleanup"),)
                )

    def start(
        self, session: SshSession, host: ManagedHost, roles: Sequence[str]
    ) -> None:
        selected = set(_selected_roles(host, roles))
        units = sorted(host.units, key=lambda unit: ("robot" in unit.roles, unit.unit_id))
        for unit in units:
            self._validate_scoped_target(session, unit, local=host.local)
            unit_roles = tuple(role for role in unit.roles if role in selected)
            if unit_roles:
                if self._scoped and unit.install_mode == "container":
                    session.run(
                        _lifecycle_command(
                            unit,
                            action="start",
                            roles=unit_roles,
                            system_id=self._topology.system_id,
                            scoped=True,
                        )
                    )
                    continue
                include_coturn = (
                    self._topology.security_profile == "sros2"
                    and "sim" in unit_roles
                    and unit.install_mode == "container"
                    and "coturn" in self._compose_services(session, unit)
                )
                session.run(
                    _lifecycle_command(
                        unit,
                        action="start",
                        roles=unit_roles,
                        include_coturn=include_coturn,
                    )
                )

    def build(
        self, session: SshSession, host: ManagedHost, output: CommandOutput
    ) -> None:
        if self._scoped:
            # Scoped instances run immutable, already-published releases.
            # Building the installation-level aggregate would be both the
            # wrong boundary and a possible cross-system mutation.
            for unit in host.runtime_units:
                self._validate_scoped_target(session, unit, local=host.local)
            return
        for unit in host.runtime_units:
            def unit_output(stream: str, text: str, *, unit_id: str = unit.unit_id) -> None:
                output(stream, f"[{unit_id}] {text}")

            session.run_streaming(
                _lifecycle_command(unit, action="build", roles=unit.roles),
                output=unit_output,
            )

    def launch(
        self,
        session: SshSession,
        host: ManagedHost,
        runtime_options: RuntimeLaunchOptions | None = None,
        viewer_user: str | None = None,
    ) -> None:
        units = sorted(host.units, key=lambda unit: ("robot" in unit.roles, unit.unit_id))
        for unit in units:
            self._validate_scoped_target(session, unit, local=host.local)
            if self._scoped and unit.install_mode == "container":
                if runtime_options is not None:
                    raise ValueError(
                        "scoped instance lifecycle does not accept runtime GPU/viewer overrides; "
                        "configure the instance compute policy at registration"
                    )
                session.run(
                    _lifecycle_command(
                        unit,
                        action="launch",
                        system_id=self._topology.system_id,
                        scoped=True,
                    )
                )
                continue
            # A role-specific manager request must be launched separately when
            # Pilot and Sim share one Compose unit.  ``elesim-up`` accepts one
            # CUDA_VISIBLE_DEVICES value per invocation; combining the roles
            # would silently let whichever role is selected last control both
            # containers.  UI has no GPU selector and therefore gets a plain
            # role-scoped invocation in this path.
            if (
                runtime_options is not None
                and runtime_options.pilot_gpu_inherit is not None
                and unit.install_mode == "container"
                and len(unit.roles) > 1
            ):
                role_groups = tuple((role,) for role in unit.roles)
            else:
                role_groups = (unit.roles,)
            for selected_roles in role_groups:
                include_coturn = (
                    self._topology.security_profile == "sros2"
                    and "sim" in selected_roles
                    and unit.install_mode == "container"
                    and "coturn" in self._compose_services(session, unit)
                )
                unit_runtime_options = runtime_options
                if (
                    runtime_options is not None
                    and runtime_options.viewer
                    and "sim" not in selected_roles
                ):
                    unit_runtime_options = RuntimeLaunchOptions(
                        gpu_inherit=runtime_options.gpu_inherit,
                        gpu_device=runtime_options.gpu_device,
                        viewer=False,
                        pilot_gpu_inherit=runtime_options.pilot_gpu_inherit,
                        pilot_gpu_device=runtime_options.pilot_gpu_device,
                        sim_gpu_inherit=runtime_options.sim_gpu_inherit,
                        sim_gpu_device=runtime_options.sim_gpu_device,
                    )
                session.run(
                    _lifecycle_command(
                        unit,
                        action="launch",
                        roles=selected_roles,
                        include_coturn=include_coturn,
                        runtime_options=unit_runtime_options,
                        viewer_user=viewer_user,
                    )
                )

    @staticmethod
    def _compose_services(
        session: SshSession, unit: DeploymentUnit
    ) -> set[str]:
        command = (*_compose_command(unit), "config", "--services")
        result = session.run(command, check=False)
        if result.exit_status != 0:
            raise RemoteCommandError(command, result)
        return set(result.stdout.split())

    def runtime_doctor(
        self,
        session: SshSession,
        host: ManagedHost,
        expected_peer_ids: Sequence[str],
        timeout_s: float,
    ) -> Mapping[str, Any]:
        if timeout_s <= 0:
            raise ValueError("runtime doctor timeout must be positive")
        payloads: dict[str, Mapping[str, Any]] = {}
        deadline = time.monotonic() + float(timeout_s)
        for unit in host.units:
            self._validate_scoped_target(session, unit, local=host.local)
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                payloads[unit.unit_id] = {
                    "ok": False,
                    "results": [
                        {
                            "name": "DDS peers",
                            "status": "fail",
                            "detail": "shared DDS readiness deadline expired",
                        }
                    ],
                }
                continue
            argv = [
                str(_net_command(unit)),
                "doctor",
                "--timeout",
                f"{remaining:g}",
                "--json",
                "--strict-peers",
                "--readiness-only",
            ]
            for endpoint_id in expected_peer_ids:
                value = str(endpoint_id).strip()
                if value:
                    argv.extend(("--expect-peer", value))
            result = session.run(tuple(argv), check=False)
            try:
                payload = json.loads(result.stdout)
            except json.JSONDecodeError as exc:
                detail = result.stderr.strip() or result.stdout.strip()
                raise RuntimeError(
                    f"elesim-net doctor returned invalid JSON on "
                    f"{host.host_id}/{unit.unit_id!r}"
                    + (f": {detail[:512]}" if detail else "")
                ) from exc
            if not isinstance(payload, Mapping):
                raise RuntimeError(
                    f"elesim-net doctor returned a non-object on "
                    f"{host.host_id}/{unit.unit_id!r}"
                )
            payloads[unit.unit_id] = dict(payload)
        if len(payloads) == 1:
            return dict(next(iter(payloads.values())))
        return {"state": "ready", "units": payloads}

    def _gpu_policies(
        self, session: SshSession, host: ManagedHost
    ) -> tuple[dict[str, dict[str, str]], list[str]]:
        """Read the immutable GPU policy recorded by each runtime unit.

        ``elesim-net show`` is an existing read-only, ownership-checked host
        command.  Keeping the query at this boundary lets the manager inspect
        a remote Sim/Pilot installation without receiving Docker access or
        inferring policy from DDS status.
        """

        policies: dict[str, dict[str, str]] = {}
        errors: list[str] = []
        for unit in host.units:
            roles = tuple(role for role in unit.roles if role in {"pilot", "sim"})
            if not roles:
                continue
            command = (str(_net_command(unit)), "show")
            result = session.run(command, check=False)
            if result.exit_status != 0:
                detail = (result.stderr.strip() or result.stdout.strip())[:256]
                errors.append(
                    f"{unit.unit_id}: elesim-net show failed"
                    + (f": {detail}" if detail else "")
                )
                continue
            try:
                raw = json.loads(result.stdout)
                if not isinstance(raw, Mapping):
                    raise ValueError("state is not an object")
                compute = raw.get("compute")
                if not isinstance(compute, Mapping):
                    raise ValueError("compute policy is missing")
                mode = str(compute.get("gpu_mode", "")).strip()
                if mode not in {"inherit", "specific", "cpu"}:
                    raise ValueError(f"unsupported GPU mode {mode!r}")
                device = _normalize_gpu_selector(
                    compute.get("gpu_device", ""), name="gpu_device"
                )
                if mode == "specific" and not device:
                    raise ValueError("specific GPU policy has no device")
                if mode != "specific":
                    device = ""
            except (TypeError, ValueError, json.JSONDecodeError) as exc:
                errors.append(f"{unit.unit_id}: invalid GPU policy: {exc}")
                continue
            policy = {"mode": mode, "device": device}
            for role in roles:
                policies[role] = dict(policy)
        return policies, errors

    def status(self, session: SshSession, host: ManagedHost) -> Mapping[str, Any]:
        """Return a bounded lifecycle snapshot without changing host state."""
        unit_status: dict[str, Mapping[str, Any]] = {}
        for unit in host.units:
            if unit.install_mode == "container":
                if self._scoped:
                    self._validate_scoped_target(session, unit, local=host.local)
                    command = _lifecycle_command(
                        unit,
                        action="status",
                        system_id=self._topology.system_id,
                        scoped=True,
                    )
                    result = session.run(command, check=False)
                    if result.exit_status != 0:
                        raise RemoteCommandError(command, result)
                    running_services = set(result.stdout.split())
                    expected = set(unit.roles)
                    scoped_turn = None
                    if "sim" in expected:
                        scoped_turn = _scoped_turn_from_instance_state(
                            self._scoped_instance_state(session, unit),
                            host,
                            system_id=self._topology.system_id,
                        )
                    managed_required = (
                        scoped_turn is not None
                        and scoped_turn.get("turn_mode") == "managed"
                    )
                    required = set(expected)
                    if managed_required:
                        # The instance status wrapper emits ``coturn`` for
                        # the exact managed service, so include it in
                        # readiness without consulting install-global TURN.
                        required.add("coturn")
                    running = tuple(sorted(running_services & expected))
                    relay_detail = (
                        "managed Coturn is not running"
                        if managed_required and "coturn" not in running_services
                        else ""
                    )
                    active = required.issubset(running_services)
                    detail = result.stderr.strip()[:512]
                    if relay_detail:
                        detail = f"{detail}; {relay_detail}" if detail else relay_detail
                    # The per-instance status wrapper has already checked the
                    # exact registered service set.  Its output is the
                    # running service list, so a registered but stopped
                    # instance is represented as present/not running here.
                    unit_status[unit.unit_id] = {
                        "state": "running" if active else (
                            "stopped" if not running else "degraded"
                        ),
                        "running_roles": list(running),
                        "containers_present": True,
                        "detail": detail,
                    }
                    continue
                all_command = (*_compose_command(unit), "ps", "--all", "--services")
                all_result = session.run(all_command, check=False)
                if all_result.exit_status != 0:
                    raise RemoteCommandError(all_command, all_result)
                running_command = (
                    *_compose_command(unit),
                    "ps",
                    "--status",
                    "running",
                    "--services",
                )
                result = session.run(running_command, check=False)
                if result.exit_status != 0:
                    raise RemoteCommandError(running_command, result)
                expected = set(unit.roles)
                all_services = set(all_result.stdout.split())
                running_services = set(result.stdout.split())
                running = tuple(sorted(value for value in running_services if value in expected))
                configured_services = self._compose_services(session, unit)
                required_services = set(expected)
                scoped_turn = None
                if "sim" in expected and self._scoped:
                    scoped_turn = _scoped_turn_from_instance_state(
                        self._scoped_instance_state(session, unit), host,
                        system_id=self._topology.system_id,
                    )
                managed_required = (
                    scoped_turn is not None and scoped_turn.get("turn_mode") == "managed"
                    if self._scoped
                    else self._topology.security_profile == "sros2"
                )
                if "sim" in expected and managed_required:
                    if "coturn" in configured_services:
                        required_services.add("coturn")
                if "tailscale" in configured_services:
                    required_services.add("tailscale")
                containers_present = required_services.issubset(all_services)
                sidecar_ok = True
                sidecar_detail = ""
                if "tailscale" in configured_services:
                    sidecar = session.run(
                        (str(_tailscale_command(unit)), "status", "--json"),
                        check=False,
                    )
                    try:
                        backend, ipv4 = _parse_tailscale_status(sidecar.stdout)
                    except RuntimeError as exc:
                        sidecar_ok = False
                        sidecar_detail = str(exc)
                    else:
                        sidecar_ok = (
                            sidecar.exit_status == 0
                            and backend.casefold() == "running"
                            and bool(ipv4)
                        )
                        if not sidecar_ok:
                            sidecar_detail = (
                                "Tailscale sidecar is not ready "
                                f"(backend={backend or 'unknown'})"
                            )
                relay_ok = True
                relay_detail = ""
                if "sim" in expected:
                    if managed_required:
                        relay_ok = "coturn" in running_services
                        if not relay_ok:
                            relay_detail = "managed Coturn is not running"
                    elif "coturn" in running_services:
                        relay_ok = False
                        relay_detail = "Coturn must be stopped for plaintext DDS"
                active = (
                    expected.issubset(running_services)
                    and relay_ok
                    and sidecar_ok
                )
                state = "running" if active else ("stopped" if not running else "degraded")
                detail = result.stderr.strip()[:512]
                for extra in (relay_detail, sidecar_detail):
                    if extra:
                        detail = f"{detail}; {extra}" if detail else extra
                unit_status[unit.unit_id] = {
                    "state": state,
                    "running_roles": list(running),
                    "containers_present": containers_present,
                    "detail": detail,
                }
            else:
                result = session.run(
                    ("sudo", "-n", "systemctl", "is-active", _robot_service(unit)),
                    check=False,
                )
                value = result.stdout.strip()
                state = value if value in {"active", "inactive", "failed", "unknown"} else "unknown"
                unit_status[unit.unit_id] = {
                    "state": "running" if state == "active" else state,
                    "running_roles": list(unit.roles if state == "active" else ()),
                    "containers_present": True,
                    "detail": result.stderr.strip()[:512],
                }
        if len(unit_status) == 1:
            snapshot: dict[str, Any] = dict(next(iter(unit_status.values())))
        else:
            running_roles = [
                role
                for unit in host.units
                for role in unit_status[unit.unit_id].get("running_roles", ())
            ]
            states = {str(value.get("state", "unknown")) for value in unit_status.values()}
            state = (
                "running"
                if states == {"running"}
                else ("stopped" if states <= {"stopped", "inactive"} else "degraded")
            )
            snapshot = {
                "state": state,
                "running_roles": running_roles,
                "containers_present": all(
                    bool(value.get("containers_present")) for value in unit_status.values()
                ),
                "units": unit_status,
            }
        policies, policy_errors = (
            ({}, []) if self._scoped else self._gpu_policies(session, host)
        )
        if policies:
            snapshot["gpu_policy"] = policies
        if policy_errors:
            snapshot["gpu_policy_error"] = "; ".join(policy_errors)[:512]
        devices = self._gpu_devices(session, host)
        if devices:
            snapshot["gpu_devices"] = devices
        return snapshot

    @staticmethod
    def _gpu_devices(
        session: SshSession, host: ManagedHost
    ) -> list[dict[str, str]]:
        """Read host GPU indices/UUIDs through the owned status wrapper.

        This is deliberately best-effort.  Older installations may not yet
        have the ``--gpu-devices`` status probe, and CPU-only hosts naturally
        return no devices.  Neither case makes an otherwise valid lifecycle
        status request fail.
        """

        found: dict[str, dict[str, str]] = {}
        for unit in host.units:
            result = session.run(
                (str(_status_command(unit)), "--gpu-devices"),
                check=False,
            )
            if result.exit_status != 0:
                continue
            for line in result.stdout.splitlines():
                parts = [part.strip() for part in line.split(",", 1)]
                if len(parts) != 2 or not parts[0].isdigit():
                    continue
                index, uuid_value = parts
                if not _GPU_SELECTOR.fullmatch(uuid_value):
                    continue
                found[index] = {"index": index, "uuid": uuid_value}
        return [found[index] for index in sorted(found, key=lambda value: int(value))]

    def verify(
        self,
        session: SshSession,
        host: ManagedHost,
        generation: str | None,
        running_roles: Sequence[str],
    ) -> None:
        selected = _selected_roles(host, running_roles)
        state = self.snapshot(session, host)
        unit_states = state.get("units")
        states = (
            unit_states
            if isinstance(unit_states, Mapping)
            else {host.primary_unit.unit_id: state}
        )
        endpoint_by_role = {
            assignment.role: assignment.endpoint_id
            for managed_host in self._topology.hosts
            for assignment in managed_host.assignments
        }
        expected = {
            "system_id": self._topology.system_id,
            "domain_id": self._topology.dds_graph.domain_id,
            "rmw_implementation": self._topology.dds_graph.rmw_implementation,
            "discovery_mode": self._topology.dds_graph.discovery_mode,
            "static_peers": list(self._topology.discovery_peers(host.host_id)),
            "interface": host.dds.interface,
            "security_profile": self._topology.security_profile,
        }
        for unit in host.units:
            unit_selected = tuple(role for role in unit.roles if role in selected)
            unit_state = states.get(unit.unit_id)
            if not isinstance(unit_state, Mapping):
                raise RuntimeError(f"DDS state is missing on {host.host_id}/{unit.unit_id}")
            if unit_selected and unit.install_mode == "container":
                if self._scoped:
                    self._validate_scoped_target(session, unit, local=host.local)
                    status_command = _lifecycle_command(
                        unit,
                        action="status",
                        system_id=self._topology.system_id,
                        scoped=True,
                    )
                    result = session.run(status_command, check=False)
                    if result.exit_status != 0:
                        raise RemoteCommandError(status_command, result)
                else:
                    result = session.run(
                        (*_compose_command(unit), "ps", "--status", "running", "--services")
                    )
                missing = sorted(set(unit_selected) - set(result.stdout.split()))
                if missing:
                    raise RuntimeError(
                        "roles are not running on "
                        f"{host.host_id}/{unit.unit_id}: {', '.join(missing)}"
                    )
                running_services = set(result.stdout.split())
                if not self._scoped and "tailscale" in self._compose_services(session, unit):
                    sidecar = session.run(
                        (str(_tailscale_command(unit)), "status", "--json")
                    )
                    backend, ipv4 = _parse_tailscale_status(sidecar.stdout)
                    if backend.casefold() != "running" or not ipv4:
                        raise RuntimeError(
                            "Tailscale sidecar is not ready on "
                            f"{host.host_id}/{unit.unit_id}: "
                            f"backend={backend or 'unknown'}"
                        )
                if "sim" in unit_selected:
                    scoped_turn = None
                    if self._scoped:
                        scoped_state = self._scoped_instance_state(session, unit)
                        scoped_turn = _scoped_turn_from_instance_state(
                            scoped_state, host, system_id=self._topology.system_id
                        )
                    managed_required = (
                        scoped_turn is not None and scoped_turn.get("turn_mode") == "managed"
                        if self._scoped
                        else self._topology.security_profile == "sros2"
                    )
                    if managed_required and "coturn" not in running_services:
                        raise RuntimeError(
                            f"managed Coturn is not running on {host.host_id}/{unit.unit_id}"
                        )
                    if (self._scoped and not managed_required and "coturn" in running_services) or (
                        not self._scoped and self._topology.security_profile == "trusted-network" and "coturn" in running_services
                    ):
                        raise RuntimeError(
                            f"Coturn must be stopped for plaintext DDS on "
                            f"{host.host_id}/{unit.unit_id}"
                        )
            elif unit_selected:
                session.run(
                    (
                        "sudo",
                        "-n",
                        "systemctl",
                        "is-active",
                        "--quiet",
                        _robot_service(unit),
                    )
                )
            dds = unit_state.get("dds")
            if not isinstance(dds, Mapping):
                raise RuntimeError(f"DDS state is missing on {host.host_id}/{unit.unit_id}")
            for name, value in expected.items():
                actual = dds.get(name)
                if name == "static_peers":
                    actual = list(actual or ())
                if actual != value:
                    raise RuntimeError(
                        f"DDS {name} did not activate on {host.host_id}/{unit.unit_id}: "
                        f"{actual!r} != {value!r}"
                    )
            if "sim" in unit.roles:
                scoped_runtime_state = (
                    self._scoped_instance_state(session, unit) if self._scoped else unit_state
                )
                # Scoped InstanceState keeps TURN URLs alongside its TURN
                # settings.  The install-level ``elesim-net`` snapshot has a
                # nested network object, but that is deliberately not the
                # source of truth for an instance lifecycle.
                network = (
                    {"turn_urls": scoped_runtime_state.get("turn_urls", ())}
                    if self._scoped
                    else scoped_runtime_state.get("network")
                )
                turn = scoped_runtime_state.get("turn")
                if not isinstance(network, Mapping) or not isinstance(turn, Mapping):
                    raise RuntimeError(
                        f"managed Coturn state is missing on {host.host_id}/{unit.unit_id}"
                    )
                scoped_turn = (
                    _scoped_turn_from_instance_state(
                        scoped_runtime_state, host,
                        system_id=self._topology.system_id,
                    )
                    if self._scoped else None
                )
                expected_mode = (
                    scoped_turn["turn_mode"] if scoped_turn is not None
                    else ("managed" if self._topology.security_profile == "sros2" else "none")
                )
                expected_turn = (
                    (scoped_turn if self._scoped else _managed_turn_from_state(unit_state, host))
                    if expected_mode == "managed"
                    else None
                )
                urls = tuple(str(value) for value in network.get("turn_urls", ()))
                expected_urls = (
                    (str((expected_turn or scoped_turn)["turn_url"]),)
                    if expected_mode in {"managed", "external"}
                    else ()
                )
                if urls != expected_urls:
                    raise RuntimeError(
                        f"TURN URL did not activate on {host.host_id}/{unit.unit_id}: "
                        f"{list(urls)!r} != {list(expected_urls)!r}"
                    )
                expected_turn_values = {
                    "mode": expected_mode,
                    **(
                        {
                            "realm": str(expected_turn["turn_realm"]),
                            "public_host": str(expected_turn["turn_public_host"]),
                            "secret_file": str(expected_turn["turn_secret_file"]),
                        }
                        if expected_turn is not None
                        else ({"credential_file": str(scoped_turn["turn_credential_file"])} if scoped_turn is not None and expected_mode == "external" else {})
                    ),
                }
                for name, expected_value in expected_turn_values.items():
                    actual_value = str(turn.get(name, ""))
                    if actual_value != expected_value:
                        raise RuntimeError(
                            f"TURN {name} did not activate on "
                            f"{host.host_id}/{unit.unit_id}: "
                            f"{actual_value!r} != {expected_value!r}"
                        )
            if generation is not None and str(dds.get("security_generation", "")) != generation:
                raise RuntimeError(
                    f"security generation did not activate on {host.host_id}/{unit.unit_id}"
                )
            if generation is not None:
                security_root = PurePosixPath(unit.install_root) / "security"
                current = session.run(("readlink", str(security_root / "current")))
                if PurePosixPath(current.stdout.strip()).name != generation:
                    raise RuntimeError(
                        f"security/current does not select {generation!r} on "
                        f"{host.host_id}/{unit.unit_id}"
                    )
                session.run(("test", "-f", str(security_root / "current/manifest.json")))
                for role in unit.roles:
                    endpoint_id = endpoint_by_role[role]
                    role_key = (
                        security_root / "apps" / role / "enclaves" / "elesim"
                        / self._topology.system_id / role
                        / canonical_endpoint_key(endpoint_id) / "key.pem"
                    )
                    session.run(("test", "-f", str(role_key)))
            if unit_selected:
                session.run((str(_net_command(unit)), "doctor", "--json"))


@dataclass(frozen=True)
class RolloutResult:
    generation: str
    previous_generations: Mapping[str, str | None]


@dataclass(frozen=True)
class IssuedSecurityGeneration:
    generation: str
    bundles: Mapping[str, SecurityBundle]
    activate_authority: Callable[[], object]
    rollback_authority: Callable[[], object]


class SecurityGenerationIssuer(Protocol):
    def issue(
        self, topology: ConnectionTopology, generation: str
    ) -> IssuedSecurityGeneration: ...


class Sros2BundleIssuer:
    """Issue least-privilege enclaves and adapt verified host exports."""

    def __init__(self, authority: object) -> None:
        # Kept structurally typed so tests can inject the ROS CLI runner through
        # Sros2Authority without duplicating cryptographic code here.
        self._authority = authority

    def issue(
        self, topology: ConnectionTopology, generation: str
    ) -> IssuedSecurityGeneration:
        from .security_authority import verify_bundle
        from .security_policy import write_role_policy

        topology.validate()
        if topology.security_profile != "sros2":
            raise ValueError("SROS2 bundle issuance requires the sros2 profile")
        _safe_generation(generation)
        endpoints = {
            assignment.role: assignment.endpoint_id
            for host in topology.hosts
            for assignment in host.assignments
        }
        transaction = self._authority.begin_generation(  # type: ignore[attr-defined]
            topology.system_id, generation=generation
        )
        with transaction, tempfile.TemporaryDirectory(
            prefix="elesim-sros2-policies-"
        ) as temporary_directory:
            policy_root = Path(temporary_directory)
            identities: dict[str, object] = {}
            for role in sorted(endpoints):
                policy = write_role_policy(
                    policy_root / f"{role}.policy.xml",
                    system_id=topology.system_id,
                    role=role,
                    endpoint_id=endpoints[role],
                    endpoints=endpoints,
                )
                if not policy.is_file():
                    raise RuntimeError(f"role policy was not created: {policy}")
                identities[role] = transaction.create_enclave(
                    role, endpoints[role], policy=policy
                )
            for host in topology.hosts:
                transaction.stage_host_bundle(
                    host.host_id,
                    tuple(identities[assignment.role] for assignment in host.assignments),
                )
            transaction.publish()

        bundles: dict[str, SecurityBundle] = {}
        for host in topology.hosts:
            source = transaction.path / "bundles" / host.host_id
            verify_bundle(source)
            bundles[host.host_id] = SecurityBundle.from_directory(
                system_id=topology.system_id,
                host_id=host.host_id,
                generation=generation,
                root=source,
            )
        return IssuedSecurityGeneration(
            generation=generation,
            bundles=bundles,
            activate_authority=transaction.activate,
            rollback_authority=lambda: _rollback_authority_if_active(
                self._authority, transaction, generation
            ),
        )


@dataclass(frozen=True)
class TopologyRolloutResult:
    previous_generations: Mapping[str, str | None]


class TopologyRollout:
    """Apply a bundle-free trusted-network graph change to every host."""

    def __init__(
        self,
        topology: ConnectionTopology,
        operations: Mapping[str, HostOperations],
    ) -> None:
        self._topology = topology.validate()
        if self._topology.security_profile != "trusted-network":
            raise ValueError("TopologyRollout is for trusted-network only")
        host_ids = {host.host_id for host in self._topology.hosts}
        if set(operations) != host_ids:
            raise ValueError("HostOperations must be supplied exactly once for every host")
        self._operations = dict(operations)

    def apply(
        self, *, progress: ProgressCallback | None = None
    ) -> TopologyRolloutResult:
        hosts = self._topology.hosts
        previous: dict[str, HostActivationState] = {}
        stopped: list[ManagedHost] = []
        configured: list[ManagedHost] = []
        phase = "network-preflight"
        try:
            for host in hosts:
                _notify_progress(progress, phase, host.host_id)
                self._operations[host.host_id].runtime_network_check(host)
            phase = "preflight"
            for host in hosts:
                _notify_progress(progress, phase, host.host_id)
                capabilities = self._operations[host.host_id].preflight(host)
                capabilities.require_for(host)
            phase = "capture-current-state"
            for host in hosts:
                _notify_progress(progress, phase, host.host_id)
                previous[host.host_id] = self._operations[
                    host.host_id
                ].capture_state(host)
            phase = "stop"
            for host in hosts:
                running = previous[host.host_id].running_roles
                if not running:
                    continue
                _notify_progress(progress, phase, host.host_id)
                stopped.append(host)
                self._operations[host.host_id].stop(host, running)
            phase = "configure"
            for host in hosts:
                _notify_progress(progress, phase, host.host_id)
                configured.append(host)
                self._operations[host.host_id].configure_topology(host)
            phase = "start"
            for host in stopped:
                _notify_progress(progress, phase, host.host_id)
                self._operations[host.host_id].runtime_network_check(host)
                self._operations[host.host_id].start(
                    host, previous[host.host_id].running_roles
                )
            phase = "verify"
            for host in hosts:
                _notify_progress(progress, phase, host.host_id)
                self._operations[host.host_id].verify_topology(
                    host, previous[host.host_id].running_roles
                )
        except BaseException as exc:
            rollback_errors = _restore_hosts(
                self._operations,
                stopped=stopped,
                changed=configured,
                previous=previous,
            )
            raise RolloutError(phase, exc, rollback_errors) from exc
        return TopologyRolloutResult(
            {host_id: state.generation for host_id, state in previous.items()}
        )


class GenerationRollout:
    """Perform an all-host generation transition without a mixed live graph."""

    def __init__(
        self,
        topology: ConnectionTopology,
        operations: Mapping[str, HostOperations],
    ) -> None:
        self._topology = topology.validate()
        host_ids = {host.host_id for host in self._topology.hosts}
        if set(operations) != host_ids:
            raise ValueError("HostOperations must be supplied exactly once for every host")
        self._operations = dict(operations)

    def _prepare_hosts(
        self,
        *,
        progress: ProgressCallback | None = None,
    ) -> dict[str, HostActivationState]:
        """Validate every host before any new security generation is issued."""

        previous: dict[str, HostActivationState] = {}
        phase = "network-preflight"
        try:
            # This is deliberately before Authority generation.  It is a
            # read-only bind/route check in the runtime namespace, so an
            # unsupported Docker Desktop/WSL path cannot leave a fresh
            # security generation behind for a graph that cannot start.
            for host in self._topology.hosts:
                _notify_progress(progress, phase, host.host_id)
                self._operations[host.host_id].runtime_network_check(host)
            phase = "preflight"
            for host in self._topology.hosts:
                _notify_progress(progress, phase, host.host_id)
                capabilities = self._operations[host.host_id].preflight(host)
                capabilities.require_for(host)
            phase = "capture-current-state"
            for host in self._topology.hosts:
                _notify_progress(progress, phase, host.host_id)
                previous[host.host_id] = self._operations[
                    host.host_id
                ].capture_state(host)
        except BaseException as exc:
            # No runtime was stopped and no Authority generation exists yet.
            raise RolloutError(phase, exc) from exc
        return previous

    def apply(
        self,
        generation: str,
        bundles: Mapping[str, SecurityBundle],
        *,
        commit: Callable[[], object] | None = None,
        rollback_commit: Callable[[], object] | None = None,
        progress: ProgressCallback | None = None,
        prepared_previous: Mapping[str, HostActivationState] | None = None,
    ) -> RolloutResult:
        _safe_generation(generation)
        if self._topology.security_profile != "sros2":
            raise ValueError("security generation rollout requires the sros2 profile")
        hosts = self._topology.hosts
        host_ids = {host.host_id for host in hosts}
        if set(bundles) != host_ids:
            raise ValueError("one security bundle is required for every host")
        for host in hosts:
            bundle = bundles[host.host_id].validate()
            if bundle.system_id != self._topology.system_id:
                raise ValueError(f"bundle system_id mismatch for {host.host_id}")
            if bundle.host_id != host.host_id or bundle.generation != generation:
                raise ValueError(f"bundle target/generation mismatch for {host.host_id}")

        if prepared_previous is None:
            previous = self._prepare_hosts(progress=progress)
        else:
            host_ids = {host.host_id for host in hosts}
            if set(prepared_previous) != host_ids:
                raise ValueError(
                    "prepared host state must be supplied exactly once for every host"
                )
            previous = dict(prepared_previous)
        stopped: list[ManagedHost] = []
        staged: list[ManagedHost] = []
        switched: list[ManagedHost] = []
        phase = "stage"
        try:
            for host in hosts:
                _notify_progress(progress, phase, host.host_id)
                self._operations[host.host_id].stage(host, bundles[host.host_id])
                staged.append(host)
            phase = "stop"
            for host in hosts:
                running = previous[host.host_id].running_roles
                if not running:
                    continue
                _notify_progress(progress, phase, host.host_id)
                stopped.append(host)
                self._operations[host.host_id].stop(host, running)
            phase = "switch"
            for host in hosts:
                _notify_progress(progress, phase, host.host_id)
                switched.append(host)
                self._operations[host.host_id].activate(host, generation)
            phase = "start"
            for host in stopped:
                _notify_progress(progress, phase, host.host_id)
                self._operations[host.host_id].runtime_network_check(host)
                self._operations[host.host_id].start(
                    host, previous[host.host_id].running_roles
                )
            phase = "verify"
            for host in hosts:
                _notify_progress(progress, phase, host.host_id)
                self._operations[host.host_id].verify(
                    host, generation, previous[host.host_id].running_roles
                )
            if commit is not None:
                phase = "commit-authority"
                _notify_progress(progress, phase, None)
                commit()
        except BaseException as exc:
            rollback_errors = list(
                self._restore_previous(stopped, switched, previous)
            )
            if phase == "commit-authority" and rollback_commit is not None:
                try:
                    rollback_commit()
                except BaseException as rollback_exc:
                    rollback_errors.append(rollback_exc)
            for host in reversed(staged):
                try:
                    self._operations[host.host_id].discard_generation(host, generation)
                except BaseException as rollback_exc:
                    rollback_errors.append(rollback_exc)
            raise RolloutError(phase, exc, rollback_errors) from exc
        return RolloutResult(
            generation,
            {host_id: state.generation for host_id, state in previous.items()},
        )

    def issue_and_apply(
        self,
        issuer: SecurityGenerationIssuer,
        generation: str,
        *,
        progress: ProgressCallback | None = None,
    ) -> RolloutResult:
        previous = self._prepare_hosts(progress=progress)
        _notify_progress(progress, "issue", None)
        try:
            issued = issuer.issue(self._topology, generation)
            if issued.generation != generation:
                raise ValueError("issuer returned a different security generation")
        except BaseException as exc:
            raise RolloutError("issue", exc) from exc
        return self.apply(
            generation,
            issued.bundles,
            commit=issued.activate_authority,
            rollback_commit=issued.rollback_authority,
            progress=progress,
            prepared_previous=previous,
        )

    def _restore_previous(
        self,
        stopped: Sequence[ManagedHost],
        switched: Sequence[ManagedHost],
        previous: Mapping[str, HostActivationState],
    ) -> tuple[BaseException, ...]:
        return _restore_hosts(
            self._operations,
            stopped=stopped,
            changed=switched,
            previous=previous,
        )


def ssh_sha256_fingerprint(key_bytes: bytes) -> str:
    digest = hashlib.sha256(key_bytes).digest()
    return "SHA256:" + base64.b64encode(digest).decode("ascii").rstrip("=")


def _notify_progress(
    callback: ProgressCallback | None, phase: str, host_id: str | None
) -> None:
    if callback is not None:
        callback(phase, host_id)


def _restore_hosts(
    operations: Mapping[str, HostOperations],
    *,
    stopped: Sequence[ManagedHost],
    changed: Sequence[ManagedHost],
    previous: Mapping[str, HostActivationState],
) -> tuple[BaseException, ...]:
    errors: list[BaseException] = []
    for host in reversed(stopped):
        try:
            operations[host.host_id].stop(host, previous[host.host_id].running_roles)
        except BaseException as exc:
            errors.append(exc)
    for host in reversed(changed):
        try:
            operations[host.host_id].rollback(host, previous[host.host_id])
        except BaseException as exc:
            errors.append(exc)
    if errors:
        return tuple(errors)
    for host in stopped:
        try:
            operations[host.host_id].start(host, previous[host.host_id].running_roles)
        except BaseException as exc:
            errors.append(exc)
    return tuple(errors)


def _unit_for_target(target: ManagedHost | DeploymentUnit) -> DeploymentUnit:
    return target.primary_unit if isinstance(target, ManagedHost) else target


def _net_command(target: ManagedHost | DeploymentUnit) -> PurePosixPath:
    return PurePosixPath(_unit_for_target(target).bin_dir) / "elesim-net"


def _status_command(target: ManagedHost | DeploymentUnit) -> PurePosixPath:
    return PurePosixPath(_unit_for_target(target).bin_dir) / "elesim-status"


def _tailscale_command(target: ManagedHost | DeploymentUnit) -> PurePosixPath:
    return PurePosixPath(_unit_for_target(target).bin_dir) / "elesim-tailscale"


def _parse_tailscale_status(payload: str) -> tuple[str, str]:
    """Parse the wrapper's deliberately small, non-secret status document."""

    try:
        raw = json.loads(payload)
    except json.JSONDecodeError as exc:
        raise RuntimeError("Tailscale sidecar status is invalid") from exc
    if not isinstance(raw, Mapping):
        raise RuntimeError("Tailscale sidecar status is not an object")
    backend = str(raw.get("BackendState", raw.get("backend_state", ""))).strip()
    ipv4 = str(raw.get("IPv4", raw.get("ipv4", ""))).strip()
    if ipv4:
        try:
            address = ipaddress.ip_address(ipv4)
        except ValueError as exc:
            raise RuntimeError("Tailscale sidecar status has an invalid IPv4") from exc
        if address.version != 4 or address.is_unspecified or address.is_loopback:
            raise RuntimeError("Tailscale sidecar status has an unusable IPv4")
        ipv4 = str(address)
    return backend, ipv4


def _compose_command(
    target: ManagedHost | DeploymentUnit,
) -> tuple[str, ...]:
    unit = _unit_for_target(target)
    compose = PurePosixPath(unit.install_root) / "containers/compose.yaml"
    return (
        str(PurePosixPath(unit.bin_dir) / "elesim-compose"),
        "-f",
        str(compose),
    )


def _compose_build_command(target: ManagedHost | DeploymentUnit) -> tuple[str, ...]:
    unit = _unit_for_target(target)
    compose = PurePosixPath(unit.install_root) / "containers/compose.yaml"
    return (
        str(PurePosixPath(unit.bin_dir) / "elesim-compose"),
        "--progress",
        "plain",
        "-f",
        str(compose),
    )


def _robot_service(target: ManagedHost | DeploymentUnit) -> str:
    roles = target.roles
    if roles != ("robot",):
        raise ValueError("systemd lifecycle is reserved for the native Robot unit")
    return "elesim-robot.service"


def _selected_roles(
    host: ManagedHost, roles: Sequence[str] | None
) -> tuple[str, ...]:
    selected = host.roles if roles is None else tuple(str(role) for role in roles)
    if len(set(selected)) != len(selected) or not set(selected).issubset(host.roles):
        raise ValueError(f"runtime role selection escapes {host.host_id!r}: {selected!r}")
    return tuple(role for role in host.roles if role in set(selected))


def _lifecycle_command(
    target: ManagedHost | DeploymentUnit,
    *,
    action: str,
    roles: Sequence[str] | None = None,
    include_coturn: bool = False,
    runtime_options: RuntimeLaunchOptions | None = None,
    viewer_user: str | None = None,
    system_id: str | None = None,
    scoped: bool | None = None,
) -> tuple[str, ...]:
    if action not in {"start", "stop", "build", "launch", "status", "logs"}:
        raise ValueError(f"unsupported lifecycle action: {action!r}")
    if scoped is None:
        scoped = system_id is not None
    if scoped:
        if not system_id:
            raise ValueError("scoped lifecycle requires the topology system_id")
        _safe_identifier(str(system_id), name="system_id")
    if isinstance(target, ManagedHost):
        selected = _selected_roles(target, roles)
        unit = target.primary_unit
    else:
        unit = target
        selected = unit.roles if roles is None else tuple(str(role) for role in roles)
        if len(set(selected)) != len(selected) or not set(selected).issubset(unit.roles):
            raise ValueError(f"runtime role selection escapes {unit.unit_id!r}: {selected!r}")
    services = tuple(selected)
    if scoped and unit.install_mode != "container":
        raise ValueError("scoped lifecycle is available only for container units")
    if scoped:
        # InstanceRuntime publishes exact wrappers under instances/<system>.
        # No role/service argument is forwarded: doing so would allow a
        # manager to escape the registered service set.
        if action == "build":
            raise ValueError(
                "scoped instances use immutable published releases; build is not a lifecycle action"
            )
        instance_action = {
            "start": "up",
            "launch": "up",
            "stop": "down",
            "status": "status",
            "logs": "logs",
        }[action]
        if runtime_options is not None:
            raise ValueError(
                "scoped instance lifecycle does not accept runtime GPU/viewer overrides; "
                "configure the instance compute policy at registration"
            )
        return (
            str(PurePosixPath(unit.bin_dir) / "elesim-instance"),
            str(system_id),
            instance_action,
            *(('--no-build',) if instance_action == "up" else ()),
        )
    if include_coturn and "sim" in selected and "coturn" not in services:
        services = (*services, "coturn")
    if unit.lifecycle == "compose":
        if action == "stop":
            return (*_compose_command(unit), "stop", *services)
        if action == "start":
            # Security/topology transactions resume the exact containers that
            # were running before the switch.  They never build or recreate.
            return (*_compose_command(unit), "start", *services)
        if action == "build":
            return (*_compose_build_command(unit), "build", *selected)
        launch_flags = (
            ()
            if runtime_options is None
            else runtime_options.launcher_flags(selected)
        )
        if (
            runtime_options is not None
            and runtime_options.viewer
            and viewer_user is not None
        ):
            if not _VIEWER_USER.fullmatch(viewer_user):
                raise ValueError("viewer SSH user is invalid")
            launch_flags = (*launch_flags, "--viewer-user", viewer_user)
        return (
            str(PurePosixPath(unit.bin_dir) / "elesim-up"),
            "--no-build",
            *launch_flags,
            *services,
        )
    if action in {"build", "launch"}:
        if action == "build":
            return ("true",)
        action = "start"
    return ("sudo", "-n", "systemctl", action, _robot_service(unit))


def _configuration_command(
    target: ManagedHost | DeploymentUnit,
    dds: Mapping[str, Any],
    *,
    sim_id: str = "",
    pilot_id: str = "",
    ui_id: str = "",
    robot_id: str = "",
) -> tuple[str, ...]:
    required = (
        "system_id",
        "domain_id",
        "rmw_implementation",
        "discovery_mode",
        "interface",
        "security_profile",
    )
    missing = [name for name in required if name not in dds]
    if missing:
        raise ValueError("DDS configuration is missing: " + ", ".join(missing))
    arguments: list[str] = [
        str(_net_command(target)),
        "configure",
        "--non-interactive",
        "--dds-system-id",
        str(dds["system_id"]),
        "--dds-domain-id",
        str(dds["domain_id"]),
        "--dds-rmw-implementation",
        str(dds["rmw_implementation"]),
        "--dds-discovery-mode",
        str(dds["discovery_mode"]),
        "--dds-interface",
        str(dds["interface"]),
        "--dds-security-profile",
        str(dds["security_profile"]),
    ]
    for role in target.roles:
        arguments.extend(("--assigned-role", role))
    peers_raw = dds.get("static_peers", ())
    if not isinstance(peers_raw, Sequence) or isinstance(
        peers_raw, (str, bytes, bytearray)
    ):
        raise ValueError("DDS static_peers must be an array")
    peers = tuple(str(value) for value in peers_raw)
    if peers:
        for peer in peers:
            arguments.extend(("--dds-static-peer", peer))
    else:
        arguments.append("--clear-dds-static-peers")

    security_profile = str(dds["security_profile"])
    if security_profile == "sros2":
        provisioning = str(dds.get("security_provisioning", "external"))
        arguments.extend(("--dds-security-provisioning", provisioning))
        generation = str(dds.get("security_generation", ""))
        bundle = str(dds.get("security_bundle", ""))
        keystore = str(dds.get("keystore", ""))
        enclave = str(dds.get("enclave", ""))
        if generation:
            arguments.extend(("--dds-security-generation", generation))
        if bundle:
            arguments.extend(("--dds-security-bundle", bundle))
        if keystore:
            arguments.extend(("--dds-keystore", keystore))
        if enclave:
            arguments.extend(("--dds-enclave", enclave))
    if sim_id:
        arguments.extend(("--sim-id", sim_id))
    if pilot_id:
        arguments.extend(("--pilot-id", pilot_id))
    if ui_id:
        arguments.extend(("--ui-id", ui_id))
    if robot_id:
        arguments.extend(("--robot-id", robot_id))
    turn_mode = str(dds.get("turn_mode", "")).strip()
    if not turn_mode:
        return tuple(arguments)
    if turn_mode == "managed":
        turn_values = (
            "turn_url",
            "turn_realm",
            "turn_public_host",
            "turn_secret_file",
        )
        missing_turn = [name for name in turn_values if not str(dds.get(name, "")).strip()]
        if missing_turn:
            raise ValueError(
                "managed Coturn configuration is missing: "
                + ", ".join(missing_turn)
            )
        arguments.extend(
            (
                "--turn-mode",
                "managed",
                "--turn-url",
                str(dds["turn_url"]),
                "--turn-realm",
                str(dds["turn_realm"]),
                "--turn-public-host",
                str(dds["turn_public_host"]),
                "--turn-secret-file",
                str(dds["turn_secret_file"]),
            )
        )
    elif turn_mode == "external":
        turn_values = ("turn_url", "turn_credential_file")
        missing_turn = [name for name in turn_values if not str(dds.get(name, "")).strip()]
        if missing_turn:
            raise ValueError(
                "external TURN configuration is missing: " + ", ".join(missing_turn)
            )
        arguments.extend(
            (
                "--turn-mode", "external",
                "--turn-url", str(dds["turn_url"]),
                "--turn-credential-file", str(dds["turn_credential_file"]),
            )
        )
    elif turn_mode == "none":
        arguments.extend(("--turn-mode", "none", "--clear-turn"))
    else:
        raise ValueError(f"unsupported managed Sim TURN mode: {turn_mode!r}")
    return tuple(arguments)


def _rollback_authority_if_active(
    authority: object, transaction: object, generation: str
) -> object | None:
    active = authority.active()  # type: ignore[attr-defined]
    if active is None or str(active.generation) != generation:
        return None
    return transaction.rollback()  # type: ignore[attr-defined]


def _check_bundle_target(host: ManagedHost, bundle: SecurityBundle) -> None:
    host.validate()
    bundle.validate()
    if bundle.host_id != host.host_id:
        raise ValueError(
            f"security bundle for {bundle.host_id!r} cannot be sent to {host.host_id!r}"
        )


def _stage_unit_bundle(
    session: SshSession,
    host: ManagedHost,
    unit: DeploymentUnit,
    security_root: PurePosixPath,
    bundle: SecurityBundle,
) -> None:
    """Stage one role-filtered bundle under one installation prefix."""

    generation_root = security_root / "generations"
    final_root = generation_root / bundle.generation
    stage_root = security_root / ".staging" / (
        f"{bundle.generation}-{unit.unit_id}-{uuid.uuid4().hex}"
    )
    session.run(("mkdir", "-p", str(generation_root), str(stage_root.parent)))
    exists = session.run(("test", "-e", str(final_root)), check=False)
    symlink = session.run(("test", "-L", str(final_root)), check=False)
    if exists.exit_status == 0 or symlink.exit_status == 0:
        raise FileExistsError(
            f"security generation already exists on {host.host_id}/{unit.unit_id}: "
            f"{bundle.generation}"
        )
    session.run(("mkdir", str(stage_root)))
    session.run(("chmod", "0700", str(stage_root)))
    try:
        for file in sorted(bundle.files, key=lambda item: item.relative_path):
            file.validate()
            session.upload_bytes(
                stage_root / PurePosixPath(file.relative_path),
                file.content,
                file.mode,
            )
        manifest_bytes = bundle.manifest_bytes()
        session.upload_bytes(stage_root / "manifest.json", manifest_bytes, 0o600)
        expected_digests = {
            file.relative_path: hashlib.sha256(file.content).hexdigest()
            for file in bundle.files
        }
        expected_digests["manifest.json"] = hashlib.sha256(manifest_bytes).hexdigest()
        for relative_path, expected_digest in expected_digests.items():
            result = session.run(("sha256sum", "--", str(stage_root / relative_path)))
            actual = result.stdout.partition(" ")[0].strip().lower()
            if not hmac.compare_digest(actual, expected_digest):
                raise RuntimeError(
                    f"staged security file digest mismatch on "
                    f"{host.host_id}/{unit.unit_id}: {relative_path}"
                )
        session.run(("mv", str(stage_root), str(final_root)))
    except BaseException:
        session.run(("rm", "-rf", "--", str(stage_root)), check=False)
        raise


def _activate_unit_generation(
    session: SshSession,
    host: ManagedHost,
    unit: DeploymentUnit,
    security_root: PurePosixPath,
    *,
    generation: str,
    roles: Sequence[str] | None = None,
) -> None:
    final_root = security_root / "generations" / generation
    temporary = security_root / f".current-{unit.unit_id}-{uuid.uuid4().hex}"
    current = security_root / "current"
    session.run(("test", "-d", str(final_root)))
    session.run(("ln", "-s", str(final_root), str(temporary)))
    try:
        session.run(("mv", "-Tf", str(temporary), str(current)))
    except BaseException:
        session.run(("rm", "-f", "--", str(temporary)), check=False)
        raise
    SshHostOperations._sync_app_views(
        session,
        host,
        security_root,
        generation=generation,
        roles=unit.roles if roles is None else roles,
    )


def _safe_identifier(value: str, *, name: str) -> str:
    text = str(value)
    if (
        not text
        or len(text) > 128
        or text in {".", ".."}
        or any(
            character
            not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789._-"
            for character in text
        )
    ):
        raise ValueError(f"{name} is not a safe identifier")
    return text


def _safe_generation(value: str) -> str:
    text = str(value)
    if (
        not text
        or len(text) > 96
        or text[0] not in "abcdefghijklmnopqrstuvwxyz0123456789"
        or any(
            character not in "abcdefghijklmnopqrstuvwxyz0123456789._-"
            for character in text
        )
    ):
        raise ValueError("generation is not a safe lower-case identifier")
    return text


def _safe_relative_path(value: str) -> PurePosixPath:
    if not isinstance(value, str) or not value or "\\" in value or "\x00" in value:
        raise ValueError("security bundle path must be a non-empty POSIX path")
    path = PurePosixPath(value)
    if path.is_absolute() or any(part in {"", ".", ".."} for part in path.parts):
        raise ValueError(f"security bundle path escapes its generation: {value!r}")
    return path


def _safe_remote_root(value: str) -> PurePosixPath:
    if not isinstance(value, str) or "\x00" in value:
        raise ValueError("security_root must be a remote absolute path")
    path = PurePosixPath(value)
    names = [part for part in path.parts if part != "/"]
    if not path.is_absolute() or ".." in path.parts or len(names) < 2:
        raise ValueError("security_root must be a contained absolute path")
    return path


def _sftp_makedirs(sftp: object, path: PurePosixPath) -> None:
    current = PurePosixPath("/")
    for part in path.parts:
        if part == "/":
            continue
        current /= part
        try:
            attributes = sftp.stat(str(current))  # type: ignore[attr-defined]
            if not stat.S_ISDIR(attributes.st_mode):
                raise NotADirectoryError(str(current))
        except OSError:
            sftp.mkdir(str(current), mode=0o700)  # type: ignore[attr-defined]


__all__ = [
    "GenerationRollout",
    "HostActivationState",
    "HostKeyVerificationError",
    "HostOperations",
    "InstalledElesimLifecycle",
    "IssuedSecurityGeneration",
    "LocalHostOperations",
    "MAX_BUNDLE_BYTES",
    "MAX_BUNDLE_FILE_BYTES",
    "MAX_BUNDLE_FILES",
    "ParamikoConnector",
    "ProgressCallback",
    "RemoteCapabilities",
    "RemoteCommandError",
    "RemoteCommandResult",
    "RemoteLifecycle",
    "RolloutError",
    "RolloutResult",
    "SecurityBundle",
    "SecurityGenerationIssuer",
    "SecurityFile",
    "Sros2BundleIssuer",
    "SshAuthenticationError",
    "SshConnectionError",
    "SshConnector",
    "SshHostOperations",
    "SshSession",
    "TopologyRollout",
    "TopologyRolloutResult",
    "ssh_sha256_fingerprint",
]
