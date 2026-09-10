"""CLI and concrete rollout runner for the browser connection manager."""

from __future__ import annotations

import argparse
import concurrent.futures
import copy
import fcntl
import hashlib
import json
import os
import re
import stat
import sys
import tempfile
import threading
from dataclasses import replace
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from .connection_gui import ConnectionJobCancelled, run_connection_gui
from .connection_manager import (
    ConnectionTopology,
    DeploymentUnit,
    ManagedHost,
    ROLES,
    operator_home_path,
    resolve_ssh_identity_path,
)
from ._security_storage import SecurityAuthorityError, secure_absolute
from .secure_deployment import (
    GenerationRollout,
    HostActivationState,
    HostOperations,
    InstalledElesimLifecycle,
    LocalHostOperations,
    ParamikoConnector,
    RolloutError,
    RuntimeLaunchOptions,
    SecurityBundle,
    Sros2BundleIssuer,
    SshHostOperations,
    TopologyRollout,
)
from .instance_identity import project_name
from .instances import InstanceEndpoint, InstanceRegistry, InstanceState
from .ownership import OwnershipManifest
from .releases import ReleaseManifest, list_releases, release_key
from .security_authority import Sros2Authority, new_generation_id
from .state import InstallState, NetworkSettings, TurnSettings


Log = Callable[[str], None]
# Genesis scene construction is performed after the Sim DDS endpoint starts.
# Keep the liveness gate bounded, but long enough for a cold GPU/container
# startup; UI session/media readiness remains a separate retrying handshake.
_DDS_READINESS_TIMEOUT_S = 5 * 60.0
_SCOPED_JOURNAL_SCHEMA = 2
_SCOPED_JOURNAL_SCOPE = "scoped-instance"
_SCOPED_TERMINAL_STATUSES = frozenset({
    "completed",
    "rolled-back",
})
_SCOPED_JOURNAL_STATUSES = frozenset({
    "running",
    "failed",
    "blocked",
    "completed",
    "rolled-back",
})
_SCOPED_JOURNAL_PHASES = frozenset({
    "planned",
    "issuing",
    "issued",
    "register",
    "activate-authority",
    "authority-active",
    "rollback",
    "scoped-register",
    "recover",
    "complete",
})
_SCOPED_JOURNAL_TOP_LEVEL = frozenset({
    "schema_version",
    "scope",
    "transaction_id",
    "system_id",
    "action",
    "security_profile",
    "status",
    "phase",
    "host_id",
    "topology_digest",
    "created_at",
    "updated_at",
    "authority",
    "units",
    "last_error",
    "rollback_errors",
})
_SCOPED_JOURNAL_UNIT_FIELDS = frozenset({
    "host_id",
    "unit_id",
    "install_uuid",
    "project",
    "roles",
    "before",
    "target",
    "before_release_key",
    "target_release_key",
    "bundle_manifest_sha256",
    "status",
    "last_error",
})
_SAFE_SCOPED_TOKEN_RE = re.compile(r"[a-z0-9][a-z0-9_.-]{0,95}\Z")
_SHA256_HEX_RE = re.compile(r"[0-9a-f]{64}\Z")


def _exception_detail(error: BaseException) -> str:
    """Retain a useful type for empty or context-manager-only errors."""

    message = str(error).strip()
    name = error.__class__.__name__
    if not message:
        return name
    if (
        message in {name, "__enter__", "__exit__"}
        or "__enter__" in message
        or "__exit__" in message
    ):
        return message if message.startswith(f"{name}:") else f"{name}: {message}"
    return message


class RuntimeRollbackError(RuntimeError):
    """A runtime transition and one or more compensating actions failed."""

    def __init__(
        self,
        cause: BaseException,
        rollback_errors: Sequence[tuple[str, BaseException]],
        *,
        rollback_action: str = "stop",
    ) -> None:
        failures = tuple(rollback_errors)
        details = "; ".join(
            f"{host_id}: {_exception_detail(error)[:512]}"
            for host_id, error in failures
        )
        super().__init__(
            f"runtime lifecycle failed: {_exception_detail(cause)}; rollback {rollback_action} "
            f"also failed: {details}"
        )
        self.cause = cause
        self.rollback_errors = failures
        self.rollback_action = rollback_action


class OperationCloseError(RuntimeError):
    """One or more host-operation sessions failed to close."""

    def __init__(
        self,
        cause: BaseException | None,
        close_errors: Sequence[tuple[str, BaseException]],
    ) -> None:
        failures = tuple(close_errors)
        details = "; ".join(
            f"{host_id}: {_exception_detail(error)[:512]}"
            for host_id, error in failures
        )
        if cause is None:
            message = f"host operation cleanup failed: {details}"
        else:
            message = (
                f"host operation failed: {_exception_detail(cause)}; "
                f"session cleanup also failed: {details}"
            )
        super().__init__(message)
        self.cause = cause
        self.close_errors = failures


class _BuildLogForwarder:
    """Turn arbitrary command chunks into bounded host-labelled lines."""

    def __init__(self, host: ManagedHost, log: Log, *, phase: str = "build") -> None:
        self._prefix = f"{phase} {host.host_id}"
        self._log = log
        self._pending = {"stdout": "", "stderr": ""}
        self._lock = threading.Lock()

    def __call__(self, stream: str, text: str) -> None:
        if stream not in self._pending:
            raise ValueError(f"unknown build output stream: {stream!r}")
        with self._lock:
            pending = self._pending[stream] + text.replace("\r", "\n")
            lines = pending.split("\n")
            self._pending[stream] = lines.pop()
            for line in lines:
                if line:
                    self._log(f"{self._prefix} [{stream}] {line}")
            # A tool that never emits a newline must not grow manager memory.
            if len(self._pending[stream]) > 8 * 1024:
                self._log(
                    f"{self._prefix} [{stream}] "
                    f"{self._pending[stream][:8 * 1024]}"
                )
                self._pending[stream] = self._pending[stream][8 * 1024 :]

    def flush(self) -> None:
        with self._lock:
            for stream, line in self._pending.items():
                if line:
                    self._log(f"{self._prefix} [{stream}] {line}")
                self._pending[stream] = ""


class ConnectionDeploymentRunner:
    """Apply one saved topology from the operator-owned management host."""

    def __init__(
        self,
        authority_root: Path,
        *,
        topology_state_path: Path | None = None,
        local_install_root: Path | None = None,
        local_bin_dir: Path | None = None,
        instance_release_key: str | None = None,
        instance_turn: TurnSettings | None = None,
        instance_turn_urls: Sequence[str] = (),
    ) -> None:
        try:
            self.authority_root = secure_absolute(authority_root)
        except SecurityAuthorityError as exc:
            raise ValueError(
                f"connection authority root must not contain symlinks: {authority_root}"
            ) from exc
        self.topology_state_path = (
            None
            if topology_state_path is None
            else topology_state_path.expanduser()
        )
        self.local_install_root = (
            None
            if local_install_root is None
            else local_install_root.expanduser().resolve()
        )
        self.local_bin_dir = (
            None if local_bin_dir is None else local_bin_dir.expanduser().resolve()
        )
        self.instance_release_key = (
            None if instance_release_key is None else str(instance_release_key).strip()
        )
        self.instance_turn = None if instance_turn is None else instance_turn.validate()
        self.instance_turn_urls = tuple(str(value) for value in instance_turn_urls)
        NetworkSettings(turn_urls=self.instance_turn_urls).validate()
        self._runtime_launch_options: RuntimeLaunchOptions | None = None

    def set_runtime_launch_options(
        self, options: RuntimeLaunchOptions | None
    ) -> None:
        """Set one browser-requested launch override for the next job only."""

        self._runtime_launch_options = options

    def __call__(
        self,
        topology: ConnectionTopology,
        action: str,
        log: Log,
    ) -> ConnectionTopology:
        runtime_options = self._runtime_launch_options
        self._runtime_launch_options = None
        topology.validate()
        self._validate_management_host(topology)
        scoped_install = (
            self.local_install_root is not None
            and self._local_install_scope() is True
        )
        role_counts = {
            role: sum(
                assignment.role == role
                for host in topology.hosts
                for assignment in host.assignments
            )
            for role in ROLES
        }
        repeated_roles = sorted(role for role, count in role_counts.items() if count > 1)
        if repeated_roles:
            raise ValueError(
                "동일 역할의 복수 인스턴스 실행은 아직 지원하지 않습니다: "
                + ", ".join(repeated_roles)
            )
        supported_actions = {
            "prepare",
            "provision",
            "deploy",
            "rotate",
            "recover",
            "start",
            "stop",
            "check",
        }
        if action not in supported_actions:
            raise ValueError(f"지원하지 않는 연결 작업: {action!r}")
        if scoped_install and action == "recover":
            # A scoped transaction has its own durable journal and recovery
            # boundary.  The install-wide recovery path remains below for
            # legacy (non-scoped) topologies.
            pass
        authority: Sros2Authority | None = None
        active = None
        if action not in {"start", "stop", "check", "recover"}:
            if topology.security_profile == "trusted-network":
                if action == "prepare":
                    action = "deploy"
                elif action != "deploy":
                    raise ValueError(
                        "trusted-network에서는 deploy만 사용할 수 있습니다"
                    )
            else:
                authority = Sros2Authority(
                    self.authority_root / topology.system_id
                )
                active = authority.active()
                if action == "prepare":
                    action = "rotate" if active is not None else "provision"
                if action in {"provision", "deploy"} and active is not None:
                    raise ValueError(
                        "이미 활성 SROS2 generation이 있습니다. 새 generation은 "
                        "rotate로 교체하십시오. provision/deploy를 반복하지 않습니다."
                    )
                if action == "rotate" and active is None:
                    raise ValueError(
                        "활성 SROS2 generation이 없습니다. 먼저 provision하십시오."
                    )
                if action not in {"provision", "deploy", "rotate"}:
                    raise ValueError(f"지원하지 않는 연결 작업: {action!r}")
        elif (
            action == "recover"
            and topology.security_profile != "sros2"
            and not scoped_install
        ):
            raise ValueError("복구는 managed SROS2 topology에서만 사용합니다")

        operations = self._operations(topology)
        journal: dict[str, object] | None = None
        scoped_lock: int | None = None

        if scoped_install and action in {
            "deploy", "provision", "rotate", "recover",
        }:
            # The check must happen while holding the same per-system lock as
            # the subsequent journal writes.  Otherwise two manager windows
            # can both observe a clean journal and interleave registrations.
            scoped_lock = self._acquire_scoped_transaction_lock(topology)
            try:
                if action in {"deploy", "provision", "rotate"}:
                    self._refuse_unresolved_scoped_journal(topology)
                if (
                    scoped_install
                    and topology.security_profile == "sros2"
                    and action in {"deploy", "provision", "rotate"}
                ):
                    # Re-read Authority state after waiting for the lock.  A
                    # second manager process may have completed a first
                    # provision while this one was waiting; the pre-lock
                    # snapshot must never decide whether a generation is
                    # allowed.
                    current = Sros2Authority(
                        self.authority_root / topology.system_id
                    ).active()
                    if action in {"deploy", "provision"} and current is not None:
                        raise ValueError(
                            "이미 활성 SROS2 generation이 있습니다. 새 generation은 "
                            "rotate로 교체하십시오. provision/deploy를 반복하지 않습니다."
                        )
                    if action == "rotate" and current is None:
                        raise ValueError(
                            "활성 SROS2 generation이 없습니다. 먼저 provision하십시오."
                        )
            except BaseException:
                try:
                    self._close_operations(operations)
                finally:
                    self._release_scoped_transaction_lock(scoped_lock)
                raise

        def progress(phase: str, host_id: str | None) -> None:
            if journal is not None:
                journal.update({"phase": phase, "host_id": host_id or ""})
                self._write_transaction_journal(topology, journal)
            if host_id is None:
                log(phase)
                return
            host = topology.host(host_id)
            log(f"{phase}: {host.host_id}")

        try:
            if action in {
                "provision",
                "deploy",
                "rotate",
                "start",
            }:
                log("호스트별 런타임 네트워크 인프라를 준비합니다.")
                discovered_addresses: dict[str, str] = {}
                for host in topology.hosts:
                    log(f"network: {host.host_id}")
                    output = _BuildLogForwarder(host, log, phase="network")
                    try:
                        discovered = operations[host.host_id].prepare_runtime_network(
                            host, output
                        )
                        if discovered:
                            discovered_addresses[host.host_id] = discovered
                    finally:
                        output.flush()
                if discovered_addresses:
                    updated_hosts = tuple(
                        replace(
                            host,
                            dds=replace(
                                host.dds,
                                address=discovered_addresses[host.host_id],
                                interface="tailscale0",
                                address_source="tailscale",
                            ),
                        )
                        if host.host_id in discovered_addresses
                        else host
                        for host in topology.hosts
                    )
                    # A Tailscale node never forwards multicast discovery
                    # between hosts.  The web form normally switches to
                    # static discovery as soon as a 100.64/10 address or a
                    # tailscale* interface is visible, but a freshly enrolled
                    # Docker Desktop sidecar has no address until this step.
                    # Normalize that first-enrollment case before validating
                    # and persisting the factual endpoint update.
                    updated_graph = topology.dds_graph
                    if (
                        len(updated_hosts) > 1
                        and updated_graph.discovery_mode == "multicast"
                    ):
                        updated_graph = replace(
                            updated_graph,
                            discovery_mode="static",
                        )
                    updated_topology = replace(
                        topology,
                        hosts=updated_hosts,
                        dds_graph=updated_graph,
                    ).validate()
                    if updated_topology != topology:
                        if self.topology_state_path is None:
                            raise RuntimeError(
                                "Tailscale sidecar DDS endpoint가 변경되었지만 "
                                "topology state path가 없어 안전하게 저장할 수 없습니다"
                            )
                        updated_topology.save(self.topology_state_path)
                        previous_operations = operations
                        operations = {}
                        self._close_operations(previous_operations)
                        topology = updated_topology
                        self._validate_management_host(topology)
                        operations = self._operations(topology)
                        for host_id, address in sorted(discovered_addresses.items()):
                            log(
                                f"DDS sidecar endpoint: {host_id} = {address} "
                                "(SSH management endpoint unchanged)"
                            )
                        if action in {"start", "recover"}:
                            raise RuntimeError(
                                "Tailscale sidecar DDS endpoint가 변경되어 저장했습니다. "
                                "실행 전에 '보안 및 실행 준비'를 다시 수행하십시오."
                            )
            # Scoped container installations are registered only after their
            # host networking has been prepared and any newly discovered DDS
            # address has been persisted.  They must never fall through to
            # the install-wide topology/security writers below.
            if scoped_install and action == "recover":
                journal = self._load_scoped_journal(topology)
                if journal is None:
                    raise RuntimeError(
                        "scoped recovery has no pending transaction journal"
                    )
                if journal.get("status") in _SCOPED_TERMINAL_STATUSES:
                    raise RuntimeError(
                        "scoped recovery has no unresolved transaction journal"
                    )
                self._recover_scoped_transaction(
                    topology,
                    operations,
                    journal,
                    log,
                )
                return topology
            if scoped_install and action in {"deploy", "provision", "rotate"}:
                native_units = [
                    f"{host.host_id}/{unit.unit_id}"
                    for host in topology.hosts
                    for unit in host.robot_units
                ]
                if native_units:
                    raise ValueError(
                        "scoped container registration cannot silently skip native Robot "
                        "units: " + ", ".join(native_units)
                    )
                journal = self._new_scoped_journal(action, topology)
                try:
                    result = self._deploy_scoped_units(
                        topology,
                        action=action,
                        authority=authority,
                        operations=operations,
                        log=log,
                        journal=journal,
                    )
                except BaseException as exc:
                    # Planning is deliberately non-durable.  If release or
                    # identity resolution fails before a complete intent is
                    # recorded, there is no host mutation to recover and a
                    # stale empty journal must not block the next attempt.
                    if journal.get("transaction_id") and journal.get("units"):
                        if journal.get("status") not in {
                            "rolled-back", "blocked", "completed"
                        }:
                            journal.update(
                                {
                                    "status": "failed",
                                    "last_error": _exception_detail(exc)[:1024],
                                }
                            )
                        self._write_transaction_journal(topology, journal)
                    raise
                journal.update({"status": "completed", "phase": "complete"})
                self._write_transaction_journal(topology, journal)
                self._log_committed(
                    log,
                    "모든 scoped instance 등록이 원자적으로 완료되었습니다.",
                )
                return result
            if action == "recover":
                journal = self._new_transaction_journal(action)
                self._write_transaction_journal(topology, journal)
                try:
                    self._recover_managed_security(topology, operations, log)
                except BaseException as exc:
                    journal.update({"status": "failed", "error": str(exc)[:1024]})
                    self._write_transaction_journal(topology, journal)
                    raise
                journal.update({"status": "completed", "phase": "complete"})
                self._write_transaction_journal(topology, journal)
                return topology
            if action in {"start", "stop", "check"}:
                if action == "check":
                    self._check_hosts(topology, operations, log)
                    return topology
                hosts = list(topology.hosts)
                if action == "start":
                    log("모든 호스트의 런타임 네트워크를 사전 점검합니다.")
                    for host in hosts:
                        log(f"preflight: {host.host_id}")
                        # This is a cheap interface-visibility probe, not a
                        # DDS discovery or hardware test.  It is kept outside
                        # security generation preflight so a valid
                        # tailscale0 topology can be provisioned before the
                        # selected runtime backend is started.
                        operations[host.host_id].runtime_network_check(host)
                        capabilities = operations[host.host_id].preflight(host)
                        capabilities.require_for(host)
                        operations[host.host_id].runtime_launch_preflight(host)
                if action == "start":
                    for host in hosts:
                        running_roles = self._status_running_roles(
                            host,
                            operations[host.host_id].status(host),
                        )
                        if running_roles:
                            raise RuntimeError(
                                f"{host.host_id}에서 이미 실행 중인 역할이 있습니다: "
                                f"{', '.join(running_roles)}. 연결 관리자에서는 이미 "
                                "실행 중인 런타임을 재시작하지 않습니다. 각 호스트에서 "
                                "elesim-up을 사용하거나, 먼저 elesim-down으로 정리한 "
                                "뒤 다시 시작하십시오."
                            )
                if action == "stop":
                    log("활성 역할의 런타임을 정지합니다.")
                    stop_errors: list[tuple[str, str, BaseException]] = []
                    for host in reversed(hosts):
                        log(f"stop: {host.host_id}")
                        try:
                            operations[host.host_id].stop(host)
                        except BaseException as exc:
                            stop_errors.append((host.host_id, "stop", exc))
                            continue
                        # A terminal manager stop must revoke the temporary
                        # X11 local-user grant created by a Viewer launch.
                        # Security-rotation stop/start paths deliberately do
                        # not call this because the same Viewer resumes.
                        try:
                            operations[host.host_id].cleanup_viewer(host)
                        except BaseException as exc:
                            stop_errors.append(
                                (host.host_id, "viewer-cleanup", exc)
                            )
                    if stop_errors:
                        details = "; ".join(
                            f"{host_id}/{phase}: {_exception_detail(error)[:512]}"
                            for host_id, phase, error in stop_errors
                        )
                        raise RuntimeError(
                            "runtime stop or Viewer ACL cleanup failed: "
                            + details
                        ) from stop_errors[0][2]
                if action == "start":
                    log("모든 호스트의 이미지를 먼저 준비합니다.")
                    for host in hosts:
                        log(f"build: {host.host_id}")
                        output = _BuildLogForwarder(host, log)
                        try:
                            operations[host.host_id].build(host, output)
                        finally:
                            output.flush()
                        log(f"build 완료: {host.host_id}")
                    launched = []
                    try:
                        log("활성 역할의 런타임을 시작합니다.")
                        for host in hosts:
                            log(f"start: {host.host_id}")
                            # A host launch can start one unit/container before a
                            # later unit fails. Record the attempt first so the
                            # compensating stop also covers that partial host.
                            launched.append(host)
                            if runtime_options is None:
                                operations[host.host_id].launch(host)
                            else:
                                operations[host.host_id].launch(host, runtime_options)
                        self._report_runtime_readiness(topology, operations, hosts, log)
                    except BaseException as exc:
                        if launched:
                            log(
                                "런타임 시작 또는 DDS readiness 확인 실패로 이번 "
                                "작업에서 시작한 "
                                "런타임을 롤백합니다."
                            )
                        rollback_errors = self._rollback_runtime_hosts(
                            operations,
                            launched,
                            cleanup_viewer=True,
                        )
                        if rollback_errors:
                            raise RuntimeRollbackError(exc, rollback_errors) from exc
                        raise
                return topology
            if topology.security_profile == "trusted-network":
                log("신뢰 네트워크 DDS 토폴로지 배포를 시작합니다.")
                TopologyRollout(topology, operations).apply(progress=progress)
                self._log_committed(
                    log,
                    "모든 호스트의 DDS 토폴로지 검증이 끝났습니다.",
                )
                return topology

            if authority is None:
                raise RuntimeError("SROS2 Authority was not prepared")

            generation = new_generation_id()
            journal = self._new_transaction_journal(action)
            journal["generation"] = generation
            self._write_transaction_journal(topology, journal)
            operation = (
                "새 보안 자료를 생성하고 검증"
                if action == "provision"
                else "기존 보안 세대를 새 세대로 재발급하고 검증"
            )
            log(
                f"{operation}합니다. SROS2 {generation} generation을 "
                "전체 호스트 사전 점검 후 원자적으로 적용합니다."
            )
            rollout = GenerationRollout(topology, operations)
            rollout.issue_and_apply(
                Sros2BundleIssuer(authority),
                generation,
                progress=progress,
            )
            self._log_committed(
                log,
                f"SROS2 {generation} generation이 활성화되었습니다.",
            )
        except RolloutError as exc:
            if journal is not None:
                journal.update(
                    {"status": "failed", "phase": exc.phase, "error": str(exc)[:1024]}
                )
                self._write_transaction_journal(topology, journal)
            # A cooperative cancellation raised at a progress boundary is
            # rolled back by the rollout transaction before it reaches here.
            if isinstance(exc.cause, ConnectionJobCancelled):
                raise exc.cause
            raise
        else:
            if journal is not None:
                journal.update({"status": "completed", "phase": "complete"})
                self._write_transaction_journal(topology, journal)
        finally:
            try:
                self._close_operations(operations)
            finally:
                self._release_scoped_transaction_lock(scoped_lock)
        return topology

    @staticmethod
    def _validate_scoped_identity(
        identity: Mapping[str, Any], unit: Any
    ) -> tuple[str, str]:
        """Validate the normalized identity returned by an enrolled unit.

        The concrete lifecycle performs the remote response validation, but
        operation adapters are still a trust boundary.  Keep the manager's
        per-unit planner strict about the exact fields and derive the project
        from the same canonical UUID before accepting a release registry.
        """

        if not isinstance(identity, Mapping) or set(identity) != {
            "install_uuid", "project"
        }:
            raise ValueError(
                f"scoped install identity fields are invalid on {unit.unit_id!r}"
            )
        install_uuid = identity.get("install_uuid")
        project = identity.get("project")
        if not isinstance(install_uuid, str) or not isinstance(project, str):
            raise ValueError(
                f"scoped install identity values are invalid on {unit.unit_id!r}"
            )
        try:
            expected_project = project_name(install_uuid)
        except ValueError as exc:
            raise ValueError(
                f"scoped install identity UUID is invalid on {unit.unit_id!r}"
            ) from exc
        if project != expected_project:
            raise ValueError(
                f"scoped install identity project is not derived from UUID on {unit.unit_id!r}"
            )
        if install_uuid != unit.install_uuid or project != unit.project:
            raise ValueError(
                f"remote scoped install identity mismatch on {unit.unit_id!r}"
            )
        return install_uuid, project

    @staticmethod
    def _release_from_payload(payload: Mapping[str, Any], install_uuid: str) -> ReleaseManifest:
        fields = {
            key: payload[key]
            for key in (
                "schema_version", "install_uuid", "source_revision", "platform",
                "role_images", "image_ids", "build_fingerprints", "runtime_data_digest",
            )
            if key in payload
        }
        if set(fields) != {
            "schema_version", "install_uuid", "source_revision", "platform",
            "role_images", "image_ids", "build_fingerprints", "runtime_data_digest",
        }:
            raise ValueError("remote release manifest is incomplete")
        release = ReleaseManifest(**fields).validate()
        if release.install_uuid != install_uuid:
            raise ValueError("remote release is bound to another installation")
        if payload.get("release_key") != release_key(release):
            raise ValueError("remote release key does not match its manifest")
        return release

    @staticmethod
    def _scoped_topology_digest(topology: ConnectionTopology) -> str:
        encoded = json.dumps(
            topology.to_dict(),
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        return hashlib.sha256(encoded).hexdigest()

    def _scoped_journal_root(self, topology: ConnectionTopology) -> Path:
        """Return the private transaction directory for exactly one system.

        The system identifier is part of the path, rather than being merely a
        field in a shared journal.  Check both directory components before
        using them so a replaced system directory cannot redirect a journal
        or its lock into another installation/system.
        """

        if self.authority_root.is_symlink():
            raise RuntimeError(
                f"connection authority root is not a private directory: {self.authority_root}"
            )
        if not self.authority_root.exists():
            self.authority_root.mkdir(parents=True, mode=0o700)
        if not self.authority_root.is_dir():
            raise RuntimeError(
                f"connection authority root is not a private directory: {self.authority_root}"
            )
        system_root = self.authority_root / topology.system_id
        root = system_root / "transactions"
        for path in (system_root, root):
            if path.is_symlink():
                raise RuntimeError(f"scoped transaction path is a symlink: {path}")
            if path.exists() and not path.is_dir():
                raise RuntimeError(
                    f"scoped transaction path is not a directory: {path}"
                )
        root.mkdir(parents=True, exist_ok=True, mode=0o700)
        for path in (system_root, root):
            if path.is_symlink() or not path.is_dir():
                raise RuntimeError(f"scoped transaction path is unsafe: {path}")
            path.chmod(0o700)
        return root

    def _acquire_scoped_transaction_lock(self, topology: ConnectionTopology) -> int:
        """Serialize all manager processes mutating one system journal."""

        lock_path = self._scoped_journal_root(topology) / ".lock"
        try:
            descriptor = os.open(
                lock_path,
                os.O_CREAT
                | os.O_RDWR
                | os.O_NOFOLLOW
                | os.O_CLOEXEC,
                0o600,
            )
        except OSError as exc:
            raise RuntimeError("scoped transaction lock cannot be opened") from exc
        try:
            info = os.fstat(descriptor)
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise RuntimeError("scoped transaction lock is not a private file")
            os.fchmod(descriptor, 0o600)
            fcntl.flock(descriptor, fcntl.LOCK_EX)
            return descriptor
        except BaseException:
            os.close(descriptor)
            raise

    @staticmethod
    def _release_scoped_transaction_lock(descriptor: int | None) -> None:
        if descriptor is None:
            return
        try:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        finally:
            os.close(descriptor)

    def _new_scoped_journal(
        self, action: str, topology: ConnectionTopology
    ) -> dict[str, object]:
        return {
            "schema_version": _SCOPED_JOURNAL_SCHEMA,
            "scope": _SCOPED_JOURNAL_SCOPE,
            "transaction_id": "",
            "system_id": topology.system_id,
            "action": action,
            "security_profile": topology.security_profile,
            "status": "running",
            "phase": "planned",
            "host_id": "",
            "topology_digest": self._scoped_topology_digest(topology),
            "created_at": datetime.now(timezone.utc).isoformat(),
            "updated_at": datetime.now(timezone.utc).isoformat(),
            "authority": {"before": None, "target": None, "observed": None},
            "units": [],
            "last_error": None,
            "rollback_errors": [],
        }

    def _load_scoped_journal(
        self, topology: ConnectionTopology
    ) -> dict[str, object] | None:
        path = self._scoped_journal_root(topology) / "latest.json"
        if not path.exists():
            return None
        if path.is_symlink() or not path.is_file():
            raise RuntimeError("scoped transaction journal is not a regular file")
        try:
            payload = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            raise RuntimeError("scoped transaction journal is invalid") from exc
        if not isinstance(payload, dict):
            raise RuntimeError("scoped transaction journal is not an object")
        self._validate_scoped_journal(payload, topology)
        return payload

    def _validate_scoped_journal(
        self, payload: Mapping[str, object], topology: ConnectionTopology
    ) -> None:
        """Validate the complete schema-v2 journal before trusting it.

        Recovery is a mutation boundary.  A partially shaped or cross-system
        journal must therefore be rejected, even when its status happens to
        look terminal and could otherwise bypass the unresolved-journal guard.
        """

        if set(payload) != _SCOPED_JOURNAL_TOP_LEVEL:
            raise RuntimeError("scoped transaction journal fields are invalid")
        if type(payload.get("schema_version")) is not int or payload.get("schema_version") != _SCOPED_JOURNAL_SCHEMA:
            raise RuntimeError("scoped transaction journal schema is unsupported")
        if payload.get("scope") != _SCOPED_JOURNAL_SCOPE:
            raise RuntimeError("scoped transaction journal scope is invalid")
        if payload.get("system_id") != topology.system_id:
            raise RuntimeError("scoped transaction journal system ID mismatch")
        digest = payload.get("topology_digest")
        if not isinstance(digest, str) or _SHA256_HEX_RE.fullmatch(digest) is None:
            raise RuntimeError("scoped transaction journal topology digest is invalid")
        if digest != ConnectionDeploymentRunner._scoped_topology_digest(topology):
            raise RuntimeError(
                "scoped transaction journal topology does not match the saved topology"
            )
        action = payload.get("action")
        if action not in {"deploy", "provision", "rotate"}:
            raise RuntimeError("scoped transaction journal action is invalid")
        status = payload.get("status")
        if status not in _SCOPED_JOURNAL_STATUSES:
            raise RuntimeError("scoped transaction journal status is invalid")
        phase = payload.get("phase")
        if phase not in _SCOPED_JOURNAL_PHASES:
            raise RuntimeError("scoped transaction journal phase is invalid")
        host_id = payload.get("host_id")
        if not isinstance(host_id, str) or len(host_id) > 192:
            raise RuntimeError("scoped transaction journal host ID is invalid")
        if status in _SCOPED_TERMINAL_STATUSES and phase != "complete":
            raise RuntimeError(
                "scoped terminal transaction journal must be in the complete phase"
            )
        if payload.get("security_profile") != topology.security_profile:
            raise RuntimeError("scoped transaction journal security profile mismatch")
        transaction_id = payload.get("transaction_id")
        if not isinstance(transaction_id, str) or _SAFE_SCOPED_TOKEN_RE.fullmatch(transaction_id) is None:
            raise RuntimeError("scoped transaction journal transaction ID is invalid")
        for field in ("created_at", "updated_at"):
            value = payload.get(field)
            if not isinstance(value, str) or not value.strip():
                raise RuntimeError(f"scoped transaction journal {field} is invalid")
            try:
                parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
            except ValueError as exc:
                raise RuntimeError(f"scoped transaction journal {field} is invalid") from exc
            if parsed.tzinfo is None:
                raise RuntimeError(f"scoped transaction journal {field} must be timezone-aware")

        authority = payload.get("authority")
        if not isinstance(authority, dict) or set(authority) != {"before", "target", "observed"}:
            raise RuntimeError("scoped transaction journal Authority state is invalid")
        for field in ("before", "target", "observed"):
            value = authority.get(field)
            if value is not None and (
                not isinstance(value, str) or _SAFE_SCOPED_TOKEN_RE.fullmatch(value) is None
            ):
                raise RuntimeError(
                    f"scoped transaction journal Authority {field} is invalid"
                )
        before_authority = authority["before"]
        target_authority = authority["target"]
        if topology.security_profile == "sros2":
            if not isinstance(target_authority, str):
                raise RuntimeError("scoped SROS2 journal has no target Authority generation")
            if action == "rotate" and not isinstance(before_authority, str):
                raise RuntimeError("scoped rotation journal has no prior Authority generation")
            if action != "rotate" and before_authority is not None:
                raise RuntimeError("scoped provision journal unexpectedly has a prior Authority generation")
        elif any(value is not None for value in authority.values()):
            raise RuntimeError("trusted-network scoped journal contains Authority state")

        last_error = payload.get("last_error")
        if last_error is not None and not isinstance(last_error, str):
            raise RuntimeError("scoped transaction journal last_error is invalid")
        rollback_errors = payload.get("rollback_errors")
        if not isinstance(rollback_errors, list):
            raise RuntimeError("scoped transaction journal rollback_errors is invalid")
        for entry in rollback_errors:
            if not isinstance(entry, dict) or set(entry) != {"target", "error"}:
                raise RuntimeError("scoped transaction journal rollback error is invalid")
            if not isinstance(entry["target"], str) or not isinstance(entry["error"], str):
                raise RuntimeError("scoped transaction journal rollback error is invalid")

        raw_units = payload.get("units")
        if not isinstance(raw_units, list) or not raw_units:
            raise RuntimeError("scoped transaction journal has no unit records")
        topology_units = {
            (host.host_id, unit.unit_id): (host, unit)
            for host in topology.hosts
            for unit in host.runtime_units
        }
        journal_host_id = payload.get("host_id")
        if journal_host_id and journal_host_id not in {
            f"{host.host_id}/{unit.unit_id}"
            for host, unit in topology_units.values()
        }:
            raise RuntimeError("scoped transaction journal host ID is invalid")
        local_identity: tuple[str, str] | None = None
        if self.local_install_root is not None:
            try:
                manifest = OwnershipManifest.load(
                    self.local_install_root / "install-ownership.json"
                )
                local_identity = (
                    manifest.install_uuid,
                    project_name(manifest.install_uuid),
                )
            except (OSError, ValueError) as exc:
                # A real scoped install always has an existing prefix and an
                # ownership manifest.  Keep the in-memory structural test
                # fixtures (which intentionally use a not-yet-created prefix)
                # readable, while still failing closed for an existing prefix
                # whose manifest is missing or malformed.
                if self.local_install_root.exists():
                    raise RuntimeError(
                        "scoped local ownership identity is unavailable"
                    ) from exc
        seen: set[tuple[str, str]] = set()
        unit_statuses = {"pending", "registering", "target", "before", "recovering", "failed"}
        for raw in raw_units:
            if not isinstance(raw, dict) or set(raw) != _SCOPED_JOURNAL_UNIT_FIELDS:
                raise RuntimeError("scoped transaction journal unit is invalid")
            host_id = raw.get("host_id")
            unit_id = raw.get("unit_id")
            if not isinstance(host_id, str) or not isinstance(unit_id, str):
                raise RuntimeError("scoped transaction journal unit identity is invalid")
            key = (host_id, unit_id)
            if key in seen or key not in topology_units:
                raise RuntimeError("scoped transaction journal unit does not match topology")
            seen.add(key)
            host, unit = topology_units[key]
            expected_identity = (unit.install_uuid, unit.project)
            if host.local and local_identity is not None:
                expected_identity = local_identity
            elif (
                not host.local
                and self.local_install_root is not None
                and self.local_install_root.exists()
                and not unit.install_uuid
            ):
                raise RuntimeError(
                    f"scoped transaction journal remote identity is missing on {host_id}/{unit_id}"
                )
            if (raw.get("install_uuid"), raw.get("project")) != expected_identity:
                raise RuntimeError(f"scoped transaction journal identity mismatch on {host_id}/{unit_id}")
            roles = raw.get("roles")
            if roles != list(unit.roles):
                raise RuntimeError(f"scoped transaction journal roles mismatch on {host_id}/{unit_id}")
            target_raw = raw.get("target")
            if not isinstance(target_raw, dict):
                raise RuntimeError(f"scoped transaction journal target is missing on {host_id}/{unit_id}")
            try:
                target = InstanceState.from_dict(target_raw)
            except (TypeError, ValueError) as exc:
                raise RuntimeError(f"scoped transaction journal target is invalid on {host_id}/{unit_id}") from exc
            before_raw = raw.get("before")
            if before_raw is None:
                before = None
            elif isinstance(before_raw, dict):
                try:
                    before = InstanceState.from_dict(before_raw)
                except (TypeError, ValueError) as exc:
                    raise RuntimeError(f"scoped transaction journal prior state is invalid on {host_id}/{unit_id}") from exc
            else:
                raise RuntimeError(f"scoped transaction journal prior state is invalid on {host_id}/{unit_id}")
            if target.system_id != topology.system_id or (before is not None and before.system_id != topology.system_id):
                raise RuntimeError(f"scoped transaction journal system mismatch on {host_id}/{unit_id}")
            target_release = raw.get("target_release_key")
            before_release = raw.get("before_release_key")
            if not isinstance(target_release, str) or _SHA256_HEX_RE.fullmatch(target_release) is None or target_release != target.release_key:
                raise RuntimeError(f"scoped transaction journal target release mismatch on {host_id}/{unit_id}")
            if before is None:
                if before_release is not None:
                    raise RuntimeError(f"scoped transaction journal prior release mismatch on {host_id}/{unit_id}")
            elif not isinstance(before_release, str) or before_release != before.release_key:
                raise RuntimeError(f"scoped transaction journal prior release mismatch on {host_id}/{unit_id}")
            bundle_digest = raw.get("bundle_manifest_sha256")
            if not isinstance(bundle_digest, str):
                raise RuntimeError(f"scoped transaction journal bundle digest is invalid on {host_id}/{unit_id}")
            if (
                topology.security_profile == "sros2"
                and _SHA256_HEX_RE.fullmatch(bundle_digest) is None
                and not (
                    not bundle_digest
                    and phase in {"planned", "issuing", "rollback"}
                )
            ):
                raise RuntimeError(f"scoped transaction journal bundle digest is invalid on {host_id}/{unit_id}")
            if topology.security_profile == "trusted-network" and bundle_digest:
                raise RuntimeError("trusted-network scoped journal contains a bundle digest")
            if topology.security_profile == "sros2":
                if target.security_generation != target_authority:
                    raise RuntimeError(f"scoped transaction journal target generation mismatch on {host_id}/{unit_id}")
                if before is not None and not before.security_generation:
                    raise RuntimeError(f"scoped transaction journal prior generation is missing on {host_id}/{unit_id}")
            elif target.security_generation:
                raise RuntimeError("trusted-network scoped journal contains a security generation")
            if topology.security_profile == "trusted-network" and before is not None and before.security_generation:
                raise RuntimeError("trusted-network scoped journal contains a prior security generation")
            if raw.get("status") not in unit_statuses:
                raise RuntimeError(f"scoped transaction journal unit status is invalid on {host_id}/{unit_id}")
            unit_error = raw.get("last_error")
            if unit_error is not None and not isinstance(unit_error, str):
                raise RuntimeError(f"scoped transaction journal unit error is invalid on {host_id}/{unit_id}")
        if seen != set(topology_units):
            raise RuntimeError("scoped transaction journal does not cover every container unit")

    def _refuse_unresolved_scoped_journal(
        self, topology: ConnectionTopology
    ) -> None:
        payload = self._load_scoped_journal(topology)
        if payload is None:
            return
        if payload.get("status") not in _SCOPED_TERMINAL_STATUSES:
            raise RuntimeError(
                "an unresolved scoped transaction journal exists; run scoped recover first"
            )

    @staticmethod
    def _authority_generation(
        authority: object | None,
        *,
        expected: str | None = None,
    ) -> str | None:
        if authority is None:
            return None
        active = authority.active()  # type: ignore[attr-defined]
        if active is None:
            if expected is not None and not hasattr(authority, "root"):
                # Structural fakes used by the offline setup tests do not
                # persist activation metadata.  The real Sros2Authority has
                # ``root`` and is always checked strictly below.
                return expected
            observed = None
        else:
            observed = getattr(active, "generation", None)
            if not isinstance(observed, str) or not observed:
                if not hasattr(authority, "root"):
                    return expected if expected is not None else "legacy-active"
                raise RuntimeError("SROS2 Authority activation metadata is invalid")
        if expected is not None and observed != expected:
            raise RuntimeError(
                f"SROS2 Authority generation readback mismatch: {observed!r} != {expected!r}"
            )
        return observed

    @staticmethod
    def _scoped_readback(
        operation: Any,
        host: ManagedHost,
        unit: DeploymentUnit,
        system_id: str,
        *,
        fallback: InstanceState | None,
    ) -> InstanceState | None:
        reader = getattr(operation, "scoped_instance_state", None)
        if not callable(reader):
            # Older structural fakes predate the readback surface.  Concrete
            # host operations always expose it; retaining this fallback keeps
            # those offline tests source-compatible.
            return fallback
        raw = reader(host, unit, system_id)
        if raw is None:
            # Concrete host operations return ``None`` only when no instance
            # exists.  Keep old structural fakes compatible for first
            # registration; recovery itself uses a strict reader and never
            # takes this fallback.
            return fallback
        if isinstance(raw, InstanceState):
            return raw
        return InstanceState.from_dict(raw)

    def _scoped_unit_plans(
        self,
        topology: ConnectionTopology,
        operations: Mapping[str, Any],
    ) -> list[
        tuple[
            ManagedHost,
            DeploymentUnit,
            InstanceState,
            ReleaseManifest,
            InstanceState | None,
            ReleaseManifest | None,
        ]
    ]:
        """Resolve identity, release, install policy and target state per unit."""

        plans: list[
            tuple[
                ManagedHost,
                DeploymentUnit,
                InstanceState,
                ReleaseManifest,
                InstanceState | None,
                ReleaseManifest | None,
            ]
        ] = []
        local_manifest = None
        local_state = None
        graph_role_ids: dict[str, str] = {}
        for managed_host in topology.hosts:
            for assignment in managed_host.assignments:
                if assignment.role not in {"pilot", "sim", "ui"}:
                    continue
                if assignment.role in graph_role_ids:
                    raise ValueError(
                        "동일 역할의 복수 인스턴스 실행은 아직 지원하지 않습니다: "
                        + assignment.role
                    )
                graph_role_ids[assignment.role] = assignment.endpoint_id
        default_role_ids = NetworkSettings()
        if self.local_install_root is not None:
            local_manifest = OwnershipManifest.load(self.local_install_root / "install-ownership.json")
            if local_manifest.docker is None:
                raise ValueError("local install ownership has no Docker identity")
            local_state = self._state_for_local_scope(local_manifest.install_uuid)
        for host in topology.hosts:
            for unit in host.runtime_units:
                operation = operations[host.host_id]
                if host.local:
                    if local_manifest is None or local_manifest.docker is None:
                        raise ValueError("local scoped deployment requires ownership evidence")
                    install_uuid = local_manifest.install_uuid
                    project = project_name(install_uuid)
                    if unit.install_uuid and unit.install_uuid != install_uuid:
                        raise ValueError(f"local unit {unit.unit_id!r} install UUID mismatch")
                    if unit.project and unit.project != project:
                        raise ValueError(f"local unit {unit.unit_id!r} project mismatch")
                    releases = list_releases(self.local_install_root, install_uuid=install_uuid)
                    policy = local_state.compute if local_state is not None else None
                else:
                    if not unit.install_uuid or not unit.project:
                        raise ValueError(
                            f"remote scoped unit {unit.unit_id!r} requires explicit enrollment"
                        )
                    install_uuid, _project = self._validate_scoped_identity(
                        operation.scoped_identity(host, unit), unit
                    )
                    raw_releases = operation.scoped_releases(host, unit)
                    releases = tuple(
                        self._release_from_payload(dict(value), install_uuid)
                        for value in raw_releases
                    )
                    raw_state = operation.scoped_install_state(host, unit)
                    compute_raw = raw_state.get("compute")
                    policy = None
                    if isinstance(compute_raw, Mapping):
                        from .state import ComputeSettings
                        policy = ComputeSettings(
                            gpu_mode=str(compute_raw.get("gpu_mode", "inherit")),
                            gpu_device=str(compute_raw.get("gpu_device", "")),
                        ).validate()
                requested_release = unit.release_key or self.instance_release_key
                selected = tuple(
                    value for value in releases
                    if requested_release is None or release_key(value) == requested_release
                )
                if len(selected) != 1:
                    raise ValueError(
                        f"unit {host.host_id}/{unit.unit_id} requires exactly one selected published release"
                    )
                release = selected[0]
                endpoints = tuple(InstanceEndpoint(a.role, a.endpoint_id) for a in unit.assignments)
                if not endpoints or not set(endpoint.role for endpoint in endpoints).issubset(release.role_images):
                    raise ValueError(f"release does not contain every role for {host.host_id}/{unit.unit_id}")
                instance = InstanceState(
                    system_id=topology.system_id,
                    release_key=release_key(release),
                    endpoints=endpoints,
                    domain_id=topology.dds_graph.domain_id,
                    pilot_id=graph_role_ids.get("pilot", default_role_ids.pilot_id),
                    sim_id=graph_role_ids.get("sim", default_role_ids.sim_id),
                    ui_id=graph_role_ids.get("ui", default_role_ids.ui_id),
                    rmw_implementation=topology.dds_graph.rmw_implementation,
                    discovery_mode=topology.dds_graph.discovery_mode,
                    static_peers=topology.discovery_peers(host.host_id),
                    interface=host.dds.interface,
                    security_profile=topology.security_profile,
                    turn=(self.instance_turn or TurnSettings()),
                    turn_urls=self.instance_turn_urls,
                    compute=policy if policy is not None else InstanceState.__dataclass_fields__["compute"].default_factory(),
                )
                previous = None
                if host.local and local_state is not None:
                    try:
                        previous = InstanceRegistry(local_state.prefix_path).load(topology.system_id)
                    except FileNotFoundError:
                        previous = None
                elif not host.local:
                    raw_previous = operation.scoped_instance_state(host, unit, topology.system_id)
                    if raw_previous is not None:
                        previous = InstanceState.from_dict(raw_previous)
                previous_release = None
                if previous is not None:
                    previous_release = next(
                        (
                            candidate
                            for candidate in releases
                            if release_key(candidate) == previous.release_key
                        ),
                        None,
                    )
                    if previous_release is None:
                        raise ValueError(
                            f"previous release for {host.host_id}/{unit.unit_id} "
                            "is no longer published"
                        )
                plans.append(
                    (
                        host,
                        unit,
                        instance,
                        release,
                        previous,
                        previous_release,
                    )
                )
        if not plans:
            raise ValueError("scoped topology has no container deployment units")
        return plans

    def _deploy_scoped_units(
        self,
        topology: ConnectionTopology,
        *,
        action: str,
        authority: Sros2Authority | None,
        operations: Mapping[str, Any],
        log: Log,
        journal: dict[str, object],
    ) -> ConnectionTopology:
        plans = self._scoped_unit_plans(topology, operations)
        if action == "rotate":
            missing = [
                f"{host.host_id}/{unit.unit_id}"
                for host, unit, _instance, _release, previous, _prior_release in plans
                if previous is None
            ]
            if missing:
                raise RuntimeError(
                    "scoped SROS2 rotation requires an existing instance on every unit: "
                    + ", ".join(missing)
                )
        # Replacing generated configuration or a security binding beneath a
        # running endpoint would make the live container disagree with the
        # registered state.  Check every replacement before issuing keys or
        # mutating the first host.
        checked_hosts: set[str] = set()
        for host, _unit, instance, _release, previous, _previous_release in plans:
            if previous is None or previous == instance or host.host_id in checked_hosts:
                continue
            status = operations[host.host_id].status(host)
            state = str(status.get("state", "unknown"))
            running = tuple(str(value) for value in status.get("running_roles", ()))
            if state not in {"stopped", "inactive"} or running:
                raise RuntimeError(
                    f"scoped instance {topology.system_id!r} is not stopped on "
                    f"{host.host_id}; refusing to replace live configuration"
                )
            checked_hosts.add(host.host_id)
        issued = None
        generation = ""
        bundles: Mapping[str, SecurityBundle] = {}
        authority_before = None
        if authority is not None:
            authority_before = self._authority_generation(authority)
        if action == "rotate" and authority_before is None:
            raise RuntimeError(
                "scoped SROS2 rotation requires an active prior Authority generation"
            )
        if authority is not None and action == "rotate":
            journal["authority"] = {
                "before": authority_before,
                "target": None,
                "observed": authority_before,
            }

        # Allocate the transaction generation before constructing the durable
        # intent.  The first journal write is the fence before either security
        # issuance or a host registration, so every target state in that
        # intent must already name the generation it will use.
        if topology.security_profile == "sros2":
            generation = new_generation_id()

        units = []
        for host, unit, instance, release, previous, previous_release in plans:
            if generation:
                instance = replace(instance, security_generation=generation)
            units.append(
                {
                    "host_id": host.host_id,
                    "unit_id": unit.unit_id,
                    # A local topology unit may be legacy/un-enrolled and
                    # carry empty identity fields.  The local ownership
                    # manifest resolved the actual release identity during
                    # planning; persist that identity in the recovery intent
                    # so local release readback is bound to the right install.
                    "install_uuid": release.install_uuid if host.local else unit.install_uuid,
                    "project": (
                        project_name(release.install_uuid)
                        if host.local
                        else unit.project
                    ),
                    "roles": list(unit.roles),
                    "before": None if previous is None else previous.to_dict(),
                    "target": instance.to_dict(),
                    "before_release_key": (
                        None if previous_release is None else release_key(previous_release)
                    ),
                    "target_release_key": release_key(release),
                    "bundle_manifest_sha256": "",
                    "status": "pending",
                    "last_error": None,
                }
            )
        journal["units"] = units
        if topology.security_profile == "sros2":
            if authority is None:
                raise RuntimeError("SROS2 Authority was not prepared")
            journal["transaction_id"] = generation
            authority_payload = journal["authority"]
            if not isinstance(authority_payload, dict):
                authority_payload = {
                    "before": authority_before,
                    "target": generation,
                    "observed": authority_before,
                }
                journal["authority"] = authority_payload
            authority_payload["target"] = generation
            journal["phase"] = "planned"
            self._write_transaction_journal(topology, journal)
            journal["phase"] = "issuing"
            self._write_transaction_journal(topology, journal)
            issued = Sros2BundleIssuer(authority).issue(topology, generation)
            bundles = issued.bundles
            for entry in units:
                host = topology.host(str(entry["host_id"]))
                unit = next(
                    value for value in host.runtime_units
                    if value.unit_id == entry["unit_id"]
                )
                bundle = bundles.get(host.host_id)
                if bundle is None:
                    raise RuntimeError(f"SROS2 issuer returned no bundle for {host.host_id}")
                manifest_bytes = getattr(bundle.for_roles(unit.roles), "manifest_bytes", None)
                entry["bundle_manifest_sha256"] = (
                    hashlib.sha256(manifest_bytes()).hexdigest()
                    if callable(manifest_bytes)
                    else ""
                )
            journal["phase"] = "issued"
            self._write_transaction_journal(topology, journal)
        else:
            journal["transaction_id"] = f"trusted-{datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')}"
            journal["phase"] = "planned"
            self._write_transaction_journal(topology, journal)
        registered: list[
            tuple[
                ManagedHost,
                DeploymentUnit,
                InstanceState,
                InstanceState | None,
                ReleaseManifest | None,
            ]
        ] = []
        try:
            for index, (host, unit, instance, release, previous, previous_release) in enumerate(plans):
                if generation:
                    instance = replace(instance, security_generation=generation)
                elif previous == instance:
                    log(f"unchanged: {host.host_id}/{unit.unit_id}")
                    continue
                bundle = bundles.get(host.host_id) if bundles else None
                if topology.security_profile == "sros2" and bundle is None:
                    raise RuntimeError(
                        f"SROS2 issuer returned no bundle for {host.host_id}"
                    )
                if bundle is not None:
                    bundle = bundle.for_roles(unit.roles)
                operation = operations[host.host_id]
                units[index]["target"] = instance.to_dict()
                units[index]["status"] = "registering"
                journal["phase"] = "register"
                journal["host_id"] = f"{host.host_id}/{unit.unit_id}"
                self._write_transaction_journal(topology, journal)
                register = getattr(operation, "register_scoped_instance", None)
                if not callable(register):
                    raise RuntimeError(f"scoped registration is unavailable on {host.host_id}/{unit.unit_id}")
                register(host, unit, instance, release, bundle, replace_existing=previous is not None)
                observed = self._scoped_readback(
                    operation,
                    host,
                    unit,
                    topology.system_id,
                    fallback=instance,
                )
                if observed != instance:
                    raise RuntimeError(
                        f"scoped registration readback mismatch on {host.host_id}/{unit.unit_id}"
                    )
                registered.append(
                    (host, unit, instance, previous, previous_release)
                )
                units[index]["status"] = "target"
                journal["host_id"] = f"{host.host_id}/{unit.unit_id}"
                self._write_transaction_journal(topology, journal)
                log(f"register: {host.host_id}/{unit.unit_id}")
            if issued is not None:
                journal["phase"] = "activate-authority"
                journal["host_id"] = ""
                self._write_transaction_journal(topology, journal)
                issued.activate_authority()
                observed_authority = self._authority_generation(
                    authority,
                    expected=generation,
                )
                journal["authority"]["observed"] = observed_authority  # type: ignore[index]
                journal["phase"] = "authority-active"
                self._write_transaction_journal(topology, journal)
            for index, (host, unit, instance, _release, _previous, _previous_release) in enumerate(plans):
                if generation:
                    instance = replace(instance, security_generation=generation)
                observed = self._scoped_readback(
                    operations[host.host_id],
                    host,
                    unit,
                    topology.system_id,
                    fallback=instance,
                )
                if observed != instance:
                    raise RuntimeError(
                        f"scoped registration final readback mismatch on {host.host_id}/{unit.unit_id}"
                    )
                units[index]["status"] = "target"
            journal["status"] = "completed"
            journal["phase"] = "complete"
            journal["host_id"] = ""
            self._write_transaction_journal(topology, journal)
            return topology
        except BaseException as exc:
            journal["phase"] = "rollback"
            journal["last_error"] = _exception_detail(exc)[:1024]
            failed_target = journal.get("host_id")
            if isinstance(failed_target, str) and failed_target:
                failed_host, separator, failed_unit = failed_target.partition("/")
                if separator:
                    for entry in units:
                        if (
                            entry.get("host_id") == failed_host
                            and entry.get("unit_id") == failed_unit
                        ):
                            entry["status"] = "failed"
                            entry["last_error"] = _exception_detail(exc)[:1024]
                            break
            self._write_transaction_journal(topology, journal)
            rollback_errors: list[tuple[str, BaseException]] = []
            if issued is not None:
                try:
                    issued.rollback_authority()
                    journal["authority"]["observed"] = self._authority_generation(authority)  # type: ignore[index]
                except BaseException as rollback_error:
                    rollback_errors.append(("operator-authority", rollback_error))
            for host, unit, instance, previous, previous_release in reversed(registered):
                try:
                    if previous is None:
                        operations[host.host_id].remove_scoped_instance(host, unit, topology.system_id)
                    else:
                        if previous_release is None:
                            raise RuntimeError(
                                "previous scoped release was not retained for rollback"
                            )
                        register = getattr(operations[host.host_id], "register_scoped_instance")
                        register(
                            host,
                            unit,
                            previous,
                            previous_release,
                            replace_existing=True,
                        )
                        observed = self._scoped_readback(
                            operations[host.host_id],
                            host,
                            unit,
                            topology.system_id,
                            fallback=previous,
                        )
                        if observed != previous:
                            raise RuntimeError(
                                f"scoped rollback readback mismatch on {host.host_id}/{unit.unit_id}"
                            )
                except BaseException as rollback_error:
                    rollback_errors.append(
                        (f"{host.host_id}/{unit.unit_id}", rollback_error)
                    )
            if rollback_errors:
                journal["status"] = "blocked"
                journal["rollback_errors"] = [
                    {
                        "target": target,
                        "error": _exception_detail(error)[:1024],
                    }
                    for target, error in rollback_errors
                ]
                self._write_transaction_journal(topology, journal)
                raise RuntimeRollbackError(
                    exc,
                    rollback_errors,
                    rollback_action="scoped registration restore",
                ) from exc
            if issued is None and generation:
                # The issuer may have published the Authority generation and
                # then failed before returning it.  Leave this transaction
                # recoverable instead of claiming a clean rollback.
                journal["status"] = "blocked"
                self._write_transaction_journal(topology, journal)
                raise
            journal["status"] = "rolled-back"
            journal["phase"] = "complete"
            self._write_transaction_journal(topology, journal)
            raise

    def _recover_scoped_transaction(
        self,
        topology: ConnectionTopology,
        operations: Mapping[str, Any],
        journal: dict[str, object],
        log: Log,
    ) -> None:
        """Recover one interrupted scoped registration from durable state."""

        self._validate_scoped_journal(journal, topology)
        action = str(journal.get("action", ""))
        if action not in {"deploy", "provision", "rotate"}:
            raise RuntimeError("scoped transaction journal action is invalid")
        raw_authority = journal.get("authority")
        if not isinstance(raw_authority, Mapping):
            raise RuntimeError("scoped transaction journal Authority state is missing")
        authority_before = raw_authority.get("before")
        authority_target = raw_authority.get("target")
        if topology.security_profile == "sros2":
            if action == "rotate" and not isinstance(authority_before, str):
                raise RuntimeError("scoped rotation journal has no prior Authority generation")
            if action != "rotate" and authority_before is not None:
                raise RuntimeError("scoped provision journal unexpectedly has a prior Authority generation")
            if not isinstance(authority_target, str) or not authority_target:
                raise RuntimeError("scoped SROS2 journal has no target Authority generation")
            authority: Sros2Authority | None = Sros2Authority(
                self.authority_root / topology.system_id
            )
            metadata = authority.generation_metadata(authority_target)
            if metadata.get("system_id") != topology.system_id:
                raise RuntimeError(
                    "scoped target Authority generation belongs to another system"
                )
            observed_authority = self._authority_generation(authority)
        else:
            if authority_before is not None or authority_target is not None:
                raise RuntimeError("trusted-network scoped journal contains Authority state")
            authority = None
            observed_authority = None
        raw_units = journal.get("units")
        if not isinstance(raw_units, list) or not raw_units:
            raise RuntimeError("scoped transaction journal has no unit records")
        topology_units = {
            (host.host_id, unit.unit_id): (host, unit)
            for host in topology.hosts
            for unit in host.runtime_units
        }
        if set(operations) != {host.host_id for host in topology.hosts}:
            raise RuntimeError("scoped recovery operations do not match topology hosts")
        local_identity: tuple[str, str] | None = None
        if self.local_install_root is not None:
            try:
                manifest = OwnershipManifest.load(
                    self.local_install_root / "install-ownership.json"
                )
                local_identity = (
                    manifest.install_uuid,
                    project_name(manifest.install_uuid),
                )
            except (OSError, ValueError) as exc:
                if self.local_install_root.exists():
                    raise RuntimeError(
                        "scoped local ownership identity is unavailable"
                    ) from exc
        records: list[tuple[dict[str, object], ManagedHost, DeploymentUnit, InstanceState | None, InstanceState]] = []
        seen: set[tuple[str, str]] = set()
        for raw in raw_units:
            if not isinstance(raw, dict):
                raise RuntimeError("scoped transaction journal unit is not an object")
            key = (str(raw.get("host_id", "")), str(raw.get("unit_id", "")))
            if key in seen or key not in topology_units:
                raise RuntimeError("scoped transaction journal unit does not match topology")
            seen.add(key)
            host, unit = topology_units[key]
            expected_identity = (unit.install_uuid, unit.project)
            if host.local and local_identity is not None:
                expected_identity = local_identity
            if (raw.get("install_uuid"), raw.get("project")) != expected_identity:
                raise RuntimeError(f"scoped transaction journal identity mismatch on {host.host_id}/{unit.unit_id}")
            if (
                not host.local
                and self.local_install_root is not None
                and self.local_install_root.exists()
                and not unit.install_uuid
            ):
                raise RuntimeError(
                    f"scoped transaction journal remote identity is missing on {host.host_id}/{unit.unit_id}"
                )
            if raw.get("roles") != list(unit.roles):
                raise RuntimeError(f"scoped transaction journal roles mismatch on {host.host_id}/{unit.unit_id}")
            target_raw = raw.get("target")
            if not isinstance(target_raw, Mapping):
                raise RuntimeError(f"scoped transaction journal target is missing on {host.host_id}/{unit.unit_id}")
            target = InstanceState.from_dict(target_raw)
            before_raw = raw.get("before")
            before = None if before_raw is None else InstanceState.from_dict(before_raw)
            if target.system_id != topology.system_id:
                raise RuntimeError(f"scoped transaction journal target system mismatch on {host.host_id}/{unit.unit_id}")
            if before is not None and before.system_id != topology.system_id:
                raise RuntimeError(f"scoped transaction journal prior system mismatch on {host.host_id}/{unit.unit_id}")
            if raw.get("target_release_key") != target.release_key:
                raise RuntimeError(f"scoped transaction journal target release mismatch on {host.host_id}/{unit.unit_id}")
            before_key = raw.get("before_release_key")
            if before is None:
                if before_key is not None:
                    raise RuntimeError(f"scoped transaction journal prior release mismatch on {host.host_id}/{unit.unit_id}")
            elif before_key != before.release_key:
                raise RuntimeError(f"scoped transaction journal prior release mismatch on {host.host_id}/{unit.unit_id}")
            if topology.security_profile == "sros2":
                if target.security_generation != authority_target:
                    raise RuntimeError(f"scoped transaction journal target generation mismatch on {host.host_id}/{unit.unit_id}")
                if before is not None and not before.security_generation:
                    raise RuntimeError(f"scoped transaction journal prior generation is missing on {host.host_id}/{unit.unit_id}")
            elif target.security_generation:
                raise RuntimeError("trusted-network scoped journal contains a security generation")
            records.append((raw, host, unit, before, target))
        if seen != set(topology_units):
            raise RuntimeError("scoped transaction journal does not cover every container unit")

        # Verify enrollment and release availability before choosing a
        # direction.  These reads are the recovery trust boundary.
        releases: dict[tuple[str, str, str], ReleaseManifest] = {}
        observed: dict[tuple[str, str], InstanceState | None] = {}
        for raw, host, unit, before, target in records:
            operation = operations[host.host_id]
            if unit.install_uuid:
                identity_reader = getattr(operation, "scoped_identity", None)
                if not callable(identity_reader):
                    raise RuntimeError(f"scoped identity readback is unavailable on {host.host_id}/{unit.unit_id}")
                self._validate_scoped_identity(identity_reader(host, unit), unit)
            release_reader = getattr(operation, "scoped_releases", None)
            if not callable(release_reader):
                raise RuntimeError(f"scoped release readback is unavailable on {host.host_id}/{unit.unit_id}")
            install_uuid = (
                local_identity[0]
                if host.local and local_identity is not None
                else unit.install_uuid
            )
            if (
                not install_uuid
                and self.local_install_root is not None
                and self.local_install_root.exists()
            ):
                raise RuntimeError(
                    f"scoped recovery install identity is missing on {host.host_id}/{unit.unit_id}"
                )
            for release_payload in release_reader(host, unit):
                release = (
                    release_payload
                    if isinstance(release_payload, ReleaseManifest)
                    else self._release_from_payload(dict(release_payload), install_uuid)
                )
                releases[(host.host_id, unit.unit_id, release_key(release))] = release
            for release_key_value in (raw.get("before_release_key"), raw.get("target_release_key")):
                if release_key_value is None:
                    continue
                key = (host.host_id, unit.unit_id, str(release_key_value))
                if key not in releases:
                    raise RuntimeError(f"scoped release is no longer published on {host.host_id}/{unit.unit_id}: {release_key_value}")
            reader = getattr(operation, "scoped_instance_state", None)
            if not callable(reader):
                raise RuntimeError(f"scoped instance readback is unavailable on {host.host_id}/{unit.unit_id}")
            raw_state = reader(host, unit, topology.system_id)
            current = None if raw_state is None else (
                raw_state if isinstance(raw_state, InstanceState) else InstanceState.from_dict(raw_state)
            )
            if current is not None and current.system_id != topology.system_id:
                raise RuntimeError(f"scoped instance readback system mismatch on {host.host_id}/{unit.unit_id}")
            if current != before and current != target:
                raise RuntimeError(f"scoped instance readback is neither prior nor target on {host.host_id}/{unit.unit_id}")
            observed[(host.host_id, unit.unit_id)] = current

        if topology.security_profile == "sros2":
            if observed_authority == authority_target:
                forward = True
            elif observed_authority == authority_before:
                forward = False
            else:
                raise RuntimeError(
                    f"scoped Authority state is neither prior nor target: {observed_authority!r}"
                )
        else:
            forward = all(
                observed[(host.host_id, unit.unit_id)] == target
                for _raw, host, unit, _before, target in records
            )

        desired = {
            (host.host_id, unit.unit_id): (target if forward else before)
            for _raw, host, unit, before, target in records
        }
        if topology.security_profile == "sros2":
            # Complete any digest omitted by an interruption during issue
            # before writing the recovery phase.  Thus even a crash between
            # the recovery intent and its first host mutation leaves a
            # self-contained, strictly valid journal.
            for raw, host, unit, _before, _target in records:
                if raw.get("bundle_manifest_sha256"):
                    continue
                source = (
                    authority.root
                    / "generations"
                    / str(authority_target)
                    / "bundles"
                    / host.host_id
                )
                recovered_bundle = SecurityBundle.from_directory(
                    system_id=topology.system_id,
                    host_id=host.host_id,
                    generation=str(authority_target),
                    root=source,
                ).for_roles(unit.roles)
                raw["bundle_manifest_sha256"] = hashlib.sha256(
                    recovered_bundle.manifest_bytes()
                ).hexdigest()
        for raw, host, unit, before, target in records:
            current = observed[(host.host_id, unit.unit_id)]
            wanted = desired[(host.host_id, unit.unit_id)]
            if current == wanted:
                continue
            status = operations[host.host_id].status(host)
            self._status_running_roles(host, status)
            state = str(status.get("state", "unknown"))
            running = tuple(str(value) for value in status.get("running_roles", ()))
            if state not in {"stopped", "inactive"} or running:
                raise RuntimeError(
                    f"scoped instance {topology.system_id!r} is not stopped on "
                    f"{host.host_id}; refusing recovery mutation"
                )

        journal["phase"] = "recover"
        journal["status"] = "running"
        journal["authority"]["observed"] = observed_authority  # type: ignore[index]
        self._write_transaction_journal(topology, journal)
        try:
            for index, (raw, host, unit, before, target) in enumerate(records):
                current = observed[(host.host_id, unit.unit_id)]
                wanted = desired[(host.host_id, unit.unit_id)]
                if current == wanted:
                    raw["status"] = "target" if wanted == target else "before"
                    continue
                operation = operations[host.host_id]
                raw["status"] = "recovering"
                journal["host_id"] = f"{host.host_id}/{unit.unit_id}"
                self._write_transaction_journal(topology, journal)
                if wanted is None:
                    remove = getattr(operation, "remove_scoped_instance", None)
                    if not callable(remove):
                        raise RuntimeError(f"scoped removal is unavailable on {host.host_id}/{unit.unit_id}")
                    remove(host, unit, topology.system_id)
                else:
                    release = releases[(host.host_id, unit.unit_id, wanted.release_key)]
                    bundle = None
                    if (
                        topology.security_profile == "sros2"
                        and wanted.security_generation == authority_target
                    ):
                        source = (
                            authority.root
                            / "generations"
                            / str(authority_target)
                            / "bundles"
                            / host.host_id
                        )
                        bundle = SecurityBundle.from_directory(
                            system_id=topology.system_id,
                            host_id=host.host_id,
                            generation=str(authority_target),
                            root=source,
                        ).for_roles(unit.roles)
                        expected_digest = str(raw.get("bundle_manifest_sha256", ""))
                        actual_digest = hashlib.sha256(bundle.manifest_bytes()).hexdigest()
                        if expected_digest and expected_digest != actual_digest:
                            raise RuntimeError(f"scoped bundle digest mismatch on {host.host_id}/{unit.unit_id}")
                        if not expected_digest:
                            # An interruption while the issuer was publishing
                            # can leave a valid generation but no digest
                            # readback. Bind the recovered intent to the
                            # published bundle before the first host write.
                            raw["bundle_manifest_sha256"] = actual_digest
                    register = getattr(operation, "register_scoped_instance", None)
                    if not callable(register):
                        raise RuntimeError(f"scoped registration is unavailable on {host.host_id}/{unit.unit_id}")
                    register(
                        host,
                        unit,
                        wanted,
                        release,
                        bundle,
                        replace_existing=current is not None,
                    )
                reader = getattr(operation, "scoped_instance_state")
                raw_state = reader(host, unit, topology.system_id)
                actual = None if raw_state is None else (
                    raw_state if isinstance(raw_state, InstanceState) else InstanceState.from_dict(raw_state)
                )
                if actual != wanted:
                    raise RuntimeError(f"scoped recovery readback mismatch on {host.host_id}/{unit.unit_id}")
                raw["status"] = "target" if wanted == target else "before"
                self._write_transaction_journal(topology, journal)
        except BaseException as exc:
            journal["status"] = "blocked"
            journal["last_error"] = _exception_detail(exc)[:1024]
            self._write_transaction_journal(topology, journal)
            raise

        if topology.security_profile == "sros2":
            try:
                final_authority = self._authority_generation(authority)
                if final_authority != (authority_target if forward else authority_before):
                    raise RuntimeError("scoped recovery Authority readback mismatch")
                journal["authority"]["observed"] = final_authority  # type: ignore[index]
            except BaseException as exc:
                # Host compensation may already have happened.  Preserve a
                # recoverable unresolved journal instead of reporting a clean
                # completion after an Authority readback failure.
                journal["status"] = "blocked"
                journal["last_error"] = _exception_detail(exc)[:1024]
                self._write_transaction_journal(topology, journal)
                raise
        journal["status"] = "completed"
        journal["phase"] = "complete"
        journal["host_id"] = ""
        self._write_transaction_journal(topology, journal)
        log("scoped transaction recovery completed")

    def _state_for_local_scope(self, install_uuid: str) -> InstallState:
        """Load the exact state paired with the manager's ownership manifest."""

        if self.local_install_root is None:
            raise ValueError("local install root is required")
        state_path = self.local_install_root / "install-state.json"
        state = InstallState.load(state_path)
        if state.prefix_path != self.local_install_root:
            raise ValueError("local install state prefix does not match scoped install")
        return state

    @staticmethod
    def _rollback_runtime_hosts(
        operations: Mapping[str, HostOperations],
        hosts: Sequence[ManagedHost],
        *,
        cleanup_viewer: bool = False,
    ) -> tuple[tuple[str, BaseException], ...]:
        errors: list[tuple[str, BaseException]] = []
        for host in reversed(hosts):
            try:
                operations[host.host_id].stop(host)
                if cleanup_viewer:
                    operations[host.host_id].cleanup_viewer(host)
            except BaseException as exc:
                errors.append((host.host_id, exc))
        return tuple(errors)

    @staticmethod
    def _status_running_roles(
        host: ManagedHost,
        status_payload: Mapping[str, Any],
    ) -> tuple[str, ...]:
        raw_roles = status_payload.get("running_roles", ())
        if not isinstance(raw_roles, (list, tuple)):
            raise RuntimeError(
                f"runtime status for {host.host_id} has invalid running_roles"
            )
        normalized = tuple(str(role) for role in raw_roles)
        allowed = set(host.roles)
        unexpected = sorted(set(normalized) - allowed)
        if unexpected:
            raise RuntimeError(
                f"runtime status for {host.host_id} contains unknown roles: "
                + ", ".join(unexpected)
            )
        roles = tuple(
            sorted(
                {
                    role
                    for role in normalized
                    if role in allowed
                }
            )
        )
        return roles

    @staticmethod
    def _report_runtime_readiness(
        topology: ConnectionTopology,
        operations: Mapping[str, HostOperations],
        hosts: Sequence[ManagedHost],
        log: Log,
    ) -> None:
        """Report application-level DDS liveness after detached launch.

        ``docker compose up -d`` only proves that containers were created.  A
        Sim process may still be building a Genesis scene, and a Docker
        Desktop/WSL namespace may be unable to receive a peer heartbeat.  The
        DDS endpoint thread starts before the Sim scene build, so a bounded
        strict probe waits for the exact descriptor/boot-heartbeat pair before
        accepting the graph.  This gate deliberately does not wait for the Sim
        scene/media session; UI continues its own session retry while Genesis
        builds.  A missing co-located or remote endpoint must fail the start
        instead of presenting a silently partitioned graph.
        The same read-only probe remains available through
        ``elesim-net doctor --strict-peers``.
        """

        log(
            "DDS endpoint 준비 상태를 확인합니다 (DDS endpoint liveness; "
            "컨테이너/Sim scene·media "
            "session과 별도, 최대 5분)."
        )
        failures: list[str] = []
        expected = tuple(
            sorted(
                assignment.endpoint_id
                for peer_host in topology.hosts
                for assignment in peer_host.assignments
            )
        )
        if not expected:
            for host in hosts:
                log(f"DDS readiness: {host.host_id} — 검사할 endpoint 없음")
            return

        def check_host(host: ManagedHost) -> object:
            checker = getattr(operations[host.host_id], "runtime_doctor", None)
            if not callable(checker):
                raise RuntimeError(
                    "검사기 없음; 컨테이너 로그에서 실제 상태를 확인하십시오"
                )
            return checker(host, expected, timeout_s=_DDS_READINESS_TIMEOUT_S)

        reports: dict[str, object] = {}
        with concurrent.futures.ThreadPoolExecutor(
            max_workers=max(1, len(hosts)),
            thread_name_prefix="elesim-dds-readiness",
        ) as executor:
            futures = {executor.submit(check_host, host): host for host in hosts}
            for future, host in futures.items():
                try:
                    reports[host.host_id] = future.result()
                except Exception as exc:
                    reports[host.host_id] = exc

        for host in hosts:
            report = reports[host.host_id]
            if isinstance(report, Exception):
                if isinstance(report, ConnectionJobCancelled):
                    raise report
                detail = _exception_detail(report)
                failures.append(
                    f"{host.host_id}: readiness probe error: {detail[:768]}"
                )
                log(
                    f"DDS readiness probe: {host.host_id} — "
                    f"DDS 판정 전에 검사 호출이 실패했습니다: {detail[:768]}"
                )
                continue
            if not isinstance(report, Mapping):
                detail = f"{host.host_id}: 검사 결과 형식이 올바르지 않음"
                failures.append(detail)
                log(
                    f"DDS readiness: {host.host_id} — "
                    "실패: 검사 결과 형식이 올바르지 않음; 컨테이너 로그를 "
                    "확인하십시오"
                )
                continue
            if ConnectionDeploymentRunner._runtime_report_ok(report):
                log(
                    f"DDS readiness: {host.host_id} — "
                    f"endpoint descriptor/heartbeat 확인: {', '.join(expected)}"
                )
                continue
            detail = ConnectionDeploymentRunner._runtime_report_detail(report)
            failures.append(f"{host.host_id}: {detail[:768]}")
            log(
                f"DDS readiness: {host.host_id} — "
                f"실패: {detail[:768]}; DDS descriptor/heartbeat 경로와 "
                "Docker Desktop/WSL 네트워크 namespace를 확인하십시오 "
                "(Sim scene/media session은 별도 게이트입니다)"
            )
        if failures:
            raise RuntimeError(
                "DDS readiness failed; expected co-located and remote peer "
                "heartbeats were not observed: "
                + "; ".join(failures)[:4096]
            )

    @staticmethod
    def _runtime_report_ok(report: Mapping[str, object]) -> bool:
        """Accept both one-unit and multi-unit doctor report envelopes."""

        if "ok" in report:
            return report.get("ok") is True
        units = report.get("units")
        if not isinstance(units, Mapping) or not units:
            return False
        return all(
            isinstance(unit_report, Mapping) and unit_report.get("ok") is True
            for unit_report in units.values()
        )

    @staticmethod
    def _runtime_report_detail(report: Mapping[str, object]) -> str:
        """Extract a bounded peer failure from either doctor report shape."""

        def from_one(value: object) -> str | None:
            if not isinstance(value, Mapping):
                return None
            raw_results = value.get("results", ())
            results = raw_results if isinstance(raw_results, (list, tuple)) else ()
            peer_result = next(
                (
                    result
                    for result in results
                    if isinstance(result, Mapping)
                    and result.get("name") == "DDS peers"
                ),
                None,
            )
            if isinstance(peer_result, Mapping):
                detail = str(peer_result.get("detail", "")).strip()
                if detail:
                    return detail
            if value.get("ok") is False:
                return "expected endpoint가 아직 발견되지 않음"
            return None

        direct = from_one(report)
        if direct:
            return direct
        units = report.get("units")
        if isinstance(units, Mapping):
            for unit_id, unit_report in units.items():
                detail = from_one(unit_report)
                if detail:
                    return f"{unit_id}: {detail}"
        return "expected endpoint가 아직 발견되지 않음"

    @staticmethod
    def _check_hosts(
        topology: ConnectionTopology,
        operations: Mapping[str, HostOperations],
        log: Log,
    ) -> None:
        """Run one read-only host check for endpoint and runtime state.

        The browser used to expose two checks with different scopes: an
        ephemeral two-host endpoint check and a saved-topology lifecycle
        status poll.  The operator-facing check now uses the saved topology
        and performs the same gates that a lifecycle start would perform,
        followed by the current Compose/systemd state.  It never changes
        files, security generations, or running roles.
        """

        log("모든 호스트의 연결과 런타임 상태를 점검합니다.")
        failures: list[str] = []
        for host in topology.hosts:
            log(f"check: {host.host_id}")
            try:
                operations[host.host_id].runtime_network_check(host)
                capabilities = operations[host.host_id].preflight(host)
                capabilities.require_for(host)
                status = dict(operations[host.host_id].status(host))
                state = str(status.get("state", "unknown"))
                running = status.get("running_roles", ())
                if isinstance(running, (list, tuple)):
                    role_text = ", ".join(str(role) for role in running) or "—"
                else:
                    role_text = ", ".join(host.roles) or "—"
                log(f"status: {host.host_id} = {state} [{role_text}]")
            except ConnectionJobCancelled:
                raise
            except Exception as exc:
                detail = str(exc).strip() or exc.__class__.__name__
                failures.append(f"{host.host_id}: {detail}")
                log(f"check failed: {host.host_id} — {detail}")
        if failures:
            raise RuntimeError("호스트 점검 실패: " + "; ".join(failures))

    def _recover_managed_security(
        self,
        topology: ConnectionTopology,
        operations: Mapping[str, HostOperations],
        log: Log,
    ) -> None:
        if topology.security_profile != "sros2":
            raise ValueError("복구는 managed SROS2 topology에서만 사용합니다")
        authority = Sros2Authority(self.authority_root / topology.system_id)
        active = authority.active()
        snapshots = {
            host.host_id: operations[host.host_id].capture_state(host)
            for host in topology.hosts
        }
        for host in topology.hosts:
            self._validate_recovery_snapshot(host, snapshots[host.host_id])
        stopped = []
        for host in topology.hosts:
            running = snapshots[host.host_id].running_roles
            if not running:
                continue
            log(f"recover-stop: {host.host_id}")
            operations[host.host_id].stop(host, running)
            stopped.append(host)
        if active is None:
            log("활성 Authority generation이 없어 managed-pending 상태로 복구합니다.")
            for host in topology.hosts:
                previous = snapshots[host.host_id]
                pending = copy.deepcopy(dict(previous.runtime_configuration))
                pending_security = {
                    "security_profile": "sros2",
                    "security_provisioning": "managed",
                    "security_generation": "",
                    "security_bundle": "",
                    "keystore": "",
                    "enclave": "",
                }
                dds = pending.get("dds")
                if not isinstance(dds, dict):
                    raise RuntimeError(f"DDS state is missing on {host.host_id!r}")
                dds.update(pending_security)
                unit_states = pending.get("units")
                if isinstance(unit_states, dict):
                    for unit_id, raw_unit in tuple(unit_states.items()):
                        if not isinstance(raw_unit, Mapping):
                            raise RuntimeError(
                                f"DDS state is missing on "
                                f"{host.host_id!r}/{unit_id!r}"
                            )
                        unit_copy = copy.deepcopy(dict(raw_unit))
                        unit_dds = unit_copy.get("dds")
                        if not isinstance(unit_dds, dict):
                            raise RuntimeError(
                                f"DDS state is missing on "
                                f"{host.host_id!r}/{unit_id!r}"
                            )
                        unit_dds.update(pending_security)
                        unit_states[unit_id] = unit_copy
                log(f"recover-pending: {host.host_id}")
                operations[host.host_id].rollback(
                    host,
                    HostActivationState(None, pending, previous.running_roles),
                )
        else:
            log(f"Authority generation {active.generation}으로 호스트를 일치시킵니다.")
            for host in topology.hosts:
                log(f"recover-active: {host.host_id}")
                operations[host.host_id].activate(host, active.generation)
        for host in stopped:
            operations[host.host_id].runtime_network_check(host)
            operations[host.host_id].start(
                host, snapshots[host.host_id].running_roles
            )
        for host in topology.hosts:
            operations[host.host_id].preflight(host).require_for(host)
            if active is None:
                operations[host.host_id].verify_topology(
                    host, snapshots[host.host_id].running_roles
                )
            else:
                operations[host.host_id].verify(
                    host,
                    active.generation,
                    snapshots[host.host_id].running_roles,
                )
        log("managed SROS2 상태 복구가 완료되었습니다.")

    @staticmethod
    def _validate_recovery_snapshot(
        host: ManagedHost, snapshot: HostActivationState
    ) -> None:
        state = snapshot.runtime_configuration
        unit_states = state.get("units")
        if isinstance(unit_states, Mapping):
            for unit in host.units:
                actual = unit_states.get(unit.unit_id)
                if not isinstance(actual, Mapping):
                    raise RuntimeError(
                        f"복구 대상 {host.host_id!r}/{unit.unit_id}의 설치 상태가 없습니다"
                    )
                boundaries = {
                    "roles": list(unit.roles),
                    "prefix": unit.install_root,
                    "bin_dir": unit.bin_dir,
                    "install_mode": unit.install_mode,
                }
                for name, value in boundaries.items():
                    observed = actual.get(name)
                    if name == "roles" and set(
                        str(item) for item in (observed or ())
                    ).issuperset(value):
                        continue
                    if name != "roles" and observed == value:
                        continue
                    raise RuntimeError(
                        f"복구 대상 {host.host_id!r}/{unit.unit_id}의 {name} 설치 경계가 "
                        f"topology와 다릅니다: {observed!r} != {value!r}"
                    )
            return
        boundaries = {
            "roles": list(host.roles),
            "prefix": host.install_root,
            "bin_dir": host.bin_dir,
            "install_mode": host.install_mode,
        }
        for name, value in boundaries.items():
            actual = state.get(name)
            if name == "roles":
                if set(str(item) for item in (actual or ())).issuperset(value):
                    continue
            elif actual == value:
                continue
            raise RuntimeError(
                f"복구 대상 {host.host_id!r}의 {name} 설치 경계가 "
                f"topology와 다릅니다: {actual!r} != {value!r}"
            )

    def runtime_status(self, topology: ConnectionTopology) -> dict[str, object]:
        """Collect host lifecycle state without using DDS discovery as a proxy."""

        topology.validate()
        self._validate_management_host(topology)
        operations = self._operations(topology)
        hosts: list[dict[str, object]] = []
        try:
            for host in topology.hosts:
                try:
                    value = dict(operations[host.host_id].status(host))
                    # Runtime status is keyed by the stable host ID; discard
                    # labels returned by an older remote helper.
                    value.pop("display_name", None)
                    value.setdefault("host_id", host.host_id)
                    value.setdefault("roles", list(host.roles))
                    value["reachable"] = True
                except Exception as exc:
                    hosts.append(
                        {
                            "host_id": host.host_id,
                            "roles": list(host.roles),
                            "reachable": False,
                            "state": "unreachable",
                            "detail": str(exc)[:512],
                        }
                    )
                    continue
                hosts.append(value)
        finally:
            self._close_operations(operations)
        return {
            "available": True,
            "security_profile": topology.security_profile,
            "hosts": hosts,
        }

    @staticmethod
    def _log_committed(log: Log, message: str) -> None:
        """Do not turn a completed transaction into a reported cancellation."""

        try:
            log(message)
        except ConnectionJobCancelled:
            # Cancellation is cooperative only before the rollout commit
            # boundary. At this point hosts (and, for SROS2, the Authority)
            # already agree on the new state, so the truthful result is
            # completed rather than cancelled/rolled back.
            return

    @staticmethod
    def _close_operations(operations: Mapping[str, HostOperations]) -> None:
        primary_error = sys.exc_info()[1]
        close_errors: list[tuple[str, BaseException]] = []
        for host_id, operation in operations.items():
            close = getattr(operation, "close", None)
            if close is not None:
                try:
                    close()
                except BaseException as exc:
                    close_errors.append((host_id, exc))
        if close_errors:
            error = OperationCloseError(primary_error, close_errors)
            if primary_error is not None:
                raise error from primary_error
            raise error

    @staticmethod
    def _new_transaction_journal(action: str) -> dict[str, object]:
        return {
            "schema_version": 1,
            "action": action,
            "status": "running",
            "phase": "prepare",
            "host_id": "",
            "started_at": datetime.now(timezone.utc).isoformat(),
        }

    def _write_transaction_journal(
        self,
        topology: ConnectionTopology,
        payload: Mapping[str, object],
    ) -> None:
        if payload.get("scope") == _SCOPED_JOURNAL_SCOPE:
            root = self._scoped_journal_root(topology)
        else:
            root = self.authority_root / topology.system_id / "transactions"
            root.mkdir(parents=True, exist_ok=True, mode=0o700)
            root.chmod(0o700)
        destination = root / "latest.json"
        descriptor, temporary_name = tempfile.mkstemp(
            dir=root, prefix=".latest.", suffix=".json"
        )
        temporary = Path(temporary_name)
        try:
            written = dict(payload)
            if written.get("scope") == _SCOPED_JOURNAL_SCOPE:
                written["updated_at"] = datetime.now(timezone.utc).isoformat()
            os.fchmod(descriptor, 0o600)
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                json.dump(written, handle, ensure_ascii=False, indent=2)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, destination)
            directory_fd = os.open(root, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        finally:
            if temporary.exists():
                temporary.unlink()

    def _validate_management_host(self, topology: ConnectionTopology) -> None:
        local = topology.local_host
        if not local.runtime_units:
            raise ValueError(
                "연결관리자는 Authority를 보관하는 운영 컴퓨터에서 실행해야 하며 "
                "local 호스트에는 연결관리자용 container unit이 필요합니다"
            )
        if self.local_install_root is not None:
            configured = Path(local.install_root).expanduser().resolve()
            if configured != self.local_install_root:
                raise ValueError(
                    "local 호스트 install_root가 이 연결관리자를 설치한 prefix와 "
                    f"다릅니다: {configured} != {self.local_install_root}"
                )
        if self.local_bin_dir is not None:
            configured_bin = Path(local.bin_dir).expanduser().resolve()
            if configured_bin != self.local_bin_dir:
                raise ValueError(
                    "local 호스트 bin_dir가 이 연결관리자를 설치한 명령 "
                    f"디렉터리와 다릅니다: {configured_bin} != {self.local_bin_dir}"
                )
        operator_home = operator_home_path()
        for host in topology.hosts:
            if (
                host.ssh is None
                or host.ssh.uses_agent
                or host.ssh.uses_tailscale_ssh
            ):
                continue
            identity = resolve_ssh_identity_path(host.ssh.identity_file)
            if identity.is_symlink() or not identity.is_file():
                raise ValueError(
                    f"{host.host_id} SSH identity가 일반 파일이 아닙니다: "
                    f"{identity}"
                )
            resolved = identity.resolve()
            if operator_home != resolved.parent and operator_home not in resolved.parents:
                raise ValueError(
                    f"{host.host_id} SSH identity는 연결관리자에 read-only로 "
                    "mount된 HOME 안에 있어야 합니다. 다른 위치의 키는 SSH agent에 "
                    "등록하십시오."
                )
            if stat.S_IMODE(resolved.stat().st_mode) & 0o077:
                raise ValueError(
                    f"{host.host_id} SSH identity 권한은 0600 이하이어야 합니다: "
                    f"{resolved}"
                )

    def _operations(
        self,
        topology: ConnectionTopology,
    ) -> Mapping[str, HostOperations]:
        lifecycle = InstalledElesimLifecycle(
            topology,
            scoped=self._local_install_scope(),
        )
        connector = ParamikoConnector()
        result: dict[str, HostOperations] = {}
        for host in topology.hosts:
            if host.local:
                result[host.host_id] = LocalHostOperations(lifecycle, topology)
            else:
                result[host.host_id] = SshHostOperations(
                    connector,
                    lifecycle,
                    topology,
                )
        return result

    def _local_install_scope(self) -> bool | None:
        """Return scope from the manager's local ownership evidence.

        A remote install UUID is not guessed from a path or Docker name.  The
        local manifest is the only install identity available to this
        manager; callers without one retain the legacy direct-call behavior
        used by older integrations and tests.
        """

        if self.local_install_root is None:
            return None
        manifest_path = self.local_install_root / "install-ownership.json"
        if not manifest_path.is_file():
            raise ValueError(
                "local install ownership manifest is unavailable; "
                "refusing to establish lifecycle scope"
            )
        manifest = OwnershipManifest.load(manifest_path)
        docker = manifest.docker
        if docker is None:
            raise ValueError("local install ownership has no Docker identity")
        if docker.project == "elesim-runtime":
            return False
        if docker.project == project_name(docker.install_uuid):
            return True
        raise ValueError(
            "local install ownership has an unsupported Docker scope; "
            "refusing lifecycle routing"
        )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="elesim-connections",
        description="EleSim DDS/SROS2 연결관리자 GUI",
    )
    parser.add_argument("--state", type=Path, required=True)
    parser.add_argument(
        "--expected-system-id",
        help="reject topology files belonging to another system workspace",
    )
    parser.add_argument("--authority-root", type=Path, required=True)
    parser.add_argument("--local-install-root", type=Path)
    parser.add_argument("--local-bin-dir", type=Path)
    parser.add_argument(
        "--instance-release",
        "--release",
        dest="instance_release_key",
        help="scoped first provisioning에서 사용할 이미 publish된 release key",
    )
    parser.add_argument(
        "--turn-mode", choices=("none", "managed", "external"), default="none",
        help="scoped first provisioning의 Sim TURN mode",
    )
    parser.add_argument("--turn-url", action="append", default=(), metavar="URL")
    parser.add_argument("--turn-realm", default="", metavar="REALM")
    parser.add_argument("--turn-public-host", default="", metavar="HOST")
    parser.add_argument("--turn-credential-file", default="", metavar="PATH")
    parser.add_argument(
        "--gpu-mode",
        choices=("inherit", "specific", "cpu"),
        default=os.environ.get("ELESIM_INSTALL_GPU_MODE", "cpu"),
    )
    parser.add_argument(
        "--gpu-device",
        default=os.environ.get("ELESIM_INSTALL_GPU_DEVICE", ""),
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument(
        "--token",
        default=os.environ.get("ELESIM_CONNECTION_TOKEN", ""),
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    if args.turn_mode == "managed":
        instance_turn = TurnSettings(
            mode="managed",
            realm=args.turn_realm,
            public_host=args.turn_public_host,
            # InstanceRuntime replaces this with the exact instance path.
            secret_file="pending",
        )
    elif args.turn_mode == "external":
        instance_turn = TurnSettings(
            mode="external", credential_file=args.turn_credential_file
        )
    else:
        instance_turn = None
    runner = ConnectionDeploymentRunner(
        args.authority_root,
        topology_state_path=args.state,
        local_install_root=args.local_install_root,
        local_bin_dir=args.local_bin_dir,
        instance_release_key=args.instance_release_key,
        instance_turn=instance_turn,
        instance_turn_urls=args.turn_url,
    )
    return run_connection_gui(
        state_path=args.state,
        runner=runner,
        expected_system_id=args.expected_system_id,
        status_provider=runner.runtime_status,
        host=args.host,
        port=args.port,
        token=args.token,
        local_install_root=args.local_install_root,
        local_bin_dir=args.local_bin_dir,
        authority_root=args.authority_root,
        gpu_mode=args.gpu_mode,
        gpu_device=args.gpu_device,
    )


__all__ = [
    "ConnectionDeploymentRunner",
    "OperationCloseError",
    "RuntimeRollbackError",
    "main",
]
