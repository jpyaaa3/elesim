"""CLI and concrete rollout runner for the browser connection manager."""

from __future__ import annotations

import argparse
import concurrent.futures
import copy
import json
import os
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
    ManagedHost,
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
    Sros2BundleIssuer,
    SshHostOperations,
    TopologyRollout,
)
from .security_authority import Sros2Authority, new_generation_id


Log = Callable[[str], None]
_DDS_READINESS_TIMEOUT_S = 60.0


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
        elif action == "recover" and topology.security_profile != "sros2":
            raise ValueError("복구는 managed SROS2 topology에서만 사용합니다")
        operations = self._operations(topology)
        journal: dict[str, object] | None = None

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
                "recover",
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
            self._close_operations(operations)
        return topology

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
        """Report application-level DDS readiness after detached launch.

        ``docker compose up -d`` only proves that containers were created.  A
        Sim process may still be building a Genesis scene, and a Docker
        Desktop/WSL namespace may be unable to receive a peer heartbeat.  The
        DDS endpoint thread starts before the Sim scene build, so a bounded
        strict probe is safe here: a missing co-located or remote endpoint must
        fail the start instead of presenting a silently partitioned graph.
        The same read-only probe remains available through
        ``elesim-net doctor --strict-peers``.
        """

        log("DDS endpoint 준비 상태를 확인합니다 (컨테이너 시작과 별도).")
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
                detail = ConnectionDeploymentRunner._exception_detail(report)
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
                f"실패: {detail[:768]}; Sim 장면 빌드 또는 Docker Desktop/WSL "
                "네트워크 namespace와 DDS UDP 경로를 확인하십시오"
            )
        if failures:
            raise RuntimeError(
                "DDS readiness failed; expected co-located and remote peer "
                "heartbeats were not observed: "
                + "; ".join(failures)[:4096]
            )

    @staticmethod
    def _exception_detail(error: BaseException) -> str:
        """Keep the exception type when its message is too terse to diagnose.

        Python's missing-context-manager error is commonly rendered only as
        ``__enter__``.  That text loses the distinction between a stale helper,
        a bad session object and a DDS probe failure, which made readiness
        rollback look like a network problem.  Preserve the bounded message,
        but always include the concrete exception class for terse errors.
        """

        return _exception_detail(error)

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
                    ) == set(value):
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
                if set(str(item) for item in (actual or ())) == set(value):
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
            "topology_mode": topology.topology_mode,
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
        root = self.authority_root / topology.system_id / "transactions"
        root.mkdir(parents=True, exist_ok=True, mode=0o700)
        root.chmod(0o700)
        destination = root / "latest.json"
        descriptor, temporary_name = tempfile.mkstemp(
            dir=root, prefix=".latest.", suffix=".json"
        )
        temporary = Path(temporary_name)
        try:
            os.fchmod(descriptor, 0o600)
            with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
                json.dump(dict(payload), handle, ensure_ascii=False, indent=2)
                handle.write("\n")
                handle.flush()
                os.fsync(handle.fileno())
            os.replace(temporary, destination)
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

    @staticmethod
    def _operations(
        topology: ConnectionTopology,
    ) -> Mapping[str, HostOperations]:
        lifecycle = InstalledElesimLifecycle(topology)
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


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="elesim-connections",
        description="EleSim DDS/SROS2 연결관리자 GUI",
    )
    parser.add_argument("--state", type=Path, required=True)
    parser.add_argument("--authority-root", type=Path, required=True)
    parser.add_argument("--local-install-root", type=Path)
    parser.add_argument("--local-bin-dir", type=Path)
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
    runner = ConnectionDeploymentRunner(
        args.authority_root,
        topology_state_path=args.state,
        local_install_root=args.local_install_root,
        local_bin_dir=args.local_bin_dir,
    )
    return run_connection_gui(
        state_path=args.state,
        runner=runner,
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
