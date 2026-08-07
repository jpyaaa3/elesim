"""CLI and concrete rollout runner for the browser connection manager."""

from __future__ import annotations

import argparse
import copy
import json
import os
import stat
import tempfile
import threading
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Mapping, Sequence

from .connection_gui import ConnectionJobCancelled, run_connection_gui
from .connection_manager import (
    ConnectionTopology,
    ManagedHost,
    operator_home_path,
    resolve_ssh_identity_path,
)
from .secure_deployment import (
    GenerationRollout,
    HostActivationState,
    HostOperations,
    InstalledElesimLifecycle,
    LocalHostOperations,
    ParamikoConnector,
    RolloutError,
    Sros2BundleIssuer,
    SshHostOperations,
    TopologyRollout,
)
from .security_authority import Sros2Authority, new_generation_id


Log = Callable[[str], None]


class _BuildLogForwarder:
    """Turn arbitrary stdout/stderr chunks into bounded host-labelled lines."""

    def __init__(self, host: ManagedHost, log: Log) -> None:
        self._prefix = f"build {host.display_name} ({host.host_id})"
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
        local_install_root: Path | None = None,
        local_bin_dir: Path | None = None,
    ) -> None:
        self.authority_root = authority_root.expanduser().resolve()
        self.local_install_root = (
            None
            if local_install_root is None
            else local_install_root.expanduser().resolve()
        )
        self.local_bin_dir = (
            None if local_bin_dir is None else local_bin_dir.expanduser().resolve()
        )

    def __call__(
        self,
        topology: ConnectionTopology,
        action: str,
        log: Log,
    ) -> None:
        topology.validate()
        self._validate_management_host(topology)
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
            log(f"{phase}: {host.display_name} ({host.host_id})")

        try:
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
                return
            if action in {"start", "stop", "restart", "check"}:
                if action == "check":
                    snapshot = self.runtime_status(topology)
                    for item in snapshot.get("hosts", ()):  # type: ignore[union-attr]
                        log(
                            f"{item.get('host_id', '?')}: "
                            f"{item.get('state', 'unknown')}"
                        )
                    return
                hosts = list(topology.hosts)
                if action in {"start", "restart"}:
                    log("모든 호스트의 런타임 네트워크를 사전 점검합니다.")
                    for host in hosts:
                        log(f"preflight: {host.display_name} ({host.host_id})")
                        # This is a cheap interface-visibility probe, not a
                        # DDS discovery or hardware test.  It is kept outside
                        # security generation preflight so a valid
                        # tailscale0 topology can be provisioned before the
                        # selected runtime backend is started.
                        operations[host.host_id].runtime_network_check(host)
                        capabilities = operations[host.host_id].preflight(host)
                        capabilities.require_for(host)
                if action in {"stop", "restart"}:
                    log("활성 역할의 런타임을 정지합니다.")
                    for host in reversed(hosts):
                        log(f"stop: {host.display_name} ({host.host_id})")
                        operations[host.host_id].stop(host)
                if action == "start":
                    log("모든 호스트의 이미지를 먼저 준비합니다.")
                    for host in hosts:
                        log(f"build: {host.display_name} ({host.host_id})")
                        output = _BuildLogForwarder(host, log)
                        try:
                            operations[host.host_id].build(host, output)
                        finally:
                            output.flush()
                        log(f"build 완료: {host.display_name} ({host.host_id})")
                    launched = []
                    try:
                        log("활성 역할의 런타임을 시작합니다.")
                        for host in hosts:
                            log(f"start: {host.display_name} ({host.host_id})")
                            operations[host.host_id].launch(host)
                            launched.append(host)
                    except BaseException:
                        for host in reversed(launched):
                            try:
                                operations[host.host_id].stop(host)
                            except BaseException:
                                pass
                        raise
                elif action == "restart":
                    log("활성 역할의 런타임을 시작합니다.")
                    for host in hosts:
                        log(f"start: {host.display_name} ({host.host_id})")
                        operations[host.host_id].start(host)
                return
            if topology.security_profile == "trusted-network":
                if action != "deploy":
                    raise ValueError(
                        "trusted-network에서는 deploy만 사용할 수 있습니다"
                    )
                log("신뢰 네트워크 DDS 토폴로지 배포를 시작합니다.")
                TopologyRollout(topology, operations).apply(progress=progress)
                self._log_committed(
                    log,
                    "모든 호스트의 DDS 토폴로지 검증이 끝났습니다.",
                )
                return

            authority = Sros2Authority(
                self.authority_root / topology.system_id
            )
            active = authority.active()
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

            generation = new_generation_id()
            journal = self._new_transaction_journal(action)
            journal["generation"] = generation
            self._write_transaction_journal(topology, journal)
            log(
                f"SROS2 {generation} generation을 전체 호스트 사전 점검 후 "
                "발급하고 원자적으로 적용합니다."
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
        try:
            for host in topology.hosts:
                running = snapshots[host.host_id].running_roles
                if not running:
                    continue
                log(f"recover-stop: {host.display_name} ({host.host_id})")
                operations[host.host_id].stop(host, running)
                stopped.append(host)
            if active is None:
                log("활성 Authority generation이 없어 managed-pending 상태로 복구합니다.")
                for host in topology.hosts:
                    previous = snapshots[host.host_id]
                    pending = copy.deepcopy(dict(previous.runtime_configuration))
                    dds = pending.get("dds")
                    if not isinstance(dds, dict):
                        raise RuntimeError(f"DDS state is missing on {host.host_id!r}")
                    dds.update(
                        {
                            "security_profile": "sros2",
                            "security_provisioning": "managed",
                            "security_generation": "",
                            "security_bundle": "",
                            "keystore": "",
                            "enclave": "",
                        }
                    )
                    log(f"recover-pending: {host.display_name} ({host.host_id})")
                    operations[host.host_id].rollback(
                        host,
                        HostActivationState(None, pending, previous.running_roles),
                    )
            else:
                log(f"Authority generation {active.generation}으로 호스트를 일치시킵니다.")
                for host in topology.hosts:
                    log(f"recover-active: {host.display_name} ({host.host_id})")
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
        except BaseException:
            # Recovery is itself resumable: the next invocation re-inspects
            # each host and converges on Authority-active or managed-pending.
            raise
        log("managed SROS2 상태 복구가 완료되었습니다.")

    @staticmethod
    def _validate_recovery_snapshot(
        host: ManagedHost, snapshot: HostActivationState
    ) -> None:
        state = snapshot.runtime_configuration
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
                    value.setdefault("host_id", host.host_id)
                    value.setdefault("display_name", host.display_name)
                    value.setdefault("roles", list(host.roles))
                    value["reachable"] = True
                except Exception as exc:
                    hosts.append(
                        {
                            "host_id": host.host_id,
                            "display_name": host.display_name,
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
        for operation in operations.values():
            close = getattr(operation, "close", None)
            if close is not None:
                close()

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
        if "robot" in local.roles:
            raise ValueError(
                "연결관리자는 Authority를 보관하는 운영 컴퓨터에서 실행해야 하며 "
                "Robot 호스트를 local로 지정할 수 없습니다"
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
                    f"{host.display_name} SSH identity가 일반 파일이 아닙니다: "
                    f"{identity}"
                )
            resolved = identity.resolve()
            if operator_home != resolved.parent and operator_home not in resolved.parents:
                raise ValueError(
                    f"{host.display_name} SSH identity는 연결관리자에 read-only로 "
                    "mount된 HOME 안에 있어야 합니다. 다른 위치의 키는 SSH agent에 "
                    "등록하십시오."
                )
            if stat.S_IMODE(resolved.stat().st_mode) & 0o077:
                raise ValueError(
                    f"{host.display_name} SSH identity 권한은 0600 이하이어야 합니다: "
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
        description="Elesim DDS/SROS2 연결관리자 GUI",
    )
    parser.add_argument("--state", type=Path, required=True)
    parser.add_argument("--authority-root", type=Path, required=True)
    parser.add_argument("--local-install-root", type=Path)
    parser.add_argument("--local-bin-dir", type=Path)
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
    )


__all__ = ["ConnectionDeploymentRunner", "main"]
