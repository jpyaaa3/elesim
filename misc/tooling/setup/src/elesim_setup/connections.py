"""CLI and concrete rollout runner for the browser connection manager."""

from __future__ import annotations

import argparse
import os
import stat
from pathlib import Path
from typing import Callable, Mapping, Sequence

from .connection_gui import ConnectionJobCancelled, run_connection_gui
from .connection_manager import (
    ConnectionTopology,
    operator_home_path,
    resolve_ssh_identity_path,
)
from .secure_deployment import (
    GenerationRollout,
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

        def progress(phase: str, host_id: str | None) -> None:
            if host_id is None:
                log(phase)
                return
            host = topology.host(host_id)
            log(f"{phase}: {host.display_name} ({host.host_id})")

        try:
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
            if action == "provision" and active is not None:
                raise ValueError(
                    "이미 활성 SROS2 generation이 있습니다. rotate 또는 deploy를 "
                    "사용하십시오."
                )
            if action == "rotate" and active is None:
                raise ValueError(
                    "활성 SROS2 generation이 없습니다. 먼저 provision하십시오."
                )
            if action not in {"provision", "deploy", "rotate"}:
                raise ValueError(f"지원하지 않는 연결 작업: {action!r}")

            generation = new_generation_id()
            log(
                f"SROS2 {generation} generation을 발급하고 전체 호스트에 "
                "원자적으로 적용합니다."
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
            # A cooperative cancellation raised at a progress boundary is
            # rolled back by the rollout transaction before it reaches here.
            if isinstance(exc.cause, ConnectionJobCancelled):
                raise exc.cause
            raise

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
            if host.ssh is None or host.ssh.uses_agent:
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
        host=args.host,
        port=args.port,
        token=args.token,
        local_install_root=args.local_install_root,
        local_bin_dir=args.local_bin_dir,
    )


__all__ = ["ConnectionDeploymentRunner", "main"]
