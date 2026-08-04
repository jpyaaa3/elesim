"""Configure installed DDS settings and run layered connectivity checks."""

from __future__ import annotations

import argparse
import json
import os
import stat
import subprocess
import sys
import tempfile
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable, Sequence

from .configuration import (
    generate_role_configs,
    generated_app_config_path,
    generated_config_path,
    generated_dds_config_path,
)
from .doctor import NetworkDoctor
from .security_provisioning import (
    provisioning_required_path,
    sync_provisioning_required,
)
from .state import DdsSettings, InstallState, NetworkSettings, TurnSettings, default_state_path


@dataclass(frozen=True)
class TailscaleDetection:
    """Read-only local Tailscale interface hint for the connection manager.

    Detection never installs, logs in, changes ACLs, or invokes ``tailscale
    up``.  It is merely a convenience for filling the local DDS address and
    interface; the operator still saves and validates the topology explicitly.
    """

    available: bool
    interface: str = ""
    addresses: tuple[str, ...] = ()
    detail: str = ""

    def to_dict(self) -> dict[str, object]:
        return {
            "available": self.available,
            "interface": self.interface,
            "addresses": list(self.addresses),
            "detail": self.detail,
        }


def detect_tailscale(
    *,
    runner: Callable[..., object] | None = None,
) -> TailscaleDetection:
    """Return the current ``tailscale0`` IPv4 addresses without side effects.

    ``ip`` is used instead of parsing ``tailscale status`` so this works on a
    minimal host and does not require the local user to have Tailscale admin
    privileges.  A missing binary/interface is a normal, actionable result.
    """

    probe = subprocess.run if runner is None else runner
    try:
        result = probe(
            ["ip", "-j", "-4", "addr", "show", "dev", "tailscale0"],
            capture_output=True,
            text=True,
            check=False,
            timeout=1.5,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return TailscaleDetection(False, detail=f"ip probe unavailable: {exc}")
    if int(getattr(result, "returncode", 1)) != 0:
        return TailscaleDetection(
            False,
            detail="tailscale0 interface was not found; enter a routed-VPN or LAN address manually",
        )
    try:
        raw = json.loads(str(getattr(result, "stdout", "") or ""))
    except (TypeError, ValueError):
        return TailscaleDetection(False, detail="ip returned invalid JSON for tailscale0")
    addresses: list[str] = []
    if isinstance(raw, list):
        for link in raw:
            if not isinstance(link, dict):
                continue
            for item in link.get("addr_info", ()):
                if not isinstance(item, dict) or item.get("family") != "inet":
                    continue
                address = str(item.get("local", "")).strip()
                if address and address not in addresses:
                    addresses.append(address)
    if not addresses:
        return TailscaleDetection(
            False,
            interface="tailscale0",
            detail="tailscale0 exists but has no IPv4 address; use IPv6 or enter a current address manually",
        )
    return TailscaleDetection(
        True,
        interface="tailscale0",
        addresses=tuple(addresses),
        detail="read-only Tailscale address hint; no installation or login was performed",
    )


def _prompt(label: str, current: str) -> str:
    value = input(f"{label} [{current}]: ").strip()
    return value or current


def _configure_interactive(state: InstallState) -> InstallState:
    print("\nElesim ROS 2/DDS 설정")
    system_id = _prompt("Elesim system ID", state.dds.system_id)
    sim_id = _prompt("Sim endpoint ID", state.network.sim_id)
    pilot_id = _prompt("Pilot endpoint ID", state.network.pilot_id)
    ui_id = _prompt("UI endpoint ID", state.network.ui_id)
    robot_id = _prompt("Robot endpoint ID", state.network.robot_id)
    domain_id = int(_prompt("ROS_DOMAIN_ID", str(state.dds.domain_id)))
    discovery_mode = _prompt(
        "DDS discovery (multicast/static)",
        state.dds.discovery_mode,
    )
    peers_raw = _prompt(
        "Static peer hostname/IP (쉼표 구분, 없으면 '-')",
        ",".join(state.dds.static_peers) or "-",
    )
    static_peers = (
        ()
        if peers_raw == "-"
        else tuple(value.strip() for value in peers_raw.split(",") if value.strip())
    )
    interface = _prompt("DDS interface (자동이면 '-')", state.dds.interface or "-")
    interface = "" if interface == "-" else interface
    security_profile = _prompt(
        "DDS security profile (trusted-network/sros2)",
        state.dds.security_profile,
    )
    keystore = state.dds.keystore
    enclave = state.dds.enclave
    security_provisioning = state.dds.security_provisioning
    security_generation = state.dds.security_generation
    security_bundle = state.dds.security_bundle
    if security_profile == "sros2":
        if (
            state.dds.security_profile == "sros2"
            and state.dds.security_provisioning == "managed"
        ):
            print("SROS2 managed bundle은 elesim-connections에서 교체하십시오.")
        else:
            security_provisioning = "external"
            security_generation = ""
            security_bundle = ""
            keystore = _prompt(
                "SROS2 keystore",
                keystore or str(state.prefix_path / "sros2"),
            )
            enclave = _prompt("SROS2 base enclave", enclave or "/elesim")
    else:
        security_provisioning = "none"
        security_generation = ""
        security_bundle = ""
        keystore = ""
        enclave = ""
    turn_raw = _prompt(
        "TURN URL (없으면 '-')",
        state.network.turn_urls[0] if state.network.turn_urls else "-",
    )
    turn_urls = () if turn_raw == "-" else (turn_raw,)
    turn = state.turn
    if not turn_urls:
        turn = TurnSettings()
    elif turn.mode == "none":
        turn = TurnSettings(mode="external")
    if turn.mode == "external" and "sim" in state.roles:
        turn = replace(
            turn,
            credential_file=_prompt(
                "External TURN username/credential JSON file",
                turn.credential_file
                or str(state.prefix_path / "secrets/turn.credentials.json"),
            ),
        )
    return replace(
        state,
        network=replace(
            state.network,
            turn_urls=turn_urls,
            sim_id=sim_id,
            pilot_id=pilot_id,
            ui_id=ui_id,
            robot_id=robot_id,
        ),
        dds=DdsSettings(
            system_id=system_id,
            domain_id=domain_id,
            rmw_implementation=state.dds.rmw_implementation,
            discovery_mode=discovery_mode,
            static_peers=static_peers,
            interface=interface,
            security_profile=security_profile,
            security_provisioning=security_provisioning,
            security_generation=security_generation,
            security_bundle=security_bundle,
            keystore=keystore,
            enclave=enclave,
        ),
        turn=turn,
    ).require_runnable_dds()


def _configure_from_args(state: InstallState, args: argparse.Namespace) -> InstallState:
    peers = state.dds.static_peers
    if args.clear_dds_static_peers:
        peers = ()
    elif args.dds_static_peer is not None:
        peers = tuple(args.dds_static_peer)
    security_profile = args.dds_security_profile or state.dds.security_profile
    requested_provisioning = args.dds_security_provisioning
    security_provisioning = (
        requested_provisioning
        or state.dds.security_provisioning
        if state.dds.security_profile == "sros2"
        else requested_provisioning or "external"
    )
    security_generation = (
        args.dds_security_generation
        if args.dds_security_generation is not None
        else state.dds.security_generation
    )
    security_bundle = (
        args.dds_security_bundle
        if args.dds_security_bundle is not None
        else state.dds.security_bundle
    )
    if security_profile == "trusted-network":
        security_provisioning = "none"
        security_generation = ""
        security_bundle = ""
    elif security_provisioning == "external":
        security_generation = ""
        security_bundle = ""
    if (
        security_profile == "sros2"
        and state.dds.security_provisioning == "managed"
        and not requested_provisioning
        and (args.dds_keystore is not None or args.dds_enclave is not None)
    ):
        raise ValueError(
            "managed SROS2 bundle은 elesim-connections에서만 교체할 수 있습니다"
        )
    keystore = (
        state.dds.keystore if args.dds_keystore is None else args.dds_keystore
    )
    enclave = state.dds.enclave if args.dds_enclave is None else args.dds_enclave
    if security_provisioning == "managed" and not keystore and security_bundle:
        keystore = security_bundle
    dds = replace(
        state.dds,
        system_id=args.dds_system_id or state.dds.system_id,
        domain_id=(
            state.dds.domain_id if args.dds_domain_id is None else args.dds_domain_id
        ),
        rmw_implementation=(
            args.dds_rmw_implementation or state.dds.rmw_implementation
        ),
        discovery_mode=args.dds_discovery_mode or state.dds.discovery_mode,
        static_peers=peers,
        interface=(
            state.dds.interface
            if args.dds_interface is None
            else args.dds_interface
        ),
        security_profile=security_profile,
        security_provisioning=security_provisioning,
        security_generation=security_generation,
        security_bundle=security_bundle,
        keystore=keystore,
        enclave=enclave,
    )
    if dds.security_profile == "trusted-network":
        dds = replace(dds, keystore="", enclave="")

    turn_urls = (
        ()
        if args.clear_turn
        else state.network.turn_urls
        if args.turn_url is None
        else tuple(args.turn_url)
    )
    network = replace(
        state.network,
        turn_urls=turn_urls,
        sim_id=args.sim_id or state.network.sim_id,
        pilot_id=args.pilot_id or state.network.pilot_id,
        ui_id=args.ui_id or state.network.ui_id,
        robot_id=args.robot_id or state.network.robot_id,
    )
    if not turn_urls:
        turn = TurnSettings()
    else:
        mode = args.turn_mode or (
            state.turn.mode if state.turn.mode != "none" else "external"
        )
        turn = TurnSettings(
            mode=mode,
            realm=(
                args.turn_realm
                if args.turn_realm is not None
                else state.turn.realm if mode == "managed" else ""
            ),
            public_host=(
                args.turn_public_host
                if args.turn_public_host is not None
                else state.turn.public_host if mode == "managed" else ""
            ),
            secret_file=(
                args.turn_secret_file
                if args.turn_secret_file is not None
                else state.turn.secret_file if mode == "managed" else ""
            ),
            credential_file=(
                args.turn_credential_file
                if args.turn_credential_file is not None
                else state.turn.credential_file if mode == "external" else ""
            ),
        )
    return replace(state, network=network, dds=dds, turn=turn).require_runnable_dds()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", default=str(default_state_path()))
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("show", help="현재 DDS/TURN 설정 출력")
    configure = subparsers.add_parser(
        "configure",
        help="DDS/TURN 설정을 바꾸고 역할별 YAML/XML을 재생성",
    )
    configure.add_argument("--dds-system-id", default="")
    configure.add_argument("--dds-domain-id", type=int)
    configure.add_argument(
        "--dds-rmw-implementation",
        choices=("rmw_cyclonedds_cpp",),
        default="",
    )
    configure.add_argument(
        "--dds-discovery-mode",
        choices=("multicast", "static"),
        default="",
    )
    configure.add_argument("--dds-static-peer", action="append")
    configure.add_argument("--clear-dds-static-peers", action="store_true")
    configure.add_argument("--dds-interface")
    configure.add_argument(
        "--dds-security-profile",
        choices=("trusted-network", "sros2"),
        default="",
    )
    configure.add_argument(
        "--dds-security-provisioning",
        choices=("external", "managed"),
        default="",
        help=argparse.SUPPRESS,
    )
    configure.add_argument(
        "--dds-security-generation",
        help=argparse.SUPPRESS,
    )
    configure.add_argument(
        "--dds-security-bundle",
        help=argparse.SUPPRESS,
    )
    configure.add_argument("--dds-keystore")
    configure.add_argument("--dds-enclave")
    configure.add_argument("--turn-url", action="append")
    configure.add_argument("--clear-turn", action="store_true")
    configure.add_argument(
        "--turn-mode",
        choices=("none", "managed", "external"),
        default="",
    )
    configure.add_argument("--turn-realm")
    configure.add_argument("--turn-public-host")
    configure.add_argument("--turn-secret-file")
    configure.add_argument("--turn-credential-file")
    configure.add_argument("--sim-id", default="")
    configure.add_argument("--pilot-id", default="")
    configure.add_argument("--ui-id", default="")
    configure.add_argument("--robot-id", default="")
    configure.add_argument("--non-interactive", action="store_true")

    doctor = subparsers.add_parser(
        "doctor",
        help="DDS graph, RGBD topic, TURN과 WebRTC 연결 검사",
    )
    doctor.add_argument("--active", action="store_true", help="실제 DDS RGBD sample까지 검사")
    doctor.add_argument("--timeout", type=float, default=4.0)
    doctor.add_argument("--json", action="store_true", help="기계 판독용 JSON 출력")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    state_path = Path(args.state).expanduser().resolve()
    try:
        state = InstallState.load(state_path)
        if args.command == "show":
            print(json.dumps(state.to_dict(), ensure_ascii=False, indent=2))
            return 0
        if args.command == "configure":
            override_names = (
                "dds_system_id",
                "dds_domain_id",
                "dds_rmw_implementation",
                "dds_discovery_mode",
                "dds_static_peer",
                "clear_dds_static_peers",
                "dds_interface",
                "dds_security_profile",
                "dds_security_provisioning",
                "dds_security_generation",
                "dds_security_bundle",
                "dds_keystore",
                "dds_enclave",
                "turn_url",
                "clear_turn",
                "turn_mode",
                "turn_realm",
                "turn_public_host",
                "turn_secret_file",
                "turn_credential_file",
                "sim_id",
                "pilot_id",
                "ui_id",
                "robot_id",
            )
            has_override = any(
                getattr(args, name) not in (None, "", False, [])
                for name in override_names
            )
            updated = (
                _configure_from_args(state, args)
                if args.non_interactive or has_override
                else _configure_interactive(state)
            )
            written = _apply_configuration_transaction(state_path, updated)
            print("갱신된 설정:")
            for role, path in written.items():
                print(f"  {role}: {path}")
            print("실행 중인 프로세스는 새 설정을 읽도록 재시작해야 합니다.")
            return 0
        if args.command == "doctor":
            report = NetworkDoctor(
                state,
                timeout_s=args.timeout,
                active=args.active,
            ).run()
            print(
                json.dumps(report.to_dict(), ensure_ascii=False, indent=2)
                if args.json
                else report.render()
            )
            return 0 if report.ok else 1
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"오류: {exc}", file=sys.stderr)
        return 2
    return 2


def _apply_configuration_transaction(
    state_path: Path, updated: InstallState
) -> dict[str, Path]:
    """Regenerate runtime files and state as one locally recoverable update."""

    current = InstallState.load(state_path)
    if updated.dds.security_profile == "sros2" and (
        updated.dds.security_provisioning == "external"
    ):
        before = (
            current.dds.security_profile,
            current.dds.security_provisioning,
            current.dds.keystore,
            current.dds.enclave,
            current.network.pilot_id,
            current.network.ui_id,
            current.network.sim_id,
            current.network.robot_id,
        )
        after = (
            updated.dds.security_profile,
            updated.dds.security_provisioning,
            updated.dds.keystore,
            updated.dds.enclave,
            updated.network.pilot_id,
            updated.network.ui_id,
            updated.network.sim_id,
            updated.network.robot_id,
        )
        if before != after:
            raise ValueError(
                "external SROS2 keystore/enclave 변경은 역할별 key view를 "
                "안전하게 다시 만들기 위해 재설치가 필요합니다"
            )

    targets = {state_path}
    targets.add(provisioning_required_path(updated))
    for role in updated.roles:
        targets.add(generated_config_path(updated, role))
        targets.add(generated_dds_config_path(updated, role))
        if role == "sim":
            targets.add(generated_app_config_path(updated, role))
    snapshots = {path: _snapshot(path) for path in targets}
    try:
        written = generate_role_configs(updated)
        sync_provisioning_required(updated)
        updated.save(state_path)
        return written
    except BaseException:
        for path, snapshot in snapshots.items():
            _restore_snapshot(path, snapshot)
        raise


def _snapshot(path: Path) -> tuple[bytes, int] | None:
    if not path.exists():
        return None
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"설정 transaction 대상이 일반 파일이 아닙니다: {path}")
    return path.read_bytes(), stat.S_IMODE(path.stat().st_mode)


def _restore_snapshot(path: Path, snapshot: tuple[bytes, int] | None) -> None:
    if snapshot is None:
        if path.exists() and path.is_file() and not path.is_symlink():
            path.unlink()
        return
    payload, mode = snapshot
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent, prefix=f".{path.name}.rollback-"
    )
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, mode)
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


if __name__ == "__main__":
    raise SystemExit(main())
