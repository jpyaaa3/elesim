"""Configure installed DDS settings and run layered connectivity checks."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path
from typing import Sequence

from .configuration import generate_role_configs
from .doctor import NetworkDoctor
from .state import DdsSettings, InstallState, NetworkSettings, TurnSettings, default_state_path


def _prompt(label: str, current: str) -> str:
    value = input(f"{label} [{current}]: ").strip()
    return value or current


def _configure_interactive(state: InstallState) -> InstallState:
    print("\nElesim ROS 2/DDS 설정")
    system_id = _prompt("Elesim system ID", state.dds.system_id)
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
    if security_profile == "sros2":
        keystore = _prompt(
            "SROS2 keystore",
            keystore or str(state.prefix_path / "sros2"),
        )
        enclave = _prompt("SROS2 base enclave", enclave or "/elesim")
    else:
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
    if turn.mode == "external" and "simulator" in state.roles:
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
        network=replace(state.network, turn_urls=turn_urls),
        dds=DdsSettings(
            system_id=system_id,
            domain_id=domain_id,
            rmw_implementation=state.dds.rmw_implementation,
            discovery_mode=discovery_mode,
            static_peers=static_peers,
            interface=interface,
            security_profile=security_profile,
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
        security_profile=(
            args.dds_security_profile or state.dds.security_profile
        ),
        keystore=(
            state.dds.keystore if args.dds_keystore is None else args.dds_keystore
        ),
        enclave=(
            state.dds.enclave if args.dds_enclave is None else args.dds_enclave
        ),
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
        simulator_id=args.simulator_id or state.network.simulator_id,
        controller_id=args.controller_id or state.network.controller_id,
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
    configure.add_argument("--simulator-id", default="")
    configure.add_argument("--controller-id", default="")
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
                "dds_keystore",
                "dds_enclave",
                "turn_url",
                "clear_turn",
                "turn_mode",
                "turn_realm",
                "turn_public_host",
                "turn_secret_file",
                "turn_credential_file",
                "simulator_id",
                "controller_id",
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
            written = generate_role_configs(updated)
            updated.save(state_path)
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


if __name__ == "__main__":
    raise SystemExit(main())
