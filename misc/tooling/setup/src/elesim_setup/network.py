"""Configure installed host addresses and run layered connectivity checks."""

from __future__ import annotations

import argparse
import json
import sys
from dataclasses import replace
from pathlib import Path
from typing import Sequence

from .configuration import generate_role_configs
from .doctor import NetworkDoctor
from .state import (
    InstallState,
    NetworkSettings,
    SecuritySettings,
    TurnSettings,
    default_state_path,
)


def _prompt(label: str, current: str) -> str:
    value = input(f"{label} [{current}]: ").strip()
    return value or current


def _configure_interactive(state: InstallState) -> InstallState:
    print("\nElesim 기기 간 네트워크 설정")
    router_host = _prompt("Router hostname/IP", state.network.router_host)
    advertise_host = _prompt("이 기기의 advertise hostname/IP", state.network.advertise_host)
    router_port = int(_prompt("Router TCP port", str(state.network.router_port)))
    rgbd_port = int(_prompt("RGBD TCP port", str(state.network.rgbd_port)))
    turn_raw = _prompt("TURN URL (없으면 '-')", state.network.turn_urls[0] if state.network.turn_urls else "-")
    turn_urls = () if turn_raw == "-" else (turn_raw,)
    print("보안 모드: loopback / curve / insecure-lan")
    security_mode = _prompt("보안 모드", state.security.mode)
    credentials_root = state.security.credentials_root
    if security_mode == "curve":
        credentials_root = _prompt("credential root", credentials_root or str(state.prefix_path / "secrets"))
    else:
        credentials_root = ""
    turn = state.turn
    if not turn_urls:
        turn = TurnSettings()
    elif turn.mode == "none":
        turn = TurnSettings(mode="external")
    return replace(
        state,
        network=NetworkSettings(
            router_host=router_host,
            advertise_host=advertise_host,
            router_port=router_port,
            rgbd_port=rgbd_port,
            turn_urls=turn_urls,
            simulator_id=state.network.simulator_id,
            controller_id=state.network.controller_id,
        ),
        security=SecuritySettings(mode=security_mode, credentials_root=credentials_root),
        turn=turn,
    ).validate()


def _configure_from_args(state: InstallState, args: argparse.Namespace) -> InstallState:
    network = state.network
    security = state.security
    updated_network = replace(
        network,
        router_host=args.router_host or network.router_host,
        advertise_host=args.advertise_host or network.advertise_host,
        router_port=network.router_port if args.router_port is None else args.router_port,
        rgbd_port=network.rgbd_port if args.rgbd_port is None else args.rgbd_port,
        turn_urls=(
            ()
            if args.clear_turn
            else network.turn_urls if args.turn_url is None else tuple(args.turn_url)
        ),
        simulator_id=args.simulator_id or network.simulator_id,
        controller_id=args.controller_id or network.controller_id,
    )
    updated_security = replace(
        security,
        mode=args.security or security.mode,
        credentials_root=(
            args.credentials_root
            if args.credentials_root is not None
            else security.credentials_root
        ),
    )
    updated_turn = state.turn
    if not updated_network.turn_urls:
        updated_turn = TurnSettings()
    elif updated_turn.mode == "none":
        updated_turn = TurnSettings(mode="external")
    return replace(
        state,
        network=updated_network,
        security=updated_security,
        turn=updated_turn,
    ).validate()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", default=str(default_state_path()))
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("show", help="현재 IP/보안 설정 출력")
    configure = subparsers.add_parser("configure", help="IP와 보안 경로를 바꾸고 역할별 YAML 재생성")
    configure.add_argument("--router-host", default="")
    configure.add_argument("--advertise-host", default="")
    configure.add_argument("--router-port", type=int)
    configure.add_argument("--rgbd-port", type=int)
    configure.add_argument("--turn-url", action="append")
    configure.add_argument("--clear-turn", action="store_true")
    configure.add_argument("--simulator-id", default="")
    configure.add_argument("--controller-id", default="")
    configure.add_argument("--security", choices=("loopback", "curve", "insecure-lan"), default="")
    configure.add_argument("--credentials-root")
    configure.add_argument("--non-interactive", action="store_true")

    doctor = subparsers.add_parser("doctor", help="DNS/TCP/ZMQ/TURN/WebRTC 연결 검사")
    doctor.add_argument("--active", action="store_true", help="실제 RGBD와 WebRTC frame까지 검사")
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
            has_override = any(
                (
                    bool(args.router_host),
                    bool(args.advertise_host),
                    args.router_port is not None,
                    args.rgbd_port is not None,
                    args.turn_url is not None,
                    bool(args.clear_turn),
                    bool(args.simulator_id),
                    bool(args.controller_id),
                    bool(args.security),
                    args.credentials_root is not None,
                )
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
            if args.active:
                print("주의: --active WebRTC 검사는 짧은 simulation session을 독점합니다.", file=sys.stderr)
            report = NetworkDoctor(state, timeout_s=args.timeout, active=args.active).run()
            print(json.dumps(report.to_dict(), ensure_ascii=False, indent=2) if args.json else report.render())
            return 0 if report.ok else 1
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"오류: {exc}", file=sys.stderr)
        return 2
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
