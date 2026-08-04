"""Interactive and non-interactive Elesim installer entry point."""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from pathlib import Path
from typing import Callable, Sequence

from .container_installer import ContainerInstaller
from .installer import Installer, preflight_notes
from .profiles import PROFILES, ROLE_ORDER, normalize_roles, roles_for_profile
from .state import (
    ComputeSettings,
    DEFAULT_BIN_DIR,
    DEFAULT_PREFIX,
    DdsSettings,
    InstallState,
    NetworkSettings,
    RuntimeTextLogSettings,
    TurnSettings,
    default_state_path,
)


Input = Callable[[str], str]


def _ask(prompt: str, default: str, *, input_fn: Input = input) -> str:
    suffix = f" [{default}]" if default else ""
    value = input_fn(f"{prompt}{suffix}: ").strip()
    return value or default


def _yes_no(prompt: str, default: bool = True, *, input_fn: Input = input) -> bool:
    marker = "Y/n" if default else "y/N"
    while True:
        value = input_fn(f"{prompt} [{marker}]: ").strip().lower()
        if not value:
            return default
        if value in {"y", "yes", "예", "ㅇ"}:
            return True
        if value in {"n", "no", "아니오", "ㄴ"}:
            return False
        print("y 또는 n을 입력하십시오.")


def _ask_roles(*, input_fn: Input = input) -> tuple[str, ...]:
    """Collect the general-install roles without exposing role presets.

    ``--profile`` remains a hidden compatibility input for old automation, but
    the interactive wizard has one source of truth: the roles selected here.
    The Robot constraint mirrors the web checkboxes so a mixed native/container
    request is rejected before any installation work starts.
    """

    print("\n설치할 프로그램을 필요한 만큼 선택하십시오 (쉼표로 구분).")
    print("  sim  Genesis 시뮬레이션과 RGBD/WebRTC 송신")
    print("  pilot 인식, IK, Pick/Gaze와 목표 생성")
    print("  ui         운영자 화면과 원격 조작")
    print("  robot      Jetson의 실제 장치와 로컬 안전 제어 (단독 설치)")
    while True:
        selected = _ask(
            "역할 (sim, pilot, ui, robot)",
            "sim,pilot,ui",
            input_fn=input_fn,
        )
        try:
            roles = normalize_roles(value.strip() for value in selected.split(","))
        except ValueError as exc:
            print(f"오류: {exc}")
            continue
        if "robot" in roles and roles != ("robot",):
            print("오류: robot은 다른 역할과 함께 설치할 수 없습니다.")
            continue
        return roles


def _ask_runtime_text_logs(*, input_fn: Input = input) -> RuntimeTextLogSettings:
    return RuntimeTextLogSettings(
        enabled=_yes_no(
            "실행 로그를 종료 시와 요청 시 평문 archive로 보관합니까?",
            default=True,
            input_fn=input_fn,
        )
    )


def _menu(
    title: str,
    choices: Sequence[tuple[str, str]],
    *,
    input_fn: Input = input,
) -> str:
    print(f"\n{title}")
    for index, (_value, label) in enumerate(choices, start=1):
        print(f"  {index}. {label}")
    while True:
        raw = input_fn("선택: ").strip()
        try:
            index = int(raw) - 1
        except ValueError:
            index = -1
        if 0 <= index < len(choices):
            return choices[index][0]
        print(f"1..{len(choices)} 중 하나를 입력하십시오.")


def run_wizard(
    *,
    source_root: Path,
    state_path: Path | None = None,
    input_fn: Input = input,
) -> int:
    print("\nElesim 설치 마법사")
    print("선택한 실행 역할과 ROS 2/DDS 구성을 격리된 환경에 설치합니다.")
    profile_name = "custom"
    roles = _ask_roles(input_fn=input_fn)

    install_mode = "native" if roles == ("robot",) else "container"
    print(
        "\n설치 방식: "
        + (
            "Robot Jetson native/systemd"
            if install_mode == "native"
            else "Docker Compose (호스트 환경 보존)"
        )
    )

    prefix = Path(
        _ask("설치 위치", str(DEFAULT_PREFIX), input_fn=input_fn)
    ).expanduser().resolve()
    bin_dir = Path(
        _ask("터미널 명령을 둘 위치", str(DEFAULT_BIN_DIR), input_fn=input_fn)
    ).expanduser().resolve()
    runtime_text_logs = _ask_runtime_text_logs(input_fn=input_fn)

    compute = ComputeSettings()
    if {"pilot", "sim"}.intersection(roles):
        gpu_mode = _menu(
            "GPU 사용 정책",
            (
                ("inherit", "외부 CUDA_VISIBLE_DEVICES를 그대로 따름 (권장)"),
                ("specific", "특정 GPU index 또는 UUID만 사용"),
                ("cpu", "GPU를 사용하지 않고 CPU로 실행"),
            ),
            input_fn=input_fn,
        )
        gpu_device = (
            _ask("GPU index 또는 UUID", "0", input_fn=input_fn)
            if gpu_mode == "specific"
            else ""
        )
        compute = ComputeSettings(gpu_mode=gpu_mode, gpu_device=gpu_device).validate()

    domain_id = int(_ask("ROS_DOMAIN_ID (모든 기기에서 동일)", "0", input_fn=input_fn))
    discovery_mode = _menu(
        "DDS discovery",
        (
            ("multicast", "같은 L2 네트워크에서 multicast 자동 발견"),
            ("static", "멀티캐스트가 막힌 네트워크의 static peer 목록"),
        ),
        input_fn=input_fn,
    )
    static_peers: tuple[str, ...] = ()
    if discovery_mode == "static":
        static_peers = tuple(
            value.strip()
            for value in _ask(
                "DDS peer hostname/IP (쉼표 구분)",
                "",
                input_fn=input_fn,
            ).split(",")
            if value.strip()
        )
    interface = _ask("DDS network interface (자동이면 비움)", "", input_fn=input_fn)
    security_profile = _menu(
        "DDS 보안 profile",
        (
            ("trusted-network", "격리된 신뢰 네트워크/VPN (DDS 보안 비활성)"),
            ("sros2", "SROS2 인증·암호화 강제"),
        ),
        input_fn=input_fn,
    )
    keystore = ""
    enclave = ""
    security_provisioning = "none"
    if security_profile == "sros2":
        security_provisioning = _menu(
            "SROS2 provisioning",
            (
                ("managed", "elesim-connections가 role bundle 생성·배포 (권장)"),
                ("external", "이미 존재하는 외부 keystore 사용"),
            ),
            input_fn=input_fn,
        )
        if security_provisioning == "external":
            keystore = str(
                Path(
                    _ask(
                        "SROS2 keystore 경로",
                        str(prefix / "sros2"),
                        input_fn=input_fn,
                    )
                ).expanduser().resolve()
            )
            enclave = _ask("SROS2 base enclave", "/elesim", input_fn=input_fn)
    dds = DdsSettings(
        domain_id=domain_id,
        discovery_mode=discovery_mode,
        static_peers=static_peers,
        interface=interface,
        security_profile=security_profile,
        security_provisioning=security_provisioning,
        keystore=keystore,
        enclave=enclave,
    ).validate()

    turn_urls: tuple[str, ...] = ()
    turn = TurnSettings()
    if _yes_no(
        "NAT를 넘는 WebRTC용 TURN relay를 사용합니까?",
        default=False,
        input_fn=input_fn,
    ):
        managed = (
            install_mode == "container"
            and "sim" in roles
            and _yes_no(
            "이 Sim 호스트가 Coturn lifecycle도 관리합니까?",
            default=False,
            input_fn=input_fn,
            )
        )
        public_host = (
            _ask("Coturn public hostname/IP", "", input_fn=input_fn)
            if managed
            else ""
        )
        turn_url = _ask(
            "TURN URL",
            f"turn:{public_host}:3478?transport=udp" if public_host else "",
            input_fn=input_fn,
        )
        turn_urls = (turn_url,)
        turn = (
            TurnSettings(
                mode="managed",
                realm=_ask("TURN realm", "elesim.local", input_fn=input_fn),
                public_host=public_host,
                secret_file=str(prefix / "secrets/turn.secret"),
            )
            if managed
            else TurnSettings(
                mode="external",
                credential_file=(
                    _ask(
                        "External TURN username/credential JSON file",
                        "",
                        input_fn=input_fn,
                    )
                    if "sim" in roles
                    else ""
                ),
            )
        )

    state = InstallState(
        profile=profile_name,
        roles=roles,
        prefix=str(prefix),
        bin_dir=str(bin_dir),
        source_root=str(source_root),
        network=NetworkSettings(turn_urls=turn_urls),
        dds=dds,
        compute=compute,
        turn=turn,
        runtime_text_logs=runtime_text_logs,
        install_mode=install_mode,
    ).require_installable_dds()

    print("\n사전 확인")
    for note in preflight_notes(roles, install_mode=install_mode):
        print(f"  - {note}")
    if security_profile == "trusted-network":
        print("  - 경고: DDS 인증·암호화는 비활성입니다. 격리된 사설망/VPN에서만 사용하십시오.")
    if not _yes_no("이 설정으로 설치를 시작합니까?", input_fn=input_fn):
        print("설치를 취소했습니다.")
        return 1

    installer_type = ContainerInstaller if install_mode == "container" else Installer
    installer_type(state, state_path=state_path).run()
    _path_note(bin_dir)
    return 0


def _path_note(bin_dir: Path) -> None:
    paths = {
        Path(value).expanduser().resolve()
        for value in os.environ.get("PATH", "").split(":")
        if value
    }
    if bin_dir not in paths:
        print(f"\n{bin_dir}가 PATH에 없습니다. shell 설정에 다음을 한 번 추가하십시오:")
        print(f'  export PATH="{bin_dir}:$PATH"')


def _source_root(explicit: str, state_path: Path) -> Path:
    if explicit:
        return Path(explicit).expanduser().resolve()
    if state_path.is_file():
        return InstallState.load(state_path).source_path
    candidate = Path.cwd().resolve()
    if (
        (candidate / "packages/protocol/pyproject.toml").is_file()
        and (candidate / "packages/elesim_interfaces/package.xml").is_file()
    ):
        return candidate
    raise FileNotFoundError("--source-root를 지정하거나 Elesim 저장소 루트에서 실행하십시오")


def _build_state(args: argparse.Namespace, source_root: Path) -> InstallState:
    if args.role:
        roles = normalize_roles(args.role)
        profile_name = "custom"
    else:
        # ``--profile`` is retained as a hidden compatibility path for old
        # scripts. New callers should use one or more ``--role`` arguments.
        roles = roles_for_profile(args.profile, ())
        profile_name = args.profile
    install_mode = (
        ("native" if roles == ("robot",) else "container")
        if args.mode == "auto"
        else args.mode
    )
    peers = tuple(args.dds_static_peer or ())
    turn_urls = tuple(args.turn_url or ())
    turn_mode = (
        args.turn_mode
        if args.turn_mode != "auto"
        else "external" if turn_urls else "none"
    )
    secret_file = args.turn_secret_file
    if turn_mode == "managed" and not secret_file:
        secret_file = str(Path(args.prefix).expanduser().resolve() / "secrets/turn.secret")
    return InstallState(
        profile=profile_name,
        roles=roles,
        prefix=str(Path(args.prefix).expanduser().resolve()),
        bin_dir=str(Path(args.bin_dir).expanduser().resolve()),
        source_root=str(source_root),
        network=NetworkSettings(
            turn_urls=turn_urls,
            sim_id=args.sim_id,
            pilot_id=args.pilot_id,
            ui_id=args.ui_id,
            robot_id=args.robot_id,
        ),
        dds=DdsSettings(
            system_id=args.dds_system_id,
            domain_id=args.dds_domain_id,
            rmw_implementation=args.dds_rmw_implementation,
            discovery_mode=args.dds_discovery_mode,
            static_peers=peers,
            interface=args.dds_interface,
            security_profile=args.dds_security_profile,
            security_provisioning=(
                args.dds_security_provisioning
                if args.dds_security_profile == "sros2"
                else "none"
            ),
            keystore=args.dds_keystore,
            enclave=args.dds_enclave,
        ),
        compute=ComputeSettings(
            gpu_mode=args.gpu_mode,
            gpu_device=args.gpu_device,
        ),
        turn=TurnSettings(
            mode=turn_mode,
            realm=args.turn_realm,
            public_host=args.turn_public_host,
            secret_file=secret_file,
            credential_file=args.turn_credential_file,
        ),
        runtime_text_logs=RuntimeTextLogSettings(
            enabled=args.runtime_text_logs,
        ),
        install_mode=install_mode,
        install_go2_mpc=not args.skip_go2_mpc,
    ).require_installable_dds()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", default="", help="Elesim source archive/checkout root")
    parser.add_argument("--state", default=str(default_state_path()), help="install-state.json path")
    subparsers = parser.add_subparsers(dest="command")

    subparsers.add_parser("wizard", help="대화형 설치 마법사")
    gui = subparsers.add_parser("gui", help="로컬 브라우저 설치 마법사")
    gui.add_argument("--host", default=os.environ.get("ELESIM_GUI_HOST", "127.0.0.1"))
    gui.add_argument("--port", type=int, default=int(os.environ.get("ELESIM_GUI_PORT", "8765")))
    gui.add_argument("--token", default=os.environ.get("ELESIM_GUI_TOKEN", ""))
    gui.add_argument("--no-open", action="store_true", help=argparse.SUPPRESS)
    gui.add_argument(
        "--invocation-dir",
        default=os.environ.get("ELESIM_INVOCATION_DIR", str(Path.cwd())),
    )
    gui.add_argument(
        "--repository",
        default=os.environ.get("ELESIM_REPOSITORY", "jpyaaa3/elesim"),
    )
    gui.add_argument("--ref", default=os.environ.get("ELESIM_REF", "main"))

    install = subparsers.add_parser("install", help="자동화용 비대화형 설치")
    install.add_argument(
        "--profile",
        choices=tuple(PROFILES),
        default="local-sim",
        help=argparse.SUPPRESS,
    )
    install.add_argument(
        "--mode",
        choices=("auto", "native", "container"),
        default="auto",
        help="auto는 Robot 단독만 native, 나머지는 Docker Compose로 설치",
    )
    install.add_argument("--role", action="append", choices=ROLE_ORDER)
    install.add_argument("--prefix", default=str(DEFAULT_PREFIX))
    install.add_argument("--bin-dir", default=str(DEFAULT_BIN_DIR))
    install.add_argument("--sim-id", default="sim-default")
    install.add_argument("--pilot-id", default="pilot-main")
    install.add_argument("--ui-id", default="ui-main")
    install.add_argument("--robot-id", default="robot-go2")
    install.add_argument("--dds-system-id", default="elesim")
    install.add_argument("--dds-domain-id", type=int, default=0)
    install.add_argument(
        "--dds-rmw-implementation",
        choices=("rmw_cyclonedds_cpp",),
        default="rmw_cyclonedds_cpp",
    )
    install.add_argument(
        "--dds-discovery-mode",
        choices=("multicast", "static"),
        default="multicast",
    )
    install.add_argument("--dds-static-peer", action="append", default=[])
    install.add_argument("--dds-interface", default="")
    install.add_argument(
        "--dds-security-profile",
        choices=("trusted-network", "sros2"),
        default="trusted-network",
    )
    install.add_argument(
        "--dds-security-provisioning",
        choices=("external", "managed"),
        default="external",
        help="SROS2 key owner; managed starts pending until elesim-connections deploys",
    )
    install.add_argument("--dds-keystore", default="")
    install.add_argument("--dds-enclave", default="")
    install.add_argument(
        "--gpu-mode",
        choices=("inherit", "specific", "cpu"),
        default="inherit",
    )
    install.add_argument("--gpu-device", default="")
    install.add_argument("--turn-url", action="append", default=[])
    install.add_argument(
        "--turn-mode",
        choices=("auto", "none", "managed", "external"),
        default="auto",
    )
    install.add_argument("--turn-realm", default="")
    install.add_argument("--turn-public-host", default="")
    install.add_argument("--turn-secret-file", default="")
    install.add_argument("--turn-credential-file", default="")
    install.add_argument("--skip-go2-mpc", action="store_true")
    install.add_argument(
        "--runtime-text-logs",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="종료 시와 elesim-logs --save에서 로컬 평문 로그 archive 저장",
    )
    install.add_argument("--dry-run", action="store_true")

    subparsers.add_parser("status", help="현재 설치 상태 출력")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    state_path = Path(args.state).expanduser().resolve()
    try:
        if args.command == "status":
            state = InstallState.load(state_path)
            print(json.dumps(state.to_dict(), ensure_ascii=False, indent=2))
            return 0
        source_root = _source_root(args.source_root, state_path)
        if args.command == "gui":
            from .capabilities import detect_install_host_capabilities
            from .gui import run_gui
            from .service import SetupService

            capabilities = detect_install_host_capabilities()
            return run_gui(
                source_root=source_root,
                invocation_dir=Path(args.invocation_dir),
                repository=args.repository,
                ref=args.ref,
                runner=lambda request, log: SetupService(
                    capabilities,
                    log=log,
                ).run(request),
                host=args.host,
                port=args.port,
                token=args.token,
                capabilities=capabilities,
            )
        if args.command in {None, "wizard"}:
            return run_wizard(source_root=source_root, state_path=state_path)
        if args.command == "install":
            state = _build_state(args, source_root)
            installer_type = (
                ContainerInstaller
                if state.install_mode == "container"
                else Installer
            )
            installer_type(
                state,
                state_path=state_path,
                dry_run=bool(args.dry_run),
            ).run()
            if not args.dry_run:
                _path_note(state.bin_path)
            return 0
    except KeyboardInterrupt:
        print("\n설치를 중단했습니다.", file=sys.stderr)
        return 130
    except EOFError:
        print("오류: 대화형 입력 terminal을 사용할 수 없습니다.", file=sys.stderr)
        return 2
    except (FileNotFoundError, RuntimeError, ValueError, subprocess.CalledProcessError) as exc:
        print(f"오류: {exc}", file=sys.stderr)
        return 2
    parser.error("unknown command")
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
