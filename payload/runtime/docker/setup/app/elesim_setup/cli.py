"""Interactive and non-interactive EleSim installer entry point."""

from __future__ import annotations

import argparse
import json
import os
import re
import subprocess
import sys
import tempfile
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable, Mapping, Sequence

from ._security_storage import (
    SecurityAuthorityError,
    remove_owned_tree,
    secure_absolute,
)
from .container_installer import ContainerInstaller
from .installer import Installer, preflight_notes
from .instance_identity import project_name
from .instance_runtime import InstanceRuntime
from .instance_security import stage_instance_security
from .instances import InstanceEndpoint, InstanceRegistry, InstanceState, instance_turn_secret_path
from .ownership import OwnershipManifest, default_manifest_path
from .profiles import PROFILES, ROLE_ORDER, normalize_roles, roles_for_profile
from .releases import list_releases, load_release, release_key
from .request import SetupRequest, container_network_settings_for_host
from .state import (
    ComputeSettings,
    DEFAULT_BIN_DIR,
    DEFAULT_PREFIX,
    DEFAULT_SOURCE_REF,
    DEFAULT_SOURCE_REPOSITORY,
    DdsSettings,
    DeveloperAttachmentSettings,
    InstallState,
    NetworkSettings,
    RuntimeTextLogSettings,
    TurnSettings,
    default_state_path,
)


Input = Callable[[str], str]


def _load_instance_context(state_path: Path) -> tuple[InstallState, OwnershipManifest]:
    """Load one scoped installation before any release/instance operation.

    Instance state and the ownership manifest are deliberately checked
    together.  In particular, a legacy ``elesim-runtime`` installation is not
    silently adopted by the instance namespace: it remains on the historical
    installer/update path until an explicit migration exists.
    """

    state = InstallState.load(state_path)
    if state.install_mode != "container":
        raise ValueError("instance operations require a container installation")
    prefix = state.prefix_path
    manifest_path = default_manifest_path(prefix)
    manifest = OwnershipManifest.load(manifest_path)
    docker = manifest.docker
    if manifest.prefix_path != prefix or docker is None or manifest.install_uuid != docker.install_uuid:
        raise ValueError("install-state and ownership manifest do not describe the same installation")
    expected_project = project_name(manifest.install_uuid)
    if docker.project != expected_project:
        raise ValueError(
            "instance operations require a scoped installation; "
            "the legacy elesim-runtime namespace is not adopted"
        )
    return state, manifest


def _load_instance_runtime(state_path: Path) -> tuple[InstallState, OwnershipManifest, InstanceRuntime]:
    state, manifest = _load_instance_context(state_path)
    return state, manifest, InstanceRuntime(
        state,
        manifest.install_uuid,
        ownership_manifest=Path(manifest.manifest_path),
    )


def _parse_instance_endpoint(value: str) -> InstanceEndpoint:
    """Parse the compact, repeatable ``role:endpoint_id`` CLI form."""

    role, separator, endpoint_id = str(value).partition(":")
    if not separator or not role or not endpoint_id:
        raise ValueError("--endpoint는 role:endpoint_id 형식이어야 합니다")
    return InstanceEndpoint(role, endpoint_id)


def _scoped_security_stage_root(
    state: InstallState,
    *,
    system_id: str,
    generation: str,
    supplied: str | os.PathLike[str],
    allow_missing: bool = False,
) -> Path:
    """Accept only the manager's exact, instance-private staging directory."""

    if re.fullmatch(r"[a-z][a-z0-9_]{0,62}", system_id) is None:
        raise ValueError("security bundle staging system ID is invalid")
    if re.fullmatch(r"[a-z0-9][a-z0-9_.-]{0,95}", generation) is None:
        raise ValueError("security bundle staging generation is invalid")
    expected = secure_absolute(
        state.prefix_path
        / "maintenance"
        / ".connection-scoped"
        / system_id
        / generation
    )
    candidate = secure_absolute(Path(supplied))
    if candidate != expected:
        raise ValueError(
            "security bundle staging root must match this instance and generation"
        )
    if candidate.is_symlink() or (
        not candidate.is_dir() and not (allow_missing and not candidate.exists())
    ):
        raise ValueError("security bundle staging root is unavailable")
    return candidate


def _instance_from_args(args: argparse.Namespace, state: InstallState) -> InstanceState:
    if not args.endpoint:
        raise ValueError("인스턴스에는 --endpoint role:endpoint_id가 하나 이상 필요합니다")
    endpoints = tuple(_parse_instance_endpoint(value) for value in args.endpoint)
    role_ids = {
        "pilot": state.network.pilot_id,
        "sim": state.network.sim_id,
        "ui": state.network.ui_id,
    }
    role_ids.update({endpoint.role: endpoint.endpoint_id for endpoint in endpoints})
    supplied_role_ids: set[str] = set()
    for value in getattr(args, "graph_endpoint", None) or ():
        endpoint = _parse_instance_endpoint(value)
        if endpoint.role in supplied_role_ids:
            raise ValueError(
                f"graph endpoint role이 중복되었습니다: {endpoint.role}"
            )
        supplied_role_ids.add(endpoint.role)
        role_ids[endpoint.role] = endpoint.endpoint_id
    dds = state.dds
    generation = ""
    if args.security_generation:
        generation = Path(args.security_generation).expanduser().resolve().name
    requested_gpu_mode = getattr(args, "gpu_mode", None)
    requested_gpu_device = getattr(args, "gpu_device", None)
    if requested_gpu_mode is None:
        # Omitted compute flags deliberately copy the installed policy.  This
        # is important for old fixed-device installs and makes registration
        # deterministic when called by the connection manager.
        if requested_gpu_device is not None:
            raise ValueError("--gpu-device requires --gpu-mode specific")
        compute = state.compute
    else:
        compute = ComputeSettings(
            gpu_mode=requested_gpu_mode,
            gpu_device="" if requested_gpu_device is None else requested_gpu_device,
        ).validate()
    turn_mode = getattr(args, "turn_mode", None) or "none"
    turn_urls = tuple(getattr(args, "turn_url", None) or ())
    if turn_mode == "none" and turn_urls:
        raise ValueError("--turn-url requires --turn-mode managed or external")
    if turn_mode == "managed":
        turn = TurnSettings(
            mode="managed",
            realm=(getattr(args, "turn_realm", None) or args.system),
            public_host=(getattr(args, "turn_public_host", None) or ""),
            # The managed secret is always generated under this instance;
            # there is intentionally no arbitrary secret-path CLI option.
            secret_file=str(instance_turn_secret_path(state.prefix_path, args.system)),
            listen_port=getattr(args, "turn_listen_port", None),
            relay_min_port=getattr(args, "turn_relay_min_port", None),
            relay_max_port=getattr(args, "turn_relay_max_port", None),
        )
    elif turn_mode == "external":
        credential = str(getattr(args, "turn_credential_file", "") or "").strip()
        if not credential:
            raise ValueError("external instance TURN requires --turn-credential-file")
        turn = TurnSettings(mode="external", credential_file=credential)
    else:
        turn = TurnSettings()
    return InstanceState(
        system_id=args.system,
        release_key=args.release,
        endpoints=endpoints,
        domain_id=args.domain_id if args.domain_id is not None else dds.domain_id,
        pilot_id=role_ids["pilot"],
        sim_id=role_ids["sim"],
        ui_id=role_ids["ui"],
        rmw_implementation=args.rmw if args.rmw is not None else dds.rmw_implementation,
        discovery_mode=(
            args.discovery_mode
            if args.discovery_mode is not None
            else dds.discovery_mode
        ),
        static_peers=tuple(
            args.static_peer if args.static_peer is not None else dds.static_peers
        ),
        interface=args.interface if args.interface is not None else dds.interface,
        security_profile=(
            args.security_profile
            if args.security_profile is not None
            else dds.security_profile
        ),
        security_generation=generation,
        turn=turn,
        turn_urls=turn_urls,
        compute=compute,
        # A registration without compute flags is a compatibility copy of the
        # install policy, not a new override.  This keeps legacy selectors
        # accepted by the install-level state validator usable here.
        compute_is_explicit=requested_gpu_mode is not None,
    )


def _docker_instance_service_verifier(
    compose: Path,
    project: str,
    services: tuple[str, ...],
    *,
    docker_context: str = "",
) -> Mapping[str, bool]:
    """Read exact target service state without issuing a Docker mutation."""

    command = (
        "docker",
        *(('--context', docker_context) if docker_context else ()),
        "compose",
        "--project-name",
        project,
        "--file",
        str(compose),
        "ps",
        "--all",
        "--format",
        "json",
    )
    completed = subprocess.run(
        command,
        capture_output=True,
        text=True,
        check=False,
    )
    if completed.returncode != 0:
        detail = (completed.stderr or completed.stdout or "").strip()
        raise RuntimeError(
            "Docker service stop verification failed"
            + (f": {detail[-400:]}" if detail else "")
        )
    rows: list[Mapping[str, object]] = []
    output = completed.stdout.strip()
    if output:
        try:
            decoded = json.loads(output)
        except json.JSONDecodeError:
            decoded = [json.loads(line) for line in output.splitlines() if line.strip()]
        if isinstance(decoded, Mapping):
            rows.append(decoded)
        elif isinstance(decoded, list) and all(isinstance(row, Mapping) for row in decoded):
            rows.extend(decoded)
        else:
            raise RuntimeError("Docker service stop verification returned malformed JSON")

    by_service: dict[str, bool] = {}
    wanted = set(services)
    for row in rows:
        service = row.get("Service", row.get("service"))
        if not isinstance(service, str) or service not in wanted:
            continue
        status = row.get("State", row.get("state", row.get("Status", row.get("status", ""))))
        if not isinstance(status, str):
            raise RuntimeError("Docker service stop verification returned malformed state")
        normalized = status.strip().lower()
        # ``False`` means *absent*, not merely stopped.  A stopped/exited
        # target still exists and must not be left behind when the aggregate
        # registry entry is removed.
        if not normalized:
            raise RuntimeError("Docker service stop verification returned an empty state")
        by_service[service] = True
    # A service with no container is absent.  Returning every target key is
    # important: InstanceRuntime rejects partial or foreign results.
    return {service: by_service.get(service, False) for service in services}


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
    print("\nEleSim 설치 마법사")
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
    developer_attachment = DeveloperAttachmentSettings()
    if install_mode == "container" and _yes_no(
        "이 설치에 개발 도구 컨테이너를 추가합니까?",
        default=False,
        input_fn=input_fn,
    ):
        workspace = Path(
            _ask("EleSim Git workspace", str(Path.cwd()), input_fn=input_fn)
        ).expanduser().resolve()
        developer_attachment = DeveloperAttachmentSettings(
            enabled=True,
            workspace=str(workspace),
        ).validate()
        from .capabilities import detect_install_host_capabilities

        attachment_capabilities = detect_install_host_capabilities()
        if not attachment_capabilities.developer_installable:
            raise ValueError(
                "developer attachment는 Ubuntu/WSL amd64에서만 지원합니다"
            )
        developer_attachment = replace(
            developer_attachment,
            wslg=attachment_capabilities.wslg_available,
        ).validate()

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
    if (
        install_mode == "container"
        and "sim" in roles
        and security_profile == "sros2"
    ):
        # SROS2 Sim owns the managed WebRTC relay.  The endpoint is deliberately
        # left pending here; elesim-connections derives it from the current Sim
        # host address after the topology is saved.  Trusted-network Sim uses
        # direct ICE and deliberately emits no Coturn service or credentials.
        turn = TurnSettings(
            mode="managed",
            realm=_ask("TURN realm", "elesim.local", input_fn=input_fn),
            secret_file=str(prefix / "secrets/turn.secret"),
        )

    state = InstallState(
        profile=profile_name,
        roles=roles,
        prefix=str(prefix),
        bin_dir=str(bin_dir),
        source_root=str(source_root),
        source_repository=os.environ.get(
            "ELESIM_REPOSITORY", DEFAULT_SOURCE_REPOSITORY
        ),
        source_ref=os.environ.get("ELESIM_REF", DEFAULT_SOURCE_REF),
        network=NetworkSettings(turn_urls=turn_urls),
        dds=dds,
        compute=compute,
        turn=turn,
        runtime_text_logs=runtime_text_logs,
        developer_attachment=developer_attachment,
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
        (candidate / "payload/runtime/common/protocol/pyproject.toml").is_file()
        and (candidate / "payload/runtime/common/elesim_interfaces/package.xml").is_file()
    ):
        return candidate
    raise FileNotFoundError("--source-root를 지정하거나 EleSim 저장소 루트에서 실행하십시오")


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
    sim_turn_required = (
        install_mode == "container"
        and "sim" in roles
        and args.dds_security_profile == "sros2"
    )
    if sim_turn_required:
        if args.turn_mode in {"none", "external"}:
            raise ValueError(
                "SROS2 Sim 설치는 Coturn을 포함한 managed TURN만 지원합니다"
            )
        turn_mode = "managed"
        if args.dds_security_profile != "sros2":
            raise ValueError(
                "Sim에 포함되는 Coturn은 SROS2 보안 profile과 함께 사용해야 합니다"
            )
    else:
        if args.turn_mode == "managed":
            raise ValueError("managed Coturn은 Sim 설치에서만 사용할 수 있습니다")
        if turn_urls:
            raise ValueError(
                "새 설치에서는 외부 TURN relay를 지정할 수 없습니다. "
                "TURN은 Sim과 함께 설치됩니다"
            )
        turn_mode = "none"
    secret_file = args.turn_secret_file
    if turn_mode == "managed" and not secret_file:
        secret_file = str(Path(args.prefix).expanduser().resolve() / "secrets/turn.secret")
    return InstallState(
        profile=profile_name,
        roles=roles,
        prefix=str(Path(args.prefix).expanduser().resolve()),
        bin_dir=str(Path(args.bin_dir).expanduser().resolve()),
        source_root=str(source_root),
        source_repository=args.repository,
        source_ref=args.ref,
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
            realm=(
                args.turn_realm
                if args.turn_realm
                else "elesim.local" if turn_mode == "managed" else ""
            ),
            public_host=args.turn_public_host,
            secret_file=secret_file,
            credential_file=getattr(args, "turn_credential_file", ""),
        ),
        runtime_text_logs=RuntimeTextLogSettings(
            enabled=args.runtime_text_logs,
        ),
        developer_attachment=DeveloperAttachmentSettings(
            enabled=bool(args.developer_attachment),
            workspace=(
                str(Path(args.developer_workspace).expanduser().resolve())
                if args.developer_attachment
                else ""
            ),
        ),
        install_mode=install_mode,
        install_go2_mpc=not args.skip_go2_mpc,
    ).require_installable_dds()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-root", default="", help="EleSim source archive/checkout root")
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
        default=os.environ.get("ELESIM_REPOSITORY", DEFAULT_SOURCE_REPOSITORY),
    )
    gui.add_argument("--ref", default=os.environ.get("ELESIM_REF", DEFAULT_SOURCE_REF))

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
    install.add_argument(
        "--developer-attachment",
        action="store_true",
        help="동일 elesim-runtime project에 persistent 개발 도구를 추가",
    )
    install.add_argument(
        "--developer-workspace",
        default=str(Path.cwd()),
        help="developer attachment가 bind mount할 EleSim Git checkout",
    )
    install.add_argument(
        "--repository",
        default=os.environ.get("ELESIM_REPOSITORY", DEFAULT_SOURCE_REPOSITORY),
        help="update가 다시 가져올 GitHub owner/repository",
    )
    install.add_argument(
        "--ref",
        default=os.environ.get("ELESIM_REF", DEFAULT_SOURCE_REF),
        help="update가 다시 가져올 Git ref",
    )
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
        choices=("auto", "managed"),
        default="auto",
    )
    install.add_argument("--turn-realm", default="")
    install.add_argument("--turn-public-host", default="")
    install.add_argument("--turn-secret-file", default="")
    install.add_argument("--skip-go2-mpc", action="store_true")
    install.add_argument(
        "--runtime-text-logs",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="종료 시와 elesim-logs --save에서 로컬 평문 로그 archive 저장",
    )
    install.add_argument("--dry-run", action="store_true")

    update = subparsers.add_parser(
        "update",
        help="기존 ownership 설치를 새 source로 재생성",
    )
    update.add_argument(
        "--edition",
        choices=("general",),
        default="general",
        help=argparse.SUPPRESS,
    )
    update.add_argument("--dry-run", action="store_true")

    subparsers.add_parser("status", help="현재 설치 상태 출력")
    instances = subparsers.add_parser("instances", help="등록된 release instance 조회")
    instances.add_argument("--prefix", default=str(DEFAULT_PREFIX))
    instances.add_argument("--system", default=None)

    releases = subparsers.add_parser("releases", help="설치된 immutable release 조회")
    releases.add_argument("--prefix", default=None, help=argparse.SUPPRESS)
    releases.add_argument("--release", default=None, help="release SHA-256 key")

    # This is intentionally separate from the legacy ``update`` command:
    # the host wrapper builds images and supplies evidence, while this
    # no-Docker command only publishes a validated immutable release.
    release = subparsers.add_parser("release", help=argparse.SUPPRESS)
    release_actions = release.add_subparsers(dest="release_action", required=True)
    publish = release_actions.add_parser("publish", help=argparse.SUPPRESS)
    publish.add_argument("--snapshot", required=True, metavar="PATH", help=argparse.SUPPRESS)
    publish.add_argument("--evidence", required=True, metavar="PATH", help=argparse.SUPPRESS)
    publish.add_argument("--source-revision", required=True, metavar="REVISION", help=argparse.SUPPRESS)

    instance = subparsers.add_parser(
        "instance",
        help="scoped immutable release instance 관리",
    )
    instance_actions = instance.add_subparsers(dest="instance_action", required=True)
    for action in ("register", "replace"):
        command = instance_actions.add_parser(
            action,
            help=("새 instance 등록" if action == "register" else "기존 instance 교체"),
        )
        command.add_argument("--system", required=True)
        command.add_argument("--release", required=True)
        command.add_argument("--domain-id", type=int, default=None)
        command.add_argument("--rmw", default=None, metavar="IMPLEMENTATION")
        command.add_argument(
            "--discovery-mode",
            choices=("multicast", "static"),
            default=None,
        )
        command.add_argument(
            "--static-peer",
            action="append",
            default=None,
            metavar="HOST",
        )
        command.add_argument("--interface", default=None, metavar="NAME")
        command.add_argument(
            "--security-profile",
            choices=("trusted-network", "sros2"),
            default=None,
        )
        command.add_argument(
            "--endpoint",
            action="append",
            default=[],
            metavar="ROLE:ID",
            help="반복 지정할 endpoint (pilot:pilot-1 등)",
        )
        command.add_argument(
            "--graph-endpoint",
            action="append",
            default=[],
            metavar="ROLE:ID",
            help=argparse.SUPPRESS,
        )
        command.add_argument(
            "--security-generation",
            default=None,
            metavar="PATH",
            help="이미 stage/publish된 SROS2 generation directory",
        )
        command.add_argument(
            "--security-bundle-root",
            default=None,
            help=argparse.SUPPRESS,
        )
        command.add_argument(
            "--gpu-mode",
            choices=("inherit", "specific", "cpu"),
            default=None,
            help="이 instance의 Pilot/Sim GPU 정책 (생략하면 설치 정책 복사)",
        )
        command.add_argument(
            "--gpu-device",
            default=None,
            metavar="INDEX_OR_UUID",
            help="specific GPU 정책의 단일 index/UUID",
        )
        command.add_argument(
            "--turn-mode", choices=("none", "managed", "external"), default=None,
            help="이 instance의 Sim WebRTC TURN 정책",
        )
        command.add_argument("--turn-url", action="append", default=None, metavar="URL")
        command.add_argument("--turn-realm", default=None, metavar="REALM")
        command.add_argument("--turn-public-host", default=None, metavar="HOST")
        command.add_argument("--turn-listen-port", type=int, default=None)
        command.add_argument("--turn-relay-min-port", type=int, default=None)
        command.add_argument("--turn-relay-max-port", type=int, default=None)
        command.add_argument(
            "--turn-credential-file", default=None, metavar="PATH",
            help="external TURN 자격 JSON (외부 파일은 EleSim이 소유하지 않음)",
        )
    rotate = instance_actions.add_parser(
        "rotate",
        help="한 instance의 managed SROS2 generation 교체 (로컬 transaction만 수행)",
    )
    rotate.add_argument("--system", required=True)
    rotate.add_argument(
        "--security-generation",
        required=True,
        metavar="PATH",
        help="이미 stage/publish된 SROS2 generation directory",
    )
    cleanup_staging = instance_actions.add_parser(
        "cleanup-staging",
        help=argparse.SUPPRESS,
    )
    cleanup_staging.add_argument("--system", required=True)
    cleanup_staging.add_argument(
        "--security-generation",
        required=True,
        metavar="GENERATION",
        help=argparse.SUPPRESS,
    )
    remove = instance_actions.add_parser("remove", help="중지된 instance 제거")
    remove.add_argument("--system", required=True)
    remove.add_argument("--host-lease", default=None, help=argparse.SUPPRESS)
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    state_path = Path(args.state).expanduser().resolve()
    try:
        if args.command == "instances":
            prefix = Path(args.prefix).expanduser()
            if prefix.is_symlink() or not prefix.is_dir():
                raise FileNotFoundError(f"instance registry prefix is unavailable: {prefix}")
            registry = InstanceRegistry(prefix)
            if args.system is None:
                payload = [value.to_dict() for value in registry.list()]
            else:
                payload = registry.load(args.system).to_dict()
            print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
            return 0
        if args.command == "releases":
            state, manifest = _load_instance_context(state_path)
            prefix = state.prefix_path if args.prefix is None else Path(args.prefix).expanduser().resolve()
            if prefix != state.prefix_path:
                raise ValueError("release prefix must match install-state prefix")
            found = [
                loaded.to_dict()
                for loaded in list_releases(prefix, install_uuid=manifest.install_uuid)
            ]
            if args.release is not None:
                selected = next(
                    (entry for entry in found if entry.get("release_key") == args.release),
                    None,
                )
                if selected is None:
                    raise FileNotFoundError(f"release not found: {args.release}")
                payload = selected
            else:
                payload = found
            print(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True))
            return 0
        if args.command == "release":
            if args.release_action != "publish":
                raise ValueError("지원하지 않는 release action입니다")
            from .release_publication import publish_from_evidence

            state, manifest = _load_instance_context(state_path)
            result = publish_from_evidence(
                state,
                manifest,
                source_revision=args.source_revision,
                runtime_snapshot=Path(args.snapshot).expanduser(),
                evidence_path=Path(args.evidence).expanduser(),
            )
            print(json.dumps(result.to_dict(), ensure_ascii=False, sort_keys=True))
            return 0
        if args.command == "instance":
            if args.instance_action == "cleanup-staging":
                state, _manifest = _load_instance_context(state_path)
                generation_path = Path(args.security_generation).expanduser()
                if generation_path.is_absolute() or len(generation_path.parts) != 1:
                    raise ValueError(
                        "security staging cleanup requires a bare generation ID"
                    )
                staging = _scoped_security_stage_root(
                    state,
                    system_id=args.system,
                    generation=generation_path.name,
                    supplied=(
                        state.prefix_path
                        / "maintenance"
                        / ".connection-scoped"
                        / args.system
                        / generation_path.name
                    ),
                    allow_missing=True,
                )
                if os.path.lexists(staging):
                    try:
                        remove_owned_tree(
                            staging,
                            owner=(
                                state.prefix_path
                                / "maintenance"
                                / ".connection-scoped"
                            ),
                        )
                    except SecurityAuthorityError as exc:
                        raise ValueError(str(exc)) from exc
                return 0
            state, manifest, runtime = _load_instance_runtime(state_path)
            if args.instance_action == "remove":
                context = manifest.docker.context if manifest.docker is not None else ""

                def verifier(compose, project, services):
                    return _docker_instance_service_verifier(
                        compose,
                        project,
                        services,
                        docker_context=context,
                    )

                runtime.remove(
                    args.system,
                    verifier=(None if args.host_lease is not None else verifier),
                    host_lease=args.host_lease,
                )
                return 0
            if args.instance_action == "rotate":
                registry = InstanceRegistry(state.prefix_path)
                instance = registry.load(args.system)
                if instance.security_profile != "sros2":
                    raise ValueError("security rotation requires the sros2 profile")
                generation_path = Path(args.security_generation).expanduser()
                runtime.rotate_security(
                    replace(instance, security_generation=generation_path.name),
                    generation_path,
                )
                return 0
            instance = _instance_from_args(args, state)
            release_path = state.prefix_path / "releases" / instance.release_key
            release = load_release(release_path)
            if release_key(release) != instance.release_key:
                raise ValueError("release key does not match its manifest")
            security_generation = args.security_generation
            if instance.security_profile == "sros2" and not security_generation:
                raise ValueError(
                    "SROS2 instance registration requires --security-generation "
                    "from an explicitly staged generation"
                )
            generation_name = (
                "" if not security_generation else Path(security_generation).name
            )
            security_stage_root = None
            validated_security: tempfile.TemporaryDirectory[str] | None = None
            security_result: str | None = None
            if args.security_bundle_root:
                try:
                    security_stage_root = _scoped_security_stage_root(
                        state,
                        system_id=instance.system_id,
                        generation=generation_name,
                        supplied=args.security_bundle_root,
                    )
                except SecurityAuthorityError as exc:
                    raise ValueError(str(exc)) from exc
            if security_stage_root is not None:
                if instance.security_profile != "sros2" or not security_generation:
                    raise ValueError("security bundle staging requires managed SROS2")
                source_views = {
                    endpoint.role: security_stage_root / "apps" / endpoint.role / "keystore"
                    for endpoint in instance.endpoints
                }
                if any(
                    path.is_symlink() or not path.is_dir()
                    for path in source_views.values()
                ):
                    raise ValueError("security bundle staging views are incomplete")
                maintenance = secure_absolute(state.prefix_path / "maintenance")
                if maintenance.is_symlink() or not maintenance.is_dir():
                    raise ValueError("installation maintenance root is unavailable")
                # Validate and publish into an operation-private temporary
                # prefix.  Publishing directly below the final instance tree
                # would escape InstanceRuntime's atomic state/Compose commit
                # and make that commit reject its own pre-created target.
                validated_security = tempfile.TemporaryDirectory(
                    prefix=".validated-instance-security-",
                    dir=maintenance,
                )
                staged = stage_instance_security(
                    Path(validated_security.name),
                    manifest.install_uuid,
                    instance,
                    Path(security_generation).name,
                    source_views,
                )
                security_result = str(staged.root)
            if security_generation:
                if instance.security_profile != "sros2":
                    raise ValueError("--security-generation is valid only for SROS2")
                generation_path = Path(security_generation).expanduser()
                if security_stage_root is None:
                    expected_generation = (
                        state.prefix_path
                        / "instances"
                        / instance.system_id
                        / "security"
                        / "generations"
                        / generation_path.name
                    )
                    if generation_path.is_absolute() and (
                        secure_absolute(generation_path)
                        != secure_absolute(expected_generation)
                    ):
                        raise ValueError(
                            "existing security generation must belong to this instance"
                        )
                    if len(generation_path.parts) != 1 and not generation_path.is_absolute():
                        raise ValueError(
                            "existing security generation must be a bare generation ID"
                        )
                    if args.instance_action == "replace":
                        # A compensating multi-host rollback selects a
                        # generation already retained under this instance.
                        # InstanceRuntime stages that generation together with
                        # state/config/Compose and swaps the whole boundary
                        # atomically; it never edits ``current`` in advance.
                        security_result = str(expected_generation)
            operation = runtime.register if args.instance_action == "register" else runtime.replace
            try:
                operation(
                    instance,
                    release,
                    security_result=security_result,
                )
            finally:
                if validated_security is not None:
                    validated_security.cleanup()
                if security_stage_root is not None:
                    # The manager stages only under maintenance; never leave
                    # role credentials there after the atomic registration.
                    if os.path.lexists(security_stage_root):
                        try:
                            remove_owned_tree(
                                security_stage_root,
                                owner=(
                                    state.prefix_path
                                    / "maintenance"
                                    / ".connection-scoped"
                                ),
                            )
                        except SecurityAuthorityError as exc:
                            raise ValueError(str(exc)) from exc
            return 0
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
            if state.install_mode == "container" and (
                not args.dry_run or state.developer_attachment.enabled
            ):
                from .capabilities import detect_install_host_capabilities

                capabilities = detect_install_host_capabilities()
                if state.developer_attachment.enabled:
                    if not capabilities.developer_installable:
                        raise ValueError(
                            "developer attachment는 Ubuntu/WSL amd64에서만 지원합니다"
                        )
                    state = replace(
                        state,
                        developer_attachment=replace(
                            state.developer_attachment,
                            wslg=capabilities.wslg_available,
                        ),
                    )
                if not args.dry_run:
                    state = replace(
                        state,
                        container_network=container_network_settings_for_host(
                            capabilities=capabilities,
                            install_mode=state.install_mode,
                            prefix=state.prefix_path,
                        ),
                    )
                state = state.validate()
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
        if args.command == "update":
            current = InstallState.load(state_path)
            if (
                current.install_mode == "container"
                and not current.container_network.docker_context.strip()
                and not current.container_network.docker_engine_id.strip()
            ):
                from .capabilities import detect_install_host_capabilities

                capabilities = detect_install_host_capabilities()
                current = replace(
                    current,
                    container_network=container_network_settings_for_host(
                        capabilities=capabilities,
                        install_mode=current.install_mode,
                        prefix=current.prefix_path,
                    ),
                )
                # The bootstrap passes an explicit source identity for every
                # update.  Apply it here so a pre-v9 state (or a deliberately
                # overridden current wrapper) records the ref it actually
                # fetched instead of silently regenerating a wrapper for main.
            state = replace(
                current,
                source_root=str(source_root),
                source_repository=os.environ.get(
                    "ELESIM_REPOSITORY", current.source_repository
                ).strip(),
                source_ref=os.environ.get(
                    "ELESIM_REF", current.source_ref
                ).strip(),
            ).validate()
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
