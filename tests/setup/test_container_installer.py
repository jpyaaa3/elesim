from __future__ import annotations

import hashlib
import json
import os
import pwd
import socket
import subprocess
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from elesim_setup.container_installer import (
    ContainerInstaller,
    TAILSCALE_CONTAINER_NAME,
    TAILSCALE_IMAGE,
    _entrypoint,
    _viewer_cleanup_wrapper,
    _tailscale_wrapper,
    _runtime_up_wrapper,
    _runtime_down_wrapper,
    _resolve_viewer_user,
    _reset_generated_context,
    build_container_plan,
    refresh_compose_dds_environment,
)
from elesim_setup.ownership import (
    DOCKER_BUILD_FINGERPRINT_LABEL,
    DOCKER_INSTALL_UUID_LABEL,
    OwnershipError,
    OwnershipManifest,
)
from elesim_setup.state import (
    ContainerNetworkSettings,
    DdsSettings,
    NetworkSettings,
    RuntimeTextLogSettings,
    TurnSettings,
)
from elesim_setup.uninstall import DockerObject, UninstallSafetyError


def test_prepacked_role_entrypoints_match_the_runtime_contract() -> None:
    root = Path(__file__).resolve().parents[2]
    for role in ("pilot", "sim", "ui"):
        entrypoint = (
            root / "payload" / "runtime" / "docker" / role / "entrypoint"
        )
        assert entrypoint.read_text(encoding="utf-8") == _entrypoint(role)
        assert entrypoint.stat().st_mode & 0o111


def _compose(state) -> dict:
    return yaml.safe_load(
        (state.prefix_path / "containers/compose.yaml").read_text(encoding="utf-8")
    )


def _create_x11_socket(directory: Path, display: int = 0) -> Path:
    directory.mkdir(parents=True, exist_ok=True)
    path = directory / f"X{display}"
    listener = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    try:
        listener.bind(str(path))
    finally:
        listener.close()
    return path


def _fake_docker(path: Path) -> Path:
    docker = path / "docker"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        "set -u\n"
        "if [[ -n ${ELESIM_FAKE_DOCKER_CALLS:-} ]]; then\n"
        "  printf '%s\\n' \"$*\" >>\"$ELESIM_FAKE_DOCKER_CALLS\"\n"
        "fi\n"
        "if [[ ${1:-} == info ]]; then\n"
        "  printf '%s\\n' \"${ELESIM_FAKE_ENGINE_ID:-}\"\n"
        "  exit 0\n"
        "fi\n"
        "if [[ ${1:-} == container && ${2:-} == inspect ]]; then\n"
        "  if [[ ${3:-} == elesim-manager && ${ELESIM_FAKE_MANAGER_PRESENT:-0} == 1 ]]; then\n"
        "    exit 0\n"
        "  fi\n"
        "  exit 1\n"
        "fi\n"
        "if [[ ${1:-} == rm && ${2:-} == -f && ${3:-} == elesim-manager ]]; then\n"
        "  if [[ -n ${ELESIM_MANAGER_PURGED_MARKER:-} ]]; then\n"
        "    : >\"$ELESIM_MANAGER_PURGED_MARKER\"\n"
        "  fi\n"
        "  exit \"${ELESIM_MANAGER_PURGE_STATUS:-0}\"\n"
        "fi\n"
        "arguments=\" $* \"\n"
        "if [[ $arguments == *' image inspect elesim/tools:local '* && $arguments == *'build_fingerprint'* ]]; then\n"
        "  printf '%s\\n' \"${ELESIM_FAKE_TOOLS_FINGERPRINT:-}\"\n"
        "  exit 0\n"
        "fi\n"
        "if [[ $arguments == *' build --quiet tools '* ]]; then\n"
        "  printf 'build progress that must not reach stdout\\n'\n"
        "  exit 0\n"
        "fi\n"
        "if [[ $arguments == *' run --rm -T tools elesim-net '* || $arguments == *' run --rm -T runtime-tools elesim-net '* ]]; then\n"
        "  printf '{\"schema_version\":1}\\n'\n"
        "  exit 0\n"
        "fi\n"
        "if [[ $arguments == *' exec -T tailscale tailscale --socket=/tmp/tailscaled.sock status --json '* ]]; then\n"
        "  printf '{\"BackendState\":\"%s\"}\\n' \"${ELESIM_FAKE_TAILSCALE_STATE:-Running}\"\n"
        "  exit 0\n"
        "fi\n"
        "if [[ $arguments == *' exec -T tailscale tailscale --socket=/tmp/tailscaled.sock ip -4 '* ]]; then\n"
        "  printf '100.64.0.10\\n'\n"
        "  exit 0\n"
        "fi\n"
        "if [[ $arguments == *' ps -q tailscale '* && ${ELESIM_FAKE_TAILSCALE_PRESENT:-0} == 1 ]]; then\n"
        "  printf 'sidecar-id\\n'\n"
        "  exit 0\n"
        "fi\n"
        "if [[ $arguments == *' ps -aq '* ]]; then\n"
        "  if [[ ${ELESIM_FAKE_RUNTIME_EMPTY:-0} == 1 ]]; then\n"
        "    if [[ ${ELESIM_FAKE_INFRA_PRESENT:-0} == 1 && $arguments == *' tailscale '* ]]; then\n"
        "      printf 'sidecar-id\\n'\n"
        "    fi\n"
        "    exit 0\n"
        "  fi\n"
        "  printf 'container-id\\n'\n"
        "  exit 0\n"
        "fi\n"
        "if [[ $arguments == *' logs --no-color --timestamps '* ]]; then\n"
        "  service=${!#}\n"
        "  printf 'saved log for %s\\n' \"$service\"\n"
        "  if [[ $service == ${ELESIM_FAIL_LOG_SERVICE:-} ]]; then\n"
        "    exit 19\n"
        "  fi\n"
        "  exit 0\n"
        "fi\n"
        "if [[ $arguments == *' logs -f '* ]]; then\n"
        "  printf 'live follow\\n'\n"
        "  exit 0\n"
        "fi\n"
        "if [[ $arguments == *' down --remove-orphans '* ]]; then\n"
        "  if [[ -n ${ELESIM_DOWN_MARKER:-} ]]; then\n"
        "    : >\"$ELESIM_DOWN_MARKER\"\n"
        "  fi\n"
        "  exit \"${ELESIM_DOWN_STATUS:-0}\"\n"
        "fi\n"
        "exit 0\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    return docker


def test_sim_entrypoint_preflights_x11_before_dds_and_passes_viewer_flag(
    tmp_path: Path,
) -> None:
    entrypoint = tmp_path / "entrypoint"
    # The generated image supplies these two overlays.  Replace only their
    # fixed source statements so this host-side shell test can exercise the
    # generated control flow without a ROS installation.
    entrypoint.write_text(
        _entrypoint("sim")
        .replace("source /opt/ros/humble/setup.bash", "true")
        .replace("source /opt/elesim/ros/install/setup.bash", "true"),
        encoding="utf-8",
    )
    entrypoint.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    python = fake_bin / "python3"
    python.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s\\n' \"$*\" >>\"${PYTHON_CALLS:?}\"\n"
        "exit \"${PYTHON_STATUS:-0}\"\n",
        encoding="utf-8",
    )
    python.chmod(0o755)
    sim = fake_bin / "elesim-sim"
    sim.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s\\n' \"$*\" >\"${SIM_ARGS:?}\"\n",
        encoding="utf-8",
    )
    sim.chmod(0o755)
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "DISPLAY": ":7",
            "ELESIM_SIM_VIEWER": "1",
            "PYTHON_CALLS": str(tmp_path / "python.calls"),
            "SIM_ARGS": str(tmp_path / "sim.args"),
        }
    )

    preflight = subprocess.run(
        (entrypoint, "--elesim-viewer-preflight"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert preflight.returncode == 0
    assert "pyglet.window" in (tmp_path / "python.calls").read_text(
        encoding="utf-8"
    )
    assert not (tmp_path / "sim.args").exists()

    launched = subprocess.run(
        (entrypoint, "--probe"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert launched.returncode == 0
    sim_args = (tmp_path / "sim.args").read_text(encoding="utf-8")
    assert "--viewer --probe" in sim_args
    assert (tmp_path / "python.calls").read_text(encoding="utf-8").count(
        "pyglet.window"
    ) == 2

    (tmp_path / "sim.args").unlink()
    environment["PYTHON_STATUS"] = "9"
    rejected = subprocess.run(
        (entrypoint,),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert rejected.returncode == 69
    assert "X11/GL context" in rejected.stderr
    assert not (tmp_path / "sim.args").exists()


def test_container_plan_is_router_free(local_state) -> None:
    state = local_state(
        roles=("sim", "pilot"),
        install_mode="container",
    )
    plan = build_container_plan(state)
    rendered = "\n".join(action.detail for action in plan)

    assert "sim" in rendered
    assert "pilot" in rendered
    assert "router" not in {action.title.lower() for action in plan}


def test_viewer_user_uses_bootstrap_host_user_without_passwd_entry(monkeypatch) -> None:
    monkeypatch.setenv("ELESIM_HOST_USER", "hckang")
    monkeypatch.delenv("USER", raising=False)
    monkeypatch.delenv("LOGNAME", raising=False)

    def missing_passwd_entry(uid: int):
        raise KeyError(f"getpwuid(): uid not found: {uid}")

    monkeypatch.setattr(pwd, "getpwuid", missing_passwd_entry)

    assert _resolve_viewer_user() == "hckang"


def test_viewer_user_has_numeric_fallback_when_account_metadata_is_missing(
    monkeypatch,
):
    for variable in ("ELESIM_HOST_USER", "USER", "LOGNAME"):
        monkeypatch.delenv(variable, raising=False)

    def missing_passwd_entry(uid: int):
        raise KeyError(f"getpwuid(): uid not found: {uid}")

    monkeypatch.setattr(pwd, "getpwuid", missing_passwd_entry)

    assert _resolve_viewer_user() == str(os.getuid())


def test_container_install_survives_missing_passwd_entry(
    local_state, monkeypatch
) -> None:
    monkeypatch.setenv("ELESIM_HOST_USER", "hckang")
    monkeypatch.delenv("USER", raising=False)
    monkeypatch.delenv("LOGNAME", raising=False)

    def missing_passwd_entry(uid: int):
        raise KeyError(f"getpwuid(): uid not found: {uid}")

    monkeypatch.setattr(pwd, "getpwuid", missing_passwd_entry)
    state = local_state(roles=("sim",), install_mode="container")

    ContainerInstaller(state).run()

    compose = _compose(state)
    assert compose["services"]["tools"]["environment"]["ELESIM_HOST_USER"] == (
        "hckang"
    )
    wrapper = (state.bin_path / "elesim-up").read_text(encoding="utf-8")
    assert "viewer_xhost_user=hckang" in wrapper


def test_pending_managed_coturn_does_not_pass_an_empty_external_ip(local_state) -> None:
    state = local_state(
        roles=("sim",),
        dds=DdsSettings(security_profile="sros2", security_provisioning="managed"),
        turn=TurnSettings(
            mode="managed",
            realm="elesim.local",
            secret_file="/tmp/install/secrets/turn.secret",
        ),
    )

    service = ContainerInstaller(state)._coturn_service()
    command = service["command"][0]

    assert 'if [ -n "$$TURN_PUBLIC_IP" ]' in command
    assert '--external-ip="$$TURN_PUBLIC_IP"' not in command


def test_managed_coturn_shares_the_tailscale_sidecar_namespace(local_state) -> None:
    prefix = local_state().prefix_path
    state = local_state(
        roles=("sim",),
        dds=DdsSettings(
            interface="tailscale0",
            security_profile="sros2",
            security_provisioning="managed",
        ),
        turn=TurnSettings(
            mode="managed",
            realm="elesim.local",
            secret_file=str(prefix / "secrets/turn.secret"),
        ),
        container_network=ContainerNetworkSettings(
            mode="tailscale-sidecar",
            docker_context="default",
            docker_engine_id="desktop-engine-id",
            tailscale_hostname="elesim-coturn-test",
            tailscale_state_dir=str(prefix / "secrets/tailscale"),
        ),
    )

    service = ContainerInstaller(state)._coturn_service()

    assert service["network_mode"] == "service:tailscale"
    assert service["depends_on"] == {
        "tailscale": {"condition": "service_healthy"},
        "sim": {"condition": "service_started"},
    }


def test_container_install_generates_ros_overlay_contexts_and_dds_environment(
    local_state,
    monkeypatch: pytest.MonkeyPatch,
    tmp_path: Path,
) -> None:
    install_time_authority = tmp_path / "install.Xauthority"
    install_time_authority.write_text("stale-cookie\n", encoding="utf-8")
    monkeypatch.setenv("XAUTHORITY", str(install_time_authority))
    state = local_state(
        roles=("sim", "pilot", "ui"),
        install_mode="container",
    )

    ContainerInstaller(state).run()
    compose = _compose(state)

    cache_root = state.prefix_path / "cache"
    data_root = state.prefix_path / "data"
    assert (data_root / "models/assemblies/zed-mini/bundle.json").is_file()
    assert (data_root / "models/assemblies/d435/bundle.json").is_file()
    assert (data_root / "models/perception/yolov8n-seg.pt").is_file()
    assert (data_root / "models/arm/default.json").is_file()
    assert (data_root / "models/objects/demo_box.obj").is_file()
    assert (data_root / "calibration/cameras/zed_mini.hand_eye.json").is_file()
    assert (data_root / "policies/wrap-grasp/README.md").is_file()
    assert cache_root.is_dir()
    assert cache_root.stat().st_mode & 0o777 == 0o700
    assert (cache_root / "genesis").is_dir()
    assert (cache_root / "genesis").stat().st_mode & 0o777 == 0o700
    assert compose["name"] == "elesim-runtime"
    assert set(compose["services"]) == {
        "sim",
        "pilot",
        "ui",
        "tools",
        "manager",
    }
    for role in state.roles:
        service = compose["services"][role]
        assert service["image"] == f"elesim/{role}:local"
        expected_container_names = {
            "pilot": "elesim-pilot",
            "ui": "elesim-ui",
            "sim": "elesim-sim",
        }
        assert service["container_name"] == expected_container_names[role]
        assert service["logging"] == {
            "driver": "json-file",
            "options": {"max-size": "10m", "max-file": "4"},
        }
        assert service["build"]["args"]["BASE_IMAGE"] == "ros:humble-ros-base-jammy"
        assert service["environment"]["ROS_DOMAIN_ID"] == "0"
        assert service["environment"]["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
        assert service["environment"]["CYCLONEDDS_URI"] == (
            "file:///opt/elesim/config/cyclonedds.xml"
        )
        if role == "sim":
            assert service["user"] == f"{os.getuid()}:{os.getgid()}"
            assert service["environment"]["HOME"] == "/tmp"
            assert service["environment"]["XDG_CACHE_HOME"] == "/tmp/elesim-cache"
            assert service["environment"]["NUMBA_CACHE_DIR"] == "/tmp/elesim-cache/numba"
            assert service["environment"]["ELESIM_WEBRTC_ENCODER"] == (
                "${ELESIM_WEBRTC_ENCODER:-}"
            )
            assert service["environment"]["ELESIM_WEBRTC_RTP_PAYLOAD_MAX"] == (
                "${ELESIM_WEBRTC_RTP_PAYLOAD_MAX:-1000}"
            )
            assert (
                f"{state.prefix_path / 'cache'}:/tmp/elesim-cache:rw"
                in service["volumes"]
            )
            assert not (state.prefix_path / "apps/sim/model").exists()
        if role in {"pilot", "sim", "ui"}:
            assert (
                f"{data_root}:/opt/elesim/data:ro" in service["volumes"]
            )
        else:
            assert all("/opt/elesim/data" not in item for item in service["volumes"])
        assert "depends_on" not in service
        role_keystore = state.prefix_path / "security/apps" / role
        security_mount = (
            f"{role_keystore}:"
            f"{role_keystore}:ro"
        )
        assert security_mount in service["volumes"]
        assert all(
            f"/security/apps/{other}:" not in volume
            for other in state.roles
            if other != role
            for volume in service["volumes"]
        )
        assert role_keystore.is_dir()
        assert role_keystore.stat().st_mode & 0o777 == 0o700
        context = state.prefix_path / f"containers/build/{role}"
        assert (context / "interfaces/elesim_interfaces/package.xml").is_file()
        assert not (context / "app/tests").exists()
        assert not (context / "protocol/tests").exists()
        installed_config = state.prefix_path / "apps" / role / "config"
        public_template = (
            "runtime.public.example.yaml"
            if role in {"pilot", "sim"}
            else "public.example.yaml"
        )
        assert not (installed_config / public_template).exists()
        if role in {"pilot", "sim"}:
            app_config = yaml.safe_load(
                (installed_config / "config.yaml").read_text(encoding="utf-8")
            )
            assert app_config["simulation"]["cameras"]["hand_eye"]["config"] == (
                "/opt/elesim/data/calibration/cameras/zed_mini.hand_eye.json"
            )
        if role == "sim":
            assert app_config["simulation"]["assembly"]["build_dir"] == (
                "/opt/elesim/data/models/assemblies/zed-mini"
            )
        if role in {"pilot", "ui"}:
            assert (
                installed_config / "perception/detector.yolo.example.json"
            ).is_file()
        entrypoint = (context / "entrypoint").read_text(encoding="utf-8")
        assert "set +u\nsource /opt/ros/humble/setup.bash" in entrypoint
        assert "source /opt/elesim/ros/install/setup.bash\nset -u" in entrypoint
        if role == "sim":
            assert service["environment"]["ELESIM_SIM_VIEWER"] == (
                "${ELESIM_SIM_VIEWER:-}"
            )
            assert service["environment"]["DISPLAY"] == "${DISPLAY:-:0}"
            assert "/tmp/.X11-unix:/tmp/.X11-unix:rw" in service["volumes"]
            assert "XAUTHORITY" not in service["environment"]
            assert not any(
                str(install_time_authority) in volume
                for volume in service["volumes"]
            )
            assert 'ELESIM_SIM_VIEWER:-' in entrypoint
            assert "--elesim-viewer-preflight" in entrypoint
            assert "Window(width=1, height=1, visible=False)" in entrypoint
            assert "sim_args+=(--viewer)" in entrypoint
            assert entrypoint.index("Window(width=1") < entrypoint.index(
                "exec elesim-sim"
            )
        if role == "ui":
            assert service["environment"]["XAUTHORITY"] == str(
                install_time_authority
            )
            assert (
                f"{install_time_authority}:{install_time_authority}:ro"
                in service["volumes"]
            )
    tools = state.prefix_path / "containers/build/tools"
    assert (tools / "interfaces/elesim_interfaces/msg/RgbdFrame.msg").is_file()
    assert not (tools / "protocol/tests").exists()
    assert not (tools / "setup/tests").exists()
    assert compose["services"]["tools"]["image"] == "elesim/tools:local"
    assert compose["services"]["tools"]["environment"]["ELESIM_HOST_USER"] == (
        _resolve_viewer_user()
    )
    assert "container_name" not in compose["services"]["tools"]
    manager = compose["services"]["manager"]
    assert manager["profiles"] == ["manager"]
    assert "container_name" not in manager
    assert "network_mode" not in manager
    assert "/var/run/docker.sock:/var/run/docker.sock:rw" not in manager["volumes"]
    assert manager["environment"]["DOCKER_CONFIG"] == "/tmp/elesim-docker-config"
    assert "group_add" not in manager
    wrapper = (state.bin_path / "elesim-connections").read_text(encoding="utf-8")
    assert "--name elesim-manager" in wrapper
    assert '--publish "127.0.0.1:${manager_port}:${manager_port}"' in wrapper
    assert "manager_args+=(--host 0.0.0.0)" in wrapper
    assert "ELESIM_INSTALL_GPU_MODE=inherit" in wrapper
    assert "ELESIM_TAILSCALE_PROXY_BIN=/usr/local/bin/elesim-host-proxy" in wrapper
    assert "/var/run/tailscale/tailscaled.sock" not in wrapper
    assert "elesim_setup.host_helper" in wrapper
    assert "ELESIM_HOST_HELPER_SOCKET=/run/elesim-host-helper/helper.sock" in wrapper
    assert "existing_manager=\"$(docker ps -aq" in wrapper
    assert "manager_running=\"$(docker inspect" in wrapper
    assert "manager_started=0" in wrapper
    assert "trap 'host_helper_cleanup; manager_cleanup' EXIT" in wrapper
    assert "docker rm elesim-manager" in wrapper
    assert "docker rm -f elesim-manager" in wrapper
    assert "manager_status=$?" in wrapper
    assert "ELESIM_DOCKER_GID" not in wrapper
    assert "elesim-manager-compose" not in wrapper
    assert "--group-add" not in wrapper
    assert "IFS=',' read -r -a compose_files" in wrapper
    assert "compose_match != 1" in wrapper
    assert f"--local-install-root {state.prefix_path}" in wrapper
    up_wrapper = (state.bin_path / "elesim-up").read_text(encoding="utf-8")
    viewer_cleanup_wrapper = (
        state.bin_path / "elesim-viewer-cleanup"
    ).read_text(encoding="utf-8")
    update_wrapper = (state.bin_path / "elesim-update").read_text(encoding="utf-8")
    down_wrapper = (state.bin_path / "elesim-down").read_text(encoding="utf-8")
    assert "up -d --build --remove-orphans" in up_wrapper
    assert "--view" in up_wrapper
    assert "--viewer-user" in up_wrapper
    assert "ELESIM_VIEWER_USER" in up_wrapper
    assert "export ELESIM_SIM_VIEWER=1" in up_wrapper
    assert "DISPLAY" in up_wrapper
    assert (
        f"viewer_xhost_user={_resolve_viewer_user()}" in up_wrapper
    )
    assert 'xhost +si:localuser:"$viewer_xhost_user"' in up_wrapper
    assert "run --rm -T --build --no-deps sim --elesim-viewer-preflight" in up_wrapper
    assert "run --rm -T --no-deps sim --elesim-viewer-preflight" in up_wrapper
    assert "viewer-xhost" in up_wrapper
    assert "viewer_xhost_select_state" in up_wrapper
    assert "viewer_display_is_owned" in up_wrapper
    assert "viewer_xhost_session_uid" in up_wrapper
    assert "SSH 사용자" in up_wrapper
    assert "xrandr --listmonitors" in up_wrapper
    assert ".runtime-cache/viewer-xhost" in up_wrapper
    assert "elesim-net configuration-check >/dev/null" in up_wrapper
    assert "elesim-net namespace-check >/dev/null" in up_wrapper
    net_wrapper = (state.bin_path / "elesim-net").read_text(encoding="utf-8")
    assert "docker_backend_name" in net_wrapper
    assert "docker_backend_kind=docker-desktop" in net_wrapper
    manager_wrapper = (state.bin_path / "elesim-connections").read_text(
        encoding="utf-8"
    )
    assert "tailscale[0-9]+" in manager_wrapper
    assert "ELESIM_TAILSCALE_INTERFACE" in manager_wrapper
    assert "down --remove-orphans" in down_wrapper
    assert "down --remove-orphans" in down_wrapper
    assert "elesim-down [--purge]" in down_wrapper
    assert "docker rm -f elesim-manager" in down_wrapper
    assert 'xhost -si:localuser:"$viewer_xhost_user"' in down_wrapper
    assert "viewer_xhost_cleanup" in down_wrapper
    assert "viewer_xhost_cleanup" in viewer_cleanup_wrapper
    assert "docker" not in viewer_cleanup_wrapper
    for role in state.roles:
        assert not (state.bin_path / f"elesim-{role}").exists()
    assert "--edition" not in update_wrapper
    assert "build sim pilot ui tools" in update_wrapper
    assert "elesim_cleanup_owned_dangling_image" in update_wrapper
    assert "docker image prune" not in update_wrapper
    assert (state.prefix_path / "security").stat().st_mode & 0o777 == 0o700
    for generated_wrapper in (
        state.bin_path / "elesim-up",
        state.bin_path / "elesim-down",
        state.bin_path / "elesim-viewer-cleanup",
    ):
        assert subprocess.run(
            ("bash", "-n", str(generated_wrapper)),
            check=False,
        ).returncode == 0
    manifest = OwnershipManifest.load(state.prefix_path / "install-ownership.json")
    assert str(state.bin_path / "elesim-viewer-cleanup") in {
        wrapper.path for wrapper in manifest.wrappers
    }


def test_docker_desktop_install_generates_stable_kernel_tailscale_sidecar(
    local_state,
) -> None:
    prefix = local_state().prefix_path
    network = ContainerNetworkSettings(
        mode="tailscale-sidecar",
        docker_context="default",
        docker_engine_id="desktop-engine-id",
        tailscale_hostname="elesim-deadbeef0123",
        tailscale_state_dir=str(prefix / "secrets/tailscale"),
    )
    state = local_state(
        roles=("pilot", "ui"),
        container_network=network,
    )

    ContainerInstaller(state).run()
    compose = _compose(state)
    services = compose["services"]

    assert set(services) == {
        "tailscale",
        "pilot",
        "ui",
        "tools",
        "runtime-tools",
        "manager",
    }
    tailscale = services["tailscale"]
    assert tailscale["image"] == TAILSCALE_IMAGE
    assert tailscale["image"] == "tailscale/tailscale:stable"
    assert tailscale["container_name"] == TAILSCALE_CONTAINER_NAME
    assert tailscale["devices"] == ["/dev/net/tun:/dev/net/tun"]
    assert tailscale["cap_add"] == ["NET_ADMIN", "NET_RAW"]
    assert tailscale["entrypoint"] == ["tailscaled"]
    assert tailscale["command"] == [
        "--statedir=/var/lib/tailscale",
        "--socket=/tmp/tailscaled.sock",
        "--tun=tailscale0",
    ]
    assert "AUTH" not in json.dumps(tailscale).upper()
    state_dir = prefix / "secrets/tailscale"
    assert tailscale["volumes"] == [f"{state_dir}:/var/lib/tailscale:rw"]
    assert state_dir.stat().st_mode & 0o777 == 0o700
    for role in ("pilot", "ui"):
        assert services[role]["network_mode"] == "service:tailscale"
        assert services[role]["depends_on"] == {
            "tailscale": {"condition": "service_healthy"}
        }
    assert services["tools"]["network_mode"] == "host"
    assert "depends_on" not in services["tools"]
    assert services["runtime-tools"]["network_mode"] == "service:tailscale"
    assert "build" not in services["runtime-tools"]
    assert services["runtime-tools"]["depends_on"] == {
        "tailscale": {"condition": "service_healthy"}
    }
    runtime_tool_volumes = services["runtime-tools"]["volumes"]
    assert all(volume.endswith(":ro") for volume in runtime_tool_volumes)
    assert not any(str(state_dir) in volume for volume in runtime_tool_volumes)
    assert f"{prefix}:{prefix}:rw" not in runtime_tool_volumes
    assert "network_mode" not in services["manager"]
    assert "depends_on" not in services["manager"]
    manager_volumes = services["manager"]["volumes"]
    assert f"{prefix}:{prefix}:rw" not in manager_volumes
    assert not any(str(state_dir) in volume for volume in manager_volumes)
    assert {
        f"{prefix / name}:{prefix / name}:rw"
        for name in ("connections", "authority", "security")
    }.issubset(set(manager_volumes))
    assert services["manager"]["tmpfs"] == [
        f"{prefix / 'secrets'}:mode=0700"
    ]
    for name in ("connections", "authority", "security", "secrets"):
        assert (prefix / name).stat().st_mode & 0o777 == 0o700

    compose_wrapper = (state.bin_path / "elesim-compose").read_text(encoding="utf-8")
    tailscale_wrapper = (state.bin_path / "elesim-tailscale").read_text(
        encoding="utf-8"
    )
    net_wrapper = (state.bin_path / "elesim-net").read_text(encoding="utf-8")
    up_wrapper = (state.bin_path / "elesim-up").read_text(encoding="utf-8")
    update_wrapper = (state.bin_path / "elesim-update").read_text(encoding="utf-8")
    assert "export DOCKER_CONTEXT=\"$expected_docker_context\"" in compose_wrapper
    assert "expected_docker_engine_id=desktop-engine-id" in compose_wrapper
    assert "for _docker_guard_attempt in {1..5}" in compose_wrapper
    assert "[[ -n $candidate_docker_engine_id ]]" in compose_wrapper
    assert "--elesim-cuda-visible-devices" in compose_wrapper
    assert "--elesim-sim-viewer" in compose_wrapper
    assert "10#$runtime_cuda_visible > 65535" in compose_wrapper
    assert "export CUDA_VISIBLE_DEVICES=$runtime_cuda_visible" in compose_wrapper
    assert "export ELESIM_SIM_VIEWER=$runtime_sim_viewer" in compose_wrapper
    assert "exec docker compose" in compose_wrapper
    assert str(state.bin_path / "elesim-compose") in up_wrapper
    assert "actual_docker_engine_id" in up_wrapper
    assert "sidecar_login_status == 78" in up_wrapper
    assert "${sidecar_backend_state,,}" not in up_wrapper
    assert "sidecar_backend_state_lower=" in up_wrapper
    assert "login --hostname=elesim-deadbeef0123" in tailscale_wrapper
    assert "up --force-reauth --hostname=elesim-deadbeef0123" in tailscale_wrapper
    assert "login [--if-needed]" in tailscale_wrapper
    assert "update)" in tailscale_wrapper
    assert "ps --status running -q" in tailscale_wrapper
    assert "--force-recreate tailscale" in tailscale_wrapper
    assert "공식 stable Tailscale 이미지를 가져오는 중" in tailscale_wrapper
    assert "이미 최신 stable 버전입니다" in tailscale_wrapper
    assert "sidecar 업데이트 완료:" in tailscale_wrapper
    assert "tailscale_runtime_services=(pilot ui runtime-tools)" in tailscale_wrapper
    assert "${login_backend_state,,}" not in tailscale_wrapper
    assert "login_backend_state_lower=" in tailscale_wrapper
    assert "needslogin|nostate" in tailscale_wrapper
    assert "trap login_cleanup EXIT TERM INT" in tailscale_wrapper
    assert "브라우저 로그인을 기다리는 중" in tailscale_wrapper
    assert "last_login_message" in tailscale_wrapper
    assert 'if [[ $last_login_message != "$login_wait_message" ]]' in tailscale_wrapper
    assert 'wait "$login_child"' in tailscale_wrapper
    assert '"BackendState"' in tailscale_wrapper
    assert '"IPv4"' in tailscale_wrapper
    assert "net_service=runtime-tools" in net_wrapper
    assert "namespace-check|doctor" in net_wrapper
    assert "configuration-check|namespace-check|doctor" not in net_wrapper
    assert "elesim-tailscale status --json" in up_wrapper
    assert "elesim-tailscale login" in up_wrapper
    assert (
        up_wrapper.index("elesim-tailscale login")
        < up_wrapper.index("elesim-tailscale status --json")
        < up_wrapper.index("elesim-net configuration-check")
        < up_wrapper.index("elesim-net namespace-check")
    )
    assert not (state.bin_path / "elesim-pilot").exists()
    assert "pull tailscale" not in update_wrapper
    assert "build pilot ui tools" in update_wrapper
    assert "elesim-tailscale login" not in update_wrapper
    # The sidecar update pulls/recreates the rolling stable image. An
    # in-container ``tailscale update`` would be lost on recreation.
    assert "exec -T tailscale tailscale update" not in tailscale_wrapper

    manifest = OwnershipManifest.load(prefix / "install-ownership.json")
    assert manifest.docker is not None
    assert TAILSCALE_CONTAINER_NAME in manifest.docker.containers
    assert TAILSCALE_IMAGE not in manifest.docker.local_images
    assert manifest.docker.context == "default"
    assert manifest.docker.engine_id == "desktop-engine-id"
    for name in ("elesim-compose", "elesim-tailscale", "elesim-update"):
        assert subprocess.run(
            ("bash", "-n", str(state.bin_path / name)),
            check=False,
        ).returncode == 0


def test_container_refresh_removes_manifest_owned_legacy_role_wrappers(
    local_state,
) -> None:
    state = local_state(roles=("pilot", "sim", "ui"))
    installer = ContainerInstaller(state)
    installer.run()

    legacy = state.bin_path / "elesim-pilot"
    legacy.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    legacy.chmod(0o755)
    manifest_path = state.prefix_path / "install-ownership.json"
    payload = json.loads(manifest_path.read_text(encoding="utf-8"))
    payload["wrappers"].append(
        {
            "path": str(legacy.resolve()),
            "sha256": hashlib.sha256(legacy.read_bytes()).hexdigest(),
        }
    )
    manifest_path.write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n",
        encoding="utf-8",
    )

    ContainerInstaller(state).run()

    assert not legacy.exists()
    manifest = OwnershipManifest.load(manifest_path)
    assert str(legacy.resolve()) not in {wrapper.path for wrapper in manifest.wrappers}


def test_sidecar_only_runtime_is_preserved_by_ordinary_down(local_state, tmp_path: Path) -> None:
    prefix = local_state().prefix_path
    state = local_state(
        roles=("ui",),
        container_network=ContainerNetworkSettings(
            mode="tailscale-sidecar",
            docker_context="default",
            docker_engine_id="desktop-engine-id",
            tailscale_hostname="elesim-deadbeef0123",
            tailscale_state_dir=str(prefix / "secrets/tailscale"),
        ),
    )
    ContainerInstaller(state).run()
    fake_bin = tmp_path / "fake-sidecar-docker"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    marker = tmp_path / "down-called"
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "ELESIM_FAKE_ENGINE_ID": "desktop-engine-id",
            "ELESIM_FAKE_RUNTIME_EMPTY": "1",
            "ELESIM_FAKE_INFRA_PRESENT": "1",
            "ELESIM_DOWN_MARKER": str(marker),
        }
    )

    result = subprocess.run(
        (state.bin_path / "elesim-down",),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert not marker.exists()
    assert "Tailscale sidecar는 유지합니다" in result.stderr


def test_sidecar_down_then_up_starts_persisted_identity_before_namespace_check(
    local_state,
    tmp_path: Path,
) -> None:
    prefix = local_state().prefix_path
    state = local_state(
        roles=("ui",),
        container_network=ContainerNetworkSettings(
            mode="tailscale-sidecar",
            docker_context="default",
            docker_engine_id="desktop-engine-id",
            tailscale_hostname="elesim-deadbeef0123",
            tailscale_state_dir=str(prefix / "secrets/tailscale"),
        ),
    )
    ContainerInstaller(state).run()
    fake_bin = tmp_path / "fake-down-up-docker"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    calls = tmp_path / "docker.calls"
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "ELESIM_FAKE_DOCKER_CALLS": str(calls),
            "ELESIM_FAKE_ENGINE_ID": "desktop-engine-id",
            "ELESIM_FAKE_RUNTIME_EMPTY": "1",
            "ELESIM_FAKE_INFRA_PRESENT": "1",
            "ELESIM_FAKE_TAILSCALE_PRESENT": "1",
            "ELESIM_FAKE_TAILSCALE_STATE": "Running",
        }
    )

    stopped = subprocess.run(
        (state.bin_path / "elesim-down",),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    started = subprocess.run(
        (state.bin_path / "elesim-up",),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert stopped.returncode == 0, stopped.stderr
    assert started.returncode == 0, started.stderr
    rendered = calls.read_text(encoding="utf-8")
    login_start = rendered.index("up -d --no-deps tailscale")
    login_status = rendered.index(
        "exec -T tailscale tailscale --socket=/tmp/tailscaled.sock status --json",
        login_start,
    )
    namespace_check = rendered.index(
        "run --rm -T runtime-tools elesim-net", login_status
    )
    runtime_start = rendered.index(
        "up -d --build --remove-orphans ui", namespace_check
    )
    assert login_start < login_status < namespace_check < runtime_start


def test_tailscale_state_directory_rejects_symlink_escape(
    local_state,
    tmp_path: Path,
) -> None:
    prefix = local_state().prefix_path
    state_path = prefix / "secrets/tailscale"
    state_path.parent.mkdir(parents=True)
    external = tmp_path / "external-tailscale-state"
    external.mkdir()
    marker = external / "keep"
    marker.write_text("owned elsewhere\n", encoding="utf-8")
    state_path.symlink_to(external, target_is_directory=True)
    state = local_state(
        roles=("ui",),
        container_network=ContainerNetworkSettings(
            mode="tailscale-sidecar",
            docker_context="default",
            docker_engine_id="desktop-engine-id",
            tailscale_hostname="elesim-symlink-test",
            tailscale_state_dir=str(state_path),
        ),
    )

    with pytest.raises(ValueError, match="실제 directory"):
        ContainerInstaller(state)._prepare_tailscale_state()

    assert marker.read_text(encoding="utf-8") == "owned elsewhere\n"


def test_legacy_unpinned_update_refuses_daemon_without_owned_objects(
    local_state,
    monkeypatch,
) -> None:
    legacy = local_state(roles=("ui",))
    ContainerInstaller(legacy).run()
    prefix = legacy.prefix_path
    pinned = replace(
        legacy,
        container_network=ContainerNetworkSettings(
            mode="tailscale-sidecar",
            docker_context="default",
            docker_engine_id="new-desktop-engine",
            tailscale_hostname="elesim-legacy-proof",
            tailscale_state_dir=str(prefix / "secrets/tailscale"),
        ),
    )
    monkeypatch.setattr(
        "elesim_setup.container_installer.validate_docker_ownership",
        lambda _ownership: ((), ()),
    )

    with pytest.raises(ValueError, match="한 번도 build하지 않은 legacy 설치"):
        ContainerInstaller(pinned).run()


def test_legacy_unpinned_update_adopts_only_exact_labeled_daemon(
    local_state,
    monkeypatch,
) -> None:
    legacy = local_state(roles=("ui",))
    ContainerInstaller(legacy).run()
    prefix = legacy.prefix_path
    pinned = replace(
        legacy,
        container_network=ContainerNetworkSettings(
            mode="tailscale-sidecar",
            docker_context="desktop-linux",
            docker_engine_id="owned-desktop-engine",
            tailscale_hostname="elesim-owned-proof",
            tailscale_state_dir=str(prefix / "secrets/tailscale"),
        ),
    )
    observed = []

    def prove(ownership):
        observed.append(ownership)
        return ((DockerObject(name="elesim-ui", object_id="owned-id"),), ())

    monkeypatch.setattr(
        "elesim_setup.container_installer.validate_docker_ownership",
        prove,
    )

    ContainerInstaller(pinned).run()

    assert len(observed) == 1
    assert observed[0].context == "desktop-linux"
    assert observed[0].engine_id == "owned-desktop-engine"
    manifest = OwnershipManifest.load(prefix / "install-ownership.json")
    assert manifest.docker is not None
    assert manifest.docker.context == "desktop-linux"
    assert manifest.docker.engine_id == "owned-desktop-engine"


def test_legacy_unpinned_update_rejects_foreign_daemon_labels(
    local_state,
    monkeypatch,
) -> None:
    legacy = local_state(roles=("ui",))
    ContainerInstaller(legacy).run()
    prefix = legacy.prefix_path
    pinned = replace(
        legacy,
        container_network=ContainerNetworkSettings(
            mode="tailscale-sidecar",
            docker_context="default",
            docker_engine_id="foreign-engine",
            tailscale_hostname="elesim-foreign-proof",
            tailscale_state_dir=str(prefix / "secrets/tailscale"),
        ),
    )

    def reject(_ownership):
        raise UninstallSafetyError("fixed container belongs to another install")

    monkeypatch.setattr(
        "elesim_setup.container_installer.validate_docker_ownership",
        reject,
    )

    with pytest.raises(ValueError, match="another install"):
        ContainerInstaller(pinned).run()


def test_tailscale_login_retries_starting_then_supports_idempotent_mode_when_running(
    tmp_path: Path,
) -> None:
    calls = tmp_path / "compose.calls"
    status_count = tmp_path / "status.count"
    compose_wrapper = tmp_path / "elesim-compose"
    compose_wrapper.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s\\n' \"$*\" >>\"$ELESIM_TEST_CALLS\"\n"
        "if [[ \" $* \" == *' status --json '* ]]; then\n"
        "  if [[ ! -e $ELESIM_TEST_STATUS_COUNT ]]; then\n"
        "    : >\"$ELESIM_TEST_STATUS_COUNT\"\n"
        "    printf '{\"BackendState\":\"Starting\"}\\n'\n"
        "  else\n"
        "    printf '{\"BackendState\":\"Running\"}\\n'\n"
        "  fi\n"
        "fi\n",
        encoding="utf-8",
    )
    compose_wrapper.chmod(0o755)
    wrapper = tmp_path / "elesim-tailscale"
    wrapper.write_text(
        _tailscale_wrapper(
            compose=tmp_path / "compose.yaml",
            compose_wrapper=compose_wrapper,
            guard="",
            hostname="elesim-idempotent",
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    environment = os.environ.copy()
    environment["ELESIM_TEST_CALLS"] = str(calls)
    environment["ELESIM_TEST_STATUS_COUNT"] = str(status_count)

    result = subprocess.run(
        (wrapper, "login", "--if-needed"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    rendered = calls.read_text(encoding="utf-8")
    assert "up -d --no-deps tailscale" in rendered
    assert rendered.count("status --json") >= 2
    assert " up --force-reauth " not in f" {rendered} "
    assert " login --hostname=" not in f" {rendered} "

    # The explicit operator command must still open a browser/device flow (or
    # reauthenticate a stale Running node) instead of silently accepting the
    # cached local state.
    forced = subprocess.run(
        (wrapper, "login"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert forced.returncode == 0, forced.stderr
    rendered = calls.read_text(encoding="utf-8")
    assert "up --force-reauth --hostname=elesim-idempotent" in rendered


def test_tailscale_login_streams_child_and_preserves_its_status(
    tmp_path: Path,
) -> None:
    compose_wrapper = tmp_path / "elesim-compose"
    compose_wrapper.write_text(
        "#!/usr/bin/env bash\n"
        "if [[ \" $* \" == *' status --json '* ]]; then\n"
        "  printf '{\"BackendState\":\"NeedsLogin\"}\\n'\n"
        "elif [[ \" $* \" == *' login --hostname=elesim-stream '* ]]; then\n"
        "  printf 'https://login.tailscale.example/device\\n'\n"
        "  exit 23\n"
        "fi\n",
        encoding="utf-8",
    )
    compose_wrapper.chmod(0o755)
    wrapper = tmp_path / "elesim-tailscale"
    wrapper.write_text(
        _tailscale_wrapper(
            compose=tmp_path / "compose.yaml",
            compose_wrapper=compose_wrapper,
            guard="",
            hostname="elesim-stream",
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)

    syntax = subprocess.run(
        ("bash", "-n", str(wrapper)),
        capture_output=True,
        text=True,
        check=False,
    )
    result = subprocess.run(
        (wrapper, "login"),
        capture_output=True,
        text=True,
        check=False,
    )

    assert syntax.returncode == 0, syntax.stderr
    assert result.returncode == 23
    assert "https://login.tailscale.example/device" in result.stdout


def test_tailscale_update_recreates_sidecar_and_only_reconnects_running_services(
    tmp_path: Path,
) -> None:
    calls = tmp_path / "compose.calls"
    compose_wrapper = tmp_path / "elesim-compose"
    compose_wrapper.write_text(
        "#!/usr/bin/env bash\n"
        "set -u\n"
        "printf '%s\\n' \"$*\" >>\"$ELESIM_TEST_CALLS\"\n"
        "if [[ ${1:-} == -f ]]; then shift 2; fi\n"
        "if [[ ${1:-} == ps && ${2:-} == --status && ${3:-} == running && ${4:-} == -q ]]; then\n"
        "  case ${5:-} in\n"
        "    tailscale) [[ ${ELESIM_TEST_SIDECAR_RUNNING:-0} == 1 ]] && printf 'sidecar-id\\n' ;;\n"
        "    pilot|coturn)\n"
        "      if [[ \" $ELESIM_TEST_RUNNING_SERVICES \" == *\" ${5} \"* ]]; then printf '%s-id\\n' \"${5}\"; fi\n"
        "      ;;\n"
        "  esac\n"
        "  exit 0\n"
        "fi\n"
        "if [[ ${1:-} == exec && \" $* \" == *' status --json '* ]]; then\n"
        "  printf '{\"BackendState\":\"Running\"}\\n'\n"
        "elif [[ ${1:-} == exec && \" $* \" == *' version '* ]]; then\n"
        "  printf '1.102.3\\n'\n"
        "fi\n",
        encoding="utf-8",
    )
    compose_wrapper.chmod(0o755)
    wrapper = tmp_path / "elesim-tailscale"
    wrapper.write_text(
        _tailscale_wrapper(
            compose=tmp_path / "compose.yaml",
            compose_wrapper=compose_wrapper,
            guard="",
            hostname="elesim-update",
            services=("pilot", "ui", "coturn"),
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)

    environment = os.environ.copy()
    environment["ELESIM_TEST_CALLS"] = str(calls)
    environment["ELESIM_TEST_SIDECAR_RUNNING"] = "1"
    environment["ELESIM_TEST_RUNNING_SERVICES"] = "pilot coturn"
    result = subprocess.run(
        (wrapper, "update"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    rendered = calls.read_text(encoding="utf-8").splitlines()
    assert rendered[0].endswith("ps --status running -q tailscale")
    assert rendered[1].endswith(
        "exec -T tailscale tailscale --socket=/tmp/tailscaled.sock version"
    )
    assert rendered[2].endswith("ps --status running -q pilot")
    assert rendered[3].endswith("ps --status running -q ui")
    assert rendered[4].endswith("ps --status running -q coturn")
    assert rendered[5].endswith("pull tailscale")
    assert rendered[6].endswith("stop pilot coturn")
    assert rendered[7].endswith(
        "up -d --no-build --no-deps --force-recreate tailscale"
    )
    assert rendered[8].endswith(
        "exec -T tailscale tailscale --socket=/tmp/tailscaled.sock status --json"
    )
    assert rendered[9].endswith("up -d --no-build --no-deps pilot coturn")
    assert rendered[10].endswith(
        "exec -T tailscale tailscale --socket=/tmp/tailscaled.sock version"
    )
    assert "이미 최신 stable 버전입니다: 1.102.3" in result.stdout
    assert not any(line.endswith("--no-build --no-deps ui") for line in rendered)


def test_tailscale_update_starts_an_unenrolled_stopped_sidecar_without_roles(
    tmp_path: Path,
) -> None:
    calls = tmp_path / "compose.calls"
    compose_wrapper = tmp_path / "elesim-compose"
    compose_wrapper.write_text(
        "#!/usr/bin/env bash\n"
        "set -u\n"
        "printf '%s\\n' \"$*\" >>\"$ELESIM_TEST_CALLS\"\n"
        "if [[ ${1:-} == -f ]]; then shift 2; fi\n"
        "if [[ ${1:-} == exec && \" $* \" == *' status --json '* ]]; then\n"
        "  printf '{\"BackendState\":\"NeedsLogin\"}\\n'\n"
        "elif [[ ${1:-} == exec && \" $* \" == *' version '* ]]; then\n"
        "  printf '1.102.3\\n'\n"
        "fi\n",
        encoding="utf-8",
    )
    compose_wrapper.chmod(0o755)
    wrapper = tmp_path / "elesim-tailscale"
    wrapper.write_text(
        _tailscale_wrapper(
            compose=tmp_path / "compose.yaml",
            compose_wrapper=compose_wrapper,
            guard="",
            hostname="elesim-update",
            services=("pilot", "ui"),
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    environment = os.environ.copy()
    environment["ELESIM_TEST_CALLS"] = str(calls)
    environment["ELESIM_TEST_RUNNING_SERVICES"] = ""

    result = subprocess.run(
        (wrapper, "update"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    rendered = calls.read_text(encoding="utf-8").splitlines()
    assert rendered[-3].endswith("up -d --no-build --no-deps tailscale")
    assert rendered[-2].endswith(
        "exec -T tailscale tailscale --socket=/tmp/tailscaled.sock status --json"
    )
    assert rendered[-1].endswith(
        "exec -T tailscale tailscale --socket=/tmp/tailscaled.sock version"
    )
    assert "not-running -> 1.102.3" in result.stdout
    assert not any(line.startswith("stop ") for line in rendered)


def test_container_install_falls_back_when_legacy_cache_is_not_writable(
    local_state, monkeypatch
) -> None:
    state = local_state(roles=("sim",), install_mode="container")
    ContainerInstaller(state).run()
    legacy_cache = state.prefix_path / "cache"
    legacy_marker = legacy_cache / "genesis" / "legacy-artifact.bin"
    legacy_marker.write_bytes(b"keep")
    original_chmod = Path.chmod

    def deny_legacy_cache(path: Path, mode: int, *args, **kwargs) -> None:
        if path == legacy_cache or path == legacy_cache / "genesis":
            raise PermissionError("legacy cache belongs to root")
        original_chmod(path, mode, *args, **kwargs)

    monkeypatch.setattr(Path, "chmod", deny_legacy_cache)
    ContainerInstaller(state).run()

    fallback = state.prefix_path / ".runtime-cache"
    assert fallback.is_dir()
    assert (fallback / "genesis").is_dir()
    assert legacy_marker.read_bytes() == b"keep"
    compose = _compose(state)
    assert (
        f"{fallback}:/tmp/elesim-cache:rw"
        in compose["services"]["sim"]["volumes"]
    )
    manifest = json.loads(
        (state.prefix_path / "install-ownership.json").read_text(encoding="utf-8")
    )
    assert str(fallback) in manifest["managed_roots"]


def test_compose_dds_environment_refresh_tracks_configured_state(local_state) -> None:
    state = local_state(
        roles=("pilot", "ui"),
        install_mode="container",
        dds=DdsSettings(
            discovery_mode="static",
            static_peers=("100.74.222.24",),
            interface="eth0",
        ),
    )
    ContainerInstaller(state).run()
    compose_path = state.prefix_path / "containers/compose.yaml"
    compose = _compose(state)
    for role in state.roles:
        compose["services"][role]["environment"]["ELESIM_DDS_NETWORK_INTERFACE"] = (
            "tailscale0"
        )
    compose["services"]["tools"]["environment"]["ELESIM_DDS_NETWORK_INTERFACE"] = (
        "tailscale0"
    )
    compose_path.write_text(yaml.safe_dump(compose, sort_keys=False), encoding="utf-8")

    refresh_compose_dds_environment(state)

    refreshed = _compose(state)
    for role in (*state.roles, "tools"):
        environment = refreshed["services"][role]["environment"]
        assert environment["ELESIM_DDS_NETWORK_INTERFACE"] == "eth0"
        assert environment["ELESIM_DDS_DISCOVERY_MODE"] == "static"
        assert environment["ELESIM_DDS_STATIC_PEERS"] == "100.74.222.24"


def test_compose_refresh_clears_sros2_values_after_switch_to_trusted_network(
    local_state,
) -> None:
    """Switching sros2 -> trusted-network has to clear the SROS2 variables.

    The refresh merges into the generated manifest, so a variable the new
    profile never writes keeps whatever the old one left behind, and
    ``network.require_generated_dds_configuration`` then refuses to start the
    runtime over a stale ROS_SECURITY_STRATEGY.
    """

    state = local_state(roles=("pilot", "ui"), install_mode="container")
    ContainerInstaller(state).run()
    compose_path = state.prefix_path / "containers/compose.yaml"
    compose = _compose(state)
    stale = {
        "ROS_SECURITY_ENABLE": "true",
        "ROS_SECURITY_STRATEGY": "Enforce",
        "ROS_SECURITY_KEYSTORE": "/leftover/keystore",
        "ELESIM_DDS_ENCLAVE": "/leftover",
    }
    for role in (*state.roles, "tools"):
        compose["services"][role]["environment"].update(stale)
    compose_path.write_text(yaml.safe_dump(compose, sort_keys=False), encoding="utf-8")

    refresh_compose_dds_environment(state)

    refreshed = _compose(state)
    for role in (*state.roles, "tools"):
        environment = refreshed["services"][role]["environment"]
        assert environment["ROS_SECURITY_ENABLE"] == "false"
        assert environment["ROS_SECURITY_STRATEGY"] == ""
        assert environment["ROS_SECURITY_KEYSTORE"] == ""
        assert environment["ELESIM_DDS_ENCLAVE"] == ""


def test_runtime_up_view_switch_discovers_remote_x11_session_and_is_one_shot(
    tmp_path: Path,
) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("name: elesim-runtime\nservices: {}\n", encoding="utf-8")
    (tmp_path / "install-state.json").write_text(
        json.dumps({"dds": {"security_profile": "trusted-network"}}),
        encoding="utf-8",
    )
    wrapper = tmp_path / "elesim-up"
    x11_socket_dir = tmp_path / ".X11-unix"
    _create_x11_socket(x11_socket_dir)
    wrapper.write_text(
        _runtime_up_wrapper(
            compose=compose,
            guard="",
            launch_guard="",
            has_sim=True,
            runtime_roles=("pilot", "sim", "ui"),
            state_path=tmp_path / "install-state.json",
            viewer_state=tmp_path / "viewer-xhost",
            viewer_user="simuser",
            viewer_x11_socket_dir=x11_socket_dir,
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s' \"${ELESIM_SIM_VIEWER-UNSET}\" > \"$VIEWER_MARKER\"\n"
        "printf '%s' \"${CUDA_VISIBLE_DEVICES-UNSET}\" > \"$CUDA_MARKER\"\n"
        "printf '%s\\n' \"$*\" > \"$DOCKER_ARGS_MARKER\"\n"
        "printf '%s\\n' \"$*\" >> \"$DOCKER_CALLS_MARKER\"\n"
        "if [[ -n ${FAIL_HEADLESS_UP:-} && -z ${ELESIM_SIM_VIEWER:-} && $* == *' up -d '* ]]; then exit 71; fi\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    xhost = fake_bin / "xhost"
    xhost.write_text(
        "#!/usr/bin/env bash\n"
        "[[ ${DISPLAY:-} == :0 ]] || exit 1\n"
        "[[ ${XAUTHORITY:-} == ${EXPECTED_XAUTHORITY:?} ]] || exit 1\n"
        "if (( $# == 0 )); then\n"
        "  printf 'SI:localuser:simuser-extra\\n'\n"
        "  if [[ -e ${XHOST_PERMISSION_MARKER:?} ]]; then\n"
        "    printf 'SI:localuser:simuser\\n'\n"
        "  fi\n"
        "  exit 0\n"
        "fi\n"
        "case $1 in\n"
        "  +si:localuser:simuser) : >\"${XHOST_PERMISSION_MARKER:?}\";;\n"
        "  -si:localuser:simuser) rm -f -- \"${XHOST_PERMISSION_MARKER:?}\";;\n"
        "esac\n",
        encoding="utf-8",
    )
    xhost.chmod(0o755)

    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
    environment["VIEWER_MARKER"] = str(tmp_path / "viewer.marker")
    environment["CUDA_MARKER"] = str(tmp_path / "cuda.marker")
    environment["DOCKER_ARGS_MARKER"] = str(tmp_path / "docker.args")
    environment["DOCKER_CALLS_MARKER"] = str(tmp_path / "docker.calls")
    environment["XHOST_PERMISSION_MARKER"] = str(tmp_path / "xhost.permission")
    viewer_home = tmp_path / "viewer-home"
    viewer_home.mkdir()
    xauthority = viewer_home / ".Xauthority"
    xauthority.write_text("cookie\n", encoding="utf-8")
    environment["HOME"] = str(viewer_home)
    environment["EXPECTED_XAUTHORITY"] = str(xauthority)
    # An inherited SSH-forwarded display must be ignored because only the
    # local Unix socket is mounted into the Sim container.
    environment["DISPLAY"] = "localhost:10.0"
    environment.pop("XAUTHORITY", None)
    missing_sim = subprocess.run(
        (wrapper, "--view", "pilot"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert missing_sim.returncode == 64
    assert "Sim 서비스" in missing_sim.stderr
    assert not Path(environment["DOCKER_CALLS_MARKER"]).exists()
    discovered_display = subprocess.run(
        (wrapper, "--view"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert discovered_display.returncode == 0
    assert Path(environment["VIEWER_MARKER"]).read_text(encoding="utf-8") == "1"
    assert (tmp_path / "viewer-xhost").read_text(encoding="utf-8") == (
        f":0\n{xauthority}\n"
    )
    first_calls = Path(environment["DOCKER_CALLS_MARKER"]).read_text(
        encoding="utf-8"
    )
    assert "run --rm -T --build --no-deps sim --elesim-viewer-preflight" in first_calls
    assert first_calls.index("--elesim-viewer-preflight") < first_calls.index(
        "up -d --build --remove-orphans"
    )

    environment["DISPLAY"] = ":0"
    environment["XAUTHORITY"] = str(xauthority)
    viewed = subprocess.run(
        (wrapper, "--no-build", "--cuda-visible-devices", "2", "--view"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert viewed.returncode == 0
    assert Path(environment["VIEWER_MARKER"]).read_text(encoding="utf-8") == "1"
    assert Path(environment["CUDA_MARKER"]).read_text(encoding="utf-8") == "2"
    assert "up -d --no-build --remove-orphans" in Path(
        environment["DOCKER_ARGS_MARKER"]
    ).read_text(encoding="utf-8")
    assert "run --rm -T --no-deps sim --elesim-viewer-preflight" in Path(
        environment["DOCKER_CALLS_MARKER"]
    ).read_text(encoding="utf-8")
    assert (tmp_path / "viewer-xhost").read_text(encoding="utf-8") == (
        f":0\n{xauthority}\n"
    )
    assert (tmp_path / "xhost.permission").is_file()

    # The grant belongs to the X server, not to one screen suffix or the
    # authority file that happened to authenticate the host-side xhost call.
    # A failed restart must preserve it for manager compensation.
    replacement_authority = viewer_home / "replacement.Xauthority"
    replacement_authority.write_text("new-cookie\n", encoding="utf-8")
    environment["DISPLAY"] = ":0.0"
    environment["XAUTHORITY"] = str(replacement_authority)
    environment["EXPECTED_XAUTHORITY"] = str(replacement_authority)
    environment["FAIL_VIEWER_PREFLIGHT"] = "1"
    docker.write_text(
        docker.read_text(encoding="utf-8").replace(
            "if [[ -n ${FAIL_HEADLESS_UP:-}",
            "if [[ -n ${FAIL_VIEWER_PREFLIGHT:-} && $* == *--elesim-viewer-preflight* ]]; then exit 69; fi\n"
            "if [[ -n ${FAIL_HEADLESS_UP:-}",
        ),
        encoding="utf-8",
    )
    failed_viewer_restart = subprocess.run(
        (wrapper, "--no-build", "--view"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert failed_viewer_restart.returncode == 69
    assert (tmp_path / "viewer-xhost").is_file()
    assert (tmp_path / "xhost.permission").is_file()
    environment.pop("FAIL_VIEWER_PREFLIGHT")
    environment["DISPLAY"] = ":0"
    environment["XAUTHORITY"] = str(xauthority)
    environment["EXPECTED_XAUTHORITY"] = str(xauthority)

    # Starting an unrelated role must not revoke the ACL of an already
    # running Sim Viewer.
    partial = subprocess.run(
        (wrapper, "--no-build", "pilot"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert partial.returncode == 0
    assert (tmp_path / "viewer-xhost").is_file()
    assert (tmp_path / "xhost.permission").is_file()

    # A failed headless transition likewise preserves the exact grant so the
    # connection manager can resume the previously stopped Viewer container.
    environment["FAIL_HEADLESS_UP"] = "1"
    failed_headless = subprocess.run(
        (wrapper, "--no-build", "sim"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert failed_headless.returncode == 71
    assert (tmp_path / "viewer-xhost").is_file()
    assert (tmp_path / "xhost.permission").is_file()
    environment.pop("FAIL_HEADLESS_UP")

    normal = subprocess.run(
        (wrapper,),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert normal.returncode == 0
    assert Path(environment["VIEWER_MARKER"]).read_text(encoding="utf-8") == ""
    assert not (tmp_path / "viewer-xhost").exists()
    assert not (tmp_path / "xhost.permission").exists()

    uuid_launch = subprocess.run(
        (wrapper, "--no-build", "--cuda-visible-devices", "GPU-fixed-123", "sim"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert uuid_launch.returncode == 0
    assert Path(environment["CUDA_MARKER"]).read_text(encoding="utf-8") == (
        "GPU-fixed-123"
    )

    xauthority.unlink()
    environment.pop("DISPLAY", None)
    environment.pop("XAUTHORITY", None)
    Path(environment["VIEWER_MARKER"]).unlink()
    # The hardened launcher validates the local X11 session before any Docker
    # operation.  Clear the marker from the previous successful launch so this
    # assertion observes only the unavailable-session invocation.
    Path(environment["DOCKER_ARGS_MARKER"]).write_text("", encoding="utf-8")
    unavailable = subprocess.run(
        (wrapper, "--view"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert unavailable.returncode == 64
    assert "X11 세션" in unavailable.stderr
    assert "up -d" not in Path(environment["DOCKER_ARGS_MARKER"]).read_text(
        encoding="utf-8"
    )


def test_runtime_up_view_prefers_same_user_physical_display_over_nx_session(
    tmp_path: Path,
) -> None:
    viewer_account = pwd.getpwuid(os.getuid()).pw_name
    compose = tmp_path / "compose.yaml"
    compose.write_text("name: elesim-runtime\nservices: {}\n", encoding="utf-8")
    state_path = tmp_path / "install-state.json"
    state_path.write_text(
        json.dumps({"dds": {"security_profile": "trusted-network"}}),
        encoding="utf-8",
    )
    x11_socket_dir = tmp_path / ".X11-unix"
    _create_x11_socket(x11_socket_dir, display=1)
    _create_x11_socket(x11_socket_dir, display=2)
    _create_x11_socket(x11_socket_dir, display=1001)
    viewer_authority = tmp_path / "viewer.Xauthority"
    viewer_authority.write_text("cookie\n", encoding="utf-8")
    wrapper = tmp_path / "elesim-up"
    wrapper.write_text(
        _runtime_up_wrapper(
            compose=compose,
            guard="",
            launch_guard="",
            has_sim=True,
            runtime_roles=("sim",),
            state_path=state_path,
            viewer_state=tmp_path / "viewer-xhost",
            viewer_user=viewer_account,
            viewer_x11_socket_dir=x11_socket_dir,
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)

    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s' \"${DISPLAY:-unset}\" >\"${VIEWER_DISPLAY_MARKER:?}\"\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    xhost = fake_bin / "xhost"
    xhost.write_text(
        "#!/usr/bin/env bash\n"
        "case ${DISPLAY:-} in :1|:2|:1001) ;; *) exit 1 ;; esac\n"
        "if (( $# == 0 )); then exit 0; fi\n"
        "case $1 in\n"
        f"  +si:localuser:{viewer_account}) : >\"${{XHOST_PERMISSION_MARKER:?}}\" ;;\n"
        f"  -si:localuser:{viewer_account}) rm -f -- \"${{XHOST_PERMISSION_MARKER:?}}\" ;;\n"
        "esac\n",
        encoding="utf-8",
    )
    xhost.chmod(0o755)
    stat = fake_bin / "stat"
    stat.write_text(
        "#!/usr/bin/env bash\n"
        "for argument in \"$@\"; do\n"
        "  [[ $argument == */.X11-unix/X1 ]] && { printf '999\\n'; exit 0; }\n"
        "done\n"
        "exec /usr/bin/stat \"$@\"\n",
        encoding="utf-8",
    )
    stat.chmod(0o755)
    xrandr = fake_bin / "xrandr"
    xrandr.write_text(
        "#!/usr/bin/env bash\n"
        "case ${DISPLAY:-} in\n"
        "  :1) printf 'Monitors: 1\\n 0: +*DP-1 2560/597x1440/336+0+0 DP-1\\n' ;;\n"
        "  :2) printf 'Monitors: 1\\n 0: +*DP-1 2560/597x1440/336+0+0 DP-1\\n' ;;\n"
        "  :1001) printf 'Monitors: 1\\n 0: +nxoutput0 768/195x576/146+0+0 nxoutput0\\n' ;;\n"
        "  *) exit 1 ;;\n"
        "esac\n",
        encoding="utf-8",
    )
    xrandr.chmod(0o755)

    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            # Start from the NX display.  The detector must choose the
            # same-user physical DP-1 display instead.
            "DISPLAY": ":1001",
            # The authority is deliberately usable against every fake X
            # display.  A foreign socket must still be rejected by ownership.
            "ELESIM_VIEWER_USER": viewer_account,
            "XAUTHORITY": str(viewer_authority),
            "VIEWER_DISPLAY_MARKER": str(tmp_path / "viewer-display"),
            "XHOST_PERMISSION_MARKER": str(tmp_path / "xhost.permission"),
        }
    )
    result = subprocess.run(
        (wrapper, "--no-build", "--view"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert Path(environment["VIEWER_DISPLAY_MARKER"]).read_text(
        encoding="utf-8"
    ) == ":2"
    assert (tmp_path / "viewer-xhost").read_text(encoding="utf-8") == (
        f":2\n{viewer_authority}\n"
    )


def test_runtime_up_view_preflight_failure_revokes_acl_and_never_starts(
    tmp_path: Path,
) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("name: elesim-runtime\nservices: {}\n", encoding="utf-8")
    state_path = tmp_path / "install-state.json"
    state_path.write_text(
        json.dumps({"dds": {"security_profile": "trusted-network"}}),
        encoding="utf-8",
    )
    x11_socket_dir = tmp_path / ".X11-unix"
    _create_x11_socket(x11_socket_dir)
    wrapper = tmp_path / "elesim-up"
    wrapper.write_text(
        _runtime_up_wrapper(
            compose=compose,
            guard="",
            launch_guard="",
            has_sim=True,
            runtime_roles=("sim",),
            state_path=state_path,
            viewer_state=tmp_path / "viewer-xhost",
            viewer_user="simuser",
            viewer_x11_socket_dir=x11_socket_dir,
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        "if [[ $* == *--elesim-viewer-preflight* ]]; then exit 69; fi\n"
        "if [[ $* == *' up -d '* ]]; then : >\"${UP_MARKER:?}\"; fi\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    xhost = fake_bin / "xhost"
    xhost.write_text(
        "#!/usr/bin/env bash\n"
        "[[ ${DISPLAY:-} == :0 ]] || exit 1\n"
        "if (( $# == 0 )); then\n"
        "  [[ -e ${XHOST_PERMISSION_MARKER:?} ]] && printf 'SI:localuser:simuser\\n'\n"
        "  exit 0\n"
        "fi\n"
        "case $1 in\n"
        "  +si:localuser:simuser) : >\"${XHOST_PERMISSION_MARKER:?}\";;\n"
        "  -si:localuser:simuser) rm -f -- \"${XHOST_PERMISSION_MARKER:?}\";;\n"
        "esac\n",
        encoding="utf-8",
    )
    xhost.chmod(0o755)
    viewer_home = tmp_path / "home"
    viewer_home.mkdir()
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "DISPLAY": ":0",
            "HOME": str(viewer_home),
            "UP_MARKER": str(tmp_path / "up.marker"),
            "XHOST_PERMISSION_MARKER": str(tmp_path / "xhost.permission"),
        }
    )
    environment.pop("XAUTHORITY", None)

    result = subprocess.run(
        (wrapper, "--no-build", "--view"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 69
    assert "X11/GL 사전 점검" in result.stderr
    assert not (tmp_path / "up.marker").exists()
    assert not (tmp_path / "viewer-xhost").exists()
    assert not (tmp_path / "xhost.permission").exists()


def test_runtime_up_selects_sim_owned_coturn_from_security_profile(
    tmp_path: Path,
) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("name: elesim-runtime\nservices: {}\n", encoding="utf-8")
    state_path = tmp_path / "install-state.json"
    state_path.write_text(
        json.dumps({"dds": {"security_profile": "trusted-network"}}),
        encoding="utf-8",
    )
    wrapper = tmp_path / "elesim-up"
    wrapper.write_text(
        _runtime_up_wrapper(
            compose=compose,
            guard="",
            launch_guard="",
            has_sim=True,
            runtime_roles=("pilot", "sim", "ui"),
            state_path=state_path,
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s\\n' \"$*\" >> \"${DOCKER_ARGS:?}\"\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
    environment["DOCKER_ARGS"] = str(tmp_path / "docker.args")

    result = subprocess.run(
        (wrapper,),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0
    calls = (tmp_path / "docker.args").read_text(encoding="utf-8").splitlines()
    assert calls[0].endswith("stop coturn")
    assert calls[-1].endswith("up -d --build --remove-orphans pilot sim ui")

    state_path.write_text(
        json.dumps({"dds": {"security_profile": "sros2"}}),
        encoding="utf-8",
    )
    (tmp_path / "docker.args").write_text("", encoding="utf-8")
    result = subprocess.run(
        (wrapper,),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0
    calls = (tmp_path / "docker.args").read_text(encoding="utf-8").splitlines()
    assert calls[-1].endswith("up -d --build --remove-orphans pilot sim ui coturn")

    (tmp_path / "docker.args").write_text("", encoding="utf-8")
    result = subprocess.run(
        (wrapper, "pilot"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0
    calls = (tmp_path / "docker.args").read_text(encoding="utf-8").splitlines()
    assert calls[0].endswith("stop coturn")
    assert calls[-1].endswith("up -d --build --remove-orphans pilot")

    (tmp_path / "docker.args").write_text("", encoding="utf-8")
    result = subprocess.run(
        (wrapper, "sim"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0
    calls = (tmp_path / "docker.args").read_text(encoding="utf-8").splitlines()
    assert calls[-1].endswith("up -d --build --remove-orphans sim coturn")

    (tmp_path / "docker.args").write_text("", encoding="utf-8")
    result = subprocess.run(
        (wrapper, "coturn"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 64
    assert not (tmp_path / "docker.args").read_text(encoding="utf-8").strip()


@pytest.mark.parametrize(
    ("actual_fingerprint", "expected_up_flag"),
    (("a" * 64, "--no-build"), ("stale", "--build")),
)
def test_runtime_up_builds_only_when_runtime_image_fingerprint_is_stale(
    tmp_path: Path,
    actual_fingerprint: str,
    expected_up_flag: str,
) -> None:
    expected_fingerprint = "a" * 64
    wrapper = tmp_path / "elesim-up"
    wrapper.write_text(
        _runtime_up_wrapper(
            compose=tmp_path / "compose.yaml",
            guard="",
            launch_guard="",
            has_sim=False,
            runtime_roles=("pilot",),
            state_path=tmp_path / "install-state.json",
            runtime_install_uuid="01234567-89ab-cdef-0123-456789abcdef",
            runtime_image_fingerprints=(("elesim/pilot:local", expected_fingerprint),),
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s\\n' \"$*\" >>\"${DOCKER_CALLS:?}\"\n"
        "if [[ $* == *build_fingerprint* ]]; then\n"
        "  printf '%s\\n' \"${FAKE_FINGERPRINT:?}\"\n"
        "elif [[ $1 == image && $2 == inspect ]]; then\n"
        "  printf 'old-image-id\\n'\n"
        "fi\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "DOCKER_CALLS": str(tmp_path / "docker.calls"),
            "FAKE_FINGERPRINT": actual_fingerprint,
        }
    )

    result = subprocess.run(
        (wrapper,),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    calls = (tmp_path / "docker.calls").read_text(encoding="utf-8").splitlines()
    up_call = next(call for call in calls if " up -d " in call)
    assert up_call.endswith(
        f"up -d {expected_up_flag} --remove-orphans pilot"
    )
    if expected_up_flag == "--no-build":
        assert not any(" image inspect" in call for call in calls[1:])
    else:
        assert any("image inspect elesim/pilot:local" in call for call in calls)


def test_runtime_up_rejects_missing_or_unknown_security_state(tmp_path: Path) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("name: elesim-runtime\nservices: {}\n", encoding="utf-8")
    state_path = tmp_path / "install-state.json"
    wrapper = tmp_path / "elesim-up"
    wrapper.write_text(
        _runtime_up_wrapper(
            compose=compose,
            guard="",
            launch_guard="",
            has_sim=True,
            runtime_roles=("pilot", "sim", "ui"),
            state_path=state_path,
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    marker = tmp_path / "docker.called"
    docker = fake_bin / "docker"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        f"touch {str(marker)!r}\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"

    missing = subprocess.run(
        (wrapper,), env=environment, text=True, capture_output=True, check=False
    )
    assert missing.returncode == 78
    assert not marker.exists()

    state_path.write_text(
        json.dumps({"dds": {"security_profile": "unexpected"}}), encoding="utf-8"
    )
    unknown = subprocess.run(
        (wrapper,), env=environment, text=True, capture_output=True, check=False
    )
    assert unknown.returncode == 78
    assert not marker.exists()


def test_runtime_down_revokes_owned_xhost_without_inheriting_stale_authority(
    tmp_path: Path,
) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("name: elesim-runtime\nservices: {}\n", encoding="utf-8")
    wrapper = tmp_path / "elesim-down"
    wrapper.write_text(
        _runtime_down_wrapper(
            compose=compose,
            logs_root=tmp_path / "logs",
            services=("sim",),
            archive_enabled=False,
            guard="",
            viewer_state=tmp_path / "viewer-xhost",
            viewer_user="simuser",
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    (tmp_path / "viewer-xhost").write_text(":7\n\n", encoding="utf-8")
    (tmp_path / "viewer-xhost").chmod(0o600)
    tmp_path.chmod(0o700)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    xhost = fake_bin / "xhost"
    xhost.write_text(
        "#!/usr/bin/env bash\n"
        "if [[ $1 == -si:localuser:simuser ]]; then\n"
        "  [[ -z ${XAUTHORITY+x} ]] || exit 19\n"
        "  printf '%s' \"${DISPLAY:?}\" > \"${XHOST_REVOKED:?}\"\n"
        "fi\n",
        encoding="utf-8",
    )
    xhost.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
    environment["XHOST_REVOKED"] = str(tmp_path / "xhost.revoked")
    environment["XAUTHORITY"] = "/stale/caller/authority"
    environment.pop("DISPLAY", None)

    result = subprocess.run(
        (wrapper,),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0
    assert (tmp_path / "xhost.revoked").read_text(encoding="utf-8") == ":7"
    assert not (tmp_path / "viewer-xhost").exists()


def test_runtime_down_purge_removes_only_the_exact_manager_container(
    tmp_path: Path,
) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("name: elesim-runtime\nservices: {}\n", encoding="utf-8")
    wrapper = tmp_path / "elesim-down"
    wrapper.write_text(
        _runtime_down_wrapper(
            compose=compose,
            logs_root=tmp_path / "logs",
            services=("sim",),
            archive_enabled=False,
            guard="",
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "ELESIM_FAKE_MANAGER_PRESENT": "1",
            "ELESIM_DOWN_MARKER": str(tmp_path / "down-called"),
            "ELESIM_MANAGER_PURGED_MARKER": str(tmp_path / "manager-purged"),
        }
    )

    normal = subprocess.run(
        (wrapper,), env=environment, text=True, capture_output=True, check=False
    )
    assert normal.returncode == 0
    assert (tmp_path / "down-called").exists()
    assert not (tmp_path / "manager-purged").exists()

    purged = subprocess.run(
        (wrapper, "--purge"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert purged.returncode == 0
    assert (tmp_path / "manager-purged").exists()


def test_runtime_down_keeps_tailscale_until_purge(tmp_path: Path) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("name: elesim-runtime\nservices: {}\n", encoding="utf-8")
    wrapper = tmp_path / "elesim-down"
    wrapper.write_text(
        _runtime_down_wrapper(
            compose=compose,
            logs_root=tmp_path / "logs",
            services=("pilot", "sim", "ui"),
            archive_enabled=False,
            guard="",
            infrastructure_services=("tailscale",),
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    calls = tmp_path / "docker.calls"
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "ELESIM_FAKE_DOCKER_CALLS": str(calls),
        }
    )

    normal = subprocess.run(
        (wrapper,), env=environment, text=True, capture_output=True, check=False
    )
    normal_calls = calls.read_text(encoding="utf-8")
    assert normal.returncode == 0
    assert "rm -f -s pilot sim ui" in normal_calls
    assert "down --remove-orphans" not in normal_calls

    calls.unlink()
    purged = subprocess.run(
        (wrapper, "--purge"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    purge_calls = calls.read_text(encoding="utf-8")
    assert purged.returncode == 0
    assert "down --remove-orphans" in purge_calls
    assert "rm -f -s pilot sim ui" not in purge_calls


def test_runtime_up_refuses_xhost_before_unwritable_state_is_mutated(
    tmp_path: Path,
) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("name: elesim-runtime\nservices: {}\n", encoding="utf-8")
    (tmp_path / "install-state.json").write_text(
        json.dumps({"dds": {"security_profile": "trusted-network"}}),
        encoding="utf-8",
    )
    blocked_parent = tmp_path / "blocked"
    blocked_parent.write_text("not a directory", encoding="utf-8")
    wrapper = tmp_path / "elesim-up"
    x11_socket_dir = tmp_path / ".X11-unix"
    _create_x11_socket(x11_socket_dir)
    wrapper.write_text(
        _runtime_up_wrapper(
            compose=compose,
            guard="",
            launch_guard="",
            has_sim=True,
            runtime_roles=("pilot", "sim", "ui"),
            state_path=tmp_path / "install-state.json",
            viewer_state=blocked_parent / "viewer-xhost",
            viewer_user="simuser",
            viewer_x11_socket_dir=x11_socket_dir,
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text("#!/usr/bin/env bash\nexit 0\n", encoding="utf-8")
    docker.chmod(0o755)
    xhost = fake_bin / "xhost"
    xhost.write_text(
        "#!/usr/bin/env bash\n"
        "case $1 in\n"
        "  +si:localuser:simuser) : >\"${XHOST_PERMISSION_MARKER:?}\";;\n"
        "  -si:localuser:simuser) rm -f -- \"${XHOST_PERMISSION_MARKER:?}\";;\n"
        "esac\n",
        encoding="utf-8",
    )
    xhost.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
    environment["DISPLAY"] = ":0"
    environment["XHOST_PERMISSION_MARKER"] = str(tmp_path / "xhost.permission")

    result = subprocess.run(
        (wrapper, "--view"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 74
    assert "상태를 기록할 수 없습니다" in result.stderr
    assert not (tmp_path / "xhost.permission").exists()


def test_runtime_up_retains_recovery_state_when_xhost_grant_reports_failure(
    tmp_path: Path,
) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("name: elesim-runtime\nservices: {}\n", encoding="utf-8")
    state_path = tmp_path / "install-state.json"
    state_path.write_text(
        json.dumps({"dds": {"security_profile": "trusted-network"}}),
        encoding="utf-8",
    )
    x11_socket_dir = tmp_path / ".X11-unix"
    _create_x11_socket(x11_socket_dir)
    viewer_state = tmp_path / "cache/viewer-xhost"
    wrapper = tmp_path / "elesim-up"
    wrapper.write_text(
        _runtime_up_wrapper(
            compose=compose,
            guard="",
            launch_guard="",
            has_sim=True,
            runtime_roles=("sim",),
            state_path=state_path,
            viewer_state=viewer_state,
            viewer_user="simuser",
            viewer_x11_socket_dir=x11_socket_dir,
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        ": >\"${DOCKER_CALLED:?}\"\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    xhost = fake_bin / "xhost"
    xhost.write_text(
        "#!/usr/bin/env bash\n"
        "if (( $# == 0 )); then exit 0; fi\n"
        "if [[ $1 == +si:localuser:simuser ]]; then\n"
        "  : >\"${XHOST_PERMISSION_MARKER:?}\"\n"
        "  exit 17\n"
        "fi\n",
        encoding="utf-8",
    )
    xhost.chmod(0o755)
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "DISPLAY": ":0",
            "HOME": str(tmp_path / "home"),
            "DOCKER_CALLED": str(tmp_path / "docker.called"),
            "XHOST_PERMISSION_MARKER": str(tmp_path / "xhost.permission"),
        }
    )
    environment.pop("XAUTHORITY", None)

    result = subprocess.run(
        (wrapper, "--view"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 64
    assert viewer_state.read_text(encoding="utf-8") == ":0\n\n"
    assert viewer_state.stat().st_mode & 0o777 == 0o600
    assert (tmp_path / "xhost.permission").is_file()
    assert not (tmp_path / "docker.called").exists()


def test_viewer_cleanup_finds_fixed_fallback_after_cache_becomes_writable(
    tmp_path: Path,
) -> None:
    canonical = tmp_path / "cache/viewer-xhost"
    fallback = tmp_path / ".runtime-cache/viewer-xhost"
    fallback.parent.mkdir()
    fallback.parent.chmod(0o700)
    fallback.write_text(":4\n\n", encoding="utf-8")
    fallback.chmod(0o600)
    canonical.parent.mkdir()
    wrapper = tmp_path / "elesim-viewer-cleanup"
    wrapper.write_text(
        _viewer_cleanup_wrapper(canonical, xhost_user="simuser"),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    xhost = fake_bin / "xhost"
    xhost.write_text(
        "#!/usr/bin/env bash\n"
        "[[ ${DISPLAY:-} == :4 ]] || exit 18\n"
        "[[ -z ${XAUTHORITY+x} ]] || exit 19\n"
        "[[ $1 == -si:localuser:simuser ]] || exit 20\n"
        ": >\"${REVOKED:?}\"\n",
        encoding="utf-8",
    )
    xhost.chmod(0o755)
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "XDG_RUNTIME_DIR": str(tmp_path / "different-runtime-dir"),
            "XAUTHORITY": "/stale/caller/authority",
            "REVOKED": str(tmp_path / "revoked"),
        }
    )

    result = subprocess.run(
        (wrapper,),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert (tmp_path / "revoked").is_file()
    assert not fallback.exists()
    assert not canonical.exists()


def test_viewer_cleanup_revokes_all_current_and_legacy_recovery_records(
    tmp_path: Path,
) -> None:
    canonical = tmp_path / "install/cache/viewer-xhost"
    fallback = tmp_path / "install/.runtime-cache/viewer-xhost"
    runtime_dir = tmp_path / "runtime"
    legacy_name = (
        "viewer-xhost-"
        + hashlib.sha256(str(canonical).encode("utf-8")).hexdigest()[:16]
    )
    legacy = runtime_dir / "elesim" / legacy_name
    for state, display in (
        (canonical, ":1"),
        (fallback, ":2"),
        (legacy, ":3"),
    ):
        state.parent.mkdir(parents=True, exist_ok=True)
        state.parent.chmod(0o700)
        state.write_text(f"{display}\n\n", encoding="utf-8")
        state.chmod(0o600)
    wrapper = tmp_path / "elesim-viewer-cleanup"
    rendered = _viewer_cleanup_wrapper(canonical, xhost_user="simuser")
    assert f"viewer_xhost_legacy_tmp=/tmp/elesim/{legacy_name}" in rendered
    wrapper.write_text(rendered, encoding="utf-8")
    wrapper.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    xhost = fake_bin / "xhost"
    xhost.write_text(
        "#!/usr/bin/env bash\n"
        "[[ $1 == -si:localuser:simuser ]] || exit 20\n"
        "printf '%s\\n' \"${DISPLAY:?}\" >>\"${REVOKED:?}\"\n",
        encoding="utf-8",
    )
    xhost.chmod(0o755)
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "XDG_RUNTIME_DIR": str(runtime_dir),
            "REVOKED": str(tmp_path / "revoked"),
        }
    )

    result = subprocess.run(
        (wrapper,),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert (tmp_path / "revoked").read_text(encoding="utf-8").splitlines() == [
        ":1",
        ":2",
        ":3",
    ]
    assert not canonical.exists()
    assert not fallback.exists()
    assert not legacy.exists()


def test_viewer_cleanup_rejects_unsafe_legacy_recovery_provenance(
    tmp_path: Path,
) -> None:
    canonical = tmp_path / "install/cache/viewer-xhost"
    runtime_dir = tmp_path / "runtime"
    legacy_name = (
        "viewer-xhost-"
        + hashlib.sha256(str(canonical).encode("utf-8")).hexdigest()[:16]
    )
    legacy = runtime_dir / "elesim" / legacy_name
    legacy.parent.mkdir(parents=True)
    legacy.parent.chmod(0o755)
    legacy.write_text(":5\n\n", encoding="utf-8")
    legacy.chmod(0o600)
    wrapper = tmp_path / "elesim-viewer-cleanup"
    wrapper.write_text(
        _viewer_cleanup_wrapper(canonical, xhost_user="simuser"),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    xhost = fake_bin / "xhost"
    xhost.write_text(
        "#!/usr/bin/env bash\n: >\"${XHOST_CALLED:?}\"\n",
        encoding="utf-8",
    )
    xhost.chmod(0o755)
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "XDG_RUNTIME_DIR": str(runtime_dir),
            "XHOST_CALLED": str(tmp_path / "xhost.called"),
        }
    )

    result = subprocess.run(
        (wrapper,),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 74
    assert "소유자/권한이 안전하지 않습니다" in result.stderr
    assert legacy.is_file()
    assert not (tmp_path / "xhost.called").exists()


def test_container_net_wrapper_keeps_json_stdout_clean(local_state, tmp_path: Path) -> None:
    state = local_state(roles=("sim",), install_mode="container")
    ContainerInstaller(state).run()

    fake_bin = tmp_path / "fake-docker"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"

    result = subprocess.run(
        (state.bin_path / "elesim-net", "show"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout) == {"schema_version": 1}
    assert "build progress" not in result.stdout
    wrapper = (state.bin_path / "elesim-net").read_text(encoding="utf-8")
    assert "build --quiet tools >/dev/null" in wrapper
    assert "net_service=tools" in wrapper
    assert 'run --rm -T "$net_service" elesim-net' in wrapper
    assert "run --rm --build tools elesim-net" not in wrapper


def test_container_net_doctor_reuses_tools_image_after_runtime_build(
    local_state, tmp_path: Path
) -> None:
    state = local_state(roles=("sim",), install_mode="container")
    ContainerInstaller(state).run()

    fake_bin = tmp_path / "fake-docker"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    calls = tmp_path / "docker.calls"
    tools_fingerprint = _compose(state)["services"]["tools"]["build"]["labels"][
        DOCKER_BUILD_FINGERPRINT_LABEL
    ]
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "ELESIM_FAKE_DOCKER_CALLS": str(calls),
            "ELESIM_FAKE_TOOLS_FINGERPRINT": tools_fingerprint,
        }
    )

    result = subprocess.run(
        (
            state.bin_path / "elesim-net",
            "doctor",
            "--timeout",
            "300",
            "--json",
        ),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 0, result.stderr
    assert json.loads(result.stdout) == {"schema_version": 1}
    commands = calls.read_text(encoding="utf-8").splitlines()
    assert any("image inspect elesim/tools:local" in command for command in commands)
    assert not any("build --quiet tools" in command for command in commands)


def test_container_install_records_host_uninstaller_and_docker_uuid(
    local_state,
) -> None:
    state = local_state(
        roles=("pilot", "ui"),
        install_mode="container",
    )

    ContainerInstaller(state).run()

    manifest_path = state.prefix_path / "install-ownership.json"
    manifest = OwnershipManifest.load(manifest_path)
    compose = _compose(state)
    assert manifest.path == manifest_path
    assert manifest.docker is not None
    assert manifest.docker.install_uuid == manifest.install_uuid
    assert manifest.docker.project == compose["name"] == "elesim-runtime"
    assert manifest.docker.compose_file == str(
        state.prefix_path / "containers/compose.yaml"
    )
    assert set(manifest.docker.containers) == {
        "elesim-pilot",
        "elesim-ui",
        "elesim-manager",
    }
    assert set(manifest.docker.local_images) == {
        "elesim/pilot:local",
        "elesim/ui:local",
        "elesim/tools:local",
    }
    for service in compose["services"].values():
        assert service["labels"][DOCKER_INSTALL_UUID_LABEL] == manifest.install_uuid
        if "build" in service:
            assert (
                service["build"]["labels"][DOCKER_INSTALL_UUID_LABEL]
                == manifest.install_uuid
            )
            fingerprint = service["build"]["labels"][DOCKER_BUILD_FINGERPRINT_LABEL]
            assert len(fingerprint) == 64
            assert all(character in "0123456789abcdef" for character in fingerprint)

    wrapper = (state.bin_path / "elesim-uninstall").read_text(encoding="utf-8")
    assert "exec python3 -B -S -m elesim_setup.uninstall" in wrapper
    assert f"--manifest {manifest_path}" in wrapper
    assert "export PYTHONNOUSERSITE=1" in wrapper
    assert "docker compose" not in wrapper


def test_static_discovery_is_exported_to_every_service(local_state) -> None:
    state = local_state(
        roles=("pilot", "ui"),
        install_mode="container",
        dds=DdsSettings(
            discovery_mode="static",
            static_peers=("192.0.2.10", "192.0.2.11"),
            interface="eth1",
        ),
    )

    ContainerInstaller(state).run()

    compose = _compose(state)
    for name in ("pilot", "ui", "tools"):
        service = compose["services"][name]
        environment = service["environment"]
        assert environment["ELESIM_DDS_DISCOVERY_MODE"] == "static"
        assert environment["ELESIM_DDS_STATIC_PEERS"] == "192.0.2.10,192.0.2.11"
        assert environment["ELESIM_DDS_NETWORK_INTERFACE"] == "eth1"
    up_wrapper = (state.bin_path / "elesim-up").read_text(encoding="utf-8")
    assert "namespace-check >/dev/null" in up_wrapper


def test_managed_coturn_is_owned_by_sim_and_shares_only_turn_secret(
    local_state,
    tmp_path: Path,
) -> None:
    secret = tmp_path / "install/secrets/turn.secret"
    keystore = tmp_path / "sros2"
    (keystore / "public").mkdir(parents=True)
    (keystore / "enclaves/elesim/sim_default").mkdir(parents=True)
    state = local_state(
        roles=("sim",),
        install_mode="container",
        network=NetworkSettings(
            turn_urls=("turn:turn.example.com:3478?transport=udp",),
        ),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="external",
            keystore=str(keystore),
            enclave="/elesim",
        ),
        turn=TurnSettings(
            mode="managed",
            realm="elesim.local",
            public_host="turn.example.com",
            secret_file=str(secret),
        ),
    )

    ContainerInstaller(state).run()
    compose = _compose(state)

    assert secret.is_file()
    assert secret.stat().st_mode & 0o777 == 0o600
    assert compose["services"]["coturn"]["depends_on"] == ["sim"]
    assert compose["services"]["coturn"]["container_name"] == "elesim-coturn"
    assert compose["services"]["coturn"]["user"] == f"{os.getuid()}:{os.getgid()}"
    assert compose["services"]["coturn"]["logging"] == {
        "driver": "json-file",
        "options": {"max-size": "10m", "max-file": "4"},
    }
    assert f"{secret}:/run/secrets/turn.secret:ro" in (
        compose["services"]["coturn"]["volumes"]
    )
    assert f"{secret}:/run/secrets/turn.secret:ro" in (
        compose["services"]["sim"]["volumes"]
    )
    command = compose["services"]["coturn"]["command"]
    assert isinstance(command, list) and len(command) == 1
    assert "--no-cli" not in command[0]
    assert 'secret="$$(cat /run/secrets/turn.secret)"' in command[0]
    assert 'test -n "$$secret"' in command[0]
    assert '--static-auth-secret="$$secret"' in command[0]
    assert "router" not in str(compose).lower()

    fake_bin = tmp_path / "fake-docker"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
    saved = subprocess.run(
        (state.bin_path / "elesim-logs", "--save"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert saved.returncode == 0
    run = next((state.prefix_path / "logs/runs").iterdir())
    assert (run / "sim.log").read_text(encoding="utf-8") == (
        "saved log for sim\n"
    )
    assert (run / "coturn.log").read_text(encoding="utf-8") == (
        "saved log for coturn\n"
    )
    unsupported = subprocess.run(
        (state.bin_path / "elesim-logs", "--tail", "10"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert unsupported.returncode == 64
    assert "elesim-logs [--save]" in unsupported.stderr


def test_managed_coturn_symlinked_secret_fails_at_unowned_install_boundary(
    local_state,
    tmp_path: Path,
) -> None:
    target = tmp_path / "outside.secret"
    target.write_text("secret\n", encoding="utf-8")
    secret = tmp_path / "install/secrets/turn.secret"
    secret.parent.mkdir(parents=True)
    secret.symlink_to(target)
    state = local_state(
        roles=("sim",),
        install_mode="container",
        network=NetworkSettings(
            turn_urls=("turn:turn.example.com:3478?transport=udp",),
        ),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="external",
            keystore=str(tmp_path / "sros2"),
            enclave="/elesim",
        ),
        turn=TurnSettings(
            mode="managed",
            realm="elesim.local",
            public_host="turn.example.com",
            secret_file=str(secret),
        ),
    )

    with pytest.raises(
        OwnershipError,
        match="ownership manifest 없는 기존 EleSim 후보 경로",
    ):
        ContainerInstaller(state).run()

    assert target.read_text(encoding="utf-8") == "secret\n"


def test_pending_managed_sros2_installs_coturn_but_refuses_application_start(
    local_state,
    tmp_path: Path,
) -> None:
    secret = tmp_path / "install/secrets/turn.secret"
    state = local_state(
        roles=("sim",),
        install_mode="container",
        network=NetworkSettings(
            turn_urls=("turn:turn.example.com:3478?transport=udp",),
        ),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="managed",
        ),
        turn=TurnSettings(
            mode="managed",
            realm="elesim.local",
            public_host="turn.example.com",
            secret_file=str(secret),
        ),
    )

    ContainerInstaller(state).run()
    compose = _compose(state)
    marker = state.prefix_path / "security/provisioning-required"

    assert "coturn" in compose["services"]
    assert compose["services"]["sim"]["environment"][
        "ROS_SECURITY_KEYSTORE"
    ] == str(state.prefix_path / "security/apps/sim")
    assert marker.is_file()
    assert marker.stat().st_mode & 0o777 == 0o600

    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text("#!/usr/bin/env bash\nexit 1\n", encoding="utf-8")
    docker.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
    result = subprocess.run(
        (state.bin_path / "elesim-up",),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 78
    assert "elesim-connections" in result.stderr


def test_managed_coturn_rejects_empty_existing_secret(
    local_state,
    tmp_path: Path,
) -> None:
    secret = tmp_path / "turn.secret"
    secret.write_text("  \n", encoding="utf-8")
    state = local_state(
        roles=("sim",),
        install_mode="container",
        network=NetworkSettings(
            turn_urls=("turn:turn.example.com:3478?transport=udp",),
        ),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="managed",
        ),
        turn=TurnSettings(
            mode="managed",
            realm="elesim.local",
            public_host="turn.example.com",
            secret_file=str(secret),
        ),
    )

    with pytest.raises(ValueError, match="1..4096"):
        ContainerInstaller(state).run()


def test_external_turn_credentials_are_mounted_only_into_sim(
    local_state,
    tmp_path: Path,
) -> None:
    credentials = tmp_path / "turn.credentials.json"
    credentials.write_text(
        '{"username":"lab-user","credential":"lab-password"}\n',
        encoding="utf-8",
    )
    state = local_state(
        roles=("sim", "pilot", "ui"),
        install_mode="container",
        network=NetworkSettings(
            turn_urls=("turn:relay.example.com:3478?transport=udp",),
        ),
        turn=TurnSettings(
            mode="external",
            credential_file=str(credentials),
        ),
    )

    ContainerInstaller(state).run()
    compose = _compose(state)
    mount = f"{credentials}:/run/secrets/turn.credentials.json:ro"

    assert credentials.stat().st_mode & 0o777 == 0o600
    assert mount in compose["services"]["sim"]["volumes"]
    assert mount not in compose["services"]["pilot"]["volumes"]
    assert mount not in compose["services"]["ui"]["volumes"]
    assert "coturn" not in compose["services"]


def test_sros2_environment_and_keystore_mount_are_role_specific(
    local_state,
    tmp_path: Path,
) -> None:
    keystore = tmp_path / "sros2"
    (keystore / "public").mkdir(parents=True)
    (keystore / "enclaves/prod/pilot_main").mkdir(parents=True)
    (keystore / "enclaves/prod/ui_main").mkdir(parents=True)
    (keystore / "public/ca.cert.pem").write_text("ca", encoding="utf-8")
    (keystore / "public/identity_ca.cert.pem").symlink_to("ca.cert.pem")
    (keystore / "enclaves/governance.p7s").write_text(
        "governance", encoding="utf-8"
    )
    (keystore / "enclaves/prod/pilot_main/key.pem").write_text(
        "pilot-key", encoding="utf-8"
    )
    (keystore / "enclaves/prod/ui_main/key.pem").write_text(
        "ui-key", encoding="utf-8"
    )
    for role in ("pilot_main", "ui_main"):
        enclave = keystore / "enclaves/prod" / role
        (enclave / "governance.p7s").symlink_to(
            os.path.relpath(keystore / "enclaves/governance.p7s", enclave)
        )
    state = local_state(
        roles=("pilot", "ui"),
        install_mode="container",
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="external",
            keystore=str(keystore),
            enclave="/prod",
        ),
    )

    ContainerInstaller(state).run()
    services = _compose(state)["services"]

    assert "ROS_SECURITY_ENCLAVE_OVERRIDE" not in services["pilot"]["environment"]
    assert "ROS_SECURITY_ENCLAVE_OVERRIDE" not in services["ui"]["environment"]
    assert services["tools"]["environment"]["ROS_SECURITY_ENCLAVE_OVERRIDE"] == (
        "/prod/pilot_main"
    )
    pilot_view = state.prefix_path / "security/apps/pilot"
    ui_view = state.prefix_path / "security/apps/ui"
    assert services["pilot"]["environment"]["ROS_SECURITY_KEYSTORE"] == str(
        pilot_view
    )
    assert f"{pilot_view}:{pilot_view}:ro" in services["pilot"][
        "volumes"
    ]
    assert f"{ui_view}:{ui_view}:ro" not in services["pilot"]["volumes"]
    assert not any(
        volume.startswith(f"{keystore}:")
        for volume in services["pilot"]["volumes"]
    )
    assert (pilot_view / "enclaves/prod/pilot_main/key.pem").is_file()
    assert not (pilot_view / "public/identity_ca.cert.pem").is_symlink()
    assert not (
        pilot_view / "enclaves/prod/pilot_main/governance.p7s"
    ).is_symlink()
    assert not (pilot_view / "enclaves/prod/ui_main").exists()
    assert (ui_view / "enclaves/prod/ui_main/key.pem").is_file()
    assert not (ui_view / "enclaves/prod/pilot_main").exists()
    assert not (state.prefix_path / "security/provisioning-required").exists()


def test_specific_gpu_uses_one_compose_device_reservation(local_state) -> None:
    from elesim_setup.state import ComputeSettings

    state = local_state(
        roles=("sim",),
        install_mode="container",
        compute=ComputeSettings(gpu_mode="specific", gpu_device="GPU-abc"),
    )

    ContainerInstaller(state).run()
    service = _compose(state)["services"]["sim"]

    device = service["deploy"]["resources"]["reservations"]["devices"][0]
    assert device["device_ids"] == ["GPU-abc"]
    assert "CUDA_VISIBLE_DEVICES" not in service["environment"]
    manager_wrapper = (state.bin_path / "elesim-connections").read_text(
        encoding="utf-8"
    )
    assert "ELESIM_INSTALL_GPU_MODE=specific" in manager_wrapper


def test_container_update_falls_back_from_unwritable_legacy_context(
    local_state,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    state = local_state(roles=("sim",), install_mode="container")
    installer = ContainerInstaller(state)
    legacy = state.prefix_path / "containers/build"
    blocked = legacy / "sim"
    blocked.mkdir(parents=True)
    fallback = state.prefix_path / "containers/.runtime-build"
    real_access = os.access

    def fake_access(path, mode, **kwargs):
        if Path(path) == blocked:
            return False
        return real_access(path, mode, **kwargs)

    monkeypatch.setattr(os, "access", fake_access)
    assert installer._prepare_build_root() == fallback
    assert blocked.is_dir()
    assert not fallback.is_symlink()


def test_generated_context_rejects_symlink_instead_of_following_it(tmp_path: Path) -> None:
    target = tmp_path / "target"
    target.mkdir()
    link = tmp_path / "context"
    link.symlink_to(target, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        _reset_generated_context(link)


def test_runtime_up_rejects_a_different_install_owner(tmp_path: Path) -> None:
    wrapper = tmp_path / "elesim-up"
    wrapper.write_text(
        _runtime_up_wrapper(
            compose=tmp_path / "compose.yaml",
            guard="",
            launch_guard="",
            has_sim=False,
            runtime_roles=("pilot",),
            state_path=tmp_path / "install-state.json",
            runtime_uid=os.getuid() + 1,
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    result = subprocess.run(
        (wrapper, "--no-build"),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 77
    assert "UID" in result.stderr


@pytest.mark.parametrize("gpu_mode", ["cpu", "specific"])
def test_runtime_up_rejects_runtime_cuda_override_for_fixed_gpu_mode(
    tmp_path: Path,
    gpu_mode: str,
) -> None:
    wrapper = tmp_path / "elesim-up"
    wrapper.write_text(
        _runtime_up_wrapper(
            compose=tmp_path / "compose.yaml",
            guard="",
            launch_guard="",
            has_sim=False,
            runtime_roles=("pilot",),
            state_path=tmp_path / "install-state.json",
            runtime_uid=os.getuid(),
            runtime_gpu_mode=gpu_mode,
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    result = subprocess.run(
        (wrapper, "--no-build", "--cuda-visible-devices", "0"),
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 64
    assert "CUDA_VISIBLE_DEVICES" in result.stderr


def test_container_dry_run_does_not_write_prefix(local_state) -> None:
    state = local_state(install_mode="container")

    ContainerInstaller(state, dry_run=True).run()

    assert not state.prefix_path.exists()


def test_runtime_logs_no_argument_follows_and_save_archives_each_service(
    local_state,
    tmp_path: Path,
) -> None:
    state = local_state(
        roles=("pilot", "ui"),
        install_mode="container",
    )
    ContainerInstaller(state).run()
    fake_bin = tmp_path / "fake-docker"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
    wrapper = state.bin_path / "elesim-logs"

    follow = subprocess.run(
        (wrapper,),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert follow.returncode == 0
    assert "live follow" in follow.stdout
    assert not (state.prefix_path / "logs").exists()

    saved = subprocess.run(
        (wrapper, "--save"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert saved.returncode == 0
    runs = tuple((state.prefix_path / "logs/runs").iterdir())
    assert len(runs) == 1
    run = runs[0]
    assert run.stat().st_mode & 0o777 == 0o700
    assert (state.prefix_path / "logs").stat().st_mode & 0o777 == 0o700
    for service in ("pilot", "ui"):
        log = run / f"{service}.log"
        assert log.read_text(encoding="utf-8") == f"saved log for {service}\n"
        assert log.stat().st_mode & 0o777 == 0o600


def test_runtime_logs_and_down_explain_an_already_stopped_runtime(
    local_state,
    tmp_path: Path,
) -> None:
    state = local_state(roles=("ui",), install_mode="container")
    ContainerInstaller(state).run()
    fake_bin = tmp_path / "fake-docker"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    marker = tmp_path / "down-called"
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "ELESIM_FAKE_RUNTIME_EMPTY": "1",
            "ELESIM_DOWN_MARKER": str(marker),
        }
    )

    logs = subprocess.run(
        (state.bin_path / "elesim-logs",),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    down = subprocess.run(
        (state.bin_path / "elesim-down",),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert logs.returncode == 3
    assert "먼저 elesim-up" in logs.stderr
    assert down.returncode == 0
    assert "이미 정지" in down.stderr
    assert not marker.exists()


def test_runtime_log_archive_keeps_only_five_latest_runs(
    local_state,
    tmp_path: Path,
) -> None:
    state = local_state(roles=("ui",), install_mode="container")
    ContainerInstaller(state).run()
    fake_bin = tmp_path / "fake-docker"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"

    for _index in range(6):
        result = subprocess.run(
            (state.bin_path / "elesim-logs", "--save"),
            env=environment,
            text=True,
            capture_output=True,
            check=False,
        )
        assert result.returncode == 0

    runs = sorted((state.prefix_path / "logs/runs").iterdir())
    assert len(runs) == 5
    assert all((run / "ui.log").is_file() for run in runs)


def test_runtime_log_failure_does_not_prevent_down_and_returns_nonzero(
    local_state,
    tmp_path: Path,
) -> None:
    state = local_state(
        roles=("pilot", "ui"),
        install_mode="container",
    )
    ContainerInstaller(state).run()
    fake_bin = tmp_path / "fake-docker"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    marker = tmp_path / "down-called"
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "ELESIM_FAIL_LOG_SERVICE": "pilot",
            "ELESIM_DOWN_MARKER": str(marker),
        }
    )

    result = subprocess.run(
        (state.bin_path / "elesim-down",),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 74
    assert marker.is_file()
    assert "pilot" in result.stderr
    run = next((state.prefix_path / "logs/runs").iterdir())
    assert (run / "pilot.log").is_file()
    assert (run / "ui.log").is_file()


def test_runtime_log_archive_rejects_a_symlinked_install_ancestor_before_write(
    local_state,
    tmp_path: Path,
) -> None:
    state = local_state(roles=("ui",), install_mode="container")
    ContainerInstaller(state).run()
    fake_bin = tmp_path / "fake-docker"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    original = tmp_path / "original-install"
    outside = tmp_path / "outside"
    outside.mkdir()
    state.prefix_path.rename(original)
    state.prefix_path.symlink_to(outside, target_is_directory=True)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"

    result = subprocess.run(
        (state.bin_path / "elesim-logs", "--save"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 74
    assert "symlink" in result.stderr
    assert not (outside / "logs").exists()


def test_disabled_runtime_archive_preserves_follow_and_down_behavior(
    local_state,
    tmp_path: Path,
) -> None:
    state = replace(
        local_state(roles=("ui",), install_mode="container"),
        runtime_text_logs=RuntimeTextLogSettings(enabled=False),
    )
    ContainerInstaller(state).run()
    fake_bin = tmp_path / "fake-docker"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    marker = tmp_path / "down-called"
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "ELESIM_DOWN_MARKER": str(marker),
        }
    )

    disabled_save = subprocess.run(
        (state.bin_path / "elesim-logs", "--save"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    down = subprocess.run(
        (state.bin_path / "elesim-down",),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert disabled_save.returncode == 64
    assert "비활성화" in disabled_save.stderr
    assert down.returncode == 0
    assert marker.is_file()
    assert not (state.prefix_path / "logs").exists()


def test_runtime_wrapper_rejects_a_container_owned_by_another_install(
    local_state,
    tmp_path: Path,
) -> None:
    state = local_state(
        roles=("ui",),
        install_mode="container",
    )
    ContainerInstaller(state).run()
    fake_bin = tmp_path / "fake-bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        "if [[ $1 == container && $2 == inspect ]]; then\n"
        "  if [[ ${3:-} == --format ]]; then\n"
        "    printf '%s\\n' \"$ELESIM_FAKE_METADATA\"\n"
        "    exit 0\n"
        "  fi\n"
        "  [[ ${3:-} == \"$ELESIM_FAKE_CONTAINER\" ]]\n"
        "  exit\n"
        "fi\n"
        "exit 0\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    environment = os.environ.copy()
    environment.update(
        {
            "PATH": f"{fake_bin}:{environment['PATH']}",
            "ELESIM_FAKE_CONTAINER": "elesim-sim",
            "ELESIM_FAKE_METADATA": "elesim-runtime|/other/compose.yaml",
        }
    )

    result = subprocess.run(
        (state.bin_path / "elesim-up",),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 73
    assert "elesim-sim" in result.stderr
    assert "기존 설치의 elesim-down" in result.stderr
