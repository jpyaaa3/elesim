from __future__ import annotations

import json
import os
import pwd
import subprocess
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from elesim_setup.container_installer import (
    ContainerInstaller,
    _runtime_up_wrapper,
    _runtime_down_wrapper,
    build_container_plan,
    refresh_compose_dds_environment,
)
from elesim_setup.ownership import (
    DOCKER_INSTALL_UUID_LABEL,
    OwnershipManifest,
)
from elesim_setup.state import (
    DdsSettings,
    NetworkSettings,
    RuntimeTextLogSettings,
    TurnSettings,
)


def _compose(state) -> dict:
    return yaml.safe_load(
        (state.prefix_path / "containers/compose.yaml").read_text(encoding="utf-8")
    )


def _fake_docker(path: Path) -> Path:
    docker = path / "docker"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        "set -u\n"
        "if [[ ${1:-} == container && ${2:-} == inspect ]]; then\n"
        "  exit 1\n"
        "fi\n"
        "arguments=\" $* \"\n"
        "if [[ $arguments == *' build --quiet tools '* ]]; then\n"
        "  printf 'build progress that must not reach stdout\\n'\n"
        "  exit 0\n"
        "fi\n"
        "if [[ $arguments == *' run --rm -T tools elesim-net '* ]]; then\n"
        "  printf '{\"schema_version\":1}\\n'\n"
        "  exit 0\n"
        "fi\n"
        "if [[ $arguments == *' ps -aq '* ]]; then\n"
        "  if [[ ${ELESIM_FAKE_RUNTIME_EMPTY:-0} == 1 ]]; then\n"
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


def test_container_install_generates_ros_overlay_contexts_and_dds_environment(
    local_state,
) -> None:
    state = local_state(
        roles=("sim", "pilot", "ui"),
        install_mode="container",
    )

    ContainerInstaller(state).run()
    compose = _compose(state)

    cache_root = state.prefix_path / "cache"
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
            assert (
                f"{state.prefix_path / 'cache/genesis'}:/tmp/elesim-cache/genesis:rw"
                in service["volumes"]
            )
        assert "depends_on" not in service
        role_keystore = state.prefix_path / "security/roles" / role
        security_mount = (
            f"{role_keystore}:"
            f"{role_keystore}:ro"
        )
        assert security_mount in service["volumes"]
        assert all(
            f"/security/roles/{other}:" not in volume
            for other in state.roles
            if other != role
            for volume in service["volumes"]
        )
        assert role_keystore.is_dir()
        assert role_keystore.stat().st_mode & 0o777 == 0o700
        context = state.prefix_path / f"containers/build/{role}"
        assert (context / "interfaces/elesim_interfaces/package.xml").is_file()
        entrypoint = (context / "entrypoint").read_text(encoding="utf-8")
        assert "set +u\nsource /opt/ros/humble/setup.bash" in entrypoint
        assert "source /opt/elesim/ros/install/setup.bash\nset -u" in entrypoint
        if role == "sim":
            assert service["environment"]["ELESIM_SIM_VIEWER"] == (
                "${ELESIM_SIM_VIEWER:-}"
            )
            assert service["environment"]["DISPLAY"] == "${DISPLAY:-:0}"
            assert "/tmp/.X11-unix:/tmp/.X11-unix:rw" in service["volumes"]
            assert 'ELESIM_SIM_VIEWER:-' in entrypoint
            assert "sim_args+=(--viewer)" in entrypoint
    tools = state.prefix_path / "containers/build/tools"
    assert (tools / "interfaces/elesim_interfaces/msg/RgbdFrame.msg").is_file()
    assert compose["services"]["tools"]["image"] == "elesim/tools:local"
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
    update_wrapper = (state.bin_path / "elesim-update").read_text(encoding="utf-8")
    down_wrapper = (state.bin_path / "elesim-down").read_text(encoding="utf-8")
    role_wrapper = (state.bin_path / "elesim-sim").read_text(
        encoding="utf-8"
    )
    assert "up -d --build --remove-orphans" in up_wrapper
    assert "--view" in up_wrapper
    assert "export ELESIM_SIM_VIEWER=1" in up_wrapper
    assert "DISPLAY" in up_wrapper
    assert (
        f"viewer_xhost_user={pwd.getpwuid(os.getuid()).pw_name}" in up_wrapper
    )
    assert 'xhost +si:localuser:"$viewer_xhost_user"' in up_wrapper
    assert "viewer-xhost" in up_wrapper
    assert "viewer_xhost_select_state" in up_wrapper
    assert "XDG_RUNTIME_DIR" in up_wrapper
    assert "elesim-net namespace-check >/dev/null" in up_wrapper
    assert "down --remove-orphans" in down_wrapper
    assert 'xhost -si:localuser:"$viewer_xhost_user"' in down_wrapper
    assert "viewer_xhost_cleanup" in down_wrapper
    assert "up --remove-orphans sim" in role_wrapper
    assert "elesim-net namespace-check >/dev/null" in role_wrapper
    assert "update --edition general" in update_wrapper
    assert "build sim pilot ui tools" in update_wrapper
    assert (state.prefix_path / "security").stat().st_mode & 0o777 == 0o700


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
        f"{fallback / 'genesis'}:/tmp/elesim-cache/genesis:rw"
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


def test_runtime_up_view_switch_is_one_shot_and_requires_display(
    tmp_path: Path,
) -> None:
    compose = tmp_path / "compose.yaml"
    compose.write_text("name: elesim-runtime\nservices: {}\n", encoding="utf-8")
    (tmp_path / "install-state.json").write_text(
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
            state_path=tmp_path / "install-state.json",
            viewer_state=tmp_path / "viewer-xhost",
            viewer_user="simuser",
        ),
        encoding="utf-8",
    )
    wrapper.chmod(0o755)
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    docker = fake_bin / "docker"
    docker.write_text(
        "#!/usr/bin/env bash\n"
        "printf '%s' \"${ELESIM_SIM_VIEWER-UNSET}\" > \"$VIEWER_MARKER\"\n",
        encoding="utf-8",
    )
    docker.chmod(0o755)
    xhost = fake_bin / "xhost"
    xhost.write_text(
        "#!/usr/bin/env bash\n"
        "if (( $# == 0 )); then\n"
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
    environment["XHOST_PERMISSION_MARKER"] = str(tmp_path / "xhost.permission")
    environment.pop("DISPLAY", None)
    missing_display = subprocess.run(
        (wrapper, "--view"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert missing_display.returncode == 64
    assert "DISPLAY" in missing_display.stderr
    assert not Path(environment["VIEWER_MARKER"]).exists()

    environment["DISPLAY"] = ":0"
    viewed = subprocess.run(
        (wrapper, "--view"),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )
    assert viewed.returncode == 0
    assert Path(environment["VIEWER_MARKER"]).read_text(encoding="utf-8") == "1"
    assert (tmp_path / "viewer-xhost").read_text(encoding="utf-8") == ":0\n\n"
    assert (tmp_path / "xhost.permission").is_file()

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


def test_runtime_down_revokes_owned_xhost_with_saved_display(tmp_path: Path) -> None:
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
    (tmp_path / "viewer-xhost").write_text(":7\n/tmp/auth\n", encoding="utf-8")
    fake_bin = tmp_path / "bin"
    fake_bin.mkdir()
    _fake_docker(fake_bin)
    xhost = fake_bin / "xhost"
    xhost.write_text(
        "#!/usr/bin/env bash\n"
        "if [[ $1 == -si:localuser:simuser ]]; then\n"
        "  printf '%s' \"${DISPLAY:?}\" > \"${XHOST_REVOKED:?}\"\n"
        "fi\n",
        encoding="utf-8",
    )
    xhost.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{fake_bin}:{environment['PATH']}"
    environment["XHOST_REVOKED"] = str(tmp_path / "xhost.revoked")
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


def test_runtime_up_rolls_back_xhost_when_state_cannot_be_written(
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
    assert "run --rm -T tools elesim-net" in wrapper
    assert "run --rm --build tools elesim-net" not in wrapper


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


def test_managed_coturn_rejects_symlinked_secret_path(local_state, tmp_path: Path) -> None:
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

    with pytest.raises(ValueError, match="symlinked path components"):
        ContainerInstaller(state).run()


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
    ] == str(state.prefix_path / "security/roles/sim")
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
    pilot_view = state.prefix_path / "security/roles/pilot"
    ui_view = state.prefix_path / "security/roles/ui"
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
