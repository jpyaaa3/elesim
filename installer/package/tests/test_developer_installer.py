from __future__ import annotations

import json
import os
import shutil
import subprocess
from pathlib import Path

import pytest
import yaml

from conftest import ROOT
from elesim_setup.capabilities import HostCapabilities
from elesim_setup.developer import DeveloperInstaller
from elesim_setup.ownership import (
    DOCKER_INSTALL_UUID_LABEL,
    OwnershipError,
    OwnershipManifest,
)
from elesim_setup.request import SetupRequest


def _request(tmp_path: Path, *, jaeger: bool = False, gpu_mode: str = "inherit") -> SetupRequest:
    workspace = tmp_path / "workspace"
    (workspace / ".git").mkdir(parents=True)
    for relative in (
        "packages/protocol/pyproject.toml",
        "packages/elesim_interfaces/package.xml",
        "packages/elesim_interfaces/CMakeLists.txt",
        "pilot/pyproject.toml",
        "ui/pyproject.toml",
        "sim/pyproject.toml",
        "robot/pyproject.toml",
    ):
        path = workspace / relative
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("[project]\nname='placeholder'\n", encoding="utf-8")
    return SetupRequest.from_dict(
        {
            "language": "ko",
            "edition": "developer",
            "roles": [],
            "prefix": str(workspace),
            "bin_dir": str(workspace / "bin"),
            "source_root": str(ROOT),
            "gpu_mode": gpu_mode,
            "gpu_device": "",
            "dds_domain_id": 7,
            "turn_mode": "none",
            "jaeger": jaeger,
        }
    )


def test_developer_install_generates_one_privileged_workspace_service(
    tmp_path: Path,
) -> None:
    request = _request(tmp_path, jaeger=True)

    DeveloperInstaller(request).run()

    compose_path = request.prefix / ".elesim/development/compose.yaml"
    compose = yaml.safe_load(compose_path.read_text(encoding="utf-8"))
    assert compose["name"] == "elesim-runtime-dev"
    assert set(compose["services"]) == {"dev", "manager", "jaeger"}
    dev = compose["services"]["dev"]
    assert dev["image"] == "elesim/dev:local"
    assert dev["container_name"] == "elesim-dev"
    assert dev["privileged"] is True
    assert dev["network_mode"] == "host"
    assert dev["gpus"] == "all"
    assert dev["environment"]["ROS_DOMAIN_ID"] == "7"
    assert dev["environment"]["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
    assert dev["environment"]["USER"] == dev["environment"]["LOGNAME"]
    assert dev["environment"]["ELESIM_HOST_USER"] == dev["environment"]["USER"]
    assert dev["build"]["args"]["USERNAME"] == dev["environment"]["USER"]
    assert dev["build"]["args"]["COMPUTE_MODE"] == "inherit"
    assert any(
        value.endswith(":/opt/elesim/config/cyclonedds.xml:ro")
        for value in dev["volumes"]
    )
    assert dev["volumes"][0] == f"{request.prefix}:{request.prefix}:rw"
    assert compose["services"]["jaeger"]["profiles"] == ["observability"]
    assert compose["services"]["jaeger"]["container_name"] == "elesim-jaeger"
    manager = compose["services"]["manager"]
    assert manager["profiles"] == ["manager"]
    assert "container_name" not in manager
    assert "network_mode" not in manager
    assert manager["environment"]["ELESIM_OPERATOR_HOME"] == str(
        Path.home().resolve()
    )
    assert manager["environment"]["DOCKER_CONFIG"] == "/tmp/elesim-docker-config"
    assert "/var/run/docker.sock:/var/run/docker.sock:rw" not in manager["volumes"]
    assert (request.bin_dir / "elesim-dev").is_file()
    assert (request.bin_dir / "elesim-connections").is_file()
    assert (request.bin_dir / "elesim-update").is_file()
    assert (request.bin_dir / "elesim-jaeger-up").is_file()
    assert (request.prefix / ".elesim/development/home").is_dir()
    assert (request.prefix / ".elesim/development/cache").is_dir()
    assert (request.prefix / ".elesim/development/build/dev-env.sh").is_file()
    update_wrapper = (request.bin_dir / "elesim-update").read_text(encoding="utf-8")
    assert "merge --ff-only FETCH_HEAD" in update_wrapper
    assert "update --edition developer" in update_wrapper
    assert "build dev" in update_wrapper
    manager_wrapper = (request.bin_dir / "elesim-connections").read_text(
        encoding="utf-8"
    )
    assert f'ELESIM_OPERATOR_HOME={Path.home().resolve()}' in manager_wrapper
    assert '--publish "127.0.0.1:${manager_port}:${manager_port}"' in manager_wrapper
    assert "manager_args+=(--host 0.0.0.0)" in manager_wrapper
    assert "ELESIM_INSTALL_GPU_MODE=inherit" in manager_wrapper
    assert "ELESIM_TAILSCALE_PROXY_BIN=/usr/local/bin/elesim-host-proxy" in manager_wrapper
    assert "/var/run/tailscale/tailscaled.sock" not in manager_wrapper
    assert "elesim_setup.host_helper" in manager_wrapper
    assert "manager_started=0" in manager_wrapper
    assert "trap 'host_helper_cleanup; manager_cleanup' EXIT" in manager_wrapper
    assert "docker rm elesim-manager" in manager_wrapper
    assert "docker rm -f elesim-manager" in manager_wrapper
    assert "manager_status=$?" in manager_wrapper
    assert "ELESIM_DOCKER_GID" not in manager_wrapper
    assert "elesim-manager-compose" not in manager_wrapper
    assert "--group-add" not in manager_wrapper
    assert "IFS=',' read -r -a compose_files" in manager_wrapper
    assert "compose_match != 1" in manager_wrapper


def test_developer_install_supplies_fallback_username_when_host_identity_is_missing(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    request = _request(tmp_path)
    for variable in ("ELESIM_HOST_USER", "USER", "LOGNAME"):
        monkeypatch.setenv(variable, "not a linux username")

    DeveloperInstaller(request).run()

    compose = yaml.safe_load(
        (request.prefix / ".elesim/development/compose.yaml").read_text(
            encoding="utf-8"
        )
    )
    dev = compose["services"]["dev"]
    assert dev["build"]["args"]["USERNAME"] == "dev"
    assert dev["environment"]["USER"] == "dev"
    assert dev["environment"]["LOGNAME"] == "dev"
    assert dev["environment"]["ELESIM_HOST_USER"] == "dev"


def test_developer_context_falls_back_when_legacy_context_is_unwritable(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    request = _request(tmp_path)
    installer = DeveloperInstaller(request)
    legacy = request.prefix / ".elesim/development/build"
    legacy.mkdir(parents=True)
    real_access = os.access

    def fake_access(path, mode, **kwargs):
        if Path(path) == legacy:
            return False
        return real_access(path, mode, **kwargs)

    monkeypatch.setattr(os, "access", fake_access)
    selected = installer._prepare_build_root()

    assert selected == request.prefix / ".elesim/development/.runtime-build"


def test_developer_install_records_nested_manifest_and_docker_uuid(
    tmp_path: Path,
) -> None:
    request = _request(tmp_path, jaeger=True)

    DeveloperInstaller(request).run()

    manifest_path = request.prefix / ".elesim/development/install-ownership.json"
    manifest = OwnershipManifest.load(manifest_path)
    compose = yaml.safe_load(
        (request.prefix / ".elesim/development/compose.yaml").read_text(
            encoding="utf-8"
        )
    )
    assert manifest.path == manifest_path
    assert not (request.prefix / "install-ownership.json").exists()
    assert manifest.docker is not None
    assert manifest.docker.install_uuid == manifest.install_uuid
    assert manifest.docker.project == compose["name"] == "elesim-runtime-dev"
    assert manifest.docker.compose_file == str(
        request.prefix / ".elesim/development/compose.yaml"
    )
    assert set(manifest.docker.containers) == {
        "elesim-dev",
        "elesim-manager",
        "elesim-jaeger",
    }
    assert manifest.docker.local_images == ("elesim/dev:local",)
    for service in compose["services"].values():
        assert service["labels"][DOCKER_INSTALL_UUID_LABEL] == manifest.install_uuid
        if "build" in service:
            assert (
                service["build"]["labels"][DOCKER_INSTALL_UUID_LABEL]
                == manifest.install_uuid
            )

    wrapper = (request.bin_dir / "elesim-uninstall").read_text(encoding="utf-8")
    assert "exec python3 -B -S -m elesim_setup.uninstall" in wrapper
    assert f"--manifest {manifest_path}" in wrapper
    assert "export PYTHONNOUSERSITE=1" in wrapper
    assert "docker compose" not in wrapper


def test_developer_shell_reuses_the_persistent_container(tmp_path: Path) -> None:
    request = _request(tmp_path)

    DeveloperInstaller(request).run()

    wrapper = (request.bin_dir / "elesim-dev").read_text(encoding="utf-8")
    assert "up -d --build --remove-orphans dev" in wrapper
    assert "exec dev /usr/local/bin/elesim-dev-env" in wrapper
    assert "run --rm" not in wrapper
    assert "set -- bash" in wrapper
    assert "up -d --build --remove-orphans dev" in (
        request.bin_dir / "elesim-up"
    ).read_text(encoding="utf-8")
    assert "down --remove-orphans" in (
        request.bin_dir / "elesim-down"
    ).read_text(encoding="utf-8")
    down_wrapper = (request.bin_dir / "elesim-down").read_text(encoding="utf-8")
    assert "elesim-down [--purge]" in down_wrapper
    assert "docker rm -f elesim-manager" in down_wrapper


def test_developer_wrapper_rejects_a_container_owned_by_another_install(
    tmp_path: Path,
) -> None:
    request = _request(tmp_path)
    DeveloperInstaller(request).run()
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
            "ELESIM_FAKE_CONTAINER": "elesim-dev",
            "ELESIM_FAKE_METADATA": "other-project|/other/compose.yaml",
        }
    )

    result = subprocess.run(
        (request.bin_dir / "elesim-dev",),
        env=environment,
        text=True,
        capture_output=True,
        check=False,
    )

    assert result.returncode == 73
    assert "elesim-dev" in result.stderr
    assert "기존 설치의 elesim-down" in result.stderr


def test_cpu_developer_install_does_not_request_gpu(tmp_path: Path) -> None:
    request = _request(tmp_path, gpu_mode="cpu")

    DeveloperInstaller(request).run()

    compose = yaml.safe_load(
        (request.prefix / ".elesim/development/compose.yaml").read_text(encoding="utf-8")
    )
    assert "gpus" not in compose["services"]["dev"]
    assert compose["services"]["dev"]["environment"]["CUDA_VISIBLE_DEVICES"] == ""
    assert "jaeger" not in compose["services"]
    assert compose["services"]["dev"]["build"]["args"]["COMPUTE_MODE"] == "cpu"
    manager_wrapper = (request.bin_dir / "elesim-connections").read_text(
        encoding="utf-8"
    )
    assert "ELESIM_INSTALL_GPU_MODE=cpu" in manager_wrapper


def test_specific_gpu_developer_install_reserves_only_selected_device(
    tmp_path: Path,
) -> None:
    request = _request(tmp_path, gpu_mode="specific")
    request = SetupRequest.from_dict(
        {
            "language": request.language,
            "edition": "developer",
            "roles": [],
            "prefix": str(request.prefix),
            "bin_dir": str(request.bin_dir),
            "source_root": str(request.source_root),
            "gpu_mode": "specific",
            "gpu_device": "GPU-deadbeef",
            "dds_domain_id": request.dds.domain_id,
            "turn_mode": "none",
        }
    )

    DeveloperInstaller(request).run()

    compose = yaml.safe_load(
        (request.prefix / ".elesim/development/compose.yaml").read_text(
            encoding="utf-8"
        )
    )
    service = compose["services"]["dev"]
    reservation = service["deploy"]["resources"]["reservations"]["devices"][0]
    assert reservation["device_ids"] == ["GPU-deadbeef"]
    assert "gpus" not in service
    assert "CUDA_VISIBLE_DEVICES" not in service["environment"]
    manager_wrapper = (request.bin_dir / "elesim-connections").read_text(
        encoding="utf-8"
    )
    assert "ELESIM_INSTALL_GPU_MODE=specific" in manager_wrapper


def test_developer_install_never_reuses_unrelated_nonempty_directory(
    tmp_path: Path,
) -> None:
    request = _request(tmp_path)
    (request.prefix / ".git").rmdir()
    (request.prefix / "notes.txt").write_text("unrelated", encoding="utf-8")

    with pytest.raises(ValueError, match="Git workspace"):
        DeveloperInstaller(request).run()


def test_developer_reinstall_accepts_only_prior_valid_uninstall_tombstone(
    tmp_path: Path,
) -> None:
    request = _request(tmp_path)
    install_uuid = "12345678-1234-5678-9234-567812345678"
    generated = request.prefix / ".elesim/development"
    generated.mkdir(parents=True)
    tombstone = generated / f"uninstall-tombstone-{install_uuid}.json"
    tombstone.write_text(
        json.dumps(
            {
                "schema_version": 1,
                "install_uuid": install_uuid,
                "edition": "developer",
                "prefix": str(request.prefix),
                "completed_at": "2026-08-03T12:34:56+00:00",
                "purged_logs": False,
                "purged_authority": False,
                "preserved_paths": [
                    str(request.prefix / ".elesim/authority"),
                ],
            }
        )
        + "\n",
        encoding="utf-8",
    )

    DeveloperInstaller(request).run()

    manifest = OwnershipManifest.load(
        generated / "install-ownership.json"
    )
    assert tombstone.is_file()
    assert str(tombstone) in manifest.external_paths


@pytest.mark.parametrize(
    ("case", "name"),
    (
        (
            "missing-required-fields",
            "uninstall-tombstone-12345678-1234-5678-9234-567812345678.json",
        ),
        (
            "naive-timestamp",
            "uninstall-tombstone-12345678-1234-5678-9234-567812345678.json",
        ),
        ("foreign-file", "foreign-notes.json"),
    ),
    ids=("missing-required-fields", "naive-timestamp", "foreign-file"),
)
def test_developer_reinstall_rejects_invalid_or_foreign_residual_without_manifest(
    tmp_path: Path,
    case: str,
    name: str,
) -> None:
    request = _request(tmp_path)
    generated = request.prefix / ".elesim/development"
    generated.mkdir(parents=True)
    residual = generated / name
    payload: dict[str, object] = {
        "schema_version": 1,
        "install_uuid": "12345678-1234-5678-9234-567812345678",
        "edition": "developer",
        "prefix": str(request.prefix),
        "completed_at": "2026-08-03T12:34:56+00:00",
        "purged_logs": False,
        "purged_authority": False,
        "preserved_paths": [],
    }
    if case == "missing-required-fields":
        payload.pop("purged_authority")
    elif case == "naive-timestamp":
        payload["completed_at"] = "2026-08-03T12:34:56"
    else:
        payload = {"owner": "operator"}
    residual.write_text(json.dumps(payload) + "\n", encoding="utf-8")

    with pytest.raises(OwnershipError, match="ownership manifest 없는"):
        DeveloperInstaller(request).run()

    assert residual.is_file()
    assert not (generated / "install-ownership.json").exists()


def test_developer_dry_run_does_not_write_generated_context(tmp_path: Path) -> None:
    request = _request(tmp_path)

    DeveloperInstaller(request, dry_run=True).run()

    assert not (request.prefix / ".elesim").exists()


def test_wslg_host_adds_wslg_mount_and_runtime_environment(tmp_path: Path) -> None:
    request = _request(tmp_path)
    capabilities = HostCapabilities(
        architecture="x86_64",
        os_id="ubuntu",
        os_version="22.04",
        jetson=False,
        robot_installable=False,
        developer_installable=True,
        display_available=True,
        ssh_agent=False,
        gpu_devices=(),
        wsl=True,
        wslg_available=True,
    )

    DeveloperInstaller(request, capabilities=capabilities).run()

    compose = yaml.safe_load(
        (request.prefix / ".elesim/development/compose.yaml").read_text(
            encoding="utf-8"
        )
    )
    service = compose["services"]["dev"]
    assert "/mnt/wslg:/mnt/wslg:rw" in service["volumes"]
    assert service["environment"]["XDG_RUNTIME_DIR"] == (
        "${XDG_RUNTIME_DIR:-/mnt/wslg/runtime-dir}"
    )
    assert service["environment"]["PULSE_SERVER"] == (
        "${PULSE_SERVER:-unix:/mnt/wslg/PulseServer}"
    )


def test_empty_existing_workspace_is_populated_without_removing_mountpoint(
    tmp_path: Path,
    monkeypatch,
) -> None:
    request = _request(tmp_path)
    shutil.rmtree(request.prefix)
    request.prefix.mkdir()
    calls: list[Path] = []

    def clone(_url, destination, **_kwargs) -> None:
        target = Path(destination)
        calls.append(target)
        (target / ".git").mkdir(parents=True)
        for relative in (
            "packages/protocol/pyproject.toml",
            "packages/elesim_interfaces/package.xml",
            "packages/elesim_interfaces/CMakeLists.txt",
            "pilot/pyproject.toml",
            "ui/pyproject.toml",
            "sim/pyproject.toml",
            "robot/pyproject.toml",
        ):
            path = target / relative
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_text("[project]\n", encoding="utf-8")

    monkeypatch.setattr("dulwich.porcelain.clone", clone)

    DeveloperInstaller(request).run()

    assert calls == [request.prefix / ".elesim-clone-staging"]
    assert request.prefix.is_dir()
    assert (request.prefix / ".git").is_dir()
    assert not (request.prefix / ".elesim-clone-staging").exists()


def test_development_entrypoint_uses_persistent_virtual_environment() -> None:
    entrypoint = (ROOT / "environment/development/entrypoint.sh").read_text(
        encoding="utf-8"
    )
    dev_env = (ROOT / "environment/development/dev-env.sh").read_text(
        encoding="utf-8"
    )
    dockerfile = (ROOT / "environment/development/Dockerfile").read_text(
        encoding="utf-8"
    )

    assert 'exec /usr/local/bin/elesim-dev-env "$@"' in entrypoint
    assert 'venv="${ELESIM_DEV_VENV:-$HOME/.elesim/venv}"' in dev_env
    assert "--system-site-packages" in dev_env
    assert "--without-pip" in dev_env
    assert '"$venv/bin/python" -m pip install' in dev_env
    assert "--no-build-isolation --no-deps" in dev_env
    assert 'editable_args+=(--editable "$project")' in dev_env
    assert "sha256sum /usr/local/bin/elesim-dev-env" in dev_env
    assert "flock" in dev_env
    assert "set +u\n  source /opt/ros/humble/setup.bash\n  set -u" in dev_env
    assert "set +u\nsource /opt/ros/humble/setup.bash" in dev_env
    assert "source \"$ros_overlay/install/setup.bash\"" in dev_env
    assert "dev-env.sh /usr/local/bin/elesim-dev-env" in dockerfile
    assert "python3-venv" in dockerfile
    assert "util-linux" in dockerfile
    assert '"torch==2.12.1+cpu"' in dockerfile
    assert "ARG COMPUTE_MODE=inherit" in dockerfile
    assert "colcon" in dev_env
    assert "packages/elesim_interfaces" in dev_env
