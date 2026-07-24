from __future__ import annotations

import shutil
from pathlib import Path

import pytest
import yaml

from conftest import ROOT
from elesim_setup.capabilities import HostCapabilities
from elesim_setup.developer import DeveloperInstaller
from elesim_setup.request import SetupRequest


def _request(tmp_path: Path, *, jaeger: bool = False, gpu_mode: str = "inherit") -> SetupRequest:
    workspace = tmp_path / "workspace"
    (workspace / ".git").mkdir(parents=True)
    for relative in (
        "packages/protocol/pyproject.toml",
        "packages/elesim_interfaces/package.xml",
        "packages/elesim_interfaces/CMakeLists.txt",
        "controller/pyproject.toml",
        "ui/pyproject.toml",
        "simulator/pyproject.toml",
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
    assert set(compose["services"]) == {"dev", "jaeger"}
    dev = compose["services"]["dev"]
    assert dev["privileged"] is True
    assert dev["network_mode"] == "host"
    assert dev["gpus"] == "all"
    assert dev["environment"]["ROS_DOMAIN_ID"] == "7"
    assert dev["environment"]["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
    assert any(
        value.endswith(":/opt/elesim/config/cyclonedds.xml:ro")
        for value in dev["volumes"]
    )
    assert dev["volumes"][0] == f"{request.prefix}:{request.prefix}:rw"
    assert compose["services"]["jaeger"]["profiles"] == ["observability"]
    assert (request.bin_dir / "elesim-dev").is_file()
    assert (request.bin_dir / "elesim-jaeger-up").is_file()
    assert (request.prefix / ".elesim/development/home").is_dir()
    assert (request.prefix / ".elesim/development/cache").is_dir()


def test_cpu_developer_install_does_not_request_gpu(tmp_path: Path) -> None:
    request = _request(tmp_path, gpu_mode="cpu")

    DeveloperInstaller(request).run()

    compose = yaml.safe_load(
        (request.prefix / ".elesim/development/compose.yaml").read_text(encoding="utf-8")
    )
    assert "gpus" not in compose["services"]["dev"]
    assert compose["services"]["dev"]["environment"]["CUDA_VISIBLE_DEVICES"] == ""
    assert "jaeger" not in compose["services"]


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


def test_developer_install_never_reuses_unrelated_nonempty_directory(
    tmp_path: Path,
) -> None:
    request = _request(tmp_path)
    (request.prefix / ".git").rmdir()
    (request.prefix / "notes.txt").write_text("unrelated", encoding="utf-8")

    with pytest.raises(ValueError, match="Git workspace"):
        DeveloperInstaller(request).run()


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
            "controller/pyproject.toml",
            "ui/pyproject.toml",
            "simulator/pyproject.toml",
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
    entrypoint = (ROOT / "misc/infra/development/entrypoint.sh").read_text(
        encoding="utf-8"
    )
    dockerfile = (ROOT / "misc/infra/development/Dockerfile").read_text(
        encoding="utf-8"
    )

    assert 'venv="${ELESIM_DEV_VENV:-$HOME/.venv}"' in entrypoint
    assert "--system-site-packages" in entrypoint
    assert '"$venv/bin/python" -m pip install' in entrypoint
    assert "python3-venv" in dockerfile
    assert "colcon" in entrypoint
    assert "packages/elesim_interfaces" in entrypoint
