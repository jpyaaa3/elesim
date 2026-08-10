from __future__ import annotations

from pathlib import Path

from elesim_setup.capabilities import (
    detect_host_capabilities,
    detect_install_host_capabilities,
)


def test_jetson_is_detected_from_l4t_release(tmp_path: Path) -> None:
    root = tmp_path / "root"
    release = root / "etc/nv_tegra_release"
    release.parent.mkdir(parents=True)
    release.write_text("# R36 (release), REVISION: 4.3\n", encoding="utf-8")

    capabilities = detect_host_capabilities(
        root=root,
        environ={"DISPLAY": ":0"},
        machine="aarch64",
        command_output=lambda _command: "",
    )

    assert capabilities.jetson is True
    assert capabilities.robot_installable is True
    assert capabilities.developer_installable is False


def test_amd64_developer_host_reports_gpu_and_never_enables_robot(
    tmp_path: Path,
) -> None:
    root = tmp_path / "root"
    os_release = root / "etc/os-release"
    os_release.parent.mkdir(parents=True)
    os_release.write_text('ID=ubuntu\nVERSION_ID="22.04"\n', encoding="utf-8")

    capabilities = detect_host_capabilities(
        root=root,
        environ={"SSH_AUTH_SOCK": "/tmp/agent.sock"},
        machine="x86_64",
        command_output=lambda command: {
            ("nvidia-smi", "-L"): "GPU 0: NVIDIA RTX A6000 (UUID: GPU-deadbeef)\n",
            ("docker", "info", "--format", "{{.Name}}"): "docker-desktop\n",
            ("docker", "context", "show"): "default\n",
            ("docker", "info", "--format", "{{.ID}}"): "engine-1\n",
            (
                "docker",
                "context",
                "inspect",
                "default",
                "--format",
                '{{(index .Endpoints "docker").Host}}',
            ): "unix:///var/run/docker.sock\n",
        }.get(command, ""),
    )

    assert capabilities.developer_installable is True
    assert capabilities.robot_installable is False
    assert capabilities.ssh_agent is True
    assert capabilities.gpu_devices[0].index == "0"
    assert capabilities.gpu_devices[0].uuid == "GPU-deadbeef"
    assert capabilities.docker_backend == "docker-desktop"
    assert capabilities.docker_context == "default"
    assert capabilities.docker_engine_id == "engine-1"
    assert capabilities.docker_endpoint == "unix:///var/run/docker.sock"
    assert capabilities.docker_host_override == ""


def test_host_capabilities_preserve_docker_host_override_and_remote_endpoint(
    tmp_path: Path,
) -> None:
    capabilities = detect_host_capabilities(
        root=tmp_path,
        environ={"DOCKER_HOST": "tcp://docker.example:2376"},
        machine="x86_64",
        command_output=lambda command: {
            ("docker", "info", "--format", "{{.Name}}"): "remote-engine\n",
            ("docker", "context", "show"): "remote\n",
            ("docker", "info", "--format", "{{.ID}}"): "remote-id\n",
            (
                "docker",
                "context",
                "inspect",
                "remote",
                "--format",
                '{{(index .Endpoints "docker").Host}}',
            ): "ssh://operator@docker.example\n",
        }.get(command, ""),
    )

    assert capabilities.docker_endpoint == "ssh://operator@docker.example"
    assert capabilities.docker_host_override == "tcp://docker.example:2376"


def test_missing_optional_host_files_are_not_errors(tmp_path: Path) -> None:
    capabilities = detect_host_capabilities(
        root=tmp_path,
        environ={},
        machine="x86_64",
        command_output=lambda _command: "",
    )

    assert capabilities.jetson is False
    assert capabilities.gpu_devices == ()
    assert capabilities.display_available is False


def test_bootstrap_preserves_wslg_and_display_facts_from_the_host() -> None:
    capabilities = detect_install_host_capabilities(
        {
            "ELESIM_HOST_ARCH": "x86_64",
            "ELESIM_HOST_OS_ID": "ubuntu",
            "ELESIM_HOST_OS_VERSION": "22.04",
            "ELESIM_HOST_JETSON": "0",
            "ELESIM_HOST_WSL": "1",
            "ELESIM_HOST_WSLG": "1",
            "ELESIM_HOST_DISPLAY": "1",
            "ELESIM_HOST_GPU_LIST": "",
            "ELESIM_HOST_DOCKER_BACKEND": "docker-desktop",
            "ELESIM_HOST_DOCKER_CONTEXT": "default",
            "ELESIM_HOST_DOCKER_ENGINE_ID": "desktop-engine",
            "ELESIM_HOST_DOCKER_ENDPOINT": "unix:///var/run/docker.sock",
            "ELESIM_HOST_DOCKER_HOST_OVERRIDE": "",
        }
    )

    assert capabilities.wsl is True
    assert capabilities.wslg_available is True
    assert capabilities.display_available is True
    assert capabilities.docker_backend == "docker-desktop"
    assert capabilities.docker_context == "default"
    assert capabilities.docker_engine_id == "desktop-engine"
    assert capabilities.docker_endpoint == "unix:///var/run/docker.sock"
    assert capabilities.docker_host_override == ""
