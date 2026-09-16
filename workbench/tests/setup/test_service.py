from __future__ import annotations

from pathlib import Path
from dataclasses import replace

from elesim_setup.capabilities import HostCapabilities
from elesim_setup.request import SetupRequest
from elesim_setup.service import SetupService


def _capabilities() -> HostCapabilities:
    return HostCapabilities(
        architecture="x86_64",
        os_id="ubuntu",
        os_version="22.04",
        jetson=False,
        robot_installable=False,
        developer_installable=True,
        display_available=True,
        ssh_agent=False,
        gpu_devices=(),
    )


def test_general_service_uses_existing_container_installer_contract(
    tmp_path: Path,
) -> None:
    request = SetupRequest.from_dict(
        {
            "edition": "general",
            "roles": ["sim"],
            "prefix": str(tmp_path / "install"),
            "bin_dir": str(tmp_path / "bin"),
            "source_root": str(Path(__file__).resolve().parents[3]),
            "gpu_mode": "cpu",
            "dds_security_profile": "trusted-network",
            "turn_mode": "none",
        }
    )
    logs: list[str] = []

    SetupService(_capabilities(), log=logs.append).run(request)

    assert (request.prefix / "containers/compose.yaml").is_file()
    assert (request.bin_dir / "elesim-up").is_file()
    assert any("[setup]" in line for line in logs)


def test_mixed_jetson_install_dispatches_independent_owned_installations(tmp_path, monkeypatch):
    calls = []

    class RecordingInstaller:
        def __init__(self, state, **kwargs):
            self.state = state

        def run(self):
            calls.append(self.state)

    monkeypatch.setattr("elesim_setup.service.ContainerInstaller", RecordingInstaller)
    monkeypatch.setattr("elesim_setup.service.Installer", RecordingInstaller)
    request = SetupRequest.from_dict({
        "roles": ["robot", "pilot", "ui"],
        "prefix": str(tmp_path / "newsim"),
        "bin_dir": str(tmp_path / "newsim/bin"),
        "source_root": str(Path(__file__).resolve().parents[3]),
        "dds_interface": "tailscale0",
        "turn_mode": "none",
    })
    capabilities = replace(_capabilities(), architecture="aarch64", jetson=True,
                           robot_installable=True, developer_installable=False)
    SetupService(capabilities).run(request)
    assert [state.install_mode for state in calls] == ["container", "native"]
    assert calls[0].roles == ("pilot", "ui")
    assert calls[1].roles == ("robot",)
    assert calls[1].prefix == str(tmp_path / "newsim-robot")
    assert calls[1].bin_dir == str(tmp_path / "newsim-robot/bin")
