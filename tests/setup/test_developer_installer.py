from __future__ import annotations

import subprocess
from pathlib import Path

import pytest
import yaml

from conftest import ROOT
from elesim_setup.container_installer import ContainerInstaller
from elesim_setup.developer import (
    resolve_developer_username,
    validate_developer_workspace,
)
from elesim_setup.ownership import OwnershipManifest
from elesim_setup.state import DeveloperAttachmentSettings


def _attachment(workspace: Path, *, wslg: bool = False) -> DeveloperAttachmentSettings:
    return DeveloperAttachmentSettings(
        enabled=True,
        workspace=str(workspace),
        wslg=wslg,
    )


def test_developer_attachment_joins_the_canonical_runtime_project(local_state) -> None:
    state = local_state(
        roles=("pilot", "sim", "ui"),
        developer_attachment=_attachment(ROOT),
    )

    ContainerInstaller(state).run()

    compose = yaml.safe_load(
        (state.prefix_path / "containers/compose.yaml").read_text(encoding="utf-8")
    )
    assert compose["name"] == "elesim-runtime"
    assert {"pilot", "sim", "ui", "dev"} <= set(compose["services"])
    dev = compose["services"]["dev"]
    assert dev["profiles"] == ["developer"]
    assert dev["image"] == "elesim/dev:local"
    assert dev["container_name"] == "elesim-dev"
    assert dev["privileged"] is True
    assert dev["working_dir"] == str(ROOT)
    assert f"{ROOT}:{ROOT}:rw" in dev["volumes"]

    # A development shell is tooling, not a runtime role or DDS identity.
    assert "ROS_DOMAIN_ID" not in dev["environment"]
    assert "RMW_IMPLEMENTATION" not in dev["environment"]
    assert "ROS_SECURITY_KEYSTORE" not in dev["environment"]
    assert not any("security/apps" in value for value in dev["volumes"])

    wrapper = (state.bin_path / "elesim-dev").read_text(encoding="utf-8")
    assert "--profile developer up -d --build dev" in wrapper
    assert "exec dev /usr/local/bin/elesim-dev-env" in wrapper
    assert "--remove-orphans" not in wrapper
    assert subprocess.run(
        ("bash", "-n"),
        input=wrapper,
        text=True,
        capture_output=True,
        check=False,
    ).returncode == 0

    manifest = OwnershipManifest.load(state.prefix_path / "install-ownership.json")
    assert manifest.docker.project == "elesim-runtime"
    assert "elesim-dev" in manifest.docker.containers
    assert "elesim/dev:local" in manifest.docker.local_images
    assert not (state.prefix_path / ".elesim/development").exists()


def test_attachment_workspace_must_be_an_existing_complete_checkout(
    local_state,
    tmp_path: Path,
) -> None:
    workspace = tmp_path / "incomplete"
    (workspace / ".git").mkdir(parents=True)
    state = local_state(developer_attachment=_attachment(workspace))

    with pytest.raises(ValueError, match="완전한 EleSim Git checkout"):
        ContainerInstaller(state).run()

    assert not state.prefix_path.exists()


def test_workspace_validator_accepts_the_payload_checkout() -> None:
    validate_developer_workspace(ROOT)


def test_developer_username_has_a_safe_fallback(monkeypatch) -> None:
    for variable in ("ELESIM_HOST_USER", "USER", "LOGNAME"):
        monkeypatch.setenv(variable, "invalid user!")
    assert resolve_developer_username() == "dev"
