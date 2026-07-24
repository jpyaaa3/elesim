from __future__ import annotations

from pathlib import Path

import pytest

from elesim_setup.credentials import (
    credential_relative_paths,
    install_staged_credentials,
)
from elesim_setup.state import NetworkSettings, SecuritySettings


def test_laptop_transfer_manifest_contains_only_laptop_role_material(
    local_state,
    tmp_path: Path,
) -> None:
    root = tmp_path / "secrets"
    state = local_state(
        roles=("controller", "ui"),
        network=NetworkSettings(
            router_host="sim.example.com",
            advertise_host="laptop.example.com",
        ),
        security=SecuritySettings(mode="curve", credentials_root=str(root)),
    )

    paths = {str(path) for path in credential_relative_paths(state)}

    assert "curve/clients/controller-main.key_secret" in paths
    assert "curve/clients/ui-main.key_secret" in paths
    assert "curve/clients/doctor-main.key_secret" in paths
    assert "curve/router/router.key" in paths
    assert all("router.key_secret" not in value for value in paths)
    assert all("robot-go2" not in value for value in paths)


def test_staged_credentials_are_installed_without_overwriting_different_keys(
    tmp_path: Path,
) -> None:
    staged = tmp_path / "staged"
    destination = tmp_path / "destination"
    private = staged / "curve/clients/controller-main.key_secret"
    private.parent.mkdir(parents=True)
    private.write_text("first", encoding="utf-8")

    installed = install_staged_credentials(staged, destination)

    target = destination / "curve/clients/controller-main.key_secret"
    assert target in installed
    assert target.read_text(encoding="utf-8") == "first"
    assert target.stat().st_mode & 0o777 == 0o600

    private.write_text("second", encoding="utf-8")
    with pytest.raises(FileExistsError):
        install_staged_credentials(staged, destination)
    assert target.read_text(encoding="utf-8") == "first"


def test_identical_existing_credentials_are_idempotent(tmp_path: Path) -> None:
    staged = tmp_path / "staged"
    destination = tmp_path / "destination"
    source = staged / "curve/router/router.key"
    target = destination / "curve/router/router.key"
    source.parent.mkdir(parents=True)
    target.parent.mkdir(parents=True)
    source.write_text("same", encoding="utf-8")
    target.write_text("same", encoding="utf-8")

    assert install_staged_credentials(staged, destination) == ()
