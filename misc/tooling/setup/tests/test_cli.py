from __future__ import annotations

import argparse
import json
from pathlib import Path

from conftest import ROOT, copy_role_configs
from elesim_setup import cli, network
from elesim_setup.state import InstallState


def test_cli_commands_match_bootstrap_contract() -> None:
    contract = json.loads(
        (ROOT / "misc/setup/bootstrap-contract.json").read_text(encoding="utf-8")
    )
    subparsers = next(
        action
        for action in cli._parser()._actions
        if isinstance(action, argparse._SubParsersAction)
    )
    assert tuple(subparsers.choices) == tuple(contract["required_commands"])


def test_noninteractive_install_dry_run_uses_same_installer(tmp_path: Path) -> None:
    state_path = tmp_path / "state.json"
    result = cli.main(
        (
            "--source-root",
            str(ROOT),
            "--state",
            str(state_path),
            "install",
            "--profile",
            "custom",
            "--role",
            "router",
            "--prefix",
            str(tmp_path / "install"),
            "--bin-dir",
            str(tmp_path / "bin"),
            "--dry-run",
        )
    )
    assert result == 0
    assert not state_path.exists()


def test_noninteractive_install_accepts_specific_gpu_policy(tmp_path: Path) -> None:
    result = cli.main(
        (
            "--source-root",
            str(ROOT),
            "--state",
            str(tmp_path / "state.json"),
            "install",
            "--profile",
            "custom",
            "--role",
            "controller",
            "--prefix",
            str(tmp_path / "install"),
            "--bin-dir",
            str(tmp_path / "bin"),
            "--gpu-mode",
            "specific",
            "--gpu-device",
            "1",
            "--dry-run",
        )
    )
    assert result == 0


def test_noninteractive_container_install_uses_container_backend(
    tmp_path: Path,
) -> None:
    state_path = tmp_path / "state.json"
    result = cli.main(
        (
            "--source-root",
            str(ROOT),
            "--state",
            str(state_path),
            "install",
            "--profile",
            "custom",
            "--role",
            "router",
            "--mode",
            "container",
            "--prefix",
            str(tmp_path / "install"),
            "--bin-dir",
            str(tmp_path / "bin"),
            "--dry-run",
        )
    )

    assert result == 0
    assert not state_path.exists()


def test_status_does_not_require_cached_source_to_still_exist(local_state, tmp_path: Path) -> None:
    state = local_state()
    raw = state.to_dict()
    raw["source_root"] = str(tmp_path / "deleted-source")
    state = InstallState.from_dict(raw)
    path = tmp_path / "state.json"
    state.save(path)
    assert cli.main(("--state", str(path), "status")) == 0


def test_network_configure_rewrites_all_installed_role_configs(local_state, tmp_path: Path) -> None:
    state = local_state(roles=("controller", "ui"))
    copy_role_configs(state)
    path = tmp_path / "state.json"
    state.save(path)

    result = network.main(
        (
            "--state",
            str(path),
            "configure",
            "--non-interactive",
            "--router-host",
            "192.0.2.10",
            "--advertise-host",
            "192.0.2.20",
            "--security",
            "insecure-lan",
        )
    )

    assert result == 0
    updated = InstallState.load(path)
    assert updated.network.router_host == "192.0.2.10"
    assert updated.security.mode == "insecure-lan"
    assert (state.prefix_path / "roles/controller/config/runtime.installed.yaml").is_file()
    assert (state.prefix_path / "roles/ui/config/installed.yaml").is_file()
