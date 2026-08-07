from __future__ import annotations

import argparse
import base64
import json
from dataclasses import replace
from pathlib import Path

import pytest
import yaml

from conftest import ROOT, copy_role_configs
from elesim_setup import cli, network
from elesim_setup.configuration import generate_role_configs, generated_config_path
from elesim_setup.security_provisioning import sync_provisioning_required
from elesim_setup.state import DdsSettings, InstallState


def test_cli_commands_match_bootstrap_contract() -> None:
    contract = json.loads(
        (ROOT / "installer/bootstrap/bootstrap-contract.json").read_text(encoding="utf-8")
    )
    subparsers = next(
        action
        for action in cli._parser()._actions
        if isinstance(action, argparse._SubParsersAction)
    )
    assert tuple(subparsers.choices) == tuple(contract["required_commands"])


def test_runtime_namespace_check_requires_configured_interface(local_state) -> None:
    state = local_state(dds=DdsSettings(interface="tailscale0"))

    network.require_runtime_network_namespace(
        state,
        interface_names=("lo", "eth0", "tailscale0"),
    )
    with pytest.raises(RuntimeError, match="Docker Desktop/WSL"):
        network.require_runtime_network_namespace(
            state,
            interface_names=("lo", "eth0"),
        )


def test_runtime_namespace_check_accepts_pending_tailscale_bind(local_state) -> None:
    # The installed state may still contain an older interface.  The
    # connection manager can validate the pending direct-bind choice without
    # rewriting that state first.
    state = local_state(dds=DdsSettings(interface="eth0"))

    network.require_runtime_network_namespace(
        state,
        interface="tailscale0",
        interface_names=("lo", "eth0", "tailscale0"),
    )


def test_runtime_namespace_check_allows_automatic_interface(local_state) -> None:
    network.require_runtime_network_namespace(
        local_state(dds=DdsSettings(interface="")),
        interface_names=(),
    )


def test_update_reuses_installed_general_state_with_new_source(
    local_state,
    monkeypatch,
    tmp_path: Path,
) -> None:
    state_path = tmp_path / "install/install-state.json"
    state = replace(
        local_state(
            roles=("pilot", "ui"),
            dds=DdsSettings(interface="tailscale0"),
        ),
        source_root=str(tmp_path / "old-source"),
    )
    state.save(state_path)
    received = []

    class FakeInstaller:
        def __init__(self, updated, **kwargs) -> None:
            received.append((updated, kwargs))

        def run(self) -> None:
            return None

    monkeypatch.setattr(cli, "ContainerInstaller", FakeInstaller)
    result = cli.main(
        (
            "--source-root",
            str(tmp_path / "new-source"),
            "--state",
            str(state_path),
            "update",
        )
    )

    assert result == 0
    updated, kwargs = received[0]
    assert updated.source_path == (tmp_path / "new-source").resolve()
    assert updated.roles == state.roles
    assert updated.dds == state.dds
    assert updated.network == state.network
    assert kwargs["state_path"] == state_path.resolve()


def test_developer_update_state_round_trip(tmp_path: Path) -> None:
    workspace = tmp_path / "workspace"
    state_path = workspace / ".elesim/development/install-state.json"
    state_path.parent.mkdir(parents=True)
    state_path.write_text(
        json.dumps(
            {
                "schema_version": 2,
                "workspace": str(workspace),
                "bin_dir": str(workspace / "bin"),
                "repository": "lab/elesim",
                "ref": "refactoring",
                "gpu_mode": "specific",
                "gpu_device": "GPU-1",
                "jaeger": True,
                "dds": {
                    "system_id": "lab",
                    "domain_id": 27,
                    "rmw_implementation": "rmw_cyclonedds_cpp",
                    "discovery_mode": "static",
                    "static_peers": ["100.64.0.2"],
                    "interface": "tailscale0",
                    "security_profile": "trusted-network",
                    "keystore": "",
                    "enclave": "",
                },
            }
        ),
        encoding="utf-8",
    )

    request = cli._developer_update_request(state_path, tmp_path / "source")

    assert request.prefix == workspace.resolve()
    assert request.ref == "refactoring"
    assert request.compute.gpu_device == "GPU-1"
    assert request.dds.static_peers == ("100.64.0.2",)
    assert request.dds.interface == "tailscale0"


def test_interactive_role_selector_has_no_computer_presets() -> None:
    selected = cli._ask_roles(input_fn=lambda _prompt: "pilot,ui")
    assert selected == ("pilot", "ui")

    subparsers = next(
        action
        for action in cli._parser()._actions
        if isinstance(action, argparse._SubParsersAction)
    )
    install_parser = subparsers.choices["install"]
    help_text = install_parser.format_help()
    assert "--role" in help_text
    assert "--profile" not in help_text
    assert "local-sim" not in help_text


def test_interactive_runtime_text_log_archive_is_optional() -> None:
    enabled = cli._ask_runtime_text_logs(input_fn=lambda _prompt: "")
    disabled = cli._ask_runtime_text_logs(input_fn=lambda _prompt: "n")

    assert enabled.enabled is True
    assert disabled.enabled is False


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
            "sim",
            "--prefix",
            str(tmp_path / "install"),
            "--bin-dir",
            str(tmp_path / "bin"),
            "--dry-run",
        )
    )
    assert result == 0
    assert not state_path.exists()


def test_noninteractive_install_mode_defaults_from_roles(tmp_path: Path) -> None:
    common = (
        "--prefix",
        str(tmp_path / "install"),
        "--bin-dir",
        str(tmp_path / "bin"),
    )
    application_args = cli._parser().parse_args(
        ("install", "--profile", "laptop", *common)
    )
    robot_args = cli._parser().parse_args(
        (
            "install",
            "--profile",
            "custom",
            "--role",
            "robot",
            *common,
        )
    )

    assert cli._build_state(application_args, ROOT).install_mode == "container"
    assert cli._build_state(robot_args, ROOT).install_mode == "native"


def test_noninteractive_roles_override_legacy_profile_default(tmp_path: Path) -> None:
    args = cli._parser().parse_args(
        (
            "install",
            "--role",
            "sim",
            "--role",
            "ui",
            "--prefix",
            str(tmp_path / "install"),
            "--bin-dir",
            str(tmp_path / "bin"),
        )
    )

    state = cli._build_state(args, ROOT)
    assert state.profile == "custom"
    assert state.roles == ("sim", "ui")
    assert state.network.sim_id == "sim-default"
    assert state.network.pilot_id == "pilot-main"
    assert state.network.ui_id == "ui-main"
    assert state.network.robot_id == "robot-go2"
    assert state.runtime_text_logs.enabled is True


def test_noninteractive_runtime_text_logs_use_boolean_optional_flag(
    tmp_path: Path,
) -> None:
    common = (
        "install",
        "--role",
        "ui",
        "--prefix",
        str(tmp_path / "install"),
        "--bin-dir",
        str(tmp_path / "bin"),
    )

    enabled = cli._parser().parse_args(common)
    disabled = cli._parser().parse_args((*common, "--no-runtime-text-logs"))

    assert cli._build_state(enabled, ROOT).runtime_text_logs.enabled is True
    assert cli._build_state(disabled, ROOT).runtime_text_logs.enabled is False


def test_noninteractive_install_rejects_explicit_topology_mode_mismatch(
    tmp_path: Path,
) -> None:
    common = (
        "--prefix",
        str(tmp_path / "install"),
        "--bin-dir",
        str(tmp_path / "bin"),
    )
    native_app = cli._parser().parse_args(
        ("install", "--profile", "laptop", "--mode", "native", *common)
    )
    container_robot = cli._parser().parse_args(
        (
            "install",
            "--profile",
            "custom",
            "--role",
            "robot",
            "--mode",
            "container",
            *common,
        )
    )

    with pytest.raises(ValueError, match="Docker/Compose"):
        cli._build_state(native_app, ROOT)
    with pytest.raises(ValueError, match="generic Ubuntu"):
        cli._build_state(container_robot, ROOT)


def test_noninteractive_install_accepts_pending_managed_sros2_and_coturn(
    tmp_path: Path,
) -> None:
    result = cli.main(
        (
            "--source-root",
            str(ROOT),
            "--state",
            str(tmp_path / "state.json"),
            "install",
            "--profile",
            "compute",
            "--mode",
            "container",
            "--prefix",
            str(tmp_path / "install"),
            "--bin-dir",
            str(tmp_path / "bin"),
            "--dds-security-profile",
            "sros2",
            "--dds-security-provisioning",
            "managed",
            "--turn-mode",
            "managed",
            "--turn-url",
            "turn:turn.example.com:3478?transport=udp",
            "--turn-realm",
            "elesim.local",
            "--turn-public-host",
            "turn.example.com",
            "--dry-run",
        )
    )

    assert result == 0


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
            "pilot",
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
            "sim",
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


def test_noninteractive_external_turn_requires_sim_credential_path(
    tmp_path: Path,
) -> None:
    credentials = tmp_path / "turn.json"
    credentials.write_text(
        '{"username":"lab-user","credential":"lab-password"}\n',
        encoding="utf-8",
    )
    base = (
        "--source-root",
        str(ROOT),
        "--state",
        str(tmp_path / "state.json"),
        "install",
        "--profile",
        "custom",
        "--role",
        "sim",
        "--mode",
        "container",
        "--prefix",
        str(tmp_path / "install"),
        "--bin-dir",
        str(tmp_path / "bin"),
        "--turn-mode",
        "external",
        "--turn-url",
        "turn:relay.example.com:3478?transport=udp",
        "--dry-run",
    )

    assert cli.main(base) == 2
    assert cli.main((*base, "--turn-credential-file", str(credentials))) == 0


def test_status_does_not_require_cached_source_to_still_exist(local_state, tmp_path: Path) -> None:
    state = local_state()
    raw = state.to_dict()
    raw["source_root"] = str(tmp_path / "deleted-source")
    state = InstallState.from_dict(raw)
    path = tmp_path / "state.json"
    state.save(path)
    assert cli.main(("--state", str(path), "status")) == 0


def test_network_configure_rewrites_all_installed_role_configs(local_state, tmp_path: Path) -> None:
    state = local_state(roles=("pilot", "ui"))
    copy_role_configs(state)
    path = tmp_path / "state.json"
    state.save(path)

    result = network.main(
        (
            "--state",
            str(path),
            "configure",
            "--non-interactive",
            "--dds-domain-id",
            "17",
            "--dds-discovery-mode",
            "static",
            "--dds-static-peer",
            "192.0.2.10",
            "--ui-id",
            "ui-field",
            "--robot-id",
            "robot-field",
        )
    )

    assert result == 0
    updated = InstallState.load(path)
    assert updated.dds.domain_id == 17
    assert updated.dds.discovery_mode == "static"
    assert updated.dds.static_peers == ("192.0.2.10",)
    assert updated.network.ui_id == "ui-field"
    assert updated.network.robot_id == "robot-field"
    assert (state.prefix_path / "roles/pilot/config/runtime.installed.yaml").is_file()
    assert (state.prefix_path / "roles/ui/config/installed.yaml").is_file()


def test_network_configure_accepts_manager_owned_sros2_generation(
    local_state, tmp_path: Path
) -> None:
    state = local_state(
        roles=("pilot", "ui"),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="managed",
        ),
    )
    copy_role_configs(state)
    path = tmp_path / "state.json"
    sync_provisioning_required(state)
    state.save(path)
    bundle = state.prefix_path / "security/current/keystore"

    result = network.main(
        (
            "--state",
            str(path),
            "configure",
            "--non-interactive",
            "--dds-security-profile",
            "sros2",
            "--dds-security-provisioning",
            "managed",
            "--dds-security-generation",
            "gen-20260803",
            "--dds-security-bundle",
            str(bundle),
            "--dds-keystore",
            str(bundle),
            "--dds-enclave",
            "/elesim/elesim",
        )
    )

    assert result == 0
    updated = InstallState.load(path)
    assert updated.dds.security_generation == "gen-20260803"
    assert updated.dds.security_bundle_path == bundle
    pilot = yaml.safe_load(
        generated_config_path(updated, "pilot").read_text(encoding="utf-8")
    )
    assert pilot["dds"]["enclave"] == (
        "/elesim/elesim/pilot/pilot_main"
    )
    assert not (state.prefix_path / "security/provisioning-required").exists()


def test_network_restore_snapshot_can_return_active_managed_state_to_pending(
    local_state, tmp_path: Path
) -> None:
    pending = local_state(
        roles=("pilot",),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="managed",
        ),
    )
    copy_role_configs(pending)
    state_path = tmp_path / "state.json"
    active = replace(
        pending,
        dds=replace(
            pending.dds,
            security_generation="g-active",
            security_bundle=str(pending.prefix_path / "security/current/keystore"),
            keystore=str(pending.prefix_path / "security/current/keystore"),
            enclave="/elesim/elesim",
        ),
    )
    generate_role_configs(active)
    sync_provisioning_required(active)
    active.save(state_path)
    payload = base64.urlsafe_b64encode(
        json.dumps(pending.to_dict()).encode("utf-8")
    ).decode("ascii")

    result = network.main(
        ("--state", str(state_path), "restore-snapshot", "--payload", payload)
    )

    restored = InstallState.load(state_path)
    assert result == 0
    assert restored.dds.security_generation == ""
    assert restored.dds.keystore == ""
    assert (pending.prefix_path / "security/provisioning-required").is_file()


def test_network_configure_rejects_external_keystore_change_without_reinstall(
    local_state, tmp_path: Path
) -> None:
    original = tmp_path / "external-old"
    replacement = tmp_path / "external-new"
    state = local_state(
        roles=("pilot",),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="external",
            keystore=str(original),
            enclave="/prod",
        ),
    )
    copy_role_configs(state)
    path = tmp_path / "state.json"
    state.save(path)

    result = network.main(
        (
            "--state",
            str(path),
            "configure",
            "--non-interactive",
            "--dds-keystore",
            str(replacement),
        )
    )

    assert result == 2
    assert InstallState.load(path).dds.keystore == str(original)

    endpoint_result = network.main(
        (
            "--state",
            str(path),
            "configure",
            "--non-interactive",
            "--pilot-id",
            "pilot-next",
        )
    )

    assert endpoint_result == 2
    assert InstallState.load(path).network.pilot_id == "pilot-main"


def test_network_configuration_rollback_restores_pending_marker(
    local_state, tmp_path: Path, monkeypatch
) -> None:
    pending = local_state(
        roles=("pilot",),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="managed",
        ),
    )
    copy_role_configs(pending)
    path = tmp_path / "state.json"
    generate_role_configs(pending)
    sync_provisioning_required(pending)
    pending.save(path)
    marker = pending.prefix_path / "security/provisioning-required"
    before = marker.read_bytes()
    trusted = replace(pending, dds=DdsSettings())

    def fail_save(self, _path=None):
        raise RuntimeError("injected state save failure")

    monkeypatch.setattr(InstallState, "save", fail_save)
    with pytest.raises(RuntimeError, match="injected"):
        network._apply_configuration_transaction(path, trusted)

    assert marker.read_bytes() == before


def test_network_trusted_configuration_clears_pending_marker(
    local_state, tmp_path: Path
) -> None:
    pending = local_state(
        roles=("ui",),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="managed",
        ),
    )
    copy_role_configs(pending)
    state_path = tmp_path / "state.json"
    sync_provisioning_required(pending)
    pending.save(state_path)
    marker = pending.prefix_path / "security/provisioning-required"

    result = network.main(
        (
            "--state",
            str(state_path),
            "configure",
            "--non-interactive",
            "--dds-security-profile",
            "trusted-network",
        )
    )

    assert result == 0
    assert not marker.exists()


def test_network_configuration_transaction_restores_files_on_failure(
    local_state, tmp_path: Path, monkeypatch
) -> None:
    state = local_state(roles=("pilot",))
    copy_role_configs(state)
    path = tmp_path / "state.json"
    generate_role_configs(state)
    state.save(path)
    config_path = generated_config_path(state, "pilot")
    state_before = path.read_bytes()
    config_before = config_path.read_bytes()

    def fail_after_write(updated):
        generated_config_path(updated, "pilot").write_text(
            "broken: true\n", encoding="utf-8"
        )
        raise RuntimeError("injected configuration failure")

    monkeypatch.setattr(network, "generate_role_configs", fail_after_write)
    updated = replace(state, dds=replace(state.dds, domain_id=19))

    with pytest.raises(RuntimeError, match="injected"):
        network._apply_configuration_transaction(path, updated)

    assert path.read_bytes() == state_before
    assert config_path.read_bytes() == config_before
