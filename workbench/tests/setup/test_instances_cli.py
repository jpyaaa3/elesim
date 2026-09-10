from __future__ import annotations

import json
from argparse import Namespace
from types import SimpleNamespace
from pathlib import Path

import pytest

from elesim_setup.cli import (
    _docker_instance_service_verifier,
    _instance_from_args,
    _scoped_security_stage_root,
    main,
)
from elesim_setup.instances import InstanceEndpoint, InstanceRegistry, InstanceState
from elesim_setup.state import DdsSettings, InstallState, NetworkSettings


def _state(system: str, domain: int) -> InstanceState:
    return InstanceState(system, "a" * 64, (InstanceEndpoint("pilot", f"{system}-ep"),), domain)


def test_instances_cli_lists_all_and_selects_one(tmp_path, capsys):
    registry = InstanceRegistry(tmp_path)
    registry.save(_state("alpha", 1))
    registry.save(_state("beta", 2))

    assert main(["instances", "--prefix", str(tmp_path)]) == 0
    listed = json.loads(capsys.readouterr().out)
    assert [entry["system_id"] for entry in listed] == ["alpha", "beta"]

    assert main(["instances", "--prefix", str(tmp_path), "--system", "beta"]) == 0
    selected = json.loads(capsys.readouterr().out)
    assert selected["system_id"] == "beta"


def test_instances_cli_missing_prefix_does_not_create(tmp_path, capsys):
    prefix = tmp_path / "missing"
    assert main(["instances", "--prefix", str(prefix)]) == 2
    assert not prefix.exists()
    assert "unavailable" in capsys.readouterr().err


def test_instances_cli_system_errors_are_nonzero(tmp_path, capsys):
    InstanceRegistry(tmp_path).save(_state("alpha", 1))
    assert main(["instances", "--prefix", str(tmp_path), "--system", "missing"]) == 2
    assert "state.json" in capsys.readouterr().err


def test_instance_service_verifier_is_read_only_and_exact(monkeypatch, tmp_path):
    calls = []

    def fake_run(command, **kwargs):
        calls.append((command, kwargs))
        return SimpleNamespace(
            returncode=0,
            stdout=json.dumps([
                {"Service": "svc-alpha-pilot", "State": "exited"},
                {"Service": "svc-alpha-ui", "State": "running"},
                {"Service": "foreign", "State": "running"},
            ]),
            stderr="",
        )

    monkeypatch.setattr("elesim_setup.cli.subprocess.run", fake_run)
    result = _docker_instance_service_verifier(
        tmp_path / "compose.yaml",
        "elesim-runtime-scoped",
        ("svc-alpha-pilot", "svc-alpha-sim"),
    )
    assert result == {"svc-alpha-pilot": True, "svc-alpha-sim": False}
    assert calls[0][0] == (
        "docker",
        "compose",
        "--project-name",
        "elesim-runtime-scoped",
        "--file",
        str(tmp_path / "compose.yaml"),
        "ps",
        "--all",
        "--format",
        "json",
    )
    assert calls[0][1] == {"capture_output": True, "text": True, "check": False}


def test_manager_security_stage_is_exactly_system_and_generation_scoped(tmp_path):
    state = InstallState(
        profile="custom",
        roles=("pilot",),
        prefix=str(tmp_path / "prefix"),
        bin_dir=str(tmp_path / "bin"),
        source_root=str(tmp_path),
    )
    expected = (
        state.prefix_path
        / "maintenance"
        / ".connection-scoped"
        / "alpha"
        / "g1"
    )
    expected.mkdir(parents=True)

    assert _scoped_security_stage_root(
        state,
        system_id="alpha",
        generation="g1",
        supplied=expected,
    ) == expected

    foreign = state.prefix_path / "instances" / "alpha" / "security"
    foreign.mkdir(parents=True)
    with pytest.raises(ValueError, match="must match"):
        _scoped_security_stage_root(
            state,
            system_id="alpha",
            generation="g1",
            supplied=foreign,
        )


def test_instance_cli_cleans_only_requested_manager_staging_generation(
    tmp_path, monkeypatch
):
    prefix = tmp_path / "prefix"
    state = InstallState(
        profile="custom",
        roles=("pilot",),
        prefix=str(prefix),
        bin_dir=str(tmp_path / "bin"),
        source_root=str(tmp_path),
    )
    selected = prefix / "maintenance" / ".connection-scoped" / "alpha" / "g1"
    foreign_generation = prefix / "maintenance" / ".connection-scoped" / "alpha" / "g2"
    foreign_system = prefix / "maintenance" / ".connection-scoped" / "bravo" / "g1"
    for path in (selected, foreign_generation, foreign_system):
        path.mkdir(parents=True)
        (path / "partial").write_text("upload", encoding="utf-8")
    monkeypatch.setattr(
        "elesim_setup.cli._load_instance_context",
        lambda _state_path: (state, SimpleNamespace(install_uuid="unused")),
    )

    argv = [
        "instance",
        "cleanup-staging",
        "--system",
        "alpha",
        "--security-generation",
        "g1",
    ]
    assert main(argv) == 0
    assert not selected.exists()
    assert foreign_generation.exists()
    assert foreign_system.exists()
    # Cleanup is retryable when an upload failed before creating any file.
    assert main(argv) == 0


def test_instance_cli_copies_dds_defaults_explicitly(tmp_path):
    state = InstallState(
        profile="custom", roles=("pilot",), prefix=str(tmp_path / "prefix"),
        bin_dir=str(tmp_path / "bin"), source_root=str(tmp_path),
        network=NetworkSettings(),
        dds=DdsSettings(
            domain_id=17, discovery_mode="static", static_peers=("10.0.0.9",),
            interface="tailscale0",
        ),
    )
    args = Namespace(
        system="alpha", release="a" * 64, endpoint=["pilot:alpha-p"],
        graph_endpoint=["pilot:alpha-p", "sim:remote-s", "ui:remote-ui"],
        domain_id=None, rmw=None, discovery_mode=None, static_peer=None,
        interface=None, security_profile=None, security_generation=None,
    )
    instance = _instance_from_args(args, state)
    assert instance.domain_id == 17
    assert instance.discovery_mode == "static"
    assert instance.static_peers == ("10.0.0.9",)
    assert instance.interface == "tailscale0"
    assert (instance.pilot_id, instance.sim_id, instance.ui_id) == (
        "alpha-p", "remote-s", "remote-ui"
    )


def test_instance_cli_rejects_graph_role_ids_that_canonicalize_to_one_name(
    tmp_path,
):
    state = InstallState(
        profile="custom",
        roles=("pilot",),
        prefix=str(tmp_path / "prefix"),
        bin_dir=str(tmp_path / "bin"),
        source_root=str(tmp_path),
    )
    args = Namespace(
        system="alpha",
        release="a" * 64,
        endpoint=["pilot:a-b"],
        graph_endpoint=["pilot:a-b", "sim:a_b", "ui:ui-1"],
        domain_id=None,
        rmw=None,
        discovery_mode=None,
        static_peer=None,
        interface=None,
        security_profile=None,
        security_generation=None,
    )

    with pytest.raises(ValueError, match="canonicalization"):
        _instance_from_args(args, state)


def test_manager_staged_sros2_registration_uses_private_validation_prefix(
    tmp_path, monkeypatch
):
    prefix = tmp_path / "install"
    maintenance_stage = prefix / "maintenance" / ".connection-scoped" / "alpha" / "g1"
    for role in ("pilot", "sim"):
        (maintenance_stage / "apps" / role / "keystore").mkdir(parents=True)
    state = InstallState(
        profile="custom",
        roles=("pilot", "sim"),
        prefix=str(prefix),
        bin_dir=str(tmp_path / "bin"),
        source_root=str(tmp_path),
        dds=DdsSettings(security_profile="sros2", security_provisioning="managed"),
    )
    manifest = SimpleNamespace(install_uuid="01234567-89ab-cdef-0123-456789abcdef")
    release = SimpleNamespace()
    security_release = "a" * 64
    runtime_calls = []

    for role, endpoint in (("pilot", "pilot-1"), ("sim", "sim-1")):
        view = maintenance_stage / "apps" / role / "keystore"
        (view / "public").mkdir(parents=True)
        (view / "public" / "identity_ca.cert.pem").write_text("public", encoding="utf-8")
        enclave = view / "enclaves" / "elesim" / "alpha" / role / endpoint.replace("-", "_")
        enclave.mkdir(parents=True)
        (enclave / "cert.pem").write_text("cert", encoding="utf-8")
        (enclave / "key.pem").write_text("key", encoding="utf-8")

    class Runtime:
        def register(self, instance, selected_release, *, security_result=None):
            final_security = state.prefix_path / "instances" / instance.system_id / "security"
            assert not final_security.exists()
            assert security_result is not None
            assert (Path(security_result) / "manifest.json").is_file()
            runtime_calls.append((instance, selected_release, security_result))

    monkeypatch.setattr("elesim_setup.cli._load_instance_runtime", lambda _path: (state, manifest, Runtime()))
    monkeypatch.setattr("elesim_setup.cli.load_release", lambda _path: release)
    monkeypatch.setattr("elesim_setup.cli.release_key", lambda _release: security_release)

    result = main(
        [
            "instance",
            "register",
            "--system",
            "alpha",
            "--release",
            security_release,
            "--security-profile",
            "sros2",
            "--security-generation",
            "g1",
            "--security-bundle-root",
            str(maintenance_stage),
            "--endpoint",
            "pilot:pilot-1",
            "--endpoint",
            "sim:sim-1",
        ]
    )

    assert result == 0
    assert len(runtime_calls) == 1
    _, selected_release, security_result = runtime_calls[0]
    assert selected_release is release
    private_generation = Path(security_result)
    private_prefix = private_generation.parents[4]
    assert private_prefix.name.startswith(".validated-instance-security-")
    assert private_prefix.parent == state.prefix_path / "maintenance"
    assert private_generation.name == "g1"
    assert not maintenance_stage.exists()
    assert not private_prefix.exists()


def _turn_args(tmp_path, **overrides):
    values = dict(
        system="alpha",
        release="a" * 64,
        endpoint=["sim:sim-1"],
        domain_id=None,
        rmw=None,
        discovery_mode=None,
        static_peer=None,
        interface=None,
        security_profile=None,
        security_generation=None,
        gpu_mode=None,
        gpu_device=None,
        turn_mode=None,
        turn_url=None,
        turn_realm=None,
        turn_public_host=None,
        turn_listen_port=None,
        turn_relay_min_port=None,
        turn_relay_max_port=None,
        turn_credential_file=None,
    )
    values.update(overrides)
    return Namespace(**values)


def test_instance_cli_managed_turn_arguments_are_per_instance(tmp_path):
    state = InstallState(
        profile="custom", roles=("sim",), prefix=str(tmp_path / "prefix"),
        bin_dir=str(tmp_path / "bin"), source_root=str(tmp_path),
        dds=DdsSettings(security_profile="sros2", security_provisioning="managed"),
    )
    instance = _instance_from_args(
        _turn_args(
            tmp_path,
            turn_mode="managed",
            turn_url=["turn:relay.example:3478?transport=udp"],
            turn_realm="alpha.example",
            turn_public_host="relay.example",
            turn_listen_port=3479,
            turn_relay_min_port=41000,
            turn_relay_max_port=41039,
        ),
        state,
    )

    assert instance.turn.mode == "managed"
    assert instance.turn.realm == "alpha.example"
    assert instance.turn.public_host == "relay.example"
    assert instance.turn.listen_port == 3479
    assert instance.turn.relay_min_port == 41000
    assert instance.turn.relay_max_port == 41039
    assert instance.turn.secret_file == str(
        state.prefix_path / "instances" / "alpha" / "secrets" / "turn.secret"
    )
    assert instance.turn_urls == ("turn:relay.example:3478?transport=udp",)


def test_instance_cli_external_turn_arguments_are_per_instance(tmp_path):
    state = InstallState(
        profile="custom", roles=("sim",), prefix=str(tmp_path / "prefix"),
        bin_dir=str(tmp_path / "bin"), source_root=str(tmp_path),
    )
    credential = tmp_path / "turn-credentials.json"
    instance = _instance_from_args(
        _turn_args(
            tmp_path,
            turn_mode="external",
            turn_url=["turns:relay.example:5349"],
            turn_credential_file=str(credential),
        ),
        state,
    )

    assert instance.turn.mode == "external"
    assert instance.turn.credential_file == str(credential)
    assert instance.turn_urls == ("turns:relay.example:5349",)
