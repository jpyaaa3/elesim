from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import yaml

from conftest import copy_role_configs
from elesim_setup.configuration import (
    generate_role_configs,
    generated_app_config_path,
    host_is_loopback,
    missing_credentials,
    tcp_endpoint,
)
from elesim_setup.state import ComputeSettings, NetworkSettings, SecuritySettings


def _load(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_local_profile_generates_loopback_configs_without_touching_sources(local_state) -> None:
    state = local_state(
        profile="local-sim",
        roles=("router", "simulator", "controller", "ui"),
    )
    copy_role_configs(state)
    source_default = state.source_path / "router/config/default.yaml"
    before = source_default.read_bytes()

    written = generate_role_configs(state)

    router = _load(written["router"])
    simulator = _load(written["simulator"])
    controller = _load(written["controller"])
    ui = _load(written["ui"])
    assert router["router"]["bind_endpoint"] == "tcp://127.0.0.1:5558"
    assert simulator["runtime"]["streams"]["rgbd_bind"] == "tcp://127.0.0.1:5568"
    assert controller["runtime"]["server_endpoint"] == "tcp://127.0.0.1:5558"
    assert ui["runtime"]["simulator_id"] == "sim-default"
    assert all(not payload["security"]["allow_insecure_remote"] for payload in (router, simulator, controller, ui))
    assert source_default.read_bytes() == before


def test_curve_configs_use_role_specific_credentials(local_state, tmp_path: Path) -> None:
    credentials = tmp_path / "secrets"
    state = local_state(
        profile="compute",
        roles=("router", "simulator"),
        network=NetworkSettings(
            router_host="sim.example.com",
            advertise_host="sim.example.com",
            turn_urls=("turn:sim.example.com:3478?transport=udp",),
        ),
        security=SecuritySettings(mode="curve", credentials_root=str(credentials)),
    )
    copy_role_configs(state)

    written = generate_role_configs(state)
    router = _load(written["router"])
    simulator = _load(written["simulator"])

    assert router["router"]["bind_endpoint"] == "tcp://0.0.0.0:5558"
    assert router["security"]["curve_server_secret_file"].endswith("curve/router/router.key_secret")
    assert router["turn"]["static_auth_secret_file"].endswith("turn.secret")
    assert simulator["runtime"]["streams"]["rgbd_bind"] == "tcp://0.0.0.0:5568"
    assert simulator["runtime"]["streams"]["rgbd_advertise"] == "tcp://sim.example.com:5568"
    assert simulator["security"]["media_server_secret_file"].endswith("simulator-media.key_secret")
    assert not simulator["security"]["allow_insecure_remote"]


def test_insecure_lan_is_explicit_in_every_generated_config(local_state) -> None:
    state = local_state(
        roles=("controller", "ui", "robot"),
        network=NetworkSettings(router_host="192.0.2.10", advertise_host="192.0.2.30"),
        security=SecuritySettings(mode="insecure-lan"),
    )
    copy_role_configs(state)
    written = generate_role_configs(state)
    assert all(_load(path)["security"]["allow_insecure_remote"] for path in written.values())


def test_missing_credentials_lists_doctor_and_role_files(local_state, tmp_path: Path) -> None:
    state = local_state(
        roles=("ui",),
        network=NetworkSettings(router_host="server", advertise_host="laptop"),
        security=SecuritySettings(mode="curve", credentials_root=str(tmp_path / "keys")),
    )
    missing = {path.name for path in missing_credentials(state)}
    assert {"ui-main.key_secret", "router.key"}.issubset(missing)
    assert "doctor-main.key_secret" not in missing


def test_custom_controller_identity_is_used_for_router_and_media_keys(local_state, tmp_path: Path) -> None:
    state = local_state(
        roles=("controller",),
        network=NetworkSettings(
            router_host="server",
            advertise_host="laptop",
            controller_id="controller-lab",
        ),
        security=SecuritySettings(mode="curve", credentials_root=str(tmp_path / "keys")),
    )
    copy_role_configs(state)
    payload = _load(generate_role_configs(state)["controller"])
    assert payload["security"]["router_client_secret_file"].endswith(
        "controller-lab.key_secret"
    )
    assert payload["security"]["media_client_secret_file"].endswith(
        "controller-lab.key_secret"
    )


def test_endpoint_helpers_cover_ipv4_ipv6_and_loopback() -> None:
    assert tcp_endpoint("192.0.2.1", 5558) == "tcp://192.0.2.1:5558"
    assert tcp_endpoint("2001:db8::1", 5558) == "tcp://[2001:db8::1]:5558"
    assert host_is_loopback("::1")
    assert host_is_loopback("[::1]")
    assert host_is_loopback("localhost")
    assert not host_is_loopback("example.com")


def test_simulator_application_config_selects_gpu_or_cpu_without_editing_defaults(
    local_state,
) -> None:
    cpu_state = local_state(
        profile="compute",
        roles=("simulator",),
        compute=ComputeSettings(gpu_mode="cpu"),
    )
    copy_role_configs(cpu_state)
    generate_role_configs(cpu_state)
    cpu_bundle = _load(generated_app_config_path(cpu_state, "simulator"))
    assert cpu_bundle["extends"] == "config.remote.yaml"
    assert cpu_bundle["simulation"]["runtime"]["use_gpu"] is False
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (
            str(cpu_state.source_path / "packages/protocol/src"),
            str(cpu_state.source_path / "simulator/src"),
        )
    )
    subprocess.run(
        (
            sys.executable,
            "-c",
            "from elesim_simulator.config import load_app_config as f; "
            f"assert not f({str(generated_app_config_path(cpu_state, 'simulator'))!r})"
            ".sim_config.use_gpu",
        ),
        cwd=cpu_state.source_path,
        env=environment,
        check=True,
    )

    gpu_state = local_state(
        profile="local-sim",
        roles=("simulator",),
        compute=ComputeSettings(gpu_mode="specific", gpu_device="1"),
    )
    copy_role_configs(gpu_state)
    generate_role_configs(gpu_state)
    gpu_bundle = _load(generated_app_config_path(gpu_state, "simulator"))
    assert gpu_bundle["extends"] == "config.pc.yaml"
    assert gpu_bundle["simulation"]["runtime"]["use_gpu"] is True

    container_state = local_state(
        profile="local-sim",
        roles=("simulator",),
        install_mode="container",
    )
    copy_role_configs(container_state)
    generate_role_configs(container_state)
    container_bundle = _load(generated_app_config_path(container_state, "simulator"))
    assert container_bundle["extends"] == "config.remote.yaml"


def test_generated_configs_load_in_each_owning_deployment(local_state) -> None:
    state = local_state(
        profile="local-sim",
        roles=("router", "simulator", "controller", "ui", "robot"),
    )
    copy_role_configs(state)
    written = generate_role_configs(state)
    probes = {
        "router": "from elesim_router.config import load_config as f",
        "controller": "from elesim_controller.config import load_runtime_role_config as f",
        "ui": "from elesim_ui.config import load_config as f",
        "simulator": "from elesim_simulator.config import load_runtime_role_config as f",
        "robot": "from elesim_robot.config import load_config as f",
    }
    root = state.source_path
    for role, statement in probes.items():
        environment = os.environ.copy()
        environment["PYTHONPATH"] = os.pathsep.join(
            (str(root / "packages/protocol/src"), str(root / role / "src"))
        )
        subprocess.run(
            (
                sys.executable,
                "-c",
                f"{statement}; f({str(written[role])!r})",
            ),
            cwd=root,
            env=environment,
            check=True,
        )
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        (str(root / "packages/protocol/src"), str(root / "simulator/src"))
    )
    subprocess.run(
        (
            sys.executable,
            "-c",
            "from elesim_simulator.config import load_app_config as f; "
            f"f({str(generated_app_config_path(state, 'simulator'))!r})",
        ),
        cwd=root,
        env=environment,
        check=True,
    )
