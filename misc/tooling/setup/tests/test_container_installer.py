from __future__ import annotations

from pathlib import Path

import yaml

from elesim_setup.container_installer import ContainerInstaller, build_container_plan
from elesim_setup.state import (
    ComputeSettings,
    NetworkSettings,
    SecuritySettings,
    TurnSettings,
)


def _compose(state) -> dict:
    path = state.prefix_path / "containers/compose.yaml"
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_container_dry_run_reports_plan_without_writing(local_state) -> None:
    state = local_state(roles=("router", "simulator"), install_mode="container")
    logs: list[str] = []

    ContainerInstaller(state, dry_run=True, log=logs.append).run()

    assert not state.prefix_path.exists()
    assert any("Compose" in action.title for action in build_container_plan(state))
    assert any("DRY-RUN" in line for line in logs)


def test_container_context_contains_only_protocol_and_owned_deployment(local_state) -> None:
    state = local_state(
        roles=("router", "controller", "simulator"),
        install_mode="container",
    )

    ContainerInstaller(state).run()

    for role in state.roles:
        context = state.prefix_path / "containers/build" / role
        assert (context / "protocol/pyproject.toml").is_file()
        assert (context / "application/pyproject.toml").is_file()
        assert (context / "Dockerfile").is_file()
        assert not (context / "simulator").exists()
        assert not (context / "robot").exists()
        assert not (context / "ui").exists()

    controller = state.prefix_path / "containers/build/controller/application"
    simulator = state.prefix_path / "containers/build/simulator/application"
    assert not (controller / "config").exists()
    assert not (simulator / "config").exists()
    assert (controller / "src/elesim_controller/config/__init__.py").is_file()
    assert (simulator / "src/elesim_simulator/config/__init__.py").is_file()


def test_compute_context_locks_tested_torch_and_robotpkg_pinocchio(local_state) -> None:
    state = local_state(
        roles=("simulator", "controller"),
        install_mode="container",
        compute=ComputeSettings(gpu_mode="cpu"),
    )

    ContainerInstaller(state).run()

    simulator = state.prefix_path / "containers/build/simulator"
    controller = state.prefix_path / "containers/build/controller"
    simulator_lock = (simulator / "requirements.lock").read_text(encoding="utf-8")
    controller_lock = (controller / "requirements.lock").read_text(encoding="utf-8")
    dockerfile = (simulator / "Dockerfile").read_text(encoding="utf-8")

    assert "torch==2.12.1" in simulator_lock
    assert "pin==" not in simulator_lock
    assert "torch==2.12.1" in controller_lock
    assert "torchvision==0.27.1" in controller_lock
    assert "robotpkg-py310-pinocchio" in dockerfile
    assert "curl -fsSL http://robotpkg" not in dockerfile
    assert (simulator / "robotpkg.asc").is_file()
    assert "https://download.pytorch.org/whl/cpu" in dockerfile


def test_local_container_compose_uses_host_network_and_role_owned_configs(local_state) -> None:
    state = local_state(
        profile="local-sim",
        roles=("router", "simulator", "controller", "ui"),
        install_mode="container",
    )

    ContainerInstaller(state).run()
    compose = _compose(state)

    assert compose["name"].startswith("elesim-")
    assert set(compose["services"]) == {
        "router",
        "simulator",
        "controller",
        "ui",
        "tools",
    }
    for role in state.roles:
        service = compose["services"][role]
        assert service["network_mode"] == "host"
        assert service["image"].startswith("elesim-managed/")
        assert service["build"]["context"].endswith(f"/build/{role}")
        assert service["build"]["args"]["COMPUTE_MODE"] == "inherit"
    assert compose["services"]["simulator"]["build"]["args"]["BASE_IMAGE"] == (
        "ros:humble-ros-base-jammy"
    )
    assert compose["services"]["controller"]["build"]["args"]["BASE_IMAGE"] == (
        "python:3.10-slim-bookworm"
    )
    assert compose["services"]["simulator"]["volumes"][0].endswith(
        ":/opt/elesim/config:ro"
    )
    assert "/tmp/.X11-unix:/tmp/.X11-unix:rw" in compose["services"]["ui"]["volumes"]


def test_cpu_container_policy_does_not_request_gpu_runtime(local_state) -> None:
    state = local_state(
        roles=("simulator", "controller"),
        install_mode="container",
        compute=ComputeSettings(gpu_mode="cpu"),
    )

    ContainerInstaller(state).run()
    compose = _compose(state)

    for role in ("simulator", "controller"):
        service = compose["services"][role]
        assert "gpus" not in service
        assert service["environment"]["CUDA_VISIBLE_DEVICES"] == ""
        assert service["build"]["args"]["COMPUTE_MODE"] == "cpu"


def test_specific_gpu_policy_is_explicit_in_compose(local_state) -> None:
    state = local_state(
        roles=("simulator",),
        install_mode="container",
        compute=ComputeSettings(gpu_mode="specific", gpu_device="1"),
    )

    ContainerInstaller(state).run()
    service = _compose(state)["services"]["simulator"]

    reservation = service["deploy"]["resources"]["reservations"]["devices"][0]
    assert reservation["driver"] == "nvidia"
    assert reservation["device_ids"] == ["1"]
    assert reservation["capabilities"] == ["gpu"]
    assert "gpus" not in service
    assert "CUDA_VISIBLE_DEVICES" not in service["environment"]


def test_inherited_gpu_policy_forwards_optional_host_selection(local_state) -> None:
    state = local_state(
        roles=("simulator",),
        install_mode="container",
        compute=ComputeSettings(gpu_mode="inherit"),
    )

    ContainerInstaller(state).run()
    service = _compose(state)["services"]["simulator"]

    assert service["gpus"] == "all"
    assert service["environment"]["CUDA_VISIBLE_DEVICES"] is None


def test_container_wrappers_delegate_to_generated_compose_project(local_state) -> None:
    state = local_state(roles=("router",), install_mode="container")

    ContainerInstaller(state).run()

    up = (state.bin_path / "elesim-up").read_text(encoding="utf-8")
    doctor = (state.bin_path / "elesim-net").read_text(encoding="utf-8")
    assert "docker compose" in up
    assert "up -d" in up
    assert "run --rm --build tools elesim-net" in doctor


def test_managed_coturn_joins_compose_and_wrapper_lifecycle(local_state) -> None:
    credentials = local_state().prefix_path / "secrets"
    state = local_state(
        roles=("router",),
        install_mode="container",
        network=NetworkSettings(
            router_host="203.0.113.10",
            advertise_host="203.0.113.10",
            turn_urls=("turn:203.0.113.10:3478?transport=udp",),
        ),
        security=SecuritySettings(mode="curve", credentials_root=str(credentials)),
        turn=TurnSettings(
            mode="managed",
            realm="sim.example.com",
            public_host="203.0.113.10",
        ),
    )
    required = (
        credentials / "curve/router/router.key_secret",
        credentials / "curve/endpoints.yaml",
        credentials / "turn.secret",
    )
    for path in required:
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("test\n", encoding="utf-8")
    (credentials / "curve/authorized").mkdir(parents=True)

    ContainerInstaller(state).run()
    compose = _compose(state)

    assert "coturn" in compose["services"]
    service = compose["services"]["coturn"]
    assert service["image"].startswith("coturn/coturn:")
    assert service["network_mode"] == "host"
    assert service["environment"]["TURN_REALM"] == "sim.example.com"
    assert "turn.secret" in service["volumes"][0]
    assert len(service["command"]) == 1
    command = service["command"][0]
    assert '$$TURN_REALM' in command
    assert '$$TURN_PUBLIC_IP' in command
    assert '$$(cat /run/secrets/turn.secret)' in command
    assert "coturn" not in service.get("ports", ())
