from __future__ import annotations

from pathlib import Path

import yaml

from elesim_setup.container_installer import ContainerInstaller, build_container_plan
from elesim_setup.state import DdsSettings, NetworkSettings, TurnSettings


def _compose(state) -> dict:
    return yaml.safe_load(
        (state.prefix_path / "containers/compose.yaml").read_text(encoding="utf-8")
    )


def test_container_plan_is_router_free(local_state) -> None:
    state = local_state(
        roles=("simulator", "controller"),
        install_mode="container",
    )
    plan = build_container_plan(state)
    rendered = "\n".join(action.detail for action in plan)

    assert "simulator" in rendered
    assert "controller" in rendered
    assert "router" not in {action.title.lower() for action in plan}


def test_container_install_generates_ros_overlay_contexts_and_dds_environment(
    local_state,
) -> None:
    state = local_state(
        roles=("simulator", "controller", "ui"),
        install_mode="container",
    )

    ContainerInstaller(state).run()
    compose = _compose(state)

    assert set(compose["services"]) == {"simulator", "controller", "ui", "tools"}
    for role in state.roles:
        service = compose["services"][role]
        assert service["build"]["args"]["BASE_IMAGE"] == "ros:humble-ros-base-jammy"
        assert service["environment"]["ROS_DOMAIN_ID"] == "0"
        assert service["environment"]["RMW_IMPLEMENTATION"] == "rmw_cyclonedds_cpp"
        assert service["environment"]["CYCLONEDDS_URI"] == (
            "file:///opt/elesim/config/cyclonedds.xml"
        )
        assert "depends_on" not in service
        context = state.prefix_path / f"containers/build/{role}"
        assert (context / "interfaces/elesim_interfaces/package.xml").is_file()
    tools = state.prefix_path / "containers/build/tools"
    assert (tools / "interfaces/elesim_interfaces/msg/RgbdFrame.msg").is_file()


def test_static_discovery_is_exported_to_every_service(local_state) -> None:
    state = local_state(
        roles=("controller", "ui"),
        install_mode="container",
        dds=DdsSettings(
            discovery_mode="static",
            static_peers=("192.0.2.10", "192.0.2.11"),
            interface="eth1",
        ),
    )

    ContainerInstaller(state).run()

    for service in _compose(state)["services"].values():
        environment = service["environment"]
        assert environment["ELESIM_DDS_DISCOVERY_MODE"] == "static"
        assert environment["ELESIM_DDS_STATIC_PEERS"] == "192.0.2.10,192.0.2.11"
        assert environment["ELESIM_DDS_NETWORK_INTERFACE"] == "eth1"


def test_managed_coturn_is_owned_by_simulator_and_shares_only_turn_secret(
    local_state,
    tmp_path: Path,
) -> None:
    secret = tmp_path / "install/secrets/turn.secret"
    keystore = tmp_path / "sros2"
    keystore.mkdir()
    state = local_state(
        roles=("simulator",),
        install_mode="container",
        network=NetworkSettings(
            turn_urls=("turn:turn.example.com:3478?transport=udp",),
        ),
        dds=DdsSettings(
            security_profile="sros2",
            keystore=str(keystore),
            enclave="/elesim",
        ),
        turn=TurnSettings(
            mode="managed",
            realm="elesim.local",
            public_host="turn.example.com",
            secret_file=str(secret),
        ),
    )

    ContainerInstaller(state).run()
    compose = _compose(state)

    assert secret.is_file()
    assert secret.stat().st_mode & 0o777 == 0o600
    assert compose["services"]["coturn"]["depends_on"] == ["simulator"]
    assert f"{secret}:/run/secrets/turn.secret:ro" in (
        compose["services"]["coturn"]["volumes"]
    )
    assert f"{secret}:/run/secrets/turn.secret:ro" in (
        compose["services"]["simulator"]["volumes"]
    )
    assert "router" not in str(compose).lower()


def test_external_turn_credentials_are_mounted_only_into_simulator(
    local_state,
    tmp_path: Path,
) -> None:
    credentials = tmp_path / "turn.credentials.json"
    credentials.write_text(
        '{"username":"lab-user","credential":"lab-password"}\n',
        encoding="utf-8",
    )
    state = local_state(
        roles=("simulator", "controller", "ui"),
        install_mode="container",
        network=NetworkSettings(
            turn_urls=("turn:relay.example.com:3478?transport=udp",),
        ),
        turn=TurnSettings(
            mode="external",
            credential_file=str(credentials),
        ),
    )

    ContainerInstaller(state).run()
    compose = _compose(state)
    mount = f"{credentials}:/run/secrets/turn.credentials.json:ro"

    assert credentials.stat().st_mode & 0o777 == 0o600
    assert mount in compose["services"]["simulator"]["volumes"]
    assert mount not in compose["services"]["controller"]["volumes"]
    assert mount not in compose["services"]["ui"]["volumes"]
    assert "coturn" not in compose["services"]


def test_sros2_environment_and_keystore_mount_are_role_specific(
    local_state,
    tmp_path: Path,
) -> None:
    keystore = tmp_path / "sros2"
    keystore.mkdir()
    state = local_state(
        roles=("controller", "ui"),
        install_mode="container",
        dds=DdsSettings(
            security_profile="sros2",
            keystore=str(keystore),
            enclave="/prod",
        ),
    )

    ContainerInstaller(state).run()
    services = _compose(state)["services"]

    assert services["controller"]["environment"]["ROS_SECURITY_ENCLAVE_OVERRIDE"] == (
        "/prod/controller_main"
    )
    assert services["ui"]["environment"]["ROS_SECURITY_ENCLAVE_OVERRIDE"] == (
        "/prod/ui_main"
    )
    assert f"{keystore}:{keystore}:ro" in services["controller"]["volumes"]


def test_specific_gpu_uses_one_compose_device_reservation(local_state) -> None:
    from elesim_setup.state import ComputeSettings

    state = local_state(
        roles=("simulator",),
        install_mode="container",
        compute=ComputeSettings(gpu_mode="specific", gpu_device="GPU-abc"),
    )

    ContainerInstaller(state).run()
    service = _compose(state)["services"]["simulator"]

    device = service["deploy"]["resources"]["reservations"]["devices"][0]
    assert device["device_ids"] == ["GPU-abc"]
    assert "CUDA_VISIBLE_DEVICES" not in service["environment"]


def test_container_dry_run_does_not_write_prefix(local_state) -> None:
    state = local_state(install_mode="container")

    ContainerInstaller(state, dry_run=True).run()

    assert not state.prefix_path.exists()
