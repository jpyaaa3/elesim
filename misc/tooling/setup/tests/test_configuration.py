from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import yaml

from elesim_setup.configuration import (
    dds_enclave,
    dds_node_key,
    generate_role_configs,
    generated_app_config_path,
    generated_dds_config_path,
    rgbd_topic,
)
from elesim_setup.state import DdsSettings, NetworkSettings, TurnSettings

from conftest import copy_role_configs


def _load(path: Path) -> dict:
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def test_generated_configs_use_dds_and_remove_router_tcp_fields(local_state) -> None:
    state = local_state(roles=("simulator", "controller", "ui"))
    copy_role_configs(state)

    written = generate_role_configs(state)

    controller = _load(written["controller"])
    simulator = _load(written["simulator"])
    ui = _load(written["ui"])
    for payload in (controller, simulator, ui):
        assert payload["dds"]["domain_id"] == 0
        assert payload["dds"]["rmw_implementation"] == "rmw_cyclonedds_cpp"
        assert payload["dds"]["network_interface"] == ""
        assert payload["dds"]["vendor_config"].endswith("cyclonedds.xml")
        assert "security" not in payload
        assert "server_endpoint" not in payload.get("runtime", {})
    streams = simulator["runtime"]["streams"]
    assert streams["rgbd_topic"] == "/elesim/sim_default/rgbd/frame"
    assert "rgbd_bind" not in streams
    assert "rgbd_advertise" not in streams
    assert ui["runtime"]["controller_id"] == "controller-main"


def test_cyclonedds_xml_contains_interface_and_static_peers(
    local_state,
) -> None:
    state = local_state(
        roles=("controller",),
        dds=DdsSettings(
            domain_id=27,
            discovery_mode="static",
            static_peers=("192.0.2.10", "sim.example.com"),
            interface="eth1",
        ),
    )
    copy_role_configs(state)

    generate_role_configs(state)
    root = ET.parse(generated_dds_config_path(state, "controller")).getroot()

    domain = root.find("Domain")
    assert domain is not None
    assert domain.attrib["id"] == "27"
    assert domain.findtext("General/AllowMulticast") == "false"
    interface = domain.find("General/Interfaces/NetworkInterface")
    assert interface is not None and interface.attrib["name"] == "eth1"
    assert [
        peer.attrib["Address"]
        for peer in domain.findall("Discovery/Peers/Peer")
    ] == ["192.0.2.10", "sim.example.com"]


def test_container_managed_turn_uses_simulator_owned_secret_mount_path(
    local_state,
    tmp_path,
) -> None:
    state = local_state(
        roles=("simulator",),
        install_mode="container",
        network=NetworkSettings(
            turn_urls=("turn:turn.example.com:3478?transport=udp",),
        ),
        dds=DdsSettings(
            security_profile="sros2",
            keystore=str(tmp_path / "sros2"),
            enclave="/lab",
        ),
        turn=TurnSettings(
            mode="managed",
            realm="lab.example",
            public_host="turn.example.com",
            secret_file=str(tmp_path / "turn.secret"),
        ),
    )
    copy_role_configs(state)

    simulator = _load(generate_role_configs(state)["simulator"])

    assert simulator["turn"] == {
        "urls": ["turn:turn.example.com:3478?transport=udp"],
        "realm": "lab.example",
        "static_auth_secret_file": "/run/secrets/turn.secret",
    }
    assert simulator["dds"]["vendor_config"] == "/opt/elesim/config/cyclonedds.xml"
    assert simulator["dds"]["enclave"] == "/lab/sim_default"


def test_container_external_turn_mounts_credentials_only_into_simulator(
    local_state,
    tmp_path,
) -> None:
    credentials = tmp_path / "turn.credentials.json"
    state = local_state(
        roles=("simulator",),
        install_mode="container",
        network=NetworkSettings(
            turn_urls=("turn:relay.example.com:3478?transport=udp",),
        ),
        turn=TurnSettings(
            mode="external",
            credential_file=str(credentials),
        ),
    )
    copy_role_configs(state)

    simulator = _load(generate_role_configs(state)["simulator"])

    assert simulator["turn"] == {
        "urls": ["turn:relay.example.com:3478?transport=udp"],
        "credential_file": "/run/secrets/turn.credentials.json",
    }


def test_custom_endpoint_ids_drive_node_keys_and_rgbd_topics(local_state) -> None:
    state = local_state(
        roles=("simulator", "controller"),
        network=NetworkSettings(
            simulator_id="Sim West-2",
            controller_id="Controller.A",
        ),
    )

    assert dds_node_key(state, "simulator") == "sim_west_2"
    assert dds_node_key(state, "controller") == "controller_a"
    assert rgbd_topic(state, "simulator") == "/elesim/sim_west_2/rgbd/frame"


def test_sros2_enclave_is_role_specific(local_state, tmp_path) -> None:
    state = local_state(
        roles=("ui",),
        dds=DdsSettings(
            security_profile="sros2",
            keystore=str(tmp_path / "sros2"),
            enclave="/elesim/prod",
        ),
    )

    assert dds_enclave(state, "ui") == "/elesim/prod/ui_main"
    assert dds_enclave(state, "doctor") == "/elesim/prod/doctor_main"


def test_generation_does_not_mutate_source_defaults(local_state) -> None:
    state = local_state(roles=("simulator",), profile="local-sim")
    source = state.source_path / "simulator/config/runtime.yaml"
    before = source.read_bytes()
    copy_role_configs(state)

    generate_role_configs(state)

    assert source.read_bytes() == before
    assert generated_app_config_path(state, "simulator").is_file()
