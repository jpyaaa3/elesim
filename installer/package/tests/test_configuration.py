from __future__ import annotations

import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import yaml

from elesim_setup.configuration import (
    copy_role_config_tree,
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


def test_runtime_config_copy_excludes_only_public_templates(tmp_path: Path) -> None:
    templates = {
        "pilot": "runtime.public.example.yaml",
        "sim": "runtime.public.example.yaml",
        "ui": "public.example.yaml",
        "robot": "public.example.yaml",
    }
    for role, template in templates.items():
        source = tmp_path / "source" / role / "config"
        perception = source / "perception"
        perception.mkdir(parents=True)
        (source / "default.yaml").write_text("runtime: true\n", encoding="utf-8")
        (source / template).write_text("public: true\n", encoding="utf-8")
        yolo = perception / "detector.yolo.example.json"
        yolo.write_text("{}\n", encoding="utf-8")
        destination = tmp_path / "destination" / role / "config"
        destination.mkdir(parents=True)
        (destination / template).write_text("stale: true\n", encoding="utf-8")

        copy_role_config_tree(source, destination, role)

        assert (destination / "default.yaml").is_file()
        assert not (destination / template).exists()
        assert (destination / "perception/detector.yolo.example.json").is_file()


def test_runtime_config_copy_rejects_source_or_destination_symlinks(
    tmp_path: Path,
) -> None:
    source = tmp_path / "source"
    source.mkdir()
    (source / "runtime.yaml").write_text("runtime: true\n", encoding="utf-8")
    outside = tmp_path / "outside"
    outside.mkdir()
    (source / "escape").symlink_to(outside, target_is_directory=True)

    with pytest.raises(ValueError, match="symlink"):
        copy_role_config_tree(source, tmp_path / "destination", "pilot")

    safe_source = tmp_path / "safe-source"
    safe_source.mkdir()
    (safe_source / "runtime.yaml").write_text("runtime: true\n", encoding="utf-8")
    linked_destination = tmp_path / "linked-destination"
    linked_destination.symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError, match="symlink"):
        copy_role_config_tree(safe_source, linked_destination, "pilot")


def test_generated_configs_use_dds_and_remove_router_tcp_fields(local_state) -> None:
    state = local_state(roles=("sim", "pilot", "ui"))
    copy_role_configs(state)

    written = generate_role_configs(state)

    pilot = _load(written["pilot"])
    sim = _load(written["sim"])
    ui = _load(written["ui"])
    for payload in (pilot, sim, ui):
        assert payload["dds"]["domain_id"] == 0
        assert payload["dds"]["rmw_implementation"] == "rmw_cyclonedds_cpp"
        assert payload["dds"]["network_interface"] == ""
        assert payload["dds"]["vendor_config"].endswith("cyclonedds.xml")
        assert "security" not in payload
        assert "server_endpoint" not in payload.get("runtime", {})
    streams = sim["runtime"]["streams"]
    assert streams["rgbd_topic"] == "/elesim/sim_default/rgbd/frame"
    assert "rgbd_bind" not in streams
    assert "rgbd_advertise" not in streams
    assert ui["runtime"]["pilot_id"] == "pilot-main"


def test_cyclonedds_xml_contains_interface_and_static_peers(
    local_state,
) -> None:
    state = local_state(
        roles=("pilot",),
        dds=DdsSettings(
            domain_id=27,
            discovery_mode="static",
            static_peers=("192.0.2.10", "sim.example.com"),
            interface="eth1",
        ),
    )
    copy_role_configs(state)

    generate_role_configs(state)
    root = ET.parse(generated_dds_config_path(state, "pilot")).getroot()

    domain = root.find("Domain")
    assert domain is not None
    assert domain.attrib["id"] == "27"
    assert domain.findtext("General/AllowMulticast") == "false"
    interface = domain.find("General/Interfaces/NetworkInterface")
    assert interface is not None and interface.attrib == {"name": "eth1"}
    assert [
        peer.attrib["Address"]
        for peer in domain.findall("Discovery/Peers/Peer")
    ] == ["192.0.2.10", "sim.example.com"]


def test_cyclonedds_xml_preserves_direct_tailscale_bind(local_state) -> None:
    state = local_state(
        roles=("pilot",),
        dds=DdsSettings(interface="tailscale0"),
    )
    copy_role_configs(state)

    generate_role_configs(state)
    root = ET.parse(generated_dds_config_path(state, "pilot")).getroot()

    interface = root.find("Domain/General/Interfaces/NetworkInterface")
    assert interface is not None
    assert interface.attrib == {"name": "tailscale0"}


def test_cyclonedds_xml_omits_legacy_automatic_interface(local_state) -> None:
    state = local_state(
        roles=("pilot",),
        dds=DdsSettings(interface="automatic"),
    )
    copy_role_configs(state)

    generate_role_configs(state)
    root = ET.parse(generated_dds_config_path(state, "pilot")).getroot()

    assert root.find("Domain/General/Interfaces/NetworkInterface") is None


def test_container_managed_turn_uses_sim_owned_secret_mount_path(
    local_state,
    tmp_path,
) -> None:
    state = local_state(
        roles=("sim",),
        install_mode="container",
        network=NetworkSettings(
            turn_urls=("turn:turn.example.com:3478?transport=udp",),
        ),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="external",
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

    sim = _load(generate_role_configs(state)["sim"])

    assert sim["turn"] == {
        "urls": ["turn:turn.example.com:3478?transport=udp"],
        "realm": "lab.example",
        "static_auth_secret_file": "/run/secrets/turn.secret",
    }
    assert sim["dds"]["vendor_config"] == "/opt/elesim/config/cyclonedds.xml"
    assert sim["dds"]["enclave"] == "/lab/sim_default"
    assert sim["dds"]["keystore"] == str(
        state.prefix_path / "security/roles/sim"
    )


def test_pending_managed_turn_omits_runtime_credentials_until_manager_configures_it(
    local_state,
    tmp_path,
) -> None:
    state = local_state(
        roles=("sim",),
        install_mode="container",
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="managed",
        ),
        turn=TurnSettings(
            mode="managed",
            realm="elesim.local",
            secret_file=str(tmp_path / "turn.secret"),
        ),
    )
    copy_role_configs(state)

    sim = _load(generate_role_configs(state)["sim"])

    # Coturn is installed with the Sim role, but the manager must first choose
    # the current advertised address before Sim receives a usable relay config.
    assert sim["turn"] == {"urls": []}


def test_container_external_turn_mounts_credentials_only_into_sim(
    local_state,
    tmp_path,
) -> None:
    credentials = tmp_path / "turn.credentials.json"
    state = local_state(
        roles=("sim",),
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

    sim = _load(generate_role_configs(state)["sim"])

    assert sim["turn"] == {
        "urls": ["turn:relay.example.com:3478?transport=udp"],
        "credential_file": "/run/secrets/turn.credentials.json",
    }


def test_custom_endpoint_ids_drive_node_keys_and_rgbd_topics(local_state) -> None:
    state = local_state(
        roles=("sim", "pilot"),
        network=NetworkSettings(
            sim_id="Sim West-2",
            pilot_id="Pilot.A",
            ui_id="UI West-2",
            robot_id="Robot West-2",
        ),
    )

    assert dds_node_key(state, "sim") == "sim_west_2"
    assert dds_node_key(state, "pilot") == "pilot_a"
    assert dds_node_key(state, "ui") == "ui_west_2"
    assert dds_node_key(state, "robot") == "robot_west_2"
    assert rgbd_topic(state, "sim") == "/elesim/sim_west_2/rgbd/frame"


def test_ui_and_robot_runtime_configs_use_configured_endpoint_ids(local_state) -> None:
    network = NetworkSettings(ui_id="ui-field", robot_id="robot-field")
    ui_state = local_state(roles=("ui",), network=network)
    robot_state = local_state(roles=("robot",), network=network)
    copy_role_configs(ui_state)
    copy_role_configs(robot_state)

    ui_written = generate_role_configs(ui_state)
    robot_written = generate_role_configs(robot_state)

    assert _load(ui_written["ui"])["runtime"]["endpoint_id"] == "ui-field"
    assert _load(robot_written["robot"])["runtime"]["endpoint_id"] == "robot-field"
    assert _load(robot_written["robot"])["camera"]["topic"] == (
        "/elesim/robot_field/rgbd/frame"
    )


def test_sros2_enclave_is_role_specific(local_state, tmp_path) -> None:
    state = local_state(
        roles=("ui",),
        dds=DdsSettings(
            security_profile="sros2",
            security_provisioning="external",
            keystore=str(tmp_path / "sros2"),
            enclave="/elesim/prod",
        ),
    )

    assert dds_enclave(state, "ui") == "/elesim/prod/ui_main"
    assert dds_enclave(state, "doctor") == "/elesim/prod/doctor_main"


def test_managed_sros2_enclave_matches_authority_role_path(
    local_state, tmp_path
) -> None:
    bundle = tmp_path / "security" / "current" / "keystore"
    state = local_state(
        roles=("pilot", "ui"),
        dds=DdsSettings(
            system_id="prod",
            security_profile="sros2",
            security_provisioning="managed",
            security_generation="gen-7",
            security_bundle=str(bundle),
            keystore=str(bundle),
            enclave="/elesim/prod",
        ),
    )

    assert dds_enclave(state, "pilot") == "/elesim/prod/pilot/pilot_main"
    assert dds_enclave(state, "ui") == "/elesim/prod/ui/ui_main"


def test_generation_does_not_mutate_source_defaults(local_state) -> None:
    state = local_state(roles=("sim",), profile="local-sim")
    source = state.source_path / "sim/config/runtime.yaml"
    before = source.read_bytes()
    copy_role_configs(state)

    generate_role_configs(state)

    assert source.read_bytes() == before
    assert generated_app_config_path(state, "sim").is_file()
