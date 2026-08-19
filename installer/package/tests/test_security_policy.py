from __future__ import annotations

import os
import sys
import xml.etree.ElementTree as ET
from unittest.mock import patch

import pytest

from elesim_protocol import (
    DdsPeerNode,
    DdsRuntimeSettings,
    DdsTransportError,
    EndpointDescriptor,
)
from elesim_protocol.dds_transport import peer_node_key
from elesim_setup.security_policy import render_role_policy, write_role_policy


ENDPOINTS = {
    "pilot": "pilot-main",
    "sim": "sim-default",
    "ui": "ui-main",
    "robot": "robot-go2",
}


def _permissions(rendered: str, permission: str) -> set[str]:
    root = ET.fromstring(rendered)
    return {
        topic.text or ""
        for group in root.findall(f".//topics[@{permission}='ALLOW']")
        for topic in group.findall("topic")
    }


def _service_permissions(rendered: str, permission: str) -> set[str]:
    root = ET.fromstring(rendered)
    return {
        service.text or ""
        for group in root.findall(f".//services[@{permission}='ALLOW']")
        for service in group.findall("service")
    }


def test_pilot_policy_limits_motion_and_rgbd_direction() -> None:
    rendered = render_role_policy(
        system_id="elesim",
        role="pilot",
        endpoint_id=ENDPOINTS["pilot"],
        endpoints=ENDPOINTS,
    )
    root = ET.fromstring(rendered)
    assert root.find(".//enclave").attrib["path"] == (
        "/elesim/elesim/pilot/pilot_main"
    )
    assert root.find(".//profile").attrib == {
        "ns": "/elesim/v6",
        "node": "*",
    }
    published = _permissions(rendered, "publish")
    subscribed = _permissions(rendered, "subscribe")
    assert (
        f"/elesim/v6/peers/{peer_node_key('robot-go2')}/*/motion"
        in published
    )
    assert (
        f"/elesim/v6/peers/{peer_node_key('sim-default')}/*/motion"
        in published
    )
    assert (
        f"/elesim/v6/peers/{peer_node_key('ui-main')}/*/motion"
        not in published
    )
    assert "/elesim/sim_default/rgbd/frame" in subscribed
    assert "/elesim/robot_go2/rgbd/frame" in subscribed
    assert "*" not in published
    assert "/elesim/v6/*/get_parameters" in _service_permissions(
        rendered, "reply"
    )
    assert not _service_permissions(rendered, "request")
    assert not root.findall(".//actions")


def _real_peer_prefix(endpoint_id: str, role: str) -> str:
    """Run the real constructor through prefix creation, then stop at ROS import."""

    node = object.__new__(DdsPeerNode)
    with patch.dict(
        os.environ,
        {"RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp", "ROS_DOMAIN_ID": "0"},
    ), patch.dict(sys.modules, {"rclpy": None}):
        with pytest.raises(DdsTransportError, match="ROS 2/DDS runtime"):
            DdsPeerNode.__init__(
                node,
                EndpointDescriptor(endpoint_id, role),
                settings=DdsRuntimeSettings(),
                boot_id="boot-a",
            )
    return node.topic_prefix


def test_policy_carrier_patterns_match_real_dds_peer_prefixes() -> None:
    rendered = render_role_policy(
        system_id="elesim",
        role="pilot",
        endpoint_id=ENDPOINTS["pilot"],
        endpoints=ENDPOINTS,
    )
    published = _permissions(rendered, "publish")
    subscribed = _permissions(rendered, "subscribe")

    pilot_prefix = _real_peer_prefix("pilot-main", "pilot")
    pilot_pattern = pilot_prefix.rsplit("/", 1)[0] + "/*"
    robot_prefix = _real_peer_prefix("robot-go2", "robot")
    robot_pattern = robot_prefix.rsplit("/", 1)[0] + "/*"

    assert f"{pilot_pattern}/control" in subscribed
    assert f"{pilot_pattern}/motion" in subscribed
    assert f"{robot_pattern}/control" in published
    assert f"{robot_pattern}/motion" in published
    assert "/elesim/pilot_main/rgbd/frame" not in subscribed
    assert "/elesim/robot_go2/rgbd/frame" in subscribed


@pytest.mark.parametrize("role", ("ui", "sim", "robot"))
def test_non_pilot_policies_cannot_publish_motion(role: str) -> None:
    rendered = render_role_policy(
        system_id="elesim",
        role=role,
        endpoint_id=ENDPOINTS[role],
        endpoints=ENDPOINTS,
    )
    assert not any(
        topic.endswith("/motion") for topic in _permissions(rendered, "publish")
    )


def test_policy_accepts_active_simulation_roles_and_rejects_invalid_endpoints() -> None:
    rendered = render_role_policy(
        system_id="elesim",
        role="ui",
        endpoint_id="ui-main",
        endpoints={
            "pilot": "pilot-main",
            "sim": "sim-main",
            "ui": "ui-main",
        },
    )
    assert "/elesim/robot_go2/rgbd/frame" not in _permissions(rendered, "subscribe")
    assert not any("robot" in topic for topic in _permissions(rendered, "publish"))

    with pytest.raises(ValueError, match="non-empty subset"):
        render_role_policy(
            system_id="elesim",
            role="ui",
            endpoint_id="ui-main",
            endpoints={},
        )
    with pytest.raises(ValueError, match="not present"):
        render_role_policy(
            system_id="elesim",
            role="robot",
            endpoint_id="robot-go2",
            endpoints={"ui": "ui-main"},
        )


def test_policy_rejects_canonical_colliding_endpoints() -> None:
    colliding = dict(ENDPOINTS, ui="sim_default")
    with pytest.raises(ValueError, match="collide"):
        render_role_policy(
            system_id="elesim",
            role="ui",
            endpoint_id="sim_default",
            endpoints=colliding,
        )


def test_write_role_policy_is_private_and_parseable(tmp_path) -> None:
    path = write_role_policy(
        tmp_path / "policies/ui.xml",
        system_id="elesim",
        role="ui",
        endpoint_id="ui-main",
        endpoints=ENDPOINTS,
    )

    assert path.stat().st_mode & 0o777 == 0o600
    assert path.parent.stat().st_mode & 0o777 == 0o700
    assert ET.parse(path).getroot().attrib["version"] == "0.2.0"
