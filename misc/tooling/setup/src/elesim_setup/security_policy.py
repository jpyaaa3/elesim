"""Render least-privilege SROS2 policies for the runtime-wired Elesim graph."""

from __future__ import annotations

import os
import tempfile
import xml.etree.ElementTree as ET
from collections.abc import Mapping
from pathlib import Path

from elesim_protocol.dds_transport import peer_node_key

from ._security_storage import EnclaveIdentity, SROS2_ROLES, secure_absolute


_CONTROL_TARGETS = {
    "controller": ("robot", "simulator", "ui"),
    "ui": ("controller", "simulator"),
    "simulator": ("controller", "ui"),
    "robot": ("controller",),
}
_MOTION_TARGETS = {
    "controller": ("robot", "simulator"),
    "ui": (),
    "simulator": (),
    "robot": (),
}


def render_role_policy(
    *,
    system_id: str,
    role: str,
    endpoint_id: str,
    endpoints: Mapping[str, str],
) -> str:
    """Return one complete SROS2 policy for a single role enclave.

    The current protocol-v5 runtime uses topics only. Generated-but-unwired ROS
    services/actions are deliberately absent from this policy.
    """

    identity = EnclaveIdentity(system_id, role, endpoint_id)
    endpoint_keys = _validated_endpoints(
        system_id, role, endpoint_id, endpoints
    )
    # Peer carrier topic keys are protocol identifiers, not enclave/RGBD path
    # components.  The protocol deliberately appends a digest to prevent two
    # differently spelled endpoint IDs from collapsing onto one ROS name.
    # Enclave paths and configured RGBD topics retain their stable, unhashed
    # operator-facing endpoint keys below.
    carrier_keys = {
        target_role: peer_node_key(str(endpoints[target_role]))
        for target_role in endpoint_keys
    }
    namespace = f"/{system_id}/v5"
    own_key = carrier_keys[role]

    publish = {
        f"{namespace}/discovery/endpoints",
        f"{namespace}/discovery/heartbeats",
        "/parameter_events",
        "/rosout",
    }
    subscribe = {
        f"{namespace}/discovery/endpoints",
        f"{namespace}/discovery/heartbeats",
        f"{namespace}/peers/{own_key}/*/control",
        f"{namespace}/peers/{own_key}/*/motion",
        "/parameter_events",
    }
    for target_role in _CONTROL_TARGETS[role]:
        if target_role not in endpoint_keys:
            continue
        publish.add(
            f"{namespace}/peers/{carrier_keys[target_role]}/*/control"
        )
    for target_role in _MOTION_TARGETS[role]:
        if target_role not in endpoint_keys:
            continue
        publish.add(
            f"{namespace}/peers/{carrier_keys[target_role]}/*/motion"
        )

    rgbd_topics = {
        target_role: f"/{system_id}/{endpoint_keys[target_role]}/rgbd/frame"
        for target_role in ("robot", "simulator")
        if target_role in endpoint_keys
    }
    if role in rgbd_topics:
        publish.add(rgbd_topics[role])
    # Every installed role may run the local active network doctor. This grants
    # read-only RGBD diagnostics without granting another role's publisher.
    subscribe.update(rgbd_topics.values())

    root = ET.Element("policy", {"version": "0.2.0"})
    enclaves = ET.SubElement(root, "enclaves")
    enclave = ET.SubElement(enclaves, "enclave", {"path": identity.enclave})
    profiles = ET.SubElement(enclave, "profiles")
    profile = ET.SubElement(
        profiles,
        "profile",
        {"ns": namespace, "node": "*"},
    )
    _topics(profile, "publish", publish)
    _topics(profile, "subscribe", subscribe)
    # rclpy creates these parameter servers for every peer/RGBD/doctor node.
    # They are implicit ROS runtime surfaces, not the unwired Elesim typed
    # service contracts.
    parameter_services = {
        f"{namespace}/*/{name}"
        for name in (
            "describe_parameters",
            "get_parameter_types",
            "get_parameters",
            "list_parameters",
            "set_parameters",
            "set_parameters_atomically",
        )
    }
    _services(profile, "reply", parameter_services)
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode", xml_declaration=True) + "\n"


def write_role_policy(
    destination: Path,
    *,
    system_id: str,
    role: str,
    endpoint_id: str,
    endpoints: Mapping[str, str],
) -> Path:
    """Atomically write one private policy input for ``create_permission``."""

    rendered = render_role_policy(
        system_id=system_id,
        role=role,
        endpoint_id=endpoint_id,
        endpoints=endpoints,
    )
    target = secure_absolute(Path(destination))
    if target.exists() and target.is_symlink():
        raise ValueError(f"refusing to replace policy symlink: {target}")
    target.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    target.parent.chmod(0o700)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=target.parent, prefix=f".{target.name}."
    )
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            handle.write(rendered)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, target)
    finally:
        if temporary.exists():
            temporary.unlink()
    return target


def _validated_endpoints(
    system_id: str,
    role: str,
    endpoint_id: str,
    endpoints: Mapping[str, str],
) -> dict[str, str]:
    active_roles = set(endpoints)
    if not active_roles or not active_roles.issubset(SROS2_ROLES):
        raise ValueError(
            "endpoints must be a non-empty subset of controller, simulator, ui and robot"
        )
    if role not in active_roles:
        raise ValueError(f"role {role!r} is not present in the active endpoints")
    identities = {
        role: EnclaveIdentity(system_id, role, str(endpoints[role]))
        for role in active_roles
    }
    if str(endpoints[role]) != endpoint_id:
        raise ValueError(f"endpoint_id does not match the {role} assignment")
    keys = [identity.endpoint_key for identity in identities.values()]
    if len(set(keys)) != len(keys):
        raise ValueError("endpoint IDs collide after ROS name canonicalization")
    return {role: identity.endpoint_key for role, identity in identities.items()}


def _topics(parent: ET.Element, permission: str, names: set[str]) -> None:
    attributes = {permission: "ALLOW"}
    group = ET.SubElement(parent, "topics", attributes)
    for name in sorted(names):
        ET.SubElement(group, "topic").text = name


def _services(parent: ET.Element, permission: str, names: set[str]) -> None:
    attributes = {permission: "ALLOW"}
    group = ET.SubElement(parent, "services", attributes)
    for name in sorted(names):
        ET.SubElement(group, "service").text = name


__all__ = ["render_role_policy", "write_role_policy"]
