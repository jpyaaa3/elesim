"""Generate deployment-owned YAML and DDS middleware configuration."""

from __future__ import annotations

import ipaddress
import os
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Mapping

import yaml

from .state import DdsSettings, InstallState


GENERATED_CONFIG = "installed.yaml"
GENERATED_RUNTIME = "runtime.installed.yaml"
GENERATED_APP = "app.installed.yaml"
GENERATED_DDS = "cyclonedds.xml"
_INVALID_ROS_NAME = re.compile(r"[^a-z0-9_]+")
PUBLIC_CONFIG_TEMPLATES = {
    "pilot": "runtime.public.example.yaml",
    "sim": "runtime.public.example.yaml",
    "ui": "public.example.yaml",
    "robot": "public.example.yaml",
}


@dataclass(frozen=True)
class RobotHostSettings:
    """Host-owned values that cannot be inferred from a portable Robot config."""

    robot_user: str
    bridge_user: str
    ros_workspace: Path
    unitree_interface: str = "eth0"
    unitree_domain_id: int = 1

    def validate(self) -> "RobotHostSettings":
        for name, value in (
            ("robot_user", self.robot_user),
            ("bridge_user", self.bridge_user),
        ):
            text = str(value).strip()
            if (
                not text
                or len(text) > 64
                or any(character.isspace() or character in ":/" for character in text)
            ):
                raise ValueError(f"{name} cannot form a safe local account name")
        workspace = self.ros_workspace.expanduser()
        if not workspace.is_absolute():
            raise ValueError("Unitree ROS 2 workspace must be an absolute path")
        interface = str(self.unitree_interface).strip()
        if (
            not interface
            or len(interface) > 128
            or any(character.isspace() or character == "/" for character in interface)
        ):
            raise ValueError("Unitree network interface must be one interface name")
        if (
            isinstance(self.unitree_domain_id, bool)
            or not 0 <= int(self.unitree_domain_id) <= 232
        ):
            raise ValueError("Unitree ROS domain ID must be in 0..232")
        return self


def role_directory(state: InstallState, role: str) -> Path:
    return state.prefix_path / "roles" / role


def copy_role_config_tree(source: Path, destination: Path, role: str) -> None:
    try:
        excluded = PUBLIC_CONFIG_TEMPLATES[role]
    except KeyError as exc:
        raise ValueError(f"unknown role: {role!r}") from exc
    _reject_symlink_path(source, name="role config source")
    if not source.is_dir():
        raise FileNotFoundError(source)
    _reject_symlink_tree(source, name="role config source")
    _reject_symlink_ancestors(destination, name="role config destination")
    if destination.exists():
        _reject_symlink_tree(destination, name="role config destination")
    destination.parent.mkdir(parents=True, exist_ok=True)
    excluded_destination = destination / excluded
    if excluded_destination.is_symlink():
        raise ValueError(
            f"role config destination must not contain a symlink: {excluded_destination}"
        )
    if excluded_destination.is_file():
        excluded_destination.unlink()
    elif excluded_destination.exists():
        raise ValueError(
            f"public config template destination must not be a directory: {excluded_destination}"
        )
    source_root = source.resolve()

    def ignore(directory: str, names: list[str]) -> set[str]:
        if Path(directory).resolve() == source_root and excluded in names:
            return {excluded}
        return set()

    shutil.copytree(source, destination, dirs_exist_ok=True, ignore=ignore)


def _reject_symlink_path(path: Path, *, name: str) -> None:
    if path.is_symlink():
        raise ValueError(f"{name} must not be a symlink: {path}")


def _reject_symlink_ancestors(path: Path, *, name: str) -> None:
    current = path
    while True:
        if current.is_symlink():
            raise ValueError(f"{name} contains a symlink ancestor: {current}")
        parent = current.parent
        if parent == current:
            break
        current = parent


def _reject_symlink_tree(root: Path, *, name: str) -> None:
    for directory, names, files in os.walk(root, followlinks=False):
        for child in (*names, *files):
            path = Path(directory) / child
            if path.is_symlink():
                raise ValueError(f"{name} contains a symlink: {path}")


def generated_config_path(state: InstallState, role: str) -> Path:
    name = GENERATED_RUNTIME if role in {"pilot", "sim"} else GENERATED_CONFIG
    return role_directory(state, role) / "config" / name


def generated_app_config_path(state: InstallState, role: str) -> Path:
    if role != "sim":
        raise ValueError(f"{role!r} does not have a generated application config")
    return role_directory(state, role) / "config" / GENERATED_APP


def generated_dds_config_path(state: InstallState, role: str) -> Path:
    return role_directory(state, role) / "config" / GENERATED_DDS


def dds_node_key(state: InstallState, role: str) -> str:
    values = {
        "pilot": state.network.pilot_id,
        "sim": state.network.sim_id,
        "ui": state.network.ui_id,
        "robot": state.network.robot_id,
        "doctor": "doctor-main",
    }
    try:
        raw = values[role]
    except KeyError as exc:
        raise ValueError(f"unknown role: {role}") from exc
    key = _INVALID_ROS_NAME.sub("_", str(raw).strip().lower().replace("-", "_"))
    key = key.strip("_")
    if not key:
        raise ValueError(f"{role} endpoint ID cannot form a ROS node key")
    if not key[0].isalpha():
        key = f"node_{key}"
    return key[:63]


def dds_enclave(state: InstallState, role: str) -> str:
    base = state.dds.enclave.rstrip("/")
    if not base:
        return ""
    if state.dds.security_provisioning == "managed":
        return f"{base}/{role}/{dds_node_key(state, role)}"
    return f"{base}/{dds_node_key(state, role)}"


def role_keystore_path(state: InstallState, role: str) -> Path:
    """Return the stable, single-role runtime keystore view."""

    if role not in state.roles:
        raise ValueError(f"role is not installed on this host: {role!r}")
    return state.prefix_path / "security" / "roles" / role


def rgbd_topic(state: InstallState, role: str) -> str:
    return f"/{state.dds.system_id}/{dds_node_key(state, role)}/rgbd/frame"


def generate_role_configs(
    state: InstallState,
    *,
    robot_host: RobotHostSettings | None = None,
) -> dict[str, Path]:
    """Write only installed copies; source-tree defaults remain untouched."""

    # A connection-managed SROS2 installation deliberately starts with no
    # keystore.  Its generated files are inert until the provisioning marker
    # is cleared by an all-host connection-manager transaction.
    state.require_installable_dds()
    written: dict[str, Path] = {}
    for role in state.roles:
        destination = generated_config_path(state, role)
        if role == "pilot":
            payload = _pilot_config(state, destination.parent / "runtime.yaml")
        elif role == "ui":
            payload = _ui_config(state, destination.parent / "default.yaml")
        elif role == "sim":
            payload = _sim_config(state, destination.parent / "runtime.yaml")
            _write_yaml(
                generated_app_config_path(state, role),
                _sim_app_config(state),
            )
        elif role == "robot":
            payload = _robot_config(
                state,
                destination.parent / "default.yaml",
                robot_host=robot_host,
            )
        else:
            raise ValueError(f"unknown role: {role}")
        _write_yaml(destination, payload)
        _write_cyclonedds(
            generated_dds_config_path(state, role),
            state.dds,
        )
        written[role] = destination
    return written


def _read_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"설정 원본이 없습니다: {path}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, Mapping):
        raise ValueError(f"{path}: YAML root가 object가 아닙니다")
    return dict(raw)


def _write_yaml(path: Path, payload: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    rendered = yaml.safe_dump(dict(payload), sort_keys=False, allow_unicode=True)
    _atomic_text(path, rendered)


def _atomic_text(path: Path, rendered: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.NamedTemporaryFile(
        "w",
        encoding="utf-8",
        dir=path.parent,
        prefix=f".{path.name}.",
        delete=False,
    ) as handle:
        handle.write(rendered)
        temporary = Path(handle.name)
    temporary.replace(path)


def _dds_payload(state: InstallState, role: str) -> dict[str, Any]:
    vendor_config = (
        "/opt/elesim/config/cyclonedds.xml"
        if state.install_mode == "container"
        else str(generated_dds_config_path(state, role))
    )
    return {
        "system_id": state.dds.system_id,
        "node_key": dds_node_key(state, role),
        "domain_id": state.dds.domain_id,
        "rmw_implementation": state.dds.rmw_implementation,
        "discovery_mode": state.dds.discovery_mode,
        "static_peers": list(state.dds.static_peers),
        "network_interface": state.dds.interface,
        "vendor_config": vendor_config,
        "security_profile": state.dds.security_profile,
        "security_provisioning": state.dds.security_provisioning,
        "security_generation": state.dds.security_generation,
        "keystore": (
            str(role_keystore_path(state, role))
            if state.dds.security_profile == "sros2"
            else ""
        ),
        "enclave": dds_enclave(state, role),
    }


def _pilot_config(state: InstallState, source: Path) -> dict[str, Any]:
    raw = _read_yaml(source)
    runtime = dict(raw.get("runtime") or {})
    runtime.pop("server_endpoint", None)
    runtime.update(
        {
            "role": "pilot",
            "endpoint_id": state.network.pilot_id,
            "active_target": state.network.sim_id,
        }
    )
    raw["runtime"] = runtime
    raw["dds"] = _dds_payload(state, "pilot")
    raw.pop("security", None)
    return raw


def _ui_config(state: InstallState, source: Path) -> dict[str, Any]:
    raw = _read_yaml(source)
    runtime = dict(raw.get("runtime") or {})
    runtime.pop("server_endpoint", None)
    runtime.update(
        {
            "endpoint_id": state.network.ui_id,
            "pilot_id": state.network.pilot_id,
            "sim_id": state.network.sim_id,
        }
    )
    raw["runtime"] = runtime
    raw["dds"] = _dds_payload(state, "ui")
    raw.pop("security", None)
    return raw


def _sim_config(state: InstallState, source: Path) -> dict[str, Any]:
    raw = _read_yaml(source)
    runtime = dict(raw.get("runtime") or {})
    runtime.pop("server_endpoint", None)
    streams = dict(runtime.get("streams") or {})
    for key in (
        "rgbd_bind",
        "rgbd_advertise",
        "observer_bind",
        "observer_advertise",
    ):
        streams.pop(key, None)
    streams.update(
        {
            "rgbd_topic": rgbd_topic(state, "sim"),
            "observer": (
                f"webrtc://{state.dds.system_id}/"
                f"{dds_node_key(state, 'sim')}/observer"
            ),
            "hand_eye_preview": (
                f"webrtc://{state.dds.system_id}/"
                f"{dds_node_key(state, 'sim')}/hand_eye_preview"
            ),
        }
    )
    runtime.update(
        {
            "role": "sim",
            "endpoint_id": state.network.sim_id,
            "streams": streams,
        }
    )
    raw["runtime"] = runtime
    raw["dds"] = _dds_payload(state, "sim")
    raw.pop("security", None)
    turn = dict(raw.get("turn") or {})
    turn["urls"] = list(state.network.turn_urls)
    if (
        state.turn.managed
        and state.network.turn_urls
        and state.turn.secret_path is not None
    ):
        turn["realm"] = state.turn.realm
        turn["static_auth_secret_file"] = (
            "/run/secrets/turn.secret"
            if state.install_mode == "container"
            else str(state.turn.secret_path)
        )
        turn.pop("credential_file", None)
    elif state.turn.mode == "external" and state.turn.credential_path is not None:
        turn.pop("realm", None)
        turn.pop("static_auth_secret_file", None)
        turn["credential_file"] = (
            "/run/secrets/turn.credentials.json"
            if state.install_mode == "container"
            else str(state.turn.credential_path)
        )
    else:
        turn.pop("realm", None)
        turn.pop("static_auth_secret_file", None)
        turn.pop("credential_file", None)
    raw["turn"] = turn
    return raw


def _sim_app_config(state: InstallState) -> dict[str, Any]:
    mode = (
        "pc"
        if state.profile == "local-sim" and state.install_mode == "native"
        else "remote"
    )
    return {
        "schema_version": 1,
        "extends": "config.yaml",
        "mode": mode,
        # Keep installer-specific GPU selection inside the selected profile so
        # it wins over that profile's default without copying the full file.
        "profiles": {
            mode: {
                "simulation": {
                    "runtime": {
                        "use_gpu": state.compute.gpu_mode != "cpu",
                    }
                }
            }
        },
    }


def _robot_config(
    state: InstallState,
    source: Path,
    *,
    robot_host: RobotHostSettings | None = None,
) -> dict[str, Any]:
    raw = _read_yaml(source)
    runtime = dict(raw.get("runtime") or {})
    runtime.pop("server_endpoint", None)
    runtime.update({"endpoint_id": state.network.robot_id})
    raw["runtime"] = runtime
    camera = dict(raw.get("camera") or {})
    camera.pop("bind", None)
    camera.pop("advertise", None)
    camera["topic"] = rgbd_topic(state, "robot")
    raw["camera"] = camera
    if robot_host is not None:
        host = robot_host.validate()
        go2 = dict(raw.get("go2") or {})
        go2.update(
            {
                "ros_workspace": str(host.ros_workspace.resolve()),
                "ipc_robot_user": host.robot_user,
                "ipc_bridge_user": host.bridge_user,
                "network_interface": host.unitree_interface,
                "ros_domain_id": int(host.unitree_domain_id),
            }
        )
        raw["go2"] = go2
    raw["dds"] = _dds_payload(state, "robot")
    raw.pop("security", None)
    return raw


def _write_cyclonedds(path: Path, dds: DdsSettings) -> None:
    root = ET.Element("CycloneDDS")
    domain = ET.SubElement(root, "Domain", {"id": str(dds.domain_id)})
    general = ET.SubElement(domain, "General")
    transport = _cyclonedds_transport(dds)
    if transport:
        # Cyclone DDS otherwise chooses its default address family.  A static
        # literal peer set gives us enough evidence to pin the family and
        # avoid advertising the other address on dual-stack VPN interfaces.
        ET.SubElement(general, "Transport").text = transport
    ET.SubElement(general, "AllowMulticast").text = (
        "true" if dds.discovery_mode == "multicast" else "false"
    )
    interface = str(dds.interface).strip()
    # ``automatic`` was accepted by older setup flows as a display value, but
    # CycloneDDS treats it as a literal interface name and refuses to create a
    # domain.  Omit the element to request the vendor's normal auto-selection.
    if interface.casefold() in {"automatic", "auto", "-"}:
        interface = ""
    if interface:
        interfaces = ET.SubElement(general, "Interfaces")
        ET.SubElement(
            interfaces,
            "NetworkInterface",
            {"name": interface},
        )
    discovery = ET.SubElement(domain, "Discovery")
    ET.SubElement(discovery, "ParticipantIndex").text = "auto"
    if dds.static_peers:
        peers = ET.SubElement(discovery, "Peers")
        for peer in dds.static_peers:
            ET.SubElement(peers, "Peer", {"Address": peer})
    ET.indent(root, space="  ")
    rendered = ET.tostring(root, encoding="unicode", xml_declaration=True) + "\n"
    _atomic_text(path, rendered)


def _cyclonedds_transport(dds: DdsSettings) -> str:
    """Return an explicit CycloneDDS transport for an unambiguous peer set.

    Hostnames and mixed address families are intentionally left to the vendor
    default: resolving them at installation time would make a generated
    topology stale when DNS or a VPN address changes.  Literal all-IPv4 and
    all-IPv6 static peers are safe to pin and must use one family consistently.
    """

    if dds.discovery_mode != "static" or not dds.static_peers:
        return ""
    try:
        versions = {
            ipaddress.ip_address(peer).version for peer in dds.static_peers
        }
    except ValueError:
        return ""
    if versions == {4}:
        return "udp"
    if versions == {6}:
        return "udp6"
    return ""


def write_cyclonedds_config(path: Path, dds: DdsSettings) -> Path:
    """Write a validated CycloneDDS vendor config for non-role environments."""

    dds.validate()
    _write_cyclonedds(path, dds)
    return path


__all__ = [
    "GENERATED_APP",
    "GENERATED_CONFIG",
    "GENERATED_DDS",
    "GENERATED_RUNTIME",
    "RobotHostSettings",
    "copy_role_config_tree",
    "dds_enclave",
    "dds_node_key",
    "generate_role_configs",
    "generated_app_config_path",
    "generated_config_path",
    "generated_dds_config_path",
    "rgbd_topic",
    "role_keystore_path",
    "role_directory",
    "write_cyclonedds_config",
]
