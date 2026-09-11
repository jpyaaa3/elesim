"""Generate deployment-owned YAML and DDS middleware configuration."""

from __future__ import annotations

import ipaddress
import json
import os
import re
import shutil
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Mapping

import yaml

from .state import DdsSettings, InstallState
from .instances import InstanceState


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


def app_directory(state: InstallState, role: str) -> Path:
    return state.prefix_path / "apps" / role


def copy_app_config_tree(source: Path, destination: Path, role: str) -> None:
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


def bind_installed_data_paths(
    config_root: Path,
    role: str,
    *,
    runtime_data_root: Path,
) -> None:
    """Point copied app configuration at the shared installed data tree."""

    if role not in {"pilot", "sim"}:
        return
    config_path = config_root / "config.yaml"
    raw = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    simulation = raw.setdefault("simulation", {})
    cameras = simulation.setdefault("cameras", {})
    hand_eye = cameras.setdefault("hand_eye", {})
    hand_eye["config"] = str(
        runtime_data_root / "calibration/cameras/zed_mini.hand_eye.json"
    )
    if role == "sim":
        simulation.setdefault("assembly", {})["build_dir"] = str(
            runtime_data_root / "models/assemblies/zed-mini"
        )
    config_path.write_text(
        yaml.safe_dump(raw, sort_keys=False, allow_unicode=True),
        encoding="utf-8",
    )

    detector = config_root / "perception/detector.yolo.example.json"
    if role == "pilot" and detector.is_file():
        detector_raw = json.loads(detector.read_text(encoding="utf-8"))
        detector_raw["model"] = str(
            runtime_data_root / "models/perception/yolov8n-seg.pt"
        )
        detector.write_text(
            json.dumps(detector_raw, indent=2, ensure_ascii=False) + "\n",
            encoding="utf-8",
        )


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
    return app_directory(state, role) / "config" / name


def generated_app_config_path(state: InstallState, role: str) -> Path:
    if role != "sim":
        raise ValueError(f"{role!r} does not have a generated application config")
    return app_directory(state, role) / "config" / GENERATED_APP


def generated_dds_config_path(state: InstallState, role: str) -> Path:
    return app_directory(state, role) / "config" / GENERATED_DDS


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


def app_keystore_path(state: InstallState, role: str) -> Path:
    """Return the stable, single-role runtime keystore view."""

    if role not in state.roles:
        raise ValueError(f"role is not installed on this host: {role!r}")
    return state.prefix_path / "security" / "apps" / role


def rgbd_topic(state: InstallState, role: str) -> str:
    return f"/{state.dds.system_id}/{dds_node_key(state, role)}/rgbd/frame"


def rgbd_broker_topic(state: InstallState) -> str:
    """Return the inter-host RGB-D topic owned by Pilot.

    Source and local handoff topics remain role-specific while the encoded
    stream has one stable owner.  Keeping this derivation in the installer
    prevents a custom Pilot endpoint from silently producing a stale topic.
    Runtime publishers/subscribers may adopt this field independently; the
    generated config is the deployment contract for that migration.
    """

    return rgbd_topic(state, "pilot")


def _rgbd_role_config(
    state: InstallState,
    *,
    source_role: str,
    local_handoff: str,
) -> dict[str, Any]:
    """Describe the bounded RGB-D edge-broker contract in role config.

    This is deliberately configuration metadata, not a second runtime role.
    Source processes encode before a source topic leaves the camera edge; the
    Pilot relay handles legacy raw input and owns the encoded latest-only
    inter-host output.
    """

    return {
        "schema_version": 1,
        "broker_role": "pilot",
        "source_role": source_role,
        "local_handoff": local_handoff,
        "wire": {
            "format": "encoded-rgbd-v1",
            "capability": "stream.rgbd.broker.v1",
            "topic": rgbd_broker_topic(state),
            "latest_only": True,
        },
    }


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
    for role in state.runtime_roles:
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


def generate_instance_configs(
    state: InstallState,
    instance: InstanceState,
    *,
    template_root: Path | None = None,
    security_views: Mapping[str, tuple[Path, str]] | None = None,
    output_prefix: Path | None = None,
) -> dict[str, Path]:
    """Generate endpoint-private configs without touching legacy role configs."""
    state.validate()
    instance.validate()
    # Capability inventory is the installed role set; manager assignment must
    # not prevent rendering a separately registered instance.
    installed_roles = tuple(state.roles)
    if "robot" in installed_roles or any(endpoint.role == "robot" for endpoint in instance.endpoints):
        raise ValueError("Robot instance configuration is not supported")
    endpoint_roles = tuple(endpoint.role for endpoint in instance.endpoints)
    if len(set(endpoint_roles)) != len(endpoint_roles):
        raise ValueError("instance endpoint roles must be unique")
    if not set(endpoint_roles).issubset(installed_roles):
        raise ValueError("instance endpoints must be a subset of installed runtime roles")
    if instance.security_profile == "sros2":
        if state.dds.security_provisioning != "managed":
            raise ValueError("sros2 instances require managed SROS2 provisioning")
        if security_views is None or set(security_views) != set(endpoint_roles):
            raise ValueError("sros2 instances require endpoint security views")
        representative = next(iter(security_views.values()))
        instance_dds = replace(
            state.dds,
            system_id=instance.system_id,
            domain_id=instance.domain_id,
            rmw_implementation=instance.rmw_implementation,
            discovery_mode=instance.discovery_mode,
            static_peers=instance.static_peers,
            interface=instance.interface,
            security_profile="sros2",
            security_provisioning="managed",
            security_generation=instance.security_generation,
            security_bundle=str(representative[0]),
            keystore=str(representative[0]),
            enclave=representative[1],
        )
    else:
        instance_dds = replace(
            state.dds,
            system_id=instance.system_id,
            domain_id=instance.domain_id,
            rmw_implementation=instance.rmw_implementation,
            discovery_mode=instance.discovery_mode,
            static_peers=instance.static_peers,
            interface=instance.interface,
            security_profile="trusted-network",
            security_provisioning="none",
            security_generation="",
            security_bundle="",
            keystore="",
            enclave="",
        )
    endpoint_ids = {endpoint.role: endpoint.endpoint_id for endpoint in instance.endpoints}
    roles = endpoint_roles
    _reject_symlink_ancestors(Path(state.prefix).expanduser(), name="install prefix")
    destination_prefix = state.prefix_path if output_prefix is None else Path(output_prefix).expanduser()
    _reject_symlink_ancestors(destination_prefix, name="instance output prefix")
    root = destination_prefix / "instances" / instance.system_id / "endpoints"
    _reject_symlink_ancestors(root, name="instance config destination")
    # Validate every source and destination before copying the first role.
    plans: list[tuple[str, Path, Path, Path]] = []
    for role in roles:
        source_root = (
            Path(template_root) / role
            if template_root is not None
            else app_directory(state, role) / "config"
        )
        _reject_symlink_ancestors(source_root, name="instance config source")
        if not source_root.is_dir() or source_root.is_symlink():
            raise FileNotFoundError(source_root)
        source = source_root / ("runtime.yaml" if role in {"pilot", "sim"} else "default.yaml")
        if not source.is_file() or source.is_symlink():
            raise FileNotFoundError(source)
        _reject_symlink_tree(source_root, name="instance config source")
        destination = root / endpoint_ids[role] / "config"
        _reject_symlink_ancestors(destination, name="instance config destination")
        plans.append((role, source_root, source, destination))
    written: dict[str, Path] = {}
    for role, source_root, source, destination in plans:
        role_state = replace(
            state,
            prefix=str(root / endpoint_ids[role]),
            roles=(role,),
            assigned_roles=None,
            network=replace(
                state.network,
                pilot_id=instance.pilot_id,
                sim_id=instance.sim_id,
                ui_id=instance.ui_id,
            ),
            dds=instance_dds,
        )
        destination = Path(role_state.prefix) / "config"
        _reject_symlink_ancestors(destination, name="instance config destination")
        copy_app_config_tree(source_root, destination, role)
        if role == "pilot":
            payload = _pilot_config(
                role_state,
                source,
                security_view=security_views.get(role) if security_views else None,
            )
        elif role == "sim":
            payload = _sim_config(
                role_state,
                source,
                security_view=security_views.get(role) if security_views else None,
            )
            _write_yaml(destination / GENERATED_APP, _sim_app_config(role_state))
        else:
            payload = _ui_config(
                role_state,
                source,
                security_view=security_views.get(role) if security_views else None,
            )
        if role in {"pilot", "sim"} and (destination / "config.yaml").is_file():
            bind_installed_data_paths(
                destination,
                role,
                runtime_data_root=Path("/opt/elesim/data"),
            )
        _write_yaml(destination / (GENERATED_RUNTIME if role in {"pilot", "sim"} else GENERATED_CONFIG), payload)
        _write_cyclonedds(destination / GENERATED_DDS, role_state.dds)
        written[role] = destination / (GENERATED_RUNTIME if role in {"pilot", "sim"} else GENERATED_CONFIG)
    return written


def _read_yaml(path: Path) -> dict[str, Any]:
    if not path.is_file():
        raise FileNotFoundError(f"Configuration source is missing: {path}")
    raw = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(raw, Mapping):
        raise ValueError(f"{path}: YAML root must be an object")
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


def _dds_payload(
    state: InstallState,
    role: str,
    *,
    security_view: tuple[Path, str] | None = None,
) -> dict[str, Any]:
    vendor_config = (
        "/opt/elesim/config/cyclonedds.xml"
        if state.install_mode == "container"
        else str(generated_dds_config_path(state, role))
    )
    keystore = (
        str(security_view[0])
        if security_view is not None
        else str(app_keystore_path(state, role))
    )
    enclave = (
        security_view[1]
        if security_view is not None
        else dds_enclave(state, role)
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
        "keystore": keystore if state.dds.security_profile == "sros2" else "",
        "enclave": enclave,
    }


def _pilot_config(
    state: InstallState,
    source: Path,
    *,
    security_view: tuple[Path, str] | None = None,
) -> dict[str, Any]:
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
    raw["dds"] = _dds_payload(state, "pilot", security_view=security_view)
    raw["rgbd"] = _rgbd_role_config(
        state,
        source_role="auto",
        local_handoff="source-dds-to-pilot",
    )
    raw.pop("security", None)
    return raw


def _ui_config(
    state: InstallState,
    source: Path,
    *,
    security_view: tuple[Path, str] | None = None,
) -> dict[str, Any]:
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
    raw["dds"] = _dds_payload(state, "ui", security_view=security_view)
    raw["rgbd"] = _rgbd_role_config(
        state,
        source_role="pilot",
        local_handoff="none",
    )
    raw.pop("security", None)
    return raw


def _sim_config(
    state: InstallState,
    source: Path,
    *,
    security_view: tuple[Path, str] | None = None,
) -> dict[str, Any]:
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
    raw["dds"] = _dds_payload(state, "sim", security_view=security_view)
    raw["rgbd"] = _rgbd_role_config(
        state,
        source_role="sim",
        local_handoff="source-dds-to-pilot",
    )
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
    raw["rgbd"] = _rgbd_role_config(
        state,
        source_role="robot",
        local_handoff="source-dds-to-pilot",
    )
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
    "copy_app_config_tree",
    "dds_enclave",
    "dds_node_key",
    "generate_role_configs",
    "generate_instance_configs",
    "generated_app_config_path",
    "generated_config_path",
    "generated_dds_config_path",
    "rgbd_broker_topic",
    "rgbd_topic",
    "app_keystore_path",
    "app_directory",
    "write_cyclonedds_config",
]
