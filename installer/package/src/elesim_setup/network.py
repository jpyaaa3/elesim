"""Configure installed DDS settings and run layered connectivity checks."""

from __future__ import annotations

import argparse
import base64
import ipaddress
import json
import os
import re
import socket
import stat
import subprocess
import sys
import tempfile
import xml.etree.ElementTree as ET
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Callable, Mapping, Sequence

import yaml

from .configuration import (
    generate_role_configs,
    generated_app_config_path,
    generated_config_path,
    generated_dds_config_path,
)
from .doctor import NetworkDoctor
from .security_provisioning import (
    provisioning_required_path,
    sync_provisioning_required,
)
from .state import DdsSettings, InstallState, NetworkSettings, TurnSettings, default_state_path


_TAILSCALE_INTERFACE = re.compile(r"^tailscale[0-9]+$")


def is_tailscale_interface(value: object) -> bool:
    """Return whether *value* names a kernel Tailscale interface.

    Tailscale normally creates ``tailscale0``, but the suffix is not a stable
    contract: an old interface can remain while a reconnect creates
    ``tailscale1``.  Keep the accepted shape narrow so an arbitrary string
    beginning with ``tailscale`` cannot silently opt into routed discovery.
    """

    return bool(_TAILSCALE_INTERFACE.fullmatch(str(value).strip()))


@dataclass(frozen=True)
class TailscaleDetection:
    """Read-only local Tailscale interface hint for the connection manager.

    Detection never installs, logs in, changes ACLs, or invokes ``tailscale
    up``.  It is merely a convenience for filling the local DDS address and
    interface; the operator still saves and validates the topology explicitly.
    """

    available: bool
    interface: str = ""
    addresses: tuple[str, ...] = ()
    detail: str = ""

    def to_dict(self) -> dict[str, object]:
        return {
            "available": self.available,
            "interface": self.interface,
            "addresses": list(self.addresses),
            "detail": self.detail,
        }


def detect_tailscale(
    *,
    runner: Callable[..., object] | None = None,
) -> TailscaleDetection:
    """Return current ``tailscale*`` IPv4 addresses without side effects.

    ``ip`` is used instead of parsing ``tailscale status`` so this works on a
    minimal host and does not require the local user to have Tailscale admin
    privileges.  A missing binary/interface is a normal, actionable result.
    """

    inherited_hint = os.environ.get("ELESIM_TAILSCALE_ADDRESS", "").strip()
    if inherited_hint:
        addresses: list[str] = []
        for candidate in inherited_hint.split(","):
            value = candidate.strip()
            try:
                address = ipaddress.ip_address(value)
            except ValueError:
                continue
            if address.version == 4 and not address.is_unspecified and value not in addresses:
                addresses.append(value)
        if addresses:
            return TailscaleDetection(
                True,
                interface=(
                    os.environ.get("ELESIM_TAILSCALE_INTERFACE", "tailscale0")
                    if is_tailscale_interface(
                        os.environ.get("ELESIM_TAILSCALE_INTERFACE", "tailscale0")
                    )
                    else "tailscale0"
                ),
                addresses=tuple(addresses),
                detail="read-only Tailscale address hint supplied by the host wrapper",
            )

    probe = subprocess.run if runner is None else runner
    try:
        result = probe(
            ["ip", "-j", "-4", "addr", "show"],
            capture_output=True,
            text=True,
            check=False,
            timeout=1.5,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        return TailscaleDetection(False, detail=f"ip probe unavailable: {exc}")
    if int(getattr(result, "returncode", 1)) != 0:
        return TailscaleDetection(
            False,
            detail="tailscale* interface was not found; enter a routed-VPN or LAN address manually",
        )
    try:
        raw = json.loads(str(getattr(result, "stdout", "") or ""))
    except (TypeError, ValueError):
        return TailscaleDetection(False, detail="ip returned invalid JSON for tailscale*")
    addresses_by_interface: dict[str, list[str]] = {}
    if isinstance(raw, list):
        for link in raw:
            if not isinstance(link, dict):
                continue
            interface = str(link.get("ifname", "")).strip()
            if not is_tailscale_interface(interface):
                continue
            interface_addresses = addresses_by_interface.setdefault(interface, [])
            for item in link.get("addr_info", ()):
                if not isinstance(item, dict) or item.get("family") != "inet":
                    continue
                address = str(item.get("local", "")).strip()
                if address and address not in interface_addresses:
                    interface_addresses.append(address)
    preferred = os.environ.get("ELESIM_TAILSCALE_INTERFACE", "tailscale0").strip()
    ordered_interfaces = sorted(
        addresses_by_interface,
        key=lambda name: (0 if name == preferred else 1, 0 if name == "tailscale0" else 1, name),
    )
    interface = ordered_interfaces[0] if ordered_interfaces else preferred
    addresses: list[str] = []
    for name in ordered_interfaces:
        for address in addresses_by_interface[name]:
            if address not in addresses:
                addresses.append(address)
    if not addresses:
        return TailscaleDetection(
            False,
            interface=interface if is_tailscale_interface(interface) else "",
            detail="tailscale* exists but has no IPv4 address; use IPv6 or enter a current address manually",
        )
    return TailscaleDetection(
        True,
        interface=interface,
        addresses=tuple(addresses),
        detail="read-only Tailscale address hint; no installation or login was performed",
    )


def require_runtime_network_namespace(
    state: InstallState,
    *,
    interface: str | None = None,
    address: str | None = None,
    interface_names: Sequence[str] | None = None,
    interface_addresses: Mapping[str, Sequence[str]] | None = None,
    peers: Sequence[str] | None = None,
    route_runner: Callable[..., object] | None = None,
) -> None:
    """Fail before launch when DDS cannot bind its configured interface.

    Container installs execute this through the tools service, so the names
    describe the same network namespace that the runtime roles will use.  This
    is intentionally a direct-bind check: a configured ``tailscale*`` remains
    a valid request and passes wherever that interface is visible.  The
    optional override is used by the connection manager so a pending topology
    is checked instead of a stale installed state.
    """

    configured_interface = (
        state.dds.interface if interface is None else str(interface)
    ).strip()
    # Older generated states used the human-facing word ``automatic``.  It is
    # not a CycloneDDS interface name; an omitted interface is the portable
    # auto-selection form and is what the generated XML must represent.
    if configured_interface.casefold() in {"automatic", "auto", "-"}:
        configured_interface = ""
    if configured_interface:
        try:
            indexed = (
                socket.if_nameindex()
                if interface_names is None
                else enumerate(interface_names)
            )
            available = {str(name) for _index, name in indexed}
        except OSError as exc:
            raise RuntimeError(
                f"DDS network interfaces could not be inspected: {exc}"
            ) from exc
        if configured_interface not in available:
            detail = ", ".join(sorted(available)) or "none"
            raise RuntimeError(
                f"configured DDS interface {configured_interface!r} is not visible in "
                f"the runtime network namespace (visible: {detail}). Direct DDS bind "
                f"to {configured_interface!r} requires that interface in the runtime "
                "namespace. Docker Desktop/WSL may place the WSL interface in another "
                "namespace. A Tailscale 100.x address may still be routable through "
                "another interface, but that is a separate routed/NAT mode and does "
                "not satisfy a direct tailscale* bind."
            )

    configured_address = "" if address is None else str(address).strip()
    if configured_interface and configured_address:
        expected_addresses = _resolve_runtime_address(configured_address)
        assigned_addresses = _runtime_interface_addresses(
            configured_interface,
            supplied=interface_addresses,
            runner=route_runner,
        )
        if not expected_addresses.intersection(assigned_addresses):
            rendered = ", ".join(sorted(assigned_addresses)) or "none"
            raise RuntimeError(
                f"configured DDS address {configured_address!r} is not assigned to "
                f"runtime interface {configured_interface!r} (assigned: {rendered}). "
                "The DDS address must belong to the selected interface in the same "
                "network namespace as the runtime roles. A host or WSL Tailscale "
                "address cannot be advertised from a separate Docker Desktop "
                "namespace; enroll the Elesim Tailscale sidecar or select the "
                "native Docker backend that owns that interface."
            )

    configured_peers = tuple(
        str(value).strip()
        for value in (state.dds.static_peers if peers is None else peers)
        if str(value).strip()
    )
    if not configured_peers:
        return

    probe = subprocess.run if route_runner is None else route_runner
    for peer in configured_peers:
        try:
            result = probe(
                ["ip", "-j", "route", "get", peer],
                capture_output=True,
                text=True,
                check=False,
                timeout=1.5,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            raise RuntimeError(
                f"DDS peer route probe is unavailable for {peer!r}: {exc}. "
                "Install iproute2 in the runtime namespace or choose a host "
                "network backend that exposes the configured route."
            ) from exc
        if int(getattr(result, "returncode", 1)) != 0:
            detail = str(getattr(result, "stderr", "") or "").strip()
            suffix = f": {detail[:512]}" if detail else ""
            raise RuntimeError(
                f"no runtime route to DDS peer {peer!r}{suffix}. "
                "SSH/Tailscale TCP reachability does not prove a DDS UDP route."
            )
        try:
            raw = json.loads(str(getattr(result, "stdout", "") or ""))
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                f"runtime route probe returned invalid JSON for DDS peer {peer!r}"
            ) from exc
        route = raw[0] if isinstance(raw, list) and raw else raw
        if not isinstance(route, Mapping):
            raise RuntimeError(
                f"runtime route probe returned no route for DDS peer {peer!r}"
            )
        device = str(route.get("dev", "")).strip()
        if not device:
            raise RuntimeError(
                f"runtime route probe returned no interface for DDS peer {peer!r}"
            )
        if configured_interface and device != configured_interface:
            raise RuntimeError(
                f"DDS peer {peer!r} routes through {device!r}, not configured "
                f"interface {configured_interface!r}. Bind DDS to the routed "
                "interface or run Docker in the namespace containing the direct "
                "Tailscale interface."
            )


def _resolve_runtime_address(value: str) -> set[str]:
    """Resolve one advertised DDS endpoint into normalized IP literals."""

    try:
        return {str(ipaddress.ip_address(value))}
    except ValueError:
        pass
    try:
        records = socket.getaddrinfo(value, None, type=socket.SOCK_DGRAM)
    except socket.gaierror as exc:
        raise RuntimeError(
            f"configured DDS address {value!r} cannot be resolved in the runtime "
            "network namespace"
        ) from exc
    resolved: set[str] = set()
    for _family, _type, _proto, _canonical, sockaddr in records:
        if sockaddr:
            try:
                resolved.add(str(ipaddress.ip_address(str(sockaddr[0]))))
            except ValueError:
                continue
    if not resolved:
        raise RuntimeError(
            f"configured DDS address {value!r} has no usable runtime IP address"
        )
    return resolved


def _runtime_interface_addresses(
    interface: str,
    *,
    supplied: Mapping[str, Sequence[str]] | None,
    runner: Callable[..., object] | None,
) -> set[str]:
    """Return normalized addresses assigned to one runtime interface."""

    if supplied is not None:
        values = supplied.get(interface, ())
    else:
        probe = subprocess.run if runner is None else runner
        try:
            result = probe(
                ["ip", "-j", "addr", "show", "dev", interface],
                capture_output=True,
                text=True,
                check=False,
                timeout=1.5,
            )
        except (OSError, subprocess.SubprocessError) as exc:
            raise RuntimeError(
                f"DDS address assignment could not be inspected on "
                f"{interface!r}: {exc}"
            ) from exc
        if int(getattr(result, "returncode", 1)) != 0:
            detail = str(getattr(result, "stderr", "") or "").strip()
            suffix = f": {detail[:512]}" if detail else ""
            raise RuntimeError(
                f"DDS address assignment probe failed on {interface!r}{suffix}"
            )
        try:
            raw = json.loads(str(getattr(result, "stdout", "") or ""))
        except (TypeError, ValueError) as exc:
            raise RuntimeError(
                f"DDS address assignment probe returned invalid JSON for "
                f"{interface!r}"
            ) from exc
        values = tuple(
            str(item.get("local", "")).strip()
            for link in raw if isinstance(raw, list) and isinstance(link, Mapping)
            for item in link.get("addr_info", ())
            if isinstance(item, Mapping)
        )
    normalized: set[str] = set()
    for value in values:
        candidate = str(value).split("%", 1)[0].strip()
        try:
            normalized.add(str(ipaddress.ip_address(candidate)))
        except ValueError:
            continue
    return normalized


def require_runtime_tcp_reachability(
    peers: Sequence[str],
    *,
    port: int = 22,
    connector: Callable[..., object] | None = None,
) -> None:
    """Run a negative-only sanity probe for Tailscale SSH peers.

    This is deliberately limited to peers whose management connection already
    uses keyless Tailscale SSH.  A failure is useful: the same runtime
    namespace cannot even reach the peer's Tailscale SSH endpoint, so starting
    DDS there would otherwise produce a long, opaque discovery wait.  A
    successful TCP connection is *not* treated as proof of DDS UDP reachability;
    the route/interface check and live DDS doctor remain separate gates.
    """

    if isinstance(port, bool) or not 1 <= int(port) <= 65535:
        raise ValueError("TCP probe port must be in 1..65535")
    connect = socket.create_connection if connector is None else connector
    for peer in tuple(str(value).strip() for value in peers if str(value).strip()):
        try:
            connection = connect((peer, int(port)), timeout=1.5)
            close = getattr(connection, "close", None)
            if callable(close):
                close()
        except (OSError, TimeoutError) as exc:
            raise RuntimeError(
                f"runtime namespace cannot reach Tailscale SSH peer "
                f"{peer}:{port}: {exc}. This negative TCP check is not a DDS "
                "proof, but it confirms that the runtime path is broken. "
                "Docker Desktop/WSL host networking commonly isolates the Docker "
                "Linux VM from WSL tailscale*; use Docker Engine in the same "
                "namespace as Tailscale or configure a genuinely routed, "
                "container-visible DDS interface."
            ) from exc


def require_generated_dds_configuration(state: InstallState) -> None:
    """Reject stale generated DDS files before a runtime can start.

    ``install-state.json``, the CycloneDDS XML, and the generated Compose
    environment are three views of one configuration.  Older connection
    manager runs updated only one or two of them, which let a process start
    with an XML interface different from the value shown by ``elesim-net``.
    Such a mismatch is especially dangerous on Docker Desktop/WSL: the
    manager may report a Tailscale choice while CycloneDDS still binds the
    Docker VM's ``eth0``.  This check is local, bounded, and does not claim
    that a route is live; it only prevents contradictory generated files.
    """

    expected_interface = _normalized_interface(state.dds.interface)
    expected_peers = tuple(state.dds.static_peers)
    expected_multicast = state.dds.discovery_mode == "multicast"

    for role in state.roles:
        xml_path = generated_dds_config_path(state, role)
        if xml_path.is_symlink() or not xml_path.is_file():
            raise RuntimeError(
                f"generated DDS config is missing for role {role!r}: {xml_path}. "
                "Run elesim-net configure or elesim-update before starting."
            )
        try:
            root = ET.parse(xml_path).getroot()
        except (ET.ParseError, OSError) as exc:
            raise RuntimeError(
                f"generated DDS XML is unreadable for role {role!r}: {xml_path}"
            ) from exc
        domain = root.find("./Domain")
        if domain is None:
            raise RuntimeError(
                f"generated DDS XML has no Domain for role {role!r}: {xml_path}"
            )
        general = domain.find("./General")
        if general is None:
            raise RuntimeError(
                f"generated DDS XML has no General section for role {role!r}"
            )
        interfaces = tuple(
            str(node.attrib.get("name", "")).strip()
            for node in general.findall("./Interfaces/NetworkInterface")
        )
        actual_interface = interfaces[0] if len(interfaces) == 1 else ""
        if len(interfaces) > 1 or actual_interface != expected_interface:
            rendered = ", ".join(interfaces) or "(automatic)"
            wanted = expected_interface or "(automatic)"
            raise RuntimeError(
                f"generated DDS XML for role {role!r} binds {rendered!r}, but "
                f"install state requires {wanted!r}. Re-run elesim-net configure "
                "or elesim-update; do not start with mixed XML/state values."
            )
        multicast_text = (general.findtext("./AllowMulticast") or "").strip().lower()
        if (multicast_text == "true") != expected_multicast:
            raise RuntimeError(
                f"generated DDS XML discovery mode for role {role!r} disagrees "
                "with install state; re-run elesim-net configure."
            )
        actual_peers = tuple(
            str(node.attrib.get("Address", "")).strip()
            for node in domain.findall("./Discovery/Peers/Peer")
        )
        if actual_peers != expected_peers:
            raise RuntimeError(
                f"generated DDS XML static peers for role {role!r} disagree "
                "with install state; re-run elesim-net configure."
            )

    if state.install_mode != "container":
        return
    compose_path = state.prefix_path / "containers" / "compose.yaml"
    if compose_path.is_symlink() or not compose_path.is_file():
        raise RuntimeError(
            f"generated Compose manifest is missing: {compose_path}. "
            "Run elesim-update before starting."
        )
    try:
        payload = yaml.safe_load(compose_path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"generated Compose manifest is unreadable: {compose_path}") from exc
    services = payload.get("services") if isinstance(payload, Mapping) else None
    if not isinstance(services, Mapping):
        raise RuntimeError(f"generated Compose manifest has no services: {compose_path}")
    expected_peer_text = ",".join(expected_peers)
    for role in state.roles:
        service = services.get(role)
        environment = service.get("environment") if isinstance(service, Mapping) else None
        if not isinstance(environment, Mapping):
            raise RuntimeError(f"generated Compose service {role!r} has no environment")
        actual_interface = _normalized_interface(environment.get("ELESIM_DDS_NETWORK_INTERFACE", ""))
        actual_peers = str(environment.get("ELESIM_DDS_STATIC_PEERS", "")).strip()
        actual_discovery = str(environment.get("ELESIM_DDS_DISCOVERY_MODE", "")).strip().lower()
        if actual_interface != expected_interface:
            raise RuntimeError(
                f"generated Compose DDS interface for role {role!r} is "
                f"{actual_interface or '(automatic)'!r}, but install state requires "
                f"{expected_interface or '(automatic)'!r}; run elesim-update."
            )
        if actual_peers != expected_peer_text or (
            actual_discovery == "multicast"
        ) != expected_multicast:
            raise RuntimeError(
                f"generated Compose DDS discovery values for role {role!r} are "
                "stale; run elesim-update before starting."
            )


def _normalized_interface(value: object) -> str:
    interface = str(value or "").strip()
    return "" if interface.casefold() in {"automatic", "auto", "-"} else interface


def _prompt(label: str, current: str) -> str:
    value = input(f"{label} [{current}]: ").strip()
    return value or current


def _configure_interactive(state: InstallState) -> InstallState:
    print("\nElesim ROS 2/DDS 설정")
    system_id = _prompt("Elesim system ID", state.dds.system_id)
    sim_id = _prompt("Sim endpoint ID", state.network.sim_id)
    pilot_id = _prompt("Pilot endpoint ID", state.network.pilot_id)
    ui_id = _prompt("UI endpoint ID", state.network.ui_id)
    robot_id = _prompt("Robot endpoint ID", state.network.robot_id)
    domain_id = int(_prompt("ROS_DOMAIN_ID", str(state.dds.domain_id)))
    discovery_mode = _prompt(
        "DDS discovery (multicast/static)",
        state.dds.discovery_mode,
    )
    peers_raw = _prompt(
        "Static peer hostname/IP (쉼표 구분, 없으면 '-')",
        ",".join(state.dds.static_peers) or "-",
    )
    static_peers = (
        ()
        if peers_raw == "-"
        else tuple(value.strip() for value in peers_raw.split(",") if value.strip())
    )
    interface = _prompt("DDS interface (자동이면 '-')", state.dds.interface or "-")
    interface = "" if interface == "-" else interface
    security_profile = _prompt(
        "DDS security profile (trusted-network/sros2)",
        state.dds.security_profile,
    )
    keystore = state.dds.keystore
    enclave = state.dds.enclave
    security_provisioning = state.dds.security_provisioning
    security_generation = state.dds.security_generation
    security_bundle = state.dds.security_bundle
    if security_profile == "sros2":
        if (
            state.dds.security_profile == "sros2"
            and state.dds.security_provisioning == "managed"
        ):
            print("SROS2 managed bundle은 elesim-connections에서 교체하십시오.")
        else:
            security_provisioning = "external"
            security_generation = ""
            security_bundle = ""
            keystore = _prompt(
                "SROS2 keystore",
                keystore or str(state.prefix_path / "sros2"),
            )
            enclave = _prompt("SROS2 base enclave", enclave or "/elesim")
    else:
        security_provisioning = "none"
        security_generation = ""
        security_bundle = ""
        keystore = ""
        enclave = ""
    turn_raw = _prompt(
        "TURN URL (없으면 '-')",
        state.network.turn_urls[0] if state.network.turn_urls else "-",
    )
    turn_urls = () if turn_raw == "-" else (turn_raw,)
    turn = state.turn
    if not turn_urls:
        turn = TurnSettings()
    elif turn.mode == "none":
        turn = TurnSettings(mode="external")
    if turn.mode == "external" and "sim" in state.roles:
        turn = replace(
            turn,
            credential_file=_prompt(
                "External TURN username/credential JSON file",
                turn.credential_file
                or str(state.prefix_path / "secrets/turn.credentials.json"),
            ),
        )
    return replace(
        state,
        network=replace(
            state.network,
            turn_urls=turn_urls,
            sim_id=sim_id,
            pilot_id=pilot_id,
            ui_id=ui_id,
            robot_id=robot_id,
        ),
        dds=DdsSettings(
            system_id=system_id,
            domain_id=domain_id,
            rmw_implementation=state.dds.rmw_implementation,
            discovery_mode=discovery_mode,
            static_peers=static_peers,
            interface=interface,
            security_profile=security_profile,
            security_provisioning=security_provisioning,
            security_generation=security_generation,
            security_bundle=security_bundle,
            keystore=keystore,
            enclave=enclave,
        ),
        turn=turn,
    ).require_runnable_dds()


def _configure_from_args(state: InstallState, args: argparse.Namespace) -> InstallState:
    peers = state.dds.static_peers
    if args.clear_dds_static_peers:
        peers = ()
    elif args.dds_static_peer is not None:
        peers = tuple(args.dds_static_peer)
    security_profile = args.dds_security_profile or state.dds.security_profile
    requested_provisioning = args.dds_security_provisioning
    security_provisioning = (
        requested_provisioning
        or state.dds.security_provisioning
        if state.dds.security_profile == "sros2"
        else requested_provisioning or "external"
    )
    security_generation = (
        args.dds_security_generation
        if args.dds_security_generation is not None
        else state.dds.security_generation
    )
    security_bundle = (
        args.dds_security_bundle
        if args.dds_security_bundle is not None
        else state.dds.security_bundle
    )
    if security_profile == "trusted-network":
        security_provisioning = "none"
        security_generation = ""
        security_bundle = ""
    elif security_provisioning == "external":
        security_generation = ""
        security_bundle = ""
    if (
        security_profile == "sros2"
        and state.dds.security_provisioning == "managed"
        and not requested_provisioning
        and (args.dds_keystore is not None or args.dds_enclave is not None)
    ):
        raise ValueError(
            "managed SROS2 bundle은 elesim-connections에서만 교체할 수 있습니다"
        )
    keystore = (
        state.dds.keystore if args.dds_keystore is None else args.dds_keystore
    )
    enclave = state.dds.enclave if args.dds_enclave is None else args.dds_enclave
    if security_provisioning == "managed" and not keystore and security_bundle:
        keystore = security_bundle
    dds = replace(
        state.dds,
        system_id=args.dds_system_id or state.dds.system_id,
        domain_id=(
            state.dds.domain_id if args.dds_domain_id is None else args.dds_domain_id
        ),
        rmw_implementation=(
            args.dds_rmw_implementation or state.dds.rmw_implementation
        ),
        discovery_mode=args.dds_discovery_mode or state.dds.discovery_mode,
        static_peers=peers,
        interface=(
            state.dds.interface
            if args.dds_interface is None
            else args.dds_interface
        ),
        security_profile=security_profile,
        security_provisioning=security_provisioning,
        security_generation=security_generation,
        security_bundle=security_bundle,
        keystore=keystore,
        enclave=enclave,
    )
    if dds.security_profile == "trusted-network":
        dds = replace(dds, keystore="", enclave="")

    # The DDS security profile is the single operator-facing WebRTC policy:
    # trusted-network/plaintext uses direct ICE and must not leave a managed
    # Coturn endpoint behind.  This also repairs older installs when an
    # operator switches profiles without remembering the retired TURN flags.
    turn_urls = (
        ()
        if args.clear_turn or security_profile == "trusted-network"
        else state.network.turn_urls
        if args.turn_url is None
        else tuple(args.turn_url)
    )
    network = replace(
        state.network,
        turn_urls=turn_urls,
        sim_id=args.sim_id or state.network.sim_id,
        pilot_id=args.pilot_id or state.network.pilot_id,
        ui_id=args.ui_id or state.network.ui_id,
        robot_id=args.robot_id or state.network.robot_id,
    )
    if not turn_urls:
        turn = TurnSettings()
    else:
        mode = args.turn_mode or (
            state.turn.mode if state.turn.mode != "none" else "external"
        )
        turn = TurnSettings(
            mode=mode,
            realm=(
                args.turn_realm
                if args.turn_realm is not None
                else state.turn.realm if mode == "managed" else ""
            ),
            public_host=(
                args.turn_public_host
                if args.turn_public_host is not None
                else state.turn.public_host if mode == "managed" else ""
            ),
            secret_file=(
                args.turn_secret_file
                if args.turn_secret_file is not None
                else state.turn.secret_file if mode == "managed" else ""
            ),
            credential_file=(
                args.turn_credential_file
                if args.turn_credential_file is not None
                else state.turn.credential_file if mode == "external" else ""
            ),
        )
    return replace(state, network=network, dds=dds, turn=turn).require_runnable_dds()


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state", default=str(default_state_path()))
    subparsers = parser.add_subparsers(dest="command", required=True)

    subparsers.add_parser("show", help="현재 DDS/TURN 설정 출력")
    namespace_check = subparsers.add_parser(
        "namespace-check",
        help="런타임 네임스페이스에서 설정된 DDS interface 확인",
    )
    namespace_check.add_argument(
        "--dds-interface",
        help="설치 상태 대신 검사할 pending DDS interface",
    )
    namespace_check.add_argument(
        "--dds-address",
        help="선택한 runtime interface에 실제 할당되어야 하는 DDS 주소",
    )
    namespace_check.add_argument(
        "--dds-peer",
        action="append",
        default=None,
        help="검사할 직접 연결 DDS peer (반복 가능)",
    )
    namespace_check.add_argument(
        "--tcp-peer",
        action="append",
        default=None,
        help="negative-only Tailscale SSH reachability probe peer (반복 가능)",
    )
    restore = subparsers.add_parser("restore-snapshot", help=argparse.SUPPRESS)
    restore.add_argument("--payload", required=True, help=argparse.SUPPRESS)
    configure = subparsers.add_parser(
        "configure",
        help="DDS/TURN 설정을 바꾸고 역할별 YAML/XML을 재생성",
    )
    configure.add_argument("--dds-system-id", default="")
    configure.add_argument("--dds-domain-id", type=int)
    configure.add_argument(
        "--dds-rmw-implementation",
        choices=("rmw_cyclonedds_cpp",),
        default="",
    )
    configure.add_argument(
        "--dds-discovery-mode",
        choices=("multicast", "static"),
        default="",
    )
    configure.add_argument("--dds-static-peer", action="append")
    configure.add_argument("--clear-dds-static-peers", action="store_true")
    configure.add_argument("--dds-interface")
    configure.add_argument(
        "--dds-security-profile",
        choices=("trusted-network", "sros2"),
        default="",
    )
    configure.add_argument(
        "--dds-security-provisioning",
        choices=("external", "managed"),
        default="",
        help=argparse.SUPPRESS,
    )
    configure.add_argument(
        "--dds-security-generation",
        help=argparse.SUPPRESS,
    )
    configure.add_argument(
        "--dds-security-bundle",
        help=argparse.SUPPRESS,
    )
    configure.add_argument("--dds-keystore")
    configure.add_argument("--dds-enclave")
    configure.add_argument("--turn-url", action="append")
    configure.add_argument("--clear-turn", action="store_true")
    configure.add_argument(
        "--turn-mode",
        choices=("none", "managed", "external"),
        default="",
    )
    configure.add_argument("--turn-realm")
    configure.add_argument("--turn-public-host")
    configure.add_argument("--turn-secret-file")
    configure.add_argument("--turn-credential-file")
    configure.add_argument("--sim-id", default="")
    configure.add_argument("--pilot-id", default="")
    configure.add_argument("--ui-id", default="")
    configure.add_argument("--robot-id", default="")
    configure.add_argument("--non-interactive", action="store_true")

    doctor = subparsers.add_parser(
        "doctor",
        help="DDS graph, RGBD topic, TURN과 WebRTC 연결 검사",
    )
    doctor.add_argument("--active", action="store_true", help="실제 DDS RGBD sample까지 검사")
    doctor.add_argument(
        "--expect-peer",
        action="append",
        default=[],
        help="기대하는 Elesim endpoint ID (반복 가능)",
    )
    doctor.add_argument(
        "--strict-peers",
        action="store_true",
        help="기대 endpoint 미발견을 실패로 반환",
    )
    doctor.add_argument("--timeout", type=float, default=4.0)
    doctor.add_argument("--json", action="store_true", help="기계 판독용 JSON 출력")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    state_path = Path(args.state).expanduser().resolve()
    try:
        state = InstallState.load(state_path)
        if args.command == "show":
            print(json.dumps(state.to_dict(), ensure_ascii=False, indent=2))
            return 0
        if args.command == "namespace-check":
            # Connection-manager preflight passes a pending interface/peer
            # override before it has configured the host.  In that mode the
            # installed generated files are expected to be old; the normal
            # launch wrapper has no override and must reject stale files.
            if args.dds_interface is None and args.dds_peer is None:
                require_generated_dds_configuration(state)
            require_runtime_network_namespace(
                state,
                interface=args.dds_interface,
                address=args.dds_address,
                peers=args.dds_peer,
            )
            if args.tcp_peer:
                require_runtime_tcp_reachability(args.tcp_peer)
            interface = (
                state.dds.interface
                if args.dds_interface is None
                else str(args.dds_interface).strip()
            )
            print(
                "DDS direct-bind interface is visible: "
                f"{interface or '(automatic)'}"
            )
            return 0
        if args.command == "restore-snapshot":
            try:
                encoded = str(args.payload).encode("ascii")
                if len(encoded) > 128 * 1024:
                    raise ValueError("rollback snapshot payload가 너무 큽니다")
                decoded = base64.urlsafe_b64decode(encoded)
                if len(decoded) > 64 * 1024:
                    raise ValueError("rollback snapshot payload가 너무 큽니다")
                raw = json.loads(decoded.decode("utf-8"))
            except (UnicodeError, ValueError, json.JSONDecodeError) as exc:
                raise ValueError("rollback snapshot payload가 유효하지 않습니다") from exc
            if not isinstance(raw, Mapping):
                raise ValueError("rollback snapshot은 object여야 합니다")
            restored = InstallState.from_dict(raw).require_installable_dds()
            immutable_before = (
                state.profile,
                state.roles,
                state.prefix,
                state.bin_dir,
                state.source_root,
                state.install_mode,
            )
            immutable_after = (
                restored.profile,
                restored.roles,
                restored.prefix,
                restored.bin_dir,
                restored.source_root,
                restored.install_mode,
            )
            if immutable_after != immutable_before:
                raise ValueError("rollback snapshot이 설치 경계를 변경하려고 합니다")
            _apply_configuration_transaction(state_path, restored)
            print("rollback snapshot restored")
            return 0
        if args.command == "configure":
            override_names = (
                "dds_system_id",
                "dds_domain_id",
                "dds_rmw_implementation",
                "dds_discovery_mode",
                "dds_static_peer",
                "clear_dds_static_peers",
                "dds_interface",
                "dds_security_profile",
                "dds_security_provisioning",
                "dds_security_generation",
                "dds_security_bundle",
                "dds_keystore",
                "dds_enclave",
                "turn_url",
                "clear_turn",
                "turn_mode",
                "turn_realm",
                "turn_public_host",
                "turn_secret_file",
                "turn_credential_file",
                "sim_id",
                "pilot_id",
                "ui_id",
                "robot_id",
            )
            has_override = any(
                getattr(args, name) not in (None, "", False, [])
                for name in override_names
            )
            updated = (
                _configure_from_args(state, args)
                if args.non_interactive or has_override
                else _configure_interactive(state)
            )
            written = _apply_configuration_transaction(state_path, updated)
            print("갱신된 설정:")
            for role, path in written.items():
                print(f"  {role}: {path}")
            print("실행 중인 프로세스는 새 설정을 읽도록 재시작해야 합니다.")
            return 0
        if args.command == "doctor":
            report = NetworkDoctor(
                state,
                timeout_s=args.timeout,
                active=args.active,
                expected_peers=args.expect_peer,
                strict_peers=args.strict_peers,
            ).run()
            print(
                json.dumps(report.to_dict(), ensure_ascii=False, indent=2)
                if args.json
                else report.render()
            )
            return 0 if report.ok else 1
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"오류: {exc}", file=sys.stderr)
        return 2
    return 2


def _apply_configuration_transaction(
    state_path: Path, updated: InstallState
) -> dict[str, Path]:
    """Regenerate runtime files and state as one locally recoverable update."""

    current = InstallState.load(state_path)
    if updated.dds.security_profile == "sros2" and (
        updated.dds.security_provisioning == "external"
    ):
        before = (
            current.dds.security_profile,
            current.dds.security_provisioning,
            current.dds.keystore,
            current.dds.enclave,
            current.network.pilot_id,
            current.network.ui_id,
            current.network.sim_id,
            current.network.robot_id,
        )
        after = (
            updated.dds.security_profile,
            updated.dds.security_provisioning,
            updated.dds.keystore,
            updated.dds.enclave,
            updated.network.pilot_id,
            updated.network.ui_id,
            updated.network.sim_id,
            updated.network.robot_id,
        )
        if before != after:
            raise ValueError(
                "external SROS2 keystore/enclave 변경은 역할별 key view를 "
                "안전하게 다시 만들기 위해 재설치가 필요합니다"
            )

    targets = {state_path}
    compose_path = updated.prefix_path / "containers" / "compose.yaml"
    if compose_path.exists():
        targets.add(compose_path)
    targets.add(provisioning_required_path(updated))
    for role in updated.roles:
        targets.add(generated_config_path(updated, role))
        targets.add(generated_dds_config_path(updated, role))
        if role == "sim":
            targets.add(generated_app_config_path(updated, role))
    snapshots = {path: _snapshot(path) for path in targets}
    try:
        written = generate_role_configs(updated)
        # ``elesim-net configure`` also owns the generated Compose manifest's
        # DDS environment.  Keep it aligned with the state/XML transaction so
        # a later lifecycle start cannot reuse stale interface or peer values.
        from .container_installer import refresh_compose_dds_environment

        refresh_compose_dds_environment(updated)
        sync_provisioning_required(updated)
        updated.save(state_path)
        return written
    except BaseException:
        for path, snapshot in snapshots.items():
            _restore_snapshot(path, snapshot)
        raise


def _snapshot(path: Path) -> tuple[bytes, int] | None:
    if not path.exists():
        return None
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"설정 transaction 대상이 일반 파일이 아닙니다: {path}")
    return path.read_bytes(), stat.S_IMODE(path.stat().st_mode)


def _restore_snapshot(path: Path, snapshot: tuple[bytes, int] | None) -> None:
    if snapshot is None:
        if path.exists() and path.is_file() and not path.is_symlink():
            path.unlink()
        return
    payload, mode = snapshot
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent, prefix=f".{path.name}.rollback-"
    )
    temporary = Path(temporary_name)
    try:
        os.fchmod(descriptor, mode)
        with os.fdopen(descriptor, "wb") as handle:
            handle.write(payload)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


if __name__ == "__main__":
    raise SystemExit(main())
