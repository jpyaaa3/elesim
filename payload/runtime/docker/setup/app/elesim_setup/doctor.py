"""Layered ROS 2/DDS, RGBD, TURN, and WebRTC diagnostics."""

from __future__ import annotations

import os
import socket
import struct
import time
from dataclasses import asdict, dataclass
from typing import Any, Callable, Mapping, Sequence
from urllib.parse import parse_qs, urlsplit

from .configuration import (
    dds_enclave,
    generated_dds_config_path,
    rgbd_topic,
    app_keystore_path,
)
from .state import InstallState


PASS = "PASS"
FAIL = "FAIL"
WARN = "WARN"
SKIP = "SKIP"
RGBD_TYPE = "elesim_interfaces/msg/RgbdFrame"
ENCODED_RGBD_TYPE = "elesim_interfaces/msg/EncodedRgbdFrame"
PEER_ENVELOPE_TYPE = "elesim_interfaces/msg/PeerEnvelope"


@dataclass(frozen=True)
class ProbeResult:
    name: str
    status: str
    detail: str
    remedy: str = ""


@dataclass(frozen=True)
class TcpTarget:
    host: str
    port: int


@dataclass(frozen=True)
class TurnTarget:
    host: str
    port: int
    transport: str


@dataclass(frozen=True)
class DdsGraphSnapshot:
    nodes: tuple[str, ...]
    topics: Mapping[str, tuple[str, ...]]
    services: Mapping[str, tuple[str, ...]]


@dataclass(frozen=True)
class DdsPeerProbe:
    """Descriptor and heartbeat observations from one bounded DDS probe."""

    descriptors: tuple[str, ...]
    heartbeats: tuple[str, ...]
    matched: tuple[str, ...] = ()

    @property
    def live_peers(self) -> tuple[str, ...]:
        """Return peers proven live by a matching descriptor and heartbeat."""

        return tuple(sorted(set(self.matched)))


def _peer_observation(message: Any) -> tuple[str, tuple[str, int]] | None:
    """Extract the identity tuple shared by descriptor and heartbeat messages."""

    peer = getattr(message, "peer", None)
    endpoint_id = str(getattr(peer, "endpoint_id", "")).strip()
    if not endpoint_id:
        return None
    boot_id = str(getattr(peer, "boot_id", "")).strip()
    try:
        revision = int(getattr(message, "descriptor_revision", 0))
    except (TypeError, ValueError):
        revision = 0
    return endpoint_id, (boot_id, revision)


def _rclpy_import() -> Any:
    """Import rclpy lazily so lightweight installer commands stay stdlib-only."""

    import rclpy

    return rclpy


def _rclpy_context(rclpy: Any, state: InstallState) -> Any:
    context = rclpy.context.Context()
    try:
        rclpy.init(args=None, context=context, domain_id=state.dds.domain_id)
    except TypeError:
        # Older Humble patch releases read ROS_DOMAIN_ID from the environment.
        rclpy.init(args=None, context=context)
    return context


def _context_executor(rclpy: Any, context: Any) -> Any | None:
    """Create an executor bound to the same context as the probe node.

    ``rclpy.spin_once(node)`` uses the process-global executor.  That executor
    is initialized against the global context, while the diagnostics create a
    private context so they can set the requested domain and security
    environment without changing a caller's ROS state.  On Humble this
    mismatch reaches ``GuardCondition`` and is reported as the unhelpful
    ``AttributeError: __enter__``.  Keep the fallback for the small injected
    rclpy doubles used by the unit tests and older lightweight environments.
    """

    module = getattr(rclpy, "executors", None)
    executor_type = getattr(module, "SingleThreadedExecutor", None)
    if not callable(executor_type):
        # Test doubles and lightweight import shims intentionally expose only
        # the small subset of rclpy they implement.  Do not import the real
        # executor module for those objects: it would bind an executor to a
        # fake context and turn a harmless compatibility fallback into a
        # constructor failure.
        if getattr(rclpy, "__name__", "") != "rclpy":
            return None
        try:
            from rclpy.executors import SingleThreadedExecutor
        except (ImportError, ModuleNotFoundError):
            return None
        executor_type = SingleThreadedExecutor
    return executor_type(context=context)


def _spin_once(
    rclpy: Any,
    node: Any,
    executor: Any | None,
    *,
    timeout_sec: float,
) -> None:
    if executor is None:
        rclpy.spin_once(node, timeout_sec=timeout_sec)
    else:
        executor.spin_once(timeout_sec=timeout_sec)


class DoctorReport:
    def __init__(self) -> None:
        self.results: list[ProbeResult] = []

    def add(self, name: str, status: str, detail: str, remedy: str = "") -> None:
        self.results.append(ProbeResult(name, status, detail, remedy))

    @property
    def ok(self) -> bool:
        return not any(result.status == FAIL for result in self.results)

    def to_dict(self) -> dict[str, Any]:
        return {
            "ok": self.ok,
            "results": [asdict(result) for result in self.results],
        }

    def render(self) -> str:
        width = max((len(result.name) for result in self.results), default=0)
        lines: list[str] = []
        for result in self.results:
            lines.append(f"[{result.status:4}] {result.name:<{width}}  {result.detail}")
            if result.remedy:
                lines.append(f"       {'':<{width}}  Action: {result.remedy}")
        passed = sum(result.status == PASS for result in self.results)
        failed = sum(result.status == FAIL for result in self.results)
        warned = sum(result.status == WARN for result in self.results)
        skipped = sum(result.status == SKIP for result in self.results)
        lines.append(
            f"\nSummary: PASS {passed}, FAIL {failed}, WARN {warned}, SKIP {skipped}"
        )
        return "\n".join(lines)


def parse_tcp_endpoint(endpoint: str) -> TcpTarget:
    """Parse a generic TCP endpoint retained for TURN and compatibility checks."""

    parsed = urlsplit(str(endpoint).strip())
    if parsed.scheme != "tcp" or not parsed.hostname or parsed.port is None:
        raise ValueError(f"invalid tcp endpoint: {endpoint!r}")
    return TcpTarget(parsed.hostname, parsed.port)


def parse_turn_url(url: str) -> TurnTarget:
    value = str(url).strip()
    parsed = urlsplit(value)
    if parsed.scheme not in {"turn", "turns"}:
        raise ValueError(f"invalid TURN URL scheme: {value!r}")
    authority = parsed.netloc or parsed.path.lstrip("/")
    target = urlsplit(f"//{authority}")
    if not target.hostname:
        raise ValueError(f"TURN URL has no host: {value!r}")
    query = parse_qs(parsed.query)
    default_transport = "tcp" if parsed.scheme == "turns" else "udp"
    transport = str(query.get("transport", [default_transport])[0]).lower()
    if transport not in {"udp", "tcp"}:
        raise ValueError(f"unsupported TURN transport: {transport!r}")
    default_port = 5349 if parsed.scheme == "turns" else 3478
    return TurnTarget(target.hostname, target.port or default_port, transport)


def tcp_connect(target: TcpTarget, *, timeout_s: float) -> None:
    with socket.create_connection((target.host, target.port), timeout=float(timeout_s)):
        return


def build_stun_binding_request(
    transaction_id: bytes | None = None,
) -> tuple[bytes, bytes]:
    txid = os.urandom(12) if transaction_id is None else bytes(transaction_id)
    if len(txid) != 12:
        raise ValueError("STUN transaction ID must be 12 bytes")
    return struct.pack("!HHI12s", 0x0001, 0, 0x2112A442, txid), txid


def validate_stun_response(payload: bytes, transaction_id: bytes) -> None:
    if len(payload) < 20:
        raise ValueError("short STUN response")
    message_type, body_length, magic, response_id = struct.unpack(
        "!HHI12s",
        payload[:20],
    )
    if message_type not in {0x0101, 0x0111}:
        raise ValueError(f"unexpected STUN response type 0x{message_type:04x}")
    if magic != 0x2112A442 or response_id != transaction_id:
        raise ValueError("STUN response transaction mismatch")
    if len(payload) < 20 + body_length:
        raise ValueError("truncated STUN response body")
    if message_type == 0x0111:
        raise ValueError("TURN/STUN server returned an error response")


def udp_stun_probe(target: TurnTarget, *, timeout_s: float) -> None:
    request, transaction_id = build_stun_binding_request()
    addresses = socket.getaddrinfo(
        target.host,
        target.port,
        type=socket.SOCK_DGRAM,
    )
    if not addresses:
        raise OSError("TURN hostname did not resolve")
    family, socket_type, protocol, _canonical, address = addresses[0]
    with socket.socket(family, socket_type, protocol) as sock:
        sock.settimeout(float(timeout_s))
        sock.sendto(request, address)
        response, _source = sock.recvfrom(2048)
    validate_stun_response(response, transaction_id)


def _prepare_dds_environment(state: InstallState) -> None:
    expected_rmw = state.dds.rmw_implementation
    current_rmw = os.environ.get("RMW_IMPLEMENTATION", "").strip()
    if current_rmw and current_rmw != expected_rmw:
        raise RuntimeError(
            f"RMW_IMPLEMENTATION={current_rmw!r}; expected {expected_rmw!r}. "
            "use the generated wrapper in a new shell"
        )
    os.environ["RMW_IMPLEMENTATION"] = expected_rmw
    os.environ["ROS_DOMAIN_ID"] = str(state.dds.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "0"
    config_role = state.runtime_roles[0]
    vendor_config = generated_dds_config_path(state, config_role)
    if vendor_config.is_file():
        os.environ["CYCLONEDDS_URI"] = f"file://{vendor_config}"
    if state.dds.security_profile == "sros2":
        os.environ["ROS_SECURITY_ENABLE"] = "true"
        os.environ["ROS_SECURITY_STRATEGY"] = "Enforce"
        os.environ["ROS_SECURITY_KEYSTORE"] = str(
            app_keystore_path(state, config_role)
        )
        os.environ["ROS_SECURITY_ENCLAVE_OVERRIDE"] = dds_enclave(
            state, config_role
        )
    else:
        os.environ["ROS_SECURITY_ENABLE"] = "false"
        os.environ.pop("ROS_SECURITY_KEYSTORE", None)
        os.environ.pop("ROS_SECURITY_ENCLAVE_OVERRIDE", None)


def probe_dds_graph(
    state: InstallState,
    *,
    timeout_s: float,
    import_rclpy: Callable[[], Any] | None = None,
) -> DdsGraphSnapshot:
    """Join the configured domain and snapshot nodes, topics, and services."""

    _prepare_dds_environment(state)
    rclpy = _rclpy_import() if import_rclpy is None else import_rclpy()
    context = _rclpy_context(rclpy, state)
    node = None
    executor = None
    try:
        node = rclpy.create_node(
            "elesim_doctor",
            namespace=f"/{state.dds.system_id}/v6",
            context=context,
            use_global_arguments=True,
        )
        executor = _context_executor(rclpy, context)
        if executor is not None:
            executor.add_node(node)
        deadline = time.monotonic() + max(0.2, float(timeout_s))
        while time.monotonic() < deadline:
            _spin_once(
                rclpy,
                node,
                executor,
                timeout_sec=min(0.1, deadline - time.monotonic()),
            )
        nodes = tuple(
            sorted(
                {
                    (
                        f"{namespace.rstrip('/')}/{name}"
                        if namespace != "/"
                        else f"/{name}"
                    )
                    for name, namespace in node.get_node_names_and_namespaces()
                    if name != "elesim_doctor"
                }
            )
        )
        topics = {
            name: tuple(types)
            for name, types in node.get_topic_names_and_types()
        }
        services = {
            name: tuple(types)
            for name, types in node.get_service_names_and_types()
        }
        return DdsGraphSnapshot(nodes=nodes, topics=topics, services=services)
    finally:
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.shutdown()


def probe_dds_peers(
    state: InstallState,
    *,
    timeout_s: float,
    import_rclpy: Callable[[], Any] | None = None,
) -> tuple[str, ...]:
    """Collect endpoint IDs advertised on the EleSim discovery carrier.

    The ROS graph can contain a node while the application-level peer
    descriptor is still absent.  Runtime control addresses endpoint IDs, so a
    node/topic snapshot alone cannot explain ``target peer ... is not active``.
    This compatibility wrapper preserves the descriptor-only API; strict
    readiness uses :func:`probe_dds_peer_state` and requires both descriptor
    and heartbeat observations.
    """

    return probe_dds_peer_state(
        state,
        timeout_s=timeout_s,
        import_rclpy=import_rclpy,
    ).descriptors


def probe_dds_peer_state(
    state: InstallState,
    *,
    timeout_s: float,
    expected_peers: Sequence[str] = (),
    import_rclpy: Callable[[], Any] | None = None,
) -> DdsPeerProbe:
    """Observe descriptors and live heartbeats on the application carrier.

    A transient-local descriptor can outlive the process that published it.
    The application peer directory therefore requires a subsequent volatile
    heartbeat before it considers an endpoint active.  The doctor mirrors
    that distinction so a stale descriptor cannot make a broken UDP path look
    healthy.
    """

    _prepare_dds_environment(state)
    try:
        from elesim_interfaces.msg import EndpointDescriptor, EndpointHeartbeat
        from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    except ImportError as exc:
        raise RuntimeError(
            "EleSim discovery message was not found in the ROS 2 overlay"
        ) from exc

    rclpy = _rclpy_import() if import_rclpy is None else import_rclpy()
    context = _rclpy_context(rclpy, state)
    node = None
    executor = None
    descriptor_ids: set[str] = set()
    heartbeat_ids: set[str] = set()
    descriptor_identity: dict[str, tuple[str, int]] = {}
    heartbeat_identity: dict[str, tuple[str, int]] = {}
    matched_ids: set[str] = set()
    expected_ids = {
        str(value).strip() for value in expected_peers if str(value).strip()
    }
    try:
        node = rclpy.create_node(
            "elesim_peer_doctor",
            namespace=f"/{state.dds.system_id}/v6",
            context=context,
            use_global_arguments=True,
        )
        executor = _context_executor(rclpy, context)
        if executor is not None:
            executor.add_node(node)
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=64,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        def refresh_match(endpoint_id: str) -> None:
            descriptor = descriptor_identity.get(endpoint_id)
            heartbeat = heartbeat_identity.get(endpoint_id)
            if (
                descriptor is not None
                and descriptor == heartbeat
                and descriptor[0]
                and descriptor[1] > 0
            ):
                matched_ids.add(endpoint_id)
            else:
                matched_ids.discard(endpoint_id)

        def record_observation(
            message: Any,
            seen: set[str],
            identities: dict[str, tuple[str, int]],
        ) -> None:
            observation = _peer_observation(message)
            if observation is None:
                return
            endpoint_id, identity = observation
            seen.add(endpoint_id)
            identities[endpoint_id] = identity
            refresh_match(endpoint_id)

        def on_descriptor(message: Any) -> None:
            record_observation(message, descriptor_ids, descriptor_identity)

        def on_heartbeat(message: Any) -> None:
            record_observation(message, heartbeat_ids, heartbeat_identity)

        descriptor_subscription = node.create_subscription(
            EndpointDescriptor,
            f"/{state.dds.system_id}/v6/discovery/endpoints",
            on_descriptor,
            qos,
        )
        heartbeat_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=64,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        heartbeat_subscription = node.create_subscription(
            EndpointHeartbeat,
            f"/{state.dds.system_id}/v6/discovery/heartbeats",
            on_heartbeat,
            heartbeat_qos,
        )
        deadline = time.monotonic() + max(0.2, float(timeout_s))
        while time.monotonic() < deadline:
            remaining = max(0.0, deadline - time.monotonic())
            _spin_once(
                rclpy,
                node,
                executor,
                timeout_sec=min(0.1, remaining),
            )
            if (
                expected_ids
                and expected_ids.issubset(matched_ids)
            ):
                break
        node.destroy_subscription(descriptor_subscription)
        node.destroy_subscription(heartbeat_subscription)
        return DdsPeerProbe(
            descriptors=tuple(sorted(descriptor_ids)),
            heartbeats=tuple(sorted(heartbeat_ids)),
            matched=tuple(sorted(matched_ids)),
        )
    finally:
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.shutdown()


def probe_rgbd_frame(
    state: InstallState,
    *,
    timeout_s: float,
    encoded: bool = False,
) -> dict[str, int]:
    """Wait for one typed DDS RGBD sample on the configured Sim topic."""

    _prepare_dds_environment(state)
    try:
        import rclpy
        from elesim_interfaces.msg import EncodedRgbdFrame, RgbdFrame
        from rclpy.qos import qos_profile_sensor_data
    except ImportError as exc:
        raise RuntimeError(
            "elesim_interfaces was not found in the ROS 2 overlay"
        ) from exc

    context = rclpy.context.Context()
    rclpy.init(args=None, context=context, domain_id=state.dds.domain_id)
    node = None
    executor = None
    received: list[Any] = []
    try:
        node = rclpy.create_node(
            "elesim_rgbd_doctor",
            namespace=f"/{state.dds.system_id}/v6",
            context=context,
            use_global_arguments=True,
        )
        executor = _context_executor(rclpy, context)
        if executor is not None:
            executor.add_node(node)
        message_type = EncodedRgbdFrame if encoded else RgbdFrame
        subscription = node.create_subscription(
            message_type,
            rgbd_topic(state, "sim"),
            received.append,
            qos_profile_sensor_data,
        )
        deadline = time.monotonic() + max(0.2, float(timeout_s))
        while not received and time.monotonic() < deadline:
            _spin_once(
                rclpy,
                node,
                executor,
                timeout_sec=min(0.1, deadline - time.monotonic()),
            )
        if not received:
            raise TimeoutError("DDS RGBD frame timeout")
        message = received[0]
        if encoded:
            return {
                "color_width": int(getattr(message, "width", 0)),
                "color_height": int(getattr(message, "height", 0)),
                "depth_width": int(getattr(message, "width", 0)) if bool(getattr(message, "has_depth", False)) else 0,
                "depth_height": int(getattr(message, "height", 0)) if bool(getattr(message, "has_depth", False)) else 0,
            }
        color = getattr(message, "color", None)
        depth = getattr(message, "depth", None)
        result = {
            "color_width": int(getattr(color, "width", 0)),
            "color_height": int(getattr(color, "height", 0)),
            "depth_width": int(getattr(depth, "width", 0)),
            "depth_height": int(getattr(depth, "height", 0)),
        }
        node.destroy_subscription(subscription)
        return result
    finally:
        if executor is not None:
            if node is not None:
                executor.remove_node(node)
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.shutdown()


class NetworkDoctor:
    def __init__(
        self,
        state: InstallState,
        *,
        timeout_s: float = 4.0,
        active: bool = False,
        expected_peers: Sequence[str] = (),
        strict_peers: bool = False,
        readiness_only: bool = False,
    ) -> None:
        self.state = state.require_runnable_dds()
        self.timeout_s = max(0.2, float(timeout_s))
        self.active = bool(active)
        self.expected_peers = tuple(
            sorted({str(value).strip() for value in expected_peers if str(value).strip()})
        )
        self.strict_peers = bool(strict_peers)
        self.readiness_only = bool(readiness_only)
        if self.readiness_only and not self.strict_peers:
            raise ValueError("readiness-only doctor requires strict peer checks")
        if self.readiness_only and not self.expected_peers:
            raise ValueError("readiness-only doctor requires expected peers")
        if self.readiness_only and self.active:
            raise ValueError("readiness-only doctor cannot run active media checks")

    def run(self) -> DoctorReport:
        report = DoctorReport()
        if self.readiness_only:
            self._peer_results(report)
            return report
        # Imported lazily because the CLI module also imports NetworkDoctor.
        from .network import detect_tailscale

        tailscale = detect_tailscale()
        if tailscale.available:
            report.add(
                "Tailscale",
                PASS,
                f"{tailscale.interface}: {', '.join(tailscale.addresses)}",
                "enter this address and interface as current values in the connection manager; do not pin them",
            )
        else:
            report.add(
                "Tailscale",
                WARN,
                tailscale.detail,
                "install and log in to Tailscale only when using it, then enter the current tailscale* address",
            )
        report.add(
            "DDS configuration",
            PASS,
            (
                f"system={self.state.dds.system_id} "
                f"domain={self.state.dds.domain_id} "
                f"rmw={self.state.dds.rmw_implementation} "
                f"discovery={self.state.dds.discovery_mode} "
                f"security={self.state.dds.security_profile}"
            ),
        )
        self._turn_results(report)
        try:
            graph = probe_dds_graph(self.state, timeout_s=self.timeout_s)
        except Exception as exc:
            report.add(
                "DDS graph",
                FAIL,
                str(exc),
                "check the ROS overlay, ROS_DOMAIN_ID, RMW, interface, multicast/static peers, and SROS2 policy",
            )
            report.add("RGBD topic", SKIP, "cannot join the DDS graph")
            report.add("WebRTC signaling", SKIP, "cannot join the DDS graph")
            return report

        if graph.nodes:
            report.add("DDS graph", PASS, ", ".join(graph.nodes))
        else:
            report.add(
                "DDS graph",
                WARN,
                "no EleSim peer found in the same domain",
                "check the peer process, ROS_DOMAIN_ID, and DDS discovery settings",
            )
        self._peer_results(report)
        self._rgbd_results(report, graph)
        self._webrtc_results(report, graph)
        return report

    def _peer_results(self, report: DoctorReport) -> None:
        if not self.expected_peers:
            report.add("DDS peers", SKIP, "no expected endpoint was specified")
            return
        try:
            probe = probe_dds_peer_state(
                self.state,
                timeout_s=self.timeout_s,
                expected_peers=self.expected_peers,
            )
        except Exception as exc:
            report.add(
                "DDS peers",
                FAIL if self.strict_peers else WARN,
                str(exc),
                "check the ROS 2 overlay and DDS discovery carrier",
            )
            return
        descriptors = set(probe.descriptors)
        heartbeats = set(probe.heartbeats)
        live_peers = set(probe.live_peers)
        missing_descriptors = tuple(
            peer for peer in self.expected_peers if peer not in descriptors
        )
        missing_heartbeats = tuple(
            peer for peer in self.expected_peers if peer not in heartbeats
        )
        missing_live = tuple(
            peer for peer in self.expected_peers if peer not in live_peers
        )
        missing = tuple(
            peer
            for peer in self.expected_peers
            if peer in missing_descriptors or peer in missing_heartbeats
        )
        if not missing and not missing_live:
            report.add(
                "DDS peers",
                PASS,
                f"checked descriptor/heartbeat for {len(self.expected_peers)} endpoints: "
                f"{', '.join(self.expected_peers)}",
            )
            return
        status = FAIL if self.strict_peers else WARN
        descriptor_text = ", ".join(sorted(descriptors)) or "none"
        heartbeat_text = ", ".join(sorted(heartbeats)) or "none"
        detail_parts: list[str] = []
        if missing:
            detail_parts.append(f"missing: {', '.join(missing)}")
        if missing_descriptors:
            detail_parts.append(f"missing descriptor: {', '.join(missing_descriptors)}")
        if missing_heartbeats:
            detail_parts.append(f"missing heartbeat: {', '.join(missing_heartbeats)}")
        mismatched = tuple(
            peer
            for peer in self.expected_peers
            if peer not in missing_descriptors
            and peer not in missing_heartbeats
            and peer in missing_live
        )
        if mismatched:
            detail_parts.append(
                "descriptor/heartbeat boot or revision mismatch: "
                + ", ".join(mismatched)
            )
        report.add(
            "DDS peers",
            status,
            "; ".join(detail_parts)
            + f" (descriptor: {descriptor_text}; heartbeat: {heartbeat_text})",
            (
                "verify that all hosts use the same DDS domain/RMW/security and that "
                "the selected interface and static-peer paths are visible in the "
                "runtime namespace. Docker Desktop/WSL Tailscale TCP helpers do not "
                "forward DDS UDP."
            ),
        )

    def _turn_results(self, report: DoctorReport) -> None:
        if not self.state.network.turn_urls:
            report.add("TURN", SKIP, "not configured; optional on the same LAN")
            return
        for index, value in enumerate(self.state.network.turn_urls, start=1):
            name = f"TURN {index}"
            try:
                target = parse_turn_url(value)
                if target.transport == "udp":
                    udp_stun_probe(target, timeout_s=self.timeout_s)
                    report.add(name, PASS, f"UDP STUN {target.host}:{target.port}")
                else:
                    tcp_connect(
                        TcpTarget(target.host, target.port),
                        timeout_s=self.timeout_s,
                    )
                    report.add(name, PASS, f"TCP {target.host}:{target.port}")
            except (OSError, ValueError) as exc:
                report.add(
                    name,
                    FAIL,
                    str(exc),
                    "check the Coturn listener, public IP, and UDP/TCP firewall",
                )

    def _rgbd_results(
        self,
        report: DoctorReport,
        graph: DdsGraphSnapshot,
    ) -> None:
        topic = rgbd_topic(self.state, "sim")
        types = graph.topics.get(topic, ())
        expected_type = ENCODED_RGBD_TYPE if ENCODED_RGBD_TYPE in types else RGBD_TYPE
        if expected_type not in types:
            detail = (
                f"{topic} is not in the graph"
                if not types
                else f"{topic}: expected {expected_type}, found {', '.join(types)}"
            )
            report.add(
                "RGBD topic",
                WARN,
                detail,
                "check the Sim publisher and system_id/sim_id",
            )
            return
        report.add("RGBD topic", PASS, f"{topic} ({expected_type}, sensor QoS)")
        if not self.active:
            report.add("RGBD frame", SKIP, "live sample checks require --active")
            return
        try:
            metadata = probe_rgbd_frame(
                self.state,
                timeout_s=max(self.timeout_s, 5.0),
                encoded=expected_type == ENCODED_RGBD_TYPE,
            )
            report.add(
                "RGBD frame",
                PASS,
                (
                    f"color={metadata['color_width']}x{metadata['color_height']} "
                    f"depth={metadata['depth_width']}x{metadata['depth_height']}"
                ),
            )
        except Exception as exc:
            report.add(
                "RGBD frame",
                FAIL,
                str(exc),
                "check the Sim camera publisher and DDS QoS/SROS2 policy",
            )

    def _webrtc_results(
        self,
        report: DoctorReport,
        graph: DdsGraphSnapshot,
    ) -> None:
        control_topics = tuple(
            sorted(
                topic
                for topic, types in graph.topics.items()
                if topic.endswith("/control") and PEER_ENVELOPE_TYPE in types
            )
        )
        if not control_topics:
            report.add(
                "WebRTC signaling",
                WARN,
                f"DDS control topic missing ({PEER_ENVELOPE_TYPE})",
                "check the Sim peer and endpoint descriptor/control topic",
            )
            return
        report.add(
            "WebRTC signaling",
            PASS,
            (
                f"found {len(control_topics)} DDS reliable control carriers; "
                "media uses DTLS-SRTP"
            ),
        )
        report.add(
            "WebRTC frames",
            SKIP,
            (
                "live ICE/DTLS-SRTP frame checks run in the UI session"
                if self.active
                else "--active checks only live RGBD DDS samples"
            ),
        )


__all__ = [
    "FAIL",
    "PASS",
    "PEER_ENVELOPE_TYPE",
    "RGBD_TYPE",
    "SKIP",
    "WARN",
    "DdsPeerProbe",
    "DdsGraphSnapshot",
    "DoctorReport",
    "NetworkDoctor",
    "ProbeResult",
    "TcpTarget",
    "TurnTarget",
    "build_stun_binding_request",
    "parse_tcp_endpoint",
    "parse_turn_url",
    "probe_dds_graph",
    "probe_dds_peers",
    "probe_dds_peer_state",
    "probe_rgbd_frame",
    "tcp_connect",
    "udp_stun_probe",
    "validate_stun_response",
]
