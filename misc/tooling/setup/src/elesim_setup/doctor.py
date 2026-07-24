"""Layered ROS 2/DDS, RGBD, TURN, and WebRTC diagnostics."""

from __future__ import annotations

import os
import socket
import struct
import time
from dataclasses import asdict, dataclass
from typing import Any, Callable, Mapping
from urllib.parse import parse_qs, urlsplit

from .configuration import generated_dds_config_path, rgbd_topic
from .state import InstallState


PASS = "PASS"
FAIL = "FAIL"
WARN = "WARN"
SKIP = "SKIP"
RGBD_TYPE = "elesim_interfaces/msg/RgbdFrame"
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
                lines.append(f"       {'':<{width}}  조치: {result.remedy}")
        passed = sum(result.status == PASS for result in self.results)
        failed = sum(result.status == FAIL for result in self.results)
        warned = sum(result.status == WARN for result in self.results)
        skipped = sum(result.status == SKIP for result in self.results)
        lines.append(
            f"\n요약: PASS {passed}, FAIL {failed}, WARN {warned}, SKIP {skipped}"
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
            "새 shell에서 generated wrapper를 사용하십시오"
        )
    os.environ["RMW_IMPLEMENTATION"] = expected_rmw
    os.environ["ROS_DOMAIN_ID"] = str(state.dds.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "0"
    config_role = state.roles[0]
    vendor_config = generated_dds_config_path(state, config_role)
    if vendor_config.is_file():
        os.environ["CYCLONEDDS_URI"] = f"file://{vendor_config}"
    if state.dds.security_profile == "sros2":
        os.environ["ROS_SECURITY_ENABLE"] = "true"
        os.environ["ROS_SECURITY_STRATEGY"] = "Enforce"
        os.environ["ROS_SECURITY_KEYSTORE"] = state.dds.keystore
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
    if import_rclpy is None:
        def import_rclpy() -> Any:
            import rclpy

            return rclpy

    rclpy = import_rclpy()
    context = rclpy.context.Context()
    rclpy.init(args=None, context=context, domain_id=state.dds.domain_id)
    node = None
    try:
        node = rclpy.create_node(
            "elesim_doctor",
            namespace=f"/{state.dds.system_id}",
            context=context,
            use_global_arguments=True,
        )
        deadline = time.monotonic() + max(0.2, float(timeout_s))
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
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
        if node is not None:
            node.destroy_node()
        context.shutdown()


def probe_rgbd_frame(
    state: InstallState,
    *,
    timeout_s: float,
) -> dict[str, int]:
    """Wait for one typed DDS RGBD sample on the configured Simulator topic."""

    _prepare_dds_environment(state)
    try:
        import rclpy
        from elesim_interfaces.msg import RgbdFrame
        from rclpy.qos import qos_profile_sensor_data
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2 overlay에서 elesim_interfaces를 찾을 수 없습니다"
        ) from exc

    context = rclpy.context.Context()
    rclpy.init(args=None, context=context, domain_id=state.dds.domain_id)
    node = None
    received: list[Any] = []
    try:
        node = rclpy.create_node(
            "elesim_rgbd_doctor",
            namespace=f"/{state.dds.system_id}",
            context=context,
            use_global_arguments=True,
        )
        subscription = node.create_subscription(
            RgbdFrame,
            rgbd_topic(state, "simulator"),
            received.append,
            qos_profile_sensor_data,
        )
        deadline = time.monotonic() + max(0.2, float(timeout_s))
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=min(0.1, deadline - time.monotonic()))
        if not received:
            raise TimeoutError("DDS RGBD frame timeout")
        message = received[0]
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
    ) -> None:
        self.state = state.require_runnable_dds()
        self.timeout_s = max(0.2, float(timeout_s))
        self.active = bool(active)

    def run(self) -> DoctorReport:
        report = DoctorReport()
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
                "ROS overlay, ROS_DOMAIN_ID, RMW, interface, multicast/static peer와 SROS2 policy를 확인하십시오",
            )
            report.add("RGBD topic", SKIP, "DDS graph에 참여할 수 없음")
            report.add("WebRTC signaling", SKIP, "DDS graph에 참여할 수 없음")
            return report

        if graph.nodes:
            report.add("DDS graph", PASS, ", ".join(graph.nodes))
        else:
            report.add(
                "DDS graph",
                WARN,
                "같은 domain에서 Elesim peer를 찾지 못함",
                "상대 프로세스, ROS_DOMAIN_ID와 DDS discovery 설정을 확인하십시오",
            )
        self._rgbd_results(report, graph)
        self._webrtc_results(report, graph)
        return report

    def _turn_results(self, report: DoctorReport) -> None:
        if not self.state.network.turn_urls:
            report.add("TURN", SKIP, "설정되지 않음; 같은 LAN에서는 선택 사항")
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
                    "Coturn listener, 공개 IP와 UDP/TCP 방화벽을 확인하십시오",
                )

    def _rgbd_results(
        self,
        report: DoctorReport,
        graph: DdsGraphSnapshot,
    ) -> None:
        topic = rgbd_topic(self.state, "simulator")
        types = graph.topics.get(topic, ())
        if RGBD_TYPE not in types:
            detail = (
                f"{topic}가 graph에 없음"
                if not types
                else f"{topic}: 예상 {RGBD_TYPE}, 실제 {', '.join(types)}"
            )
            report.add(
                "RGBD topic",
                WARN,
                detail,
                "Simulator publisher와 system_id/simulator_id를 확인하십시오",
            )
            return
        report.add("RGBD topic", PASS, f"{topic} ({RGBD_TYPE}, sensor QoS)")
        if not self.active:
            report.add("RGBD frame", SKIP, "실제 sample 검사는 --active에서 수행")
            return
        try:
            metadata = probe_rgbd_frame(
                self.state,
                timeout_s=max(self.timeout_s, 5.0),
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
                "Simulator camera publisher와 DDS QoS/SROS2 policy를 확인하십시오",
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
                f"DDS control topic 미발견 ({PEER_ENVELOPE_TYPE})",
                "Simulator peer와 endpoint descriptor/control topic을 확인하십시오",
            )
            return
        report.add(
            "WebRTC signaling",
            PASS,
            (
                f"DDS reliable control carrier {len(control_topics)}개 발견; "
                "media는 DTLS-SRTP"
            ),
        )
        report.add(
            "WebRTC frames",
            SKIP,
            (
                "실제 ICE/DTLS-SRTP frame 검사는 UI session에서 수행"
                if self.active
                else "--active는 RGBD DDS sample만 직접 검사"
            ),
        )


__all__ = [
    "FAIL",
    "PASS",
    "PEER_ENVELOPE_TYPE",
    "RGBD_TYPE",
    "SKIP",
    "WARN",
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
    "probe_rgbd_frame",
    "tcp_connect",
    "udp_stun_probe",
    "validate_stun_response",
]
