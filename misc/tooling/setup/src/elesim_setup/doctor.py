"""Layered network diagnostics for Router, ZMQ RGBD, TURN and WebRTC."""

from __future__ import annotations

import asyncio
import json
import os
import socket
import struct
import threading
import time
import uuid
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Iterable, Optional
from urllib.parse import parse_qs, urlsplit

import zmq

from elesim_protocol import (
    CloseSimulationSessionRequest,
    CurveClientConfig,
    EndpointClient,
    EndpointDescriptor,
    MEDIA_SECURITY_CURVE,
    MEDIA_TRANSPORT_WEBRTC,
    MEDIA_TRANSPORT_ZMQ,
    OpenSimulationSessionRequest,
    SIMULATION_STREAMS,
    SimulationSessionOpenedPayload,
    TurnCredentials,
    WebRtcSignalPayload,
    configure_curve_client,
)

from .configuration import tcp_endpoint
from .state import InstallState


PASS = "PASS"
FAIL = "FAIL"
WARN = "WARN"
SKIP = "SKIP"


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
        lines.append(f"\n요약: PASS {passed}, FAIL {failed}, WARN {warned}, SKIP {skipped}")
        return "\n".join(lines)


def parse_tcp_endpoint(endpoint: str) -> TcpTarget:
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


def build_stun_binding_request(transaction_id: bytes | None = None) -> tuple[bytes, bytes]:
    txid = os.urandom(12) if transaction_id is None else bytes(transaction_id)
    if len(txid) != 12:
        raise ValueError("STUN transaction ID must be 12 bytes")
    return struct.pack("!HHI12s", 0x0001, 0, 0x2112A442, txid), txid


def validate_stun_response(payload: bytes, transaction_id: bytes) -> None:
    if len(payload) < 20:
        raise ValueError("short STUN response")
    message_type, body_length, magic, response_id = struct.unpack("!HHI12s", payload[:20])
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
    addresses = socket.getaddrinfo(target.host, target.port, type=socket.SOCK_DGRAM)
    if not addresses:
        raise OSError("TURN hostname did not resolve")
    family, socket_type, protocol, _canonical, address = addresses[0]
    with socket.socket(family, socket_type, protocol) as sock:
        sock.settimeout(float(timeout_s))
        sock.sendto(request, address)
        response, _source = sock.recvfrom(2048)
    validate_stun_response(response, transaction_id)


def _router_curve(state: InstallState) -> CurveClientConfig | None:
    root = state.security.root
    if state.security.mode != "curve" or root is None:
        return None
    return CurveClientConfig.from_files(
        client_secret_file=root / "curve/clients/doctor-main.key_secret",
        server_public_file=root / "curve/router/router.key",
    )


class ProtocolSession:
    """A short-lived UI identity used only by an explicit doctor invocation."""

    def __init__(self, state: InstallState, *, timeout_s: float) -> None:
        self.state = state
        self.timeout_s = float(timeout_s)
        self.client = EndpointClient(
            tcp_endpoint(state.network.router_host, state.network.router_port),
            EndpointDescriptor(endpoint_id="doctor-main", role="ui"),
            curve=_router_curve(state),
            allow_insecure_remote=state.security.allow_insecure_remote,
        )
        self.session_id = ""

    def register(self) -> None:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            self.client.heartbeat()
            for message in self.client.receive(timeout_ms=100):
                if message.message_type == "error":
                    raise RuntimeError(str((message.payload or {}).get("reason", "Router rejected registration")))
            if self.client.registered:
                return
        raise TimeoutError("Router protocol registration timed out")

    def discover(self) -> tuple[EndpointDescriptor, ...]:
        self.client.send("discover", payload={"role": "", "capability": ""})
        message = self._wait_for({"endpoint_list", "error"})
        if message.message_type == "error":
            raise RuntimeError(str((message.payload or {}).get("reason", "discovery failed")))
        raw_endpoints = (message.payload or {}).get("endpoints", ())
        if not isinstance(raw_endpoints, list):
            raise ValueError("Router endpoint_list is malformed")
        return tuple(EndpointDescriptor.from_dict(raw) for raw in raw_endpoints)

    def probe_webrtc(self, simulator_id: str) -> dict[str, int]:
        if not webrtc_available():
            raise RuntimeError("aiortc/av가 설치되지 않았습니다")
        request = OpenSimulationSessionRequest(
            request_id=uuid.uuid4().hex,
            simulator_id=simulator_id,
            streams=tuple(SIMULATION_STREAMS),
        )
        self.client.send("open_simulation_session", payload=request.to_payload())
        message = self._wait_for({"simulation_session_opened", "error"})
        if message.message_type == "error":
            raise RuntimeError(str((message.payload or {}).get("reason", "session open failed")))
        opened = SimulationSessionOpenedPayload.from_payload(message.payload or {})
        self.session_id = opened.session_id
        peers: dict[str, _VideoPeer] = {}
        try:
            for stream in opened.streams:
                peer = _VideoPeer()
                peers[stream] = peer
                offer = peer.create_offer(turn=opened.turn)
                signal = WebRtcSignalPayload(
                    session_id=opened.session_id,
                    stream=stream,
                    signal="offer",
                    sdp=offer["sdp"],
                    type=offer["type"],
                )
                self.client.send(
                    "webrtc_signal",
                    target_id=opened.simulator_id,
                    payload=signal.to_payload(),
                    lease_id=opened.session_id,
                )

            answered: set[str] = set()
            deadline = time.monotonic() + max(self.timeout_s, 8.0)
            while time.monotonic() < deadline:
                self.client.heartbeat()
                for reply in self.client.receive(timeout_ms=100):
                    if reply.message_type == "error":
                        raise RuntimeError(str((reply.payload or {}).get("reason", "WebRTC signaling failed")))
                    if reply.message_type != "webrtc_signal":
                        continue
                    signal = WebRtcSignalPayload.from_payload(reply.payload or {})
                    if signal.signal != "answer" or signal.stream not in peers:
                        continue
                    peers[signal.stream].accept_answer(signal.sdp, signal.type)
                    answered.add(signal.stream)
                if answered == set(peers) and all(peer.has_frame for peer in peers.values()):
                    return {name: peer.frame_count for name, peer in peers.items()}
            missing_answers = sorted(set(peers) - answered)
            missing_frames = sorted(name for name, peer in peers.items() if not peer.has_frame)
            raise TimeoutError(
                f"WebRTC timeout; unanswered={missing_answers or '-'} no_frame={missing_frames or '-'}"
            )
        finally:
            self._close_simulation_session()
            for peer in peers.values():
                peer.close()

    def close(self) -> None:
        self._close_simulation_session()
        self.client.close()

    def _close_simulation_session(self) -> None:
        if not self.session_id:
            return
        session_id = self.session_id
        request = CloseSimulationSessionRequest(
            request_id=uuid.uuid4().hex,
            session_id=session_id,
        )
        try:
            self.client.send(
                "close_simulation_session",
                payload=request.to_payload(),
                lease_id=session_id,
            )
            deadline = time.monotonic() + min(1.0, self.timeout_s)
            while time.monotonic() < deadline:
                for message in self.client.receive(timeout_ms=50):
                    if message.message_type in {"simulation_session_revoked", "error"}:
                        return
        except Exception:
            pass
        finally:
            self.session_id = ""

    def _wait_for(self, message_types: set[str]) -> Any:
        deadline = time.monotonic() + self.timeout_s
        while time.monotonic() < deadline:
            self.client.heartbeat()
            for message in self.client.receive(timeout_ms=100):
                if message.message_type in message_types:
                    return message
        raise TimeoutError(f"protocol response timed out: {sorted(message_types)}")


def probe_rgbd_frame(
    descriptor: Any,
    state: InstallState,
    *,
    timeout_s: float,
) -> dict[str, Any]:
    if descriptor.transport != MEDIA_TRANSPORT_ZMQ:
        raise ValueError(f"RGBD transport is {descriptor.transport!r}, not ZMQ")
    context = zmq.Context()
    socket_ = context.socket(zmq.SUB)
    socket_.setsockopt(zmq.LINGER, 0)
    socket_.setsockopt(zmq.RCVHWM, 1)
    try:
        if descriptor.security == MEDIA_SECURITY_CURVE:
            root = state.security.root
            if root is None:
                raise RuntimeError("CURVE RGBD에는 controller media client credential이 필요합니다")
            curve = CurveClientConfig.from_client_file(
                client_secret_file=root / f"curve/clients/{state.network.controller_id}.key_secret",
                server_key=descriptor.curve_server_key,
            )
            configure_curve_client(socket_, curve)
        socket_.setsockopt(zmq.SUBSCRIBE, b"")
        socket_.connect(descriptor.endpoint)
        poller = zmq.Poller()
        poller.register(socket_, zmq.POLLIN)
        events = dict(poller.poll(timeout=max(1, int(timeout_s * 1000))))
        if socket_ not in events:
            raise TimeoutError("RGBD multipart frame timeout")
        parts = socket_.recv_multipart()
        if len(parts) < 2:
            raise ValueError(f"RGBD frame has only {len(parts)} part(s)")
        metadata = json.loads(parts[0].decode("utf-8"))
        if not isinstance(metadata, dict):
            raise ValueError("RGBD metadata is not an object")
        return {
            "parts": len(parts),
            "width": int(metadata.get("width", 0)),
            "height": int(metadata.get("height", 0)),
            "seq": int(metadata.get("seq", 0)),
        }
    finally:
        socket_.close(0)
        context.term()


class NetworkDoctor:
    def __init__(self, state: InstallState, *, timeout_s: float = 4.0, active: bool = False) -> None:
        self.state = state.validate()
        self.timeout_s = max(0.2, float(timeout_s))
        self.active = bool(active)

    def run(self) -> DoctorReport:
        report = DoctorReport()
        router_target = TcpTarget(self.state.network.router_host, self.state.network.router_port)
        try:
            addresses = socket.getaddrinfo(router_target.host, router_target.port, type=socket.SOCK_STREAM)
            rendered = ", ".join(sorted({str(item[4][0]) for item in addresses}))
            report.add("Router DNS", PASS, rendered)
        except OSError as exc:
            report.add("Router DNS", FAIL, str(exc), "hostname 또는 DNS/hosts 설정을 확인하십시오")

        router_tcp_ok = self._tcp_result(report, "Router TCP", router_target)
        self._turn_results(report)
        if not router_tcp_ok:
            report.add("ZMQ protocol", SKIP, "Router TCP 실패로 생략")
            report.add("RGBD stream", SKIP, "endpoint discovery를 할 수 없음")
            report.add("WebRTC", SKIP, "signaling Router에 연결할 수 없음")
            return report

        protocol: ProtocolSession | None = None
        endpoints: tuple[EndpointDescriptor, ...] = ()
        try:
            protocol = ProtocolSession(self.state, timeout_s=self.timeout_s)
            protocol.register()
            endpoints = protocol.discover()
            roles = ", ".join(f"{item.endpoint_id}({item.role})" for item in endpoints) or "등록 endpoint 없음"
            report.add("ZMQ protocol", PASS, f"protocol-v4 register/discover: {roles}")
        except Exception as exc:
            report.add(
                "ZMQ protocol",
                FAIL,
                str(exc),
                "CURVE key/endpoint registry와 Router 로그를 확인하십시오",
            )

        simulator = next(
            (item for item in endpoints if item.endpoint_id == self.state.network.simulator_id),
            None,
        )
        try:
            self._rgbd_results(report, endpoints)
            self._webrtc_results(report, protocol, simulator)
        finally:
            if protocol is not None:
                protocol.close()
        return report

    def _tcp_result(self, report: DoctorReport, name: str, target: TcpTarget) -> bool:
        try:
            tcp_connect(target, timeout_s=self.timeout_s)
            report.add(name, PASS, f"{target.host}:{target.port}")
            return True
        except OSError as exc:
            report.add(name, FAIL, str(exc), "프로세스, bind 주소, 방화벽과 포트 포워딩을 확인하십시오")
            return False

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
                    tcp_connect(TcpTarget(target.host, target.port), timeout_s=self.timeout_s)
                    report.add(name, PASS, f"TCP {target.host}:{target.port}")
            except (OSError, ValueError) as exc:
                report.add(name, FAIL, str(exc), "Coturn listener, 공개 IP와 UDP/TCP 방화벽을 확인하십시오")

    def _rgbd_results(self, report: DoctorReport, endpoints: Iterable[EndpointDescriptor]) -> None:
        candidates = []
        for endpoint in endpoints:
            descriptor = (endpoint.streams or {}).get("rgbd")
            if descriptor is not None:
                candidates.append((endpoint, descriptor))
        if not candidates:
            report.add("RGBD stream", WARN, "광고 중인 rgbd stream이 없음")
            return
        endpoint, descriptor = next(
            ((item, stream) for item, stream in candidates if item.endpoint_id == self.state.network.simulator_id),
            candidates[0],
        )
        try:
            target = parse_tcp_endpoint(descriptor.endpoint)
            tcp_connect(target, timeout_s=self.timeout_s)
            report.add(
                "RGBD endpoint",
                PASS,
                f"{endpoint.endpoint_id}: {descriptor.endpoint} ({descriptor.security})",
            )
        except (OSError, ValueError) as exc:
            report.add("RGBD endpoint", FAIL, str(exc), "publisher advertise 주소와 TCP 5568 방화벽을 확인하십시오")
            return
        if not self.active:
            report.add("RGBD frame", SKIP, "실제 frame 검사는 --active에서 수행")
            return
        try:
            metadata = probe_rgbd_frame(descriptor, self.state, timeout_s=max(self.timeout_s, 5.0))
            report.add(
                "RGBD frame",
                PASS,
                f"multipart={metadata['parts']} {metadata['width']}x{metadata['height']} seq={metadata['seq']}",
            )
        except Exception as exc:
            report.add("RGBD frame", FAIL, str(exc), "카메라 publisher와 media CURVE allowlist를 확인하십시오")

    def _webrtc_results(
        self,
        report: DoctorReport,
        protocol: ProtocolSession | None,
        simulator: EndpointDescriptor | None,
    ) -> None:
        if simulator is None:
            report.add("WebRTC", WARN, f"Simulator {self.state.network.simulator_id!r}가 등록되지 않음")
            return
        streams = simulator.streams or {}
        missing = [
            name
            for name in SIMULATION_STREAMS
            if name not in streams or streams[name].transport != MEDIA_TRANSPORT_WEBRTC
        ]
        if missing:
            report.add("WebRTC advertise", FAIL, f"누락: {', '.join(missing)}")
            return
        report.add("WebRTC advertise", PASS, "observer + hand_eye_preview (DTLS-SRTP)")
        if not self.active:
            report.add("WebRTC frames", SKIP, "실제 ICE/signaling/frame 검사는 --active에서 수행")
            return
        if protocol is None:
            report.add("WebRTC frames", SKIP, "protocol session이 없음")
            return
        try:
            frames = protocol.probe_webrtc(simulator.endpoint_id)
            detail = ", ".join(f"{name}={count}" for name, count in sorted(frames.items()))
            report.add("WebRTC frames", PASS, detail)
        except Exception as exc:
            report.add(
                "WebRTC frames",
                FAIL,
                str(exc),
                "기존 UI session을 종료하고 ICE/TURN, aiortc와 Simulator frame source를 확인하십시오",
            )


try:
    from aiortc import RTCConfiguration, RTCIceServer, RTCPeerConnection, RTCSessionDescription
except ImportError:
    RTCConfiguration = None  # type: ignore[assignment]
    RTCIceServer = None  # type: ignore[assignment]
    RTCPeerConnection = None  # type: ignore[assignment]
    RTCSessionDescription = None  # type: ignore[assignment]


def webrtc_available() -> bool:
    return RTCPeerConnection is not None


def _ice_configuration(turn: Optional[TurnCredentials]) -> Any:
    if RTCConfiguration is None:
        return None
    if turn is None:
        return RTCConfiguration(iceServers=[])
    return RTCConfiguration(
        iceServers=[
            RTCIceServer(
                urls=list(turn.urls),
                username=turn.username,
                credential=turn.credential,
            )
        ]
    )


class _VideoPeer:
    def __init__(self) -> None:
        if not webrtc_available():
            raise RuntimeError("aiortc is unavailable")
        self.loop = asyncio.new_event_loop()
        self.thread = threading.Thread(target=self.loop.run_forever, daemon=True)
        self.thread.start()
        self.peer: Any = None
        self.frame_count = 0

    @property
    def has_frame(self) -> bool:
        return self.frame_count > 0

    def create_offer(self, *, turn: Optional[TurnCredentials]) -> dict[str, str]:
        future = asyncio.run_coroutine_threadsafe(self._create_offer(turn), self.loop)
        return future.result(timeout=15.0)

    async def _create_offer(self, turn: Optional[TurnCredentials]) -> dict[str, str]:
        self.peer = RTCPeerConnection(configuration=_ice_configuration(turn))
        self.peer.addTransceiver("video", direction="recvonly")

        @self.peer.on("track")
        def on_track(track: Any) -> None:
            if track.kind == "video":
                asyncio.create_task(self._consume(track))

        offer = await self.peer.createOffer()
        await self.peer.setLocalDescription(offer)
        return {"sdp": self.peer.localDescription.sdp, "type": self.peer.localDescription.type}

    async def _consume(self, track: Any) -> None:
        while True:
            try:
                await track.recv()
            except Exception:
                return
            self.frame_count += 1

    def accept_answer(self, sdp: str, answer_type: str) -> None:
        future = asyncio.run_coroutine_threadsafe(
            self.peer.setRemoteDescription(RTCSessionDescription(sdp=sdp, type=answer_type)),
            self.loop,
        )
        future.result(timeout=15.0)

    def close(self) -> None:
        if self.peer is not None and self.loop.is_running():
            try:
                asyncio.run_coroutine_threadsafe(self.peer.close(), self.loop).result(timeout=5.0)
            except Exception:
                pass
        if self.loop.is_running():
            self.loop.call_soon_threadsafe(self.loop.stop)
        if self.thread is not threading.current_thread():
            self.thread.join(timeout=3.0)


__all__ = [
    "DoctorReport",
    "FAIL",
    "NetworkDoctor",
    "PASS",
    "ProbeResult",
    "SKIP",
    "TcpTarget",
    "TurnTarget",
    "WARN",
    "build_stun_binding_request",
    "parse_tcp_endpoint",
    "parse_turn_url",
    "probe_rgbd_frame",
    "tcp_connect",
    "udp_stun_probe",
    "validate_stun_response",
    "webrtc_available",
]
