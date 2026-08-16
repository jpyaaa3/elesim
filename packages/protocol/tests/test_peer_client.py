from __future__ import annotations

import threading
from collections import deque
from dataclasses import replace
from types import SimpleNamespace

from elesim_protocol import (
    DdsPeerNode,
    DdsTransportError,
    EndpointDescriptor,
    PeerClient,
    PeerDescriptor,
    PeerDirectory,
    PeerHeartbeat,
    PeerIdentity,
    make_envelope,
)
from elesim_protocol.dds_transport import DiscoveredPeer, peer_node_key


class _Bus:
    def __init__(self) -> None:
        self.nodes: dict[str, _Node] = {}
        self.boot = 0

    def factory(self, descriptor: EndpointDescriptor, **_kwargs: object) -> "_Node":
        self.boot += 1
        node = _Node(self, descriptor, f"boot-{self.boot}")
        self.nodes[descriptor.endpoint_id] = node
        return node


class _Node:
    def __init__(
        self,
        bus: _Bus,
        descriptor: EndpointDescriptor,
        boot_id: str,
    ) -> None:
        self.bus = bus
        self.identity = PeerIdentity(descriptor.endpoint_id, boot_id)
        self.descriptor = replace(descriptor, instance_id=boot_id)
        self.registered = True
        self.queue: list[tuple[object, PeerIdentity]] = []

    def heartbeat(self, *, force: bool = False) -> None:
        del force

    def transport_ready(self) -> bool:
        return self.registered

    def discover(
        self,
        *,
        role: str = "",
        capability: str = "",
    ) -> tuple[DiscoveredPeer, ...]:
        result = []
        for node in self.bus.nodes.values():
            if node is self:
                continue
            descriptor = node.descriptor
            if role and descriptor.role != role:
                continue
            if capability and capability not in descriptor.capabilities:
                continue
            result.append(self._discovered(node))
        return tuple(result)

    def resolve(self, endpoint_id: str) -> DiscoveredPeer | None:
        node = self.bus.nodes.get(endpoint_id)
        return None if node is None else self._discovered(node)

    def describe(self, identity: PeerIdentity) -> DiscoveredPeer | None:
        node = self.bus.nodes.get(identity.endpoint_id)
        if node is None or node.identity != identity:
            return None
        return self._discovered(node)

    def publish(self, envelope: object, *, motion: bool = False) -> None:
        del motion
        target = self.bus.nodes.get(envelope.target_id)
        if target is None:
            raise DdsTransportError("target unavailable")
        target.queue.append((envelope, self.identity))

    def receive(self, timeout_ms: int = 0):
        del timeout_ms
        while self.queue:
            yield self.queue.pop(0)

    def close(self) -> None:
        self.registered = False
        self.bus.nodes.pop(self.identity.endpoint_id, None)

    @staticmethod
    def _discovered(node: "_Node") -> DiscoveredPeer:
        key = peer_node_key(node.identity.endpoint_id)
        prefix = f"/elesim/v6/peers/{key}/{node.identity.boot_id}"
        return DiscoveredPeer(
            descriptor=node.descriptor,
            identity=node.identity,
            node_key=key,
            service_prefix=prefix,
            topic_prefix=prefix,
        )


def _messages(client: PeerClient) -> list[object]:
    return list(client.receive())


def test_startup_racing_message_waits_for_exact_source_descriptor() -> None:
    now = [10.0]
    node = object.__new__(DdsPeerNode)
    node.clock = lambda: now[0]
    node.settings = SimpleNamespace(heartbeat_timeout_s=3.5)
    node.directory = PeerDirectory(heartbeat_timeout_s=3.5)
    node._inbox = deque()
    node._pending_inbound = deque()
    node._inbox_lock = threading.Lock()

    source = PeerIdentity("pilot-a", "boot-a")
    message = make_envelope(
        "ack",
        source.endpoint_id,
        target_id="robot-a",
        payload={"reply_to": "request-a", "ok": True, "reason": ""},
        seq=1,
    )
    node._defer_inbound(message, source)
    assert list(node._inbox) == []

    node.directory.announce(
        PeerDescriptor(source, role="pilot"),
        now=now[0],
    )
    node.directory.heartbeat(
        PeerHeartbeat(source, descriptor_revision=1, sequence=1),
        now=now[0],
    )
    node._release_pending_inbound(source)
    assert list(node._inbox) == [(message, source)]
    assert list(node._pending_inbound) == []

    node._defer_inbound(message, source)
    now[0] = 14.0
    node._expire_pending_inbound(now[0])
    assert list(node._pending_inbound) == []


def test_heartbeat_waiting_for_descriptor_is_replayed_once_descriptor_arrives() -> None:
    now = [10.0]
    node = object.__new__(DdsPeerNode)
    node.clock = lambda: now[0]
    node.identity = PeerIdentity("ui-a", "boot-ui")
    node.settings = SimpleNamespace(
        heartbeat_timeout_s=3.5,
        security_profile="trusted-network",
    )
    node.directory = PeerDirectory(heartbeat_timeout_s=3.5)
    node._wire_descriptors = {}
    node._pending_heartbeats = {}
    node._diagnostic_seen = {}
    node._inbox = deque()
    node._pending_inbound = deque()
    node._inbox_lock = threading.Lock()
    diagnostics: list[dict[str, object]] = []
    node._diagnostic = lambda _channel, **fields: diagnostics.append(fields)
    node._release_pending_inbound = lambda _identity: None

    source = PeerIdentity("sim-a", "boot-sim")
    heartbeat = SimpleNamespace(
        peer=SimpleNamespace(endpoint_id=source.endpoint_id, boot_id=source.boot_id),
        descriptor_revision=1,
        sequence=1,
    )
    node._on_heartbeat(heartbeat)
    assert node.directory.resolve("sim-a", now=now[0]) is None
    assert any(
        item.get("state") == "heartbeat-before-descriptor" for item in diagnostics
    )

    descriptor = SimpleNamespace(
        protocol_major=6,
        peer=SimpleNamespace(endpoint_id=source.endpoint_id, boot_id=source.boot_id),
        role="sim",
        capabilities=[],
        streams=[],
        descriptor_revision=1,
        service_prefix="/elesim/v6/peers/sim_a/boot_sim",
        topic_prefix="/elesim/v6/peers/sim_a/boot_sim",
        interface_hash="elesim-v6",
    )
    node._on_descriptor(descriptor)

    assert node.directory.resolve("sim-a", now=now[0]) is not None
    assert any(item.get("state") == "ready" for item in diagnostics)
    assert node._pending_heartbeats == {}


def test_direct_discovery_motion_lease_and_fenced_command() -> None:
    bus = _Bus()
    pilot = PeerClient(
        EndpointDescriptor("pilot-a", "pilot"),
        node_factory=bus.factory,
    )
    robot = PeerClient(
        EndpointDescriptor("robot-a", "robot", ("motion.arm",)),
        node_factory=bus.factory,
    )

    pilot.send("discover", payload={"role": "robot"})
    endpoint_list = _messages(pilot)
    assert endpoint_list[0].message_type == "endpoint_list"
    assert endpoint_list[0].payload["endpoints"][0]["endpoint_id"] == "robot-a"

    pilot.send("select_target", payload={"target_id": "robot-a"})
    grants = _messages(robot)
    assert [message.message_type for message in grants] == ["lease_granted"]
    lease_id = grants[0].lease_id
    selected = _messages(pilot)
    assert selected[0].message_type == "target_selected"
    assert selected[0].lease_id == lease_id

    pilot.send(
        "motion_command",
        target_id="robot-a",
        payload={"command": "torque_off"},
        lease_id=lease_id,
    )
    motion = _messages(robot)
    assert [message.message_type for message in motion] == ["motion_command"]

    pilot.send("release_target", payload={})
    revoked = _messages(robot)
    assert [message.message_type for message in revoked] == ["lease_revoked"]
    released = _messages(pilot)
    assert [message.message_type for message in released] == ["target_released"]


def test_sim_owns_ui_session_and_webrtc_signaling_fence() -> None:
    bus = _Bus()
    ui = PeerClient(
        EndpointDescriptor("ui-a", "ui"),
        node_factory=bus.factory,
    )
    sim = PeerClient(
        EndpointDescriptor("sim-a", "sim"),
        node_factory=bus.factory,
    )
    open_payload = {
        "schema_version": 1,
        "request_id": "open-1",
        "sim_id": "sim-a",
        "streams": ["observer"],
    }
    ui.send("open_simulation_session", payload=open_payload)
    local = _messages(sim)
    assert [message.message_type for message in local] == [
        "simulation_session_granted"
    ]
    session_id = local[0].lease_id
    opened = _messages(ui)
    assert opened[0].message_type == "simulation_session_opened"
    assert opened[0].lease_id == session_id

    ui.send(
        "webrtc_signal",
        target_id="sim-a",
        lease_id=session_id,
        payload={
            "schema_version": 1,
            "session_id": session_id,
            "stream": "observer",
            "signal": "offer",
            "sdp": "v=0",
            "type": "offer",
        },
    )
    assert [message.message_type for message in _messages(sim)] == [
        "webrtc_signal"
    ]

    ui.send(
        "close_simulation_session",
        lease_id=session_id,
        payload={
            "schema_version": 1,
            "request_id": "close-1",
            "session_id": session_id,
        },
    )
    assert [message.message_type for message in _messages(sim)] == [
        "simulation_session_revoked"
    ]
    assert [message.message_type for message in _messages(ui)] == [
        "simulation_session_revoked"
    ]


def test_sim_rejects_session_until_runtime_readiness_gate_opens() -> None:
    bus = _Bus()
    ui = PeerClient(
        EndpointDescriptor("ui-a", "ui"),
        node_factory=bus.factory,
    )
    sim = PeerClient(
        EndpointDescriptor("sim-a", "sim"),
        node_factory=bus.factory,
        simulation_session_ready_provider=lambda: (False, "scene is still building"),
    )

    ui.send(
        "open_simulation_session",
        payload={
            "schema_version": 1,
            "request_id": "open-before-ready",
            "sim_id": "sim-a",
            "streams": ["observer"],
        },
    )

    assert _messages(sim) == []
    errors = _messages(ui)
    assert len(errors) == 1
    assert errors[0].message_type == "error"
    assert "scene is still building" in errors[0].payload["reason"]

    sim.simulation_session_ready_provider = lambda: (True, "ready")
    ui.send(
        "open_simulation_session",
        payload={
            "schema_version": 1,
            "request_id": "open-after-ready",
            "sim_id": "sim-a",
            "streams": ["observer"],
        },
    )
    assert [message.message_type for message in _messages(sim)] == [
        "simulation_session_granted"
    ]


def test_second_pilot_cannot_take_busy_target() -> None:
    bus = _Bus()
    first = PeerClient(
        EndpointDescriptor("pilot-a", "pilot"),
        node_factory=bus.factory,
    )
    second = PeerClient(
        EndpointDescriptor("pilot-b", "pilot"),
        node_factory=bus.factory,
    )
    robot = PeerClient(
        EndpointDescriptor("robot-a", "robot"),
        node_factory=bus.factory,
    )
    first.send("select_target", payload={"target_id": "robot-a"})
    _messages(robot)
    _messages(first)

    second.send("select_target", payload={"target_id": "robot-a"})
    assert _messages(robot) == []
    error = _messages(second)
    assert error[0].message_type == "error"
    assert "unavailable" in error[0].payload["reason"]
