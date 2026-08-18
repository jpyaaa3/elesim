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


def test_dds_inbox_drops_oldest_item_at_fixed_bound() -> None:
    node = object.__new__(DdsPeerNode)
    node.clock = lambda: 10.0
    node._max_inbox = 2
    node._inbox = deque()
    node._inbox_lock = threading.Lock()
    node._diagnostic_seen = {}

    source = PeerIdentity("pilot-a", "boot-a")
    messages = [
        make_envelope(
            "ack",
            source.endpoint_id,
            target_id="robot-a",
            payload={"reply_to": f"request-{index}"},
            seq=index,
        )
        for index in range(3)
    ]
    for message in messages:
        node._enqueue_inbox((message, source))

    assert len(node._inbox) == 2
    assert [item[0].seq for item in node._inbox] == [1, 2]


def test_dds_receive_caps_one_pump_pass() -> None:
    node = object.__new__(DdsPeerNode)
    node._inbox = deque()
    node._inbox_lock = threading.Lock()
    node.spin_once = lambda *, timeout_s: None
    source = PeerIdentity("pilot-a", "boot-a")
    for index in range(70):
        node._inbox.append(
            (
                make_envelope(
                    "ack",
                    source.endpoint_id,
                    target_id="robot-a",
                    payload={},
                    seq=index,
                ),
                source,
            )
        )

    assert len(list(node.receive())) == 64
    assert len(node._inbox) == 6


def test_peer_client_local_reply_queue_uses_max_pending_bound() -> None:
    bus = _Bus()
    client = PeerClient(
        EndpointDescriptor("ui-a", "ui"),
        node_factory=bus.factory,
        max_pending=2,
    )
    for index in range(3):
        client._local_envelope(
            "endpoint_list",
            payload={"index": index},
        )

    assert len(client._local_queue) == 2
    assert [message.payload["index"] for message in client._local_queue] == [1, 2]


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


def test_simulation_session_survives_a_bounded_discovery_gap() -> None:
    class Clock:
        now = 0.0

        def __call__(self) -> float:
            return self.now

    clock = Clock()
    bus = _Bus()
    ui = PeerClient(
        EndpointDescriptor("ui-a", "ui"),
        node_factory=bus.factory,
        clock=clock,
    )
    sim = PeerClient(
        EndpointDescriptor("sim-a", "sim"),
        node_factory=bus.factory,
        clock=clock,
    )
    ui.send(
        "open_simulation_session",
        payload={
            "schema_version": 1,
            "request_id": "gap-open",
            "sim_id": "sim-a",
            "streams": ["observer"],
        },
    )
    assert [message.message_type for message in _messages(sim)] == [
        "simulation_session_granted"
    ]
    assert [message.message_type for message in _messages(ui)] == [
        "simulation_session_opened"
    ]

    sim_node = bus.nodes["sim-a"]
    bus.nodes.pop("sim-a")
    clock.now = 1.0
    ui.heartbeat()
    assert _messages(ui) == []

    # The UI retains the exact old boot/session long enough for discovery to
    # recover, while the sim-owned session TTL remains strictly longer.
    clock.now = ui.simulation_session_grace_s - 0.1
    ui.heartbeat()
    assert _messages(ui) == []
    assert sim._session_authority is not None
    assert sim._session_authority.active(now=clock.now) is not None

    bus.nodes["sim-a"] = sim_node
    clock.now += 0.2
    ui.heartbeat()
    assert _messages(sim) == []
    assert ui._remote_session is not None


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
