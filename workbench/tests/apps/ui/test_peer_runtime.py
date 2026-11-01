from __future__ import annotations

from collections import deque

import time
import threading
from types import SimpleNamespace

import pytest

from elesim_protocol import DdsTransportError, Envelope, make_envelope
from elesim_ui.peer_runtime import UiPeerHub
from elesim_ui.sim_session import UiSimSession
from elesim_protocol import CAPABILITY_SIM_SCENE_REBUILD, EndpointDescriptor


def test_slow_shared_discovery_does_not_block_ui_session_snapshot():
    entered, release, readable = threading.Event(), threading.Event(), threading.Event()
    errors = []

    def discover(**_):
        entered.set()
        assert release.wait(2.), 'discovery test was not released'
        return ()

    client = Client()
    client.node = SimpleNamespace(discover=discover)
    hub = UiPeerHub(endpoint_id='ui-a', client=client, autostart=False)
    session = UiSimSession(ui_id='ui-a', sim_id='sim-a', peer=hub.channel('sim'), autostart=False)

    def cycle():
        try:
            session.run_cycle(session.peer)
        except BaseException as error:
            errors.append(error)

    def read_snapshot():
        session.snapshot
        readable.set()

    worker = threading.Thread(target=cycle)
    reader = threading.Thread(target=read_snapshot)
    try:
        worker.start()
        assert entered.wait(1.)
        reader.start()
        assert readable.wait(.5), 'UI snapshot waited for network discovery'
    finally:
        release.set()
        worker.join(2.)
        if reader.ident is not None:
            reader.join(2.)
        session.close()
        hub.close()
    assert not errors
    assert not worker.is_alive() and not reader.is_alive()


def test_production_shared_channel_exposes_selected_sim_rebuild_capability():
    peers = [SimpleNamespace(descriptor=EndpointDescriptor('sim-a','sim',
        (CAPABILITY_SIM_SCENE_REBUILD,)))]
    client = Client()
    client.node = SimpleNamespace(discover=lambda **_:tuple(peers))
    hub = UiPeerHub(endpoint_id='ui-a',client=client,autostart=False)
    channel = hub.channel('sim')
    session = UiSimSession(ui_id='ui-a',sim_id='sim-a',peer=channel,autostart=False)
    session._session_id, session._active_sim_id = 'lease-a','sim-a'
    try:
        session.run_cycle(channel)
        assert session.supports_scene_rebuild
        peers.clear()
        session.run_cycle(channel)
        assert not session.supports_scene_rebuild
    finally:
        session.close()
        hub.close()


def test_shared_discovery_result_is_discarded_after_target_changes():
    def discover(**_):
        session.switch_target('sim-b')
        return (SimpleNamespace(descriptor=EndpointDescriptor('sim-a', 'sim',
            (CAPABILITY_SIM_SCENE_REBUILD,))),)

    client = Client()
    client.node = SimpleNamespace(discover=discover)
    hub = UiPeerHub(endpoint_id='ui-a', client=client, autostart=False)
    session = UiSimSession(ui_id='ui-a', sim_id='sim-a', peer=hub.channel('sim'), autostart=False)
    session._session_id, session._active_sim_id = 'lease-a', 'sim-a'
    try:
        session.run_cycle(session.peer)
        assert not session.supports_scene_rebuild
    finally:
        session.close()
        hub.close()


@pytest.mark.parametrize('endpoint,capabilities', [('other-sim',(CAPABILITY_SIM_SCENE_REBUILD,)),('sim-a',())])
def test_shared_channel_never_borrows_another_sim_or_missing_capability(endpoint, capabilities):
    client = Client()
    client.node = SimpleNamespace(discover=lambda **_:tuple([
        SimpleNamespace(descriptor=EndpointDescriptor(endpoint,'sim',capabilities))]))
    hub = UiPeerHub(endpoint_id='ui-a',client=client,autostart=False)
    session = UiSimSession(ui_id='ui-a',sim_id='sim-a',peer=hub.channel('sim'),autostart=False)
    session._session_id, session._active_sim_id = 'lease-a','sim-a'
    try:
        session.run_cycle(session.peer)
        assert not session.supports_scene_rebuild
    finally:
        session.close()
        hub.close()


class Client:
    def __init__(self) -> None:
        self.registered = True
        self.inbox: deque[Envelope] = deque()
        self.sent: list[Envelope] = []
        self.closed = False
        self.fail_heartbeat = False
        self.peers: set[str] = set()

    def heartbeat(self) -> None:
        if self.fail_heartbeat:
            raise DdsTransportError("DDS socket reset")
        return None

    def has_peer(self, endpoint_id: str) -> bool:
        return str(endpoint_id) in self.peers

    def receive(self, timeout_ms: int = 0):
        while self.inbox:
            yield self.inbox.popleft()

    def send(self, message_type: str, **kwargs: object) -> Envelope:
        message = make_envelope(
            message_type,
            "ui-a",
            target_id=str(kwargs.get("target_id", "sim-a")),
            payload=dict(kwargs.get("payload", {})),
            seq=len(self.sent) + 1,
        )
        self.sent.append(message)
        return message

    def close(self) -> None:
        self.closed = True


def test_one_ui_peer_demultiplexes_operator_and_sim_messages() -> None:
    client = Client()
    hub = UiPeerHub(endpoint_id="ui-a", client=client, autostart=False)
    operator = hub.channel("operator")
    sim = hub.channel("sim")

    hub._dispatch(
        make_envelope(
            "operator_result",
            "pilot-a",
            target_id="ui-a",
            payload={"request_id": "one", "ok": True},
            seq=1,
        )
    )
    hub._dispatch(
        make_envelope(
            "simulation_status",
            "sim-a",
            target_id="ui-a",
            payload={},
            seq=2,
        )
    )

    assert [item.message_type for item in operator.receive()] == ["operator_result"]
    assert [item.message_type for item in sim.receive()] == ["simulation_status"]
    hub.close()
    assert client.closed is True


def test_peer_error_returns_to_the_channel_that_sent_the_request() -> None:
    client = Client()
    hub = UiPeerHub(endpoint_id="ui-a", client=client, autostart=False)
    sim = hub.channel("sim")
    sent = sim.send("open_simulation_session", payload={})
    hub._dispatch(
        make_envelope(
            "error",
            "sim-a",
            target_id="ui-a",
            payload={"reply_to": sent.message_id, "reason": "unavailable"},
            seq=2,
        )
    )

    assert [item.message_type for item in sim.receive()] == ["error"]
    assert tuple(hub.channel("operator").receive()) == ()
    hub.close()


def _wait_until(predicate, *, timeout: float = 1.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.01)
    assert predicate()


def test_sim_channel_waits_for_a_live_descriptor() -> None:
    client = Client()
    hub = UiPeerHub(endpoint_id="ui-a", client=client, autostart=False)
    sim = hub.channel("sim")

    assert sim.has_peer("sim-a") is False
    client.peers.add("sim-a")
    assert sim.has_peer("sim-a") is True
    hub.close()


def test_owned_peer_is_recreated_after_initial_or_heartbeat_failure() -> None:
    clients: list[Client] = []
    attempts = 0

    def factory(*_args: object, **_kwargs: object) -> Client:
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise DdsTransportError("DDS participant unavailable")
        value = Client()
        if not clients:
            value.fail_heartbeat = True
        clients.append(value)
        return value

    hub = UiPeerHub(endpoint_id="ui-a", client_factory=factory)
    try:
        _wait_until(lambda: len(clients) >= 1)
        first = clients[0]
        _wait_until(lambda: len(clients) >= 2)
        assert first.closed is True
        assert hub.registered is True
    finally:
        hub.close()
