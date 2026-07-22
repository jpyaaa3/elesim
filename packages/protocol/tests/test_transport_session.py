from __future__ import annotations

from elesim_protocol import EndpointDescriptor, make_envelope
from elesim_protocol.transport import EndpointSession


def session() -> EndpointSession:
    return EndpointSession(
        EndpointDescriptor("robot-a", "robot", instance_id="instance-a"),
        heartbeat_s=1.0,
        registration_retry_s=0.5,
        server_timeout_s=3.0,
    )


def test_session_retries_registration_until_acknowledged() -> None:
    value = session()
    assert value.next_action(now=0.0) == "register"
    value.note_sent("register", now=0.0)
    assert value.next_action(now=0.49) is None
    assert value.next_action(now=0.5) == "register"

    registered = make_envelope(
        "registered",
        "server",
        target_id="robot-a",
        payload={"ok": True, "endpoint": value.descriptor.to_dict()},
        seq=1,
    )
    assert value.observe(registered, now=0.6) is True
    assert value.registered is True
    assert value.next_action(now=1.59) is None
    assert value.next_action(now=1.6) == "heartbeat"


def test_session_returns_to_registration_after_server_timeout() -> None:
    value = session()
    value.note_sent("register", now=0.0)
    value.observe(
        make_envelope(
            "registered",
            "server",
            target_id="robot-a",
            payload={"ok": True, "endpoint": value.descriptor.to_dict()},
            seq=1,
        ),
        now=0.1,
    )

    assert value.next_action(now=3.11) == "register"
    assert value.registered is False
    assert value.server_alive(now=3.11) is False


def test_server_not_registered_error_forces_immediate_reregistration() -> None:
    value = session()
    value.observe(
        make_envelope(
            "registered",
            "server",
            target_id="robot-a",
            payload={"ok": True, "endpoint": value.descriptor.to_dict()},
            seq=1,
        ),
        now=1.0,
    )
    error = make_envelope(
        "error",
        "server",
        target_id="robot-a",
        payload={"ok": False, "reason": "endpoint is not registered"},
        seq=2,
    )

    value.observe(error, now=1.1)

    assert value.registered is False
    assert value.next_action(now=1.1) == "register"
